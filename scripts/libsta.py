"""libsta -- tiny Liberty parser + cell-level static timing for Yosys JSON netlists.

Good enough for early architecture decisions, not sign-off:
  * NLDM delay/transition lookup (bilinear, clamped) per timing arc,
  * load = fanout pin capacitance + a per-net wire estimate,
  * slews propagated along the worst arc, flop clk->Q launch and setup at capture,
  * no wire RC delay, no clock tree/skew, single corner per run.
"""
import re
import sys

WIRE_CAP_BASE = 0.004       # pF per net
WIRE_CAP_PIN = 0.002        # pF per fanout pin
CLK_SLEW = 0.10             # ns
IN_SLEW = 0.10              # ns, top-level inputs


# ----------------------------------------------------------------------------- parser
_TOK = re.compile(r'"(?:[^"\\]|\\.)*"|[{}();:,]|[^\s{}();:,"]+')


class Group:
    __slots__ = ("kind", "args", "attrs", "kids")

    def __init__(self, kind, args):
        self.kind, self.args, self.attrs, self.kids = kind, args, {}, []

    def sub(self, kind):
        return [g for g in self.kids if g.kind == kind]


def parse_liberty(path):
    text = open(path).read()
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S).replace("\\\n", " ")
    # some vendor libs end simple attributes at the newline without ';'
    text = re.sub(r"^(\s*\w+\s*:\s*[^;{}\n]+?)\s*$", r"\1;", text, flags=re.M)
    toks = _TOK.findall(text)
    pos = 0

    def unq(s):
        return s[1:-1] if s.startswith('"') else s

    def group_body(g):
        nonlocal pos
        while toks[pos] != "}":
            name = toks[pos]
            pos += 1
            if toks[pos] == ":":                               # simple attribute
                pos += 1
                val = []
                while toks[pos] != ";":
                    val.append(unq(toks[pos]))
                    pos += 1
                pos += 1
                g.attrs[name] = " ".join(val)
            elif toks[pos] == "(":                             # group or complex attribute
                pos += 1
                args = []
                while toks[pos] != ")":
                    if toks[pos] != ",":
                        args.append(unq(toks[pos]))
                    pos += 1
                pos += 1
                if toks[pos] == "{":
                    pos += 1
                    child = Group(name, args)
                    group_body(child)
                    pos += 1
                    g.kids.append(child)
                else:
                    if toks[pos] == ";":
                        pos += 1
                    g.attrs[name] = args
            else:                                              # stray token
                pos += 1

    assert toks[0] == "library"
    pos = toks.index("{") + 1
    lib = Group("library", [])
    group_body(lib)
    return lib


def _floats(v):
    if isinstance(v, list):
        v = ",".join(v)
    return [float(x) for x in v.replace(",", " ").split()]


class Table:
    def __init__(self, g, templates):
        tmpl = templates.get(g.args[0] if g.args else "", {})
        self.i1 = _floats(g.attrs["index_1"]) if "index_1" in g.attrs else tmpl.get("index_1", [0.0])
        self.i2 = _floats(g.attrs["index_2"]) if "index_2" in g.attrs else tmpl.get("index_2", [0.0])
        self.var1 = tmpl.get("variable_1", "input_net_transition")
        self.var2 = tmpl.get("variable_2", "total_output_net_capacitance")
        vals = [_floats(r) for r in g.attrs["values"]]
        self.v = vals if len(vals) > 1 or len(self.i1) == 1 else [vals[0][k:k + len(self.i2)]
                                                                     for k in range(0, len(vals[0]), len(self.i2))]

    @staticmethod
    def _seg(axis, x):
        if len(axis) == 1:
            return 0, 0, 0.0
        x = min(max(x, axis[0]), axis[-1])
        for k in range(len(axis) - 1):
            if x <= axis[k + 1]:
                return k, k + 1, (x - axis[k]) / (axis[k + 1] - axis[k])
        return len(axis) - 2, len(axis) - 1, 1.0

    def __call__(self, slew, load):
        """Delay/transition table at (input slew, output load)."""
        return self.lookup(input_net_transition=slew, total_output_net_capacitance=load)

    def lookup(self, **vals):
        """Table value with each axis picked by its template variable name."""
        x1, x2 = vals.get(self.var1, 0.0), vals.get(self.var2, 0.0)
        a, b, fa = self._seg(self.i1, x1)
        c, d, fc = self._seg(self.i2, x2)
        r0 = self.v[a][c] + (self.v[a][d] - self.v[a][c]) * fc
        r1 = self.v[b][c] + (self.v[b][d] - self.v[b][c]) * fc
        return r0 + (r1 - r0) * fa


class Liberty:
    def __init__(self, path):
        lib = parse_liberty(path)
        templates = {}
        for t in lib.sub("lu_table_template"):
            templates[t.args[0]] = {k: (_floats(v) if k.startswith("index") else v) for k, v in t.attrs.items()}
        self.cells = {}
        for c in lib.sub("cell"):
            pins, arcs, setup, seq = {}, {}, {}, bool(c.sub("ff") or c.sub("latch"))
            for p in c.sub("pin"):
                name = p.args[0]
                pins[name] = (p.attrs.get("direction", "input"), float(p.attrs.get("capacitance", 0) or 0))
                for t in p.sub("timing"):
                    tt = t.attrs.get("timing_type", "combinational")
                    tabs = {k.kind: Table(k, templates) for k in t.kids if "values" in k.attrs}
                    rel = t.attrs.get("related_pin", "")
                    if tt.startswith("setup"):
                        setup[name] = tabs
                    elif tt in ("combinational", "rising_edge", "falling_edge", "combinational_rise",
                                "combinational_fall", "") and "cell_rise" in tabs:
                        arcs.setdefault(name, []).append((rel, tabs))
            self.cells[c.args[0]] = {"area": float(c.attrs.get("area", 0)), "pins": pins,
                                     "arcs": arcs, "setup": setup, "seq": seq}

    def area(self, cell):
        return self.cells[cell]["area"]


# ----------------------------------------------------------------------------- timing
MACRO_PIN_CAP = 0.004       # pF, input pins of cells not in the liberty (hard macros)


class Timer:
    """Arrival times on every net bit of a flat Yosys JSON module.

    mode "max": latest arrival (setup); mode "min": earliest arrival (hold).
    Flops launch at clk->Q; cells not in the liberty (hard macros) launch on their
    outputs at `macro_launch`; cells without timing arcs (ties) are constants:
    they never limit setup (arrival 0) and never toggle for hold (arrival +inf).
    """

    def __init__(self, module, lib, macro_launch=0.0):
        self.module, self.lib, self.macro_launch = module, lib, macro_launch
        self.cells = module["cells"]
        self.names = {}
        for n, v in module["netnames"].items():
            for bit in v["bits"]:
                if isinstance(bit, int) and (bit not in self.names or self.names[bit].startswith("$")):
                    self.names[bit] = n
        self.load, self.drv, self.fanout = {}, {}, {}
        for p in module["ports"].values():
            for b in p["bits"]:
                if isinstance(b, int) and p["direction"] == "input":
                    self.drv[b] = ("PI", None)
        for cn, c in self.cells.items():
            cl = lib.cells.get(c["type"])
            for pin, bits in c["connections"].items():
                if cl is None:                                   # hard macro
                    d, cap = c.get("port_directions", {}).get(pin, "input"), MACRO_PIN_CAP
                else:
                    d, cap = cl["pins"].get(pin, ("input", 0.0))
                for b in bits:
                    if not isinstance(b, int):
                        continue
                    if d == "output":
                        self.drv[b] = ("MACRO", None) if cl is None else (cn, pin)
                    else:
                        self.load[b] = self.load.get(b, WIRE_CAP_BASE) + cap + WIRE_CAP_PIN
                        self.fanout[b] = self.fanout.get(b, 0) + 1
        self.memo = {"max": {}, "min": {}}
        self.info = {}
        sys.setrecursionlimit(50000)

    def at(self, b, mode="max"):
        """-> (arrival ns, slew ns, cells on path, previous bit)"""
        memo = self.memo[mode]
        if b in memo:
            return memo[b]
        src = self.drv.get(b)
        if src is None or src[0] == "PI":
            return (0.0, IN_SLEW, 0, None)
        if src[0] == "MACRO":
            return (self.macro_launch, IN_SLEW, 0, None)
        memo[b] = None                                           # loop guard
        cn, opin = src
        c = self.cells[cn]
        cl = self.lib.cells[c["type"]]
        ld = self.load.get(b, WIRE_CAP_BASE)
        pick = max if mode == "max" else min
        best = None
        for rel, tabs in cl["arcs"].get(opin, []):
            rbits = [x for x in c["connections"].get(rel, []) if isinstance(x, int)]
            if cl["seq"]:
                arr0, slew0, n0, prev = 0.0, CLK_SLEW, 0, None   # launch on the clock edge
            elif rbits:
                st = self.at(rbits[0], mode)
                if st is None:
                    sys.exit("combinational loop")
                arr0, slew0, n0, _ = st
                prev = rbits[0]
            else:
                continue
            dly = pick(tabs[k](slew0, ld) for k in ("cell_rise", "cell_fall") if k in tabs)
            slw = pick([tabs[k](slew0, ld) for k in ("rise_transition", "fall_transition") if k in tabs] or [slew0])
            cand = (arr0 + dly, slw, n0 + 1, prev)
            if best is None or pick(cand[0], best[0]) == cand[0] and cand[0] != best[0]:
                best = cand
                if mode == "max":
                    self.info[b] = (c["type"], dly, ld, self.fanout.get(b, 0))
        if best is None:                                         # tie cell: constant
            best = (0.0 if mode == "max" else float("inf"), IN_SLEW, 0, None)
        memo[b] = best
        return best

    def path(self, b):
        """Worst (max) path ending at bit b, as [(net, cell, delay, load, fanout)], plus its start net."""
        out, start = [], None
        while b is not None:
            start = b
            if b in self.info:
                out.append((self.names.get(b, f"bit{b}"),) + self.info[b])
            b = self.at(b)[3]
        return out[::-1], self.names.get(start, "?")

    def flop_endpoints(self):
        """Yield (flop cell, data bit, setup tables, Q bit) for every flop data input."""
        for cn, c in self.cells.items():
            cl = self.lib.cells.get(c["type"])
            if cl is None or not cl["seq"]:
                continue
            q = next((x for p, bits in c["connections"].items() if cl["pins"].get(p, ("",))[0] == "output"
                      for x in bits if isinstance(x, int)), None)
            for pin, tabs in cl["setup"].items():
                for b in c["connections"].get(pin, []):
                    if isinstance(b, int):
                        yield cn, b, tabs, q


def setup_time(tabs, data_slew):
    return max([t.lookup(related_pin_transition=CLK_SLEW, constrained_pin_transition=data_slew)
                for k, t in tabs.items() if k in ("rise_constraint", "fall_constraint")] or [0.0])


def sta(module, lib, macro_launch=0.0):
    """-> dict(worst=ns, cells=n, start=net, end=net, path=[...]) for the worst path into a flop.
    Cells not in the liberty (hard macros) launch on their outputs `macro_launch` ns
    after the clock edge; their inputs are not checked here."""
    tm = Timer(module, lib, macro_launch)
    worst = (0.0, 0, None, None)
    for _, b, tabs, q in tm.flop_endpoints():
        arr, slew, n, _ = tm.at(b)
        req = arr + setup_time(tabs, slew)
        if req > worst[0]:
            worst = (req, n, b, q)
    path, start = tm.path(worst[2]) if worst[2] is not None else ([], "?")
    return {"worst": worst[0], "cells": worst[1], "path": path,
            "start": start, "end": tm.names.get(worst[3], tm.names.get(worst[2], "?"))}
