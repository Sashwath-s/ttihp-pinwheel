#!/usr/bin/env python3
"""check_tt_config -- static checks of src/config.json's SRAM macro setup.

Run from the repo root. Checks what can be checked without LibreLane:
  * JSON parses; every dir:: path resolves (relative to src/)
  * the black-box stub's ports match the macro LEF pins (names and widths)
  * PDN_MACRO_CONNECTIONS pins exist in the LEF
  * the placement (with the PDN halo) lies inside the 6x4 core and clear of the TT pins
  * the PDN stripes left by PDN_OBSTRUCTIONS cross the SRAM only on same-net power
    straps, clear of its obstructions, and feed VDD!, VDDARRAY! and VSS!
  * Yosys, reading the sources the way the TT flow does (info.yaml sources incl. the
    stub, VERILOG_DEFINES, flatten), keeps an instance with the exact MACROS name
"""
import json
import os
import re
import sys

import yowasp_yosys

ROOT = os.getcwd()
SRC = os.path.join(ROOT, "src")
DIE = (1289.28, 710.64)                 # tt-support-tools tile_sizes.yaml, 6x4
CORE = (2.88, 3.78, 1286.40, 706.86)    # rows in tt_block_6x4_pgvdd.def
TT_PINS_X_MAX, TT_PINS_Y = 192.0, 710.14
PDN_HALO = 10.0                          # LibreLane PDN_*_HALO default
errors = 0


def check(ok, msg):
    global errors
    errors += not ok
    print(("ok    " if ok else "FAIL  ") + msg)


def resolve(p):
    return os.path.normpath(os.path.join(SRC, p[5:])) if p.startswith("dir::") else p


def lef_pins(path):
    t = open(path).read()
    pins = {}
    for name in re.findall(r"^\s*PIN\s+(\S+)", t, re.M):
        base, idx = re.match(r"([^\[]+)(?:\[(\d+)\])?$", name).groups()
        pins.setdefault(base, set()).add(int(idx) if idx is not None else None)
    w, h = re.search(r"SIZE\s+([0-9.]+)\s+BY\s+([0-9.]+)", t).groups()
    return pins, (float(w), float(h))


def lef_rects(path, layer):
    """Metal rectangles of the macro: {pin name: [rects]} and the OBS rects, on one layer."""
    t = open(path).read()

    def grab(block):
        out, cur = [], None
        for line in block.splitlines():
            m = re.match(r"\s*LAYER\s+(\w+)", line)
            if m:
                cur = m.group(1)
                continue
            if cur == layer:
                out += [tuple(map(float, r)) for r in
                        re.findall(r"RECT\s+([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)", line)]
        return out
    pins = {p: grab(b) for p, b in re.findall(r"PIN\s+(\S+)(.*?)END\s+\1(?=\s)", t, re.S)}
    return pins, grab(re.search(r"\bOBS\b(.*?)\n\s*END\b", t, re.S).group(1))


def stripes(cfg):
    """Metal4 stripe x-ranges as LibreLane drew them in GDS run 2:
    VPWR centre = core_xl + offset + k*pitch, VGND centre = VPWR + width + spacing."""
    off = float(cfg.get("FP_PDN_VOFFSET", 10.0))           # PDK default
    pitch, width = float(cfg["FP_PDN_VPITCH"]), float(cfg["FP_PDN_VWIDTH"])
    spacing = float(cfg.get("FP_PDN_VSPACING", 2.0))        # PDK default
    out, c = [], CORE[0] + off
    while c + width + spacing < CORE[2]:
        out += [("VPWR", c - width / 2, c + width / 2), ("VGND", c + width + spacing - width / 2,
                                                         c + width + spacing + width / 2)]
        c += pitch
    return out


def cut(y1, y2, holes):
    """Subtract y-intervals from [y1, y2]."""
    segs = [(y1, y2)]
    for h1, h2 in holes:
        segs = [s for a, b in segs for s in ((a, min(b, h1)), (max(a, h2), b)) if s[1] - s[0] > 1e-6]
    return segs


def main():
    cfg = json.load(open(os.path.join(SRC, "config.json")))
    check(True, "src/config.json parses")
    check("PW_MEM_IHP_SRAM" in cfg.get("VERILOG_DEFINES", []), "VERILOG_DEFINES has PW_MEM_IHP_SRAM")
    paths = [cfg["PDN_CFG"]]
    for m in cfg["MACROS"].values():
        for k in ("gds", "lef", "nl", "spice"):
            paths += m.get(k, [])
        for v in m.get("lib", {}).values():
            paths += v
    for p in paths:
        check(os.path.isfile(resolve(p)), f"exists: {p}")

    (mname, macro), = cfg["MACROS"].items()
    pins, (mw, mh) = lef_pins(resolve(macro["lef"][0]))
    stub = open(resolve(macro["nl"][0])).read()
    ports = {}
    for d, msb, name in re.findall(r"\b(input|output)\s+(?:\[(\d+):0\]\s+)?(\w+)", stub):
        ports[name] = set(range(int(msb) + 1)) if msb else {None}
    sig = {k: v for k, v in pins.items() if not k.endswith("!")}
    check(ports == sig, f"stub ports match LEF signal pins ({len(sig)} ports)")
    if ports != sig:
        print("      stub-only:", sorted(set(ports) - set(sig)), " lef-only:", sorted(set(sig) - set(ports)))
    for conn in cfg["PDN_MACRO_CONNECTIONS"]:
        inst, vdd, gnd, vpin, gpin = conn.split()
        check(vpin in pins and gpin in pins, f"PDN pins exist in LEF: {vpin} {gpin}")

    (iname, place), = macro["instances"].items()
    x, y = place["location"]
    x1, y1, x2, y2 = x - PDN_HALO, y - PDN_HALO, x + mw + PDN_HALO, y + mh + PDN_HALO
    check(place["orientation"] == "R0", "orientation R0 (required by pdn_cfg.tcl)")
    check(CORE[0] <= x1 and CORE[1] <= y1 and x2 <= CORE[2] and y2 <= CORE[3],
          f"macro+halo ({x1:.2f},{y1:.2f})-({x2:.2f},{y2:.2f}) inside core {CORE}")
    check(x1 > TT_PINS_X_MAX, f"macro+halo clear of the TT pins (x < {TT_PINS_X_MAX} at the top edge)")
    # PDN: simulate the Metal4 stripes, cut them with PDN_OBSTRUCTIONS, and check that every
    # piece crossing the SRAM lies inside a same-net power strap and clear of its OBS
    pins4, obs4 = lef_rects(resolve(macro["lef"][0]), "Metal4")
    supply = {"VPWR": ("VDD!", "VDDARRAY!"), "VGND": ("VSS!",)}
    holes = [tuple(map(float, o.split()[1:])) for o in cfg.get("PDN_OBSTRUCTIONS", []) if o.split()[0] == "Metal4"]
    fed, bad = set(), []
    for net, sx1, sx2 in stripes(cfg):
        hs = [(b, d) for a, b, c, d in holes if a < sx2 and c > sx1]
        for sy1, sy2 in cut(CORE[1], CORE[3], hs):
            if not (sx1 < x + mw and sx2 > x and sy1 < y + mh and sy2 > y):
                continue                                         # does not cross the macro
            rx1, rx2 = sx1 - x, sx2 - x                          # macro coordinates
            ry1, ry2 = max(sy1, y) - y, min(sy2, y + mh) - y
            home = [p for p in supply[net] for r in pins4.get(p, [])
                    if r[0] <= rx1 + 1e-6 and r[2] >= rx2 - 1e-6 and r[1] <= ry1 + 1e-6 and r[3] >= ry2 - 1e-6]
            touch_obs = any(o[0] < rx2 and o[2] > rx1 and o[1] < ry2 and o[3] > ry1 for o in obs4)
            touch_other = any(r[0] < rx2 and r[2] > rx1 and r[1] < ry2 and r[3] > ry1
                              for p, rs in pins4.items() if p.endswith("!") and p not in supply[net] for r in rs)
            if home and not touch_obs and not touch_other:
                fed.update(home)
            else:
                bad.append(f"{net} x[{sx1:.3f},{sx2:.3f}] y[{sy1:.2f},{sy2:.2f}]")
    check(not bad, "every PDN stripe crossing the SRAM sits inside a same-net power strap"
          + ("" if not bad else f": bad {bad}"))
    for p in ("VDD!", "VDDARRAY!", "VSS!"):
        check(p in fed, f"SRAM {p} is fed by a PDN stripe")
    check("define_pdn_grid -macro" not in re.sub(r"#.*", "", open(resolve(cfg["PDN_CFG"])).read())
          and "-macro" not in re.sub(r"#.*", "", open(resolve(cfg["PDN_CFG"])).read()),
          "pdn_cfg.tcl defines no macro grid (an empty one is fatal, PDN-0233)")
    check(abs(y - CORE[1] - 3.78 * round((y - CORE[1]) / 3.78)) < 1e-6, "location on the row grid")

    # synthesize like the TT flow: info.yaml sources (incl. the stub) + VERILOG_DEFINES, flattened
    srcs = re.findall(r'^\s*-\s*"([^"]+)"', open(os.path.join(ROOT, "info.yaml")).read(), re.M)
    defines = " ".join(f"-D{d}" for d in cfg["VERILOG_DEFINES"])
    os.makedirs("build/yosys", exist_ok=True)
    with open("build/yosys/tt_like.ys", "w") as f:
        f.write(f"read_verilog {defines} " + " ".join(f"src/{s}" for s in srcs) + "\n"
                "hierarchy -check -top tt_um_sashwath_s_pinwheel\nsynth -flatten -top tt_um_sashwath_s_pinwheel\n"
                "tee -q -o build/yosys/tt_like_cells.txt select -list t:" + mname + "\n")
    rc = yowasp_yosys.run_yosys(["-q", "-l", "build/yosys/tt_like.log", "-s", "build/yosys/tt_like.ys"])
    check(rc == 0, "Yosys reads info.yaml sources with VERILOG_DEFINES and flattens")
    found = open("build/yosys/tt_like_cells.txt").read().split() if rc == 0 else []
    names = [n.split("/", 1)[-1].lstrip("\\") for n in found]
    check(iname in names, f"flattened netlist has {mname} instance '{iname}' (found {names})")

    print(f"\n{errors} problem(s)")
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
