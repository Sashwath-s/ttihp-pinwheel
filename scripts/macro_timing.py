#!/usr/bin/env python3
"""macro_timing -- SRAM macro setup / hold / access check for Pinwheel.

Run from the repo root after `python scripts/yosys_area.py --sram` (which writes the
flat mapped netlist build/yosys/pinwheel_flat.json with the macro as a black box).

For each corner (std-cell liberty + macro liberty) it reports:
  * the macro's requirements from its liberty: setup/hold per pin group,
    clk -> A_DOUT access time at the real load, min clock pulse width
  * latest / earliest arrival at every macro input (scripts/libsta.py Timer),
    giving setup and hold slack per pin group
  * the A_DOUT -> flop path including the macro access time
Ideal clocks (no CTS latency or skew) and no wire RC: hold slack shrinks by any
clock skew between the launching flops and the macro's A_CLK.
Writes build/yosys/macro_timing.md.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import libsta  # noqa: E402

MACRO = "RM_IHPSG13_1P_512x16_c2_bm_bist"
MDIR = f"macro/{MACRO}"
CORNERS = [
    ("typ 1.20 V / 25 °C", "build/pdk/sg13cmos5l_stdcell_typ_1p20V_25C.lib", f"{MDIR}/{MACRO}_typ_1p20V_25C.lib"),
    ("slow 1.08 V / 125 °C", "build/pdk/sg13cmos5l_stdcell_slow_1p08V_125C.lib", f"{MDIR}/{MACRO}_slow_1p08V_125C.lib"),
    ("fast 1.32 V / −40…−55 °C", "build/pdk/sg13cmos5l_stdcell_fast_1p32V_m40C.lib", f"{MDIR}/{MACRO}_fast_1p32V_m55C.lib"),
]
PERIOD = 20.0
NETLIST = "build/yosys/pinwheel_flat.json"


def macro_groups(path):
    """pin group (A_ADDR, A_WEN, ...) -> {setup, hold, delay, mpw: [tables dict, ...]}"""
    lib = libsta.parse_liberty(path)
    templates = {t.args[0]: {k: (libsta._floats(v) if k.startswith("index") else v) for k, v in t.attrs.items()}
                 for t in lib.sub("lu_table_template")}
    groups = {}
    for g in lib.sub("cell")[0].kids:
        # timing groups sit in the pin group, or (for buses in this library) directly in the bus group
        owners = [g] if g.kind == "pin" else ([g] + g.sub("pin") if g.kind == "bus" else [])
        for p in owners:
            d = groups.setdefault(p.args[0].split("[")[0], {"setup": [], "hold": [], "delay": [], "mpw": []})
            for t in p.sub("timing"):
                tt = t.attrs.get("timing_type", "").strip('"')
                tabs = {k.kind: libsta.Table(k, templates) for k in t.kids if "values" in k.attrs}
                key = ("setup" if tt.startswith("setup") else "hold" if tt.startswith("hold")
                       else "delay" if tt == "rising_edge" else "mpw" if tt == "min_pulse_width" else None)
                if key:
                    d[key].append(tabs)
    return groups


def constraint(tabss, data_slew):
    v = [t.lookup(related_pin_transition=libsta.CLK_SLEW, constrained_pin_transition=data_slew)
         for tabs in tabss for k, t in tabs.items() if k in ("rise_constraint", "fall_constraint")]
    return max(v) if v else None


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    mod = next(v for v in json.load(open(NETLIST))["modules"].values() if v["attributes"].get("top"))
    inst = next((c for c in mod["cells"].values() if c["type"] == MACRO), None)
    if inst is None:
        sys.exit(f"{MACRO} not found in {NETLIST}: run scripts/yosys_area.py --sram first")
    iname = next(n for n, c in mod["cells"].items() if c is inst)
    dirs = inst["port_directions"]
    L = [f"# {MACRO} timing — Pinwheel at {PERIOD:.0f} ns\n",
         f"Instance `{iname}`. Ideal clocks, no wire RC; clock slew {libsta.CLK_SLEW} ns.\n"]
    for corner, stdlib_path, mlib_path in CORNERS:
        lib = libsta.Liberty(stdlib_path)
        mg = macro_groups(mlib_path)
        tm = libsta.Timer(mod, lib)
        L += [f"## {corner}\n", "| Pin | macro setup | macro hold | latest arrival | earliest arrival | setup slack | hold slack |",
              "|---|---:|---:|---:|---:|---:|---:|"]
        worst_setup, worst_hold = float("inf"), float("inf")
        for pin, bits in sorted(inst["connections"].items()):
            if dirs.get(pin) != "input" or pin == "A_CLK" or pin not in mg:
                continue
            live = [b for b in bits if isinstance(b, int)]
            if not live:                                          # tied off (A_BM, A_DLY, BIST)
                continue
            mx = [tm.at(b, "max") for b in live]
            mn = [tm.at(b, "min") for b in live]
            su = max(a + constraint(mg[pin]["setup"], s) for a, s, _, _ in mx)
            ho = constraint(mg[pin]["hold"], min(s for _, s, _, _ in mn))
            amax, amin = max(a for a, _, _, _ in mx), min(a for a, _, _, _ in mn)
            s_sl, h_sl = PERIOD - su, amin - ho
            worst_setup, worst_hold = min(worst_setup, s_sl), min(worst_hold, h_sl)
            L.append(f"| `{pin}` ×{len(live)} | {constraint(mg[pin]['setup'], 0.1):.3f} | {ho:.3f} | {amax:.2f} | "
                     f"{amin:.3f} | {s_sl:.2f} | {h_sl:+.3f} |")
        # access path: A_DOUT -> flops
        dout = [b for b in inst["connections"]["A_DOUT"] if isinstance(b, int)]
        ld = max(tm.load.get(b, libsta.WIRE_CAP_BASE) for b in dout)
        acc = max(t(libsta.CLK_SLEW, ld) for tabs in mg["A_DOUT"]["delay"] for k, t in tabs.items()
                  if k in ("cell_rise", "cell_fall"))
        r = libsta.sta(mod, lib, macro_launch=1000.0)
        dpath = r["worst"] - 1000.0 if r["worst"] > 500.0 else None
        mpw = constraint(mg["A_CLK"]["mpw"], libsta.CLK_SLEW)
        L.append("")
        L.append(f"- clk → `A_DOUT` access at {ld * 1000:.0f} fF: **{acc:.2f} ns**; then {dpath:.2f} ns of logic + setup "
                 f"to `{r['end']}` ⇒ **{acc + dpath:.2f} ns** of {PERIOD:.0f} ns (slack {PERIOD - acc - dpath:.2f} ns)")
        L.append(f"- worst input setup slack **{worst_setup:.2f} ns**, worst hold slack **{worst_hold:+.3f} ns**"
                 f" (before clock skew)")
        L.append(f"- `A_CLK` min pulse width {mpw:.2f} ns vs {PERIOD / 2:.0f} ns half period\n")
    rep = "\n".join(L) + "\n"
    os.makedirs("build/yosys", exist_ok=True)
    with open("build/yosys/macro_timing.md", "w", encoding="utf-8") as f:
        f.write(rep)
    print(rep)


if __name__ == "__main__":
    main()
