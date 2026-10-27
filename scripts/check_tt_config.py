#!/usr/bin/env python3
"""check_tt_config -- static checks of src/config.json's SRAM macro setup.

Run from the repo root. Checks what can be checked without LibreLane:
  * JSON parses; every dir:: path resolves (relative to src/)
  * the black-box stub's ports match the macro LEF pins (names and widths)
  * PDN_MACRO_CONNECTIONS pins exist in the LEF
  * the placement (with the PDN halo) lies inside the 6x4 core and clear of the TT pins
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
    obs = [o.split() for o in cfg.get("PDN_OBSTRUCTIONS", [])]
    covered = any(o[0] == "Metal4" and float(o[1]) <= x1 + 1e-6 and float(o[2]) <= y1 + 1e-6
                  and float(o[3]) >= x2 - 1e-6 and float(o[4]) >= y2 - 1e-6 for o in obs)
    check(covered, "PDN_OBSTRUCTIONS keeps Metal4 stripes off the macro + halo")
    check("define_pdn_grid -macro" not in re.sub(r"#.*", "", open(resolve(cfg["PDN_CFG"])).read())
          and "-macro" not in re.sub(r"#.*", "", open(resolve(cfg["PDN_CFG"])).read()),
          "pdn_cfg.tcl defines no macro grid (an empty one is fatal, PDN-0233)")
    check(abs((x - CORE[0]) / 0.48 - round((x - CORE[0]) / 0.48)) < 1e-6
          and abs((y - CORE[1]) / 3.78 - round((y - CORE[1]) / 3.78)) < 1e-6,
          "location on the site/row grid")

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
