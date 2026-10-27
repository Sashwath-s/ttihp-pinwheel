#!/usr/bin/env python3
"""yosys_area -- synthesize Pinwheel onto IHP sg13cmos5l standard cells with
Yosys (YoWASP build) and report cell area per block.

Run from the repo root (liberty files: see README "Area estimate"):
    python scripts/yosys_area.py
    python scripts/yosys_area.py --slow-lib build/pdk/sg13cmos5l_stdcell_slow_1p08V_125C.lib

Hierarchical run -> per-block area table (JSON netlist + liberty cell areas).
Flat run -> LibreLane-like mapping (ABC with driving cell/load constraints, so it
buffers and sizes), total area, and a cell-level timing estimate (scripts/libsta.py:
NLDM lookup at real fanout loads, no wire RC, no clock tree).
Writes build/yosys/area_report.md.
"""
import argparse
import json
import os
import re
import sys

import yowasp_yosys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import libsta  # noqa: E402

SRC = ["pw_fifo.v", "pw_mem.v", "pw_lane.v", "pw_core.v", "pw_host_spi.v", "pw_top.v", "tt_um_pinwheel.v"]
OUT = "build/yosys"
BUDGET_UM2 = 1289.28 * 710.64        # 6x4 die (tt-support-tools ihp-sg13cmos5l tile_sizes.yaml)
DENSITY = 0.60                        # TT template PL_TARGET_DENSITY_PCT
CLOCK_NS = 20.0


def yosys(script, name):
    sp, log = f"{OUT}/{name}.ys", f"{OUT}/{name}.log"
    with open(sp, "w") as f:
        f.write(script)
    if yowasp_yosys.run_yosys(["-q", "-l", log, "-s", sp]) != 0:
        sys.exit(f"yosys failed ({name}), see {log}")


SRAM = "RM_IHPSG13_1P_512x16_c2_bm_bist"
SRAM_V = f"macro/{SRAM}/{SRAM}.v"
SRAM_LEF = f"macro/{SRAM}/{SRAM}.lef"
MACROS = {}                                         # hard macro -> area (um^2) from its LEF SIZE


def lef_area(path):
    w, h = re.search(r"^\s*SIZE\s+([0-9.]+)\s+BY\s+([0-9.]+)", open(path).read(), re.M).groups()
    return float(w) * float(h)


def walk(mods, lib, mod):
    """-> (area, flops, {child_instance: (module, area, flops)}) for one module, children included."""
    a, s, kids = 0.0, 0, {}
    for cname, c in mods[mod]["cells"].items():
        t = c["type"]
        if t in MACROS:
            a += MACROS[t]
        elif t in lib.cells:
            a += lib.area(t)
            s += lib.cells[t]["seq"]
        elif t in mods:
            ka, ks, _ = walk(mods, lib, t)
            kids[cname] = (t, ka, ks)
            a += ka
            s += ks
        else:
            sys.exit(f"unmapped cell type {t} in {mod}")
    return a, s, kids


def local(mods, lib, mod):
    cells = [c["type"] for c in mods[mod]["cells"].values() if c["type"] in lib.cells]
    return sum(lib.area(t) for t in cells), sum(lib.cells[t]["seq"] for t in cells)


def top_module(path):
    return next(v for v in json.load(open(path))["modules"].values() if v["attributes"].get("top"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lib", default="build/pdk/sg13cmos5l_stdcell_typ_1p20V_25C.lib")
    ap.add_argument("--slow-lib", default=None)
    ap.add_argument("-D", "--define", action="append", default=[])
    ap.add_argument("--sram", action="store_true", help=f"memory = {SRAM} macro (black box, LEF area)")
    a = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.makedirs(OUT, exist_ok=True)
    lib = libsta.Liberty(a.lib)
    head = ""
    if a.sram:
        a.define.append("PW_MEM_IHP_SRAM")
        MACROS[SRAM] = lef_area(SRAM_LEF)
        head = f"read_verilog -lib -DFUNCTIONAL -DSYNTHESIS {SRAM_V}\n"   # skips the specify block
    head += (f"read_verilog {' '.join('-D' + x for x in a.define)} "
             + " ".join(f"src/{s}" for s in SRC) + "\nhierarchy -check -top tt_um_sashwath_s_pinwheel\n")

    # ------------------------------------------------ hierarchical: per-block areas
    yosys(head + f"""synth -top tt_um_sashwath_s_pinwheel
dfflibmap -liberty {a.lib}
abc -liberty {a.lib}
opt_clean -purge
write_json {OUT}/pinwheel_hier.json
""", "hier")
    mods = json.load(open(f"{OUT}/pinwheel_hier.json"))["modules"]
    top = next(m for m, v in mods.items() if v["attributes"].get("top"))
    total, total_seq, kids = walk(mods, lib, top)
    pw_mod = kids["u_pw"][0]
    _, _, pw_kids = walk(mods, lib, pw_mod)

    groups = {}                                     # label -> [instances, area, flops]

    def add(label, area, seq):
        g = groups.setdefault(label, [0, 0.0, 0])
        g[0] += 1
        g[1] += area
        g[2] += seq

    labels = {"u_mem": f"Memory ({SRAM} macro)" if a.sram else "Memory (pw_mem, flop array)",
              "u_core": "Core (4 threads, RF, pipeline, GPIO)", "u_host": "Host SPI bridge"}
    for inst, (_, ar, sq) in pw_kids.items():
        if inst in labels:
            add(labels[inst], ar, sq)
        elif "u_lane" in inst:
            add("Waveform lanes", ar, sq)
        elif "g_mbox" in inst:
            add("Mailbox FIFOs (h2t + t2h)", ar, sq)
        else:
            add(f"other: {inst}", ar, sq)
    add("Top glue (pin mux, IO decode, host regs, syncs)", *local(mods, lib, pw_mod))
    lane_mod = next(m for i, (m, _, _) in pw_kids.items() if "u_lane" in i)
    _, _, lk = walk(mods, lib, lane_mod)
    lane_rows = [("engine + config", *local(mods, lib, lane_mod))] + [(i, ar, sq) for i, (_, ar, sq) in lk.items()]

    # ------------------------------------------------ flat, buffered + sized: total area and timing
    with open(f"{OUT}/abc.constr", "w") as f:
        f.write("set_driving_cell sg13cmos5l_buf_4\nset_load 0.005\n")
    yosys(head + f"""synth -flatten -top tt_um_sashwath_s_pinwheel
dfflibmap -liberty {a.lib}
abc -liberty {a.lib} -constr {OUT}/abc.constr -D {int(CLOCK_NS * 1000)}
opt_clean -purge
write_json {OUT}/pinwheel_flat.json
""", "flat")
    flat = top_module(f"{OUT}/pinwheel_flat.json")
    flat_area = sum(MACROS.get(c["type"]) or lib.area(c["type"]) for c in flat["cells"].values())
    nbuf = sum("_buf_" in c["type"] for c in flat["cells"].values())
    corners = [("typ 1.20 V / 25 °C", lib)] + ([("slow 1.08 V / 125 °C", libsta.Liberty(a.slow_lib))] if a.slow_lib else [])
    timing = [(name, libsta.sta(flat, l)) for name, l in corners]
    # worst path that starts at a macro output, measured from DOUT valid (add the macro access time)
    from_macro = [(name, libsta.sta(flat, l, macro_launch=1000.0)) for name, l in corners] if MACROS else []

    # ------------------------------------------------ report
    mem = groups.get(labels["u_mem"], [0, 0.0, 0])
    budget = BUDGET_UM2 * DENSITY
    macro_area = sum(MACROS.values())
    if MACROS:                                      # macro + 10 um halo is not placeable
        w, h = re.search(r"^\s*SIZE\s+([0-9.]+)\s+BY\s+([0-9.]+)", open(SRAM_LEF).read(), re.M).groups()
        budget = (BUDGET_UM2 - (float(w) + 20) * (float(h) + 20)) * DENSITY
    L = [f"# Pinwheel cell area — IHP sg13cmos5l ({os.path.basename(a.lib)})\n",
         f"Yosys 0.70 (YoWASP): `synth` + `dfflibmap` + `abc -liberty`; defines: {a.define or 'none'}.\n",
         "| Block | Inst | µm² / inst | µm² total | % | flops |", "|---|---:|---:|---:|---:|---:|"]
    for label, (n, ar, sq) in sorted(groups.items(), key=lambda kv: -kv[1][1]):
        L.append(f"| {label} | {n} | {ar / n:,.0f} | {ar:,.0f} | {100 * ar / total:.1f} | {sq:,} |")
    L.append(f"| **Total (hierarchical)** | | | **{total:,.0f}** | 100 | {total_seq:,} |")
    L.append(f"| Total excluding memory | | | {total - mem[1]:,.0f} | {100 * (total - mem[1]) / total:.1f} | {total_seq - mem[2]:,} |\n")
    L += ["Lane breakdown (one instance):\n", "| Part | µm² | flops |", "|---|---:|---:|"]
    L += [f"| {n} | {ar:,.0f} | {sq} |" for n, ar, sq in lane_rows]
    L.append(f"\nFlat, buffered + sized (ABC `-constr`, LibreLane-like): **{flat_area:,.0f} µm²**, "
             f"{nbuf} buffers ({100 * (flat_area / total - 1):+.1f}% vs hierarchical).\n")
    if MACROS:
        cells = flat_area - macro_area
        L.append(f"Budget: 6x4 die {BUDGET_UM2 / 1e6:.3f} mm² minus the macro and a 10 µm halo, at "
                 f"{DENSITY:.0%} placement density ≈ {budget:,.0f} µm² of cells.\n")
        L.append(f"- standard cells: {cells:,.0f} µm² = **{cells / budget:.0%} of budget**; "
                 f"macro {macro_area:,.0f} µm²; raw utilization {flat_area / BUDGET_UM2:.0%}\n")
    else:
        L.append(f"Budget: 6x4 die {BUDGET_UM2 / 1e6:.3f} mm²; at {DENSITY:.0%} placement density ≈ {budget:,.0f} µm² of cells.\n")
        L.append(f"- full design: {flat_area / budget:.0%} of budget (raw utilization {flat_area / BUDGET_UM2:.0%})")
        L.append(f"- without memory: {(flat_area - mem[1]) / budget:.0%} of budget (raw {(flat_area - mem[1]) / BUDGET_UM2:.0%})\n")
    L.append(f"Timing estimate, flop to flop (cell delays at real fanout loads + setup; no wire RC or clock tree), {CLOCK_NS:.0f} ns clock:\n")
    for corner, r in timing:
        L.append(f"- {corner}: **{r['worst']:.2f} ns** ({r['cells']} cells), `{r['start']}` → `{r['end']}`")
    for corner, r in from_macro:
        if r["worst"] > 1000.0:
            L.append(f"- {corner}, from SRAM `A_DOUT` valid: **{r['worst'] - 1000.0:.2f} ns** ({r['cells']} cells) "
                     f"to `{r['end']}` — add the macro clock-to-DOUT access time")
    rep = "\n".join(L) + "\n"
    name = "area_report_sram.md" if MACROS else "area_report.md"
    with open(f"{OUT}/{name}", "w", encoding="utf-8") as f:
        f.write(rep)
    print(rep)


if __name__ == "__main__":
    main()
