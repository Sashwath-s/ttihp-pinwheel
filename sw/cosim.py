#!/usr/bin/env python3
"""cosim -- constrained-random RTL vs ISS co-simulation for Pinwheel.

For each seed: generate a random 4-thread program (randprog), assemble it,
run the golden ISS (pwsim) and the RTL (xsim, tb_rand with PW_TRACE), then
compare every thread's retire trace and the final memory image.

Usage: python sw/cosim.py --seeds 1-20 [-n 50] [--mem flop|sram]
  --mem sram simulates the IHP RM_IHPSG13_1P_512x16 behavioural model
  (vendored in macro/) instead of the flop array.
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WORK = os.path.join(ROOT, "test", "xsim_work")
sys.path.insert(0, HERE)

import pwasm      # noqa: E402
import pwsim      # noqa: E402
import randprog   # noqa: E402

VIVADO = os.environ.get("VIVADO", r"C:\AMDDesignTools\2026.1\Vivado\bin")
SRC = ["pw_fifo.v", "pw_mem.v", "pw_lane.v", "pw_core.v", "pw_host_spi.v", "pw_top.v", "tt_um_pinwheel.v"]


def sh(args):
    r = subprocess.run(["cmd", "/c"] + args, cwd=WORK, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"command failed: {' '.join(args)}\n{r.stdout[-2000:]}{r.stderr[-2000:]}")
    return r.stdout


SRAM_DIR = os.path.join(ROOT, "macro", "RM_IHPSG13_1P_512x16_c2_bm_bist")
SRAM_SRC = ["RM_IHPSG13_1P_core_behavioral_bm_bist.v", "RM_IHPSG13_1P_512x16_c2_bm_bist.v"]


def build(mem="flop"):
    os.makedirs(WORK, exist_ok=True)
    srcs = [os.path.join(ROOT, "src", s) for s in SRC] + [os.path.join(ROOT, "test", "tb_rand.sv")]
    defs = ["-d", "SIM", "-d", "PW_TRACE"]
    if mem == "sram":
        missing = [s for s in SRAM_SRC if not os.path.exists(os.path.join(SRAM_DIR, s))]
        if missing:
            sys.exit(f"IHP SRAM model missing in {SRAM_DIR}: {missing}")
        defs += ["-d", "PW_MEM_IHP_SRAM", "-d", "FUNCTIONAL"]      # IHP zero-delay model
        srcs = [os.path.join(SRAM_DIR, s) for s in SRAM_SRC] + srcs
    sh([os.path.join(VIVADO, "xvlog.bat"), "--sv"] + defs + ["-i", os.path.join(ROOT, "test")] + srcs)
    sh([os.path.join(VIVADO, "xelab.bat"), "tb_rand", "-s", "tb_rand_trace",
        "--timescale", "1ns/1ps", "--debug", "off"])


def split_threads(lines):
    per = [[] for _ in range(4)]
    for l in lines:
        l = l.strip()
        if l:
            per[int(l.split()[0])].append(l)
    return per


def run_seed(seed, n):
    src = randprog.generate(seed, n)
    mem, _, _ = pwasm.assemble(src, f"rand{seed}")
    text = "".join(f"{w:04x}\n" for w in mem)
    for name in (f"rand_{seed}.hex", "rand.hex"):          # tb_rand loads rand.hex
        with open(os.path.join(WORK, name), "w") as f:
            f.write(text)

    sim = pwsim.PwSim(mem)
    sim.run = [True] * 4
    slots = sim.run_slots()

    dump = os.path.join(WORK, "rand_rtl_mem.hex")           # tb_rand default dump name
    out = sh([os.path.join(VIVADO, "xsim.bat"), "tb_rand_trace", "-R"])
    if "-> PASS" not in out:
        return False, "tb_rand reported failure:\n" + out[-1500:]

    with open(os.path.join(WORK, "pw_trace.txt")) as f:
        rtl = split_threads(f.readlines())
    for t in range(4):
        exp, got = sim.trace[t], rtl[t]
        for i, (e, g) in enumerate(zip(exp, got)):
            if e != g:
                return False, f"T{t} event {i}: ISS '{e}' != RTL '{g}'"
        if len(exp) != len(got):
            return False, f"T{t}: ISS retired {len(exp)} events, RTL {len(got)}"
    with open(dump) as f:
        words = [w for w in f.read().split() if not w.startswith("//")]
    for a, (e, g) in enumerate(zip(sim.mem, words)):
        if g.lower() == "x" * len(g) or int(g, 16) != e:
            return False, f"memory[0x{a:03x}]: ISS {e:04x} != RTL {g}"
    return True, f"{sum(len(x) for x in sim.trace)} events, {slots} slots"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="1-10")
    ap.add_argument("-n", type=int, default=50)
    ap.add_argument("--mem", choices=["flop", "sram"], default="flop")
    a = ap.parse_args()
    lo, _, hi = a.seeds.partition("-")
    seeds = range(int(lo), int(hi or lo) + 1)
    build(a.mem)
    print(f"cosim: memory model = {a.mem}", flush=True)
    fails = 0
    for s in seeds:
        ok, msg = run_seed(s, a.n)
        fails += not ok
        print(f"seed {s:4d}: {'PASS' if ok else 'FAIL'}  {msg}", flush=True)
    print(f"cosim: {len(seeds) - fails}/{len(seeds)} seeds passed")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
