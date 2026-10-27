#!/usr/bin/env python3
"""randprog -- constrained-random Pinwheel programs for RTL-vs-ISS co-simulation.

Each of the 4 threads gets its own code and data region and runs a random mix
of ALU/immediate/shift ops, loads/stores, forward branches (all 4 kinds),
counted loops and JAL/JR calls, then stores its registers and halts.
Constraints keep every program terminating and every thread's trace
independent of the barrel interleaving (private memory, no shared reads).

Usage: python randprog.py --seed 7 [-n 50] -o rand.pws
"""
import argparse
import random

CODE = [0x010, 0x080, 0x0F0, 0x160]          # 112 words per thread
DATA = [0x1C0, 0x1D0, 0x1E0, 0x1F0]          # 16 words per thread
ALU = ["add", "sub", "and", "or", "xor", "shl", "shr", "sltu"]
SHI = ["shli", "shri", "srai", "rori"]
DST = ["r1", "r2", "r3", "r4", "r5", "r7"]    # r6 = data base (never written)
SRC = [f"r{i}" for i in range(8)]


class Gen:
    def __init__(self, rnd, t):
        self.r, self.t, self.lab = rnd, t, 0

    def label(self):
        self.lab += 1
        return f"t{self.t}_L{self.lab}"

    def simple(self, dst=DST):
        r = self.r
        k = r.choices(["alu", "addi", "li", "lui", "andi", "shi", "ld", "st", "tid"],
                      [30, 12, 8, 5, 5, 12, 10, 10, 2])[0]
        d = r.choice(dst)
        if k == "alu":
            return f"{r.choice(ALU)} {d}, {r.choice(SRC)}, {r.choice(SRC)}"
        if k == "addi":
            return f"addi {d}, {r.choice(SRC)}, {r.randint(-32, 31)}"
        if k == "li":
            return f"li {d}, {r.randint(-256, 255)}"
        if k in ("lui", "andi"):
            return f"{k} {d}, 0x{r.randint(0, 255):02x}"
        if k == "shi":
            return f"{r.choice(SHI)} {d}, {r.choice(SRC)}, {r.randint(0, 15)}"
        if k == "ld":
            return f"ld {d}, [r6+{r.randint(0, 15)}]"
        if k == "st":
            return f"st {r.choice(SRC)}, [r6+{r.randint(0, 15)}]"
        return f"in {d}, TID"

    def body(self, n):
        r, out = self.r, []
        while n > 0:
            k = r.choices(["simple", "branch", "loop", "call"], [70, 15, 8, 7])[0]
            if k == "simple":
                out.append(self.simple()); n -= 1
            elif k == "branch":
                lab, skip = self.label(), r.randint(1, 3)
                br = r.choice(["beq", "bne", "bbc", "bbs"])
                if br in ("beq", "bne"):
                    out.append(f"{br} {r.choice(SRC)}, {r.choice(SRC)}, {lab}")
                else:
                    out.append(f"{br} {r.choice(SRC)}, {r.randint(0, 15)}, {lab}")
                out += [self.simple() for _ in range(skip)]
                out.append(f"{lab}:")
                n -= skip + 1
            elif k == "loop":
                lab = self.label()
                out.append(f"li r5, {r.randint(1, 4)}")
                out.append(f"{lab}:")
                out += [self.simple(dst=["r1", "r2", "r3", "r4", "r7"]) for _ in range(r.randint(1, 4))]
                out += ["addi r5, r5, -1", f"bne r5, r0, {lab}"]
                n -= 6
            else:
                out.append(f"jal t{self.t}_sub"); n -= 1
        return out

    def thread(self, n):
        r, t = self.r, self.t
        lines = [f".org 0x{CODE[t]:03x}", f"t{t}_main:"]
        lines += [f"li16 {d}, 0x{r.randint(0, 0xFFFF):04x}" for d in DST]
        lines += [f"li16 r6, 0x{DATA[t]:03x}"]
        lines += self.body(n)
        lines += [f"st {d}, [r6+{8 + i}]" for i, d in enumerate(DST)]
        lines += ["halt", f"t{t}_sub:"]
        lines += [self.simple(dst=["r1", "r2", "r3", "r4", "r5"]) for _ in range(r.randint(2, 5))]
        lines += ["ret"]
        return lines


def generate(seed, n=50):
    rnd = random.Random(seed)
    out = [f"; randprog seed={seed}", ".org 0"] + [f"jmp t{t}_main" for t in range(4)]
    for t in range(4):
        out += Gen(rnd, t).thread(n)
    for t in range(4):
        out += [f".org 0x{DATA[t]:03x}"] + [f".word 0x{rnd.randint(0, 0xFFFF):04x}" for _ in range(16)]
    return "\n".join(("    " + l) if not l.endswith(":") and not l.startswith(";") else l for l in out) + "\n"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("-n", type=int, default=50, help="body length per thread")
    ap.add_argument("-o", "--out", default="rand.pws")
    a = ap.parse_args()
    with open(a.out, "w") as f:
        f.write(generate(a.seed, a.n))
