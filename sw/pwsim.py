#!/usr/bin/env python3
"""pwsim -- instruction-set simulator (golden model) for the Pinwheel core.

Models the 4-thread barrel schedule (1 slot per instruction, 2 for LD/ST) and
emits a retire trace in exactly the format of the RTL's PW_TRACE output, so the
two can be compared per thread. Lanes/mailboxes/pin waits are not modelled:
programs used for co-simulation must stick to compute, memory, branches,
calls, GPIO/TID/EVENTS ports and HALT.

Usage: python pwsim.py prog.hex [--run 0xF] [--trace out.txt] [--dump mem.hex]
"""
import argparse
import sys

M16 = 0xFFFF


def sext(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


class SimError(Exception):
    pass


class PwSim:
    def __init__(self, image, aw=9):
        self.aw = aw
        self.amask = (1 << aw) - 1
        self.mem = [0] * (1 << aw)
        self.mem[:len(image)] = image
        self.pc = [k & self.amask for k in range(4)]
        self.regs = [[0] * 8 for _ in range(4)]
        self.run = [False] * 4
        self.skip = [False] * 4           # next slot consumed by a LD/ST data access
        self.illegal = [False] * 4
        self.attn = [False] * 4
        self.gpout = self.gpoe = self.gpod = self.ev = 0
        self.trace = [[] for _ in range(4)]

    # ------------------------------------------------------------------
    def step_thread(self, t):
        if self.skip[t]:
            self.skip[t] = False
            return
        if not self.run[t]:
            return
        pc = self.pc[t]
        ir = self.mem[pc]
        r = self.regs[t]
        op = ir >> 12
        rd = (ir >> 9) & 7
        a = r[(ir >> 6) & 7]                                  # port A = [8:6]
        b_idx = (ir >> 3) & 7 if op == 0 else ((ir >> 5) & 7 if op == 0xC else rd)
        b = r[b_idx]                                          # port B
        pc1 = (pc + 1) & self.amask
        npc = pc1
        wb = None                                             # (rd, value)
        store = None                                          # (addr, value)
        load = None
        u8 = ir & 0xFF

        if op == 0x0:
            fn = ir & 7
            v = [a + b, a - b, a & b, a | b, a ^ b, a << (b & 15), a >> (b & 15), int(a < b)][fn]
            wb = (rd, v & M16)
        elif op == 0x1:
            wb = (rd, (a + sext(ir, 6)) & M16)
        elif op == 0x2:
            wb = (rd, sext(ir, 9) & M16)
        elif op == 0x3:
            wb = (rd, (b & u8) if (ir >> 8) & 1 else ((u8 << 8) | (b & 0xFF)))
        elif op == 0x4:
            n = ir & 15
            t_ = (ir >> 4) & 3
            if t_ == 0:
                v = a << n
            elif t_ == 1:
                v = a >> n
            elif t_ == 2:
                v = sext(a, 16) >> n
            else:
                v = (a >> n) | (a << (16 - n))
            wb = (rd, v & M16)
        elif op in (0x5, 0x6):
            ea = (a + (ir & 0x3F)) & self.amask
            if op == 0x5:
                load = (rd, self.mem[ea])
            else:
                store = (ea, b)
        elif op == 0x7:
            if b == a:
                npc = (pc1 + sext(ir, 6)) & self.amask
        elif op == 0x8:
            if b != a:
                npc = (pc1 + sext(ir, 6)) & self.amask
        elif op == 0x9:
            if ((b >> ((ir >> 5) & 15)) & 1) == ((ir >> 4) & 1):
                npc = (pc1 + sext(ir, 4)) & self.amask
        elif op == 0xA:
            npc = ir & self.amask
        elif op == 0xB:
            npc = ir & self.amask
            wb = (7, pc1)
        elif op == 0xC:
            sub = (ir >> 8) & 15
            n = ir & 7
            if sub == 0:
                self.run[t] = False
            elif sub == 1:
                npc = b & self.amask
            elif sub == 3:
                self.ev |= 1 << n
            elif sub == 4:
                self.ev &= ~(1 << n)
            elif sub == 5:
                if not (self.ev >> n) & 1:
                    return                                    # blocked: replay next slot
                self.ev &= ~(1 << n)
            elif sub == 6:
                self.attn[t] = True
            elif sub == 2:
                raise SimError("waitt not modelled")
            else:
                self.run[t] = False
                self.illegal[t] = True
                npc = pc
        elif op == 0xD:
            o = (ir >> 9) & 7
            bank = (ir >> 8) & 1
            if o in (0, 1, 2):
                cur = (self.gpout >> 8) & 0x7F if bank else self.gpout & 0xFF
                m = u8 & (0x7F if bank else 0xFF)
                new = [cur | m, cur & ~m, cur ^ m][o]
                if bank:
                    self.gpout = (self.gpout & 0xFF) | (new & 0x7F) << 8
                else:
                    self.gpout = (self.gpout & 0x7F00) | (new & 0xFF)
            elif o == 3:
                self.gpoe |= u8
            elif o == 4:
                self.gpoe &= ~u8 & 0xFF
            elif o in (5, 6):
                raise SimError("pin waits not modelled")
        else:                                                 # IN / OUT / OUTI
            if op == 0xF:
                raise SimError("outi (lanes) not modelled")
            port = ir & 0xFF
            out = (ir >> 8) & 1
            if out:
                if port == 0x0A:
                    self.gpout = b & 0x7FFF
                elif port == 0x0B:
                    self.gpoe = b & 0xFF
                elif port == 0x0C:
                    self.gpod = b & 0xFF
                elif port == 0x11:
                    self.ev = b & 0xFF
                else:
                    raise SimError(f"out to port 0x{port:02x} not modelled")
            else:
                vals = {0x0A: self.gpout, 0x0B: self.gpoe, 0x0C: self.gpod, 0x10: t, 0x11: self.ev}
                if port not in vals:
                    raise SimError(f"in from port 0x{port:02x} not modelled")
                wb = (rd, vals[port])

        line = f"{t} {pc:03x} {ir:04x}"
        if wb is not None:
            r_, v_ = wb
            if r_ != 0:
                r[r_] = v_
                line += f" r{r_}={v_:04x}"
        if store is not None:
            self.mem[store[0]] = store[1]
            self.skip[t] = True
            line += f" m{store[0]:03x}={store[1]:04x}"
        self.trace[t].append(line)
        if load is not None:
            self.skip[t] = True
            if load[0] != 0:
                r[load[0]] = load[1]
                self.trace[t].append(f"{t} ld r{load[0]}={load[1]:04x}")
        self.pc[t] = npc

    def run_slots(self, max_slots=200000):
        for s in range(max_slots):
            if not any(self.run) and not any(self.skip):
                return s
            self.step_thread(s & 3)
        raise SimError("timeout: threads still running")


def load_hex(path):
    with open(path) as f:
        return [int(x, 16) for x in f.read().split()]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hex")
    ap.add_argument("--run", default="0xF", help="thread run mask")
    ap.add_argument("--aw", type=int, default=9)
    ap.add_argument("--trace", help="write retire trace")
    ap.add_argument("--dump", help="write final memory image")
    a = ap.parse_args()
    sim = PwSim(load_hex(a.hex), a.aw)
    mask = int(a.run, 0)
    sim.run = [bool(mask >> t & 1) for t in range(4)]
    try:
        slots = sim.run_slots()
    except SimError as e:
        print(f"pwsim: {e}", file=sys.stderr)
        sys.exit(1)
    if a.trace:
        with open(a.trace, "w") as f:
            for t in range(4):
                f.write("".join(l + "\n" for l in sim.trace[t]))
    if a.dump:
        with open(a.dump, "w") as f:
            f.write("".join(f"{w:04x}\n" for w in sim.mem))
    print(f"pwsim: halted after {slots} slots, retired {[len(x) for x in sim.trace]}")


if __name__ == "__main__":
    main()
