#!/usr/bin/env python3
"""pwasm -- assembler for the Pinwheel protocol-emulator ISA.

Usage:  python pwasm.py prog.pws -o prog.hex [-l prog.lst] [--words 512]

Syntax:
    ; comment            // comment
    label:
    .org  expr           .word expr, ...      .equ NAME, expr     NAME = expr
    add r1, r2, r3       addi r1, r1, -1      li r1, 100          li16 r1, 0xBEEF
    ld r1, [r2+3]        st r1, [r2]          beq r1, r2, label   bbs r1, 15, label
    in r1, MBOX          out LDATA0, r1       outi LCTRL0, DRIVE(dout=0, clk=1)
    pset uio, 0x03       waitl ui, 0x01       oeset 0x03

Operand expressions are Python expressions over symbols plus helpers:
    PH(dat, clk, sample, exit)  TPL(p0, p1, p2, p3)  CFG0(...)  CFG1(...)
    PINS0(...)  PINS1(...)  DRIVE(...)  WAIT(n)  WAITPIN(cond, src)  SETN(n)  MARK(t)
"""
import argparse
import re
import sys

# --------------------------------------------------------------------------- ISA tables
ALU_FN = {"add": 0, "sub": 1, "and": 2, "or": 3, "xor": 4, "shl": 5, "shr": 6, "sltu": 7}
SHI_T = {"shli": 0, "shri": 1, "srai": 2, "rori": 3}
SYS_SUB = {"halt": 0, "jr": 1, "waitt": 2, "evset": 3, "evclr": 4, "evwait": 5, "signal": 6}
PIN_OP = {"pset": 0, "pclr": 1, "ptgl": 2, "oeset": 3, "oeclr": 4, "waith": 5, "waitl": 6}
BANKS = {"uio": 0, "uo": 1, "ui": 1}
REGS = {f"r{i}": i for i in range(8)}
REGS.update({"zero": 0, "lr": 7})


def _sym_table():
    s = {
        "LDATA0": 0x00, "LDATA1": 0x01, "LDATA2": 0x02, "LDATA3": 0x03,
        "LCTRL0": 0x04, "LCTRL1": 0x05, "LCTRL2": 0x06, "LCTRL3": 0x07,
        "MBOX": 0x08, "MBSTAT": 0x09, "GPOUT": 0x0A, "GPOE": 0x0B, "GPOD": 0x0C,
        "PINLO": 0x0D, "PINHI": 0x0E, "TIME": 0x0F, "TID": 0x10, "EVENTS": 0x11,
        # template fields
        "D_HOLD": 0, "D_BIT": 1, "D_NBIT": 2, "D_ZERO": 3, "D_ONE": 4, "D_REL": 5,
        "C_HOLD": 0, "C_LO": 1, "C_HI": 2, "C_TGL": 3,
        "X_TICK": 0, "X_RISE": 1, "X_FALL": 2, "X_HIGH": 3,
        "RISE": 0, "FALL": 1, "HIGH": 2, "LOW": 3,
    }
    names = ["CFG0", "CFG1", "DIV", "TPL01", "TPL23", "PINS0", "PINS1", "STAT"]
    for k in range(4):
        for r, n in enumerate(names):
            s[f"L{k}_{n}"] = 0x20 + 16 * k + r
        s[f"L{k}_CTRL"] = 0x20 + 16 * k + 7
    for i in range(8):
        s[f"UIO{i}"] = i
        s[f"UO{i}"] = 8 + i
        s[f"UI{i}"] = 16 + i
    return s


# helpers available inside expressions -------------------------------------------------
def PH(dat=0, clk=0, sample=0, exit=0):
    return (dat & 7) | (clk & 3) << 3 | (sample & 1) << 5 | (exit & 3) << 6


def TPL(p0=0, p1=0, p2=0, p3=0):
    return p0 | p1 << 8 | p2 << 16 | p3 << 24


def CFG0(nbits=8, msb=0, nph=1, rxen=0, auto=0, trig=0):
    assert 1 <= nbits <= 16 and 1 <= nph <= 4
    return (nbits & 15) | msb << 4 | (nph - 1) << 5 | rxen << 7 | auto << 8 | (trig & 3) << 9


def CFG1(en=1, dout=0, clk=0, aux=0, diff=0, dout_idle=0, clk_idle=0, aux_idle=0):
    return (en | dout << 1 | clk << 2 | aux << 3 | diff << 4
            | dout_idle << 5 | clk_idle << 6 | aux_idle << 7)


def PINS0(dout=0, clk=0, din=0):
    return dout | clk << 5 | din << 10


def PINS1(trig=0, clkin=0, aux=0):
    return trig | clkin << 5 | aux << 10


def DRIVE(dout=0, clk=0, aux=0, z=0, hold=0):
    assert 0 <= hold <= 3
    return 0 << 6 | dout | clk << 1 | aux << 2 | z << 3 | hold << 4


def WAIT(n):
    assert 0 <= n < (1 << 13)
    return 1 << 6 | (n & 0x3F) | (n >> 6) << 9


def WAITPIN(cond, src=0):
    return 2 << 6 | (cond & 3) | (src & 1) << 2


def SETN(n):
    return 3 << 6 | (n & 15)


def MARK(tok):
    return 4 << 6 | (tok & 0x3F)


HELPERS = dict(PH=PH, TPL=TPL, CFG0=CFG0, CFG1=CFG1, PINS0=PINS0, PINS1=PINS1,
               DRIVE=DRIVE, WAIT=WAIT, WAITPIN=WAITPIN, SETN=SETN, MARK=MARK,
               lo=lambda v: v & 0xFFFF, hi=lambda v: (v >> 16) & 0xFFFF)


class AsmError(Exception):
    pass


def _split_operands(s):
    """Split on commas that are not inside (), [] ."""
    out, depth, cur = [], 0, ""
    for ch in s:
        if ch in "([":
            depth += 1
        elif ch in ")]":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def _size(mn):
    return 2 if mn == "li16" else 1


class Assembler:
    def __init__(self, words=512):
        self.nwords = words
        self.syms = _sym_table()

    # ------------------------------------------------------------------ helpers
    def ev(self, expr, final):
        ns = dict(HELPERS)
        ns.update(self.syms)
        try:
            return int(eval(expr, {"__builtins__": {}}, ns))
        except NameError:
            if final:
                raise
            return 0                                   # forward reference in pass 1

    def reg(self, tok):
        t = tok.strip().lower()
        if t not in REGS:
            raise AsmError(f"bad register '{tok}'")
        return REGS[t]

    def mem_operand(self, tok, final):
        m = re.fullmatch(r"\[\s*(\w+)\s*(?:\+\s*(.+))?\]", tok.strip())
        if not m:
            raise AsmError(f"bad memory operand '{tok}' (use [rN+off])")
        return self.reg(m.group(1)), (self.ev(m.group(2), final) if m.group(2) else 0)

    @staticmethod
    def chk(v, lo, hi, what):
        if not lo <= v <= hi:
            raise AsmError(f"{what} {v} out of range [{lo}, {hi}]")
        return v

    # ------------------------------------------------------------------ assemble
    def assemble(self, src, fname="<src>"):
        lines = src.splitlines()
        for final in (False, True):
            pc = 0
            image = {}
            listing = []
            for ln, raw in enumerate(lines, 1):
                line = re.split(r";|//", raw, maxsplit=1)[0].strip()
                try:
                    while True:                               # labels
                        m = re.match(r"^([A-Za-z_]\w*)\s*:(.*)$", line)
                        if not m:
                            break
                        if not final:
                            if m.group(1) in self.syms and self.syms[m.group(1)] != pc:
                                raise AsmError(f"duplicate symbol '{m.group(1)}'")
                            self.syms[m.group(1)] = pc
                        line = m.group(2).strip()
                    if not line:
                        continue
                    m = re.match(r"^([A-Za-z_]\w*)\s*=\s*(.+)$", line)
                    if m:
                        self.syms[m.group(1)] = self.ev(m.group(2), final)
                        continue
                    parts = line.split(None, 1)
                    mn = parts[0].lower()
                    ops = _split_operands(parts[1]) if len(parts) > 1 else []
                    if mn == ".org":
                        pc = self.ev(ops[0], final)
                        continue
                    if mn == ".equ":
                        self.syms[ops[0]] = self.ev(ops[1], final)
                        continue
                    if mn == ".word":
                        for o in ops:
                            image[pc] = self.ev(o, final) & 0xFFFF
                            listing.append((pc, image[pc], raw.strip() if o == ops[0] else ""))
                            pc += 1
                        continue
                    words = self.encode(mn, ops, pc, final) if final else [0] * _size(mn)
                    for i, w in enumerate(words):
                        if final and pc in image:
                            raise AsmError(f"address 0x{pc:03x} assigned twice")
                        image[pc] = w
                        listing.append((pc, w, raw.strip() if i == 0 else ""))
                        pc += 1
                except AsmError as e:
                    raise AsmError(f"{fname}:{ln}: {e}\n    {raw.strip()}") from None
                except (NameError, SyntaxError, AssertionError, IndexError, ValueError) as e:
                    raise AsmError(f"{fname}:{ln}: {type(e).__name__}: {e}\n    {raw.strip()}") from None
        if image and max(image) >= self.nwords:
            raise AsmError(f"program exceeds memory ({max(image) + 1} > {self.nwords} words)")
        size = (max(image) + 1) if image else 0
        mem = [image.get(a, 0) for a in range(size)]
        return mem, dict(self.syms), listing

    def encode(self, mn, ops, pc, final):
        E = lambda x: self.ev(x, final)  # noqa: E731
        R = self.reg

        def rel(target, bits):
            off = E(target) - (pc + 1)
            lim = 1 << (bits - 1)
            self.chk(off, -lim, lim - 1, "branch offset")
            return off & ((1 << bits) - 1)

        # pseudo-instructions
        if mn == "nop":
            return [0x0000]
        if mn == "mov":
            mn, ops = "add", [ops[0], ops[1], "r0"]
        if mn == "ret":
            mn, ops = "jr", ["r7"]
        if mn == "call":
            mn = "jal"
        if mn == "b":
            mn, ops = "beq", ["r0", "r0", ops[0]]
        if mn in ("bz", "bnz"):
            mn, ops = ("beq" if mn == "bz" else "bne"), [ops[0], "r0", ops[1]]
        if mn == "subi":
            mn, ops = "addi", [ops[0], ops[1], f"-({ops[2]})"]
        if mn == "li16":
            v = E(ops[1]) & 0xFFFF
            rd = R(ops[0])
            return [0x2000 | rd << 9 | (v & 0xFF), 0x3000 | rd << 9 | (v >> 8)]

        if mn in ALU_FN:
            return [0x0000 | R(ops[0]) << 9 | R(ops[1]) << 6 | R(ops[2]) << 3 | ALU_FN[mn]]
        if mn == "addi":
            v = self.chk(E(ops[2]), -32, 31, "addi immediate")
            return [0x1000 | R(ops[0]) << 9 | R(ops[1]) << 6 | (v & 0x3F)]
        if mn == "li":
            v = self.chk(E(ops[1]), -256, 255, "li immediate (use li16)")
            return [0x2000 | R(ops[0]) << 9 | (v & 0x1FF)]
        if mn in ("lui", "andi"):
            v = self.chk(E(ops[1]), 0, 255, f"{mn} immediate")
            return [0x3000 | R(ops[0]) << 9 | (mn == "andi") << 8 | v]
        if mn in SHI_T:
            n = self.chk(E(ops[2]), 0, 15, "shift amount")
            return [0x4000 | R(ops[0]) << 9 | R(ops[1]) << 6 | SHI_T[mn] << 4 | n]
        if mn in ("ld", "st"):
            base, off = self.mem_operand(ops[1], final)
            self.chk(off, 0, 63, "memory offset")
            return [(0x5000 if mn == "ld" else 0x6000) | R(ops[0]) << 9 | base << 6 | off]
        if mn in ("beq", "bne"):
            return [(0x7000 if mn == "beq" else 0x8000) | R(ops[0]) << 9 | R(ops[1]) << 6 | rel(ops[2], 6)]
        if mn in ("bbc", "bbs"):
            bit = self.chk(E(ops[1]), 0, 15, "bit index")
            return [0x9000 | R(ops[0]) << 9 | bit << 5 | (mn == "bbs") << 4 | rel(ops[2], 4)]
        if mn in ("jmp", "jal"):
            t = self.chk(E(ops[0]), 0, 0xFFF, "jump target")
            return [(0xA000 if mn == "jmp" else 0xB000) | t]
        if mn in SYS_SUB:
            sub = SYS_SUB[mn]
            low = 0
            if mn in ("jr", "waitt"):
                low = R(ops[0]) << 5
            elif mn in ("evset", "evclr", "evwait"):
                low = self.chk(E(ops[0]), 0, 7, "event number")
            return [0xC000 | sub << 8 | low]
        if mn in PIN_OP:
            if mn in ("oeset", "oeclr"):
                bank, mask = 0, E(ops[0])
            else:
                if ops[0].lower() not in BANKS:
                    raise AsmError(f"bank must be uio/uo/ui, got '{ops[0]}'")
                bank, mask = BANKS[ops[0].lower()], E(ops[1])
            self.chk(mask, 0, 255, "pin mask")
            return [0xD000 | PIN_OP[mn] << 9 | bank << 8 | mask]
        if mn == "in":
            port = self.chk(E(ops[1]), 0, 255, "port")
            return [0xE000 | R(ops[0]) << 9 | port]
        if mn == "out":
            port = self.chk(E(ops[0]), 0, 255, "port")
            return [0xE000 | R(ops[1]) << 9 | 1 << 8 | port]
        if mn == "outi":
            port = E(ops[0])
            if 4 <= port <= 7:
                sp = port - 4                 # LCTRLk
            elif 0 <= port <= 3:
                sp = 4 + port                 # LDATAk
            else:
                raise AsmError("outi port must be LCTRL0-3 or LDATA0-3")
            v = self.chk(E(ops[1]), 0, 511, "outi immediate")
            return [0xF000 | sp << 9 | v]
        raise AsmError(f"unknown mnemonic '{mn}'")


def assemble(src, fname="<src>", words=512):
    return Assembler(words).assemble(src, fname)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src")
    ap.add_argument("-o", "--out", help="hex output ($readmemh format)")
    ap.add_argument("-l", "--lst", help="listing output")
    ap.add_argument("--words", type=int, default=512, help="memory size in words")
    a = ap.parse_args()
    with open(a.src) as f:
        src = f.read()
    try:
        mem, syms, listing = assemble(src, a.src, a.words)
    except AsmError as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)
    out = a.out or re.sub(r"\.\w+$", "", a.src) + ".hex"
    with open(out, "w") as f:
        f.write("".join(f"{w:04x}\n" for w in mem))
    if a.lst:
        with open(a.lst, "w") as f:
            for addr, w, text in listing:
                f.write(f"{addr:03x}  {w:04x}  {text}\n")
    print(f"{a.src}: {len(mem)} words -> {out}")


if __name__ == "__main__":
    main()
