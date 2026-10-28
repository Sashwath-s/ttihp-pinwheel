# Pinwheel — a barrel-threaded protocol emulator ASIC

Target: Jane Street protocol-emulator ASIC competition, Tiny Tapeout IHP `sg13cmos5l`, 6x4 tiles, 50 MHz nominal clock.
Status: **v0 architecture** — RTL, assembler and simulation tests in this repo implement everything marked v0.

---

## 1. Idea in one paragraph

Pinwheel is a **4-thread barrel processor** (one instruction per thread every 4 clocks, fixed round-robin, no hazards, no stalls of one thread caused by another) wrapped around **four waveform lanes**. Threads do the *byte-level* work (framing, command parsing, CRC, buffering, host communication) at a deterministic 12.5 MIPS each; lanes do the *bit-level* work with **1-clock timing resolution**, executing a stream of commands (`DATA`, `DRIVE`, `WAIT`, `WAITPIN`, `SETN`, `MARK`) through a small, fully programmable **per-bit phase template**. A protocol is firmware plus a 32-bit template, not a fixed block.

### Why this and not a PIO/PRU clone

| | RP2040 PIO | TI PRU | **Pinwheel** |
|---|---|---|---|
| Program store | 32 instr / block, flops, 4 read ports | 8 KB SRAM per core | **single-port SRAM shared by 4 threads** (barrel ⇒ exactly one fetch per cycle) |
| Concurrency | 4 SMs | 1 core (2 per PRUSS) | 4 hardware threads + 4 lanes |
| Bit timing | instruction-cycle exact (delay/side-set) | instruction-cycle exact, 200 MHz | **lane-cycle exact**, independent of software |
| Byte-level compute | none (no add, 2 scratch regs) | full 32-bit RISC | 16-bit RISC, 7 GPRs/thread |
| Timing proof burden | every program path must be cycle-counted | same | **lanes are cycle-exact by construction**; software only has to meet *throughput* (FIFO never empties) |

Key consequences:

1. **Memory density.** PIO's 32-instruction limit exists because 4 state machines fetch every cycle. A barrel fetches once per cycle, so instructions *and data* live in one dense single-port SRAM.
2. **Deterministic memory access.** A `LD`/`ST` uses the issuing thread's *own next fetch slot*, so it costs that thread exactly one extra slot and never perturbs another thread. Every instruction has a fixed cost (1 slot, `LD`/`ST` 2 slots).
3. **Timing decoupled from code.** Software pushes time-tagged work into lane FIFOs; the lane emits it with 1-cycle precision. Wake-up jitter of a thread (one slot = 4 cycles) never reaches the pins.

---

## 2. Block diagram

```
                 +-------------------------------------------------------------+
 host SPI  ----> | pw_host_spi --> host bus --+--> regs (TCTRL, PCs, MBSTAT..) |
 (RP2040)  <---- |   (mode 0 slave)           +--> mailbox FIFOs x4 (h2t/t2h) |
                 |                            +--> MEM port (slot arbitration) |
                 |                                                             |
                 |   +----------- pw_core (barrel, 4 threads) ------------+    |
                 |   |  F  -> D -> X -> W -> W2(-> F: data slot)           |    |
                 |   |  PC[4], RF[4][8]x16, TIME, EVENTS, GPIO regs        |    |
                 |   +---+--------------------------+----------------------+    |
                 |       | IO ports (IN/OUT/OUTI)    | single-port MEM          |
                 |   +---v--------+  x4              +--> pw_mem (SRAM / latch) |
                 |   | pw_lane    | cmd FIFO -> phase engine -> pins            |
                 |   |            | <- RX FIFO <- sampler <- pins               |
                 |   +------------+                                             |
                 |   pin mux: lanes (priority L0..L3) > GPIO; open-drain option |
                 +-------------------------------------------------------------+
```

---

## 3. Pins

| TT pin | Function |
|---|---|
| `ui_in[4]` | HOST_CS_N |
| `ui_in[5]` | HOST_SCK |
| `ui_in[6]` | HOST_MOSI |
| `uo_out[7]` | HOST_MISO |
| `uio[7:0]` | protocol bidir pins P0–P7 (output, input, open-drain capable) |
| `uo_out[6:0]` | protocol output pins P8–P14 |
| `ui_in[3:0]`, `ui_in[7]` | protocol input pins P16–P19, P23 |

**Pin index space** (5-bit, used by lane config and docs):

| Index | Pin | Out | In |
|---|---|---|---|
| 0–7 | `uio[0..7]` | yes (+OE, +open-drain) | yes (synchronized) |
| 8–14 | `uo_out[0..6]` | yes | reads back output value (registered, 1 cycle) |
| 15 | `uo_out[7]` (HOST_MISO) | no | reads back (registered) |
| 16–23 | `ui_in[0..7]` | no | yes (synchronized) |
| 24–31 | reserved (future: lane-to-lane virtual pins) | no | reads 0 |

All external inputs pass through a 2-flop synchronizer. Lane **data sampling** taps the first flop (1-cycle latency); lane **triggers/edges** and `WAITH/WAITL` use the second flop (metastability-safe control).

---

## 4. Thread processor

### 4.1 Barrel pipeline

A 2-bit `slot` counter selects the thread in F. Stage registers carry the thread id.

| Stage | Cycle | Work |
|---|---|---|
| F | c | memory address = `PC[t]`, or the pending `LD`/`ST` address of thread *t* (from W2), or a host access |
| D | c+1 | memory data valid → decode, read RF (2 ports) |
| X | c+2 | ALU, branches, IO side effects, block check, `PC[t]` update |
| W | c+3 | RF write (1 port) |
| W2 | c+4 | holds thread *t*'s pending memory op — exactly when F is back on thread *t* |

There are no hazards: thread *t*'s next instruction is read from the RF at c+5, after its previous write at c+3.

**Blocking** (`IN` from an empty FIFO, `OUT` to a full one, `WAITT`, `WAITH/L`, `EVWAIT`): the instruction has no side effects and `PC` is not advanced, so it re-executes in the thread's next slot. Other threads are unaffected.

**Timing model.** Every instruction costs 1 slot (4 clocks) except `LD`/`ST` (2 slots). A thread sees `TIME` advance by exactly 4 between consecutive slots, so `WAITT` deadlines that are multiples of 4 apart are met with zero jitter.

### 4.2 Memory

Unified 16-bit-word single-port memory: **512 words** (`MEM_AW=9`). With `PW_MEM_IHP_SRAM` defined, `pw_mem` is one IHP `RM_IHPSG13_1P_512x16_c2_bm_bist` macro: `A_MEN=en`, `A_WEN=en&we`, `A_REN=en&~we` (no write-through), `A_BM=16'hFFFF`, `A_DLY=1` (the model requires it), BIST port tied off. Without the define it's a behavioural flop array with identical timing: synchronous read, data valid the next cycle and held until the next read. Simulate the macro with IHP's `FUNCTIONAL` model, as TT's gate-level flow does. Slot ownership:

1. W2 pending `LD`/`ST` of the slot's thread (always wins; this *is* the thread's own slot).
2. Host access, if the slot's thread is not fetching (halted) or the host has waited ≥ 4 cycles (then the slot's thread loses one fetch: perturbs that thread's timing by one slot — use mailboxes, not memory, for real-time host traffic).
3. Instruction fetch.

Silicon: map to an IHP SRAM macro (TT has a working 1024x8 test design on this node), or a latch array fallback (128–256 words). See §9.

### 4.3 Architectural state per thread

`PC` (`MEM_AW` bits, reset value = thread id, so thread *k* starts at address *k* — put a `JMP` vector table at 0–3), `r1`–`r7` (16-bit), `r0` ≡ 0, run / step / blocked / attention / illegal flags.

Shared: `TIME` (16-bit free-running cycle counter), `EVENTS` (8 flags), `GPOUT[14:0]`, `GPOE[7:0]`, `GPOD[7:0]`.

---

## 5. Instruction set (16-bit)

`s6/s9/s4` = signed immediates, `u6/u8/u12` = unsigned. Branch offsets are relative to `PC+1`.

| op | Format `[15:12] [11:0]` | Assembly | Semantics |
|---|---|---|---|
| 0 | `0000 ddd aaa bbb fff` | `add/sub/and/or/xor/shl/shr/sltu rd, ra, rb` | `rd = ra fn rb` (shift amount `rb[3:0]`, `sltu` → 1/0) |
| 1 | `0001 ddd aaa iiiiii` | `addi rd, ra, s6` | `rd = ra + s6` |
| 2 | `0010 ddd iiiiiiiii` | `li rd, s9` | `rd = sext(s9)` |
| 3 | `0011 ddd 0 iiiiiiii` | `lui rd, u8` | `rd = (u8<<8) \| rd[7:0]` |
| 3 | `0011 ddd 1 iiiiiiii` | `andi rd, u8` | `rd = rd & u8` |
| 4 | `0100 ddd aaa tt nnnn` | `shli/shri/srai/rori rd, ra, n` | shift/rotate by immediate |
| 5 | `0101 ddd aaa iiiiii` | `ld rd, [ra+u6]` | `rd = MEM[ra+u6]` (2 slots) |
| 6 | `0110 sss aaa iiiiii` | `st rs, [ra+u6]` | `MEM[ra+u6] = rs` (2 slots) |
| 7 | `0111 sss aaa iiiiii` | `beq rs, ra, label` | branch if equal (±32) |
| 8 | `1000 sss aaa iiiiii` | `bne rs, ra, label` | branch if not equal |
| 9 | `1001 sss nnnn v iiii` | `bbc/bbs rs, bit, label` | branch if `rs[bit]==v` (−8..+7) |
| A | `1010 u12` | `jmp label` | `PC = u12` |
| B | `1011 u12` | `jal label` | `r7 = PC+1; PC = u12` |
| C | `1100 ssss xxxxxxxx` | system | see below |
| D | `1101 ooo b mmmmmmmm` | pin ops | see below |
| E | `1110 rrr d pppppppp` | `in rd, port` / `out port, ra` | port I/O (may block) |
| F | `1111 ppp iiiiiiiii` | `outi port, u9` | `OUT` an immediate to `LCTRL0-3` (p=0-3) or `LDATA0-3` (p=4-7) |

`0x0000` = `add r0,r0,r0` = `nop`.

**SYS** (`[11:8]`): `0 halt` (PC advances, so the host can resume), `1 jr ra` (`ra` in `[7:5]`), `2 waitt ra` (block while `(int16)(TIME-ra) < 0`), `3 evset n`, `4 evclr n`, `5 evwait n` (block until set, then clear), `6 signal` (set this thread's host-attention flag), `7–15` illegal → halt + illegal flag.

**PIN** (`[11:9]`): `0 pset`, `1 pclr`, `2 ptgl` on `GPOUT` (bank 0 = uio P0–7, bank 1 = uo P8–14); `3 oeset`, `4 oeclr` on `GPOE`; `5 waith` (block until all masked pins are 1), `6 waitl` (until all are 0) — bank 0 = `uio_in`, bank 1 = `ui_in`.

### 5.1 IO ports

| Port | Name | IN | OUT |
|---|---|---|---|
| 0x00–03 | `LDATAk` | pop lane *k* RX word (blocks if empty) | push DATA command (blocks if full) |
| 0x04–07 | `LCTRLk` | block until lane *k* idle and cmd FIFO empty; returns status | push CTRL command (blocks if full) |
| 0x08 | `MBOX` | pop own host→thread FIFO (blocks) | push own thread→host FIFO (blocks) |
| 0x09 | `MBSTAT` | `[3:0]` h2t count · `[7:4]` t2h count · `[8]` t2h full · `[9]` h2t empty (non-blocking) | — |
| 0x0A/0B/0C | `GPOUT/GPOE/GPOD` | read | write |
| 0x0D | `PINLO` | `{uo_out[7:0], uio_in[7:0]}` | — |
| 0x0E | `PINHI` | `{8'h0, ui_in[7:0]}` | — |
| 0x0F | `TIME` | cycle counter | — |
| 0x10 | `TID` | thread id | — |
| 0x11 | `EVENTS` | read | write |
| 0x20+16k+r | lane *k* register *r* | read | write |

---

## 6. Waveform lanes

Each lane: 4-entry command FIFO (17-bit: kind + 16-bit arg), 2-entry RX FIFO, phase engine, sampler.

### 6.1 Registers (per lane, port `0x20 + 16k + r`)

| r | Name | Fields |
|---|---|---|
| 0 | `CFG0` | `[3:0]` NBITS (0=16) · `[4]` MSB_FIRST · `[6:5]` NPH−1 · `[7]` RXEN · `[8]` AUTO · `[10:9]` TRIG cond (0 rise, 1 fall, 2 high, 3 low) |
| 1 | `CFG1` | `[0]` EN · `[1]` DOUT_EN · `[2]` CLK_EN · `[3]` AUX_EN · `[4]` AUX_DIFF (AUX = ~DOUT during bits) · `[5]` DOUT_IDLE · `[6]` CLK_IDLE · `[7]` AUX_IDLE |
| 2 | `DIV` | phase length − 1 (cycles) |
| 3 | `TPL01` | phase 0 `[7:0]`, phase 1 `[15:8]` |
| 4 | `TPL23` | phase 2 `[7:0]`, phase 3 `[15:8]` |
| 5 | `PINS0` | `[4:0]` DOUT · `[9:5]` CLK · `[14:10]` DIN |
| 6 | `PINS1` | `[4:0]` TRIG · `[9:5]` CLKIN · `[14:10]` AUX |
| 7 | `STAT` (R) / `CTRL` (W) | R: `[2:0]` cmd level · `[4:3]` rx level · `[5]` busy · `[6]` rx overflow · `[7]` dout · `[8]` clk. W: `[0]` flush cmd · `[1]` flush rx · `[2]` clear overflow |

Writing `CFG1.EN=0` resets the lane (FIFOs flushed, pins released to GPIO). `EN` 0→1 drives the idle levels.

### 6.2 Phase template (8 bits per phase, up to 4 phases per bit)

| Bits | Field | Values |
|---|---|---|
| `[2:0]` | DAT action on entry | 0 hold · 1 bit · 2 ~bit · 3 zero · 4 one · 5 release (hi-Z) |
| `[4:3]` | CLK action on entry | 0 hold · 1 low · 2 high · 3 toggle |
| `[5]` | SAMPLE | sample DIN into the RX word on entry |
| `[7:6]` | EXIT | 0 after DIV+1 cycles · 1 CLKIN rise · 2 CLKIN fall · 3 CLKIN high, then DIV+1 cycles (clock stretching) |

A word is `NBITS` bits; bit *i* is `data[i]` (LSB first) or `data[N-1-i]` (MSB first). Received bits are right-justified the same way. Consecutive `DATA` commands chain with **no gap**: the next word's phase 0 starts on the same cycle the previous word's last phase ends.

### 6.3 Commands

`OUT LDATAk, r` pushes a DATA word. `OUT LCTRLk, r` / `OUTI LCTRLk, imm9` pushes a CTRL command `[8:6]=op [5:0]=arg`:

| op | Name | arg |
|---|---|---|
| 0 | `DRIVE` | `[0]` DOUT · `[1]` CLK · `[2]` AUX · `[3]` Z (release DOUT/AUX) · `[5:4]` hold ticks |
| 1 | `WAIT` | ticks (`{[15:9],[5:0]}` with the register form) |
| 2 | `WAITPIN` | `[1:0]` cond (rise/fall/high/low) · `[2]` pin (0 TRIG, 1 CLKIN) |
| 3 | `SETN` | NBITS for following DATA words (0=16) |
| 4 | `MARK` | push `arg` into the RX FIFO (fence / sync token) |

**AUTO mode**: a DATA word (the queued one, or `0xFFFF` if the FIFO is empty) starts only when the TRIG condition holds. UART RX = AUTO + TRIG fall; SPI slave = AUTO + TRIG low on CS.

### 6.4 Templates for the core protocols

| Protocol | Phases (`PH(dat,clk,sample,exit)`) | Notes |
|---|---|---|
| UART TX/RX | `PH(BIT)` · `PH(-,-,S)` | DIV = half bit − 1; TX word `{1,data,0}`, 10 bits LSB first; RX = AUTO, TRIG fall |
| SPI mode 0 | `PH(BIT)` · `PH(-,HI)` · `PH(-,-,S)` · `PH(-,LO)` | SCK = clk/(4·(DIV+1)); modes 1–3 = rotate/invert |
| I2C | `PH(BIT)` · `PH(-,HI,-,EX_HIGH)` · `PH(-,-,S)` · `PH(-,LO)` | open-drain pins, 9-bit words `{byte, ack}`, clock stretching built in |
| Manchester / 10BASE-T TX | `PH(NBIT)` · `PH(BIT)` | 60 MHz clk, DIV=2 → 10 Mb/s; AUX_DIFF drives TD− |
| WS2812 | `PH(ONE)` · `PH(BIT)` · `PH(ZERO)` | 0.4 µs phases |
| SPI slave mode 0 | `PH(BIT,-,-,RISE)` · `PH(-,-,S,FALL)` | AUTO, TRIG = CS low, CLKIN = SCK |
| PS/2 host RX | `PH(-,-,-,FALL)` · `PH(-,-,S)` | AUTO, TRIG = DATA fall, CLKIN = PS/2 CLK |

---

## 7. Host interface (SPI mode 0 slave, MSB first, SCK ≤ clk/8)

```
write: CMD(0b0i000000) ADDR_H ADDR_L  D0_H D0_L  D1_H D1_L ...   (until CS_N rises)
read : CMD(0b1i000000) ADDR_H ADDR_L  N    D0_H D0_L ... D(N-1)_L  (N = word count, 0 = 256)
```

`i` = auto-increment. During the CMD byte MISO returns a status byte `{t2h_avail[3:0], attn[3:0]}`. Reads are prefetched exactly N times, so FIFO reads never pop more than requested.

| Address | Name | Access |
|---|---|---|
| `0x0000–0x7FFF` | MEM | R/W (word addressed, wraps) |
| `0x8000` | ID = `0x5057` ("PW") | R |
| `0x8001` | VERSION `{MEM_AW, ver}` | R |
| `0x8002` | TCTRL | W: `[3:0]` run · `[7:4]` halt · `[11:8]` step · `[15:12]` clear attention. R: `[3:0]` running · `[7:4]` blocked · `[11:8]` attention · `[15:12]` illegal |
| `0x8003` | MBSTAT | R: `[3:0]` t2h not-empty · `[7:4]` h2t full · `[11:8]` h2t empty · `[15:12]` t2h full |
| `0x8004–7` | PC0–3 | R/W (write while halted) |
| `0x8008` | TIME | R |
| `0x8009–B` | GPOUT/GPOE/GPOD | R |
| `0x800C/D` | PINLO/PINHI | R |
| `0x800E` | EVENTS | R |
| `0x8010–3` | MBOX0–3 | W: push host→thread k · R: pop thread→host k (0 if empty) |

---

## 8. Verification plan

| Layer | Method | Status |
|---|---|---|
| ISA | Python ISS (`sw/pwsim.py`) is the golden model. `sw/cosim.py` generates constrained-random 4-thread programs (`sw/randprog.py`: ALU/imm/shift, LD/ST, all 4 branch kinds, counted loops, JAL/JR), runs ISS and RTL (`PW_TRACE` retire trace), and compares every thread's trace and the final memory. Memory is loaded and dumped through the host bus, so every word goes through the real memory port | **70/70 seeds on IHP's `RM_IHPSG13_1P_512x16` model** (`--mem sram`) and 70/70 on the flop array; ~400 retire events + 512 memory words each; a mutated ISS is caught |
| Lanes + system | `test/tb_demo.sv`: real host-SPI program load + readback; UART TX/RX (incl. ±3% baud), SPI mode 0 (SCK period measured), I2C master vs EEPROM model (START/RESTART/STOP, ACK/NACK, clock stretching on every ACK), UART RX traffic concurrent with SPI and I2C | **31/31 on the SRAM model** (`run_xsim.bat tb_demo sram`) and on the flop array |
| Synthesis lint | Vivado OOC synthesis (`scripts/vivado_synth.tcl`): no latches, multi-drivers or loops | clean |
| Area / timing | Yosys onto IHP `sg13cmos5l` cells (`scripts/yosys_area.py`) + cell-level STA (`scripts/libsta.py`) | §9 |
| Formal | SymbiYosys: thread isolation (a thread's RF/PC only change in its own X/W slot), FIFO invariants, lane phase length = DIV+1, no X on outputs | planned |
| Gate level | TT GL test flow (`test/test.py` cocotb smoke test) | written, **not yet run** (no iverilog/cocotb locally) |
| FPGA | Zybo (xc7z010) bring-up with a Pico as host | planned |

---

## 9. Area, timing and silicon integration

The 6x4 die is **1289.28 × 710.64 µm = 0.916 mm²** (tt-support-tools `tile_sizes.yaml` for `ihp-sg13cmos5l`). At the template's 60% placement density that leaves ~550k µm² for cells. Earlier estimates used the blog's ~0.72 mm²; the numbers below use the real die.

### 9.1 Area on IHP `sg13cmos5l` cells

Synthesized with Yosys 0.70 (`scripts/yosys_area.py`; typ 1.20 V liberty). This hierarchical run uses the flop memory, to show why it can't ship:

| Block | Inst | µm² / inst | µm² total | flops |
|---|---:|---:|---:|---:|
| Memory 256×16 (flop array) | 1 | 331,148 | 331,148 | 4,112 |
| Waveform lanes | 4 | 29,233 | 116,933 | 1,176 |
| Core (4 threads, 28×16 RF, pipeline, GPIO) | 1 | 79,255 | 79,255 | 764 |
| Mailbox FIFOs (4×16 each) | 8 | 5,695 | 45,563 | 584 |
| Top glue (pin mux, IO decode, host regs, syncs) | 1 | 17,860 | 17,860 | 84 |
| Host SPI bridge | 1 | 11,842 | 11,842 | 136 |
| **Total excluding memory** | | | **271,454** | 2,744 |

One lane is 20.4k µm² of engine + config, 6.0k for the command FIFO and 2.9k for the RX FIFO. With the flop memory, the flat buffered total is 607k µm², **110% of the budget**.

**With the SRAM macro** (`--sram`, the configuration that ships): **275.6k µm² of standard cells, 53% of the ~517k µm² placeable** once the macro and a 10 µm halo are removed, plus the 45.3k µm² macro.

### 9.2 Memory: IHP SRAM macro

`sg13cmos5l_sram` is a symlink to `sg13g2_sram`. The macros use only Metal1–Metal4, and CMOS5L's stack is Metal1–4 + TopMetal1. Native 16-bit single-port options:

| Macro | Footprint (µm) | µm² | Logic share of the remaining budget* |
|---|---|---:|---:|
| `RM_IHPSG13_1P_256x16_c2_bm_bist` | 236.8 × 118.8 | 28,127 | 52% |
| **`RM_IHPSG13_1P_512x16_c2_bm_bist`** | 236.8 × 191.3 | 45,309 | **53%** |
| `RM_IHPSG13_1P_1024x16_c2_bm_bist` | 236.8 × 336.5 | 79,674 | 56% |

\*Macro plus a 10 µm halo removed from the die, 60% of the rest available to ~276k µm² of logic.

Chosen: **512×16** (`MEM_AW=9`). `pw_mem.v` maps it as `A_MEN=en`, `A_WEN=en&we`, `A_REN=en&~we` (no write-through), `A_BM=16'hFFFF`, `A_DLY=1` (required by the model), BIST tied off. It is verified on IHP's behavioural model (§8).

### 9.3 Timing

Estimates from `scripts/libsta.py`: NLDM cell delays at real fanout loads, setup and hold from the liberties, ideal clocks, no wire RC. The flop-to-flop worst is 5.6 ns typ / 8.9 ns slow (lane `dcnt` zero-detect → phase entry). Without synthesis-time buffering, five X1-driven nets with fanout 14–53 pushed it to 10.3 ns.

Macro paths from its own liberty (`scripts/macro_timing.py`, 20 ns clock):

| | typ | slow | fast |
|---|---:|---:|---:|
| clk → `A_DOUT` access (≈32 fF) | 3.80 ns | 6.36 ns | 2.35 ns |
| read path: access + decode/RF read + setup into `dx_b` | 6.72 ns | 10.95 ns | 4.23 ns |
| worst input setup slack | 18.1 ns | 17.0 ns | 18.8 ns |
| **worst input hold slack** (`A_ADDR`; `A_DIN` similar) | **−0.18 ns** | **−0.26 ns** | −0.13 ns |
| control pins (`A_MEN/A_WEN/A_REN`) hold slack | ≥ +0.12 | ≥ +0.26 | ≥ +0.06 |

The macro needs 0.29–0.84 ns of hold on `A_ADDR`/`A_DIN`, while those nets arrive 0.22–0.58 ns after the clock, from flops through one mux. Expect the flow's hold repair to buffer those 25 nets. The macro liberties are in every STA corner, so it can see them. CTS skew between the flops and `A_CLK` adds to this. The macro also requires clock slew ≤ 0.476 ns at `A_CLK`, and its minimum pulse width is 0.12–0.40 ns.

### 9.4 Tiny Tapeout integration (`src/config.json`)

Pattern from [urish/ttihp-sram-test](https://github.com/urish/ttihp-sram-test), the SRAM test that TT taped out on IHP (SG13G2), adapted to CMOS5L:

- **Defines:** `VERILOG_DEFINES: ["PW_MEM_IHP_SRAM"]`.
- **Macro files:** `MACROS` with the GDS, LEF, three liberty corners (keyed by PVT: `*_typ_1p20V_25C`, `*_slow_1p08V_125C`, `*_fast_1p32V_m40C`) and the CDL. They're vendored in `macro/RM_IHPSG13_1P_512x16_c2_bm_bist/` from IHP-Open-PDK `2bbec755`, the revision `tt-gds-action@ihp-cmos5l` installs, byte-identical to `dev`.
- **Netlist stub:** `src/RM_IHPSG13_1P_512x16_c2_bm_bist.v` (`(* blackbox *)`), also listed in `info.yaml`.
- **Placement:** instance `u_pw.u_mem.u_sram` (the flattened Yosys name) at (307.355, 260.82) µm, R0. That x is what lines the PDN stripes up with the macro's straps (below), and it leaves standard-cell rows both above and below the SRAM. The bottom-edge Metal2 pins face the logic, and the macro is well clear of the TT pins (x < 192 on the top edge).
- **Power connections:** `PDN_MACRO_CONNECTIONS` ties `VDD!` and `VDDARRAY!` to `VPWR` and `VSS!` to `VGND`.
- **Sign-off settings, copied from the reference:** `MAGIC_MACRO_STD_CELL_SOURCE: PDK`, `ERROR_ON_MAGIC_DRC: false` (so check the Magic DRC report by hand), `MAGIC_EXT_ABSTRACT_CELLS: ["RM_IHPSG13_.*"]`.
- **Gate-level test:** `test/Makefile` simulates the vendored model, and `test/tb.v` now matches the CMOS5L template (no power ports).

**PDN on CMOS5L is the open risk.** TT forces `RT_MAX_LAYER=Metal4` for CMOS5L user blocks (`project_top_metal_layer`) and uses TopMetal1 for its own top-level grid. The SRAM blocks Metal1–4 over its whole footprint (Metal4 is free only on its pins). Its supplies are vertical Metal4 straps 2.81 µm wide:

- `VSS!`: 20, full height.
- `VDD!`: 4 full-height straps plus 16 in the bottom 38.8 µm.
- `VDDARRAY!`: 16 from y = 45.5 µm to the top.

The SG13G2 reference strapped Metal4 up to TopMetal1 inside the block, which CMOS5L forbids. Two attempts failed on GitHub before the current scheme:

1. **Run 1: a macro grid.** The stripe layer *is* the macro's pin layer, so a macro grid can never create a via, and pdngen rejects the empty grid (`PDN-0232`/`PDN-0233`).
2. **Run 2: exporting the macro straps as block pins.** Legal, but the SRAM is then connected only through TT's top level. OpenROAD's power analysis rejects the islands (`PSM-0069`, "unconnected shape on net VPWR"), and LVS would likely do the same.

**Current scheme: ordinary PDN stripes run straight over same-net straps.** In run 2, LibreLane put Metal4 stripe centres at VPWR = 2.88 + 10 + 50k µm and VGND = VPWR + width + spacing. The stripes are 2.1 µm wide and the straps 2.81 µm, so a stripe centred on a strap fits inside it with ±0.355 µm to spare. With the macro at x = 307.355 and `FP_PDN_VSPACING` = 3.05:

| Stripe (centre, µm) | Lands on (macro x) | Covers |
|---|---|---|
| VPWR 412.88 | full-height `VDD!` strap (104.12–106.93) | whole macro height |
| VGND 418.03 | full-height `VSS!` strap (109.27–112.08) | whole macro height |
| VPWR 312.88 | `VDDARRAY!` strap (4.26–7.07) | from y = 60 µm (macro) up to the top edge |

Four `PDN_OBSTRUCTIONS` rectangles cut every other stripe over the macro plus a 10 µm halo, and stop the `VDDARRAY!` stripe 14 µm inside its strap. That keeps it clear of the gap above the short `VDD!` straps. The three stripes continue into the core, so pdngen connects them to the standard-cell rails above and below the SRAM like any other stripe.

`scripts/check_tt_config.py` simulates this geometry against the macro LEF. It checks that every stripe piece over the SRAM lies inside a same-net strap and clear of the macro's Metal4 obstruction shapes, and that `VDD!`, `VDDARRAY!` and `VSS!` are each fed. All checks pass, and shifting the macro by 0.3 µm makes them fail, so the margin is real. The scheme depends on LibreLane's stripe rule staying the same; any change to the PDN offset, pitch, width or spacing needs the macro position recomputed.

Run 2 also gave the first real post-route numbers, with the macro in its old spot:

- **Routing:** DRC clean after 71 minutes of detailed routing.
- **Timing:** no violations. Worst setup slack is +2.51 ns at the slow corner and worst hold slack +0.10 ns at the fast corner, so hold repair fixed the SRAM inputs.
- **Slew:** 72 max-slew violations at the slow corner (up to 4.6 ns against a 2.5 ns limit), on long or high-fanout nets.

### 9.5 Next steps

1. **GDS run 3** (aligned stripes): power analysis, LVS, Magic DRC report, TT precheck, gate-level test.
2. **Repair the max-slew violations** (post-global-route design repair, or buffering the worst nets).
3. **If area or routing gets tight:** `MBOX_AW=1` (−~23k µm²); 2-deep lane command FIFOs (−~12k µm²). The fallback that avoids the macro entirely is a 256×16 latch RAM (~150k µm²).

---

## 10. Roadmap

* **v0 (this repo)**: core, lanes (templates, DRIVE/WAIT/WAITPIN/SETN/MARK, AUTO), host SPI, UART/SPI/I2C firmware + tests.
* **v1**: lane line coding (NRZI, bit stuffing with run length 5/6, Manchester RX with edge resync), per-lane CRC-16 with configurable poly (USB CRC5/16, CAN CRC15, SMBus PEC), edge-timestamp capture mode (logic analyzer / pulse measurement), host register debug access (read/write RF).
* **Stretch**: USB low-speed device (48 MHz), 10BASE-T TX (60 MHz, NLPs), SWD/JTAG probe firmware, CAN.
* **Toolchain**: assembler macros → small structured language; ISS with cycle-accurate lane model for firmware-in-the-loop timing checks.

## 11. Open questions / risks

1. SRAM macro power on `sg13cmos5l`: user blocks stop at Metal4, the macro blocks Metal1–4, so its Metal4 straps are exported as block power pins for TT's TopMetal1 grid. This needs a GDS run and TT confirmation (§9.4).
2. TT IO mux max toggle rate — limits SPI/10BASE-T speeds more than the core does.
3. Host SPI clocked through synchronizers caps SCK at clk/8; a source-synchronous SCK-domain front end would allow ~clk/2.
4. Input sampling latency (1 cycle data, 2 cycles control) bounds SPI master SCK to about clk/4.
