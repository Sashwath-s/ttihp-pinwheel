![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Pinwheel — protocol emulator ASIC

A software-defined protocol engine for the Jane Street protocol-emulator ASIC competition (Tiny Tapeout IHP `sg13cmos5l`, 6x4 tiles).

**Pinwheel = a 4-thread barrel processor + 4 cycle-exact waveform lanes.** Threads do byte-level work (framing, parsing, buffering, host I/O) at a deterministic 12.5 MIPS each. Lanes do bit-level work at 1-clock resolution through a programmable per-bit phase template. A protocol is firmware plus a template, not a fixed block. The barrel fetches once per cycle, so one dense single-port SRAM holds code and data for all four threads. That removes PIO's 32-instruction limit, and a load/store never perturbs another thread's timing.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the full spec (ISA, lanes, host protocol, verification, area).

## Status (v0)

| | |
|---|---|
| RTL | core, lanes, host SPI, mailboxes, pin mux — Verilog-2005, synthesizes clean |
| Firmware | UART TX, UART RX, SPI master, I2C master running on 4 threads at once: 146 words total ([sw/fw/demo4.pws](sw/fw/demo4.pws)) |
| System test | [test/tb_demo.sv](test/tb_demo.sv) — 31/31 checks (timing measured on the pins, I2C clock stretching, concurrent traffic) |
| Random co-sim | RTL vs Python ISS, retire traces + memory per thread — 70/70 seeds on the IHP SRAM model and on the flop array |
| Memory | 512 words. `PW_MEM_IHP_SRAM` = one IHP `RM_IHPSG13_1P_512x16` macro (45k µm²), verified on IHP's behavioural model; without the define, a flop array for simulation/FPGA |
| Area (Yosys, IHP cells) | 275.6k µm² of standard cells (~2,750 flops) = 53% of the 6x4 budget, plus the 45.3k µm² macro |
| TT integration | `src/config.json` + `src/pdn_cfg.tcl` + vendored macro; static checks pass (`scripts/check_tt_config.py`); macro timing checked (`scripts/macro_timing.py`) |
| Not done yet | GDS run (real P&R/STA/LVS), TT confirmation of macro power on CMOS5L, cocotb test not run locally, formal, FPGA, v1 lane codecs (NRZI/bit-stuffing/CRC) |

## Layout

```
src/            RTL (tt_um_pinwheel.v is the TT top; pw_* are the blocks)
sw/pwasm.py     assembler (labels, expressions, lane/template helper functions)
sw/pwsim.py     instruction-set simulator, golden model for co-simulation
sw/randprog.py  constrained-random program generator
sw/cosim.py     RTL-vs-ISS random regression (Vivado xsim)
sw/fw/          firmware (demo4.pws: UART TX/RX + SPI + I2C)
test/           tb_demo.sv / tb_rand.sv (xsim), run_xsim.bat; Makefile/tb.v/test.py (TT cocotb CI)
scripts/        vivado_synth.tcl (synthesis lint), yosys_area.py + libsta.py (IHP area/timing),
                macro_timing.py (SRAM setup/hold/access), check_tt_config.py (TT config checks)
macro/          IHP SRAM macro files (GDS, LEF, libs, CDL, behavioural model)
docs/           ARCHITECTURE.md (spec), info.md (TT datasheet page)
```

## Quick start (Windows + Vivado xsim)

```bash
python sw/pwasm.py sw/fw/demo4.pws -o sw/fw/demo4.hex -l sw/fw/demo4.lst
```

```bash
test\run_xsim.bat tb_demo
```

```bash
python sw/cosim.py --seeds 1-20
```

```bash
vivado -mode batch -source scripts/vivado_synth.tcl -tclargs build/vivado
```

`run_xsim.bat` and `cosim.py` expect Vivado at `C:\AMDDesignTools\2026.1\Vivado\bin`. Set `VIVADO` to override.

## Area estimate (Yosys on IHP sg13cmos5l cells)

The WebAssembly Yosys build runs anywhere Python does. The liberty file comes from the IHP-Open-PDK `dev` branch:

```bash
pip install --user yowasp-yosys
```

```bash
mkdir -p build/pdk
```

```bash
curl -sfL -o build/pdk/sg13cmos5l_stdcell_typ_1p20V_25C.lib https://raw.githubusercontent.com/IHP-GmbH/IHP-Open-PDK/dev/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/sg13cmos5l_stdcell_typ_1p20V_25C.lib
```

```bash
python scripts/yosys_area.py
```

The script writes `build/yosys/area_report.md` (per-block area, flops, flat total, timing estimate). With `--sram`, it counts the memory as the IHP macro, a black box with its LEF footprint, and writes `area_report_sram.md`. That needs the model and LEF in `build/pdk/` (see SRAM model below). Current results and the SRAM macro comparison are in ARCHITECTURE §9.

## SRAM model

On silicon the 512-word memory is one IHP `RM_IHPSG13_1P_512x16_c2_bm_bist` macro, selected with the `PW_MEM_IHP_SRAM` define. Without the define, `pw_mem` is a behavioural flop array: faster to simulate and fine for an FPGA, but far too big for the tile. The macro's GDS, LEF, libs, CDL and IHP's behavioural model are vendored in [macro/](macro/RM_IHPSG13_1P_512x16_c2_bm_bist/README.md), so these run with no downloads:

```bash
test
un_xsim.bat tb_demo sram
```

```bash
python sw/cosim.py --seeds 1-20 --mem sram
```

```bash
python scripts/yosys_area.py --sram
```

```bash
python scripts/macro_timing.py
```

`macro_timing.py` checks the macro's setup, hold, access time and pulse width from its liberty against the design. It needs the flat netlist from `yosys_area.py --sram` and the three sg13cmos5l stdcell liberties in `build/pdk/`.

## Tiny Tapeout submission notes

1. Create the GitHub repo from the [cmos5l template](https://github.com/TinyTapeout/ttihp-verilog-template/tree/cmos5l). Copy in `src/` (including our `config.json`, `pdn_cfg.tcl` and the SRAM black-box stub), `macro/`, `test/`, `docs/`, `info.yaml` and `sw/`. Keep the template's `.github/`.
2. The top module is `tt_um_sashwath_s_pinwheel` (in `src/tt_um_pinwheel.v`). Set `author` in `info.yaml` to your name or team.
3. `src/config.json` already integrates the 512×16 SRAM macro. It sets `VERILOG_DEFINES=PW_MEM_IHP_SRAM` and adds `MACROS` with the vendored GDS/LEF/libs/CDL and the placement. `PDN_MACRO_CONNECTIONS` and the custom `pdn_cfg.tcl` handle power. Run `python scripts/check_tt_config.py` after any change.
4. **SRAM power on CMOS5L:** user blocks stop at Metal4, the same layer as the SRAM's power pins. So the macro is placed where ordinary PDN stripes run straight over its own power straps, with `PDN_OBSTRUCTIONS` cutting every other stripe (ARCHITECTURE §9.4). Don't move the macro or change the PDN offset, pitch, width or spacing without re-running `check_tt_config.py`.
5. **After each GDS run, check:** power analysis, LVS, the Magic DRC report (`ERROR_ON_MAGIC_DRC` is off), post-route timing and max-slew violations.

## Writing a protocol

```asm
; UART TX on UO0 at 1 Mbaud (50 MHz clock): 2 phases x 25 cycles per bit
        li   r1, CFG0(nbits=10, nph=2)               ; 10-bit frames, LSB first
        out  L0_CFG0, r1
        li   r1, 24
        out  L0_DIV, r1
        li16 r1, TPL(PH(D_BIT), PH())                ; phase 0 drives the bit
        out  L0_TPL01, r1
        li   r1, PINS0(dout=UO0)
        out  L0_PINS0, r1
        li   r1, CFG1(en=1, dout=1, dout_idle=1)
        out  L0_CFG1, r1
loop:   in   r1, MBOX                                ; byte from host
        shli r1, r1, 1                               ; start bit
        or   r1, r1, r2                              ; r2 = 0x200 stop bit
        out  LDATA0, r1                              ; lane shifts it out, cycle exact
        jmp  loop
```
