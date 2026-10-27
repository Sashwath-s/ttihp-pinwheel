# RM_IHPSG13_1P_512x16_c2_bm_bist

512 x 16 single-port SRAM from the IHP Open PDK, used as Pinwheel's unified memory (`src/pw_mem.v` with `PW_MEM_IHP_SRAM`).

**Source:** [IHP-Open-PDK](https://github.com/IHP-GmbH/IHP-Open-PDK) `ihp-sg13g2/libs.ref/sg13g2_sram/` (`ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram` is a symlink to it) at commit `2bbec755dc67ca3db0261c3d6163e15735d66710`. That's the revision `tt-gds-action@ihp-cmos5l` installs, and the files are byte-identical to `dev` as of 2026-10-27. Licensed under Apache-2.0 by the IHP PDK Authors.

| File | Use |
|---|---|
| `*.gds`, `*.lef` | layout and abstract (236.8 x 191.34 um; signal pins Metal2 on the bottom edge; power pins vertical Metal4 straps; OBS Metal1-Metal4) |
| `*_typ_1p20V_25C.lib`, `*_slow_1p08V_125C.lib`, `*_fast_1p32V_m55C.lib` | timing for STA (mapped to the LibreLane corners in `src/config.json`) |
| `*.cdl` | netlist for LVS (the macro is abstracted during extraction) |
| `RM_IHPSG13_1P_512x16_c2_bm_bist.v`, `RM_IHPSG13_1P_core_behavioral_bm_bist.v` | behavioural simulation model; compile with `FUNCTIONAL` defined |

The synthesis/lint black box is `src/RM_IHPSG13_1P_512x16_c2_bm_bist.v`.

Timing highlights from the liberty, checked against the design by `scripts/macro_timing.py`:

- clk to `A_DOUT` access: 3.80 ns typ, 6.36 ns slow, 2.35 ns fast.
- `A_ADDR`/`A_DIN` hold requirement: up to 0.84 ns at the slow corner.
- `A_CLK` max transition: 0.476 ns.
