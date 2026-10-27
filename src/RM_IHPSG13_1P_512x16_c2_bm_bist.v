// Black-box stub of the IHP SRAM macro for synthesis and lint (the real cell comes from
// macro/RM_IHPSG13_1P_512x16_c2_bm_bist/ via MACROS in src/config.json). Never simulate
// this file: use the IHP behavioural model in the same macro directory instead.
/* verilator lint_off UNDRIVEN */
/* verilator lint_off UNUSEDSIGNAL */
(* blackbox *)
module RM_IHPSG13_1P_512x16_c2_bm_bist (
    input         A_CLK,
    input         A_MEN,
    input         A_WEN,
    input         A_REN,
    input  [8:0]  A_ADDR,
    input  [15:0] A_DIN,
    input         A_DLY,
    output [15:0] A_DOUT,
    input  [15:0] A_BM,
    input         A_BIST_CLK,
    input         A_BIST_EN,
    input         A_BIST_MEN,
    input         A_BIST_WEN,
    input         A_BIST_REN,
    input  [8:0]  A_BIST_ADDR,
    input  [15:0] A_BIST_DIN,
    input  [15:0] A_BIST_BM
);

endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNDRIVEN */
