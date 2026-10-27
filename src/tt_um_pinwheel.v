/*
 * Copyright (c) 2026 Pinwheel authors
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Tiny Tapeout wrapper (top module; file name kept generic).
module tt_um_sashwath_s_pinwheel (
    input  wire [7:0] ui_in,    // [4] HOST_CS_N [5] HOST_SCK [6] HOST_MOSI; others = protocol inputs
    output wire [7:0] uo_out,   // [7] HOST_MISO; [6:0] protocol outputs
    input  wire [7:0] uio_in,   // protocol bidirectional pins
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,   // 1 = output
    input  wire       ena,      // always 1 when powered
    input  wire       clk,
    input  wire       rst_n
);
    // 512 words = one RM_IHPSG13_1P_512x16 SRAM macro (define PW_MEM_IHP_SRAM; see
    // docs/ARCHITECTURE.md section 9). Without the define pw_mem is a flop array:
    // fine for simulation and FPGA, far too big for the 6x4 tile.
    localparam MEM_AW = 9;

    pw_top #(.MEM_AW(MEM_AW), .MBOX_AW(2)) u_pw (
        .clk(clk), .rst_n(rst_n),
        .ui_in(ui_in), .uo_out(uo_out),
        .uio_in(uio_in), .uio_out(uio_out), .uio_oe(uio_oe));

    wire _unused = &{ena, 1'b0};

endmodule
