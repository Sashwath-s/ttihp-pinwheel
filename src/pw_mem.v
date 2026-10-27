`default_nettype none
// pw_mem.v -- unified instruction/data memory, 16-bit words, single port,
// synchronous read (data valid the cycle after the address, held while en=0).
//
// PW_MEM_IHP_SRAM: one IHP RM_IHPSG13_1P_512x16_c2_bm_bist macro (AW must be 9),
// functional port only, BIST port tied off. Otherwise a behavioural array that
// synthesizes to flops (simulation / FPGA; far too big for the 6x4 tile).

module pw_mem #(
    parameter AW = 9
) (
    input  wire          clk,
    input  wire          en,
    input  wire          we,
    input  wire [AW-1:0] addr,
    input  wire [15:0]   wdata,
`ifdef PW_MEM_IHP_SRAM
    output wire [15:0]   rdata
`else
    output reg  [15:0]   rdata
`endif
);
`ifdef PW_MEM_IHP_SRAM
    generate if (AW != 9) begin : g_bad_aw
        initial $error("pw_mem: the IHP 512x16 macro needs AW = 9 (got %0d)", AW);
    end endgenerate

    RM_IHPSG13_1P_512x16_c2_bm_bist u_sram (
        .A_CLK      (clk),
        .A_MEN      (en),
        .A_WEN      (en & we),
        .A_REN      (en & ~we),
        .A_ADDR     (addr[8:0]),
        .A_DIN      (wdata),
        .A_DLY      (1'b1),
        .A_DOUT     (rdata),
        .A_BM       (16'hFFFF),
        .A_BIST_CLK (1'b0),
        .A_BIST_EN  (1'b0),
        .A_BIST_MEN (1'b0),
        .A_BIST_WEN (1'b0),
        .A_BIST_REN (1'b0),
        .A_BIST_ADDR(9'd0),
        .A_BIST_DIN (16'd0),
        .A_BIST_BM  (16'd0)
    );
`else
    reg [15:0] m [0:(1<<AW)-1];

    always @(posedge clk) begin
        if (en) begin
            if (we) m[addr] <= wdata;
            else    rdata   <= m[addr];
        end
    end
`endif

endmodule
