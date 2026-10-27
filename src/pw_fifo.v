`default_nettype none
// pw_fifo.v -- small synchronous flop FIFO, depth 2**AW (AW >= 1).
// Push when full and pop when empty are ignored. dout shows the head.

module pw_fifo #(
    parameter W  = 16,
    parameter AW = 2
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         clr,
    input  wire         push,
    input  wire [W-1:0] din,
    input  wire         pop,
    output wire [W-1:0] dout,
    output wire         empty,
    output wire         full,
    output wire [AW:0]  count
);
    reg [W-1:0]  mem [0:(1<<AW)-1];
    reg [AW-1:0] wp, rp;
    reg [AW:0]   cnt;

    wire do_push = push & ~full;
    wire do_pop  = pop & ~empty;

    assign empty = (cnt == {(AW+1){1'b0}});
    assign full  = cnt[AW];
    assign count = cnt;
    assign dout  = mem[rp];

    always @(posedge clk) begin
        if (!rst_n || clr) begin
            wp  <= {AW{1'b0}};
            rp  <= {AW{1'b0}};
            cnt <= {(AW+1){1'b0}};
        end else begin
            if (do_push) wp <= wp + 1'b1;
            if (do_pop)  rp <= rp + 1'b1;
            case ({do_push, do_pop})
                2'b10:   cnt <= cnt + 1'b1;
                2'b01:   cnt <= cnt - 1'b1;
                default: ;
            endcase
        end
    end

    // storage has no reset
    always @(posedge clk)
        if (do_push) mem[wp] <= din;

endmodule
