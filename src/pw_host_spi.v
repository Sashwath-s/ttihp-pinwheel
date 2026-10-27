`default_nettype none
// pw_host_spi.v -- host interface: SPI mode 0 slave (oversampled, SCK <= clk/8)
// bridged to a 16-bit word bus.
//
//   write: CMD(0b0i......) ADDR_H ADDR_L  D0_H D0_L  D1_H D1_L ...  (until CS_N rises)
//   read : CMD(0b1i......) ADDR_H ADDR_L  N    D0_H D0_L ...         (N words, 0 = 256)
//   i = auto-increment. MISO carries `status` during CMD. Reads are issued
//   exactly N times (one word of prefetch), so FIFO reads never over-pop.
// bus_req is a one-cycle strobe; read data is captured on bus_ack.

module pw_host_spi (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sck,
    input  wire        cs_n,
    input  wire        mosi,
    output reg         miso,
    input  wire [7:0]  status,
    output reg         bus_req,
    output reg         bus_we,
    output reg  [15:0] bus_addr,
    output reg  [15:0] bus_wdata,
    input  wire        bus_ack,
    input  wire [15:0] bus_rdata
);
    localparam ST_CMD = 3'd0, ST_AH = 3'd1, ST_AL = 3'd2, ST_N  = 3'd3,
               ST_RH  = 3'd4, ST_RL = 3'd5, ST_WH = 3'd6, ST_WL = 3'd7;

    reg [2:0] sck_s, cs_s;
    reg [1:0] mosi_s;
    wire sck_r  = sck_s[1] & ~sck_s[2];
    wire sck_f  = ~sck_s[1] & sck_s[2];
    wire cs_on  = ~cs_s[1];
    wire cs_beg = ~cs_s[1] & cs_s[2];

    reg [2:0]  st, bitc;
    reg [6:0]  sin;
    reg [7:0]  tx, ahi, whi;
    reg        rd, inc;
    reg [15:0] addr, outw, pf;
    reg [8:0]  rem;                     // words still to send after the current one

    wire [7:0]  byte_in   = {sin, mosi_s[1]};
    wire        byte_done = cs_on & sck_r & (bitc == 3'd7);
    wire [15:0] addr_inc  = addr + {15'd0, inc};

    always @(posedge clk) begin
        if (!rst_n) begin
            sck_s <= 3'b000; cs_s <= 3'b111; mosi_s <= 2'b00;
            st <= ST_CMD; bitc <= 3'd0; sin <= 7'd0; tx <= 8'd0; miso <= 1'b0;
            ahi <= 8'd0; whi <= 8'd0; rd <= 1'b0; inc <= 1'b0;
            addr <= 16'd0; outw <= 16'd0; pf <= 16'd0; rem <= 9'd0;
            bus_req <= 1'b0; bus_we <= 1'b0; bus_addr <= 16'd0; bus_wdata <= 16'd0;
        end else begin
            sck_s  <= {sck_s[1:0], sck};
            cs_s   <= {cs_s[1:0], cs_n};
            mosi_s <= {mosi_s[0], mosi};
            bus_req <= 1'b0;
            if (bus_ack && !bus_we) pf <= bus_rdata;

            if (!cs_on) begin
                st <= ST_CMD; bitc <= 3'd0;
            end else begin
                if (cs_beg) begin
                    tx <= status; miso <= status[7]; bitc <= 3'd0; st <= ST_CMD;
                end
                if (sck_r) begin
                    sin  <= byte_in[6:0];
                    bitc <= bitc + 3'd1;
                end
                if (sck_f) miso <= tx[~bitc];          // next bit; MSB of a new byte when bitc==0

                if (byte_done) begin
                    case (st)
                        ST_CMD: begin rd <= byte_in[7]; inc <= byte_in[6]; tx <= 8'h00; st <= ST_AH; end
                        ST_AH:  begin ahi <= byte_in; st <= ST_AL; end
                        ST_AL:  begin
                            if (rd) begin
                                bus_req <= 1'b1; bus_we <= 1'b0; bus_addr <= {ahi, byte_in};
                                addr <= {ahi, byte_in} + {15'd0, inc};
                                st <= ST_N;
                            end else begin
                                addr <= {ahi, byte_in};
                                st <= ST_WH;
                            end
                        end
                        ST_N: begin
                            rem  <= ((byte_in == 8'd0) ? 9'd256 : {1'b0, byte_in}) - 9'd1;
                            outw <= pf; tx <= pf[15:8];
                            if (byte_in != 8'd1) begin
                                bus_req <= 1'b1; bus_we <= 1'b0; bus_addr <= addr; addr <= addr_inc;
                            end
                            st <= ST_RH;
                        end
                        ST_RH: begin tx <= outw[7:0]; st <= ST_RL; end
                        ST_RL: begin
                            if (rem != 9'd0) begin
                                outw <= pf; tx <= pf[15:8]; rem <= rem - 9'd1;
                                if (rem != 9'd1) begin
                                    bus_req <= 1'b1; bus_we <= 1'b0; bus_addr <= addr; addr <= addr_inc;
                                end
                                st <= ST_RH;
                            end else begin
                                tx <= 8'hFF;                   // read past N: 0xFF
                            end
                        end
                        ST_WH: begin whi <= byte_in; st <= ST_WL; end
                        default: begin                         // ST_WL
                            bus_req <= 1'b1; bus_we <= 1'b1; bus_addr <= addr; bus_wdata <= {whi, byte_in};
                            addr <= addr_inc;
                            st <= ST_WH;
                        end
                    endcase
                end
            end
        end
    end

endmodule
