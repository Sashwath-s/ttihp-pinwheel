`default_nettype none
// pw_top.v -- Pinwheel protocol emulator: barrel core + unified memory +
// 4 waveform lanes + 4 host mailboxes + host SPI + pin mux.

module pw_top #(
    parameter MEM_AW  = 9,              // memory words = 2**MEM_AW
    parameter MBOX_AW = 2               // mailbox depth = 2**MBOX_AW (1..3)
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe
);
    localparam [7:0] MEMAW8 = MEM_AW;
    genvar g;
    integer p, k;

    // ---------------------------------------------------------------- input synchronizers
    // uo readback is registered too, so no pin read is combinational from the pin mux
    reg [7:0] uio_s1, uio_s2, ui_s1, ui_s2, uo_q;
    always @(posedge clk) begin
        uio_s1 <= uio_in; uio_s2 <= uio_s1;
        ui_s1  <= ui_in;  ui_s2  <= ui_s1;
        uo_q   <= uo_out;
    end

    // ---------------------------------------------------------------- core <-> peripherals
    wire              mem_en, mem_we;
    wire [MEM_AW-1:0] mem_addr;
    wire [15:0]       mem_wdata, mem_rdata;
    wire              hm_ack;
    wire [15:0]       hm_rdata;
    wire [4*MEM_AW-1:0] pc_flat;
    wire [3:0]        run, blocked, attn, illegal;
    wire              io_valid, io_dir, io_commit;
    wire [7:0]        io_port;
    wire [15:0]       io_wdata;
    wire [1:0]        io_tid;
    reg               io_ready;
    reg  [15:0]       io_rdata;
    wire [14:0]       gpout;
    wire [7:0]        gpoe, gpod, events;
    wire [15:0]       tmr;

    // host request (from pw_host_spi)
    wire        hb_req, hb_we, hb_miso;
    wire [15:0] hb_addr, hb_wdata;
    reg         hb_ack;
    reg  [15:0] hb_rdata;
    reg         hp, hp_we;
    reg  [15:0] hp_addr, hp_wdata;
    wire        hp_reg = hp & hp_addr[15];
    wire        hp_mem = hp & ~hp_addr[15];
    wire        rwr = hp_reg & hp_we;
    wire        rrd = hp_reg & ~hp_we;
    wire [7:0]  ra  = hp_addr[7:0];
    reg  [15:0] reg_rdata;

    wire        r_tctrl    = rwr & (ra == 8'h02);
    wire [3:0]  h_run      = r_tctrl ? hp_wdata[3:0]   : 4'd0;
    wire [3:0]  h_halt     = r_tctrl ? hp_wdata[7:4]   : 4'd0;
    wire [3:0]  h_step     = r_tctrl ? hp_wdata[11:8]  : 4'd0;
    wire [3:0]  h_attn_clr = r_tctrl ? hp_wdata[15:12] : 4'd0;
    wire        h_pc_we    = rwr & (ra[7:2] == 6'b000001);
    wire [3:0]  h2t_push   = (rwr & (ra[7:2] == 6'b000100)) ? (4'b0001 << ra[1:0]) : 4'd0;
    wire [3:0]  t2h_hpop   = (rrd & (ra[7:2] == 6'b000100)) ? (4'b0001 << ra[1:0]) : 4'd0;

    // mailboxes
    wire [3:0]  h2t_empty, h2t_full, t2h_empty, t2h_full;
    wire [15:0] h2t_dout [0:3];
    wire [15:0] t2h_dout [0:3];
    wire [MBOX_AW:0] h2t_cnt [0:3];
    wire [MBOX_AW:0] t2h_cnt [0:3];

    // lanes
    wire [3:0]  l_cmd_full, l_drained, l_rx_empty;
    wire [15:0] l_rx_data [0:3];
    wire [15:0] l_rdata   [0:3];
    wire [3:0]  l_dout_drv, l_dout_val, l_dout_oe, l_clk_drv, l_clk_val, l_aux_drv, l_aux_val;
    wire [4:0]  l_dout_pin [0:3];
    wire [4:0]  l_clk_pin  [0:3];
    wire [4:0]  l_aux_pin  [0:3];

    // ---------------------------------------------------------------- host interface
    pw_host_spi u_host (
        .clk(clk), .rst_n(rst_n),
        .sck(ui_in[5]), .cs_n(ui_in[4]), .mosi(ui_in[6]), .miso(hb_miso),
        .status({~t2h_empty, attn}),
        .bus_req(hb_req), .bus_we(hb_we), .bus_addr(hb_addr), .bus_wdata(hb_wdata),
        .bus_ack(hb_ack), .bus_rdata(hb_rdata));

    always @(posedge clk) begin
        if (!rst_n) begin
            hp <= 1'b0; hp_we <= 1'b0; hp_addr <= 16'd0; hp_wdata <= 16'd0;
            hb_ack <= 1'b0; hb_rdata <= 16'd0;
        end else begin
            hb_ack <= 1'b0;
            if (hb_req) begin
                hp <= 1'b1; hp_we <= hb_we; hp_addr <= hb_addr; hp_wdata <= hb_wdata;
            end else if (hp_reg) begin
                hp <= 1'b0; hb_ack <= 1'b1; hb_rdata <= reg_rdata;
            end else if (hp_mem && hm_ack) begin
                hp <= 1'b0; hb_ack <= 1'b1; hb_rdata <= hm_rdata;
            end
        end
    end

    always @* begin
        case (ra)
            8'h00:   reg_rdata = 16'h5057;                           // "PW"
            8'h01:   reg_rdata = {MEMAW8, 8'h00};                    // {MEM_AW, version 0}
            8'h02:   reg_rdata = {illegal, attn, blocked, run};
            8'h03:   reg_rdata = {t2h_full, h2t_empty, h2t_full, ~t2h_empty};
            8'h04:   reg_rdata = pc_flat[0*MEM_AW +: MEM_AW];
            8'h05:   reg_rdata = pc_flat[1*MEM_AW +: MEM_AW];
            8'h06:   reg_rdata = pc_flat[2*MEM_AW +: MEM_AW];
            8'h07:   reg_rdata = pc_flat[3*MEM_AW +: MEM_AW];
            8'h08:   reg_rdata = tmr;
            8'h09:   reg_rdata = {1'b0, gpout};
            8'h0A:   reg_rdata = {8'd0, gpoe};
            8'h0B:   reg_rdata = {8'd0, gpod};
            8'h0C:   reg_rdata = {uo_q, uio_s2};
            8'h0D:   reg_rdata = {8'd0, ui_s2};
            8'h0E:   reg_rdata = {8'd0, events};
            8'h10:   reg_rdata = t2h_empty[0] ? 16'd0 : t2h_dout[0];
            8'h11:   reg_rdata = t2h_empty[1] ? 16'd0 : t2h_dout[1];
            8'h12:   reg_rdata = t2h_empty[2] ? 16'd0 : t2h_dout[2];
            8'h13:   reg_rdata = t2h_empty[3] ? 16'd0 : t2h_dout[3];
            default: reg_rdata = 16'd0;
        endcase
    end

    // ---------------------------------------------------------------- core + memory
    pw_core #(.AW(MEM_AW)) u_core (
        .clk(clk), .rst_n(rst_n),
        .mem_en(mem_en), .mem_we(mem_we), .mem_addr(mem_addr),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata),
        .hm_req(hp_mem), .hm_we(hp_we), .hm_addr(hp_addr[MEM_AW-1:0]), .hm_wdata(hp_wdata),
        .hm_ack(hm_ack), .hm_rdata(hm_rdata),
        .h_run(h_run), .h_halt(h_halt), .h_step(h_step), .h_attn_clr(h_attn_clr),
        .h_pc_we(h_pc_we), .h_pc_sel(ra[1:0]), .h_pc_wdata(hp_wdata[MEM_AW-1:0]),
        .pc_flat(pc_flat), .run(run), .blocked(blocked), .attn(attn), .illegal(illegal),
        .io_valid(io_valid), .io_dir(io_dir), .io_port(io_port), .io_wdata(io_wdata),
        .io_tid(io_tid), .io_ready(io_ready), .io_rdata(io_rdata), .io_commit(io_commit),
        .uio_sync(uio_s2), .ui_sync(ui_s2), .uo_cur(uo_q),
        .gpout(gpout), .gpoe(gpoe), .gpod(gpod), .time_o(tmr), .events_o(events));

    pw_mem #(.AW(MEM_AW)) u_mem (
        .clk(clk), .en(mem_en), .we(mem_we), .addr(mem_addr),
        .wdata(mem_wdata), .rdata(mem_rdata));

    // ---------------------------------------------------------------- IO port decode
    wire [1:0] io_lane   = io_port[1:0];
    wire       is_ldata  = (io_port[7:2] == 6'd0);
    wire       is_lctrl  = (io_port[7:2] == 6'd1);
    wire       is_mbox   = (io_port == 8'h08);
    wire       is_mbstat = (io_port == 8'h09);
    wire       is_lreg   = ~io_port[7] & (io_port[6:4] >= 3'd2) & (io_port[6:4] <= 3'd5) & ~io_port[3];
    wire [1:0] lreg_lane = io_port[5:4] + 2'd2;          // 0x20->0 .. 0x50->3
    wire [3:0] mb_h2t_c  = h2t_cnt[io_tid];
    wire [3:0] mb_t2h_c  = t2h_cnt[io_tid];

    always @* begin
        io_ready = 1'b1;
        io_rdata = 16'd0;
        if (is_ldata) begin
            io_ready = io_dir ? ~l_cmd_full[io_lane] : ~l_rx_empty[io_lane];
            io_rdata = l_rx_data[io_lane];
        end else if (is_lctrl) begin
            io_ready = io_dir ? ~l_cmd_full[io_lane] : l_drained[io_lane];
            io_rdata = l_rdata[io_lane];                     // STAT (reg 7)
        end else if (is_mbox) begin
            io_ready = io_dir ? ~t2h_full[io_tid] : ~h2t_empty[io_tid];
            io_rdata = h2t_dout[io_tid];
        end else if (is_mbstat) begin
            io_rdata = {6'd0, h2t_empty[io_tid], t2h_full[io_tid], mb_t2h_c, mb_h2t_c};
        end else if (is_lreg) begin
            io_rdata = l_rdata[lreg_lane];
        end
    end

    // ---------------------------------------------------------------- mailboxes
    generate for (g = 0; g < 4; g = g + 1) begin : g_mbox
        wire mine = (io_tid == g);
        pw_fifo #(.W(16), .AW(MBOX_AW)) u_h2t (
            .clk(clk), .rst_n(rst_n), .clr(1'b0),
            .push(h2t_push[g]), .din(hp_wdata),
            .pop(io_commit & ~io_dir & is_mbox & mine), .dout(h2t_dout[g]),
            .empty(h2t_empty[g]), .full(h2t_full[g]), .count(h2t_cnt[g]));
        pw_fifo #(.W(16), .AW(MBOX_AW)) u_t2h (
            .clk(clk), .rst_n(rst_n), .clr(1'b0),
            .push(io_commit & io_dir & is_mbox & mine), .din(io_wdata),
            .pop(t2h_hpop[g]), .dout(t2h_dout[g]),
            .empty(t2h_empty[g]), .full(t2h_full[g]), .count(t2h_cnt[g]));
    end endgenerate

    // ---------------------------------------------------------------- lanes
    wire [31:0] pin_fast = {8'd0, ui_s1, uo_q, uio_s1};
    wire [31:0] pin_sync = {8'd0, ui_s2, uo_q, uio_s2};

    generate for (g = 0; g < 4; g = g + 1) begin : g_lane
        wire lsel = is_lreg & (lreg_lane == g);
        wire csel = (is_ldata | is_lctrl) & (io_lane == g);
        pw_lane u_lane (
            .clk(clk), .rst_n(rst_n),
            .reg_we(io_commit & io_dir & lsel),
            .reg_addr(lsel ? io_port[2:0] : 3'd7),
            .reg_wdata(io_wdata), .reg_rdata(l_rdata[g]),
            .cmd_push(io_commit & io_dir & csel), .cmd_ctrl(is_lctrl), .cmd_data(io_wdata),
            .cmd_full(l_cmd_full[g]), .drained(l_drained[g]),
            .rx_pop(io_commit & ~io_dir & is_ldata & (io_lane == g)),
            .rx_data(l_rx_data[g]), .rx_empty(l_rx_empty[g]),
            .pin_fast(pin_fast), .pin_sync(pin_sync),
            .dout_drv(l_dout_drv[g]), .dout_pin(l_dout_pin[g]),
            .dout_val(l_dout_val[g]), .dout_oe(l_dout_oe[g]),
            .clk_drv(l_clk_drv[g]), .clk_pin(l_clk_pin[g]), .clk_val(l_clk_val[g]),
            .aux_drv(l_aux_drv[g]), .aux_pin(l_aux_pin[g]), .aux_val(l_aux_val[g]));
    end endgenerate

    // ---------------------------------------------------------------- pin mux
    // per pin: lane 0 > lane 1 > lane 2 > lane 3 > GPIO; DOUT > CLK > AUX within a lane
    reg [14:0] pv, po;
    always @* begin
        for (p = 0; p < 15; p = p + 1) begin
            pv[p] = gpout[p];
            po[p] = (p < 8) ? gpoe[p % 8] : 1'b1;
            for (k = 3; k >= 0; k = k - 1) begin
                if (l_aux_drv[k]  && l_aux_pin[k]  == p) begin pv[p] = l_aux_val[k];  po[p] = l_dout_oe[k]; end
                if (l_clk_drv[k]  && l_clk_pin[k]  == p) begin pv[p] = l_clk_val[k];  po[p] = 1'b1;         end
                if (l_dout_drv[k] && l_dout_pin[k] == p) begin pv[p] = l_dout_val[k]; po[p] = l_dout_oe[k]; end
            end
        end
    end

    generate for (g = 0; g < 8; g = g + 1) begin : g_uio
        // open drain: never drive high, drive low when the value is 0
        assign uio_out[g] = gpod[g] ? 1'b0 : pv[g];
        assign uio_oe[g]  = gpod[g] ? (po[g] & ~pv[g]) : po[g];
    end endgenerate

    assign uo_out = {hb_miso, pv[14:8]};

    wire _unused = &{1'b0, io_valid, ui_s1[4], ui_s1[5], ui_s1[6]};

endmodule
