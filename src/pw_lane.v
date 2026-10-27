`default_nettype none
// pw_lane.v -- Pinwheel waveform lane.
//
// Executes a stream of commands from a 4-entry FIFO:
//   DATA word : shift NBITS bits out (and in) through a programmable per-bit
//               phase template (up to 4 phases, 8 bits each)
//   CTRL      : DRIVE pins / WAIT ticks / WAITPIN / SETN / MARK
// Every phase lasts exactly DIV+1 cycles (or ends on a CLKIN edge), so pin
// timing is cycle exact regardless of what the threads are doing.
// Register map and encodings: docs/ARCHITECTURE.md section 6.

module pw_lane (
    input  wire        clk,
    input  wire        rst_n,
    // register access (core X stage)
    input  wire        reg_we,
    input  wire [2:0]  reg_addr,
    input  wire [15:0] reg_wdata,
    output reg  [15:0] reg_rdata,
    // command FIFO
    input  wire        cmd_push,
    input  wire        cmd_ctrl,
    input  wire [15:0] cmd_data,
    output wire        cmd_full,
    output wire        drained,
    // receive FIFO
    input  wire        rx_pop,
    output wire [15:0] rx_data,
    output wire        rx_empty,
    // pin index space (see ARCHITECTURE.md section 3)
    input  wire [31:0] pin_fast,
    input  wire [31:0] pin_sync,
    // pin drive requests
    output wire        dout_drv,
    output wire [4:0]  dout_pin,
    output reg         dout_val,
    output reg         dout_oe,
    output wire        clk_drv,
    output wire [4:0]  clk_pin,
    output reg         clk_val,
    output wire        aux_drv,
    output wire [4:0]  aux_pin,
    output reg         aux_val
);
    localparam S_IDLE = 2'd0, S_BIT = 2'd1, S_WAIT = 2'd2, S_WPIN = 2'd3;

    // ---------------------------------------------------------------- config
    reg [10:0] cfg0;    // [3:0] NBITS [4] MSB [6:5] NPH-1 [7] RXEN [8] AUTO [10:9] TRIG
    reg [7:0]  cfg1;    // [0] EN [1] DOUT_EN [2] CLK_EN [3] AUX_EN [4] AUX_DIFF [7:5] idle levels
    reg [15:0] div;
    reg [31:0] tpl;
    reg [14:0] pins0;   // [4:0] DOUT [9:5] CLK [14:10] DIN
    reg [14:0] pins1;   // [4:0] TRIG [9:5] CLKIN [14:10] AUX

    wire       msb     = cfg0[4];
    wire [1:0] nph_m1  = cfg0[6:5];
    wire       rxen    = cfg0[7];
    wire       autom   = cfg0[8];
    wire [1:0] trigc   = cfg0[10:9];
    wire       en      = cfg1[0];
    wire       auxdiff = cfg1[4];

    assign dout_pin = pins0[4:0];
    assign clk_pin  = pins0[9:5];
    assign aux_pin  = pins1[14:10];
    assign dout_drv = en & cfg1[1];
    assign clk_drv  = en & cfg1[2];
    assign aux_drv  = en & cfg1[3];

    // ---------------------------------------------------------------- engine state
    reg [1:0]  state, phase;
    reg [3:0]  bitn, nbits;
    reg [15:0] data, rxw, dcnt;
    reg [12:0] wcnt;
    reg [2:0]  wcond;
    reg        trig_q, clkin_q, ovf;

    // ---------------------------------------------------------------- pins
    wire din    = pin_fast[pins0[14:10]];
    wire trig   = pin_sync[pins1[4:0]];
    wire clkin  = pin_sync[pins1[9:5]];
    wire trig_r = trig & ~trig_q;
    wire trig_f = ~trig & trig_q;
    wire ck_r   = clkin & ~clkin_q;
    wire ck_f   = ~clkin & clkin_q;

    function cond_ok;
        input [1:0] c;
        input lvl, r, f;
        begin
            case (c)
                2'd0:    cond_ok = r;
                2'd1:    cond_ok = f;
                2'd2:    cond_ok = lvl;
                default: cond_ok = ~lvl;
            endcase
        end
    endfunction

    wire trig_ok = cond_ok(trigc, trig, trig_r, trig_f);
    wire wsel    = wcond[2];
    wire wpin_ok = cond_ok(wcond[1:0], wsel ? clkin : trig,
                           wsel ? ck_r : trig_r, wsel ? ck_f : trig_f);

    // ---------------------------------------------------------------- FIFOs
    wire        flush_cmd = reg_we & (reg_addr == 3'd7) & reg_wdata[0];
    wire        flush_rx  = reg_we & (reg_addr == 3'd7) & reg_wdata[1];
    wire        cq_empty, rq_full;
    wire [16:0] cq_head;
    wire [2:0]  cq_count;
    wire [1:0]  rq_count;
    reg         cq_pop, rq_push;
    reg  [15:0] rq_din;

    pw_fifo #(.W(17), .AW(2)) u_cmdq (
        .clk(clk), .rst_n(rst_n), .clr(~en | flush_cmd),
        .push(cmd_push), .din({cmd_ctrl, cmd_data}),
        .pop(cq_pop), .dout(cq_head),
        .empty(cq_empty), .full(cmd_full), .count(cq_count));

    pw_fifo #(.W(16), .AW(1)) u_rxq (
        .clk(clk), .rst_n(rst_n), .clr(~en | flush_rx),
        .push(rq_push), .din(rq_din),
        .pop(rx_pop), .dout(rx_data),
        .empty(rx_empty), .full(rq_full), .count(rq_count));

    wire        hctrl = cq_head[16];
    wire [15:0] harg  = cq_head[15:0];
    wire [12:0] wticks = {harg[15:9], harg[5:0]};
    wire [3:0]  lastb = nbits - 4'd1;          // NBITS=0 means 16
    wire [7:0]  tp    = tpl[{phase, 3'b000} +: 8];
    wire        tick  = (dcnt == 16'd0);

    reg exit_now;
    always @* begin
        case (tp[7:6])
            2'd0:    exit_now = tick;
            2'd1:    exit_now = ck_r;
            2'd2:    exit_now = ck_f;
            default: exit_now = clkin & tick;   // wait for CLKIN high (clock stretching), then DIV+1
        endcase
    end

    // ---------------------------------------------------------------- next state
    reg [1:0]  n_state, n_phase, e_phase;
    reg [3:0]  n_bitn, n_nbits, e_bitn;
    reg [15:0] n_data, n_rxw, n_dcnt, e_data;
    reg [12:0] n_wcnt;
    reg [2:0]  n_wcond;
    reg        n_dout, n_oe, n_clk, n_aux, n_ovf;
    reg        enter, e_clear, eb;
    reg [7:0]  et;

    always @* begin
        n_state = state;  n_phase = phase;  n_bitn = bitn;  n_nbits = nbits;
        n_data  = data;   n_rxw   = rxw;    n_dcnt = dcnt;  n_wcnt  = wcnt;
        n_wcond = wcond;
        n_dout  = dout_val; n_oe = dout_oe; n_clk = clk_val; n_aux = aux_val;
        n_ovf   = ovf;
        cq_pop  = 1'b0; rq_push = 1'b0; rq_din = rxw;
        enter   = 1'b0; e_clear = 1'b0; e_phase = 2'd0; e_bitn = 4'd0; e_data = data;

        case (state)
        S_IDLE: begin
            if (!cq_empty && hctrl) begin
                cq_pop = 1'b1;
                case (harg[8:6])
                    3'd0: begin                                     // DRIVE
                        n_dout = harg[0]; n_clk = harg[1]; n_aux = harg[2]; n_oe = ~harg[3];
                        if (harg[5:4] != 2'd0) begin
                            n_state = S_WAIT;
                            n_wcnt  = {11'd0, harg[5:4]} - 13'd1;
                            n_dcnt  = div;
                        end
                    end
                    3'd1: if (wticks != 13'd0) begin                // WAIT
                        n_state = S_WAIT;
                        n_wcnt  = wticks - 13'd1;
                        n_dcnt  = div;
                    end
                    3'd2: begin n_state = S_WPIN; n_wcond = harg[2:0]; end   // WAITPIN
                    3'd3: n_nbits = harg[3:0];                      // SETN
                    3'd4: begin rq_push = 1'b1; rq_din = {10'd0, harg[5:0]}; end  // MARK
                    default: ;
                endcase
            end else if (autom ? trig_ok : !cq_empty) begin
                cq_pop  = ~cq_empty;
                enter   = 1'b1; e_clear = 1'b1;
                e_data  = cq_empty ? 16'hFFFF : harg;
                n_state = S_BIT;
            end
        end

        S_BIT: begin
            if (exit_now) begin
                if (phase != nph_m1) begin
                    enter = 1'b1; e_phase = phase + 2'd1; e_bitn = bitn;
                end else if (bitn != lastb) begin
                    enter = 1'b1; e_bitn = bitn + 4'd1;
                end else begin
                    // word complete; chain the next DATA word with no gap
                    rq_push = rxen;
                    if ((!cq_empty && !hctrl && (!autom || trig_ok)) || (autom && cq_empty && trig_ok)) begin
                        cq_pop = ~cq_empty;
                        enter  = 1'b1; e_clear = 1'b1;
                        e_data = cq_empty ? 16'hFFFF : harg;
                    end else begin
                        n_state = S_IDLE;
                    end
                end
            end else if (tp[7:6] == 2'd3 && !clkin) begin
                n_dcnt = div;                                       // stretched: hold the count
            end else if (!tick) begin
                n_dcnt = dcnt - 16'd1;
            end
        end

        S_WAIT: begin
            if (tick) begin
                if (wcnt == 13'd0) n_state = S_IDLE;
                else begin n_wcnt = wcnt - 13'd1; n_dcnt = div; end
            end else begin
                n_dcnt = dcnt - 16'd1;
            end
        end

        default: begin                                              // S_WPIN
            if (wpin_ok) n_state = S_IDLE;
        end
        endcase

        if (rq_push && rq_full) n_ovf = 1'b1;

        // phase entry: apply the template actions of the phase being entered
        et = 8'd0; eb = 1'b0;
        if (enter) begin
            n_phase = e_phase; n_bitn = e_bitn; n_data = e_data; n_dcnt = div;
            if (e_clear) n_rxw = 16'd0;
            et = tpl[{e_phase, 3'b000} +: 8];
            eb = msb ? e_data[lastb - e_bitn] : e_data[e_bitn];
            case (et[2:0])
                3'd1: begin n_dout = eb;   n_oe = 1'b1; end
                3'd2: begin n_dout = ~eb;  n_oe = 1'b1; end
                3'd3: begin n_dout = 1'b0; n_oe = 1'b1; end
                3'd4: begin n_dout = 1'b1; n_oe = 1'b1; end
                3'd5: n_oe = 1'b0;
                default: ;
            endcase
            if (auxdiff && et[2:0] != 3'd0 && et[2:0] != 3'd5) n_aux = ~n_dout;
            case (et[4:3])
                2'd1:    n_clk = 1'b0;
                2'd2:    n_clk = 1'b1;
                2'd3:    n_clk = ~clk_val;
                default: ;
            endcase
            if (et[5]) begin
                if (msb) n_rxw = {n_rxw[14:0], din};
                else     n_rxw[e_bitn] = din;
            end
        end
    end

    // ---------------------------------------------------------------- registers
    always @(posedge clk) begin
        if (!rst_n) begin
            cfg0 <= 11'd0; cfg1 <= 8'd0; div <= 16'd0; tpl <= 32'd0;
            pins0 <= 15'd0; pins1 <= 15'd0;
            state <= S_IDLE; phase <= 2'd0; bitn <= 4'd0; nbits <= 4'd0;
            data <= 16'd0; rxw <= 16'd0; dcnt <= 16'd0; wcnt <= 13'd0; wcond <= 3'd0;
            dout_val <= 1'b0; dout_oe <= 1'b0; clk_val <= 1'b0; aux_val <= 1'b0;
            trig_q <= 1'b0; clkin_q <= 1'b0; ovf <= 1'b0;
        end else begin
            trig_q  <= trig;
            clkin_q <= clkin;
            if (!en) begin
                // disabled: hold idle levels so enabling starts glitch free
                state <= S_IDLE; phase <= 2'd0; bitn <= 4'd0; dcnt <= 16'd0;
                dout_val <= cfg1[5]; clk_val <= cfg1[6]; aux_val <= cfg1[7]; dout_oe <= 1'b1;
                ovf <= 1'b0;
            end else begin
                state <= n_state; phase <= n_phase; bitn <= n_bitn; nbits <= n_nbits;
                data <= n_data; rxw <= n_rxw; dcnt <= n_dcnt; wcnt <= n_wcnt; wcond <= n_wcond;
                dout_val <= n_dout; dout_oe <= n_oe; clk_val <= n_clk; aux_val <= n_aux;
                ovf <= n_ovf;
            end
            if (reg_we) begin
                case (reg_addr)
                    3'd0: begin cfg0 <= reg_wdata[10:0]; nbits <= reg_wdata[3:0]; end
                    3'd1: begin
                        cfg1 <= reg_wdata[7:0];
                        if (!en) begin                   // enabling: start from the new idle levels
                            dout_val <= reg_wdata[5]; clk_val <= reg_wdata[6];
                            aux_val  <= reg_wdata[7]; dout_oe <= 1'b1;
                        end
                    end
                    3'd2: div <= reg_wdata;
                    3'd3: tpl[15:0] <= reg_wdata;
                    3'd4: tpl[31:16] <= reg_wdata;
                    3'd5: pins0 <= reg_wdata[14:0];
                    3'd6: pins1 <= reg_wdata[14:0];
                    default: if (reg_wdata[2]) ovf <= 1'b0;
                endcase
            end
        end
    end

    always @* begin
        case (reg_addr)
            3'd0:    reg_rdata = {5'd0, cfg0};
            3'd1:    reg_rdata = {8'd0, cfg1};
            3'd2:    reg_rdata = div;
            3'd3:    reg_rdata = tpl[15:0];
            3'd4:    reg_rdata = tpl[31:16];
            3'd5:    reg_rdata = {1'b0, pins0};
            3'd6:    reg_rdata = {1'b0, pins1};
            default: reg_rdata = {7'd0, clk_val, dout_val, ovf, (state != S_IDLE), rq_count, cq_count};
        endcase
    end

    assign drained = (state == S_IDLE) & cq_empty;

endmodule
