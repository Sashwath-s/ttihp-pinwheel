`default_nettype none
// pw_core.v -- Pinwheel 4-thread barrel processor.
//
// Pipeline F D X W (+W2). slot = thread in F; each stage holds a different
// thread, so there are no hazards and no forwarding. A blocked instruction
// simply does not advance its PC and re-executes in the thread's next slot.
// LD/ST use the issuing thread's *next* fetch slot (W2 -> F), so a memory
// access costs that thread one slot and never disturbs the other threads.
// ISA and port map: docs/ARCHITECTURE.md sections 4-5.

module pw_core #(
    parameter AW = 9                        // memory address bits (>= 7)
) (
    input  wire            clk,
    input  wire            rst_n,
    // single-port memory
    output wire            mem_en,
    output wire            mem_we,
    output wire [AW-1:0]   mem_addr,
    output wire [15:0]     mem_wdata,
    input  wire [15:0]     mem_rdata,
    // host memory access (hold hm_req until hm_ack)
    input  wire            hm_req,
    input  wire            hm_we,
    input  wire [AW-1:0]   hm_addr,
    input  wire [15:0]     hm_wdata,
    output reg             hm_ack,
    output wire [15:0]     hm_rdata,
    // host thread control (single-cycle strobes)
    input  wire [3:0]      h_run,
    input  wire [3:0]      h_halt,
    input  wire [3:0]      h_step,
    input  wire [3:0]      h_attn_clr,
    input  wire            h_pc_we,
    input  wire [1:0]      h_pc_sel,
    input  wire [AW-1:0]   h_pc_wdata,
    output wire [4*AW-1:0] pc_flat,
    output reg  [3:0]      run,
    output reg  [3:0]      blocked,
    output reg  [3:0]      attn,
    output reg  [3:0]      illegal,
    // external IO ports (lanes, mailboxes), driven from the X stage
    output wire            io_valid,
    output wire            io_dir,          // 1 = OUT
    output wire [7:0]      io_port,
    output wire [15:0]     io_wdata,
    output wire [1:0]      io_tid,
    input  wire            io_ready,
    input  wire [15:0]     io_rdata,
    output wire            io_commit,
    // pins
    input  wire [7:0]      uio_sync,
    input  wire [7:0]      ui_sync,
    input  wire [7:0]      uo_cur,
    output reg  [14:0]     gpout,
    output reg  [7:0]      gpoe,
    output reg  [7:0]      gpod,
    output wire [15:0]     time_o,
    output wire [7:0]      events_o
);
    localparam K_NONE = 2'd0, K_INSTR = 2'd1, K_LOAD = 2'd2;

    // ---------------------------------------------------------------- state
    reg [1:0]      slot;
    reg [4*AW-1:0] pcv;
    reg [3:0]      step;
    reg [15:0]     tmr;
    reg [7:0]      ev;
    reg [2:0]      hwait;
    reg [15:0]     rf [0:31];               // {tid, reg}; r0 never written

    assign pc_flat  = pcv;
    assign time_o   = tmr;
    assign events_o = ev;

    // pipeline registers
    reg [1:0]    fd_kind, fd_tid;
    reg [2:0]    fd_rd;
    reg [1:0]    dx_kind, dx_tid;
    reg [2:0]    dx_rd;
    reg [15:0]   dx_ir, dx_a, dx_b;
    reg          xw_we, xw_mem, xw_mst;
    reg [1:0]    xw_tid;
    reg [2:0]    xw_rd;
    reg [15:0]   xw_val, xw_mdata;
    reg [AW-1:0] xw_maddr;
    reg          w2_mem, w2_mst;
    reg [1:0]    w2_tid;
    reg [2:0]    w2_rd;
    reg [15:0]   w2_mdata;
    reg [AW-1:0] w2_maddr;

    // ---------------------------------------------------------------- F
    wire          f_data  = w2_mem;                     // this slot's own LD/ST
    wire          f_want  = run[slot] | step[slot];
    wire          h_grant = hm_req & ~hm_ack & ~f_data & (~f_want | hwait[2]);
    wire          f_fetch = ~f_data & ~h_grant & f_want;
    wire [AW-1:0] pc_slot = pcv[slot*AW +: AW];

    assign mem_en    = f_data | h_grant | f_fetch;
    assign mem_we    = f_data ? w2_mst : (h_grant & hm_we);
    assign mem_addr  = f_data ? w2_maddr : (h_grant ? hm_addr : pc_slot);
    assign mem_wdata = f_data ? w2_mdata : hm_wdata;
    assign hm_rdata  = mem_rdata;

    // ---------------------------------------------------------------- D
    wire [3:0]  d_op   = mem_rdata[15:12];
    wire [2:0]  d_ra   = mem_rdata[8:6];
    wire [2:0]  d_rb   = (d_op == 4'h0) ? mem_rdata[5:3] :
                         (d_op == 4'hC) ? mem_rdata[7:5] : mem_rdata[11:9];
    wire [15:0] d_aval = (d_ra == 3'd0) ? 16'd0 : rf[{fd_tid, d_ra}];
    wire [15:0] d_bval = (d_rb == 3'd0) ? 16'd0 : rf[{fd_tid, d_rb}];

    // ---------------------------------------------------------------- X decode
    wire [3:0]    op     = dx_ir[15:12];
    wire [2:0]    rd     = dx_ir[11:9];
    wire [7:0]    u8     = dx_ir[7:0];
    wire [15:0]   s6     = {{10{dx_ir[5]}}, dx_ir[5:0]};
    wire [15:0]   s9     = {{7{dx_ir[8]}}, dx_ir[8:0]};
    wire [AW-1:0] cur_pc = pcv[dx_tid*AW +: AW];
    wire [AW-1:0] pc1    = cur_pc + 1'b1;
    wire [AW-1:0] br6    = pc1 + {{(AW-6){dx_ir[5]}}, dx_ir[5:0]};
    wire [AW-1:0] br4    = pc1 + {{(AW-4){dx_ir[3]}}, dx_ir[3:0]};
    wire [AW-1:0] ea     = dx_a[AW-1:0] + {{(AW-6){1'b0}}, dx_ir[5:0]};
    wire [15:0]   tdiff  = tmr - dx_b;
    wire [7:0]    pbank  = dx_ir[8] ? ui_sync : uio_sync;

    reg [15:0] alu_r, sh_r;
    always @* begin
        case (dx_ir[2:0])
            3'd0:    alu_r = dx_a + dx_b;
            3'd1:    alu_r = dx_a - dx_b;
            3'd2:    alu_r = dx_a & dx_b;
            3'd3:    alu_r = dx_a | dx_b;
            3'd4:    alu_r = dx_a ^ dx_b;
            3'd5:    alu_r = dx_a << dx_b[3:0];
            3'd6:    alu_r = dx_a >> dx_b[3:0];
            default: alu_r = {15'd0, (dx_a < dx_b)};
        endcase
        case (dx_ir[5:4])
            2'd0:    sh_r = dx_a << dx_ir[3:0];
            2'd1:    sh_r = dx_a >> dx_ir[3:0];
            2'd2:    sh_r = $signed(dx_a) >>> dx_ir[3:0];
            default: sh_r = (dx_a >> dx_ir[3:0]) | (dx_a << (5'd16 - {1'b0, dx_ir[3:0]}));
        endcase
    end

    // IO: OUTI short port p -> LCTRLp (0x04+p) for p<4, LDATA(p-4) for p>=4
    wire        is_io    = (op == 4'hE) | (op == 4'hF);
    wire        is_outi  = (op == 4'hF);
    wire [7:0]  port     = is_outi ? {5'd0, ~dx_ir[11], dx_ir[10:9]} : dx_ir[7:0];
    wire        pdir     = is_outi | dx_ir[8];
    wire [15:0] pwdata   = is_outi ? {7'd0, dx_ir[8:0]} : dx_b;
    wire        int_port = (port >= 8'h0A) && (port <= 8'h11);

    assign io_valid  = (dx_kind == K_INSTR) & is_io & ~int_port;
    assign io_dir    = pdir;
    assign io_port   = port;
    assign io_wdata  = pwdata;
    assign io_tid    = dx_tid;
    assign io_commit = io_valid & io_ready;

    reg [15:0] int_rdata;
    always @* begin
        case (port)
            8'h0A:   int_rdata = {1'b0, gpout};
            8'h0B:   int_rdata = {8'd0, gpoe};
            8'h0C:   int_rdata = {8'd0, gpod};
            8'h0D:   int_rdata = {uo_cur, uio_sync};
            8'h0E:   int_rdata = {8'd0, ui_sync};
            8'h0F:   int_rdata = tmr;
            8'h10:   int_rdata = {14'd0, dx_tid};
            8'h11:   int_rdata = {8'd0, ev};
            default: int_rdata = 16'd0;
        endcase
    end

    // ---------------------------------------------------------------- X execute
    reg          x_blk, x_pcwe, x_we, x_mem, x_mst, x_halt, x_sig, x_ill;
    reg [AW-1:0] x_npc;
    reg [2:0]    x_rd;
    reg [15:0]   x_val;
    reg [14:0]   n_gpout;
    reg [7:0]    n_gpoe, n_gpod, n_ev;

    always @* begin
        x_blk = 1'b0; x_pcwe = 1'b0; x_we = 1'b0; x_mem = 1'b0; x_mst = 1'b0;
        x_halt = 1'b0; x_sig = 1'b0; x_ill = 1'b0;
        x_npc = pc1; x_rd = rd; x_val = 16'd0;
        n_gpout = gpout; n_gpoe = gpoe; n_gpod = gpod; n_ev = ev;

        if (dx_kind == K_LOAD) begin
            x_we = 1'b1; x_rd = dx_rd; x_val = dx_a;            // load data arrives here
        end else if (dx_kind == K_INSTR) begin
            x_pcwe = 1'b1;
            case (op)
                4'h0: begin x_we = 1'b1; x_val = alu_r; end
                4'h1: begin x_we = 1'b1; x_val = dx_a + s6; end
                4'h2: begin x_we = 1'b1; x_val = s9; end
                4'h3: begin x_we = 1'b1; x_val = dx_ir[8] ? (dx_b & {8'h00, u8}) : {u8, dx_b[7:0]}; end
                4'h4: begin x_we = 1'b1; x_val = sh_r; end
                4'h5: x_mem = 1'b1;                              // LD
                4'h6: begin x_mem = 1'b1; x_mst = 1'b1; end      // ST
                4'h7: if (dx_b == dx_a) x_npc = br6;
                4'h8: if (dx_b != dx_a) x_npc = br6;
                4'h9: if (dx_b[dx_ir[8:5]] == dx_ir[4]) x_npc = br4;
                4'hA: x_npc = dx_ir[AW-1:0];
                4'hB: begin x_npc = dx_ir[AW-1:0]; x_we = 1'b1; x_rd = 3'd7; x_val = {{(16-AW){1'b0}}, pc1}; end
                4'hC: case (dx_ir[11:8])
                        4'h0: x_halt = 1'b1;
                        4'h1: x_npc = dx_b[AW-1:0];              // JR
                        4'h2: x_blk = tdiff[15];                 // WAITT: until TIME >= ra
                        4'h3: n_ev[dx_ir[2:0]] = 1'b1;
                        4'h4: n_ev[dx_ir[2:0]] = 1'b0;
                        4'h5: if (ev[dx_ir[2:0]]) n_ev[dx_ir[2:0]] = 1'b0; else x_blk = 1'b1;
                        4'h6: x_sig = 1'b1;
                        default: begin x_halt = 1'b1; x_ill = 1'b1; x_npc = cur_pc; end
                      endcase
                4'hD: case (dx_ir[11:9])
                        3'd0: if (dx_ir[8]) n_gpout[14:8] = gpout[14:8] | u8[6:0];
                              else          n_gpout[7:0]  = gpout[7:0]  | u8;
                        3'd1: if (dx_ir[8]) n_gpout[14:8] = gpout[14:8] & ~u8[6:0];
                              else          n_gpout[7:0]  = gpout[7:0]  & ~u8;
                        3'd2: if (dx_ir[8]) n_gpout[14:8] = gpout[14:8] ^ u8[6:0];
                              else          n_gpout[7:0]  = gpout[7:0]  ^ u8;
                        3'd3: n_gpoe = gpoe | u8;
                        3'd4: n_gpoe = gpoe & ~u8;
                        3'd5: x_blk = ((pbank & u8) != u8);
                        3'd6: x_blk = ((pbank & u8) != 8'd0);
                        default: ;
                      endcase
                default: begin                                   // 0xE IN/OUT, 0xF OUTI
                    if (int_port) begin
                        if (pdir) begin
                            case (port)
                                8'h0A: n_gpout = pwdata[14:0];
                                8'h0B: n_gpoe  = pwdata[7:0];
                                8'h0C: n_gpod  = pwdata[7:0];
                                8'h11: n_ev    = pwdata[7:0];
                                default: ;
                            endcase
                        end else begin
                            x_we = 1'b1; x_val = int_rdata;
                        end
                    end else begin
                        x_blk = ~io_ready;
                        if (!pdir) begin x_we = 1'b1; x_val = io_rdata; end
                    end
                end
            endcase

            if (x_blk) begin                                     // no side effects, replay
                x_pcwe = 1'b0; x_we = 1'b0; x_mem = 1'b0; x_halt = 1'b0;
                x_sig = 1'b0; x_ill = 1'b0;
                n_gpout = gpout; n_gpoe = gpoe; n_gpod = gpod; n_ev = ev;
            end
        end
    end

    wire [3:0] tmask   = 4'b0001 << dx_tid;
    wire [3:0] x_haltm = x_halt ? tmask : 4'd0;
    wire [3:0] x_sigm  = x_sig  ? tmask : 4'd0;
    wire [3:0] x_illm  = x_ill  ? tmask : 4'd0;
    wire [3:0] f_stepm = f_fetch ? (4'b0001 << slot) : 4'd0;

    // ---------------------------------------------------------------- sequential
    always @(posedge clk) begin
        if (!rst_n) begin
            slot <= 2'd0; tmr <= 16'd0; ev <= 8'd0; hwait <= 3'd0; hm_ack <= 1'b0;
            pcv  <= {{(AW-2){1'b0}}, 2'd3, {(AW-2){1'b0}}, 2'd2,
                     {(AW-2){1'b0}}, 2'd1, {(AW-2){1'b0}}, 2'd0};     // thread k starts at k
            run <= 4'd0; step <= 4'd0; blocked <= 4'd0; attn <= 4'd0; illegal <= 4'd0;
            gpout <= 15'd0; gpoe <= 8'd0; gpod <= 8'd0;
            fd_kind <= K_NONE; fd_tid <= 2'd0; fd_rd <= 3'd0;
            dx_kind <= K_NONE; dx_tid <= 2'd0; dx_rd <= 3'd0;
            dx_ir <= 16'd0; dx_a <= 16'd0; dx_b <= 16'd0;
            xw_we <= 1'b0; xw_mem <= 1'b0; xw_mst <= 1'b0; xw_tid <= 2'd0; xw_rd <= 3'd0;
            xw_val <= 16'd0; xw_mdata <= 16'd0; xw_maddr <= {AW{1'b0}};
            w2_mem <= 1'b0; w2_mst <= 1'b0; w2_tid <= 2'd0; w2_rd <= 3'd0;
            w2_mdata <= 16'd0; w2_maddr <= {AW{1'b0}};
        end else begin
            slot <= slot + 2'd1;
            tmr  <= tmr + 16'd1;

            // host arbitration bookkeeping
            hm_ack <= h_grant;
            if (h_grant || !hm_req) hwait <= 3'd0;
            else if (!hwait[2])     hwait <= hwait + 3'd1;

            // F -> D
            fd_tid  <= slot;
            fd_rd   <= w2_rd;
            fd_kind <= f_data ? (w2_mst ? K_NONE : K_LOAD) : (f_fetch ? K_INSTR : K_NONE);

            // D -> X
            dx_kind <= fd_kind;
            dx_tid  <= fd_tid;
            dx_rd   <= fd_rd;
            dx_ir   <= mem_rdata;
            dx_a    <= (fd_kind == K_LOAD) ? mem_rdata : d_aval;
            dx_b    <= d_bval;

            // X
            if (x_pcwe) pcv[dx_tid*AW +: AW] <= x_npc;
            if (dx_kind == K_INSTR) blocked[dx_tid] <= x_blk;
            gpout <= n_gpout; gpoe <= n_gpoe; gpod <= n_gpod; ev <= n_ev;
            xw_we <= x_we; xw_tid <= dx_tid; xw_rd <= x_rd; xw_val <= x_val;
            xw_mem <= x_mem; xw_mst <= x_mst; xw_maddr <= ea; xw_mdata <= dx_b;

            // W -> W2 (RF write is in its own block below)
            w2_mem <= xw_mem; w2_mst <= xw_mst; w2_tid <= xw_tid; w2_rd <= xw_rd;
            w2_mdata <= xw_mdata; w2_maddr <= xw_maddr;

            // thread control (host wins on conflicts)
            run     <= (run & ~h_halt & ~x_haltm) | h_run;
            attn    <= (attn & ~h_attn_clr) | x_sigm;
            illegal <= (illegal & ~h_run) | x_illm;
            step    <= (step & ~f_stepm) | h_step;
            if (h_pc_we) pcv[h_pc_sel*AW +: AW] <= h_pc_wdata;
        end
    end

    // register file write (W stage)
    always @(posedge clk)
        if (xw_we && xw_rd != 3'd0) rf[{xw_tid, xw_rd}] <= xw_val;

`ifdef PW_TRACE
    // retire trace for co-simulation against sw/pwsim.py (one line per event, per thread in order)
    integer trace_fd;
    initial trace_fd = $fopen("pw_trace.txt", "w");
    always @(posedge clk) if (rst_n) begin
        if (dx_kind == K_INSTR && !x_blk) begin
            $fwrite(trace_fd, "%0d %03x %04x", dx_tid, cur_pc, dx_ir);
            if (x_we && x_rd != 3'd0) $fwrite(trace_fd, " r%0d=%04x", x_rd, x_val);
            if (x_mem && x_mst)       $fwrite(trace_fd, " m%03x=%04x", ea, dx_b);
            $fwrite(trace_fd, "\n");
        end
        if (dx_kind == K_LOAD && dx_rd != 3'd0)
            $fwrite(trace_fd, "%0d ld r%0d=%04x\n", dx_tid, dx_rd, dx_a);
    end
`endif

`ifdef SIM
    // W2 must always belong to the thread in F
    always @(posedge clk) if (rst_n && w2_mem && w2_tid != slot)
        $error("pw_core: W2 slot mismatch (w2_tid=%0d slot=%0d)", w2_tid, slot);
`endif

endmodule
