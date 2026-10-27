`timescale 1ns/1ps
// tb_rand.sv -- co-simulation harness: load a program, start all threads through
// the host SPI, wait for every thread to halt, dump memory.
// Memory is loaded and dumped through the host bus (the SPI bridge's request strobe
// is forced), so the same test runs on the flop array and on the IHP SRAM model and
// exercises the real memory port. Compile with PW_TRACE so the core writes
// pw_trace.txt; sw/cosim.py compares trace + memory against sw/pwsim.py.

module tb_rand;
    `include "tb_common.svh"

    // the SPI bridge's request outputs are forced from these while attached
    reg        bus_req_r = 1'b0, bus_we_r = 1'b0;
    reg [15:0] bus_addr_r = 16'h0, bus_wdata_r = 16'h0;

    task automatic bus_attach();
        force dut.u_pw.hb_we    = bus_we_r;
        force dut.u_pw.hb_addr  = bus_addr_r;
        force dut.u_pw.hb_wdata = bus_wdata_r;
        force dut.u_pw.hb_req   = bus_req_r;
    endtask

    // one host-bus access: 1-cycle request strobe, then wait for the ack
    // (everything sampled on negedges, away from the posedge NBA updates)
    task automatic bus_access(input bit we, input [15:0] addr, input [15:0] wdata, output [15:0] rdata);
        @(negedge clk);
        bus_we_r = we; bus_addr_r = addr; bus_wdata_r = wdata; bus_req_r = 1'b1;
        @(negedge clk);
        bus_req_r = 1'b0;
        while (dut.u_pw.hb_ack !== 1'b1) @(negedge clk);
        rdata = dut.u_pw.hb_rdata;
    endtask

    task automatic bus_release();
        release dut.u_pw.hb_we;
        release dut.u_pw.hb_addr;
        release dut.u_pw.hb_wdata;
        release dut.u_pw.hb_req;
    endtask

    initial begin
        string     hexf, dumpf;
        reg [15:0] v, r;
        reg [15:0] img  [0:511];
        reg [15:0] dump [0:511];
        int        n, words;
        words = 1 << dut.MEM_AW;
        if (!$value$plusargs("HEX=%s", hexf))   hexf  = "rand.hex";
        if (!$value$plusargs("DUMP=%s", dumpf)) dumpf = "rand_rtl_mem.hex";
        for (int i = 0; i < 512; i++) img[i] = 16'hxxxx;
        $readmemh(hexf, img);

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);
        n = 0;
        bus_attach();
        for (int i = 0; i < words; i++)
            if (!$isunknown(img[i])) begin bus_access(1'b1, i, img[i], r); n++; end
        bus_release();
        check(n > 0, $sformatf("loaded %0d words through the host bus", n));

        reg_write(16'h8002, 16'h000F);                     // run T0-T3
        fork : wait_halt
            wait (dut.u_pw.u_core.run == 4'h0);
            #(5ms);
        join_any
        disable wait_halt;
        repeat (20) @(posedge clk);
        reg_read(16'h8002, v);
        check(v[3:0] == 4'h0, $sformatf("all threads halted (TCTRL=%04x)", v));
        check(v[15:12] == 4'h0, "no illegal instructions");

        bus_attach();
        for (int i = 0; i < words; i++) begin
            bus_access(1'b0, i, 16'h0000, r);
            dump[i] = r;
        end
        bus_release();
        $writememh(dumpf, dump, 0, words - 1);
        $fflush();
        $display("==== tb_rand: %0d checks, %0d errors -> %s ====", checks, errors, errors ? "FAIL" : "PASS");
        $finish;
    end
endmodule
