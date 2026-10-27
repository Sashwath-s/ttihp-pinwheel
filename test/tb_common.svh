// tb_common.svh -- DUT instance, board wiring and host SPI BFM.
// `include inside a testbench module.

    // ------------------------------------------------------------ clock / reset
    localparam real CLK_NS = 20.0;                 // 50 MHz
    reg clk = 1'b0;
    always #(CLK_NS / 2) clk = ~clk;
    reg rst_n = 1'b0;

    // ------------------------------------------------------------ board wiring
    reg  host_cs_n = 1'b1, host_sck = 1'b0, host_mosi = 1'b0;
    reg  uart_tx   = 1'b1;                         // TB UART -> DUT UI0
    reg  spi_miso  = 1'b0;                         // TB SPI slave -> DUT UI1
    wire [7:0] ui_in = {1'b0, host_mosi, host_sck, host_cs_n, 2'b00, spi_miso, uart_tx};
    wire [7:0] uo_out, uio_out, uio_oe;

    // uio: every line has a pull-up; DUT drives when oe=1; models pull low
    tri1 [7:0] uio_bus;
    reg  [7:0] uio_model_low = 8'h00;
    genvar gi;
    generate for (gi = 0; gi < 8; gi = gi + 1) begin : g_bus
        assign uio_bus[gi] = uio_oe[gi] ? uio_out[gi] : 1'bz;
        assign uio_bus[gi] = uio_model_low[gi] ? 1'b0 : 1'bz;
    end endgenerate
    wire [7:0] uio_in = uio_bus;

    tt_um_sashwath_s_pinwheel dut (
        .ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in), .uio_out(uio_out),
        .uio_oe(uio_oe), .ena(1'b1), .clk(clk), .rst_n(rst_n));

    // ------------------------------------------------------------ checking
    int errors = 0, checks = 0;
    task automatic check(input bit cond, input string msg);
        checks++;
        if (!cond) begin
            errors++;
            $display("[%0t] FAIL: %s", $time, msg);
        end else begin
            $display("[%0t] ok:   %s", $time, msg);
        end
    endtask

    // ------------------------------------------------------------ host SPI BFM (mode 0)
    localparam real HOST_HALF = 100.0;             // SCK = 5 MHz (<= clk/8)

    task automatic spi_byte(input [7:0] tx, output [7:0] rx);
        for (int i = 7; i >= 0; i--) begin
            host_mosi = tx[i];
            #(HOST_HALF);
            host_sck = 1'b1;
            rx[i] = uo_out[7];
            #(HOST_HALF);
            host_sck = 1'b0;
        end
    endtask

    task automatic host_begin();
        host_cs_n = 1'b0;
        #(HOST_HALF);
    endtask

    task automatic host_end();
        #(HOST_HALF);
        host_cs_n = 1'b1;
        #(4 * HOST_HALF);
    endtask

    // status byte returned during CMD: {t2h_avail[3:0], attn[3:0]}
    task automatic host_status(output [7:0] st);
        reg [7:0] r;
        host_begin(); spi_byte(8'h80, st); host_end();
    endtask

    task automatic host_write(input [15:0] addr, input [15:0] w[$], input bit inc = 1);
        reg [7:0] r;
        host_begin();
        spi_byte({1'b0, inc, 6'd0}, r);
        spi_byte(addr[15:8], r);
        spi_byte(addr[7:0], r);
        foreach (w[i]) begin
            spi_byte(w[i][15:8], r);
            spi_byte(w[i][7:0], r);
        end
        host_end();
    endtask

    task automatic host_read(input [15:0] addr, input int n, output [15:0] w[$], input bit inc = 1);
        reg [7:0] r, hi, lo;
        w = {};
        host_begin();
        spi_byte({1'b1, inc, 6'd0}, r);
        spi_byte(addr[15:8], r);
        spi_byte(addr[7:0], r);
        spi_byte(n[7:0], r);
        for (int i = 0; i < n; i++) begin
            spi_byte(8'h00, hi);
            spi_byte(8'h00, lo);
            w.push_back({hi, lo});
        end
        host_end();
    endtask

    task automatic reg_write(input [15:0] addr, input [15:0] v);
        reg [15:0] q[$];
        q = {v};
        host_write(addr, q, 1'b0);
    endtask

    task automatic reg_read(input [15:0] addr, output [15:0] v);
        reg [15:0] q[$];
        host_read(addr, 1, q, 1'b0);
        v = q[0];
    endtask

    task automatic mbox_put(input int k, input [15:0] v);
        reg [15:0] st;
        do reg_read(16'h8003, st); while (st[4 + k]);          // wait while host->thread FIFO full
        reg_write(16'h8010 + k, v);
    endtask

    task automatic mbox_get(input int k, output [15:0] v);
        reg [7:0] st;
        int tries = 0;
        do begin
            host_status(st);
            if (++tries > 5000) $fatal(1, "mbox_get(%0d): timeout", k);
        end while (!st[4 + k]);
        reg_read(16'h8010 + k, v);
    endtask

    // load a $readmemh image through the host interface; returns word count
    task automatic load_program(input string fname, output int n);
        reg [15:0] img [0:511];
        reg [15:0] q[$];
        for (int i = 0; i < 512; i++) img[i] = 16'hxxxx;
        $readmemh(fname, img);
        n = 0;
        while (n < 512 && !$isunknown(img[n])) n++;
        if (n > (1 << dut.MEM_AW)) $fatal(1, "program (%0d words) larger than memory", n);
        for (int i = 0; i < n; i++) q.push_back(img[i]);
        host_write(16'h0000, q, 1'b1);
    endtask
