`timescale 1ns/1ps
// tb_demo.sv -- system test: load sw/fw/demo4.hex through the host SPI, start all
// four threads, and check UART TX/RX, SPI master and I2C master against device
// models, with UART RX traffic running concurrently with the SPI and I2C tests.

module tb_demo;
    `include "tb_common.svh"

    // ================================================================ UART (1 Mbaud)
    localparam real UART_BIT = 1000.0;
    bit        uart_mon_en = 0;
    reg  [7:0] uart_rx_q[$];
    real       uart_bit_meas[$];

    // monitor DUT TX (UO0): sample mid-bit
    initial begin
        reg [7:0] b;
        realtime t0;
        wait (uart_mon_en);
        forever begin
            @(negedge uo_out[0]);
            t0 = $realtime;
            #(UART_BIT / 2);
            if (uo_out[0] !== 1'b0) continue;
            for (int i = 0; i < 8; i++) begin
                #(UART_BIT);
                b[i] = uo_out[0];
            end
            #(UART_BIT);
            if (uo_out[0] !== 1'b1) $display("[%0t] UART monitor: framing error", $time);
            uart_rx_q.push_back(b);
        end
    end

    // measure the start-bit width of each frame whose bit 0 is 1 (start 0 -> data 1 edge)
    initial begin
        realtime tf;
        wait (uart_mon_en);
        forever begin
            @(negedge uo_out[0]);
            tf = $realtime;
            @(posedge uo_out[0]);
            if ($realtime - tf < 1.5 * UART_BIT) uart_bit_meas.push_back($realtime - tf);
        end
    end

    task automatic uart_send(input [7:0] b, input real bit_ns = UART_BIT);
        uart_tx = 1'b0; #(bit_ns);
        for (int i = 0; i < 8; i++) begin uart_tx = b[i]; #(bit_ns); end
        uart_tx = 1'b1; #(bit_ns);
    endtask

    // ================================================================ SPI slave (mode 0)
    // CS=UO3 SCK=UO1 MOSI=UO2 MISO=UI1. Responds 0xA0, 0xA1, ... ; records MOSI.
    bit        spi_en = 0;
    reg  [7:0] spi_mosi_q[$];
    reg  [7:0] spi_tx, spi_sh;
    int        spi_bit, spi_cnt = 0;
    realtime   spi_rise_t[$];

    always @(negedge uo_out[3]) if (spi_en) begin
        spi_tx = 8'hA0 + spi_cnt[7:0]; spi_cnt++;
        spi_bit = 0; spi_miso = spi_tx[7];
    end
    always @(posedge uo_out[1]) if (spi_en && uo_out[3] === 1'b0) begin
        spi_rise_t.push_back($realtime);
        spi_sh = {spi_sh[6:0], uo_out[2]};
        spi_bit++;
        if (spi_bit == 8) begin
            spi_mosi_q.push_back(spi_sh);
            spi_bit = 0;
            spi_tx = 8'hA0 + spi_cnt[7:0]; spi_cnt++;
        end
    end
    always @(negedge uo_out[1]) if (spi_en && uo_out[3] === 1'b0)
        spi_miso = spi_tx[7 - spi_bit];

    // ================================================================ I2C EEPROM model @0x50
    // SCL=UIO0 SDA=UIO1, pull-ups on the bus, stretches SCL during every ACK.
    wire scl = uio_bus[0];
    wire sda = uio_bus[1];
    reg  [7:0] eep [0:255];
    reg  [7:0] i2c_ptr, i2c_sh, i2c_tx;
    reg        i2c_active = 0, i2c_addrph, i2c_match, i2c_rw, i2c_ackph, i2c_send, i2c_ptr_set, i2c_mack;
    int        i2c_bit, i2c_starts = 0, i2c_stops = 0, i2c_stretches = 0;
    realtime   i2c_scl_rise[$];
    localparam real I2C_STRETCH = 3000.0;

    task automatic i2c_pull_sda(input bit low); uio_model_low[1] = low; endtask
    task automatic i2c_pull_scl(input bit low); uio_model_low[0] = low; endtask

    always @(negedge sda) if (scl === 1'b1) begin            // START / repeated START
        i2c_starts++;
        i2c_active = 1; i2c_addrph = 1; i2c_match = 0; i2c_ackph = 0; i2c_send = 0; i2c_bit = 0;
        i2c_pull_sda(0);
    end
    always @(posedge sda) if (scl === 1'b1 && rst_n) begin   // STOP
        i2c_stops++;
        i2c_active = 0; i2c_send = 0;
        i2c_pull_sda(0);
    end
    always @(posedge scl) begin
        i2c_scl_rise.push_back($realtime);
        if (i2c_active) begin
            if (i2c_ackph) begin
                if (i2c_send) i2c_mack = sda;
            end else begin
                if (!i2c_send) i2c_sh = {i2c_sh[6:0], sda};
                i2c_bit++;
            end
        end
    end
    always @(negedge scl) if (i2c_active) begin
        if (!i2c_ackph && i2c_bit == 8) begin                 // byte done -> ACK clock
            i2c_bit = 0; i2c_ackph = 1;
            if (!i2c_send) begin
                if (i2c_addrph) begin
                    i2c_addrph = 0;
                    i2c_match = (i2c_sh[7:1] == 7'h50);
                    i2c_rw = i2c_sh[0];
                    if (!i2c_rw) i2c_ptr_set = 0;
                end else if (i2c_match) begin
                    if (!i2c_ptr_set) begin i2c_ptr = i2c_sh; i2c_ptr_set = 1; end
                    else begin eep[i2c_ptr] = i2c_sh; i2c_ptr++; end
                end
                i2c_pull_sda(i2c_match);                     // ACK only if addressed
            end else begin
                i2c_pull_sda(0);                             // let the master ACK/NACK
            end
            if (i2c_match) begin                             // clock stretching
                i2c_stretches++;
                i2c_pull_scl(1); #(I2C_STRETCH); i2c_pull_scl(0);
            end
        end else if (i2c_ackph) begin                         // end of the ACK clock
            i2c_ackph = 0;
            i2c_pull_sda(0);
            if (i2c_send) begin
                if (i2c_mack == 1'b0) begin
                    i2c_tx = eep[i2c_ptr]; i2c_ptr++; i2c_pull_sda(~i2c_tx[7]);
                end else begin
                    i2c_send = 0;
                end
            end else if (i2c_match && i2c_rw) begin
                i2c_send = 1; i2c_tx = eep[i2c_ptr]; i2c_ptr++; i2c_pull_sda(~i2c_tx[7]);
            end
            if (!i2c_match) i2c_active = 0;                  // not us: ignore until next START
        end else if (i2c_send) begin
            i2c_pull_sda(~i2c_tx[7 - i2c_bit]);
        end
    end

    task automatic i2c_cmd(input [15:0] cmd, output [15:0] reply);
        mbox_put(3, cmd);
        mbox_get(3, reply);
    endtask

    // ================================================================ test sequence
    initial begin
        #(30ms);
        $fatal(1, "global timeout");
    end

    initial begin
        reg [15:0] v, w[$];
        reg [15:0] img [0:511];
        string     hexf;
        int        n;
        realtime   per;
        reg  [7:0] uart_bg[4] = '{8'hA3, 8'h0F, 8'h5A, 8'hC4};

        if (!$value$plusargs("HEX=%s", hexf)) hexf = "../../sw/fw/demo4.hex";
        for (int i = 0; i < 256; i++) eep[i] = 8'hFF;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // ---------------- host interface + program load
        reg_read(16'h8000, v);
        check(v == 16'h5057, $sformatf("ID register = %04x", v));
        load_program(hexf, n);
        $readmemh(hexf, img);
        host_read(16'h0000, n, w, 1'b1);
        begin
            bit same = (w.size() == n);
            for (int i = 0; i < n && same; i++) same = (w[i] == img[i]);
            check(same, $sformatf("program readback (%0d words)", n));
        end

        reg_write(16'h8002, 16'h000F);                      // run all threads
        #(20us);
        reg_read(16'h8002, v);
        check(v[3:0] == 4'hF && v[15:12] == 4'h0, $sformatf("all threads running, no illegal (TCTRL=%04x)", v));

        // ---------------- UART TX
        uart_mon_en = 1;
        mbox_put(0, "H"); mbox_put(0, "i"); mbox_put(0, 8'h55);
        fork : wait_uart
            wait (uart_rx_q.size() == 3);
            #(200us);
        join_any
        disable wait_uart;
        check(uart_rx_q.size() == 3 && uart_rx_q[0] == "H" && uart_rx_q[1] == "i" && uart_rx_q[2] == 8'h55,
              $sformatf("UART TX bytes %p", uart_rx_q));
        if (uart_bit_meas.size() > 0) begin
            per = uart_bit_meas[uart_bit_meas.size() - 1];
            check(per > 990.0 && per < 1010.0, $sformatf("UART TX bit time %0.1f ns (expect 1000)", per));
        end else check(0, "UART TX bit time measured");

        // ---------------- UART RX in the background (concurrent with SPI and I2C)
        fork
            begin
                #(5us);
                uart_send(uart_bg[0]);
                uart_send(uart_bg[1], UART_BIT * 1.03);     // +3% baud error
                #(15us);
                uart_send(uart_bg[2]);
                uart_send(uart_bg[3], UART_BIT * 0.97);     // -3% baud error
            end
        join_none

        // ---------------- SPI master
        spi_en = 1;
        mbox_put(2, 4);
        mbox_put(2, 8'h9F); mbox_put(2, 8'h01); mbox_put(2, 8'h02); mbox_put(2, 8'h03);
        for (int i = 0; i < 4; i++) begin
            mbox_get(2, v);
            check(v == 16'h00A0 + i, $sformatf("SPI MISO byte %0d = %02x", i, v));
        end
        check(spi_mosi_q.size() == 4 && spi_mosi_q[0] == 8'h9F && spi_mosi_q[3] == 8'h03,
              $sformatf("SPI MOSI bytes %p", spi_mosi_q));
        per = spi_rise_t[1] - spi_rise_t[0];
        check(per > 159.0 && per < 161.0, $sformatf("SPI SCK period %0.1f ns (expect 160)", per));
        #(2us);
        check(uo_out[3] === 1'b1, "SPI CS released after transfer");

        // ---------------- I2C master: write 2 bytes at 0x10, read them back, NACK test
        i2c_cmd(16'h0100, v); check(v == 0, "I2C START");
        i2c_cmd(16'h02A0, v); check(v == 0, "I2C addr 0x50 W ACKed (after clock stretch)");
        i2c_cmd(16'h0210, v); check(v == 0, "I2C pointer 0x10 ACKed");
        i2c_cmd(16'h02DE, v); check(v == 0, "I2C data 0xDE ACKed");
        i2c_cmd(16'h02AD, v); check(v == 0, "I2C data 0xAD ACKed");
        i2c_cmd(16'h0400, v); check(v == 0, "I2C STOP");
        check(eep[8'h10] == 8'hDE && eep[8'h11] == 8'hAD, $sformatf("EEPROM holds %02x %02x", eep[8'h10], eep[8'h11]));

        i2c_cmd(16'h0100, v);
        i2c_cmd(16'h02A0, v);
        i2c_cmd(16'h0210, v);
        i2c_cmd(16'h0100, v); check(v == 0, "I2C repeated START");
        i2c_cmd(16'h02A1, v); check(v == 0, "I2C addr 0x50 R ACKed");
        i2c_cmd(16'h0300, v); check(v == 16'h00DE, $sformatf("I2C read byte 0 = %02x (ACK)", v));
        i2c_cmd(16'h0301, v); check(v == 16'h00AD, $sformatf("I2C read byte 1 = %02x (NACK)", v));
        i2c_cmd(16'h0400, v);

        i2c_cmd(16'h0100, v);
        i2c_cmd(16'h0290, v); check(v == 1, "I2C addr 0x48 NACKed (no device)");
        i2c_cmd(16'h0400, v);
        check(i2c_starts == 4 && i2c_stops == 3, $sformatf("I2C bus saw %0d STARTs (incl. 1 repeated), %0d STOPs", i2c_starts, i2c_stops));
        check(i2c_stretches > 0, $sformatf("I2C clock stretched %0d times", i2c_stretches));

        // ---------------- UART RX results (bytes arrived while SPI/I2C ran)
        for (int i = 0; i < 4; i++) begin
            mbox_get(1, v);
            check(v == {8'h00, uart_bg[i]}, $sformatf("UART RX byte %0d = %03x", i, v));
        end

        // ---------------- final state
        reg_read(16'h8002, v);
        check(v[15:12] == 4'h0, "no illegal instructions");
        $display("");
        $display("==== tb_demo: %0d checks, %0d errors -> %s ====", checks, errors, errors ? "FAIL" : "PASS");
        $finish;
    end
endmodule
