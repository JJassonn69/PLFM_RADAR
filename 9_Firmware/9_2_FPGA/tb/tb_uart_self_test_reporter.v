`timescale 1ns / 1ps
// ============================================================================
// AERIS-10 UART Self-Test Reporter Testbench
// ============================================================================
//
// Verifies the uart_self_test_reporter module by:
//   1. Triggering a report via self_test_done pulse and capturing all 20 bytes
//   2. Verifying sync markers, packet type, flags, checksum
//   3. Triggering via UART RX 'S' command (status request)
//   4. Verifying ASCII pass/fail characters for different flag combinations
//
// Uses uart_tx instance inside reporter -> serial wire -> uart_rx to capture
// the transmitted bytes.
// ============================================================================

module tb_uart_self_test_reporter;

    parameter CLK_FREQ  = 50_000_000;
    parameter BAUD_RATE = 1_000_000;  // Fast for simulation
    parameter BAUD_DIV  = CLK_FREQ / BAUD_RATE;
    parameter PKT_LEN   = 20;

    reg        clk;
    reg        reset_n;

    // Self-test interface (simulated)
    reg        self_test_done;
    reg        self_test_busy;
    reg [4:0]  result_flags;
    reg [7:0]  result_detail;

    // UART RX command interface (simulated host sending 'S')
    reg [7:0]  host_rx_data;
    reg        host_rx_valid;

    // Heartbeat counter
    reg [31:0] heartbeat_cnt;

    // Reporter -> TX interface
    wire [7:0] rpt_tx_data;
    wire       rpt_tx_start;
    wire       tx_busy;
    wire       report_busy;

    // TX serial output
    wire       txd;
    wire       tx_done;

    // Capture RX (reads the serial output)
    wire [7:0] cap_rx_data;
    wire       cap_rx_valid;
    wire       cap_rx_error;

    // Instantiate reporter
    uart_self_test_reporter #(
        .VERSION_MAJOR(8'd0),
        .VERSION_MINOR(8'd2)
    ) u_reporter (
        .clk(clk),
        .reset_n(reset_n),
        .self_test_done(self_test_done),
        .self_test_busy(self_test_busy),
        .result_flags(result_flags),
        .result_detail(result_detail),
        .rx_data(host_rx_data),
        .rx_data_valid(host_rx_valid),
        .heartbeat_cnt(heartbeat_cnt),
        .tx_data(rpt_tx_data),
        .tx_start(rpt_tx_start),
        .tx_busy(tx_busy),
        .report_busy(report_busy)
    );

    // Instantiate UART TX (driven by reporter)
    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_tx (
        .clk(clk),
        .reset_n(reset_n),
        .tx_data(rpt_tx_data),
        .tx_start(rpt_tx_start),
        .txd(txd),
        .tx_busy(tx_busy),
        .tx_done(tx_done)
    );

    // Instantiate UART RX (captures serial output)
    uart_rx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_cap_rx (
        .clk(clk),
        .reset_n(reset_n),
        .rxd(txd),
        .rx_data(cap_rx_data),
        .rx_data_valid(cap_rx_valid),
        .rx_frame_error(cap_rx_error)
    );

    // Clock
    initial clk = 0;
    always #10 clk = ~clk;

    // Packet capture buffer
    reg [7:0] pkt [0:PKT_LEN-1];
    integer   pkt_idx;

    // Test counters
    integer pass_count;
    integer fail_count;

    // Task: collect N bytes from capture RX with timeout
    task collect_packet;
        input integer count;
        integer timeout;
        integer i;
        begin
            for (i = 0; i < count; i = i + 1) begin
                timeout = 0;
                while (!cap_rx_valid && timeout < (BAUD_DIV * 15)) begin
                    @(posedge clk);
                    timeout = timeout + 1;
                end
                if (cap_rx_valid) begin
                    pkt[i] = cap_rx_data;
                    @(posedge clk);  // consume the valid pulse
                end else begin
                    pkt[i] = 8'hXX;
                    $display("[FAIL] Timeout waiting for byte %0d", i);
                    fail_count = fail_count + 1;
                end
            end
        end
    endtask

    // Task: verify checksum
    task verify_checksum;
        integer i;
        reg [7:0] xor_sum;
        begin
            xor_sum = 8'd0;
            for (i = 0; i < PKT_LEN - 1; i = i + 1) begin
                xor_sum = xor_sum ^ pkt[i];
            end
            if (xor_sum == pkt[PKT_LEN - 1]) begin
                $display("[PASS] Checksum valid: 0x%02h", pkt[PKT_LEN-1]);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Checksum mismatch: computed=0x%02h, received=0x%02h", xor_sum, pkt[PKT_LEN-1]);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_uart_self_test_reporter.vcd");
        $dumpvars(0, tb_uart_self_test_reporter);

        pass_count     = 0;
        fail_count     = 0;
        reset_n        = 0;
        self_test_done = 0;
        self_test_busy = 0;
        result_flags   = 5'b00000;
        result_detail  = 8'h00;
        host_rx_data   = 8'h00;
        host_rx_valid  = 0;
        heartbeat_cnt  = 32'hDEADBEEF;

        // Reset
        repeat (20) @(posedge clk);
        reset_n = 1;
        repeat (10) @(posedge clk);

        // =============================================================
        // TEST SET 1: Trigger via self_test_done, all tests PASS
        // =============================================================
        $display("");
        $display("--- Test Set 1: self_test_done, all PASS ---");

        result_flags  = 5'b11111;  // All pass
        result_detail = 8'h00;
        heartbeat_cnt = 32'h12345678;

        // Pulse self_test_done
        @(posedge clk);
        self_test_done = 1;
        @(posedge clk);
        self_test_done = 0;

        // Collect 20-byte packet
        collect_packet(PKT_LEN);

        // Wait for reporter to go idle
        repeat (BAUD_DIV * 2) @(posedge clk);

        // Verify sync markers
        if (pkt[0] == 8'hA5 && pkt[1] == 8'h5A) begin
            $display("[PASS] Sync markers: 0x%02h 0x%02h", pkt[0], pkt[1]);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Sync markers: 0x%02h 0x%02h (expected A5 5A)", pkt[0], pkt[1]);
            fail_count = fail_count + 1;
        end

        // Packet type
        if (pkt[2] == 8'h01) begin
            $display("[PASS] Packet type: 0x%02h", pkt[2]);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Packet type: 0x%02h (expected 01)", pkt[2]);
            fail_count = fail_count + 1;
        end

        // Result flags
        if (pkt[3] == 8'h1F) begin
            $display("[PASS] Result flags: 0x%02h (all pass)", pkt[3]);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Result flags: 0x%02h (expected 1F)", pkt[3]);
            fail_count = fail_count + 1;
        end

        // Result detail
        if (pkt[4] == 8'h00) begin
            $display("[PASS] Result detail: 0x%02h", pkt[4]);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Result detail: 0x%02h (expected 00)", pkt[4]);
            fail_count = fail_count + 1;
        end

        // Busy flag
        if (pkt[5] == 8'h00) begin
            $display("[PASS] Busy flag: 0x%02h (idle)", pkt[5]);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Busy flag: 0x%02h (expected 00)", pkt[5]);
            fail_count = fail_count + 1;
        end

        // Version
        if (pkt[6] == 8'h00 && pkt[7] == 8'h02) begin
            $display("[PASS] Version: %0d.%0d", pkt[6], pkt[7]);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Version: %0d.%0d (expected 0.2)", pkt[6], pkt[7]);
            fail_count = fail_count + 1;
        end

        // Heartbeat counter
        if (pkt[8] == 8'h12 && pkt[9] == 8'h34 && pkt[10] == 8'h56 && pkt[11] == 8'h78) begin
            $display("[PASS] Heartbeat: 0x%02h%02h%02h%02h", pkt[8], pkt[9], pkt[10], pkt[11]);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Heartbeat: 0x%02h%02h%02h%02h (expected 12345678)", pkt[8], pkt[9], pkt[10], pkt[11]);
            fail_count = fail_count + 1;
        end

        // ASCII pass/fail characters (all pass = 'P' = 0x50)
        if (pkt[12] == 8'h50 && pkt[13] == 8'h50 && pkt[14] == 8'h50 && pkt[15] == 8'h50 && pkt[16] == 8'h50) begin
            $display("[PASS] ASCII flags: all 'P'");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] ASCII flags: %c%c%c%c%c (expected PPPPP)", pkt[12], pkt[13], pkt[14], pkt[15], pkt[16]);
            fail_count = fail_count + 1;
        end

        // CR/LF
        if (pkt[17] == 8'h0D && pkt[18] == 8'h0A) begin
            $display("[PASS] CR/LF terminator");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Terminator: 0x%02h 0x%02h (expected 0D 0A)", pkt[17], pkt[18]);
            fail_count = fail_count + 1;
        end

        // Checksum
        verify_checksum;

        // =============================================================
        // TEST SET 2: Trigger via host 'S' command, partial failure
        // =============================================================
        $display("");
        $display("--- Test Set 2: host 'S' request, partial FAIL ---");

        repeat (BAUD_DIV * 5) @(posedge clk);

        result_flags  = 5'b10110;  // BRAM fail, CIC pass, FFT pass, ARITH fail, ADC pass
        result_detail = 8'hAD;     // ADC timeout marker
        heartbeat_cnt = 32'hCAFEBABE;

        // Send 'S' command
        @(posedge clk);
        host_rx_data  = 8'h53;  // 'S'
        host_rx_valid = 1;
        @(posedge clk);
        host_rx_valid = 0;

        // Collect packet
        collect_packet(PKT_LEN);
        repeat (BAUD_DIV * 2) @(posedge clk);

        // Result flags (5'b10110 = 0x16)
        if (pkt[3] == 8'h16) begin
            $display("[PASS] Result flags: 0x%02h (partial fail)", pkt[3]);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Result flags: 0x%02h (expected 16)", pkt[3]);
            fail_count = fail_count + 1;
        end

        // Detail byte
        if (pkt[4] == 8'hAD) begin
            $display("[PASS] Detail: 0x%02h (ADC timeout)", pkt[4]);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Detail: 0x%02h (expected AD)", pkt[4]);
            fail_count = fail_count + 1;
        end

        // ASCII: BRAM=F, CIC=P, FFT=P, ARITH=F, ADC=P
        // flags[0]=0(F), flags[1]=1(P), flags[2]=1(P), flags[3]=0(F), flags[4]=1(P)
        if (pkt[12] == 8'h46 && pkt[13] == 8'h50 && pkt[14] == 8'h50 && pkt[15] == 8'h46 && pkt[16] == 8'h50) begin
            $display("[PASS] ASCII flags: FPPFP (correct partial fail)");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] ASCII flags: %c%c%c%c%c (expected FPPFP)", pkt[12], pkt[13], pkt[14], pkt[15], pkt[16]);
            fail_count = fail_count + 1;
        end

        // Heartbeat
        if (pkt[8] == 8'hCA && pkt[9] == 8'hFE && pkt[10] == 8'hBA && pkt[11] == 8'hBE) begin
            $display("[PASS] Heartbeat: 0xCAFEBABE");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] Heartbeat: 0x%02h%02h%02h%02h (expected CAFEBABE)", pkt[8], pkt[9], pkt[10], pkt[11]);
            fail_count = fail_count + 1;
        end

        // Checksum
        verify_checksum;

        // =============================================================
        // TEST SET 3: report_busy signal behavior
        // =============================================================
        $display("");
        $display("--- Test Set 3: report_busy signal ---");

        repeat (BAUD_DIV * 5) @(posedge clk);

        result_flags  = 5'b11111;
        heartbeat_cnt = 32'h00000001;

        // Trigger report
        @(posedge clk);
        self_test_done = 1;
        @(posedge clk);
        self_test_done = 0;

        // Wait a few cycles for FSM to enter LOAD/SEND
        repeat (5) @(posedge clk);
        if (report_busy == 1'b1) begin
            $display("[PASS] report_busy asserts during packet send");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] report_busy not asserted during send");
            fail_count = fail_count + 1;
        end

        // Drain the packet
        collect_packet(PKT_LEN);
        repeat (BAUD_DIV * 3) @(posedge clk);

        if (report_busy == 1'b0) begin
            $display("[PASS] report_busy clears after packet complete");
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] report_busy still asserted after complete");
            fail_count = fail_count + 1;
        end

        // =============================================================
        // SUMMARY
        // =============================================================
        repeat (100) @(posedge clk);
        $display("");
        $display("Self-Test Reporter Testbench: %0d passed, %0d failed", pass_count, fail_count);

        if (fail_count > 0)
            $display("[FAIL] Reporter testbench FAILED");
        else
            $display("[PASS] All reporter tests passed");

        $finish;
    end

    // Timeout watchdog
    initial begin
        #(100 * 1000 * 1000);  // 100 ms
        $display("[FAIL] TIMEOUT: reporter testbench did not complete");
        $finish;
    end

endmodule
