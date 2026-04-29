`timescale 1ns / 1ps
// ============================================================================
// tb_fft_engine_axi_bridge.v — verifies the bridge's AXI tready handling
// ============================================================================
// Bug under test (AUDIT-C10): the bridge previously asserted axi_din_tvalid /
// advanced in_count / asserted tlast based on din_valid alone, ignoring the
// IP's tready handshake. With LogiCORE FFT v9.1 in nonrealtime throttle mode
// (per .xci), tready CAN deassert briefly during pipeline / BFP normalization
// events, silently dropping input samples and shifting tlast off-by-N.
//
// Fix under test: 1-deep skid buffer + AXI-correct handshake. Phase-1 handshake
// drains active beat and shifts skid up; Phase-2 loads new samples respecting
// post-handshake slot availability. Sustained 2+ cycle backpressure with active
// upstream sets overflow_sticky for visibility.
//
// This TB substitutes xfft_2048 with a stub (below) whose s_axis_data_tready
// is driven from a TB-level register, so we can deterministically inject
// backpressure patterns. The output side is tied off — tests verify only the
// S_FEED phase and reset between cases.
//
// Test cases (all 2048-pt forward FFT):
//   1. tready always 1 — baseline throughput
//   2. tready dips 1 cycle near START of frame (cycle 3)
//   3. tready dips 1 cycle MID-frame (cycle 100)
//   4. tready held low 3 cycles mid-frame — exhausts skid, asserts overflow_sticky
//
// Note on capacity: with a 1-deep skid and CONTINUOUS din_valid (no upstream
// gaps — which is how matched_filter_processing_chain feeds N cycles back-to-
// back), the bridge can absorb exactly ONE 1-cycle tready dip per frame.
// After the dip, the skid stays permanently full, sliding 1 sample behind.
// Any SECOND dip in the same frame → both slots full → overflow_sticky fires.
// This is documented in the bridge header; the overflow flag is the safety net
// for pathological IP behavior. PG109 indicates 0-1 dips per frame is typical.
//
// PASS criteria for tests 1-3:
//   - 2048 beats accepted by IP (tvalid && tready)
//   - in-order data: each beat's re=index, im=0
//   - tlast asserted on exactly the 2048th accepted beat
//   - overflow_sticky stays 0
//
// PASS criteria for test 4:
//   - overflow_sticky asserts (sample(s) lost)
// ============================================================================

module tb_fft_engine_axi_bridge;
    localparam N        = 2048;
    localparam LOG2N    = 11;
    localparam DATA_W   = 16;
    localparam CLK_PER  = 10.0;          // 100 MHz

    reg                        clk = 1'b0;
    reg                        reset_n = 1'b0;
    reg                        start = 1'b0;
    reg                        inverse = 1'b0;

    reg signed [DATA_W-1:0]    din_re = 0;
    reg signed [DATA_W-1:0]    din_im = 0;
    reg                        din_valid = 1'b0;

    wire signed [DATA_W-1:0]   dout_re;
    wire signed [DATA_W-1:0]   dout_im;
    wire                       dout_valid;
    wire                       busy;
    wire                       done;

    reg [31:0] received [0:N-1];
    reg        received_last [0:N-1];
    integer    beats_received;

    // Backpressure pattern (driven by parallel always block based on selectors)
    reg        tb_tready_value = 1'b1;
    integer    pattern_id = 0;        // 0 = always-1, 1 = every-7, 2 = single mid, 3 = sustained
    reg        pattern_active = 1'b0;
    integer    pattern_cycle = 0;

    integer    pass = 0;
    integer    fail = 0;

    integer    i;

    always #(CLK_PER/2) clk = ~clk;

    fft_engine_axi_bridge #(
        .N(N),
        .LOG2N(LOG2N),
        .DATA_W(DATA_W)
    ) u_dut (
        .clk(clk),
        .reset_n(reset_n),
        .start(start),
        .inverse(inverse),
        .din_re(din_re),
        .din_im(din_im),
        .din_valid(din_valid),
        .dout_re(dout_re),
        .dout_im(dout_im),
        .dout_valid(dout_valid),
        .busy(busy),
        .done(done)
    );

    // Capture every beat the IP accepts
    always @(posedge clk) begin
        if (reset_n && u_dut.axi_din_tvalid && u_dut.axi_din_tready) begin
            received[beats_received]      <= u_dut.axi_din_tdata;
            received_last[beats_received] <= u_dut.axi_din_tlast;
            beats_received                <= beats_received + 1;
        end
    end

    // Backpressure pattern driver (runs in parallel with main test thread)
    always @(posedge clk) begin
        if (!pattern_active) begin
            tb_tready_value <= 1'b1;
            pattern_cycle   <= 0;
        end else begin
            pattern_cycle <= pattern_cycle + 1;
            case (pattern_id)
                0: tb_tready_value <= 1'b1;
                // Pattern 1: single 1-cycle dip near start (cycle 3)
                1: tb_tready_value <= (pattern_cycle == 3) ? 1'b0 : 1'b1;
                // Pattern 2: single 1-cycle dip mid-frame (cycle 100)
                2: tb_tready_value <= (pattern_cycle == 100) ? 1'b0 : 1'b1;
                // Pattern 3: sustained 3-cycle backpressure starting cycle 50
                3: tb_tready_value <= (pattern_cycle >= 50 && pattern_cycle <= 52) ? 1'b0 : 1'b1;
                default: tb_tready_value <= 1'b1;
            endcase
        end
    end

    // ------------------------------------------------------------
    // Reset/init helper
    // ------------------------------------------------------------
    task do_reset;
        begin
            reset_n         = 1'b0;
            start           = 1'b0;
            din_valid       = 1'b0;
            din_re          = 0;
            din_im          = 0;
            pattern_active  = 1'b0;
            pattern_id      = 0;
            beats_received  = 0;
            for (i = 0; i < N; i = i + 1) begin
                received[i]      = 32'h0;
                received_last[i] = 1'b0;
            end
            @(posedge clk); @(posedge clk);
            reset_n = 1'b1;
            @(posedge clk); @(posedge clk);
        end
    endtask

    // ------------------------------------------------------------
    // Main test driver: assert start, wait for S_FEED, feed N samples,
    // wait for transition out of S_FEED (or overflow).
    // ------------------------------------------------------------
    task run_one_test;
        input integer test_id;
        input integer pat_id;
        integer       k;
        integer       timeout;
        begin
            do_reset();
            pattern_id     = pat_id;
            pattern_active = 1'b1;

            @(posedge clk); #1;
            start = 1'b1;
            @(posedge clk); #1;
            start = 1'b0;

            // Wait until bridge enters S_FEED (state = 2'd2)
            timeout = 100;
            while (u_dut.state != 2'd2 && timeout > 0) begin
                @(posedge clk);
                timeout = timeout - 1;
            end
            if (timeout == 0) begin
                $display("[FAIL] Test %0d: bridge never reached S_FEED", test_id);
                fail = fail + 1;
                pattern_active = 1'b0;
                $finish;
            end

            // Feed N samples (one per cycle)
            for (k = 0; k < N; k = k + 1) begin
                #1;
                din_re    = k[DATA_W-1:0];
                din_im    = 0;
                din_valid = 1'b1;
                @(posedge clk);
            end
            #1;
            din_valid = 1'b0;

            // Wait for bridge to leave S_FEED (or for overflow to set + grace)
            timeout = N * 4;   // 8192 cycles
            while (u_dut.state == 2'd2 && timeout > 0) begin
                @(posedge clk);
                timeout = timeout - 1;
            end
            // Extra grace period for overflow visibility
            for (k = 0; k < 20; k = k + 1) @(posedge clk);

            pattern_active = 1'b0;
        end
    endtask

    // ------------------------------------------------------------
    // Scoreboard for continuous tests
    // ------------------------------------------------------------
    task check_continuous;
        input integer test_id;
        integer       k;
        integer       errors;
        begin
            errors = 0;
            if (beats_received != N) begin
                $display("[FAIL] Test %0d: received %0d beats (expected %0d)",
                         test_id, beats_received, N);
                errors = errors + 1;
            end else begin
                for (k = 0; k < N; k = k + 1) begin
                    if (received[k][DATA_W-1:0] !== k[DATA_W-1:0]) begin
                        if (errors < 5)
                            $display("[FAIL] Test %0d: beat %0d: got re=%0d, expected %0d",
                                     test_id, k, received[k][DATA_W-1:0], k);
                        errors = errors + 1;
                    end
                    if (received[k][31:DATA_W] !== {DATA_W{1'b0}}) begin
                        if (errors < 5)
                            $display("[FAIL] Test %0d: beat %0d: im=%0d (expected 0)",
                                     test_id, k, received[k][31:DATA_W]);
                        errors = errors + 1;
                    end
                    if (k == N - 1) begin
                        if (received_last[k] !== 1'b1) begin
                            $display("[FAIL] Test %0d: beat N-1 tlast=0 (expected 1)",
                                     test_id);
                            errors = errors + 1;
                        end
                    end else begin
                        if (received_last[k] !== 1'b0) begin
                            $display("[FAIL] Test %0d: beat %0d tlast=1 (expected 0)",
                                     test_id, k);
                            errors = errors + 1;
                        end
                    end
                end
            end
            if (u_dut.overflow_sticky) begin
                $display("[FAIL] Test %0d: overflow_sticky asserted (unexpected)",
                         test_id);
                errors = errors + 1;
            end
            if (errors == 0) begin
                $display("[PASS] Test %0d: %0d beats in order, tlast on N-1, no overflow",
                         test_id, beats_received);
                pass = pass + 1;
            end else begin
                fail = fail + 1;
            end
        end
    endtask

    // ------------------------------------------------------------
    // Top-level
    // ------------------------------------------------------------
    initial begin
        $display("=========================================================");
        $display("tb_fft_engine_axi_bridge — AXI tready handshake regression");
        $display("=========================================================");

        // Test 1: tready always 1
        $display("\n[TEST 1] tready always 1 - baseline");
        run_one_test(1, 0);
        check_continuous(1);

        // Test 2: tready dips 1 cycle near start (cycle 3)
        $display("\n[TEST 2] tready dips 1 cycle at cycle 3 (early in feed)");
        run_one_test(2, 1);
        check_continuous(2);

        // Test 3: tready dips 1 cycle at cycle 100 of feed
        $display("\n[TEST 3] tready dips 1 cycle at cycle 100");
        run_one_test(3, 2);
        check_continuous(3);

        // Test 4: tready held low for 3 cycles - overflow expected
        $display("\n[TEST 4] tready held low 3 cycles - overflow expected");
        run_one_test(4, 3);
        if (u_dut.overflow_sticky) begin
            $display("[PASS] Test 4: overflow_sticky=1 (sustained backpressure detected)");
            pass = pass + 1;
        end else begin
            $display("[FAIL] Test 4: overflow_sticky NOT asserted (expected 1)");
            fail = fail + 1;
        end

        $display("\n---------------------------------------------------------");
        $display("RESULTS: %0d PASS, %0d FAIL", pass, fail);
        $display("---------------------------------------------------------");
        if (fail == 0)
            $display("[OVERALL PASS]");
        else
            $display("[OVERALL FAIL]");
        $finish;
    end

    initial begin
        #(CLK_PER * 200000);   // safety timeout
        $display("[FATAL] Global timeout");
        $finish;
    end

endmodule

// ============================================================================
// Stub xfft_2048 — replaces the production wrapper for this TB.
// ============================================================================
module xfft_2048 (
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [7:0]  s_axis_config_tdata,
    input  wire        s_axis_config_tvalid,
    output wire        s_axis_config_tready,
    input  wire [31:0] s_axis_data_tdata,
    input  wire        s_axis_data_tvalid,
    input  wire        s_axis_data_tlast,
    output wire        s_axis_data_tready,
    output wire [31:0] m_axis_data_tdata,
    output wire [7:0]  m_axis_data_tuser,
    output wire        m_axis_data_tvalid,
    output wire        m_axis_data_tlast,
    input  wire        m_axis_data_tready
);

    assign s_axis_config_tready = 1'b1;
    assign s_axis_data_tready   = tb_fft_engine_axi_bridge.tb_tready_value;

    assign m_axis_data_tdata    = 32'd0;
    assign m_axis_data_tuser    = 8'd0;
    assign m_axis_data_tvalid   = 1'b0;
    assign m_axis_data_tlast    = 1'b0;

endmodule
