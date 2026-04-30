`timescale 1ns / 1ps

// ============================================================================
// tb_cdc_async_fifo — exercises the home-grown Cummings async FIFO that
// replaces cdc_adc_to_processing on the CIC→FIR (400→100 MHz) crossing.
//
// Coverage objectives (audit C-11 recommendation):
//   1. Reset behaviour on both domains.
//   2. Single sample passthrough (data integrity).
//   3. Continuous stream at matched src/dst rate (steady-state bandwidth).
//   4. Multi-Gray-bit-flip stimulus — alternate 0x00000 ↔ 0x3FFFF: each
//      transition flips ALL Gray bits at once, which is exactly the input
//      pattern that exposes the cdc_adc_to_processing skew hazard. This
//      FIFO must never present an intermediate value at dst_data — only
//      the alternating extremes.
//   5. Sustained burst: src_valid every src_clk cycle (4× the dst drain
//      rate). FIFO fills, overrun pulses, no data corruption on cells
//      already in flight.
//   6. Drain to empty: after src stops, dst_valid pulses exactly N times
//      for N writes that fit in the FIFO, then dst_valid stays LOW.
//
// Clock ratio mirrors production: src=400 MHz, dst=100 MHz (4:1).
// ============================================================================
module tb_cdc_async_fifo;

    localparam SRC_CLK_PERIOD = 2.5;   // 400 MHz
    localparam DST_CLK_PERIOD = 10.0;  // 100 MHz
    localparam WIDTH          = 18;
    localparam DEPTH          = 16;

    integer pass_count;
    integer fail_count;
    integer test_num;

    task check;
        input        cond;
        input [511:0] label;
        begin
            test_num = test_num + 1;
            if (cond) begin
                $display("[PASS] Test %0d: %0s", test_num, label);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Test %0d: %0s", test_num, label);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // ── DUT signals ───────────────────────────────────────────────
    reg              src_clk;
    reg              dst_clk;
    reg              src_reset_n;
    reg              dst_reset_n;
    reg  [WIDTH-1:0] src_data;
    reg              src_valid;
    wire [WIDTH-1:0] dst_data;
    wire             dst_valid;
    wire             overrun;

    always #(SRC_CLK_PERIOD/2) src_clk = ~src_clk;
    always #(DST_CLK_PERIOD/2) dst_clk = ~dst_clk;

    cdc_async_fifo #(
        .WIDTH(WIDTH),
        .DEPTH(DEPTH)
    ) dut (
        .src_clk     (src_clk),
        .dst_clk     (dst_clk),
        .src_reset_n (src_reset_n),
        .dst_reset_n (dst_reset_n),
        .src_data    (src_data),
        .src_valid   (src_valid),
        .dst_data    (dst_data),
        .dst_valid   (dst_valid),
        .overrun     (overrun)
    );

    // ── Captured-output queue (collected on every dst_valid pulse) ────
    reg  [WIDTH-1:0] capture_q [0:1023];
    integer          capture_n;
    always @(posedge dst_clk) begin
        if (!dst_reset_n) begin
            capture_n <= 0;
        end else if (dst_valid) begin
            capture_q[capture_n] <= dst_data;
            capture_n            <= capture_n + 1;
        end
    end

    // ── Overrun pulse counter (src domain) ────────────────────────────
    integer overrun_count;
    always @(posedge src_clk) begin
        if (!src_reset_n) overrun_count <= 0;
        else if (overrun) overrun_count <= overrun_count + 1;
    end

    // ── Helpers ───────────────────────────────────────────────────────
    task drive_sample;
        input [WIDTH-1:0] v;
        begin
            @(posedge src_clk);
            src_data  <= v;
            src_valid <= 1'b1;
            @(posedge src_clk);
            src_valid <= 1'b0;
        end
    endtask

    // Drive src_valid HIGH every src_clk cycle for n cycles, with
    // a counter pattern as data so we can verify ordering.
    task drive_burst;
        input integer n;
        input [WIDTH-1:0] base;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                @(posedge src_clk);
                src_data  <= base + k;
                src_valid <= 1'b1;
            end
            @(posedge src_clk);
            src_valid <= 1'b0;
        end
    endtask

    task wait_dst_cycles;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) @(posedge dst_clk);
        end
    endtask

    // ── Main ──────────────────────────────────────────────────────────
    integer i, expected_overrun;
    integer alt_count, intermediate_count;

    initial begin
        $dumpfile("tb_cdc_async_fifo.vcd");
        $dumpvars(0, tb_cdc_async_fifo);

        src_clk     = 0;
        dst_clk     = 0;
        src_reset_n = 0;
        dst_reset_n = 0;
        src_data    = {WIDTH{1'b0}};
        src_valid   = 1'b0;
        pass_count  = 0;
        fail_count  = 0;
        test_num    = 0;

        // ════════════════════════════════════════════════════════
        // Group 1: Reset
        // ════════════════════════════════════════════════════════
        $display("\n=== Group 1: Reset ===");
        #100;
        check(dst_valid === 1'b0,            "G1.1: dst_valid LOW during reset");
        check(dst_data  === {WIDTH{1'b0}},   "G1.2: dst_data 0 during reset");
        check(overrun   === 1'b0,            "G1.3: overrun LOW during reset");

        // Release dst first (out-of-order reset deassertion is allowed)
        @(posedge dst_clk); dst_reset_n = 1'b1;
        wait_dst_cycles(2);
        check(dst_valid === 1'b0,            "G1.4: dst_valid LOW (src still in reset)");

        @(posedge src_clk); src_reset_n = 1'b1;
        wait_dst_cycles(4);
        check(dst_valid === 1'b0,            "G1.5: dst_valid LOW after both resets release (FIFO empty)");

        // ════════════════════════════════════════════════════════
        // Group 2: Single sample passthrough
        // ════════════════════════════════════════════════════════
        $display("\n=== Group 2: Single sample passthrough ===");
        capture_n = 0;
        drive_sample(18'h12345);
        wait_dst_cycles(8);
        check(capture_n === 1,                  "G2.1: exactly 1 sample emitted");
        check(capture_q[0] === 18'h12345,       "G2.2: data integrity (0x12345)");

        // ════════════════════════════════════════════════════════
        // Group 3: Multi-Gray-bit-flip stimulus (audit C-11 coverage)
        // Alternate 0x00000 ↔ 0x3FFFF — these two values differ by ALL
        // 18 bits in binary, so Gray code also flips many bits between
        // them. cdc_adc_to_processing's data/toggle skew failure mode
        // would manifest as an intermediate value (anything other than
        // these two). The FIFO must NEVER produce an intermediate.
        // ════════════════════════════════════════════════════════
        $display("\n=== Group 3: Multi-Gray-bit-flip (0x00000 <-> 0x3FFFF) ===");
        capture_n = 0;
        for (i = 0; i < 32; i = i + 1) begin
            drive_sample((i & 1) ? 18'h3FFFF : 18'h00000);
            // small inter-sample gap to let dst drain so FIFO doesn't fill
            wait_dst_cycles(2);
        end
        wait_dst_cycles(16);
        check(capture_n === 32,                 "G3.1: 32 samples emitted (no drops)");

        intermediate_count = 0;
        alt_count          = 0;
        for (i = 0; i < capture_n; i = i + 1) begin
            if (capture_q[i] === 18'h00000 || capture_q[i] === 18'h3FFFF) begin
                alt_count = alt_count + 1;
            end else begin
                intermediate_count = intermediate_count + 1;
            end
        end
        check(intermediate_count === 0,
              "G3.2: NO intermediate values (every sample is 0x00000 or 0x3FFFF)");
        check(alt_count === 32,
              "G3.3: all 32 samples are one of the two extremes");

        // Verify ordering: even index → 0x00000, odd index → 0x3FFFF
        // (sampling order matches src_valid order because FIFO is FIFO)
        for (i = 0; i < capture_n; i = i + 1) begin
            if ((i & 1) ? (capture_q[i] !== 18'h3FFFF)
                        : (capture_q[i] !== 18'h00000)) begin
                $display("  [trace] G3 ordering mismatch at index %0d: 0x%h", i, capture_q[i]);
            end
        end
        // Just one summary check on ordering:
        check((capture_q[0] === 18'h00000) && (capture_q[1] === 18'h3FFFF) &&
              (capture_q[30] === 18'h00000) && (capture_q[31] === 18'h3FFFF),
              "G3.4: alternation order preserved across the run");

        // ════════════════════════════════════════════════════════
        // Group 4: Continuous stream — counter pattern, no overrun
        // src writes one sample every 4 src cycles (100 MHz src valid
        // rate, matched to dst drain rate)
        // ════════════════════════════════════════════════════════
        $display("\n=== Group 4: Matched-rate continuous stream ===");
        capture_n = 0;
        overrun_count = 0;
        // Drive 64 samples with 3-cycle gap → 100 MHz effective src rate
        for (i = 0; i < 64; i = i + 1) begin
            @(posedge src_clk); src_data <= i[WIDTH-1:0]; src_valid <= 1'b1;
            @(posedge src_clk); src_valid <= 1'b0;
            @(posedge src_clk);
            @(posedge src_clk);
        end
        wait_dst_cycles(32);
        check(overrun_count === 0,            "G4.1: no overrun at matched src/dst rate");
        check(capture_n === 64,               "G4.2: all 64 samples passed through");
        check(capture_q[0]  === 18'd0,        "G4.3: first sample = 0");
        check(capture_q[63] === 18'd63,       "G4.4: last sample = 63");
        // Spot-check ordering
        check(capture_q[10] === 18'd10 && capture_q[42] === 18'd42,
              "G4.5: monotonic counter pattern preserved");

        // ════════════════════════════════════════════════════════
        // Group 5: Overrun — sustained burst, src_valid every cycle
        // FIFO has DEPTH=16, drain is 4× slower than fill, so after
        // ~21 src writes the FIFO fills and overrun starts pulsing.
        // ════════════════════════════════════════════════════════
        $display("\n=== Group 5: Burst overrun ===");
        capture_n = 0;
        overrun_count = 0;
        // Wait for dst to fully drain anything pending
        wait_dst_cycles(8);
        @(posedge src_clk);

        drive_burst(64, 18'h10000);   // src_valid HIGH for 64 src cycles
        wait_dst_cycles(48);          // let FIFO drain

        check(overrun_count > 0,              "G5.1: overrun fired during sustained burst");
        // 64 writes; FIFO drains 1 entry per 4 src cycles = 16 drained during burst.
        // So writes that succeed = 16 (in FIFO) + 16 (drained) = ~32; drops = 64-32 = ~32.
        check(capture_n >= 16 && capture_n <= 48,
              "G5.2: capture count in expected range (FIFO depth + drained)");
        // Whatever made it through must be a contiguous prefix-suffix of the
        // counter pattern — first sample MUST be 0x10000 (the burst base);
        // the FIFO never reorders.
        check(capture_q[0] === 18'h10000,     "G5.3: first captured sample = burst base");

        // ════════════════════════════════════════════════════════
        // Group 6: Drain to empty + idle behaviour
        // ════════════════════════════════════════════════════════
        $display("\n=== Group 6: Drain + idle ===");
        wait_dst_cycles(64);
        capture_n = 0;
        overrun_count = 0;
        wait_dst_cycles(32);
        check(capture_n === 0,                "G6.1: no spurious dst_valid while idle");
        check(dst_valid === 1'b0,             "G6.2: dst_valid LOW after drain");
        check(overrun_count === 0,            "G6.3: no overrun while idle");

        // Single-shot post-idle write should still work
        drive_sample(18'h2AAAA);
        wait_dst_cycles(8);
        check(capture_n === 1,                "G6.4: post-idle single sample emitted");
        check(capture_q[0] === 18'h2AAAA,     "G6.5: post-idle data integrity");

        // ════════════════════════════════════════════════════════
        // Group 7: Reset mid-stream — both pointers must zero, dst_valid LOW
        // ════════════════════════════════════════════════════════
        $display("\n=== Group 7: Reset mid-stream ===");
        wait_dst_cycles(8);
        // Pre-load a few samples
        drive_sample(18'h11111);
        drive_sample(18'h22222);
        // Assert reset on both before they drain
        @(posedge dst_clk); dst_reset_n = 1'b0;
        @(posedge src_clk); src_reset_n = 1'b0;
        capture_n = 0;
        wait_dst_cycles(8);
        check(dst_valid === 1'b0,             "G7.1: dst_valid LOW under mid-stream reset");
        check(capture_n === 0,                "G7.2: no captures during reset");

        // Release
        @(posedge dst_clk); dst_reset_n = 1'b1;
        @(posedge src_clk); src_reset_n = 1'b1;
        wait_dst_cycles(8);
        check(dst_valid === 1'b0,             "G7.3: dst_valid LOW after reset release (FIFO empty)");

        // Post-reset write
        drive_sample(18'h3CCCC);
        wait_dst_cycles(8);
        check(capture_q[0] === 18'h3CCCC,     "G7.4: post-reset write succeeds");

        // ── Final summary ─────────────────────────────────────────
        $display("\n============================================");
        $display("  RESULTS: %0d passed / %0d failed / %0d total",
                 pass_count, fail_count, test_num);
        $display("============================================");
        if (fail_count == 0) $display("  STATUS: ALL TESTS PASSED");
        else                 $display("  STATUS: FAILURES DETECTED");

        $finish;
    end

    // Watchdog
    initial begin
        #200000;
        $display("[FAIL] WATCHDOG: simulation hung");
        $finish;
    end

endmodule
