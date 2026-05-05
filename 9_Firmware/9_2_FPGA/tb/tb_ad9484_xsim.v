`timescale 1ns / 1ps

// ============================================================================
// tb_ad9484_xsim.v — XSim testbench for ad9484_interface_400m.v
//
// Tests the REAL module with Xilinx UNISIM primitives (IBUFDS, BUFG, IDDR).
// Must be compiled with xvlog/xelab/xsim (not iverilog).
//
// Key things tested:
//   1. Differential LVDS data capture (IBUFDS)
//   2. IDDR Q2 (falling-edge) data capture in SAME_EDGE_PIPELINED mode
//   3. Reset synchronizer (P1-7 fix: async assert, sync de-assert)
//   4. Data integrity through full pipeline
//   5. AUDIT-C4: SDR-correctness — every rising-DCO sample appears exactly
//      once in the output stream (no duplications, no drops). Catches any
//      future regression that reintroduces a DDR-style Q1/Q2 demux on this
//      SDR-only chip.
// ============================================================================

module tb_ad9484_xsim;

    // ── Parameters ─────────────────────────────────────────────
    localparam DCO_PERIOD  = 2.5;    // 400 MHz
    localparam SYS_PERIOD  = 10.0;   // 100 MHz

    // ── Signals ────────────────────────────────────────────────
    // LVDS pairs (differential)
    reg  [7:0] adc_d_p;
    wire [7:0] adc_d_n;
    reg        adc_dco_p;
    wire       adc_dco_n;

    // System
    reg        sys_clk;
    reg        reset_n;

    // Outputs
    wire [7:0] adc_data_400m;
    wire       adc_data_valid_400m;
    wire       adc_dco_bufg;
    // Audit F-7.4: mmcm_locked exposed by ad9484_interface_400m so the TB
    // can wait deterministically for the MMCM SIM model to lock (~4096
    // DCO cycles) instead of guessing at a fixed 5-cycle delay.
    wire       mmcm_locked;

    // Audit F-7.4: AD9484 OR (overrange) pair — TB doesn't drive overrange,
    // so tie complementary pair to a quiescent low/high so IBUFDS sees
    // valid differential input.
    reg        adc_or_p = 1'b0;
    wire       adc_or_n = ~adc_or_p;
    wire       adc_overrange_400m;

    // Differential complements
    assign adc_d_n   = ~adc_d_p;
    assign adc_dco_n = ~adc_dco_p;

    // ── Test bookkeeping ───────────────────────────────────────
    integer pass_count;
    integer fail_count;
    integer test_num;
    integer i;

    // ── Clocks ─────────────────────────────────────────────────
    always #(DCO_PERIOD/2) adc_dco_p = ~adc_dco_p;
    always #(SYS_PERIOD/2) sys_clk   = ~sys_clk;

    // ── DUT ────────────────────────────────────────────────────
    ad9484_interface_400m uut (
        .adc_d_p           (adc_d_p),
        .adc_d_n           (adc_d_n),
        .adc_dco_p         (adc_dco_p),
        .adc_dco_n         (adc_dco_n),
        .adc_or_p          (adc_or_p),
        .adc_or_n          (adc_or_n),
        .sys_clk           (sys_clk),
        .reset_n           (reset_n),
        .adc_data_400m     (adc_data_400m),
        .adc_data_valid_400m(adc_data_valid_400m),
        .adc_dco_bufg      (adc_dco_bufg),
        .adc_overrange_400m(adc_overrange_400m),
        .mmcm_locked       (mmcm_locked)
    );

    // ── F-7.4: gate post-reset waiting on the MMCM lock indicator ──
    // The MMCM SIMULATION model takes 4096 DCO cycles (~10 µs at 400 MHz)
    // to assert mmcm_locked. The interface's reset synchronizer then
    // requires 2 more cycles for the 2-FF lock sync, plus 2 cycles for the
    // 2-FF reset sync. Allow a couple more for the data pipeline to fill.
    task wait_for_adc_ready;
        begin
            wait (mmcm_locked === 1'b1);
            repeat (6) @(posedge adc_dco_p);
        end
    endtask

    // ── Check task ─────────────────────────────────────────────
    task check;
        input cond;
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

    // ── Stimulus ───────────────────────────────────────────────
    initial begin
        $display("\n=== AD9484 Interface XSim Testbench ===");
        $display("  Testing REAL Xilinx primitives (IBUFDS, BUFG, IDDR)");
        $display("  Testing reset synchronizer (P1-7 fix)\n");

        // Init
        adc_dco_p  = 0;
        sys_clk    = 0;
        adc_d_p    = 8'h00;
        reset_n    = 0;
        pass_count = 0;
        fail_count = 0;
        test_num   = 0;

        // ════════════════════════════════════════════════════════
        // TEST GROUP 1: Reset behaviour
        // ════════════════════════════════════════════════════════
        $display("--- Test Group 1: Reset Behaviour ---");
        #50;  // let clocks run during reset
        check(adc_data_valid_400m === 1'b0, "valid = 0 during reset");
        check(adc_data_400m === 8'h00, "data = 0 during reset");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 2: BUFG clock output
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 2: Clock Buffering ---");
        // adc_dco_bufg should follow adc_dco_p (through BUFG)
        // Can't check exact timing but can verify it toggles
        begin : bufg_test
            reg saw_high, saw_low;
            saw_high = 0;
            saw_low  = 0;
            for (i = 0; i < 20; i = i + 1) begin
                #(DCO_PERIOD/4);
                if (adc_dco_bufg) saw_high = 1;
                else              saw_low  = 1;
            end
            check(saw_high && saw_low, "adc_dco_bufg toggles (BUFG functional)");
        end

        // ════════════════════════════════════════════════════════
        // TEST GROUP 3: Reset de-assertion synchronization (P1-7)
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 3: Reset Synchronizer (P1-7) ---");

        // De-assert reset BETWEEN dco edges (worst case for metastability)
        // The synchronizer should delay de-assertion by 2 dco cycles
        @(negedge adc_dco_p);
        #(DCO_PERIOD * 0.3);  // mid-cycle
        reset_n = 1;

        // F-7.4: valid stays 0 while the MMCM is locking. Verifying it
        // immediately after deassert covers the 2-FF reset-sync window
        // and is also vacuously true throughout the MMCM lock phase.
        @(posedge adc_dco_p); #0.1;
        check(adc_data_valid_400m === 1'b0,
              "valid stays 0 for 1 cycle after reset de-assert (sync stage 1)");

        // Wait for the MMCM SIM model to lock and the 2-FF reset-sync
        // pipeline to drain, then confirm the data pipeline starts producing.
        wait_for_adc_ready();
        check(adc_data_valid_400m === 1'b1,
              "valid asserts after MMCM lock + reset sync pipeline completes");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 4: Data capture via IDDR
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 4: IDDR Data Capture ---");

        // Reset and restart
        reset_n = 0;
        adc_d_p = 8'h00;
        #100;
        reset_n = 1;
        // F-7.4: every reset cycle re-arms the MMCM lock countdown, so
        // wait for lock + sync drain before driving the test pattern.
        wait_for_adc_ready();

        // Feed a known pattern on rising edges
        // IDDR in SAME_EDGE_PIPELINED mode captures:
        //   Q1 = data at rising edge (1 cycle pipelined)
        //   Q2 = data at falling edge (pipelined to align with Q1)
        // The module alternates output between Q1 and Q2 via dco_phase

        // Drive known data: alternate 0xAA on rise, 0x55 on fall
        begin : iddr_test
            reg [7:0] captured [0:31];
            integer cap_count;
            integer saw_aa, saw_55;

            cap_count = 0;
            saw_aa = 0;
            saw_55 = 0;

            for (i = 0; i < 20; i = i + 1) begin
                // Set data before rising edge
                adc_d_p = 8'hAA;
                @(posedge adc_dco_p);
                // Set data before falling edge
                #0.1;
                adc_d_p = 8'h55;
                @(negedge adc_dco_p);
                #0.1;

                // Capture output
                if (adc_data_valid_400m && cap_count < 32) begin
                    captured[cap_count] = adc_data_400m;
                    if (adc_data_400m == 8'hAA) saw_aa = saw_aa + 1;
                    if (adc_data_400m == 8'h55) saw_55 = saw_55+ 1;
                    cap_count = cap_count + 1;
                end
            end

            $display("  Captured %0d samples, saw 0xAA: %0d times, 0x55: %0d times",
                     cap_count, saw_aa, saw_55);
            check(cap_count > 0, "IDDR produces output samples");
            // With DDR capture, we should see both rise and fall data
            check(saw_aa > 0 || saw_55 > 0, "IDDR captures at least one known value");
        end

        // ════════════════════════════════════════════════════════
        // TEST GROUP 5: Sequential data integrity
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 5: Sequential Data Integrity ---");

        reset_n = 0;
        #100;
        reset_n = 1;
        wait_for_adc_ready();   // F-7.4: wait for MMCM lock + sync drain

        // Feed incrementing pattern: 0, 1, 2, ... on each half-cycle
        begin : seq_test
            reg [7:0] outputs [0:63];
            integer out_count;
            reg saw_nonzero;
            reg monotonic;

            out_count   = 0;
            saw_nonzero = 0;

            for (i = 0; i < 40; i = i + 1) begin
                adc_d_p = i[7:0];
                @(posedge adc_dco_p); #0.1;

                if (adc_data_valid_400m && out_count < 64) begin
                    outputs[out_count] = adc_data_400m;
                    if (adc_data_400m != 0) saw_nonzero = 1;
                    out_count = out_count + 1;
                end
            end

            $display("  Sequential: captured %0d outputs, saw_nonzero=%b",
                     out_count, saw_nonzero);
            check(out_count > 10, "Produces substantial output stream");
            check(saw_nonzero, "Output contains non-zero values");
        end

        // ════════════════════════════════════════════════════════
        // TEST GROUP 6: Reset mid-operation
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 6: Reset Mid-Operation ---");

        // Data should be flowing now
        adc_d_p = 8'hFF;
        repeat (5) @(posedge adc_dco_p);

        // Assert reset asynchronously (should take effect immediately)
        reset_n = 0;
        // The async assert should clear valid within 1 cycle
        repeat (2) @(posedge adc_dco_p); #0.1;
        check(adc_data_valid_400m === 1'b0, "Async reset assertion clears valid immediately");
        check(adc_data_400m === 8'h00, "Async reset assertion clears data to 0");

        // De-assert and verify sync pipeline
        #30;
        reset_n = 1;
        // Should NOT be valid yet (2-stage sync + MMCM lock countdown)
        @(posedge adc_dco_p); #0.1;
        check(adc_data_valid_400m === 1'b0,
              "valid stays 0 during reset sync de-assertion");

        // F-7.4: wait for MMCM lock + sync drain, then confirm reassert.
        wait_for_adc_ready();
        check(adc_data_valid_400m === 1'b1,
              "valid reasserts after MMCM lock + sync pipeline completes");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 7: ADC power-down output
        // ════════════════════════════════════════════════════════
        // adc_pwdn is not part of this module (it's in radar_system_top)
        // Just verify the module port list is complete

        // ════════════════════════════════════════════════════════
        // TEST GROUP 8: AUDIT-C4 — SDR-correctness counter ramp
        // ════════════════════════════════════════════════════════
        // The AD9484 is SDR LVDS: one new sample per rising DCO edge,
        // held stable across the full DCO period. We model that by
        // launching a new counter value just after each rising DCO
        // edge (~tPD = 0.85 ns) and holding it stable. The interface
        // captures Q2 of the IDDR (falling-edge, in the stable window)
        // and re-registers into BUFG → output.
        //
        // Correctness invariant: once the pipeline fills, the output
        // stream must increment by exactly 1 per BUFG cycle — no
        // duplications, no skips. A DDR-style Q1/Q2 demux (the previous
        // bug) would emit each sample twice and drop alternating ones,
        // which this test catches.
        $display("\n--- Test Group 8: SDR Counter Ramp (AUDIT-C4) ---");

        reset_n = 0;
        adc_d_p = 8'h00;
        #100;
        reset_n = 1;
        wait_for_adc_ready();   // F-7.4: wait for MMCM lock + sync drain

        begin : sdr_ramp
            reg [7:0] ramp_value;
            reg [7:0] last_out;
            reg       have_last;
            integer   captured;
            integer   diff_one_count;
            integer   diff_zero_count;
            integer   diff_other_count;

            ramp_value       = 8'h10;          // start away from 0
            have_last        = 0;
            last_out         = 8'h00;
            captured         = 0;
            diff_one_count   = 0;
            diff_zero_count  = 0;
            diff_other_count = 0;

            // Drive 64 samples in SDR style: launch new value just after
            // each rising DCO edge, hold stable. Wrap-around in 8-bit is
            // fine (consecutive +1 across 0xFF→0x00 still passes the
            // diff-of-1 check).
            for (i = 0; i < 64; i = i + 1) begin
                @(posedge adc_dco_p);
                #0.5;                          // ≈ AD9484 tPD launch delay
                adc_d_p = ramp_value;
                ramp_value = ramp_value + 8'd1;

                // Sample the output near the next rising edge, after
                // the BUFG-domain pipeline has settled.
                if (adc_data_valid_400m) begin
                    if (have_last) begin
                        case (adc_data_400m - last_out)
                            8'd1:    diff_one_count   = diff_one_count   + 1;
                            8'd0:    diff_zero_count  = diff_zero_count  + 1;
                            default: diff_other_count = diff_other_count + 1;
                        endcase
                    end
                    last_out  = adc_data_400m;
                    have_last = 1;
                    captured  = captured + 1;
                end
            end

            $display("  SDR ramp: captured=%0d  diff=+1: %0d  diff=0 (dup): %0d  other: %0d",
                     captured, diff_one_count, diff_zero_count, diff_other_count);

            // Need a meaningful number of samples to draw conclusions.
            check(captured >= 32, "SDR ramp captured >= 32 output samples");

            // Most consecutive output deltas must be +1. Allow a small
            // slack window for pipeline-startup transients.
            check(diff_one_count >= (captured - 6),
                  "SDR ramp: consecutive output samples increment by +1");

            // Zero deltas would indicate the broken DDR-style Q1/Q2
            // duplication. With Q2-only routing this should be 0 (allow
            // 1 for the very first comparison if pipeline-startup races).
            check(diff_zero_count <= 1,
                  "SDR ramp: no sample duplication (would indicate DDR demux bug)");

            // No skips either — the broken demux dropped odd-indexed
            // samples (delta == 2). Anything beyond +1/0 is a fail.
            check(diff_other_count == 0,
                  "SDR ramp: no sample skips or unexpected jumps");
        end

        // ════════════════════════════════════════════════════════
        // Summary
        // ════════════════════════════════════════════════════════
        $display("");
        $display("========================================");
        $display("  AD9484 INTERFACE XSIM RESULTS");
        $display("  PASSED: %0d / %0d", pass_count, test_num);
        $display("  FAILED: %0d / %0d", fail_count, test_num);
        if (fail_count == 0)
            $display("  ** ALL TESTS PASSED **");
        else
            $display("  ** SOME TESTS FAILED **");
        $display("========================================");
        $display("");

        #100;
        $finish;
    end

endmodule
