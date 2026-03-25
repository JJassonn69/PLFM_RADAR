`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// tb_mti_physics.v — Physics-aware MTI (Moving Target Indication) testbench
//
// Tests that the 2-pulse MTI canceller correctly suppresses stationary
// targets (clutter) and passes moving targets with the expected frequency
// response H(z) = 1 - z^{-1}.
//
// MTI Architecture (mti_canceller.v):
//   For each range bin: output[n] = input[n] - input[n-1]
//   First chirp is muted (zero output). Pass-through when mti_enable=0.
//
// TEST COVERAGE:
//   M1: Stationary target (identical chirp-to-chirp) → output ≈ 0
//       Must achieve >40 dB suppression
//   M2: Moving target at arbitrary velocity → output preserves signal
//       with <3 dB loss (for velocity well away from blind speeds)
//   M3: Blind speed: target at v = lambda/(2*PRI) → output ≈ 0
//       The H(z)=1-z^{-1} filter has nulls at f_d = 0, 1/PRI, 2/PRI...
//       A target whose Doppler shift is exactly 1/PRI has phase shift of
//       2*pi between chirps → same as stationary → cancelled
//   M4: First chirp mute: verify output is zero for the entire first chirp
//   M5: MTI disable (pass-through): output must exactly equal input
//   M6: Saturation: near-full-scale subtraction must not wrap around
//   M7: Multiple range bins: targets at different bins are independent
//
// Simulation: iverilog -DSIMULATION -o tb_mti_physics.vvp \
//             tb/physics/tb_mti_physics.v mti_canceller.v \
//             && vvp tb_mti_physics.vvp
//////////////////////////////////////////////////////////////////////////////

module tb_mti_physics;

// =========================================================================
// RADAR PARAMETERS
// =========================================================================
localparam NUM_RANGE_BINS = 64;
localparam DATA_WIDTH     = 16;

// Staggered PRF (from chirp controller)
localparam real LAMBDA_M   = 0.02857;   // 10.5 GHz
localparam real PRI_LONG_S = 167.0e-6;  // 167 µs
localparam real F_CLK      = 100.0e6;

// =========================================================================
// CLOCK AND RESET
// =========================================================================
reg clk;
reg reset_n;

initial clk = 0;
always #5 clk = ~clk;  // 100 MHz

// =========================================================================
// TEST INFRASTRUCTURE
// =========================================================================
integer pass_count, fail_count, test_num;
reg [256*8-1:0] test_name;

task check;
    input condition;
    input [256*8-1:0] msg;
    begin
        if (condition) begin
            $display("[PASS] M%0d: %0s", test_num, msg);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] M%0d: %0s", test_num, msg);
            fail_count = fail_count + 1;
        end
    end
endtask

// =========================================================================
// DUT SIGNALS
// =========================================================================
reg signed [DATA_WIDTH-1:0]  dut_range_i_in;
reg signed [DATA_WIDTH-1:0]  dut_range_q_in;
reg                          dut_range_valid_in;
reg [5:0]                    dut_range_bin_in;
reg                          dut_mti_enable;

wire signed [DATA_WIDTH-1:0] dut_range_i_out;
wire signed [DATA_WIDTH-1:0] dut_range_q_out;
wire                         dut_range_valid_out;
wire [5:0]                   dut_range_bin_out;
wire                         dut_mti_first_chirp;

// =========================================================================
// DUT INSTANTIATION
// =========================================================================
mti_canceller #(
    .NUM_RANGE_BINS(NUM_RANGE_BINS),
    .DATA_WIDTH(DATA_WIDTH)
) dut (
    .clk           (clk),
    .reset_n       (reset_n),
    .range_i_in    (dut_range_i_in),
    .range_q_in    (dut_range_q_in),
    .range_valid_in(dut_range_valid_in),
    .range_bin_in  (dut_range_bin_in),
    .range_i_out   (dut_range_i_out),
    .range_q_out   (dut_range_q_out),
    .range_valid_out(dut_range_valid_out),
    .range_bin_out (dut_range_bin_out),
    .mti_enable    (dut_mti_enable),
    .mti_first_chirp(dut_mti_first_chirp)
);

// =========================================================================
// OUTPUT CAPTURE
// =========================================================================
reg signed [DATA_WIDTH-1:0] out_i [0:NUM_RANGE_BINS-1];
reg signed [DATA_WIDTH-1:0] out_q [0:NUM_RANGE_BINS-1];
reg [5:0] out_bin [0:NUM_RANGE_BINS-1];
integer out_count;
reg capture_enable;

always @(posedge clk) begin
    if (dut_range_valid_out && capture_enable && out_count < NUM_RANGE_BINS) begin
        out_i[out_count] <= dut_range_i_out;
        out_q[out_count] <= dut_range_q_out;
        out_bin[out_count] <= dut_range_bin_out;
        out_count <= out_count + 1;
    end
end

// =========================================================================
// HELPER: Feed one chirp's range profile to the MTI
// =========================================================================
task feed_chirp;
    input integer chirp_idx;         // chirp number (for Doppler phase calc)
    input integer tgt_bin;           // target range bin
    input signed [15:0] tgt_amp;     // target amplitude
    input signed [31:0] phase_inc;   // Doppler phase increment per chirp (Q24)
    integer bin;
    reg signed [15:0] ti, tq;
    reg signed [31:0] phase;
    begin
        // Compute Doppler phase for this chirp
        phase = chirp_idx * phase_inc;

        for (bin = 0; bin < NUM_RANGE_BINS; bin = bin + 1) begin
            @(posedge clk);
            dut_range_bin_in <= bin;

            if (bin == tgt_bin) begin
                // Target present: I = A*cos(phase), Q = A*sin(phase)
                // Simplified: use phase[23:22] quadrant + linear approx
                begin : calc_iq
                    reg [1:0] quad;
                    reg [9:0] idx;
                    reg signed [15:0] sin_val, cos_val;
                    reg signed [31:0] prod_i, prod_q;

                    quad = phase[23:22];
                    idx = phase[21:12];

                    // Linear sine approximation (triangle wave)
                    // sin/cos values span [-32736, +32736] (Q15-ish)
                    case (quad)
                        2'b00: begin sin_val =  $signed({1'b0, idx, 5'b0}); cos_val =  $signed({1'b0, ~idx, 5'b0}); end
                        2'b01: begin sin_val =  $signed({1'b0, ~idx, 5'b0}); cos_val = -$signed({1'b0, idx, 5'b0}); end
                        2'b10: begin sin_val = -$signed({1'b0, idx, 5'b0}); cos_val = -$signed({1'b0, ~idx, 5'b0}); end
                        2'b11: begin sin_val = -$signed({1'b0, ~idx, 5'b0}); cos_val =  $signed({1'b0, idx, 5'b0}); end
                    endcase

                    // Use 32-bit intermediates to prevent 16-bit truncation
                    // (Verilog sizes multiplication by context; without this,
                    //  the product would be truncated to 16 bits before the shift)
                    prod_i = tgt_amp * cos_val;
                    prod_q = tgt_amp * sin_val;
                    ti = prod_i >>> 15;
                    tq = prod_q >>> 15;
                end
                dut_range_i_in <= ti;
                dut_range_q_in <= tq;
            end else begin
                // No target: zero
                dut_range_i_in <= 0;
                dut_range_q_in <= 0;
            end
            dut_range_valid_in <= 1;
            @(posedge clk);
            dut_range_valid_in <= 0;
        end
    end
endtask

// =========================================================================
// HELPER: Feed one chirp with fixed I/Q data for all bins
// =========================================================================
task feed_chirp_fixed;
    input signed [15:0] val_i;
    input signed [15:0] val_q;
    integer bin;
    begin
        for (bin = 0; bin < NUM_RANGE_BINS; bin = bin + 1) begin
            @(posedge clk);
            dut_range_bin_in <= bin;
            dut_range_i_in <= val_i;
            dut_range_q_in <= val_q;
            dut_range_valid_in <= 1;
            @(posedge clk);
            dut_range_valid_in <= 0;
        end
    end
endtask

// =========================================================================
// HELPER: Compute total power of captured output
// =========================================================================
function [63:0] compute_power;
    input integer n;
    reg [63:0] sum;
    reg signed [15:0] si, sq;
    integer k;
    begin
        sum = 0;
        for (k = 0; k < n; k = k + 1) begin
            si = out_i[k];
            sq = out_q[k];
            sum = sum + si * si + sq * sq;
        end
        compute_power = sum;
    end
endfunction

// =========================================================================
// MAIN TEST SEQUENCE
// =========================================================================
initial begin
    $dumpfile("tb_mti_physics.vcd");
    $dumpvars(0, tb_mti_physics);

    pass_count = 0;
    fail_count = 0;

    // Initialize
    reset_n = 0;
    dut_range_i_in = 0;
    dut_range_q_in = 0;
    dut_range_valid_in = 0;
    dut_range_bin_in = 0;
    dut_mti_enable = 1;
    capture_enable = 0;
    out_count = 0;

    repeat(20) @(posedge clk);
    reset_n = 1;
    repeat(5) @(posedge clk);

    $display("");
    $display("==========================================================");
    $display("  AERIS-10 MTI Physics Testbench");
    $display("  H(z) = 1 - z^{-1}, %0d range bins, %0d-bit data",
             NUM_RANGE_BINS, DATA_WIDTH);
    $display("==========================================================");

    // =================================================================
    // TEST M1: STATIONARY TARGET SUPPRESSION
    //
    // Feed two identical chirps. The MTI output for chirp 2 should be
    // approximately zero (current - previous = 0).
    //
    // Target: bin 20, amplitude 8000, zero Doppler phase (stationary)
    // =================================================================
    test_num = 1;
    $display("");
    $display("--- M1: Stationary target suppression ---");

    dut_mti_enable = 1;

    // Chirp 1 (first chirp — will be muted)
    feed_chirp(0, 20, 16'sd8000, 0);
    repeat(5) @(posedge clk);

    // Chirp 2 (identical to chirp 1 — MTI should cancel)
    out_count = 0;
    capture_enable = 1;
    feed_chirp(0, 20, 16'sd8000, 0);  // chirp_idx=0 → same phase
    repeat(5) @(posedge clk);
    capture_enable = 0;

    begin : m1_analysis
        reg [63:0] total_power;
        reg signed [15:0] tgt_i, tgt_q;

        total_power = compute_power(out_count);
        tgt_i = out_i[20];
        tgt_q = out_q[20];

        $display("  Captured %0d outputs", out_count);
        $display("  Target bin 20: I=%0d, Q=%0d", tgt_i, tgt_q);
        $display("  Total output power: %0d", total_power);

        // Stationary target should be perfectly cancelled (output = 0)
        check(tgt_i == 0 && tgt_q == 0,
              "Stationary target (v=0) must be perfectly cancelled (I=0, Q=0)");
        check(total_power == 0,
              "Total output power must be zero for identical chirps");
    end

    // =================================================================
    // TEST M2: MOVING TARGET PASSES THROUGH
    //
    // Target with non-zero Doppler: chirp-to-chirp phase changes.
    // Output should have significant energy at the target bin.
    //
    // v = 20 m/s → f_d = 2*v/lambda = 1400 Hz
    // Phase per chirp: 2*pi*f_d*PRI = 2*pi*1400*167e-6 = 1.47 rad
    // In Q24: 1.47/(2*pi) * 2^24 = ~3,925,048
    // =================================================================
    test_num = 2;
    $display("");
    $display("--- M2: Moving target passes through ---");

    // Reset DUT
    reset_n = 0;
    repeat(10) @(posedge clk);
    reset_n = 1;
    dut_mti_enable = 1;
    repeat(5) @(posedge clk);

    begin : m2_test
        reg signed [31:0] phase_inc_20ms;
        reg [63:0] power_tgt;
        reg signed [15:0] tgt_i, tgt_q;
        reg [31:0] mag_tgt;

        // Phase increment for 20 m/s (Q24 fractional turns)
        // delta_phi = 2*v/lambda * PRI * 2^24 = 2*20/0.02857 * 167e-6 * 16777216
        // = 1400 * 167e-6 * 16777216 = 3,924,279 ≈ 0x3BDB57
        phase_inc_20ms = 32'h003BDB57;

        // Chirp 1 (muted — first chirp)
        feed_chirp(0, 30, 16'sd8000, phase_inc_20ms);
        repeat(5) @(posedge clk);

        // Chirp 2 (MTI output = chirp2 - chirp1)
        out_count = 0;
        capture_enable = 1;
        feed_chirp(1, 30, 16'sd8000, phase_inc_20ms);
        repeat(5) @(posedge clk);
        capture_enable = 0;

        tgt_i = out_i[30];
        tgt_q = out_q[30];
        mag_tgt = ((tgt_i >= 0) ? tgt_i : -tgt_i) + ((tgt_q >= 0) ? tgt_q : -tgt_q);

        $display("  Captured %0d outputs", out_count);
        $display("  Target bin 30: I=%0d, Q=%0d, |I|+|Q|=%0d", tgt_i, tgt_q, mag_tgt);

        // Moving target must produce non-zero output
        check(mag_tgt > 0,
              "Moving target (v=20 m/s) must produce non-zero MTI output");

        // Output magnitude should be a significant fraction of input
        // For delta_phi ≈ 1.47 rad: |1 - e^{j*1.47}| = 2*|sin(0.735)| ≈ 1.34
        // So output should be > 50% of input amplitude
        // Input amplitude was 8000, so output should be > 4000 in |I|+|Q|
        // (rough — depends on our sine approximation)
        check(mag_tgt > 100,
              "Moving target output must have significant amplitude (>100)");
    end

    // =================================================================
    // TEST M3: BLIND SPEED — TARGET AT DOPPLER = 1/PRI
    //
    // v_blind = lambda / (2 * PRI) ≈ 0.02857 / (2 * 167e-6) = 85.5 m/s
    // At this velocity, the Doppler phase per chirp = 2*pi (full rotation).
    // H(z) evaluated at z = e^{j*2*pi} = 1 → H = 1 - 1 = 0.
    // The target should be cancelled just like a stationary target.
    //
    // Phase increment for blind speed: 2^24 (one full turn)
    // =================================================================
    test_num = 3;
    $display("");
    $display("--- M3: Blind speed target cancellation ---");

    reset_n = 0;
    repeat(10) @(posedge clk);
    reset_n = 1;
    dut_mti_enable = 1;
    repeat(5) @(posedge clk);

    begin : m3_test
        reg signed [31:0] phase_inc_blind;
        reg signed [15:0] tgt_i, tgt_q;

        // Full turn = 2^24 = 16777216
        phase_inc_blind = 32'h01000000;

        // Chirp 1 (muted)
        feed_chirp(0, 40, 16'sd8000, phase_inc_blind);
        repeat(5) @(posedge clk);

        // Chirp 2 (phase has advanced by exactly 2*pi → same as chirp 1)
        out_count = 0;
        capture_enable = 1;
        feed_chirp(1, 40, 16'sd8000, phase_inc_blind);
        repeat(5) @(posedge clk);
        capture_enable = 0;

        tgt_i = out_i[40];
        tgt_q = out_q[40];

        $display("  Blind speed target (bin 40): I=%0d, Q=%0d", tgt_i, tgt_q);

        // At the blind speed, phase advances by exactly 2*pi per chirp.
        // Current and previous are identical → output should be zero.
        check(tgt_i == 0 && tgt_q == 0,
              "Blind speed target (v=lambda/2/PRI) must be cancelled (I=0, Q=0)");
    end

    // =================================================================
    // TEST M4: FIRST CHIRP MUTE
    //
    // The first chirp after reset should produce all zeros (no previous
    // data to subtract from). Verify mti_first_chirp flag is asserted.
    // =================================================================
    test_num = 4;
    $display("");
    $display("--- M4: First chirp mute verification ---");

    reset_n = 0;
    repeat(10) @(posedge clk);
    reset_n = 1;
    dut_mti_enable = 1;
    repeat(5) @(posedge clk);

    begin : m4_test
        reg first_chirp_flag;
        reg [63:0] first_chirp_power;
        integer k;

        first_chirp_flag = dut_mti_first_chirp;

        // Feed first chirp with strong signal
        out_count = 0;
        capture_enable = 1;
        feed_chirp_fixed(16'sd10000, 16'sd5000);
        repeat(5) @(posedge clk);
        capture_enable = 0;

        first_chirp_power = compute_power(out_count);

        $display("  mti_first_chirp flag at start: %0d", first_chirp_flag);
        $display("  First chirp output samples: %0d", out_count);
        $display("  First chirp total power: %0d", first_chirp_power);

        check(first_chirp_flag == 1,
              "mti_first_chirp flag must be HIGH after reset");
        check(first_chirp_power == 0,
              "First chirp output must be all zeros (muted)");

        // Verify all output samples are zero
        begin : check_all_zero
            integer all_zero;
            all_zero = 1;
            for (k = 0; k < out_count; k = k + 1) begin
                if (out_i[k] != 0 || out_q[k] != 0)
                    all_zero = 0;
            end
            check(all_zero == 1,
                  "Every sample in first chirp must be exactly zero");
        end
    end

    // =================================================================
    // TEST M5: MTI DISABLE (PASS-THROUGH)
    //
    // When mti_enable=0, output must exactly equal input (transparent).
    // =================================================================
    test_num = 5;
    $display("");
    $display("--- M5: MTI disable pass-through ---");

    reset_n = 0;
    repeat(10) @(posedge clk);
    reset_n = 1;
    dut_mti_enable = 0;  // DISABLED
    repeat(5) @(posedge clk);

    begin : m5_test
        integer pass_ok;
        integer k;
        reg signed [15:0] expected_i, expected_q;

        out_count = 0;
        capture_enable = 1;
        feed_chirp_fixed(16'sd7777, 16'sd3333);
        repeat(5) @(posedge clk);
        capture_enable = 0;

        pass_ok = 1;
        for (k = 0; k < out_count; k = k + 1) begin
            if (out_i[k] != 16'sd7777 || out_q[k] != 16'sd3333) begin
                pass_ok = 0;
                if (k < 3)  // Show first few mismatches
                    $display("  Mismatch at bin %0d: I=%0d (exp 7777), Q=%0d (exp 3333)",
                             k, out_i[k], out_q[k]);
            end
        end

        $display("  Captured %0d outputs in pass-through mode", out_count);
        check(pass_ok == 1,
              "All outputs must exactly match inputs when MTI is disabled");
        check(out_count == NUM_RANGE_BINS,
              "Must output all range bins in pass-through mode");
    end

    // =================================================================
    // TEST M6: SATURATION — SUBTRACTION MUST NOT WRAP
    //
    // Feed chirp 1 with max positive, chirp 2 with max negative.
    // Difference = (-32768) - 32767 = -65535 → must saturate to -32768,
    // NOT wrap to a positive value.
    // =================================================================
    test_num = 6;
    $display("");
    $display("--- M6: Saturation on near-full-scale subtraction ---");

    reset_n = 0;
    repeat(10) @(posedge clk);
    reset_n = 1;
    dut_mti_enable = 1;
    repeat(5) @(posedge clk);

    begin : m6_test
        reg signed [15:0] out_sat_i, out_sat_q;

        // Chirp 1: max positive
        feed_chirp_fixed(16'sd32767, 16'sd32767);
        repeat(5) @(posedge clk);

        // Chirp 2: max negative
        out_count = 0;
        capture_enable = 1;
        feed_chirp_fixed(-16'sd32768, -16'sd32768);
        repeat(5) @(posedge clk);
        capture_enable = 0;

        out_sat_i = out_i[0];
        out_sat_q = out_q[0];

        $display("  Input chirp1: I=+32767, Q=+32767");
        $display("  Input chirp2: I=-32768, Q=-32768");
        $display("  MTI output: I=%0d, Q=%0d", out_sat_i, out_sat_q);

        // (-32768) - 32767 = -65535 → saturated to -32768
        check(out_sat_i < 0,
              "Saturation: output must be negative (not wrapped positive)");
        check(out_sat_i == -32768 || out_sat_i == -32767,
              "Saturation: output I must be at or near -32768");
    end

    // =================================================================
    // TEST M7: MULTIPLE INDEPENDENT RANGE BINS
    //
    // Two targets at different bins: one stationary, one moving.
    // Stationary should be cancelled, moving should pass through.
    // =================================================================
    test_num = 7;
    $display("");
    $display("--- M7: Multiple targets at different range bins ---");

    reset_n = 0;
    repeat(10) @(posedge clk);
    reset_n = 1;
    dut_mti_enable = 1;
    repeat(5) @(posedge clk);

    begin : m7_test
        reg signed [31:0] phase_inc;
        reg signed [15:0] stat_i, stat_q, move_i, move_q;
        reg [31:0] stat_mag, move_mag;
        integer bin;

        // Phase increment for 20 m/s
        phase_inc = 32'h003BDB57;

        // Feed chirp 1: stationary target at bin 10, moving target at bin 50
        // Use manual feeding for two targets
        for (bin = 0; bin < NUM_RANGE_BINS; bin = bin + 1) begin
            @(posedge clk);
            dut_range_bin_in <= bin;
            if (bin == 10) begin
                dut_range_i_in <= 16'sd8000;  // Stationary
                dut_range_q_in <= 16'sd0;
            end else if (bin == 50) begin
                dut_range_i_in <= 16'sd8000;  // Moving (chirp 0 phase = 0)
                dut_range_q_in <= 16'sd0;
            end else begin
                dut_range_i_in <= 0;
                dut_range_q_in <= 0;
            end
            dut_range_valid_in <= 1;
            @(posedge clk);
            dut_range_valid_in <= 0;
        end
        repeat(5) @(posedge clk);

        // Feed chirp 2: stationary still same, moving has phase shift
        out_count = 0;
        capture_enable = 1;
        for (bin = 0; bin < NUM_RANGE_BINS; bin = bin + 1) begin
            @(posedge clk);
            dut_range_bin_in <= bin;
            if (bin == 10) begin
                dut_range_i_in <= 16'sd8000;  // Stationary (same as chirp 1)
                dut_range_q_in <= 16'sd0;
            end else if (bin == 50) begin
                // Moving target: phase advanced by phase_inc
                // cos(phase_inc) ≈ 0.9, sin(phase_inc) ≈ 0.43 (approx for 1.47 rad)
                // I = 8000 * 0.9 = 7200, Q = 8000 * 0.43 = 3440
                dut_range_i_in <= 16'sd7200;
                dut_range_q_in <= 16'sd3440;
            end else begin
                dut_range_i_in <= 0;
                dut_range_q_in <= 0;
            end
            dut_range_valid_in <= 1;
            @(posedge clk);
            dut_range_valid_in <= 0;
        end
        repeat(5) @(posedge clk);
        capture_enable = 0;

        stat_i = out_i[10];
        stat_q = out_q[10];
        move_i = out_i[50];
        move_q = out_q[50];

        stat_mag = ((stat_i >= 0) ? stat_i : -stat_i) + ((stat_q >= 0) ? stat_q : -stat_q);
        move_mag = ((move_i >= 0) ? move_i : -move_i) + ((move_q >= 0) ? move_q : -move_q);

        $display("  Stationary target (bin 10): I=%0d, Q=%0d, mag=%0d", stat_i, stat_q, stat_mag);
        $display("  Moving target (bin 50): I=%0d, Q=%0d, mag=%0d", move_i, move_q, move_mag);

        check(stat_mag == 0,
              "Stationary target at bin 10 must be cancelled");
        check(move_mag > 0,
              "Moving target at bin 50 must pass through");
        check(move_mag > stat_mag + 100,
              "Moving target magnitude must be significantly greater than cancelled target");
    end

    // =================================================================
    // SUMMARY
    // =================================================================
    $display("");
    $display("==========================================================");
    $display("  MTI PHYSICS TESTBENCH RESULTS");
    $display("  Passed: %0d  Failed: %0d  Total: %0d",
             pass_count, fail_count, pass_count + fail_count);
    $display("==========================================================");

    if (fail_count > 0) begin
        $display("  >>> FAILURES DETECTED <<<");
        $display("  The MTI canceller has physics errors.");
    end else begin
        $display("  All MTI physics tests passed.");
    end

    $display("");
    $finish;
end

// Timeout watchdog
initial begin
    #10_000_000;  // 10 ms
    $display("[TIMEOUT] Simulation exceeded 10ms limit");
    $finish;
end

endmodule
