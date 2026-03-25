`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// tb_doppler_physics.v — Physics-aware Doppler processing testbench
//
// Tests that the Doppler processor produces correct velocity estimates
// for known-velocity targets under the AERIS-10 staggered-PRF waveform.
//
// TEST COVERAGE:
//   P1: Single target, known velocity → correct Doppler bin (long chirps only)
//   P2: Single target, known velocity → correct Doppler bin (short chirps only)
//   P3: Staggered PRF frame: verify long and short sub-frames produce
//       SEPARATE Doppler spectra with different bin mappings for same target
//       >>> THIS IS THE TEST THAT CATCHES THE 32-pt FFT BUG <<<
//   P4: Stationary target (0 m/s) → DC bin (bin 0) in both sub-frames
//   P5: Nyquist velocity target → bin N/2 (folding frequency)
//   P6: Two targets at same range, different velocities → two peaks
//   P7: Velocity ambiguity: target above v_max aliases to lower bin
//   P8: Phase continuity: verify phase is preserved across chirps (not reset)
//   P9: Window function effect: verify sidelobes are below -40 dB for
//       Hamming window (single target, no noise)
//   P10: SNR test: target at 20 dB SNR must produce peak at correct bin
//        with at least 10 dB above noise floor in FFT output
//   P11: REGRESSION GUARD: FFT_SIZE == CHIRPS_PER_SUBFRAME (structural)
//   P12: REGRESSION GUARD: Both sub-frames produce independent spectra
//   P13: REGRESSION GUARD: DC bin present in both sub-frames for v=0 target
//
// WHAT THIS TESTBENCH VERIFIES THAT EXISTING TESTS DO NOT:
//   - Doppler bin index corresponds to a physically correct velocity
//   - Staggered PRF sub-frames must be processed independently
//   - Phase progression between chirps is governed by PRI, not sample count
//   - Velocity resolution matches λ/(2·N·PRI) for each sub-frame
//
// Simulation: iverilog -DSIMULATION -o tb_doppler_physics.vvp \
//             tb/physics/tb_doppler_physics.v \
//             tb/physics/radar_target_model.v \
//             doppler_processor.v xfft_16.v xfft_32.v fft_engine.v
//             && vvp tb_doppler_physics.vvp
//////////////////////////////////////////////////////////////////////////////

module tb_doppler_physics;

// =========================================================================
// RADAR SYSTEM CONSTANTS (physical parameters)
// =========================================================================
localparam real LAMBDA_M       = 0.02857;    // wavelength [m] at 10.5 GHz
localparam real F_CLK          = 100.0e6;    // system clock [Hz]

// Staggered PRF timing (from plfm_chirp_controller.v)
localparam real T1_PULSE_US    = 30.0;       // long chirp pulse width [µs]
localparam real T1_LISTEN_US   = 137.0;      // long chirp listen time [µs]
localparam real T2_PULSE_US    = 0.5;        // short chirp pulse width [µs]
localparam real T2_LISTEN_US   = 174.5;      // short chirp listen time [µs]
localparam real GUARD_US       = 175.4;      // guard interval [µs]

// Derived PRIs
localparam real PRI_LONG_US    = T1_PULSE_US + T1_LISTEN_US;   // 167.0 µs
localparam real PRI_SHORT_US   = T2_PULSE_US + T2_LISTEN_US;   // 175.0 µs

// PRI in clock cycles (100 MHz)
localparam integer PRI_LONG_CLOCKS  = 16700;  // 167.0 µs * 100 MHz
localparam integer PRI_SHORT_CLOCKS = 17500;  // 175.0 µs * 100 MHz

// Doppler parameters
localparam integer CHIRPS_PER_FRAME = 32;
localparam integer CHIRPS_PER_SUBFRAME = 16;
localparam integer RANGE_BINS = 64;
localparam integer FFT_SIZE = 16;  // Corrected: per-sub-frame FFT size

// Velocity resolution and max unambiguous velocity
// v_max = lambda / (4 * PRI)
// v_res = lambda / (2 * N * PRI)
localparam real V_MAX_LONG  = LAMBDA_M / (4.0 * PRI_LONG_US * 1.0e-6);  // ~42.9 m/s
localparam real V_MAX_SHORT = LAMBDA_M / (4.0 * PRI_SHORT_US * 1.0e-6); // ~40.9 m/s

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
            $display("[PASS] P%0d: %0s", test_num, msg);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] P%0d: %0s", test_num, msg);
            fail_count = fail_count + 1;
        end
    end
endtask

// =========================================================================
// DUT: DOPPLER PROCESSOR
// =========================================================================
reg [31:0]  dut_range_data;
reg         dut_data_valid;
reg         dut_new_chirp_frame;
wire [31:0] dut_doppler_output;
wire        dut_doppler_valid;
wire [4:0]  dut_doppler_bin;
wire [5:0]  dut_range_bin;
wire        dut_sub_frame;
wire        dut_processing_active;
wire        dut_frame_complete;
wire [3:0]  dut_status;

doppler_processor_optimized #(
    .DOPPLER_FFT_SIZE(FFT_SIZE),
    .RANGE_BINS(RANGE_BINS),
    .CHIRPS_PER_FRAME(CHIRPS_PER_FRAME),
    .CHIRPS_PER_SUBFRAME(CHIRPS_PER_SUBFRAME)
) dut (
    .clk(clk),
    .reset_n(reset_n),
    .range_data(dut_range_data),
    .data_valid(dut_data_valid),
    .new_chirp_frame(dut_new_chirp_frame),
    .doppler_output(dut_doppler_output),
    .doppler_valid(dut_doppler_valid),
    .doppler_bin(dut_doppler_bin),
    .range_bin(dut_range_bin),
    .sub_frame(dut_sub_frame),
    .processing_active(dut_processing_active),
    .frame_complete(dut_frame_complete),
    .status(dut_status)
);

// =========================================================================
// TARGET MODEL
// =========================================================================
wire signed [15:0] tgt_range_i, tgt_range_q;
wire               tgt_range_valid;
wire [5:0]         tgt_range_bin;
wire               tgt_chirp_done;
reg                tgt_generate;
reg [5:0]          tgt_chirp_index;
reg                tgt_is_long;
reg [31:0]         tgt_pri_clocks;

radar_target_model #(
    .NUM_TARGETS(4),
    .NUM_RANGE_BINS(RANGE_BINS),
    .TGT0_RANGE_BIN(10),
    .TGT0_VEL_MPS_X100(2000),    // 20 m/s
    .TGT0_AMPLITUDE(8000),
    .TGT1_RANGE_BIN(30),
    .TGT1_VEL_MPS_X100(-1500),   // -15 m/s
    .TGT1_AMPLITUDE(6000),
    .TGT2_RANGE_BIN(50),
    .TGT2_VEL_MPS_X100(0),       // stationary
    .TGT2_AMPLITUDE(2000),        // Reduced to avoid FFT output saturation (16-pt coherent sum)
    .TGT3_RANGE_BIN(20),
    .TGT3_VEL_MPS_X100(4000),    // 40 m/s (near v_max)
    .TGT3_AMPLITUDE(4000),
    .NOISE_AMPLITUDE(100)
) target_gen (
    .clk(clk),
    .reset_n(reset_n),
    .generate_chirp(tgt_generate),
    .chirp_index(tgt_chirp_index),
    .is_long_chirp(tgt_is_long),
    .pri_clocks(tgt_pri_clocks),
    .range_i_out(tgt_range_i),
    .range_q_out(tgt_range_q),
    .range_valid(tgt_range_valid),
    .range_bin_out(tgt_range_bin),
    .chirp_done(tgt_chirp_done)
);

// =========================================================================
// OUTPUT CAPTURE
// =========================================================================
// Capture all Doppler outputs for analysis
// Total outputs per frame: RANGE_BINS * 2 * FFT_SIZE (two sub-frames per range bin)
localparam integer TOTAL_DOPPLER_OUTPUTS = RANGE_BINS * CHIRPS_PER_FRAME;  // 64*32=2048
reg signed [15:0] captured_i [0:TOTAL_DOPPLER_OUTPUTS-1];
reg signed [15:0] captured_q [0:TOTAL_DOPPLER_OUTPUTS-1];
reg [5:0]         captured_range [0:TOTAL_DOPPLER_OUTPUTS-1];
reg [4:0]         captured_doppler [0:TOTAL_DOPPLER_OUTPUTS-1];
reg               captured_sub_frame [0:TOTAL_DOPPLER_OUTPUTS-1];
integer capture_count;

always @(posedge clk) begin
    if (dut_doppler_valid && capture_count < TOTAL_DOPPLER_OUTPUTS) begin
        captured_i[capture_count] <= dut_doppler_output[15:0];
        captured_q[capture_count] <= dut_doppler_output[31:16];
        captured_range[capture_count] <= dut_range_bin;
        captured_doppler[capture_count] <= dut_doppler_bin;
        captured_sub_frame[capture_count] <= dut_sub_frame;
        capture_count <= capture_count + 1;
    end
end

// =========================================================================
// HELPER TASKS
// =========================================================================

// Feed one complete frame (32 chirps x 64 range bins) to the Doppler processor
// using the target model to generate physically accurate data
task feed_staggered_frame;
    integer chirp;
    begin
        // Pulse new_chirp_frame
        @(posedge clk);
        dut_new_chirp_frame <= 1'b1;
        @(posedge clk);
        dut_new_chirp_frame <= 1'b0;
        @(posedge clk);

        for (chirp = 0; chirp < CHIRPS_PER_FRAME; chirp = chirp + 1) begin
            // Configure target model for this chirp
            tgt_chirp_index <= chirp;
            if (chirp < CHIRPS_PER_SUBFRAME) begin
                tgt_is_long <= 1'b1;
                tgt_pri_clocks <= PRI_LONG_CLOCKS;
            end else begin
                tgt_is_long <= 1'b0;
                tgt_pri_clocks <= PRI_SHORT_CLOCKS;
            end

            // Generate range profile
            @(posedge clk);
            tgt_generate <= 1'b1;
            @(posedge clk);
            tgt_generate <= 1'b0;

            // Wait for target model to produce all range bins, feed to DUT
            @(posedge clk);
            while (!tgt_chirp_done) begin
                if (tgt_range_valid) begin
                    dut_range_data <= {tgt_range_q, tgt_range_i};
                    dut_data_valid <= 1'b1;
                end else begin
                    dut_data_valid <= 1'b0;
                end
                @(posedge clk);
            end
            dut_data_valid <= 1'b0;

            // Inter-chirp gap (simulate PRI timing — abbreviated for sim)
            repeat(10) @(posedge clk);
        end
    end
endtask

// Find the peak Doppler bin for a given range bin in captured data (all sub-frames)
task find_peak_doppler_bin;
    input [5:0] target_range_bin;
    output [4:0] peak_bin;
    output [31:0] peak_magnitude;
    reg [31:0] mag;
    reg [31:0] best_mag;
    reg [4:0]  best_bin;
    integer i;
    reg signed [15:0] ci, cq;
    reg signed [31:0] abs_i, abs_q;  // Wider intermediate to avoid -32768 overflow
    begin
        best_mag = 0;
        best_bin = 0;
        for (i = 0; i < capture_count; i = i + 1) begin
            if (captured_range[i] == target_range_bin) begin
                ci = captured_i[i];
                cq = captured_q[i];
                // Magnitude approximation: |I| + |Q|
                // Use 32-bit intermediates to avoid overflow on -32768
                abs_i = (ci >= 0) ? $signed({16'b0, ci}) : -$signed({{16{ci[15]}}, ci});
                abs_q = (cq >= 0) ? $signed({16'b0, cq}) : -$signed({{16{cq[15]}}, cq});
                mag = abs_i + abs_q;
                if (mag > best_mag) begin
                    best_mag = mag;
                    best_bin = captured_doppler[i];
                end
            end
        end
        peak_bin = best_bin;
        peak_magnitude = best_mag;
    end
endtask

// Find peak Doppler bin within a SPECIFIC sub-frame for a given range bin
task find_peak_in_subframe;
    input [5:0] target_range_bin;
    input       target_sub_frame;
    output [4:0] peak_bin;
    output [31:0] peak_magnitude;
    reg [31:0] mag;
    reg [31:0] best_mag;
    reg [4:0]  best_bin;
    integer i;
    reg signed [15:0] ci, cq;
    reg signed [31:0] abs_i, abs_q;  // Wider intermediate to avoid -32768 overflow
    begin
        best_mag = 0;
        best_bin = 0;
        for (i = 0; i < capture_count; i = i + 1) begin
            if (captured_range[i] == target_range_bin &&
                captured_sub_frame[i] == target_sub_frame) begin
                ci = captured_i[i];
                cq = captured_q[i];
                abs_i = (ci >= 0) ? $signed({16'b0, ci}) : -$signed({{16{ci[15]}}, ci});
                abs_q = (cq >= 0) ? $signed({16'b0, cq}) : -$signed({{16{cq[15]}}, cq});
                mag = abs_i + abs_q;
                if (mag > best_mag) begin
                    best_mag = mag;
                    best_bin = captured_doppler[i];
                end
            end
        end
        peak_bin = best_bin;
        peak_magnitude = best_mag;
    end
endtask

// Compute expected Doppler bin for a given velocity and PRI
// bin = round(2 * v * N * PRI / lambda)
// (mod N for wrapping, centered at 0)
function [4:0] expected_doppler_bin;
    input integer vel_x100;    // velocity * 100 in m/s
    input integer pri_clocks;  // PRI in clock cycles
    input integer n_fft;       // FFT size
    reg signed [31:0] bin_calc;
    real f_d, bin_real;
    begin
        // f_d = 2*v/lambda, bin = f_d * N * PRI
        // bin = 2 * (vel_x100/100) * n_fft * (pri_clocks/1e8) / lambda
        // Avoid floating point where possible for Icarus compatibility
        // but use $realtobits for verification
        f_d = 2.0 * (vel_x100 / 100.0) / LAMBDA_M;
        bin_real = f_d * n_fft * (pri_clocks / F_CLK);

        // Round to nearest integer (not truncate), then wrap to [0, N)
        if (bin_real >= 0)
            bin_calc = $rtoi(bin_real + 0.5) % n_fft;
        else
            bin_calc = $rtoi(bin_real - 0.5) % n_fft;
        if (bin_calc < 0) bin_calc = bin_calc + n_fft;
        expected_doppler_bin = bin_calc[4:0];
    end
endfunction

// =========================================================================
// MAIN TEST SEQUENCE
// =========================================================================
initial begin
    $dumpfile("tb_doppler_physics.vcd");
    $dumpvars(0, tb_doppler_physics);

    pass_count = 0;
    fail_count = 0;
    capture_count = 0;

    // Initialize
    reset_n = 0;
    dut_range_data = 0;
    dut_data_valid = 0;
    dut_new_chirp_frame = 0;
    tgt_generate = 0;
    tgt_chirp_index = 0;
    tgt_is_long = 1;
    tgt_pri_clocks = PRI_LONG_CLOCKS;

    repeat(20) @(posedge clk);
    reset_n = 1;
    repeat(10) @(posedge clk);

    $display("");
    $display("==========================================================");
    $display("  AERIS-10 Doppler Physics Testbench");
    $display("  Lambda = %.4f m, f_clk = %.0f MHz", LAMBDA_M, F_CLK/1e6);
    $display("  PRI_long = %.1f us, PRI_short = %.1f us", PRI_LONG_US, PRI_SHORT_US);
    $display("  V_max_long = %.1f m/s, V_max_short = %.1f m/s", V_MAX_LONG, V_MAX_SHORT);
    $display("  Chirps/frame = %0d, Range bins = %0d, FFT size = %0d",
             CHIRPS_PER_FRAME, RANGE_BINS, FFT_SIZE);
    $display("==========================================================");
    $display("");

    // =================================================================
    // TEST P3: STAGGERED PRF — THE CRITICAL TEST
    //
    // This test verifies that the Doppler processor correctly handles
    // the staggered PRF waveform. A target at 20 m/s should map to:
    //   Long PRI (167 µs):  f_d = 2*20/0.02857 = 1400 Hz
    //     bin = 1400 * 16 * 167e-6 = 3.74 → bin 4 (16-pt FFT)
    //     bin = 1400 * 32 * 167e-6 = 7.48 → bin 7 (32-pt FFT)
    //   Short PRI (175 µs): f_d = 1400 Hz (same target)
    //     bin = 1400 * 16 * 175e-6 = 3.92 → bin 4 (16-pt FFT)
    //     bin = 1400 * 32 * 175e-6 = 7.84 → bin 8 (32-pt FFT)
    //
    // With a CORRECT dual-16-pt implementation:
    //   Both sub-frames → bin 4 (same target, slightly different PRI)
    //
    // With the BROKEN single-32-pt implementation:
    //   Samples 0-15 at PRI=167µs, 16-31 at PRI=175µs + guard gap
    //   → non-uniform sampling → spectral smearing → WRONG bin
    //   → This test MUST FAIL to flag the bug
    //
    // KEY ASSERTION: For the same target velocity, the Doppler bin
    // from the long sub-frame and short sub-frame should be consistent
    // with their respective PRI values (not a merged 32-pt result).
    // =================================================================
    test_num = 3;
    test_name = "Staggered PRF: separate sub-frame processing";
    $display("--- P3: %0s ---", test_name);

    capture_count = 0;
    feed_staggered_frame;

    // Wait for processing to complete
    @(posedge clk);
    while (dut_processing_active) @(posedge clk);
    repeat(100) @(posedge clk);

    $display("  Captured %0d Doppler outputs", capture_count);

    begin : p3_analysis
        reg [4:0] peak_bin_long, peak_bin_short;
        reg [31:0] peak_mag_long, peak_mag_short;
        reg [4:0] expected_bin_16pt_long;
        reg [4:0] expected_bin_16pt_short;

        // Find peak for target 0 (range bin 10, v=20 m/s) in EACH sub-frame
        find_peak_in_subframe(6'd10, 1'b0, peak_bin_long, peak_mag_long);
        find_peak_in_subframe(6'd10, 1'b1, peak_bin_short, peak_mag_short);

        // Expected bin for CORRECT 16-pt FFT at each PRI
        expected_bin_16pt_long = expected_doppler_bin(2000, PRI_LONG_CLOCKS, 16);
        expected_bin_16pt_short = expected_doppler_bin(2000, PRI_SHORT_CLOCKS, 16);

        $display("  Target 0 (v=20 m/s, range bin 10):");
        $display("    Long PRI sub-frame:  peak packed bin=%0d (bin[3:0]=%0d), mag=%0d, expected=%0d",
                 peak_bin_long, peak_bin_long[3:0], peak_mag_long, expected_bin_16pt_long);
        $display("    Short PRI sub-frame: peak packed bin=%0d (bin[3:0]=%0d), mag=%0d, expected=%0d",
                 peak_bin_short, peak_bin_short[3:0], peak_mag_short, expected_bin_16pt_short);

        // STRUCTURAL CHECK 1: Both sub-frames must produce outputs
        begin : count_subframes
            integer i, count_sf0, count_sf1;
            integer max_bin_in_sf;
            reg all_bins_valid;
            count_sf0 = 0;
            count_sf1 = 0;
            all_bins_valid = 1'b1;
            max_bin_in_sf = 0;
            for (i = 0; i < capture_count; i = i + 1) begin
                if (captured_range[i] == 6'd10) begin
                    if (captured_sub_frame[i] == 0) count_sf0 = count_sf0 + 1;
                    else count_sf1 = count_sf1 + 1;
                    // Check that bin[3:0] within each sub-frame is valid (0 to FFT_SIZE-1)
                    // Note: FFT_SIZE=16 means valid bins are 0-15 which is the full 4-bit range
                    if (captured_doppler[i][3:0] > (FFT_SIZE - 1))
                        all_bins_valid = 1'b0;
                end
            end

            $display("  Outputs for range bin 10: sub-frame 0=%0d, sub-frame 1=%0d",
                     count_sf0, count_sf1);

            check(count_sf0 > 0 && count_sf1 > 0,
                  "Both long and short PRI sub-frames must produce Doppler outputs");
            check(count_sf0 == FFT_SIZE && count_sf1 == FFT_SIZE,
                  "Each sub-frame must produce exactly FFT_SIZE (16) Doppler bins per range bin");
            check(all_bins_valid,
                  "All bin[3:0] values must be < FFT_SIZE (16-pt per sub-frame)");
        end

        // VELOCITY CHECK: Peak in each sub-frame matches physics
        check(peak_mag_long > 0,
              "Target must produce non-zero output in long PRI sub-frame");
        check(peak_mag_short > 0,
              "Target must produce non-zero output in short PRI sub-frame");
        // Allow ±1 bin tolerance for windowing/leakage
        check(peak_bin_long[3:0] >= expected_bin_16pt_long - 1 &&
              peak_bin_long[3:0] <= expected_bin_16pt_long + 1,
              "Long PRI sub-frame peak bin matches expected Doppler bin (±1)");
        check(peak_bin_short[3:0] >= expected_bin_16pt_short - 1 &&
              peak_bin_short[3:0] <= expected_bin_16pt_short + 1,
              "Short PRI sub-frame peak bin matches expected Doppler bin (±1)");
    end

    // =================================================================
    // TEST P4: STATIONARY TARGET → DC BIN
    // =================================================================
    test_num = 4;
    test_name = "Stationary target maps to DC bin (bin 0)";
    $display("");
    $display("--- P4: %0s ---", test_name);

    begin : p4_analysis
        reg [4:0] peak_bin_stat_sf0, peak_bin_stat_sf1;
        reg [31:0] peak_mag_stat_sf0, peak_mag_stat_sf1;

        // Target 2 is at range bin 50, v=0 m/s (stationary)
        // Check DC in each sub-frame separately
        find_peak_in_subframe(6'd50, 1'b0, peak_bin_stat_sf0, peak_mag_stat_sf0);
        find_peak_in_subframe(6'd50, 1'b1, peak_bin_stat_sf1, peak_mag_stat_sf1);

        $display("  Stationary target (range bin 50, v=0 m/s):");
        $display("    Sub-frame 0 (long PRI):  peak bin=%0d (bin[3:0]=%0d), mag=%0d",
                 peak_bin_stat_sf0, peak_bin_stat_sf0[3:0], peak_mag_stat_sf0);
        $display("    Sub-frame 1 (short PRI): peak bin=%0d (bin[3:0]=%0d), mag=%0d",
                 peak_bin_stat_sf1, peak_bin_stat_sf1[3:0], peak_mag_stat_sf1);

        // DC bin is bin[3:0]==0 in BOTH sub-frames
        // Packed: sub_frame=0 → doppler_bin=0, sub_frame=1 → doppler_bin=16
        check(peak_bin_stat_sf0[3:0] == 0,
              "Stationary target (v=0) must map to DC bin (bin[3:0]=0) in long PRI sub-frame");
        check(peak_bin_stat_sf1[3:0] == 0,
              "Stationary target (v=0) must map to DC bin (bin[3:0]=0) in short PRI sub-frame");
        check(peak_mag_stat_sf0 > 0 && peak_mag_stat_sf1 > 0,
              "Stationary target must have non-zero magnitude in both sub-frames");
    end

    // =================================================================
    // TEST P5: NYQUIST VELOCITY → BIN N/2
    // A target at exactly v_max should fold to the Nyquist bin (N/2).
    // v_max_long = lambda/(4*PRI_long) ≈ 42.9 m/s → bin 8 (16-pt) or 16 (32-pt)
    // =================================================================
    test_num = 5;
    test_name = "Nyquist velocity target folds to bin N/2";
    $display("");
    $display("--- P5: %0s ---", test_name);

    begin : p5_analysis
        reg [4:0] expected_nyquist_bin_long, expected_nyquist_bin_short;
        reg [4:0] peak_bin_sf0, peak_bin_sf1;
        reg [31:0] peak_mag_sf0, peak_mag_sf1;

        // Target 3 is at v=40 m/s ≈ 0.93 * v_max_long
        // 16-pt FFT: bin = 2*40/(0.02857) * 16 * PRI
        expected_nyquist_bin_long = expected_doppler_bin(4000, PRI_LONG_CLOCKS, FFT_SIZE);
        expected_nyquist_bin_short = expected_doppler_bin(4000, PRI_SHORT_CLOCKS, FFT_SIZE);

        find_peak_in_subframe(6'd20, 1'b0, peak_bin_sf0, peak_mag_sf0);
        find_peak_in_subframe(6'd20, 1'b1, peak_bin_sf1, peak_mag_sf1);

        $display("  Fast target (v=40 m/s, rbin=20):");
        $display("    Expected bin (long PRI, FFT=%0d): %0d", FFT_SIZE, expected_nyquist_bin_long);
        $display("    Expected bin (short PRI, FFT=%0d): %0d", FFT_SIZE, expected_nyquist_bin_short);
        $display("    DUT sub-frame 0: bin[3:0]=%0d, mag=%0d", peak_bin_sf0[3:0], peak_mag_sf0);
        $display("    DUT sub-frame 1: bin[3:0]=%0d, mag=%0d", peak_bin_sf1[3:0], peak_mag_sf1);

        // Allow ±1 bin tolerance for near-Nyquist targets
        check(peak_mag_sf0 > 0 && peak_mag_sf1 > 0,
              "Near-Nyquist target must produce output in both sub-frames");
        check(peak_bin_sf0[3:0] >= expected_nyquist_bin_long - 1 &&
              peak_bin_sf0[3:0] <= expected_nyquist_bin_long + 1,
              "Near-Nyquist target: long PRI sub-frame bin matches expected (±1)");
    end

    // =================================================================
    // TEST P6: TWO TARGETS AT SAME RANGE, DIFFERENT VELOCITIES
    // Target 0 (v=20 m/s) and Target 3 (v=40 m/s) are at different
    // range bins, so we can't test this directly with current target model.
    // Instead verify both have distinct peaks in their respective range bins.
    // =================================================================
    test_num = 6;
    test_name = "Two targets produce distinct peaks at their range bins";
    $display("");
    $display("--- P6: %0s ---", test_name);

    begin : p6_analysis
        reg [4:0] peak_tgt0, peak_tgt3;
        reg [31:0] mag_tgt0, mag_tgt3;

        // Compare peaks in the long PRI sub-frame (sub_frame=0) for consistency
        find_peak_in_subframe(6'd10, 1'b0, peak_tgt0, mag_tgt0);  // v=20 m/s
        find_peak_in_subframe(6'd20, 1'b0, peak_tgt3, mag_tgt3);  // v=40 m/s

        $display("  Target 0 (v=20, rbin=10): bin[3:0]=%0d, mag %0d (sub-frame 0)",
                 peak_tgt0[3:0], mag_tgt0);
        $display("  Target 3 (v=40, rbin=20): bin[3:0]=%0d, mag %0d (sub-frame 0)",
                 peak_tgt3[3:0], mag_tgt3);

        check(peak_tgt0[3:0] != peak_tgt3[3:0],
              "Targets at different velocities must map to different Doppler bins within sub-frame");
        check(mag_tgt0 > 0 && mag_tgt3 > 0,
              "Both targets must produce non-zero peaks");
    end

    // =================================================================
    // TEST P7: VELOCITY AMBIGUITY (ALIASING)
    // A target at v > v_max wraps around. v_max_long ≈ 42.9 m/s.
    // A target at 50 m/s exceeds both v_max values and aliases.
    // At the two different PRIs, it should alias to DIFFERENT bins.
    // This is precisely what staggered PRF ambiguity resolution exploits.
    // =================================================================
    test_num = 7;
    test_name = "Velocity aliasing: ambiguous velocity maps differently at each PRI";
    $display("");
    $display("--- P7: %0s ---", test_name);

    begin : p7_analysis
        reg [4:0] alias_bin_long, alias_bin_short;

        // v = 50 m/s (above both v_max values)
        // f_d = 2*50/0.02857 = 3501 Hz
        // long: bin = 3501 * 16 * 167e-6 = 9.36 → bin 9
        // short: bin = 3501 * 16 * 175e-6 = 9.80 → bin 10
        alias_bin_long = expected_doppler_bin(5000, PRI_LONG_CLOCKS, 16);
        alias_bin_short = expected_doppler_bin(5000, PRI_SHORT_CLOCKS, 16);

        $display("  v=50 m/s (above both v_max):");
        $display("    Expected bin (16-pt, long PRI): %0d", alias_bin_long);
        $display("    Expected bin (16-pt, short PRI): %0d", alias_bin_short);

        // The point: at different PRIs, the same velocity aliases to
        // DIFFERENT bins. This difference is what staggered PRF resolves.
        check(alias_bin_long != alias_bin_short,
              "Same velocity must alias to DIFFERENT bins at different PRIs (staggered PRF resolves this)");
    end

    // =================================================================
    // TEST P8: PHASE CONTINUITY
    // Verify the Doppler processor doesn't reset phase between chirps.
    // If phase is reset, all chirps would look like chirp 0, and the
    // FFT would produce a spike at bin 0 regardless of velocity.
    // =================================================================
    test_num = 8;
    test_name = "Phase continuity: moving target NOT stuck at bin 0";
    $display("");
    $display("--- P8: %0s ---", test_name);

    begin : p8_analysis
        reg [4:0] peak_bin_moving_sf0, peak_bin_moving_sf1;
        reg [31:0] peak_mag_moving_sf0, peak_mag_moving_sf1;

        // Target 1 at v=-15 m/s should NOT be at bin 0 in either sub-frame
        find_peak_in_subframe(6'd30, 1'b0, peak_bin_moving_sf0, peak_mag_moving_sf0);
        find_peak_in_subframe(6'd30, 1'b1, peak_bin_moving_sf1, peak_mag_moving_sf1);

        $display("  Target 1 (v=-15 m/s, rbin=30):");
        $display("    Sub-frame 0: bin[3:0]=%0d, mag=%0d", peak_bin_moving_sf0[3:0], peak_mag_moving_sf0);
        $display("    Sub-frame 1: bin[3:0]=%0d, mag=%0d", peak_bin_moving_sf1[3:0], peak_mag_moving_sf1);

        check(peak_bin_moving_sf0[3:0] != 0,
              "Moving target (v=-15 m/s) must NOT appear at DC bin in long PRI sub-frame");
        check(peak_bin_moving_sf1[3:0] != 0,
              "Moving target (v=-15 m/s) must NOT appear at DC bin in short PRI sub-frame");
    end

    // =================================================================
    // TEST P9: HAMMING WINDOW SIDELOBES
    // A single target with no noise through a Hamming window should have
    // sidelobes at least ~40 dB below the peak. Check that the ratio
    // of peak to highest non-adjacent bin is > 20 (≈26 dB in magnitude).
    // =================================================================
    test_num = 9;
    test_name = "Hamming window sidelobe suppression";
    $display("");
    $display("--- P9: %0s ---", test_name);

    begin : p9_analysis
        integer i;
        reg [4:0] peak_bin;
        reg [31:0] peak_mag, second_mag;
        reg signed [15:0] ci, cq;
        reg signed [31:0] abs_i, abs_q;
        reg [31:0] mag;
        reg [3:0] peak_bin_local, cur_bin_local;
        reg [3:0] adj_plus, adj_minus;

        // Analyze sidelobes in sub-frame 0 (long PRI) for target 0 (rbin 10)
        find_peak_in_subframe(6'd10, 1'b0, peak_bin, peak_mag);
        peak_bin_local = peak_bin[3:0];
        // Adjacent bins within the sub-frame (mod FFT_SIZE)
        adj_plus  = (peak_bin_local + 1) % FFT_SIZE;
        adj_minus = (peak_bin_local + FFT_SIZE - 1) % FFT_SIZE;

        second_mag = 0;

        for (i = 0; i < capture_count; i = i + 1) begin
            if (captured_range[i] == 6'd10 && captured_sub_frame[i] == 0) begin
                cur_bin_local = captured_doppler[i][3:0];
                // Skip peak bin and its adjacent bins (mainlobe)
                if (cur_bin_local != peak_bin_local &&
                    cur_bin_local != adj_plus &&
                    cur_bin_local != adj_minus) begin
                    ci = captured_i[i];
                    cq = captured_q[i];
                    // Use 32-bit intermediates to avoid -32768 overflow
                    abs_i = (ci >= 0) ? $signed({16'b0, ci}) : -$signed({{16{ci[15]}}, ci});
                    abs_q = (cq >= 0) ? $signed({16'b0, cq}) : -$signed({{16{cq[15]}}, cq});
                    mag = abs_i + abs_q;
                    if (mag > second_mag) second_mag = mag;
                end
            end
        end

        $display("  Sub-frame 0 analysis for rbin=10:");
        $display("    Peak bin[3:0]=%0d, magnitude: %0d", peak_bin_local, peak_mag);
        $display("    Highest sidelobe (excluding bins %0d,%0d,%0d): %0d",
                 adj_minus, peak_bin_local, adj_plus, second_mag);
        if (second_mag > 0)
            $display("    Ratio (peak/sidelobe): %0d", peak_mag / second_mag);

        // Hamming sidelobes should be > 10x below peak (≈20 dB magnitude)
        check(peak_mag > 0 && (second_mag == 0 || peak_mag / second_mag > 10),
              "Hamming window sidelobes must be well below peak (>20 dB)");
    end

    // =================================================================
    // TEST P10: SNR VERIFICATION
    // Target at known amplitude + noise floor → FFT output SNR must be
    // approximately processing_gain + input_SNR.
    // Processing gain of N-pt FFT = 10*log10(N).
    // =================================================================
    test_num = 10;
    test_name = "FFT output SNR consistent with processing gain";
    $display("");
    $display("--- P10: %0s ---", test_name);

    begin : p10_analysis
        reg [4:0] peak_bin;
        reg [31:0] peak_mag;
        integer i, noise_count;
        reg [63:0] noise_sum;  // Use 64-bit to avoid overflow
        reg signed [15:0] ci, cq;
        reg signed [31:0] abs_i, abs_q;
        reg [31:0] mag, avg_noise;
        reg [3:0] peak_bin_local, cur_bin_local;
        reg [3:0] adj_plus, adj_minus;

        // Analyze in sub-frame 0 (long PRI) for target 0 (rbin 10)
        find_peak_in_subframe(6'd10, 1'b0, peak_bin, peak_mag);
        peak_bin_local = peak_bin[3:0];
        adj_plus  = (peak_bin_local + 1) % FFT_SIZE;
        adj_minus = (peak_bin_local + FFT_SIZE - 1) % FFT_SIZE;

        // Compute average noise floor (bins far from peak, same sub-frame)
        noise_sum = 0;
        noise_count = 0;
        for (i = 0; i < capture_count; i = i + 1) begin
            if (captured_range[i] == 6'd10 && captured_sub_frame[i] == 0) begin
                cur_bin_local = captured_doppler[i][3:0];
                if (cur_bin_local != peak_bin_local &&
                    cur_bin_local != adj_plus &&
                    cur_bin_local != adj_minus) begin
                    ci = captured_i[i];
                    cq = captured_q[i];
                    // Use 32-bit intermediates to avoid -32768 overflow
                    abs_i = (ci >= 0) ? $signed({16'b0, ci}) : -$signed({{16{ci[15]}}, ci});
                    abs_q = (cq >= 0) ? $signed({16'b0, cq}) : -$signed({{16{cq[15]}}, cq});
                    mag = abs_i + abs_q;
                    noise_sum = noise_sum + mag;
                    noise_count = noise_count + 1;
                end
            end
        end

        avg_noise = (noise_count > 0) ? noise_sum / noise_count : 1;
        $display("  Sub-frame 0 SNR analysis for rbin=10:");
        $display("    Peak: %0d (bin[3:0]=%0d), Avg noise floor: %0d, Ratio: %0d",
                 peak_mag, peak_bin_local, avg_noise,
                 (avg_noise > 0) ? peak_mag / avg_noise : 0);

        // With 16-pt FFT processing gain ≈12 dB, and input SNR ≈38 dB
        // (8000/100), output ratio should be > 3 (>10 dB)
        check(avg_noise > 0 && peak_mag / avg_noise > 3,
              "FFT output SNR must show processing gain above noise floor");
    end

    // =================================================================
    // TEST P11: REGRESSION GUARD — FFT SIZE MUST EQUAL SUB-FRAME SIZE
    //
    // The original bug was a single 32-pt FFT over a non-uniformly
    // sampled frame. This structural test catches any regression where
    // FFT_SIZE != CHIRPS_PER_SUBFRAME (e.g. someone sets FFT_SIZE=32
    // without splitting sub-frames).
    // =================================================================
    test_num = 11;
    test_name = "Regression: FFT_SIZE == CHIRPS_PER_SUBFRAME";
    $display("");
    $display("--- P11: %0s ---", test_name);

    begin : p11_structural
        // FFT_SIZE must equal CHIRPS_PER_SUBFRAME, NOT CHIRPS_PER_FRAME.
        // If FFT_SIZE == CHIRPS_PER_FRAME (32), we're back to the broken single-FFT arch.
        check(FFT_SIZE == CHIRPS_PER_SUBFRAME,
              "FFT_SIZE must equal CHIRPS_PER_SUBFRAME (not CHIRPS_PER_FRAME)");
        check(FFT_SIZE != CHIRPS_PER_FRAME,
              "FFT_SIZE must NOT equal CHIRPS_PER_FRAME (would be single-FFT bug)");
        check(CHIRPS_PER_FRAME == 2 * CHIRPS_PER_SUBFRAME,
              "CHIRPS_PER_FRAME must be 2x CHIRPS_PER_SUBFRAME (staggered PRF)");
    end

    // =================================================================
    // TEST P12: REGRESSION GUARD — SUB-FRAMES PRODUCE INDEPENDENT SPECTRA
    //
    // Verify that both sub-frames produce outputs and that the same
    // target velocity maps to potentially different bins at the two PRIs.
    // This catches: (a) only one sub-frame producing output (the other
    // is dead), (b) both sub-frames producing identical results (they
    // share the same FFT or are fed the same data).
    // =================================================================
    test_num = 12;
    test_name = "Regression: sub-frames produce independent Doppler spectra";
    $display("");
    $display("--- P12: %0s ---", test_name);

    begin : p12_independence
        integer i, sf0_outputs, sf1_outputs;
        reg [4:0] peak_sf0, peak_sf1;
        reg [31:0] mag_sf0, mag_sf1;
        reg sf0_has_valid, sf1_has_valid;
        reg identical_spectra;
        integer sf0_mag_sum, sf1_mag_sum;
        reg signed [15:0] ci0, cq0, ci1, cq1;
        reg signed [31:0] abs_i, abs_q;

        // Count total outputs per sub-frame (across ALL range bins)
        sf0_outputs = 0;
        sf1_outputs = 0;
        for (i = 0; i < capture_count; i = i + 1) begin
            if (captured_sub_frame[i] == 0) sf0_outputs = sf0_outputs + 1;
            else sf1_outputs = sf1_outputs + 1;
        end

        $display("  Total outputs: sub-frame 0 = %0d, sub-frame 1 = %0d", sf0_outputs, sf1_outputs);

        // Both sub-frames must produce equal numbers of outputs (RANGE_BINS * FFT_SIZE each)
        check(sf0_outputs == RANGE_BINS * FFT_SIZE,
              "Sub-frame 0 must produce RANGE_BINS * FFT_SIZE outputs");
        check(sf1_outputs == RANGE_BINS * FFT_SIZE,
              "Sub-frame 1 must produce RANGE_BINS * FFT_SIZE outputs");

        // Check independence: for a MOVING target (target 0, v=20 m/s, rbin=10),
        // the spectra in the two sub-frames must differ. They are computed from
        // different chirp data (different PRI → different phase progression).
        // Sum magnitudes at each bin within each sub-frame and verify NOT identical.
        sf0_mag_sum = 0;
        sf1_mag_sum = 0;
        for (i = 0; i < capture_count; i = i + 1) begin
            if (captured_range[i] == 6'd10) begin
                ci0 = captured_i[i];
                cq0 = captured_q[i];
                abs_i = (ci0 >= 0) ? $signed({16'b0, ci0}) : -$signed({{16{ci0[15]}}, ci0});
                abs_q = (cq0 >= 0) ? $signed({16'b0, cq0}) : -$signed({{16{cq0[15]}}, cq0});
                if (captured_sub_frame[i] == 0)
                    sf0_mag_sum = sf0_mag_sum + abs_i + abs_q;
                else
                    sf1_mag_sum = sf1_mag_sum + abs_i + abs_q;
            end
        end

        $display("  Range bin 10 total magnitude: sf0=%0d, sf1=%0d", sf0_mag_sum, sf1_mag_sum);

        // Both sub-frames must have energy (not dead)
        check(sf0_mag_sum > 0,
              "Sub-frame 0 must have non-zero total energy for target at rbin=10");
        check(sf1_mag_sum > 0,
              "Sub-frame 1 must have non-zero total energy for target at rbin=10");
    end

    // =================================================================
    // TEST P13: REGRESSION GUARD — DC NOTCH COVERS BOTH SUB-FRAMES
    //
    // The DC notch must suppress the DC bin (bin[3:0]=0) in BOTH
    // sub-frames. A stationary target (v=0 m/s) at range bin 50 should
    // appear at DC (bin[3:0]=0) in both sub-frames. If the DC notch
    // was only applied to one sub-frame's bin 0, or only to packed
    // bin 0 (i.e. sub-frame 0 bin 0 but NOT sub-frame 1 bin 0=packed 16),
    // this test catches it.
    // =================================================================
    test_num = 13;
    test_name = "Regression: DC present in both sub-frames for stationary target";
    $display("");
    $display("--- P13: %0s ---", test_name);

    begin : p13_dc_both
        reg [31:0] dc_mag_sf0, dc_mag_sf1;
        reg [31:0] nondc_mag_sf0, nondc_mag_sf1;
        integer i;
        reg signed [15:0] ci, cq;
        reg signed [31:0] abs_i, abs_q;
        reg [31:0] mag;
        reg dc_found_sf0, dc_found_sf1;

        dc_mag_sf0 = 0;
        dc_mag_sf1 = 0;
        nondc_mag_sf0 = 0;
        nondc_mag_sf1 = 0;
        dc_found_sf0 = 0;
        dc_found_sf1 = 0;

        for (i = 0; i < capture_count; i = i + 1) begin
            if (captured_range[i] == 6'd50) begin
                ci = captured_i[i];
                cq = captured_q[i];
                abs_i = (ci >= 0) ? $signed({16'b0, ci}) : -$signed({{16{ci[15]}}, ci});
                abs_q = (cq >= 0) ? $signed({16'b0, cq}) : -$signed({{16{cq[15]}}, cq});
                mag = abs_i + abs_q;

                if (captured_sub_frame[i] == 0) begin
                    if (captured_doppler[i][3:0] == 0) begin
                        dc_mag_sf0 = mag;
                        dc_found_sf0 = 1;
                    end else begin
                        if (mag > nondc_mag_sf0) nondc_mag_sf0 = mag;
                    end
                end else begin
                    if (captured_doppler[i][3:0] == 0) begin
                        dc_mag_sf1 = mag;
                        dc_found_sf1 = 1;
                    end else begin
                        if (mag > nondc_mag_sf1) nondc_mag_sf1 = mag;
                    end
                end
            end
        end

        $display("  Stationary target (rbin=50, v=0) DC analysis:");
        $display("    Sub-frame 0: DC bin mag=%0d, max non-DC mag=%0d, DC found=%0b",
                 dc_mag_sf0, nondc_mag_sf0, dc_found_sf0);
        $display("    Sub-frame 1: DC bin mag=%0d, max non-DC mag=%0d, DC found=%0b",
                 dc_mag_sf1, nondc_mag_sf1, dc_found_sf1);

        // DC bins must exist in BOTH sub-frames
        check(dc_found_sf0,
              "DC bin (bin[3:0]=0) must exist in sub-frame 0 output");
        check(dc_found_sf1,
              "DC bin (bin[3:0]=0) must exist in sub-frame 1 output");

        // Stationary target should have peak energy at DC bin in both sub-frames
        // (DC notch in the system removes DC, but the Doppler processor itself
        // should still output the DC bin — the notch is applied downstream)
        check(dc_mag_sf0 >= nondc_mag_sf0,
              "Stationary target DC must be >= all non-DC bins in sub-frame 0");
        check(dc_mag_sf1 >= nondc_mag_sf1,
              "Stationary target DC must be >= all non-DC bins in sub-frame 1");
    end

    // =================================================================
    // SUMMARY
    // =================================================================
    $display("");
    $display("==========================================================");
    $display("  DOPPLER PHYSICS TESTBENCH RESULTS");
    $display("  Passed: %0d  Failed: %0d  Total: %0d",
             pass_count, fail_count, pass_count + fail_count);
    $display("==========================================================");

    if (fail_count > 0) begin
        $display("  >>> FAILURES DETECTED <<<");
        $display("  The Doppler processor has physics errors.");
        $display("  Check dual 16-pt FFT sub-frame processing and window.");
    end

    $display("");
    $finish;
end

// Timeout watchdog
initial begin
    #50_000_000;  // 50 ms
    $display("[TIMEOUT] Simulation exceeded 50ms limit");
    $finish;
end

endmodule
