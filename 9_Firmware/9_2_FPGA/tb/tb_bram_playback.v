`timescale 1ns / 1ps
//
// tb_bram_playback.v — Testbench for BRAM Playback Top-Level
//
// Exercises the full pipeline: BRAM → Decimator → MTI → Doppler → DC Notch → CFAR
// Uses the same real ADI CN0566 data as tb_fullchain_realdata.v but through
// the playback engine and full top-level wrapper.
//
// Tests:
//   1. Playback engine starts on trigger, streams 32 chirps
//   2. Decimator produces correct number of outputs (2048)
//   3. Doppler processor completes frame
//   4. CFAR detector processes and reports detections (with CFAR disabled = simple threshold)
//   5. Doppler output matches golden reference (bit-exact)
//   6. CFAR produces expected detection count
//
// Compile:
//   iverilog -Wall -DSIMULATION -g2012 \
//     -o tb/tb_bram_playback.vvp \
//     tb/tb_bram_playback.v \
//     bram_playback.v radar_system_top_te0713_playback.v \
//     range_bin_decimator.v mti_canceller.v \
//     doppler_processor.v xfft_32.v fft_engine.v \
//     cfar_ca.v fpga_self_test.v
//
// Run from: 9_Firmware/9_2_FPGA/
//   vvp tb/tb_bram_playback.vvp
//

module tb_bram_playback;

// ============================================================================
// Parameters
// ============================================================================
localparam CLK_PERIOD = 15.0;   // ~66 MHz (matches STARTUPE2 ~65 MHz)
localparam CHIRPS = 32;
localparam INPUT_BINS = 1024;
localparam RANGE_BINS = 64;
localparam DOPPLER_BINS = 32;
localparam TOTAL_DECIM_OUT = CHIRPS * RANGE_BINS;       // 2048
localparam TOTAL_DOPPLER_OUT = RANGE_BINS * DOPPLER_BINS; // 2048

// Generous timeout: playback + processing + CFAR
localparam MAX_CYCLES = 4_000_000;

// ============================================================================
// DUT
// ============================================================================
wire [3:0] user_led;
wire [3:0] system_status;

radar_system_top_te0713_playback dut (
    .user_led(user_led),
    .system_status(system_status)
);

// ============================================================================
// Clock: DUT generates its own sim_clk internally via ifdef SIMULATION.
// No external clock driver needed.
// ============================================================================

// ============================================================================
// Reference Data
// ============================================================================
reg signed [15:0] ref_doppler_i [0:TOTAL_DOPPLER_OUT-1];
reg signed [15:0] ref_doppler_q [0:TOTAL_DOPPLER_OUT-1];

initial begin
    $readmemh("tb/cosim/real_data/hex/fullchain_doppler_ref_i.hex", ref_doppler_i);
    $readmemh("tb/cosim/real_data/hex/fullchain_doppler_ref_q.hex", ref_doppler_q);
end

// ============================================================================
// Internal Signal Access (hierarchical references)
// ============================================================================
wire        clk_buf         = dut.clk_buf;
wire        reset_n         = dut.reset_n;
wire        pb_done         = dut.pb_playback_done;
wire        pb_active       = dut.pb_playback_active;
wire [5:0]  pb_chirp        = dut.pb_chirp_count;
wire        pb_valid        = dut.pb_data_valid;
wire [31:0] pb_data         = dut.pb_data_out;
wire        decim_valid     = dut.decim_valid_out;
wire [15:0] decim_i         = dut.decim_i_out;
wire [15:0] decim_q         = dut.decim_q_out;
wire [5:0]  decim_bin       = dut.decim_bin_index;
wire        doppler_valid   = dut.doppler_valid;
wire [31:0] doppler_out     = dut.doppler_output;
wire [4:0]  doppler_bin     = dut.doppler_bin;
wire [5:0]  doppler_rbin    = dut.doppler_range_bin;
wire        frame_done      = dut.doppler_frame_done;
wire        cfar_busy       = dut.cfar_busy_w;
wire        cfar_det_valid  = dut.cfar_detect_valid;
wire        cfar_det_flag   = dut.cfar_detect_flag;
wire [15:0] cfar_det_count  = dut.cfar_detect_count;
wire [5:0]  cfar_det_range  = dut.cfar_detect_range;
wire [4:0]  cfar_det_doppler = dut.cfar_detect_doppler;

// ============================================================================
// Counters
// ============================================================================
integer decim_out_count = 0;
integer doppler_out_count = 0;
integer cfar_det_total = 0;
integer pb_sample_count = 0;

// Capture Doppler output for comparison
reg signed [15:0] cap_dop_i [0:TOTAL_DOPPLER_OUT-1];
reg signed [15:0] cap_dop_q [0:TOTAL_DOPPLER_OUT-1];
reg [5:0]  cap_rbin  [0:TOTAL_DOPPLER_OUT-1];
reg [4:0]  cap_dbin  [0:TOTAL_DOPPLER_OUT-1];

// Capture first N decimator outputs for comparison
localparam DECIM_CAPTURE_N = 128;  // First 2 chirps
reg signed [15:0] cap_decim_i [0:DECIM_CAPTURE_N-1];
reg signed [15:0] cap_decim_q [0:DECIM_CAPTURE_N-1];
reg [5:0]         cap_decim_bin [0:DECIM_CAPTURE_N-1];

always @(posedge clk_buf) begin
    if (pb_valid)
        pb_sample_count <= pb_sample_count + 1;

    if (decim_valid) begin
        if (decim_out_count < DECIM_CAPTURE_N) begin
            cap_decim_i[decim_out_count]   <= decim_i;
            cap_decim_q[decim_out_count]   <= decim_q;
            cap_decim_bin[decim_out_count] <= decim_bin;
        end
        decim_out_count <= decim_out_count + 1;
    end

    if (doppler_valid && doppler_out_count < TOTAL_DOPPLER_OUT) begin
        cap_dop_i[doppler_out_count] <= doppler_out[15:0];
        cap_dop_q[doppler_out_count] <= doppler_out[31:16];
        cap_rbin[doppler_out_count]  <= doppler_rbin;
        cap_dbin[doppler_out_count]  <= doppler_bin;
        doppler_out_count <= doppler_out_count + 1;
    end

    if (cfar_det_valid && cfar_det_flag)
        cfar_det_total <= cfar_det_total + 1;
end

// ============================================================================
// Pass/Fail Tracking
// ============================================================================
integer pass_count = 0;
integer fail_count = 0;
integer test_count = 0;

task check;
    input cond;
    input [511:0] label;
    begin
        test_count = test_count + 1;
        if (cond)
            pass_count = pass_count + 1;
        else begin
            $display("  [FAIL] %0s", label);
            fail_count = fail_count + 1;
        end
    end
endtask

// ============================================================================
// Main Test Sequence
// ============================================================================
integer cycle_count;
integer i;
integer n_exact, n_within_tol, max_err_i, max_err_q, mismatches_printed;
reg signed [31:0] diff_i, diff_q;
integer abs_diff_i, abs_diff_q;

initial begin
    // ---- Wait for reset to complete ----
    // The DUT has a ~500ms POR. In simulation, the counter rolls over quickly.
    // Wait for reset_n to go high.
    @(posedge reset_n);
    #(CLK_PERIOD * 10);

    $display("============================================================");
    $display("  BRAM Playback Full-Pipeline Testbench");
    $display("  BRAM -> Decimator(peak) -> MTI -> Doppler -> DC Notch -> CFAR");
    $display("  Real ADI CN0566 10.525 GHz X-band FMCW data");
    $display("============================================================");

    // ---- Check 1: DUT in idle state ----
    check(!pb_active, "Playback not active before trigger");
    check(!pb_done,   "Playback not done before trigger");

    // ---- Configure: CFAR disabled (simple threshold mode), MTI off, DC notch off ----
    dut.sim_vio_cfar_enable = 1'b0;   // Simple threshold mode
    dut.sim_vio_mti_enable  = 1'b0;   // MTI passthrough
    dut.sim_vio_dc_notch    = 3'd0;   // DC notch off

    // ---- Trigger playback ----
    $display("\n--- Triggering playback ---");
    // Set trigger BETWEEN clock edges to avoid delta-order race conditions.
    // The edge detector in the DUT needs to see trigger=0 on one posedge,
    // then trigger=1 on the next posedge to generate playback_trigger_pulse.
    @(negedge clk_buf);
    dut.sim_vio_playback_trigger = 1'b1;
    @(posedge clk_buf);   // Edge detector latches vio_playback_d=0, sees trigger=1 → pulse
    @(posedge clk_buf);   // bram_playback sees playback_start=1 from previous cycle's pulse
    @(posedge clk_buf);   // Extra cycle for state machine to advance
    @(negedge clk_buf);
    dut.sim_vio_playback_trigger = 1'b0;

    // ---- Wait for playback to start ----
    cycle_count = 0;
    while (!pb_active && cycle_count < 100) begin
        @(posedge clk_buf);
        cycle_count = cycle_count + 1;
    end
    check(pb_active, "Playback started after trigger");
    $display("  Playback started after %0d cycles", cycle_count);

    // ---- Wait for playback to complete ----
    $display("--- Waiting for playback to stream 32 chirps x 1024 samples ---");
    cycle_count = 0;
    while (!pb_done && cycle_count < MAX_CYCLES) begin
        @(posedge clk_buf);
        cycle_count = cycle_count + 1;
        // Progress updates
        if (cycle_count % 500000 == 0)
            $display("  ... %0d cycles, chirp=%0d, decim_out=%0d, dop_out=%0d",
                     cycle_count, pb_chirp, decim_out_count, doppler_out_count);
    end

    check(pb_done, "Playback completed");
    $display("  Playback done after %0d cycles, chirp_count=%0d", cycle_count, pb_chirp);
    $display("  Total playback samples: %0d (expected %0d)", pb_sample_count, CHIRPS * INPUT_BINS);

    // ---- Check decimator output count ----
    // Wait a few more cycles for decimator to emit final samples
    repeat(500) @(posedge clk_buf);
    $display("  Decimator outputs: %0d (expected %0d)", decim_out_count, TOTAL_DECIM_OUT);
    check(decim_out_count == TOTAL_DECIM_OUT, "Decimator output count == 2048");

    // ---- Wait for Doppler to finish processing ----
    $display("\n--- Waiting for Doppler processing ---");
    cycle_count = 0;
    while (doppler_out_count < TOTAL_DOPPLER_OUT && cycle_count < MAX_CYCLES) begin
        @(posedge clk_buf);
        cycle_count = cycle_count + 1;
    end
    $display("  Doppler outputs: %0d (expected %0d) in %0d extra cycles",
             doppler_out_count, TOTAL_DOPPLER_OUT, cycle_count);
    check(doppler_out_count == TOTAL_DOPPLER_OUT, "Doppler output count == 2048");

    // ---- Wait for CFAR to finish ----
    $display("\n--- Waiting for CFAR processing ---");
    cycle_count = 0;
    // Wait for frame_done, then CFAR processes
    while (cfar_busy && cycle_count < MAX_CYCLES) begin
        @(posedge clk_buf);
        cycle_count = cycle_count + 1;
    end
    // Give CFAR a few extra cycles to finish
    repeat(2000) @(posedge clk_buf);

    $display("  CFAR detections: %0d", cfar_det_total);
    $display("  CFAR detect_count register: %0d", cfar_det_count);
    check(cfar_det_count > 0, "CFAR detected at least 1 target (simple threshold mode)");

    // ==================================================================
    // BIT-FOR-BIT Doppler comparison against golden reference
    // ==================================================================
    $display("");
    $display("--- Comparing Doppler RTL output vs Python golden reference ---");

    max_err_i = 0;
    max_err_q = 0;
    n_exact = 0;
    n_within_tol = 0;
    mismatches_printed = 0;

    for (i = 0; i < doppler_out_count && i < TOTAL_DOPPLER_OUT; i = i + 1) begin
        diff_i = cap_dop_i[i] - ref_doppler_i[i];
        diff_q = cap_dop_q[i] - ref_doppler_q[i];

        abs_diff_i = (diff_i < 0) ? -diff_i : diff_i;
        abs_diff_q = (diff_q < 0) ? -diff_q : diff_q;

        if (abs_diff_i > max_err_i) max_err_i = abs_diff_i;
        if (abs_diff_q > max_err_q) max_err_q = abs_diff_q;

        if (diff_i == 0 && diff_q == 0)
            n_exact = n_exact + 1;

        if (abs_diff_i <= 0 && abs_diff_q <= 0)
            n_within_tol = n_within_tol + 1;

        if ((abs_diff_i > 0 || abs_diff_q > 0) && mismatches_printed < 20) begin
            $display("    [%4d] rbin=%2d dbin=%2d RTL=(%6d,%6d) REF=(%6d,%6d) ERR=(%4d,%4d)",
                     i, cap_rbin[i], cap_dbin[i],
                     $signed(cap_dop_i[i]), $signed(cap_dop_q[i]),
                     $signed(ref_doppler_i[i]), $signed(ref_doppler_q[i]),
                     diff_i, diff_q);
            mismatches_printed = mismatches_printed + 1;
        end
    end

    // Per-sample check
    for (i = 0; i < doppler_out_count && i < TOTAL_DOPPLER_OUT; i = i + 1) begin
        diff_i = cap_dop_i[i] - ref_doppler_i[i];
        diff_q = cap_dop_q[i] - ref_doppler_q[i];
        abs_diff_i = (diff_i < 0) ? -diff_i : diff_i;
        abs_diff_q = (diff_q < 0) ? -diff_q : diff_q;
        check(abs_diff_i == 0 && abs_diff_q == 0, "Doppler output bin exact match");
    end

    // ==================================================================
    // Summary
    // ==================================================================
    $display("");
    $display("============================================================");
    $display("  SUMMARY: BRAM Playback Full-Pipeline Testbench");
    $display("============================================================");
    $display("  Pipeline: BRAM -> Decim(peak) -> MTI(off) -> Doppler -> CFAR");
    $display("  Decimator outputs:  %0d (expected %0d)", decim_out_count, TOTAL_DECIM_OUT);
    $display("  Doppler outputs:    %0d (expected %0d)", doppler_out_count, TOTAL_DOPPLER_OUT);
    $display("  CFAR detections:    %0d", cfar_det_total);
    $display("  Doppler exact match: %0d / %0d", n_exact, doppler_out_count);
    $display("  Max error (I):       %0d", max_err_i);
    $display("  Max error (Q):       %0d", max_err_q);
    $display("  Structural checks:   6");
    $display("  Data match checks:   %0d", doppler_out_count);
    $display("  Pass: %0d  Fail: %0d", pass_count, fail_count);
    $display("============================================================");

    if (fail_count == 0)
        $display("RESULT: ALL TESTS PASSED (%0d/%0d)", pass_count, test_count);
    else
        $display("RESULT: %0d TESTS FAILED", fail_count);
    $display("============================================================");

    #(CLK_PERIOD * 10);
    $finish;
end

// ============================================================================
// Watchdog
// ============================================================================
initial begin
    #(CLK_PERIOD * MAX_CYCLES * 3);
    $display("[TIMEOUT] Simulation exceeded maximum time — aborting");
    $display("  decim_out=%0d dop_out=%0d cfar_det=%0d",
             decim_out_count, doppler_out_count, cfar_det_total);
    $display("SOME TESTS FAILED");
    $finish;
end

endmodule
