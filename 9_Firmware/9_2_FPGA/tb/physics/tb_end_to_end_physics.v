`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// tb_end_to_end_physics.v
//
// Physics-aware end-to-end testbench for the post-DDC AERIS-10 processing
// chain:
//   matched_filter_processing_chain -> range_bin_decimator -> mti_canceller
//   -> doppler_processor_optimized -> cfar_ca
//
// Tests:
//   E1: Single stationary target, MTI disabled -> detect at range bin 16, DC
//   E2: Single moving target (v=20 m/s), MTI enabled -> detect at range~16, doppler~3-4
//   E3: Single stationary target, MTI enabled -> suppressed (no detect near range 16)
//   E4: Two targets (stationary + moving), MTI enabled -> only moving target detected
//
// Simulation:
//   iverilog -DSIMULATION -o tb_end_to_end_physics.vvp \
//       tb/physics/tb_end_to_end_physics.v \
//       matched_filter_processing_chain.v range_bin_decimator.v mti_canceller.v \
//       doppler_processor.v xfft_16.v xfft_32.v fft_engine.v cfar_ca.v \
//       && vvp tb_end_to_end_physics.vvp
////////////////////////////////////////////////////////////////////////////////

module tb_end_to_end_physics;

// -----------------------------------------------------------------------------
// Parameters
// -----------------------------------------------------------------------------
localparam integer CLK_PERIOD_NS     = 10;
localparam integer FFT_SAMPLES       = 1024;
localparam integer RANGE_BINS        = 64;
localparam integer DOPPLER_BINS      = 32;
localparam integer CHIRPS_PER_FRAME  = 32;
localparam integer TOTAL_DET_CELLS   = RANGE_BINS * DOPPLER_BINS;

localparam integer TARGET_DELAY_E1   = 256;
localparam integer TARGET_DELAY_E4_M = 512;
localparam integer TARGET_RBIN_E1    = TARGET_DELAY_E1 / 16;
localparam integer TARGET_RBIN_E4_M  = TARGET_DELAY_E4_M / 16;

localparam real PI                   = 3.141592653589793;
localparam real LAMBDA_M             = 0.02857;
localparam real PRI_S                = 167.0e-6;
localparam real V_MOVING_MPS         = 20.0;
localparam real FD_MOVING_HZ         = (2.0 * V_MOVING_MPS) / LAMBDA_M;
localparam real DELTA_PHI_MOVING     = 2.0 * PI * FD_MOVING_HZ * PRI_S; // ~1.47 rad

localparam integer CHIRP_REF_AMPL    = 250;
localparam integer TARGET_AMPL       = 300;
localparam real CHIRP_SLOPE          = 0.35;

// -----------------------------------------------------------------------------
// Clock / Reset
// -----------------------------------------------------------------------------
reg clk;
reg reset_n;

initial clk = 1'b0;
always #(CLK_PERIOD_NS/2) clk = ~clk;  // 100 MHz

// -----------------------------------------------------------------------------
// Test infrastructure
// -----------------------------------------------------------------------------
integer pass_count;
integer fail_count;
integer test_num;

task check;
    input condition;
    input [256*8-1:0] msg;
    begin
        if (condition) begin
            $display("[PASS] E%0d: %0s", test_num, msg);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] E%0d: %0s", test_num, msg);
            fail_count = fail_count + 1;
        end
    end
endtask

function integer abs_int;
    input integer x;
    begin
        if (x < 0) abs_int = -x;
        else abs_int = x;
    end
endfunction

function signed [15:0] sat16;
    input integer x;
    begin
        if (x > 32767)
            sat16 = 16'sd32767;
        else if (x < -32768)
            sat16 = -16'sd32768;
        else
            sat16 = x[15:0];
    end
endfunction

function integer det_flat_idx;
    input integer rb;
    input integer db;
    begin
        det_flat_idx = rb * DOPPLER_BINS + db;
    end
endfunction

// -----------------------------------------------------------------------------
// Chirp reference ROM
// -----------------------------------------------------------------------------
reg signed [15:0] chirp_i [0:FFT_SAMPLES-1];
reg signed [15:0] chirp_q [0:FFT_SAMPLES-1];

integer gen_n;
real gen_phase;
initial begin
    for (gen_n = 0; gen_n < FFT_SAMPLES; gen_n = gen_n + 1) begin
        gen_phase = PI * CHIRP_SLOPE * (gen_n * gen_n) / (FFT_SAMPLES * 1.0);
        chirp_i[gen_n] = $rtoi(CHIRP_REF_AMPL * $cos(gen_phase));
        chirp_q[gen_n] = $rtoi(CHIRP_REF_AMPL * $sin(gen_phase));
    end
end

// -----------------------------------------------------------------------------
// Interconnect signals
// -----------------------------------------------------------------------------
reg signed [15:0] adc_data_i;
reg signed [15:0] adc_data_q;
reg               adc_valid;
reg [5:0]         chirp_counter;

reg signed [15:0] long_chirp_real;
reg signed [15:0] long_chirp_imag;
reg signed [15:0] short_chirp_real;
reg signed [15:0] short_chirp_imag;

wire signed [15:0] mf_range_i;
wire signed [15:0] mf_range_q;
wire               mf_range_valid;
wire [3:0]         mf_state;

wire signed [15:0] decim_i;
wire signed [15:0] decim_q;
wire               decim_valid;
wire [5:0]         decim_bin;
wire               decim_watchdog_timeout;

reg                mti_enable;
wire signed [15:0] mti_i;
wire signed [15:0] mti_q;
wire               mti_valid;
wire [5:0]         mti_bin;
wire               mti_first_chirp;

reg                new_chirp_frame;
wire [31:0]        doppler_output;
wire               doppler_valid;
wire [4:0]         doppler_bin;
wire [5:0]         doppler_range_bin;
wire               doppler_sub_frame;
wire               doppler_processing_active;
wire               doppler_frame_complete;
wire [3:0]         doppler_status;

reg [3:0]          cfg_guard_cells;
reg [4:0]          cfg_train_cells;
reg [7:0]          cfg_alpha;
reg [1:0]          cfg_cfar_mode;
reg                cfg_cfar_enable;
reg [15:0]         cfg_simple_threshold;

wire               cfar_detect_flag;
wire               cfar_detect_valid;
wire [5:0]         cfar_detect_range;
wire [4:0]         cfar_detect_doppler;
wire [16:0]        cfar_detect_magnitude;
wire [16:0]        cfar_detect_threshold;
wire [15:0]        cfar_detect_count;
wire               cfar_busy;
wire [7:0]         cfar_status;

wire [31:0] doppler_range_data = {mti_q, mti_i};

// -----------------------------------------------------------------------------
// DUT chain instantiation
// -----------------------------------------------------------------------------
matched_filter_processing_chain u_matched_filter (
    .clk(clk),
    .reset_n(reset_n),
    .adc_data_i(adc_data_i),
    .adc_data_q(adc_data_q),
    .adc_valid(adc_valid),
    .chirp_counter(chirp_counter),
    .long_chirp_real(long_chirp_real),
    .long_chirp_imag(long_chirp_imag),
    .short_chirp_real(short_chirp_real),
    .short_chirp_imag(short_chirp_imag),
    .range_profile_i(mf_range_i),
    .range_profile_q(mf_range_q),
    .range_profile_valid(mf_range_valid),
    .chain_state(mf_state)
);

range_bin_decimator #(
    .INPUT_BINS(1024),
    .OUTPUT_BINS(64),
    .DECIMATION_FACTOR(16)
) u_decimator (
    .clk(clk),
    .reset_n(reset_n),
    .range_i_in(mf_range_i),
    .range_q_in(mf_range_q),
    .range_valid_in(mf_range_valid),
    .range_i_out(decim_i),
    .range_q_out(decim_q),
    .range_valid_out(decim_valid),
    .range_bin_index(decim_bin),
    .decimation_mode(2'b00),
    .start_bin(10'd0),
    .watchdog_timeout(decim_watchdog_timeout)
);

mti_canceller #(
    .NUM_RANGE_BINS(64),
    .DATA_WIDTH(16)
) u_mti (
    .clk(clk),
    .reset_n(reset_n),
    .range_i_in(decim_i),
    .range_q_in(decim_q),
    .range_valid_in(decim_valid),
    .range_bin_in(decim_bin),
    .range_i_out(mti_i),
    .range_q_out(mti_q),
    .range_valid_out(mti_valid),
    .range_bin_out(mti_bin),
    .mti_enable(mti_enable),
    .mti_first_chirp(mti_first_chirp)
);

doppler_processor_optimized #(
    .DOPPLER_FFT_SIZE(16),
    .RANGE_BINS(64),
    .CHIRPS_PER_FRAME(32),
    .CHIRPS_PER_SUBFRAME(16)
) u_doppler (
    .clk(clk),
    .reset_n(reset_n),
    .range_data(doppler_range_data),
    .data_valid(mti_valid),
    .new_chirp_frame(new_chirp_frame),
    .doppler_output(doppler_output),
    .doppler_valid(doppler_valid),
    .doppler_bin(doppler_bin),
    .range_bin(doppler_range_bin),
    .sub_frame(doppler_sub_frame),
    .processing_active(doppler_processing_active),
    .frame_complete(doppler_frame_complete),
    .status(doppler_status)
);

cfar_ca #(
    .NUM_RANGE_BINS(64),
    .NUM_DOPPLER_BINS(32),
    .MAG_WIDTH(17),
    .ALPHA_WIDTH(8),
    .MAX_GUARD(8),
    .MAX_TRAIN(16)
) u_cfar (
    .clk(clk),
    .reset_n(reset_n),
    .doppler_data(doppler_output),
    .doppler_valid(doppler_valid),
    .doppler_bin_in(doppler_bin),
    .range_bin_in(doppler_range_bin),
    .frame_complete(doppler_frame_complete),
    .cfg_guard_cells(cfg_guard_cells),
    .cfg_train_cells(cfg_train_cells),
    .cfg_alpha(cfg_alpha),
    .cfg_cfar_mode(cfg_cfar_mode),
    .cfg_cfar_enable(cfg_cfar_enable),
    .cfg_simple_threshold(cfg_simple_threshold),
    .detect_flag(cfar_detect_flag),
    .detect_valid(cfar_detect_valid),
    .detect_range(cfar_detect_range),
    .detect_doppler(cfar_detect_doppler),
    .detect_magnitude(cfar_detect_magnitude),
    .detect_threshold(cfar_detect_threshold),
    .detect_count(cfar_detect_count),
    .cfar_busy(cfar_busy),
    .cfar_status(cfar_status)
);

// -----------------------------------------------------------------------------
// Detection capture — store peak magnitude per (range, doppler) cell
// -----------------------------------------------------------------------------
integer det_mag  [0:TOTAL_DET_CELLS-1];  // peak magnitude per cell
integer det_cnt  [0:TOTAL_DET_CELLS-1];  // detection count per cell
integer det_total;
integer det_flat;
integer mti_valid_count;

always @(posedge clk) begin
    if (!reset_n) begin
        det_total <= 0;
        mti_valid_count <= 0;
    end else begin
        if (mti_valid)
            mti_valid_count <= mti_valid_count + 1;

        if (cfar_detect_valid && cfar_detect_flag) begin
            det_total <= det_total + 1;
            det_flat = det_flat_idx(cfar_detect_range, cfar_detect_doppler);
            if (det_flat >= 0 && det_flat < TOTAL_DET_CELLS) begin
                det_cnt[det_flat] = det_cnt[det_flat] + 1;
                if ($signed({1'b0, cfar_detect_magnitude}) > det_mag[det_flat])
                    det_mag[det_flat] = cfar_detect_magnitude;
            end
        end
    end
end

task clear_detections;
    integer i;
    begin
        det_total = 0;
        for (i = 0; i < TOTAL_DET_CELLS; i = i + 1) begin
            det_mag[i] = 0;
            det_cnt[i] = 0;
        end
    end
endtask

// find_peak_near_range — returns the cell with the HIGHEST MAGNITUDE
// detection within the range window [target_r - tol_r, target_r + tol_r].
task find_peak_near_range;
    input integer target_r;
    input integer tol_r;
    output integer peak_r;
    output integer peak_d;
    output integer peak_mag;
    output integer sum_cnt;
    integer r;
    integer d;
    integer m;
    integer c;
    begin
        peak_r = 0;
        peak_d = 0;
        peak_mag = 0;
        sum_cnt = 0;
        for (r = 0; r < RANGE_BINS; r = r + 1) begin
            if (r >= (target_r - tol_r) && r <= (target_r + tol_r)) begin
                for (d = 0; d < DOPPLER_BINS; d = d + 1) begin
                    c = det_cnt[det_flat_idx(r, d)];
                    m = det_mag[det_flat_idx(r, d)];
                    sum_cnt = sum_cnt + c;
                    if (m > peak_mag) begin
                        peak_mag = m;
                        peak_r = r;
                        peak_d = d;
                    end
                end
            end
        end
    end
endtask

function integer range_window_count;
    input integer target_r;
    input integer tol_r;
    integer r;
    integer d;
    integer s;
    begin
        s = 0;
        for (r = 0; r < RANGE_BINS; r = r + 1) begin
            if (r >= (target_r - tol_r) && r <= (target_r + tol_r)) begin
                for (d = 0; d < DOPPLER_BINS; d = d + 1)
                    s = s + det_cnt[det_flat_idx(r, d)];
            end
        end
        range_window_count = s;
    end
endfunction

// -----------------------------------------------------------------------------
// Stimulus helpers
// -----------------------------------------------------------------------------
task apply_reset;
    begin
        reset_n <= 1'b0;
        adc_data_i <= 16'sd0;
        adc_data_q <= 16'sd0;
        adc_valid <= 1'b0;
        chirp_counter <= 6'd0;
        long_chirp_real <= 16'sd0;
        long_chirp_imag <= 16'sd0;
        short_chirp_real <= 16'sd0;
        short_chirp_imag <= 16'sd0;
        new_chirp_frame <= 1'b0;
        mti_enable <= 1'b0;
        cfg_guard_cells <= 4'd2;
        cfg_train_cells <= 5'd8;
        cfg_alpha <= 8'h05;
        cfg_cfar_mode <= 2'b00;
        cfg_cfar_enable <= 1'b1;
        cfg_simple_threshold <= 16'd600;
        repeat (20) @(posedge clk);
        reset_n <= 1'b1;
        repeat (20) @(posedge clk);
    end
endtask

task pulse_new_frame;
    begin
        @(posedge clk);
        new_chirp_frame <= 1'b1;
        @(posedge clk);
        new_chirp_frame <= 1'b0;
    end
endtask

task feed_one_chirp;
    input integer delay0;
    input integer amp0;
    input real phase0;
    input integer delay1;
    input integer amp1;
    input real phase1;
    integer n;
    integer idx;
    integer base_i;
    integer base_q;
    real c0;
    real s0;
    real c1;
    real s1;
    real acc_i;
    real acc_q;
    integer mti_start;
    integer timeout;
    begin
        c0 = $cos(phase0);
        s0 = $sin(phase0);
        c1 = $cos(phase1);
        s1 = $sin(phase1);

        mti_start = mti_valid_count;

        for (n = 0; n < FFT_SAMPLES; n = n + 1) begin
            acc_i = 0.0;
            acc_q = 0.0;

            if (amp0 != 0 && n >= delay0 && (n - delay0) < FFT_SAMPLES) begin
                idx = n - delay0;
                base_i = chirp_i[idx];
                base_q = chirp_q[idx];
                acc_i = acc_i + (amp0 / (CHIRP_REF_AMPL * 1.0)) * ((base_i * c0) - (base_q * s0));
                acc_q = acc_q + (amp0 / (CHIRP_REF_AMPL * 1.0)) * ((base_i * s0) + (base_q * c0));
            end

            if (amp1 != 0 && n >= delay1 && (n - delay1) < FFT_SAMPLES) begin
                idx = n - delay1;
                base_i = chirp_i[idx];
                base_q = chirp_q[idx];
                acc_i = acc_i + (amp1 / (CHIRP_REF_AMPL * 1.0)) * ((base_i * c1) - (base_q * s1));
                acc_q = acc_q + (amp1 / (CHIRP_REF_AMPL * 1.0)) * ((base_i * s1) + (base_q * c1));
            end

            @(posedge clk);
            adc_data_i <= sat16($rtoi(acc_i));
            adc_data_q <= sat16($rtoi(acc_q));
            long_chirp_real <= chirp_i[n];
            long_chirp_imag <= chirp_q[n];
            short_chirp_real <= chirp_i[n];
            short_chirp_imag <= chirp_q[n];
            adc_valid <= 1'b1;
        end

        @(posedge clk);
        adc_valid <= 1'b0;
        adc_data_i <= 16'sd0;
        adc_data_q <= 16'sd0;
        long_chirp_real <= 16'sd0;
        long_chirp_imag <= 16'sd0;
        short_chirp_real <= 16'sd0;
        short_chirp_imag <= 16'sd0;

        // Wait until this chirp has produced all 64 MTI-valid bins.
        timeout = 0;
        while ((mti_valid_count - mti_start) < RANGE_BINS && timeout < 3000000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
    end
endtask

task run_frame;
    input integer t0_delay;
    input integer t0_amp;
    input real    t0_dphi;
    input integer t1_delay;
    input integer t1_amp;
    input real    t1_dphi;
    integer chirp;
    real phase0;
    real phase1;
    begin
        pulse_new_frame;
        for (chirp = 0; chirp < CHIRPS_PER_FRAME; chirp = chirp + 1) begin
            chirp_counter <= chirp[5:0];
            phase0 = t0_dphi * chirp;
            phase1 = t1_dphi * chirp;
            feed_one_chirp(t0_delay, t0_amp, phase0, t1_delay, t1_amp, phase1);
            repeat (8) @(posedge clk);
        end
    end
endtask

task wait_processing_done;
    integer timeout;
    begin
        timeout = 0;
        while ((doppler_processing_active || cfar_busy) && timeout < 5000000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        repeat (200) @(posedge clk);
    end
endtask

// -----------------------------------------------------------------------------
// Main sequence
// -----------------------------------------------------------------------------
integer peak_r;
integer peak_d;
integer peak_mag_val;
integer sum_cnt;
integer cnt_range16;
integer cnt_range32;

initial begin
    $dumpfile("tb_end_to_end_physics.vcd");
    $dumpvars(0, tb_end_to_end_physics);

    pass_count = 0;
    fail_count = 0;

    $display("");
    $display("==========================================================");
    $display("  AERIS-10 End-to-End Physics Testbench");
    $display("  Chain: MF -> Decimator -> MTI -> Doppler -> CFAR");
    $display("  v=20 m/s: fd=%.1f Hz, delta_phi=%.3f rad/chirp", FD_MOVING_HZ, DELTA_PHI_MOVING);
    $display("==========================================================");

    // -----------------------------------------------------------------
    // E1: Single stationary target, MTI disabled
    // -----------------------------------------------------------------
    test_num = 1;
    $display("");
    $display("--- E1: Single stationary target, MTI disabled ---");

    apply_reset;
    clear_detections;
    mti_enable <= 1'b0;

    run_frame(TARGET_DELAY_E1, TARGET_AMPL, 0.0,
              0, 0, 0.0);
    wait_processing_done;

    find_peak_near_range(TARGET_RBIN_E1, 1, peak_r, peak_d, peak_mag_val, sum_cnt);
    $display("  E1 peak near rbin %0d: range=%0d doppler=%0d mag=%0d sum=%0d",
             TARGET_RBIN_E1, peak_r, peak_d, peak_mag_val, sum_cnt);

    check(sum_cnt > 0,
          "CFAR detects stationary target near range bin 16");
    check(abs_int(peak_r - TARGET_RBIN_E1) <= 1,
          "Detection range bin is near expected bin 16");
    check((peak_d == 0) || (peak_d == 1) || (peak_d == 15) ||
          (peak_d == 16) || (peak_d == 17) || (peak_d == 31),
          "Stationary target appears near Doppler DC bin (sub-frame 0 or 1)");

    // -----------------------------------------------------------------
    // E2: Single moving target, MTI enabled
    // -----------------------------------------------------------------
    test_num = 2;
    $display("");
    $display("--- E2: Single moving target, MTI enabled ---");

    apply_reset;
    clear_detections;
    mti_enable <= 1'b1;

    run_frame(TARGET_DELAY_E1, TARGET_AMPL, DELTA_PHI_MOVING,
              0, 0, 0.0);
    wait_processing_done;

    find_peak_near_range(TARGET_RBIN_E1, 1, peak_r, peak_d, peak_mag_val, sum_cnt);
    $display("  E2 peak near rbin %0d: range=%0d doppler=%0d mag=%0d sum=%0d",
             TARGET_RBIN_E1, peak_r, peak_d, peak_mag_val, sum_cnt);

    check(sum_cnt > 0,
          "CFAR detects moving target near range bin 16");
    check(abs_int(peak_r - TARGET_RBIN_E1) <= 1,
          "Moving target detection range is near bin 16");
    check(((peak_d >= 2) && (peak_d <= 5)) || ((peak_d >= 18) && (peak_d <= 21)),
          "Moving target Doppler bin is near expected bin ~3-4 (16-pt FFT, either sub-frame)");

    // -----------------------------------------------------------------
    // E3: Stationary target with MTI enabled should be suppressed
    // -----------------------------------------------------------------
    test_num = 3;
    $display("");
    $display("--- E3: Stationary target suppressed by MTI ---");

    apply_reset;
    clear_detections;
    mti_enable <= 1'b1;

    run_frame(TARGET_DELAY_E1, TARGET_AMPL, 0.0,
              0, 0, 0.0);
    wait_processing_done;

    cnt_range16 = range_window_count(TARGET_RBIN_E1, 1);
    $display("  E3 detections near range bin 16: %0d", cnt_range16);

    check(cnt_range16 == 0,
          "MTI suppresses stationary target (no detections near range 16)");

    // -----------------------------------------------------------------
    // E4: Multi-target (stationary + moving), MTI enabled
    // -----------------------------------------------------------------
    test_num = 4;
    $display("");
    $display("--- E4: Multi-target scenario ---");

    apply_reset;
    clear_detections;
    mti_enable <= 1'b1;

    run_frame(TARGET_DELAY_E1, TARGET_AMPL, 0.0,
              TARGET_DELAY_E4_M, TARGET_AMPL, DELTA_PHI_MOVING);
    wait_processing_done;

    cnt_range16 = range_window_count(TARGET_RBIN_E1, 1);
    cnt_range32 = range_window_count(TARGET_RBIN_E4_M, 1);
    find_peak_near_range(TARGET_RBIN_E4_M, 1, peak_r, peak_d, peak_mag_val, sum_cnt);

    $display("  E4 detections near stationary range 16: %0d", cnt_range16);
    $display("  E4 detections near moving range 32: %0d", cnt_range32);
    $display("  E4 moving peak: range=%0d doppler=%0d mag=%0d", peak_r, peak_d, peak_mag_val);

    check(cnt_range32 > 0,
          "Moving target detected near range bin 32");
    check(((peak_d >= 2) && (peak_d <= 5)) || ((peak_d >= 18) && (peak_d <= 21)),
          "Moving target appears near expected Doppler bin ~3-4 (16-pt FFT, either sub-frame)");
    check(cnt_range16 == 0,
          "Stationary target near range bin 16 is MTI-suppressed");

    // -----------------------------------------------------------------
    // Summary
    // -----------------------------------------------------------------
    $display("");
    $display("==========================================================");
    $display("  END-TO-END PHYSICS TESTBENCH RESULTS");
    $display("  Passed: %0d  Failed: %0d  Total: %0d",
             pass_count, fail_count, pass_count + fail_count);
    $display("==========================================================");
    $display("");

    $finish;
end

// -----------------------------------------------------------------------------
// Timeout watchdog (500 ms)
// -----------------------------------------------------------------------------
initial begin
    #500_000_000;
    $display("[TIMEOUT] Simulation exceeded 500ms watchdog");
    $finish;
end

endmodule
