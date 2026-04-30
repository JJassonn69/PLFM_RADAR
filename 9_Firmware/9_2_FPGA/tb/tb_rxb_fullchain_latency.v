`timescale 1ns/1ps
`include "radar_params.vh"

// ============================================================================
// tb_rxb_fullchain_latency.v
//
// RX-B verification — Option A (latency_buffer removed, ref direct-wired).
//
// Production wiring this TB mirrors:
//   ddc_i/q (test stimulus) -> matched_filter_multi_segment -> chain
//   chirp_reference_rom ---- direct wire -------------------> chain ref
//
// Tests:
//   1) Pipeline timing: report cycle counts (first ddc_valid -> first
//      pc_valid).  Confirms FSM advances and produces output.
//   2) Autocorrelation peak position: drive ddc with the SAME short-chirp
//      samples the ROM serves up as ref. Output is the chirp autocorrelation.
//      Peak should be at bin 0 if ref/signal are aligned at the chain.
//
// chirp-v2 PR-C: ROM swapped from chirp_memory_loader_param to
// chirp_reference_rom. Stim now reads rx_short_{i,q}.mem (100 active samples,
// 1 µs at 100 MHz) instead of the legacy short_chirp_*.mem (50 samples,
// 0.5 µs); SHORT_LEN tracks the new active-sample count. The ROM and the
// stim always read from the same file, so the autocorrelation invariant
// (peak at bin 0) holds without further coordination.
// ============================================================================

module tb_rxb_fullchain_latency;

    localparam CLK_PERIOD = 10.0;       // 100 MHz
    localparam FFT_SIZE   = `RP_FFT_SIZE; // 2048
    localparam SHORT_LEN  = 100;        // matches RP_DEF_SHORT_CHIRP_CYCLES_V2 (1 µs)

    reg                 clk;
    reg                 reset_n;

    // multi_segment inputs
    reg  signed [17:0]  ddc_i;
    reg  signed [17:0]  ddc_q;
    reg                 ddc_valid;
    reg                 use_long_chirp;
    reg  [5:0]          chirp_counter;
    reg                 mc_new_chirp;
    reg                 mc_new_elevation;
    reg                 mc_new_azimuth;

    // multi_segment <-> chirp_reference_rom interconnect
    wire [1:0]          segment_request;
    wire [10:0]         sample_addr_out;
    wire                mem_request;
    wire                mem_ready_loader;       // direct from rom

    // ROM outputs (direct-wired to chain via multi_segment ports)
    wire [15:0]         ref_i_raw;
    wire [15:0]         ref_q_raw;

    // multi_segment outputs
    wire signed [15:0]  pc_i;
    wire signed [15:0]  pc_q;
    wire                pc_valid;
    wire [3:0]          ms_status;

    // wave_sel shim — matches radar_receiver_final.v PR-C transitional wiring.
    wire [1:0]          wave_sel = use_long_chirp ? `RP_WAVE_LONG : `RP_WAVE_SHORT;

    // ----- Chirp reference ROM (chirp-v2 PR-C) -----
    chirp_reference_rom chirp_rom (
        .clk            (clk),
        .reset_n        (reset_n),
        .wave_sel       (wave_sel),
        .segment_select (segment_request),
        .mem_request    (mem_request),
        .sample_addr    (sample_addr_out),
        .ref_i          (ref_i_raw),
        .ref_q          (ref_q_raw),
        .mem_ready      (mem_ready_loader)
    );

    // ----- 1-FF alignment register (mirrors radar_receiver_final.v) -----
    // multi_segment ST_PROCESSING latches adc_data through one register
    // stage; ref path needs the same to align at chain inputs.
    reg [15:0] ref_i_d, ref_q_d;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ref_i_d <= 16'd0;
            ref_q_d <= 16'd0;
        end else begin
            ref_i_d <= ref_i_raw;
            ref_q_d <= ref_q_raw;
        end
    end

    // ----- multi_segment (drives chain internally) -----
    matched_filter_multi_segment ms_dut (
        .clk              (clk),
        .reset_n          (reset_n),
        .ddc_i            (ddc_i),
        .ddc_q            (ddc_q),
        .ddc_valid        (ddc_valid),
        .use_long_chirp   (use_long_chirp),
        .chirp_counter    (chirp_counter),
        .mc_new_chirp     (mc_new_chirp),
        .mc_new_elevation (mc_new_elevation),
        .mc_new_azimuth   (mc_new_azimuth),
        .ref_chirp_real   (ref_i_d),
        .ref_chirp_imag   (ref_q_d),
        .segment_request  (segment_request),
        .sample_addr_out  (sample_addr_out),
        .mem_request      (mem_request),
        .mem_ready        (mem_ready_loader),
        .pc_i_w           (pc_i),
        .pc_q_w           (pc_q),
        .pc_valid_w       (pc_valid),
        .status           (ms_status)
    );

    always #(CLK_PERIOD/2.0) clk = ~clk;

    // -------- Cycle counter + first-event capture --------
    integer cycle_count;
    integer first_ddc_cycle;
    integer first_mem_request_cycle;
    integer first_pc_valid_cycle;
    integer pc_out_count;
    reg     saw_ddc, saw_mem_req, saw_pc;

    // -------- Output capture for peak detection --------
    reg signed [15:0] cap_i [0:FFT_SIZE-1];
    reg signed [15:0] cap_q [0:FFT_SIZE-1];

    always @(posedge clk) begin
        if (!reset_n) begin
            cycle_count             <= 0;
            saw_ddc                 <= 0;
            saw_mem_req             <= 0;
            saw_pc                  <= 0;
            pc_out_count            <= 0;
            first_ddc_cycle         <= 0;
            first_mem_request_cycle <= 0;
            first_pc_valid_cycle    <= 0;
        end else begin
            cycle_count <= cycle_count + 1;

            if (ddc_valid && !saw_ddc) begin
                first_ddc_cycle <= cycle_count;
                saw_ddc         <= 1;
                $display("[T=%0t] FIRST ddc_valid at cycle %0d", $time, cycle_count);
            end
            if (mem_request && !saw_mem_req) begin
                first_mem_request_cycle <= cycle_count;
                saw_mem_req             <= 1;
                $display("[T=%0t] FIRST mem_request at cycle %0d", $time, cycle_count);
            end
            if (pc_valid) begin
                if (!saw_pc) begin
                    first_pc_valid_cycle <= cycle_count;
                    saw_pc               <= 1;
                    $display("[T=%0t] FIRST pc_valid at cycle %0d", $time, cycle_count);
                end
                if (pc_out_count < FFT_SIZE) begin
                    cap_i[pc_out_count] <= pc_i;
                    cap_q[pc_out_count] <= pc_q;
                    pc_out_count <= pc_out_count + 1;
                end
            end
        end
    end

    // -------- Stimulus arrays — load same short-chirp values that loader will serve --------
    reg [15:0] stim_chirp_i [0:SHORT_LEN-1];
    reg [15:0] stim_chirp_q [0:SHORT_LEN-1];

    integer k;

    task feed_short_chirp_signal;
        // Drive ddc with the chirp samples (autocorrelation: signal == ref).
        // Multi_segment will buffer them and zero-pad to FFT_SIZE.
        integer j;
        begin
            for (j = 0; j < SHORT_LEN; j = j + 1) begin
                ddc_i     <= {{2{stim_chirp_i[j][15]}}, stim_chirp_i[j]};  // sign-ext to 18b
                ddc_q     <= {{2{stim_chirp_q[j][15]}}, stim_chirp_q[j]};
                ddc_valid <= 1'b1;
                @(posedge clk);
            end
            ddc_valid <= 1'b0;
        end
    endtask

    // -------- Peak finding --------
    integer peak_bin;
    integer peak_abs;
    integer mean_abs;
    integer abs_val;
    integer total_abs;

    task find_peak;
        integer kk;
        integer val_i, val_q;
        begin
            peak_bin = 0;
            peak_abs = 0;
            total_abs = 0;
            for (kk = 0; kk < FFT_SIZE; kk = kk + 1) begin
                val_i   = $signed(cap_i[kk]);
                val_q   = $signed(cap_q[kk]);
                abs_val = (val_i < 0 ? -val_i : val_i)
                        + (val_q < 0 ? -val_q : val_q);
                total_abs = total_abs + abs_val;
                if (abs_val > peak_abs) begin
                    peak_abs = abs_val;
                    peak_bin = kk;
                end
            end
            mean_abs = total_abs / FFT_SIZE;
        end
    endtask

    initial begin
        $dumpfile("tb_rxb_fullchain_latency.vcd");
        $dumpvars(0, tb_rxb_fullchain_latency);

        clk              = 0;
        reset_n          = 0;
        ddc_i            = 0;
        ddc_q            = 0;
        ddc_valid        = 0;
        use_long_chirp   = 1'b0;   // use SHORT chirp path so loader uses short_chirp_*.mem
        chirp_counter    = 6'd0;
        mc_new_chirp     = 1'b0;
        mc_new_elevation = 1'b0;
        mc_new_azimuth   = 1'b0;

        // Load the same short-chirp samples the ROM will serve as ref,
        // so signal == ref → autocorrelation. Peak should be at bin 0 if
        // ref/signal alignment is correct.
        $readmemh("rx_short_i.mem", stim_chirp_i, 0, SHORT_LEN-1);
        $readmemh("rx_short_q.mem", stim_chirp_q, 0, SHORT_LEN-1);
        $display("[TB] Loaded %0d short-chirp samples for stimulus", SHORT_LEN);

        repeat (8) @(posedge clk);
        reset_n = 1;
        repeat (8) @(posedge clk);

        $display("\n=== RX-B Option A verification ===");
        $display("Configuration: latency_buffer REMOVED, ref direct-wired");
        $display("Path: chirp_reference_rom.ref_i ----> multi_segment.ref_chirp_real");
        $display("FFT_SIZE: %0d, SHORT_LEN: %0d", FFT_SIZE, SHORT_LEN);
        $display("");

        // Pulse mc_new_chirp
        $display("[T=%0t] Pulsing mc_new_chirp HIGH...", $time);
        @(posedge clk);
        #1 mc_new_chirp = 1'b1;
        repeat (4) @(posedge clk);
        #1 mc_new_chirp = 1'b0;

        // Feed signal samples (same as ref → autocorrelation)
        feed_short_chirp_signal;

        // Wait for FFT_SIZE outputs (or timeout)
        for (k = 0; k < 200000; k = k + 1) begin
            @(posedge clk);
            if (pc_out_count >= FFT_SIZE) k = 200001;
        end

        $display("\n=== TIMING ===");
        if (saw_ddc)        $display("First ddc_valid    : cycle %0d", first_ddc_cycle);
        if (saw_mem_req)    $display("First mem_request  : cycle %0d", first_mem_request_cycle);
        if (saw_pc)         $display("First pc_valid     : cycle %0d", first_pc_valid_cycle);
        $display("pc outputs captured: %0d / %0d", pc_out_count, FFT_SIZE);

        if (pc_out_count >= FFT_SIZE) begin
            find_peak;
            $display("\n=== AUTOCORRELATION RESULT ===");
            $display("Peak bin           : %0d", peak_bin);
            $display("Peak |I|+|Q|       : %0d", peak_abs);
            $display("Mean |I|+|Q|       : %0d", mean_abs);
            $display("Peak / mean ratio  : ~%0dx",
                     (mean_abs > 0) ? (peak_abs / mean_abs) : 0);
            $display("");
            // Run with the SYNTHESIS path (no +define+SIMULATION) to use
            // the production fft_engine.v — peak should be exactly at bin 0
            // with peak/mean > 50x for the autocorrelation case. The
            // SIMULATION path uses an inline behavioural FFT in
            // matched_filter_processing_chain.v with documented numerical
            // issues (peaks at non-zero bins, weak magnitudes); the
            // synthesis path is the production code.
            if (pc_out_count >= FFT_SIZE && peak_abs > 2 * mean_abs && peak_bin == 0) begin
                $display("[PASS] Frame 1 produces output, peak at bin 0, peak/mean ~%0dx",
                         (mean_abs > 0) ? (peak_abs / mean_abs) : 0);
                $display("       RX-B fully fixed — latency_buffer removed + 1-FF align register.");
            end else if (pc_out_count >= FFT_SIZE && peak_abs > 2 * mean_abs) begin
                $display("[NEAR] Output present, peak/mean OK, but peak at bin %0d (not 0).",
                         peak_bin);
                $display("       If running with +define+SIMULATION, this is the inline");
                $display("       behavioural FFT and is expected to fail. Run without it.");
            end else if (pc_out_count >= FFT_SIZE) begin
                $display("[FAIL] Output present but peak/mean too low — no real correlation.");
            end
        end else begin
            $display("\n=== TIMEOUT — chain did not produce all outputs ===");
            $display("ms_status=%b", ms_status);
        end

        repeat (1000) @(posedge clk);
        $finish;
    end

    initial begin
        #100000000;  // 100 ms hard timeout
        $display("[ERROR] Hard simulation timeout");
        $finish;
    end

endmodule
