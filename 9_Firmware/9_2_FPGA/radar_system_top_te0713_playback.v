`timescale 1ns / 1ps
//
// radar_system_top_te0713_playback.v — AERIS-10 BRAM Playback Top-Level
//
// Runs real ADI CN0566 radar data (pre-loaded in BRAM) through the full
// digital signal processing pipeline on the FPGA:
//
//   BRAM Playback → Range Bin Decimator (1024→64, peak)
//     → MTI Canceller → Doppler Processor (2×16-pt FFT)
//     → DC Notch → CFAR Detector
//
// Uses STARTUPE2 internal oscillator (~65 MHz) as sole clock.
// VIO debug core on JTAG USER scan chain 3 for control and readback.
//
// VIO Output Probes (Vivado → FPGA):
//   probe_out0 [0:0]  — playback_trigger (pulse to start playback)
//   probe_out1 [0:0]  — cfar_enable
//   probe_out2 [0:0]  — mti_enable
//   probe_out3 [2:0]  — dc_notch_width (0=off, 1..7)
//   probe_out4 [0:0]  — trigger_self_test
//
// VIO Input Probes (FPGA → Vivado):
//   probe_in0  [15:0] — detect_count (CFAR detections)
//   probe_in1  [5:0]  — detect_range (last detection range bin)
//   probe_in2  [4:0]  — detect_doppler (last detection Doppler bin)
//   probe_in3  [16:0] — detect_magnitude (last detection magnitude)
//   probe_in4  [0:0]  — detect_flag (latest detection flag)
//   probe_in5  [0:0]  — cfar_busy
//   probe_in6  [0:0]  — doppler_frame_done
//   probe_in7  [0:0]  — playback_done
//   probe_in8  [0:0]  — playback_active
//   probe_in9  [5:0]  — chirp_count
//   probe_in10 [31:0] — heartbeat_counter
//   probe_in11 [4:0]  — self_test_flags (BRAM/CIC/FFT/ARITH/ADC)
//   probe_in12 [7:0]  — self_test_detail
//   probe_in13 [0:0]  — self_test_done
//   probe_in14 [16:0] — detect_threshold (last CFAR threshold)
//   probe_in15 [7:0]  — cfar_status
//

module radar_system_top_te0713_playback (
    output wire [3:0] user_led,
    output wire [3:0] system_status
);

// =========================================================================
// Parameters
// =========================================================================
localparam [7:0] VERSION_MAJOR = 8'd0;
localparam [7:0] VERSION_MINOR = 8'd6;   // v0.6 = BRAM playback build

// =========================================================================
// Internal FPGA Configuration Oscillator (~65 MHz)
// =========================================================================
wire clk_internal;
wire clk_buf;

`ifdef SIMULATION
    reg sim_clk = 0;
    always #7.5 sim_clk = ~sim_clk; // ~66 MHz
    assign clk_internal = sim_clk;
    assign clk_buf = clk_internal;  // No BUFG in simulation
`else
    STARTUPE2 #(
        .PROG_USR("FALSE"),
        .SIM_CCLK_FREQ(0.0)
    ) startup_inst (
        .CFGCLK(),
        .CFGMCLK(clk_internal),
        .EOS(),
        .PREQ(),
        .CLK(1'b0),
        .GSR(1'b0),
        .GTS(1'b0),
        .KEYCLEARB(1'b1),
        .PACK(1'b0),
        .USRCCLKO(1'b0),
        .USRCCLKTS(1'b0),
        .USRDONEO(1'b1),
        .USRDONETS(1'b1)
    );

    BUFG bufg_clk (
        .I(clk_internal),
        .O(clk_buf)
    );
`endif

// =========================================================================
// Power-on Reset
// Synthesis: ~500ms at ~65 MHz = 33M cycles (25-bit counter, bit 24)
// Simulation: ~256 cycles (8-bit counter, bit 7) for fast startup
// =========================================================================
`ifdef SIMULATION
    reg [7:0] por_cnt = 8'd0;
    reg       reset_n = 1'b0;
    always @(posedge clk_buf) begin
        if (!por_cnt[7])
            por_cnt <= por_cnt + 8'd1;
        else
            reset_n <= 1'b1;
    end
`else
    reg [24:0] por_cnt = 25'd0;
    reg        reset_n = 1'b0;
    always @(posedge clk_buf) begin
        if (!por_cnt[24])
            por_cnt <= por_cnt + 25'd1;
        else
            reset_n <= 1'b1;
    end
`endif

// =========================================================================
// Heartbeat Counter
// =========================================================================
reg [31:0] hb_counter = 32'd0;

always @(posedge clk_buf) begin
    hb_counter <= hb_counter + 1'b1;
end

// =========================================================================
// VIO Probe Wires
// =========================================================================
// Output probes (VIO → FPGA)
wire        vio_playback_trigger;
wire        vio_cfar_enable;
wire        vio_mti_enable;
wire [2:0]  vio_dc_notch_width;
wire        vio_trigger_self_test;

// =========================================================================
// Edge Detection on VIO Outputs
// =========================================================================
reg vio_playback_d = 1'b0;
reg vio_selftest_d = 1'b0;

wire playback_trigger_pulse = vio_playback_trigger && !vio_playback_d;
wire selftest_trigger_pulse = vio_trigger_self_test && !vio_selftest_d;

always @(posedge clk_buf) begin
    vio_playback_d <= vio_playback_trigger;
    vio_selftest_d <= vio_trigger_self_test;
end

// =========================================================================
// Auto-trigger self-test on power-up
// =========================================================================
reg reset_n_d = 1'b0;
reg auto_trigger = 1'b0;

always @(posedge clk_buf) begin
    reset_n_d    <= reset_n;
    auto_trigger <= (reset_n && !reset_n_d);
end

wire self_test_trigger = auto_trigger || selftest_trigger_pulse;

// =========================================================================
// FPGA Self-Test Controller (carried over from VIO build)
// =========================================================================
wire        self_test_busy;
wire        self_test_done;
wire [4:0]  result_flags;
wire [7:0]  result_detail;
wire        capture_active;
wire [15:0] capture_data;
wire        capture_valid;

fpga_self_test self_test_inst (
    .clk(clk_buf),
    .reset_n(reset_n),
    .trigger(self_test_trigger),
    .busy(self_test_busy),
    .result_valid(self_test_done),
    .result_flags(result_flags),
    .result_detail(result_detail),
    .adc_data_in(16'd0),
    .adc_valid_in(1'b0),
    .capture_active(capture_active),
    .capture_data(capture_data),
    .capture_valid(capture_valid)
);

// Latch self-test results
reg [4:0] result_flags_latched = 5'd0;
reg [7:0] result_detail_latched = 8'd0;
reg       test_done_latched = 1'b0;

always @(posedge clk_buf) begin
    if (!reset_n || self_test_trigger) begin
        test_done_latched <= 1'b0;
    end else if (self_test_done) begin
        test_done_latched     <= 1'b1;
        result_flags_latched  <= result_flags;
        result_detail_latched <= result_detail;
    end
end

// =========================================================================
// BRAM Playback Engine
// =========================================================================
wire [31:0] pb_data_out;
wire        pb_data_valid;
wire        pb_new_chirp_frame;
wire        pb_playback_done;
wire        pb_playback_active;
wire [5:0]  pb_chirp_count;

bram_playback #(
    .NUM_CHIRPS(32),
    .SAMPLES_PER_CHIRP(1024),
    .INTER_CHIRP_GAP(200),
    .FRAME_START_GAP(4)
) playback_inst (
    .clk(clk_buf),
    .reset_n(reset_n),
    .playback_start(playback_trigger_pulse),
    .data_out(pb_data_out),
    .data_valid(pb_data_valid),
    .new_chirp_frame(pb_new_chirp_frame),
    .playback_done(pb_playback_done),
    .playback_active(pb_playback_active),
    .chirp_count(pb_chirp_count)
);

// Unpack I/Q from playback data
wire signed [15:0] pb_i = pb_data_out[15:0];
wire signed [15:0] pb_q = pb_data_out[31:16];

// =========================================================================
// Range Bin Decimator (1024 → 64, peak detection mode)
// =========================================================================
wire signed [15:0] decim_i_out;
wire signed [15:0] decim_q_out;
wire               decim_valid_out;
wire [5:0]         decim_bin_index;

range_bin_decimator #(
    .INPUT_BINS(1024),
    .OUTPUT_BINS(64),
    .DECIMATION_FACTOR(16)
) range_decim (
    .clk(clk_buf),
    .reset_n(reset_n),
    .range_i_in(pb_i),
    .range_q_in(pb_q),
    .range_valid_in(pb_data_valid),
    .range_i_out(decim_i_out),
    .range_q_out(decim_q_out),
    .range_valid_out(decim_valid_out),
    .range_bin_index(decim_bin_index),
    .decimation_mode(2'b01),    // Peak detection mode
    .start_bin(10'd0),
    .watchdog_timeout()
);

// =========================================================================
// MTI Canceller (64 range bins, 2-pulse clutter cancellation)
// =========================================================================
wire signed [15:0] mti_i_out;
wire signed [15:0] mti_q_out;
wire               mti_valid_out;
wire [5:0]         mti_bin_out;

mti_canceller #(
    .NUM_RANGE_BINS(64),
    .DATA_WIDTH(16)
) mti_inst (
    .clk(clk_buf),
    .reset_n(reset_n),
    .range_i_in(decim_i_out),
    .range_q_in(decim_q_out),
    .range_valid_in(decim_valid_out),
    .range_bin_in(decim_bin_index),
    .range_i_out(mti_i_out),
    .range_q_out(mti_q_out),
    .range_valid_out(mti_valid_out),
    .range_bin_out(mti_bin_out),
    .mti_enable(vio_mti_enable),
    .mti_first_chirp()
);

// =========================================================================
// Doppler Processor (dual 16-pt FFT, 64 range × 32 chirps)
// =========================================================================
wire [31:0] range_data_32bit = {mti_q_out, mti_i_out};
wire        range_data_valid = mti_valid_out;

wire [31:0] doppler_output;
wire        doppler_valid;
wire [4:0]  doppler_bin;
wire [5:0]  doppler_range_bin;
wire        doppler_sub_frame;
wire        doppler_processing;
wire        doppler_frame_done;
wire [3:0]  doppler_status;

doppler_processor_optimized #(
    .DOPPLER_FFT_SIZE(16),
    .RANGE_BINS(64),
    .CHIRPS_PER_FRAME(32),
    .CHIRPS_PER_SUBFRAME(16)
) doppler_proc (
    .clk(clk_buf),
    .reset_n(reset_n),
    .range_data(range_data_32bit),
    .data_valid(range_data_valid),
    .new_chirp_frame(pb_new_chirp_frame),
    .doppler_output(doppler_output),
    .doppler_valid(doppler_valid),
    .doppler_bin(doppler_bin),
    .range_bin(doppler_range_bin),
    .sub_frame(doppler_sub_frame),
    .processing_active(doppler_processing),
    .frame_complete(doppler_frame_done),
    .status(doppler_status)
);

// =========================================================================
// DC Notch Filter (post-Doppler, pre-CFAR)
// =========================================================================
// Zeros Doppler bins within ±dc_notch_width of DC in both sub-frames.
// Sub-frame 0: DC at bin 0; Sub-frame 1: DC at bin 16.
// Within each 16-bin sub-frame, negative Doppler wraps to bins 15,14,...
wire dc_notch_active;
wire [3:0] bin_in_subframe = doppler_bin[3:0];

assign dc_notch_active = (vio_dc_notch_width != 3'd0) &&
                          (bin_in_subframe < {1'b0, vio_dc_notch_width} ||
                           bin_in_subframe > (4'd15 - {1'b0, vio_dc_notch_width} + 4'd1));

wire [31:0] notched_doppler_data  = dc_notch_active ? 32'd0 : doppler_output;
wire        notched_doppler_valid = doppler_valid;
wire [4:0]  notched_doppler_bin   = doppler_bin;
wire [5:0]  notched_range_bin     = doppler_range_bin;

// =========================================================================
// CFAR Detector (CA/GO/SO-CFAR, 64 range × 32 Doppler)
// =========================================================================
wire        cfar_detect_flag;
wire        cfar_detect_valid;
wire [5:0]  cfar_detect_range;
wire [4:0]  cfar_detect_doppler;
wire [16:0] cfar_detect_magnitude;
wire [16:0] cfar_detect_threshold;
wire [15:0] cfar_detect_count;
wire        cfar_busy_w;
wire [7:0]  cfar_status_w;

cfar_ca #(
    .NUM_RANGE_BINS(64),
    .NUM_DOPPLER_BINS(32)
) cfar_inst (
    .clk(clk_buf),
    .reset_n(reset_n),

    // Doppler data (DC-notch filtered)
    .doppler_data(notched_doppler_data),
    .doppler_valid(notched_doppler_valid),
    .doppler_bin_in(notched_doppler_bin),
    .range_bin_in(notched_range_bin),
    .frame_complete(doppler_frame_done),

    // Configuration — sensible defaults for real data
    .cfg_guard_cells(4'd2),
    .cfg_train_cells(5'd8),
    .cfg_alpha(8'h30),           // Q4.4 = 3.0
    .cfg_cfar_mode(2'd0),       // CA-CFAR
    .cfg_cfar_enable(vio_cfar_enable),
    .cfg_simple_threshold(16'd500),

    // Detection outputs
    .detect_flag(cfar_detect_flag),
    .detect_valid(cfar_detect_valid),
    .detect_range(cfar_detect_range),
    .detect_doppler(cfar_detect_doppler),
    .detect_magnitude(cfar_detect_magnitude),
    .detect_threshold(cfar_detect_threshold),

    // Status
    .detect_count(cfar_detect_count),
    .cfar_busy(cfar_busy_w),
    .cfar_status(cfar_status_w)
);

// =========================================================================
// Latch latest detection for VIO readback
// =========================================================================
reg [5:0]  last_detect_range    = 6'd0;
reg [4:0]  last_detect_doppler  = 5'd0;
reg [16:0] last_detect_mag      = 17'd0;
reg [16:0] last_detect_thr      = 17'd0;
reg        last_detect_flag     = 1'b0;

always @(posedge clk_buf) begin
    if (!reset_n || playback_trigger_pulse) begin
        last_detect_range   <= 6'd0;
        last_detect_doppler <= 5'd0;
        last_detect_mag     <= 17'd0;
        last_detect_thr     <= 17'd0;
        last_detect_flag    <= 1'b0;
    end else if (cfar_detect_valid && cfar_detect_flag) begin
        last_detect_range   <= cfar_detect_range;
        last_detect_doppler <= cfar_detect_doppler;
        last_detect_mag     <= cfar_detect_magnitude;
        last_detect_thr     <= cfar_detect_threshold;
        last_detect_flag    <= 1'b1;
    end
end

// =========================================================================
// VIO Debug Core (synthesis only — stubbed for simulation)
// =========================================================================
`ifdef SIMULATION
    reg sim_vio_playback_trigger = 1'b0;
    reg sim_vio_cfar_enable      = 1'b0;
    reg sim_vio_mti_enable       = 1'b0;
    reg [2:0] sim_vio_dc_notch   = 3'd0;
    reg sim_vio_selftest         = 1'b0;

    assign vio_playback_trigger  = sim_vio_playback_trigger;
    assign vio_cfar_enable       = sim_vio_cfar_enable;
    assign vio_mti_enable        = sim_vio_mti_enable;
    assign vio_dc_notch_width    = sim_vio_dc_notch;
    assign vio_trigger_self_test = sim_vio_selftest;
`else
    vio_playback vio_debug_inst (
        .clk         (clk_buf),
        // Input probes: FPGA → Vivado (16 probes)
        .probe_in0   (cfar_detect_count),       // [15:0]  detect count
        .probe_in1   (last_detect_range),       // [5:0]   last detect range
        .probe_in2   (last_detect_doppler),     // [4:0]   last detect doppler
        .probe_in3   (last_detect_mag),         // [16:0]  last detect magnitude
        .probe_in4   (last_detect_flag),        // [0:0]   detect flag
        .probe_in5   (cfar_busy_w),             // [0:0]   cfar busy
        .probe_in6   (doppler_frame_done),      // [0:0]   doppler frame done
        .probe_in7   (pb_playback_done),        // [0:0]   playback done
        .probe_in8   (pb_playback_active),      // [0:0]   playback active
        .probe_in9   (pb_chirp_count),          // [5:0]   chirp count
        .probe_in10  (hb_counter),              // [31:0]  heartbeat
        .probe_in11  (result_flags_latched),    // [4:0]   self-test flags
        .probe_in12  (result_detail_latched),   // [7:0]   self-test detail
        .probe_in13  (test_done_latched),       // [0:0]   self-test done
        .probe_in14  (last_detect_thr),         // [16:0]  detect threshold
        .probe_in15  (cfar_status_w),           // [7:0]   cfar status
        // Output probes: Vivado → FPGA (5 probes)
        .probe_out0  (vio_playback_trigger),    // [0:0]   start playback
        .probe_out1  (vio_cfar_enable),         // [0:0]   cfar enable
        .probe_out2  (vio_mti_enable),          // [0:0]   mti enable
        .probe_out3  (vio_dc_notch_width),      // [2:0]   dc notch width
        .probe_out4  (vio_trigger_self_test)    // [0:0]   trigger self-test
    );
`endif

// =========================================================================
// LED Outputs
// =========================================================================
assign user_led[0] = hb_counter[24];       // ~1 Hz heartbeat
assign user_led[1] = pb_playback_active;   // Lit during playback
assign user_led[2] = last_detect_flag;     // Lit if detection occurred
assign user_led[3] = hb_counter[26];       // Slow blink

// =========================================================================
// Status Outputs
// =========================================================================
assign system_status[0] = pb_playback_active;
assign system_status[1] = pb_playback_done;
assign system_status[2] = cfar_busy_w;
assign system_status[3] = hb_counter[26];

endmodule
