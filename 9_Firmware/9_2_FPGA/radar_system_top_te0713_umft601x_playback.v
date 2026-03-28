`timescale 1ns / 1ps
//
// radar_system_top_te0713_umft601x_playback.v — AERIS-10 Unified Playback + USB Top
//
// v9 — Full DSP pipeline with Doppler + CFAR streaming over USB
//
// Changes from v8c:
//   - Doppler FIFO (2048 entries × 44 bits) buffers full range-Doppler map
//   - CFAR FIFO (2048 entries × 46 bits) buffers detection reports
//   - Three independent pop FSMs (range, doppler, cfar)
//   - USB interface gets rich Doppler data (range_bin, doppler_bin, sub_frame, I, Q)
//   - USB interface gets rich CFAR data (flag, range, doppler, magnitude, threshold)
//   - New packet types: 0xCC (Doppler), 0xDD (CFAR) alongside 0xAA (range)
//   - CFAR FIFO only stores actual detections (flag=1) to save bandwidth
//
// Data flow:
//   BRAM (real ADI CN0566 3x5 array data)
//     → Range Bin Decimator (1024→64, peak)
//     → MTI Canceller (2-pulse, configurable)
//     → Doppler Processor (2×16-pt FFT)  ─→ Doppler FIFO ─→ USB (0xCC packets)
//     → DC Notch (configurable)
//     → CFAR Detector (CA/GO/SO)         ─→ CFAR FIFO ──→ USB (0xDD packets)
//   Range data from decimator             ─→ Range FIFO ──→ USB (0xAA packets)
//
// Clock: Everything runs on ft601_clk_in (100 MHz from FT601 chip).
//
// Control: All configuration via USB commands (host register map):
//   0x02: Playback trigger
//   0x03: Detection threshold
//   0x04: Stream control [2:0] (bit0=range, bit1=doppler, bit2=cfar)
//   0x21-0x27: CFAR/MTI/DC notch configuration
//   0xFF: Status readback
//

module radar_system_top_te0713_umft601x_playback (
    input  wire        osc_50m,           // TE0713 on-board 50 MHz oscillator (U20)
    input  wire        ft601_clk_in,      // 100 MHz clock from FT601 chip
    inout  wire [31:0] ft601_data,
    output wire [3:0]  ft601_be,
    input  wire        ft601_txe,
    input  wire        ft601_rxf,
    output wire        ft601_wr_n,
    output wire        ft601_rd_n,
    output wire        ft601_oe_n,
    output wire        ft601_siwu_n,
    output wire        ft601_chip_reset_n,
    output wire        ft601_wakeup_n,
    output wire        ft601_gpio0,
    output wire        ft601_gpio1
);

// =========================================================================
// FT601 chip reset — keep HIGH (deasserted) at all times
// =========================================================================
assign ft601_chip_reset_n = 1'b1;

// =========================================================================
// FT601-domain system POR — waits for ft601_clk_in to be alive
// =========================================================================
reg [15:0] por_counter = 16'd0;
reg [31:0] hb_counter = 32'd0;

wire sys_reset_n = por_counter[15];

assign ft601_wakeup_n = 1'b1;
assign ft601_gpio0 = hb_counter[24];  // ~3 Hz heartbeat at 100 MHz
assign ft601_gpio1 = sys_reset_n;

// =========================================================================
// Host Configuration Registers (set via USB commands)
// =========================================================================
reg [2:0]  stream_control_reg = 3'b000;  // OFF until host enables
reg        status_request_reg = 1'b0;
reg        playback_trigger_reg = 1'b0;
reg [15:0] detect_threshold_reg = 16'd500;
reg [3:0]  cfar_guard_reg = 4'd2;
reg [4:0]  cfar_train_reg = 5'd8;
reg [7:0]  cfar_alpha_reg = 8'h05;       // Q4.4 = 0.3125 (~5x avg, Pfa~1e-4)
reg [1:0]  cfar_mode_reg = 2'd0;         // CA-CFAR
reg        cfar_enable_reg = 1'b0;
reg        mti_enable_reg = 1'b0;
reg [2:0]  dc_notch_width_reg = 3'd0;

// Command interface from usb_data_interface
wire [31:0] cmd_data;
wire        cmd_valid;
wire [7:0]  cmd_opcode;
wire [7:0]  cmd_addr;
wire [15:0] cmd_value;
wire        ft601_clk_out_unused;
wire        ft601_txe_n_unused;
wire        ft601_rxf_n_unused;

// Debug instrumentation
wire [15:0] dbg_wr_strobes;
wire [15:0] dbg_txe_blocks;
wire [15:0] dbg_pkt_starts;
wire [15:0] dbg_pkt_completions;

// Self-test wiring
reg         self_test_trigger = 1'b0;
wire        self_test_busy;
wire        self_test_result_valid;
wire [4:0]  self_test_result_flags;
wire [7:0]  self_test_result_detail;
wire        self_test_capture_active;
wire [15:0] self_test_capture_data;
wire        self_test_capture_valid;
reg  [4:0]  self_test_flags_latched = 5'b00000;
reg  [7:0]  self_test_detail_latched = 8'd0;

// =========================================================================
// DSP data signals — feed into USB data interface
// =========================================================================
// Range data (from range FIFO pop FSM)
reg [31:0] range_profile_usb = 32'd0;
reg        range_valid_usb = 1'b0;

// v9: Doppler data (from Doppler FIFO pop FSM)
reg [15:0] doppler_real_usb = 16'd0;
reg [15:0] doppler_imag_usb = 16'd0;
reg        doppler_valid_usb = 1'b0;
reg [5:0]  doppler_range_bin_usb = 6'd0;
reg [4:0]  doppler_doppler_bin_usb = 5'd0;
reg        doppler_sub_frame_usb = 1'b0;

// v9: CFAR data (from CFAR FIFO pop FSM)
reg        cfar_detection_usb = 1'b0;
reg        cfar_valid_usb = 1'b0;
reg [5:0]  cfar_range_usb = 6'd0;
reg [4:0]  cfar_doppler_usb = 5'd0;
reg [16:0] cfar_mag_usb = 17'd0;
reg [16:0] cfar_thr_usb = 17'd0;

// v8b: write_idle from USB interface — HIGH when FSM can accept new data
wire usb_write_idle;

// v9d: Pending flag handshake outputs from USB interface
// Pop FSMs must check pending=0 before popping next entry, to prevent
// overwriting captured data before the USB FSM has consumed it.
wire usb_range_pending;
wire usb_doppler_pending;
wire usb_cfar_pending;

// =========================================================================
// RANGE DATA FIFO (v8b: 2048 entries × 32 bits = 8 KB BRAM)
// =========================================================================
localparam RANGE_FIFO_DEPTH = 2048;
localparam RANGE_FIFO_AW = 11;  // log2(2048)

reg [31:0] range_fifo [0:RANGE_FIFO_DEPTH-1];
reg [RANGE_FIFO_AW:0] range_fifo_wr = 0;
reg [RANGE_FIFO_AW:0] range_fifo_rd = 0;

wire range_fifo_empty = (range_fifo_wr == range_fifo_rd);
wire range_fifo_full  = (range_fifo_wr[RANGE_FIFO_AW] != range_fifo_rd[RANGE_FIFO_AW]) &&
                        (range_fifo_wr[RANGE_FIFO_AW-1:0] == range_fifo_rd[RANGE_FIFO_AW-1:0]);
wire [31:0] range_fifo_dout = range_fifo[range_fifo_rd[RANGE_FIFO_AW-1:0]];

reg [15:0] range_overflow_count = 16'd0;

// v9: Range pop FSM (same handshake as v8b but now one of three independent pop FSMs)
localparam [1:0] POP_IDLE = 2'd0,
                 POP_WAIT = 2'd1,
                 POP_DONE = 2'd2;
reg [1:0] range_pop_state = POP_IDLE;
reg [15:0] range_pop_timeout = 16'd0;

// =========================================================================
// v9: DOPPLER DATA FIFO (2048 entries × 44 bits)
// =========================================================================
// Stores the full range-Doppler map output from the Doppler processor.
// Each entry: {sub_frame[0], doppler_bin[4:0], range_bin[5:0], Q[15:0], I[15:0]}
// = 1 + 5 + 6 + 16 + 16 = 44 bits. We use 48-bit entries for clean BRAM mapping.
//
localparam DOPPLER_FIFO_DEPTH = 2048;
localparam DOPPLER_FIFO_AW = 11;
localparam DOPPLER_FIFO_W = 48;  // Rounded up from 44 for BRAM alignment

reg [DOPPLER_FIFO_W-1:0] doppler_fifo [0:DOPPLER_FIFO_DEPTH-1];
reg [DOPPLER_FIFO_AW:0] doppler_fifo_wr = 0;
reg [DOPPLER_FIFO_AW:0] doppler_fifo_rd = 0;

wire doppler_fifo_empty = (doppler_fifo_wr == doppler_fifo_rd);
wire doppler_fifo_full  = (doppler_fifo_wr[DOPPLER_FIFO_AW] != doppler_fifo_rd[DOPPLER_FIFO_AW]) &&
                          (doppler_fifo_wr[DOPPLER_FIFO_AW-1:0] == doppler_fifo_rd[DOPPLER_FIFO_AW-1:0]);
wire [DOPPLER_FIFO_W-1:0] doppler_fifo_dout = doppler_fifo[doppler_fifo_rd[DOPPLER_FIFO_AW-1:0]];

reg [15:0] doppler_overflow_count = 16'd0;

// Doppler pop FSM
reg [1:0] doppler_pop_state = POP_IDLE;
reg [15:0] doppler_pop_timeout = 16'd0;

// =========================================================================
// v9: CFAR DETECTION FIFO (2048 entries × 46 bits)
// =========================================================================
// Only stores actual detections (detect_flag=1) to save bandwidth.
// Each entry: {detect_flag[0], range[5:0], doppler[4:0], magnitude[16:0], threshold[16:0]}
// = 1 + 6 + 5 + 17 + 17 = 46 bits. Use 48-bit entries.
//
// v9a: Increased from 128 to 2048 to prevent overflow — CFAR produces ~1300
// detections per frame with real radar data, which overflowed the 128-entry FIFO.
//
localparam CFAR_FIFO_DEPTH = 2048;
localparam CFAR_FIFO_AW = 11;
localparam CFAR_FIFO_W = 48;

reg [CFAR_FIFO_W-1:0] cfar_fifo [0:CFAR_FIFO_DEPTH-1];
reg [CFAR_FIFO_AW:0] cfar_fifo_wr = 0;
reg [CFAR_FIFO_AW:0] cfar_fifo_rd = 0;

wire cfar_fifo_empty = (cfar_fifo_wr == cfar_fifo_rd);
wire cfar_fifo_full  = (cfar_fifo_wr[CFAR_FIFO_AW] != cfar_fifo_rd[CFAR_FIFO_AW]) &&
                       (cfar_fifo_wr[CFAR_FIFO_AW-1:0] == cfar_fifo_rd[CFAR_FIFO_AW-1:0]);
wire [CFAR_FIFO_W-1:0] cfar_fifo_dout = cfar_fifo[cfar_fifo_rd[CFAR_FIFO_AW-1:0]];

reg [15:0] cfar_overflow_count = 16'd0;

// CFAR pop FSM
reg [1:0] cfar_pop_state = POP_IDLE;
reg [15:0] cfar_pop_timeout = 16'd0;

// =========================================================================
// POR, heartbeat, and command decode
// =========================================================================
always @(posedge ft601_clk_in) begin
    if (!sys_reset_n) begin
        por_counter <= por_counter + 1'b1;
        hb_counter <= 32'd0;
        stream_control_reg <= 3'b000;
        status_request_reg <= 1'b0;
        playback_trigger_reg <= 1'b0;
        detect_threshold_reg <= 16'd500;
        cfar_guard_reg <= 4'd2;
        cfar_train_reg <= 5'd8;
        cfar_alpha_reg <= 8'h05;
        cfar_mode_reg <= 2'd0;
        cfar_enable_reg <= 1'b0;
        mti_enable_reg <= 1'b0;
        dc_notch_width_reg <= 3'd0;
        self_test_trigger <= 1'b0;
        self_test_flags_latched <= 5'b00000;
        self_test_detail_latched <= 8'd0;
    end else begin
        hb_counter <= hb_counter + 1'b1;

        // Default: clear one-shot signals
        status_request_reg <= 1'b0;
        playback_trigger_reg <= 1'b0;
        self_test_trigger <= 1'b0;

        // Latch self-test results
        if (self_test_result_valid) begin
            self_test_flags_latched  <= self_test_result_flags;
            self_test_detail_latched <= self_test_result_detail;
        end

        // Host command decode (full register map)
        if (cmd_valid) begin
            case (cmd_opcode)
                8'h02: playback_trigger_reg <= 1'b1;
                8'h03: detect_threshold_reg <= cmd_value;
                8'h04: stream_control_reg   <= cmd_value[2:0];
                8'h21: cfar_guard_reg       <= cmd_value[3:0];
                8'h22: cfar_train_reg       <= cmd_value[4:0];
                8'h23: cfar_alpha_reg       <= cmd_value[7:0];
                8'h24: cfar_mode_reg        <= cmd_value[1:0];
                8'h25: cfar_enable_reg      <= cmd_value[0];
                8'h26: mti_enable_reg       <= cmd_value[0];
                8'h27: dc_notch_width_reg   <= cmd_value[2:0];
                8'h30: self_test_trigger    <= 1'b1;
                8'h31: status_request_reg   <= 1'b1;
                8'hFF: status_request_reg   <= 1'b1;
                default: ;
            endcase
        end
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
    .clk(ft601_clk_in),
    .reset_n(sys_reset_n),
    .playback_start(playback_trigger_reg),
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
    .clk(ft601_clk_in),
    .reset_n(sys_reset_n),
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
    .clk(ft601_clk_in),
    .reset_n(sys_reset_n),
    .range_i_in(decim_i_out),
    .range_q_in(decim_q_out),
    .range_valid_in(decim_valid_out),
    .range_bin_in(decim_bin_index),
    .range_i_out(mti_i_out),
    .range_q_out(mti_q_out),
    .range_valid_out(mti_valid_out),
    .range_bin_out(mti_bin_out),
    .mti_enable(mti_enable_reg),
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
    .clk(ft601_clk_in),
    .reset_n(sys_reset_n),
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
wire dc_notch_active;
wire [3:0] bin_in_subframe = doppler_bin[3:0];

assign dc_notch_active = (dc_notch_width_reg != 3'd0) &&
                          (bin_in_subframe < {1'b0, dc_notch_width_reg} ||
                           bin_in_subframe > (4'd15 - {1'b0, dc_notch_width_reg} + 4'd1));

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
wire [15:0] cfar_dbg_cells;
wire [7:0]  cfar_dbg_cols;
wire [15:0] cfar_dbg_valid;

cfar_ca #(
    .NUM_RANGE_BINS(64),
    .NUM_DOPPLER_BINS(32)
) cfar_inst (
    .clk(ft601_clk_in),
    .reset_n(sys_reset_n),

    // Doppler data (DC-notch filtered)
    .doppler_data(notched_doppler_data),
    .doppler_valid(notched_doppler_valid),
    .doppler_bin_in(notched_doppler_bin),
    .range_bin_in(notched_range_bin),
    .frame_complete(doppler_frame_done),

    // Configuration from host registers
    .cfg_guard_cells(cfar_guard_reg),
    .cfg_train_cells(cfar_train_reg),
    .cfg_alpha(cfar_alpha_reg),
    .cfg_cfar_mode(cfar_mode_reg),
    .cfg_cfar_enable(cfar_enable_reg),
    .cfg_simple_threshold(detect_threshold_reg),

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
    .cfar_status(cfar_status_w),

    // Debug counters (v9c)
    .dbg_cells_processed(cfar_dbg_cells),
    .dbg_cols_completed(cfar_dbg_cols),
    .dbg_valid_count(cfar_dbg_valid)
);

// =========================================================================
// DSP → USB Data Bridge (v9: Three independent FIFO + pop FSM channels)
// =========================================================================
//
// FIFO push logic:
//   Range:   push on decim_valid_out (64 per chirp, 2048 per frame)
//   Doppler: push on doppler_valid (2048 per frame, burst after 32 chirps)
//   CFAR:    push on cfar_detect_valid && cfar_detect_flag (sparse, ~10-50 per frame)
//
// FIFO pop logic:
//   Each pop FSM waits for its FIFO non-empty AND usb_write_idle, then:
//     POP_IDLE: pop entry, pulse *_valid_usb, go to POP_WAIT
//     POP_WAIT: wait for write_idle=0 (USB FSM accepted), safety timeout
//     POP_DONE: wait for write_idle=1 (USB FSM finished packet)
//
// Priority between the three is handled by the USB FSM's IDLE state arbiter:
//   Status > Range > Doppler > CFAR
// So we gate each pop on usb_write_idle (which is only HIGH when ALL three
// pending flags in the USB FSM are clear and it's truly idle).
//
always @(posedge ft601_clk_in) begin
    if (!sys_reset_n) begin
        // Range FIFO
        range_fifo_wr <= 0;
        range_fifo_rd <= 0;
        range_profile_usb <= 32'd0;
        range_valid_usb <= 1'b0;
        range_overflow_count <= 16'd0;
        range_pop_state <= POP_IDLE;
        range_pop_timeout <= 16'd0;

        // Doppler FIFO
        doppler_fifo_wr <= 0;
        doppler_fifo_rd <= 0;
        doppler_real_usb <= 16'd0;
        doppler_imag_usb <= 16'd0;
        doppler_valid_usb <= 1'b0;
        doppler_range_bin_usb <= 6'd0;
        doppler_doppler_bin_usb <= 5'd0;
        doppler_sub_frame_usb <= 1'b0;
        doppler_overflow_count <= 16'd0;
        doppler_pop_state <= POP_IDLE;
        doppler_pop_timeout <= 16'd0;

        // CFAR FIFO
        cfar_fifo_wr <= 0;
        cfar_fifo_rd <= 0;
        cfar_detection_usb <= 1'b0;
        cfar_valid_usb <= 1'b0;
        cfar_range_usb <= 6'd0;
        cfar_doppler_usb <= 5'd0;
        cfar_mag_usb <= 17'd0;
        cfar_thr_usb <= 17'd0;
        cfar_overflow_count <= 16'd0;
        cfar_pop_state <= POP_IDLE;
        cfar_pop_timeout <= 16'd0;
    end else begin
        // Default: clear valid strobes
        range_valid_usb <= 1'b0;
        doppler_valid_usb <= 1'b0;
        cfar_valid_usb <= 1'b0;

        // =============================================================
        // RANGE FIFO PUSH: on decimated range data
        // =============================================================
        if (decim_valid_out) begin
            if (!range_fifo_full) begin
                range_fifo[range_fifo_wr[RANGE_FIFO_AW-1:0]] <= {decim_q_out, decim_i_out};
                range_fifo_wr <= range_fifo_wr + 1;
            end else begin
                range_overflow_count <= range_overflow_count + 1;
            end
        end

        // =============================================================
        // DOPPLER FIFO PUSH: on Doppler FFT output valid
        // =============================================================
        if (doppler_valid) begin
            if (!doppler_fifo_full) begin
                // Pack: {4'b0, sub_frame, doppler_bin[4:0], range_bin[5:0], Q[15:0], I[15:0]}
                doppler_fifo[doppler_fifo_wr[DOPPLER_FIFO_AW-1:0]] <=
                    {4'b0000,
                     doppler_sub_frame,
                     doppler_bin,
                     doppler_range_bin,
                     doppler_output[31:16],  // Q
                     doppler_output[15:0]};  // I
                doppler_fifo_wr <= doppler_fifo_wr + 1;
            end else begin
                doppler_overflow_count <= doppler_overflow_count + 1;
            end
        end

        // =============================================================
        // CFAR FIFO PUSH: only for actual detections (flag=1)
        // =============================================================
        if (cfar_detect_valid && cfar_detect_flag) begin
            if (!cfar_fifo_full) begin
                // Pack: {2'b0, flag, range[5:0], doppler[4:0], magnitude[16:0], threshold[16:0]}
                cfar_fifo[cfar_fifo_wr[CFAR_FIFO_AW-1:0]] <=
                    {2'b00,
                     cfar_detect_flag,
                     cfar_detect_range,
                     cfar_detect_doppler,
                     cfar_detect_magnitude,
                     cfar_detect_threshold};
                cfar_fifo_wr <= cfar_fifo_wr + 1;
            end else begin
                cfar_overflow_count <= cfar_overflow_count + 1;
            end
        end

        // =============================================================
        // RANGE POP FSM
        // v9d: Added !usb_range_pending check to prevent popping while
        // the USB FSM hasn't consumed the previous entry's pending flag.
        // =============================================================
        case (range_pop_state)
            POP_IDLE: begin
                if (!range_fifo_empty && usb_write_idle && !usb_range_pending) begin
                    range_profile_usb <= range_fifo_dout;
                    range_valid_usb <= 1'b1;
                    range_fifo_rd <= range_fifo_rd + 1;
                    range_pop_state <= POP_WAIT;
                    range_pop_timeout <= 16'd0;
                end
            end

            POP_WAIT: begin
                range_pop_timeout <= range_pop_timeout + 1;
                if (!usb_write_idle) begin
                    range_pop_state <= POP_DONE;
                end else if (range_pop_timeout >= 16'd100) begin
                    range_pop_state <= POP_IDLE;
                end
            end

            POP_DONE: begin
                if (usb_write_idle) begin
                    range_pop_state <= POP_IDLE;
                end
            end

            default: range_pop_state <= POP_IDLE;
        endcase

        // =============================================================
        // DOPPLER POP FSM
        // Only pops when range FIFO is empty (range has priority)
        // v9d: Added !usb_doppler_pending check — handshake fix
        // =============================================================
        case (doppler_pop_state)
            POP_IDLE: begin
                if (!doppler_fifo_empty && usb_write_idle && range_fifo_empty
                    && !usb_doppler_pending) begin
                    // Unpack FIFO entry
                    doppler_real_usb       <= doppler_fifo_dout[15:0];   // I
                    doppler_imag_usb       <= doppler_fifo_dout[31:16];  // Q
                    doppler_range_bin_usb  <= doppler_fifo_dout[37:32];  // range_bin
                    doppler_doppler_bin_usb <= doppler_fifo_dout[42:38]; // doppler_bin
                    doppler_sub_frame_usb  <= doppler_fifo_dout[43];     // sub_frame
                    doppler_valid_usb      <= 1'b1;
                    doppler_fifo_rd <= doppler_fifo_rd + 1;
                    doppler_pop_state <= POP_WAIT;
                    doppler_pop_timeout <= 16'd0;
                end
            end

            POP_WAIT: begin
                doppler_pop_timeout <= doppler_pop_timeout + 1;
                if (!usb_write_idle) begin
                    doppler_pop_state <= POP_DONE;
                end else if (doppler_pop_timeout >= 16'd100) begin
                    doppler_pop_state <= POP_IDLE;
                end
            end

            POP_DONE: begin
                if (usb_write_idle) begin
                    doppler_pop_state <= POP_IDLE;
                end
            end

            default: doppler_pop_state <= POP_IDLE;
        endcase

        // =============================================================
        // CFAR POP FSM
        // Only pops when both range and Doppler FIFOs are empty
        // v9d: Added !usb_cfar_pending check — handshake fix
        // =============================================================
        case (cfar_pop_state)
            POP_IDLE: begin
                if (!cfar_fifo_empty && usb_write_idle &&
                    range_fifo_empty && doppler_fifo_empty
                    && !usb_cfar_pending) begin
                    // Unpack FIFO entry
                    cfar_thr_usb     <= cfar_fifo_dout[16:0];   // threshold
                    cfar_mag_usb     <= cfar_fifo_dout[33:17];  // magnitude
                    cfar_doppler_usb <= cfar_fifo_dout[38:34];  // doppler
                    cfar_range_usb   <= cfar_fifo_dout[44:39];  // range
                    cfar_detection_usb <= cfar_fifo_dout[45];   // flag
                    cfar_valid_usb   <= 1'b1;
                    cfar_fifo_rd <= cfar_fifo_rd + 1;
                    cfar_pop_state <= POP_WAIT;
                    cfar_pop_timeout <= 16'd0;
                end
            end

            POP_WAIT: begin
                cfar_pop_timeout <= cfar_pop_timeout + 1;
                if (!usb_write_idle) begin
                    cfar_pop_state <= POP_DONE;
                end else if (cfar_pop_timeout >= 16'd100) begin
                    cfar_pop_state <= POP_IDLE;
                end
            end

            POP_DONE: begin
                if (usb_write_idle) begin
                    cfar_pop_state <= POP_IDLE;
                end
            end

            default: cfar_pop_state <= POP_IDLE;
        endcase
    end
end

// =========================================================================
// USB Data Interface (FT601 245 Sync FIFO) — v9
// =========================================================================
usb_data_interface usb_inst (
    .clk(ft601_clk_in),
    .reset_n(sys_reset_n),
    .ft601_reset_n(sys_reset_n),

    // Range data
    .range_profile(range_profile_usb),
    .range_valid(range_valid_usb),

    // Doppler data (v9: expanded)
    .doppler_real(doppler_real_usb),
    .doppler_imag(doppler_imag_usb),
    .doppler_valid(doppler_valid_usb),
    .doppler_range_bin(doppler_range_bin_usb),
    .doppler_doppler_bin(doppler_doppler_bin_usb),
    .doppler_sub_frame(doppler_sub_frame_usb),

    // CFAR data (v9: expanded)
    .cfar_detection(cfar_detection_usb),
    .cfar_valid(cfar_valid_usb),
    .cfar_detect_range(cfar_range_usb),
    .cfar_detect_doppler(cfar_doppler_usb),
    .cfar_detect_mag(cfar_mag_usb),
    .cfar_detect_thr(cfar_thr_usb),

    // FT601 physical interface
    .ft601_data(ft601_data),
    .ft601_be(ft601_be),
    .ft601_txe_n(ft601_txe_n_unused),
    .ft601_rxf_n(ft601_rxf_n_unused),
    .ft601_txe(ft601_txe),
    .ft601_rxf(ft601_rxf),
    .ft601_wr_n(ft601_wr_n),
    .ft601_rd_n(ft601_rd_n),
    .ft601_oe_n(ft601_oe_n),
    .ft601_siwu_n(ft601_siwu_n),
    .ft601_srb(2'b00),
    .ft601_swb(2'b00),
    .ft601_clk_out(ft601_clk_out_unused),
    .ft601_clk_in(ft601_clk_in),

    // Command interface
    .cmd_data(cmd_data),
    .cmd_valid(cmd_valid),
    .cmd_opcode(cmd_opcode),
    .cmd_addr(cmd_addr),
    .cmd_value(cmd_value),

    // Stream control
    .stream_control(stream_control_reg),

    // Status readback
    .status_request(status_request_reg),
    .status_cfar_threshold(detect_threshold_reg),
    .status_stream_ctrl(stream_control_reg),
    .status_radar_mode(2'b10),              // Mode 2 = BRAM playback
    .status_long_chirp(16'd3000),
    .status_long_listen(16'd13700),
    .status_guard(16'd17540),
    .status_short_chirp(16'd50),
    .status_short_listen(16'd17450),
    .status_chirps_per_elev(6'd32),
    .status_range_mode(2'b01),              // Range-only by default
    .status_self_test_flags(self_test_flags_latched),
    .status_self_test_detail(self_test_detail_latched),
    .status_self_test_busy(self_test_busy),

    // Debug
    .dbg_wr_strobes(dbg_wr_strobes),
    .dbg_txe_blocks(dbg_txe_blocks),
    .dbg_pkt_starts(dbg_pkt_starts),
    .dbg_pkt_completions(dbg_pkt_completions),

    // CFAR debug counters (v9c)
    .cfar_dbg_cells_processed(cfar_dbg_cells),
    .cfar_dbg_cols_completed(cfar_dbg_cols),
    .cfar_dbg_valid_count(cfar_dbg_valid),
    .cfar_detect_count(cfar_detect_count),

    .write_idle(usb_write_idle),

    // v9d: Pending flag handshake — Pop FSMs must wait for pending=0
    .range_pending_out(usb_range_pending),
    .doppler_pending_out(usb_doppler_pending),
    .cfar_pending_out(usb_cfar_pending)
);

// =========================================================================
// Board bring-up self-test controller
// =========================================================================
fpga_self_test self_test_inst (
    .clk(ft601_clk_in),
    .reset_n(sys_reset_n),
    .trigger(self_test_trigger),
    .busy(self_test_busy),
    .result_valid(self_test_result_valid),
    .result_flags(self_test_result_flags),
    .result_detail(self_test_result_detail),
    .adc_data_in(16'd0),
    .adc_valid_in(1'b0),
    .capture_active(self_test_capture_active),
    .capture_data(self_test_capture_data),
    .capture_valid(self_test_capture_valid)
);

endmodule
