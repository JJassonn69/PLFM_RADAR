`timescale 1ns / 1ps
//
// radar_system_top_te0713_umft601x_playback.v — AERIS-10 Unified Playback + USB Top
//
// Merges the BRAM playback DSP pipeline (from radar_system_top_te0713_playback.v)
// with the FT601 USB streaming interface (from radar_system_top_te0713_umft601x_dev.v).
//
// Data flow:
//   BRAM (real ADI CN0566 3x5 array data)
//     → Range Bin Decimator (1024→64, peak)
//     → MTI Canceller (2-pulse, configurable)
//     → Doppler Processor (2×16-pt FFT)
//     → DC Notch (configurable)
//     → CFAR Detector (CA/GO/SO, configurable)
//     → FT601 USB 3.0 Streaming → Host GUI
//
// Clock: Everything runs on ft601_clk_in (100 MHz from FT601 chip).
//        No STARTUPE2 needed — the DSP chain was designed for ~65 MHz
//        and runs fine at 100 MHz with better timing margin.
//
// Control: All configuration via USB commands (host register map):
//   0x02: Playback trigger (pulse to start/restart BRAM playback)
//   0x03: Detection threshold
//   0x04: Stream control [2:0] (range/doppler/cfar enables)
//   0x21: CFAR guard cells
//   0x22: CFAR training cells
//   0x23: CFAR alpha (Q4.4)
//   0x24: CFAR mode (0=CA, 1=GO, 2=SO)
//   0x25: CFAR enable
//   0x26: MTI enable
//   0x27: DC notch width
//   0xFF: Status readback
//
// v8 — First unified playback + USB build
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
reg [7:0]  cfar_alpha_reg = 8'h30;       // Q4.4 = 3.0
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
reg [31:0] range_profile_usb = 32'd0;
reg        range_valid_usb = 1'b0;
reg [15:0] doppler_real_usb = 16'd0;
reg [15:0] doppler_imag_usb = 16'd0;
reg        doppler_valid_usb = 1'b0;
reg        cfar_detection_usb = 1'b0;
reg        cfar_valid_usb = 1'b0;

// v8b: write_idle from USB interface — HIGH when FSM can accept new data
wire usb_write_idle;

// =========================================================================
// v8b: Range Data FIFO — buffers decimated range bins between DSP and USB
// =========================================================================
// The decimator outputs one range bin every 16 input clocks. The USB write
// FSM needs ~7 cycles per 24-byte range-only packet (header + 4 data +
// footer + WAIT_ACK), but FT601 TXE backpressure can stall it for hundreds
// of cycles. Without buffering, range_valid pulses are lost while the USB
// FSM is busy — this caused only 2/2048 packets to be received in v8.
//
// This FIFO is 2048 entries of 32 bits (8 KB BRAM). Worst case: 32 chirps
// × 64 bins = 2048 entries for a full playback. BRAM budget allows this.
//
// Push: on decim_valid_out (DSP produces data)
// Pop:  when FIFO non-empty AND usb_write_idle (USB ready for next packet)
//
localparam FIFO_DEPTH = 2048;
localparam FIFO_ADDR_W = 11;  // log2(2048)

reg [31:0] range_fifo [0:FIFO_DEPTH-1];
reg [FIFO_ADDR_W:0] fifo_wr_ptr = 0;  // Extra bit for full/empty detection
reg [FIFO_ADDR_W:0] fifo_rd_ptr = 0;

wire fifo_empty = (fifo_wr_ptr == fifo_rd_ptr);
wire fifo_full  = (fifo_wr_ptr[FIFO_ADDR_W] != fifo_rd_ptr[FIFO_ADDR_W]) &&
                  (fifo_wr_ptr[FIFO_ADDR_W-1:0] == fifo_rd_ptr[FIFO_ADDR_W-1:0]);
wire [31:0] fifo_rd_data = range_fifo[fifo_rd_ptr[FIFO_ADDR_W-1:0]];

// Debug: FIFO overflow counter (visible in range_mode field of status)
reg [15:0] fifo_overflow_count = 16'd0;

// v8b: FIFO pop handshake FSM
// We can't just pop whenever write_idle is HIGH, because the USB FSM
// takes 2+ cycles to see the range_valid edge (CDC sync chain) and
// leave IDLE. During that window, we'd pop again and corrupt data.
// Instead, we use a 3-state handshake:
//   POP_IDLE → pop one entry, pulse range_valid_usb
//   POP_WAIT → wait for write_idle to go LOW (FSM accepted data)
//   POP_DONE → wait for write_idle to go HIGH (FSM finished packet)
localparam [1:0] POP_IDLE = 2'd0,
                 POP_WAIT = 2'd1,
                 POP_DONE = 2'd2;
reg [1:0] pop_state = POP_IDLE;
reg [15:0] pop_timeout = 16'd0;  // Safety timeout for POP_WAIT

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
        cfar_alpha_reg <= 8'h30;
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
                8'h02: playback_trigger_reg <= 1'b1;          // Trigger playback
                8'h03: detect_threshold_reg <= cmd_value;      // Detection threshold
                8'h04: stream_control_reg   <= cmd_value[2:0]; // Stream control
                8'h21: cfar_guard_reg       <= cmd_value[3:0]; // CFAR guard cells
                8'h22: cfar_train_reg       <= cmd_value[4:0]; // CFAR training cells
                8'h23: cfar_alpha_reg       <= cmd_value[7:0]; // CFAR alpha
                8'h24: cfar_mode_reg        <= cmd_value[1:0]; // CFAR mode
                8'h25: cfar_enable_reg      <= cmd_value[0];   // CFAR enable
                8'h26: mti_enable_reg       <= cmd_value[0];   // MTI enable
                8'h27: dc_notch_width_reg   <= cmd_value[2:0]; // DC notch width
                8'h30: self_test_trigger    <= 1'b1;           // Self-test trigger
                8'h31: status_request_reg   <= 1'b1;           // Status readback
                8'hFF: status_request_reg   <= 1'b1;           // Status readback
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
    .cfar_status(cfar_status_w)
);

// =========================================================================
// DSP → USB Data Bridge (v8b: FIFO-buffered range data)
// =========================================================================
// Range profile: The decimator produces 64 bins per chirp, each as a
// {Q, I} pair. We push into a FIFO on decim_valid_out, and pop when the
// USB write FSM signals it's ready (write_idle). This prevents lost
// range_valid pulses when the USB FSM is busy with a previous packet.
//
// For Doppler and CFAR: unchanged — these arrive at frame boundaries
// (much slower rate) and don't suffer from the throughput bottleneck.
//
always @(posedge ft601_clk_in) begin
    if (!sys_reset_n) begin
        fifo_wr_ptr <= 0;
        fifo_rd_ptr <= 0;
        range_profile_usb <= 32'd0;
        range_valid_usb <= 1'b0;
        doppler_real_usb <= 16'd0;
        doppler_imag_usb <= 16'd0;
        doppler_valid_usb <= 1'b0;
        cfar_detection_usb <= 1'b0;
        cfar_valid_usb <= 1'b0;
        fifo_overflow_count <= 16'd0;
        pop_state <= POP_IDLE;
        pop_timeout <= 16'd0;
    end else begin
        // Default: clear valid strobes
        range_valid_usb <= 1'b0;
        doppler_valid_usb <= 1'b0;
        cfar_valid_usb <= 1'b0;

        // --- FIFO Write: push decimated range data ---
        if (decim_valid_out) begin
            if (!fifo_full) begin
                range_fifo[fifo_wr_ptr[FIFO_ADDR_W-1:0]] <= {decim_q_out, decim_i_out};
                fifo_wr_ptr <= fifo_wr_ptr + 1;
            end else begin
                fifo_overflow_count <= fifo_overflow_count + 1;
            end
        end

        // --- FIFO Read: handshake-based pop (one packet at a time) ---
        case (pop_state)
            POP_IDLE: begin
                // Pop when FIFO has data and USB FSM is ready
                if (!fifo_empty && usb_write_idle) begin
                    range_profile_usb <= fifo_rd_data;
                    range_valid_usb <= 1'b1;
                    fifo_rd_ptr <= fifo_rd_ptr + 1;
                    pop_state <= POP_WAIT;
                    pop_timeout <= 16'd0;
                end
            end

            POP_WAIT: begin
                // Wait for USB FSM to leave IDLE (accepted our data).
                // The range_valid_usb pulse takes 2 cycles through the
                // CDC sync chain before the FSM sees it and transitions
                // out of IDLE. We wait for write_idle to go LOW.
                pop_timeout <= pop_timeout + 1;
                if (!usb_write_idle) begin
                    // FSM accepted data — wait for it to finish
                    pop_state <= POP_DONE;
                end else if (pop_timeout >= 16'd100) begin
                    // Safety: if write_idle stays HIGH for 100 cycles,
                    // the FSM probably ignored our pulse (stream disabled).
                    // Return to POP_IDLE to try again or drain FIFO.
                    pop_state <= POP_IDLE;
                end
            end

            POP_DONE: begin
                // Wait for USB FSM to return to IDLE (packet complete)
                if (usb_write_idle) begin
                    pop_state <= POP_IDLE;
                end
            end

            default: pop_state <= POP_IDLE;
        endcase

        // Doppler data → USB (unchanged, frame-rate)
        if (doppler_valid) begin
            doppler_real_usb <= doppler_output[15:0];   // I
            doppler_imag_usb <= doppler_output[31:16];  // Q
            doppler_valid_usb <= 1'b1;
        end

        // CFAR detections → USB (unchanged, frame-rate)
        if (cfar_detect_valid) begin
            cfar_detection_usb <= cfar_detect_flag;
            cfar_valid_usb <= 1'b1;
        end
    end
end

// =========================================================================
// USB Data Interface (FT601 245 Sync FIFO)
// =========================================================================
usb_data_interface usb_inst (
    .clk(ft601_clk_in),
    .reset_n(sys_reset_n),
    .ft601_reset_n(sys_reset_n),
    .range_profile(range_profile_usb),
    .range_valid(range_valid_usb),
    .doppler_real(doppler_real_usb),
    .doppler_imag(doppler_imag_usb),
    .doppler_valid(doppler_valid_usb),
    .cfar_detection(cfar_detection_usb),
    .cfar_valid(cfar_valid_usb),
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
    .cmd_data(cmd_data),
    .cmd_valid(cmd_valid),
    .cmd_opcode(cmd_opcode),
    .cmd_addr(cmd_addr),
    .cmd_value(cmd_value),
    .stream_control(stream_control_reg),
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
    .dbg_wr_strobes(dbg_wr_strobes),
    .dbg_txe_blocks(dbg_txe_blocks),
    .dbg_pkt_starts(dbg_pkt_starts),
    .dbg_pkt_completions(dbg_pkt_completions),
    .write_idle(usb_write_idle)
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
