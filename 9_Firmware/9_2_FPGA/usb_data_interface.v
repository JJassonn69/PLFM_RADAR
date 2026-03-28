module usb_data_interface (
    input wire clk,              // Main clock (100MHz recommended)
    input wire reset_n,
    input wire ft601_reset_n,    // FT601-domain synchronized reset
    
    // Radar data inputs — Range (per-chirp bin stream)
    input wire [31:0] range_profile,
    input wire range_valid,

    // Radar data inputs — Doppler (per-frame cell stream)
    // v9: expanded to include range/doppler bin indices and sub_frame
    input wire [15:0] doppler_real,
    input wire [15:0] doppler_imag,
    input wire doppler_valid,
    input wire [5:0]  doppler_range_bin,   // v9: which range bin [0..63]
    input wire [4:0]  doppler_doppler_bin, // v9: which Doppler bin [0..31]
    input wire        doppler_sub_frame,   // v9: 0=long PRI, 1=short PRI

    // Radar data inputs — CFAR detection reports
    // v9: expanded from 1-bit flag to full detection report
    input wire        cfar_detection,      // 1=detection, 0=no detection (backward compat)
    input wire cfar_valid,
    input wire [5:0]  cfar_detect_range,   // v9: range bin of detection
    input wire [4:0]  cfar_detect_doppler, // v9: Doppler bin of detection
    input wire [16:0] cfar_detect_mag,     // v9: magnitude at CUT
    input wire [16:0] cfar_detect_thr,     // v9: CFAR threshold
    
    // FT601 Interface (Slave FIFO mode)
    // Data bus
    inout wire [31:0] ft601_data,    // 32-bit bidirectional data bus
    output reg [3:0] ft601_be,       // Byte enable (4 lanes for 32-bit mode)
    
    // Control signals
    output reg ft601_txe_n,          // Transmit enable (active low)
    output reg ft601_rxf_n,          // Receive enable (active low)
    input wire ft601_txe,             // Transmit FIFO empty
    input wire ft601_rxf,             // Receive FIFO full
    output reg ft601_wr_n,            // Write strobe (active low)
    output reg ft601_rd_n,            // Read strobe (active low)
    output reg ft601_oe_n,            // Output enable (active low)
    output reg ft601_siwu_n,          // Send immediate / Wakeup
    
    // FIFO flags
    input wire [1:0] ft601_srb,       // Selected read buffer
    input wire [1:0] ft601_swb,       // Selected write buffer
    
    // Clock
    output wire ft601_clk_out,        // Output clock to FT601 (forwarded via ODDR)
    input wire ft601_clk_in,          // Clock from FT601 (60/100MHz)
    
    // ========== HOST COMMAND OUTPUTS (Gap 4: USB Read Path) ==========
    output reg [31:0] cmd_data,      // Last received command word
    output reg cmd_valid,            // Pulse: new command received (ft601_clk domain)
    output reg [7:0] cmd_opcode,     // Decoded opcode for convenience
    output reg [7:0] cmd_addr,       // Decoded register address
    output reg [15:0] cmd_value,     // Decoded value

    // Gap 2: Stream control input (clk_100m domain, CDC'd internally)
    // Bit 0 = range stream enable
    // Bit 1 = doppler stream enable
    // Bit 2 = cfar/detection stream enable
    input wire [2:0] stream_control,

    // Gap 2: Status readback inputs (clk_100m domain, CDC'd internally)
    input wire status_request,
    input wire [15:0] status_cfar_threshold,
    input wire [2:0]  status_stream_ctrl,
    input wire [1:0]  status_radar_mode,
    input wire [15:0] status_long_chirp,
    input wire [15:0] status_long_listen,
    input wire [15:0] status_guard,
    input wire [15:0] status_short_chirp,
    input wire [15:0] status_short_listen,
    input wire [5:0]  status_chirps_per_elev,
    input wire [1:0]  status_range_mode,

    // Self-test status readback
    input wire [4:0]  status_self_test_flags,
    input wire [7:0]  status_self_test_detail,
    input wire        status_self_test_busy,

    // ========== DEBUG INSTRUMENTATION (v7b) ==========
    output wire [15:0] dbg_wr_strobes,
    output wire [15:0] dbg_txe_blocks,
    output wire [15:0] dbg_pkt_starts,
    output wire [15:0] dbg_pkt_completions,

    // ========== CFAR DEBUG COUNTERS (v9c) ==========
    input wire [15:0] cfar_dbg_cells_processed,
    input wire [7:0]  cfar_dbg_cols_completed,
    input wire [15:0] cfar_dbg_valid_count,
    input wire [15:0] cfar_detect_count,

    // ========== WRITE IDLE OUTPUT (v8b) ==========
    output wire write_idle
);

// ============================================================================
// USB packet header markers
// ============================================================================
localparam HEADER_RANGE   = 8'hAA;  // Range data packet (backward compat)
localparam HEADER_DOPPLER = 8'hCC;  // v9: Doppler cell packet
localparam HEADER_CFAR    = 8'hDD;  // v9: CFAR detection report packet
localparam HEADER_STATUS  = 8'hBB;  // Status response packet
localparam FOOTER         = 8'h55;

// FT601 configuration
localparam FT601_DATA_WIDTH = 32;
localparam FT601_BURST_SIZE = 512;

// ============================================================================
// WRITE FSM State definitions (Verilog-2001 compatible)
// ============================================================================
// v9: expanded to 4 bits to accommodate new Doppler/CFAR packet states.
// Packet types are now independent — each has its own header/data/footer path.
localparam [3:0] IDLE                 = 4'd0,
                 SEND_RANGE_HDR       = 4'd1,   // Send 0xAA header for range packet
                 SEND_RANGE_DATA      = 4'd2,   // Send 4 range data words
                 SEND_DOPPLER_HDR     = 4'd3,   // v9: Send 0xCC header for Doppler packet
                 SEND_DOPPLER_DATA    = 4'd4,   // v9: Send Doppler cell data words
                 SEND_CFAR_HDR        = 4'd5,   // v9: Send 0xDD header for CFAR packet
                 SEND_CFAR_DATA       = 4'd6,   // v9: Send CFAR detection report words
                 SEND_FOOTER          = 4'd7,   // Send 0x55 footer (shared by all types)
                 WAIT_ACK             = 4'd8,   // 1-cycle completion handshake
                 SEND_STATUS          = 4'd9;   // Status readback (0xBB header)

(* fsm_encoding = "none" *) reg [3:0] current_state;
reg [7:0] byte_counter;
reg [31:0] data_buffer;
reg [31:0] ft601_data_out;
reg ft601_data_oe;

// ========== DEBUG COUNTERS (v7b) ==========
reg [15:0] dbg_wr_strobes_r;
reg [15:0] dbg_txe_blocks_r;
reg [15:0] dbg_pkt_starts_r;
reg [15:0] dbg_pkt_completions_r;

assign dbg_wr_strobes     = dbg_wr_strobes_r;
assign dbg_txe_blocks     = dbg_txe_blocks_r;
assign dbg_pkt_starts     = dbg_pkt_starts_r;
assign dbg_pkt_completions = dbg_pkt_completions_r;

// ============================================================================
// READ FSM State definitions (Gap 4: USB Read Path)
// ============================================================================
localparam [2:0] RD_IDLE      = 3'd0,
                 RD_OE_ASSERT = 3'd1,
                 RD_READING   = 3'd2,
                 RD_DEASSERT  = 3'd3,
                 RD_PROCESS   = 3'd4;

(* fsm_encoding = "none" *) reg [2:0] read_state;
reg [31:0] rx_data_captured;

// ========== POST-POR STARTUP LOCKOUT ==========
reg [7:0] startup_lockout_ctr;
wire startup_lockout_active = (startup_lockout_ctr != 8'hFF);

// ========== SEND_STATUS WATCHDOG ==========
reg [15:0] status_watchdog_ctr;

// ========== INPUT CAPTURE REGISTERS ==========
// NOTE: In the playback wrapper, clk and ft601_clk_in are the SAME clock
// (both driven by ft601_clk_in from the FT601 chip). The 2-stage CDC
// synchronizers that were here in v9 are NOT needed for same-clock operation
// and caused a subtle bug: the 2-cycle latency in the valid→pending path
// allowed the pop FSM's POP_WAIT timeout (100 cycles) to race against the
// USB FSM's consumption of the pending flag, resulting in lost CFAR packets
// on hardware (only 4 of ~1300 delivered). See v9b fix notes.
//
// For future dual-clock designs (e.g., real ADC at different rate), the CDC
// synchronizers can be re-added behind a `define or parameter.

// v9b: No holding registers needed — data captured directly from inputs
// in the ft601_clk_in domain (same clock as the wrapper).

// v8b: Write-idle indicator
assign write_idle = (current_state == IDLE) && !startup_lockout_active &&
                    (read_state == RD_IDLE);

// v9b: Direct data capture — same clock domain, no CDC needed.
// Data is captured directly from inputs on the valid pulse.
// The _cap registers are what the write FSM reads to build packets.
reg [31:0] range_profile_cap;
reg [15:0] doppler_real_cap;
reg [15:0] doppler_imag_cap;
reg [5:0]  doppler_range_bin_cap;     // v9
reg [4:0]  doppler_doppler_bin_cap;   // v9
reg        doppler_sub_frame_cap;     // v9
reg        cfar_detection_cap;
reg [5:0]  cfar_range_cap;            // v9
reg [4:0]  cfar_doppler_cap;          // v9
reg [16:0] cfar_mag_cap;              // v9
reg [16:0] cfar_thr_cap;              // v9

// v9: Data-pending flags — set on valid edge, cleared when consumed.
// Now used as independent packet triggers (not piggybacked on range).
reg range_data_pending;
reg doppler_data_pending;
reg cfar_data_pending;

// Gap 2: CDC for stream_control
(* ASYNC_REG = "TRUE" *) reg [2:0] stream_ctrl_sync_0;
(* ASYNC_REG = "TRUE" *) reg [2:0] stream_ctrl_sync_1;
wire stream_range_en   = stream_ctrl_sync_1[0];
wire stream_doppler_en = stream_ctrl_sync_1[1];
wire stream_cfar_en    = stream_ctrl_sync_1[2];

// Status request — direct capture (same clock domain in playback wrapper).
// For dual-clock designs, this would need toggle-based CDC.
reg status_req_prev;
wire status_req_edge;
assign status_req_edge = status_request && !status_req_prev;

reg status_req_pending;
reg status_busy;

// Status snapshot
reg [31:0] status_words [0:7];
reg [3:0] status_word_idx;

// v9b: Direct capture — valid signals are used directly (same clock domain).
// No synchronizer needed. Data is captured on the valid pulse, pending flag
// is set immediately, and the FSM picks it up on the next cycle.

// FT601-domain capture always block (v9b: simplified, no CDC)
always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n) begin
        range_profile_cap  <= 32'd0;
        doppler_real_cap   <= 16'd0;
        doppler_imag_cap   <= 16'd0;
        doppler_range_bin_cap  <= 6'd0;
        doppler_doppler_bin_cap <= 5'd0;
        doppler_sub_frame_cap  <= 1'b0;
        cfar_detection_cap <= 1'b0;
        cfar_range_cap     <= 6'd0;
        cfar_doppler_cap   <= 5'd0;
        cfar_mag_cap       <= 17'd0;
        cfar_thr_cap       <= 17'd0;
        stream_ctrl_sync_0 <= 3'b000;
        stream_ctrl_sync_1 <= 3'b000;
        status_req_prev    <= 1'b0;
    end else begin
        // Stream control CDC (2-stage) — kept because stream_control
        // is written by the command decoder which could be in a different
        // timing path. Harmless to keep.
        stream_ctrl_sync_0 <= stream_control;
        stream_ctrl_sync_1 <= stream_ctrl_sync_0;

        // Status request edge detection (direct, same clock)
        status_req_prev <= status_request;

        // Capture status snapshot when request arrives
        if (status_req_edge) begin
            status_words[0] <= {8'hFF, 3'b000, status_radar_mode,
                                5'b00000, status_stream_ctrl,
                                status_cfar_threshold};
            status_words[1] <= {status_long_chirp, status_long_listen};
            status_words[2] <= {status_guard, status_short_chirp};
            status_words[3] <= {status_short_listen, 10'd0, status_chirps_per_elev};
            status_words[4] <= {30'd0, status_range_mode};
            status_words[5] <= {7'd0, status_self_test_busy,
                                8'd0, status_self_test_detail,
                                3'd0, status_self_test_flags};
            status_words[6] <= {cfar_dbg_cells_processed, cfar_dbg_cols_completed, 8'd0};
            status_words[7] <= {cfar_detect_count, cfar_dbg_valid_count};
        end

        // v9b: Direct data capture on valid pulse (no synchronizer delay)
        if (range_valid)
            range_profile_cap <= range_profile;
        if (doppler_valid) begin
            doppler_real_cap       <= doppler_real;
            doppler_imag_cap       <= doppler_imag;
            doppler_range_bin_cap  <= doppler_range_bin;
            doppler_doppler_bin_cap <= doppler_doppler_bin;
            doppler_sub_frame_cap  <= doppler_sub_frame;
        end
        if (cfar_valid) begin
            cfar_detection_cap <= cfar_detection;
            cfar_range_cap     <= cfar_detect_range;
            cfar_doppler_cap   <= cfar_detect_doppler;
            cfar_mag_cap       <= cfar_detect_mag;
            cfar_thr_cap       <= cfar_detect_thr;
        end
    end
end

// FT601 data bus direction control
assign ft601_data = ft601_data_oe ? ft601_data_out : 32'hzzzz_zzzz;

// Input pipeline registers (timing fix)
reg ft601_txe_r;
reg ft601_rxf_r;

// ============================================================================
// Main FSM (ft601_clk_in domain)
// ============================================================================
always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n) begin
        current_state <= IDLE;
        read_state <= RD_IDLE;
        byte_counter <= 0;
        ft601_data_out <= 0;
        ft601_data_oe <= 0;
        ft601_be <= 4'b1111;
        ft601_txe_n <= 1;
        ft601_rxf_n <= 1;
        ft601_wr_n <= 1;
        ft601_rd_n <= 1;
        ft601_oe_n <= 1;
        ft601_siwu_n <= 1;
        rx_data_captured <= 32'd0;
        cmd_data <= 32'd0;
        cmd_valid <= 1'b0;
        cmd_opcode <= 8'd0;
        cmd_addr <= 8'd0;
        cmd_value <= 16'd0;
        range_data_pending <= 1'b0;
        doppler_data_pending <= 1'b0;
        cfar_data_pending <= 1'b0;
        ft601_txe_r <= 1'b1;
        ft601_rxf_r <= 1'b1;
        startup_lockout_ctr <= 8'd0;
        status_watchdog_ctr <= 16'd0;
        dbg_wr_strobes_r <= 16'd0;
        dbg_txe_blocks_r <= 16'd0;
        dbg_pkt_starts_r <= 16'd0;
        dbg_pkt_completions_r <= 16'd0;
        status_word_idx <= 4'd0;
        status_req_pending <= 1'b0;
        status_busy <= 1'b0;
    end else begin
        // Default: clear one-shot signals
        cmd_valid <= 1'b0;

        // Input pipeline registers
        ft601_txe_r <= ft601_txe;
        ft601_rxf_r <= ft601_rxf;

        // Post-POR startup lockout
        if (startup_lockout_active) begin
            startup_lockout_ctr <= startup_lockout_ctr + 8'd1;
            status_req_pending <= 1'b0;
        end else begin
            if (status_req_edge && !status_busy)
                status_req_pending <= 1'b1;
        end

        // v9b: Data-pending flag management — direct capture (no CDC delay).
        // Set on valid pulse, cleared when FSM consumes the data.
        // Since valid and FSM are in the same clock domain, the pending flag
        // is visible to the FSM on the NEXT cycle after it's set.
        if (range_valid)
            range_data_pending <= 1'b1;
        if (doppler_valid)
            doppler_data_pending <= 1'b1;
        if (cfar_valid)
            cfar_data_pending <= 1'b1;

        // ================================================================
        // READ FSM — host-to-FPGA command path
        // ================================================================
        case (read_state)
            RD_IDLE: begin
                if (current_state == IDLE && !ft601_rxf_r) begin
                    ft601_oe_n <= 1'b0;
                    ft601_data_oe <= 1'b0;
                    read_state <= RD_OE_ASSERT;
                end
            end

            RD_OE_ASSERT: begin
                if (!ft601_rxf_r) begin
                    ft601_rd_n <= 1'b0;
                    read_state <= RD_READING;
                end else begin
                    ft601_oe_n <= 1'b1;
                    read_state <= RD_IDLE;
                end
            end

            RD_READING: begin
                rx_data_captured <= ft601_data;
                ft601_rd_n <= 1'b1;
                read_state <= RD_DEASSERT;
            end

            RD_DEASSERT: begin
                ft601_oe_n <= 1'b1;
                read_state <= RD_PROCESS;
            end

            RD_PROCESS: begin
                cmd_data   <= rx_data_captured;
                cmd_opcode <= rx_data_captured[7:0];
                cmd_addr   <= rx_data_captured[15:8];
                cmd_value  <= {rx_data_captured[23:16], rx_data_captured[31:24]};
                cmd_valid  <= 1'b1;
                read_state <= RD_IDLE;
            end

            default: read_state <= RD_IDLE;
        endcase

        // ================================================================
        // WRITE FSM — FPGA-to-host data streaming
        // v9: Independent packet triggers for range, Doppler, and CFAR.
        // Priority: Status > Range > Doppler > CFAR
        //
        // Packet formats:
        //   Range (0xAA):   6 words (24 bytes) — backward compatible
        //     [AA] [data] [data<<8] [data<<16] [data<<24] [55]
        //   Doppler (0xCC): 4 words (16 bytes)
        //     [CC] [{range_bin[5:0], doppler_bin[4:0], sub_frame, 3'b0, I[15:0]}]
        //          [{Q[15:0], I[15:0]}] [55]
        //   CFAR (0xDD):    4 words (16 bytes)
        //     [DD] [{flag, range[5:0], doppler[4:0], 4'b0, mag[15:0]}]
        //          [{thr[16:0], 15'b0}] [55]
        //   Status (0xBB): 10 words (40 bytes) — unchanged
        // ================================================================
        if (read_state == RD_IDLE) begin
            // Per-cycle safe defaults
            ft601_wr_n <= 1'b1;
            ft601_data_oe <= 1'b0;

            // Debug counter: TXE blocks
            if (current_state != IDLE && current_state != WAIT_ACK && ft601_txe_r)
                dbg_txe_blocks_r <= dbg_txe_blocks_r + 16'd1;

            case (current_state)
                IDLE: begin
                    if (!startup_lockout_active) begin
                        // Priority 1: Status readback
                        if (status_req_pending && !status_busy && ft601_rxf_r) begin
                            current_state <= SEND_STATUS;
                            status_word_idx <= 4'd0;
                            status_watchdog_ctr <= 16'd0;
                            status_busy <= 1'b1;
                            status_req_pending <= 1'b0;
                        end
                        // Priority 2: Range data (most time-critical during streaming)
                        else if (range_data_pending && stream_range_en && ft601_rxf_r) begin
                            current_state <= SEND_RANGE_HDR;
                            byte_counter <= 0;
                            range_data_pending <= 1'b0;
                        end
                        // Priority 3: Doppler data (burst after frame accumulation)
                        else if (doppler_data_pending && stream_doppler_en && ft601_rxf_r) begin
                            current_state <= SEND_DOPPLER_HDR;
                            byte_counter <= 0;
                            doppler_data_pending <= 1'b0;
                        end
                        // Priority 4: CFAR detections (sparse, after Doppler)
                        else if (cfar_data_pending && stream_cfar_en && ft601_rxf_r) begin
                            current_state <= SEND_CFAR_HDR;
                            byte_counter <= 0;
                            cfar_data_pending <= 1'b0;
                        end
                    end
                end

                // ============================================================
                // RANGE PACKET: [0xAA] [data×4 shifted] [0x55]  = 6 words
                // Backward compatible with v7c/v8c format.
                // ============================================================
                SEND_RANGE_HDR: begin
                    if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_data_out <= {24'h000000, HEADER_RANGE};
                        ft601_be <= 4'b1111;
                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        dbg_pkt_starts_r <= dbg_pkt_starts_r + 16'd1;
                        current_state <= SEND_RANGE_DATA;
                    end
                end

                SEND_RANGE_DATA: begin
                    if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;

                        case (byte_counter)
                            0: ft601_data_out <= range_profile_cap;
                            1: ft601_data_out <= {range_profile_cap[23:0], 8'h00};
                            2: ft601_data_out <= {range_profile_cap[15:0], 16'h0000};
                            3: ft601_data_out <= {range_profile_cap[7:0], 24'h000000};
                        endcase

                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;

                        if (byte_counter == 3) begin
                            byte_counter <= 0;
                            current_state <= SEND_FOOTER;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end
                end

                // ============================================================
                // DOPPLER PACKET: [0xCC] [word1] [word2] [0x55] = 4 words
                // v9 new packet type.
                //
                // Word 1 (metadata + I):
                //   [31:26] = range_bin[5:0]
                //   [25:21] = doppler_bin[4:0]
                //   [20]    = sub_frame
                //   [19:16] = 4'b0000 (reserved)
                //   [15:0]  = I (real, signed 16-bit)
                //
                // Word 2 (Q + I for cross-check):
                //   [31:16] = Q (imag, signed 16-bit)
                //   [15:0]  = I (real, signed 16-bit, duplicate for easy parsing)
                // ============================================================
                SEND_DOPPLER_HDR: begin
                    if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_data_out <= {24'h000000, HEADER_DOPPLER};
                        ft601_be <= 4'b1111;
                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        dbg_pkt_starts_r <= dbg_pkt_starts_r + 16'd1;
                        current_state <= SEND_DOPPLER_DATA;
                    end
                end

                SEND_DOPPLER_DATA: begin
                    if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;

                        case (byte_counter)
                            0: ft601_data_out <= {doppler_range_bin_cap,
                                                  doppler_doppler_bin_cap,
                                                  doppler_sub_frame_cap,
                                                  4'b0000,
                                                  doppler_real_cap};
                            1: ft601_data_out <= {doppler_imag_cap, doppler_real_cap};
                        endcase

                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;

                        if (byte_counter == 1) begin
                            byte_counter <= 0;
                            current_state <= SEND_FOOTER;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end
                end

                // ============================================================
                // CFAR PACKET: [0xDD] [word1] [word2] [0x55] = 4 words
                // v9 new packet type. Only sent for cells where detect_flag=1.
                //
                // Word 1 (metadata + magnitude):
                //   [31]    = detect_flag (always 1 for transmitted packets)
                //   [30:25] = range_bin[5:0]
                //   [24:20] = doppler_bin[4:0]
                //   [19:17] = 3'b000 (reserved)
                //   [16:0]  = magnitude[16:0]
                //
                // Word 2 (threshold):
                //   [31:17] = 15'b0 (reserved)
                //   [16:0]  = threshold[16:0]
                // ============================================================
                SEND_CFAR_HDR: begin
                    if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_data_out <= {24'h000000, HEADER_CFAR};
                        ft601_be <= 4'b1111;
                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        dbg_pkt_starts_r <= dbg_pkt_starts_r + 16'd1;
                        current_state <= SEND_CFAR_DATA;
                    end
                end

                SEND_CFAR_DATA: begin
                    if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;

                        case (byte_counter)
                            0: ft601_data_out <= {cfar_detection_cap,
                                                  cfar_range_cap,
                                                  cfar_doppler_cap,
                                                  3'b000,
                                                  cfar_mag_cap};
                            1: ft601_data_out <= {15'b0, cfar_thr_cap};
                        endcase

                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;

                        if (byte_counter == 1) begin
                            byte_counter <= 0;
                            current_state <= SEND_FOOTER;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end
                end

                // ============================================================
                // FOOTER (shared by all packet types)
                // ============================================================
                SEND_FOOTER: begin
                    if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;
                        ft601_data_out <= {24'h000000, FOOTER};
                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        current_state <= WAIT_ACK;
                    end
                end

                // ============================================================
                // STATUS PACKET: [0xBB] [8 words] [0x55] = 10 words — unchanged
                // ============================================================
                SEND_STATUS: begin
                    status_watchdog_ctr <= status_watchdog_ctr + 16'd1;
                    if (status_watchdog_ctr == 16'hFFFF) begin
                        status_word_idx <= 4'd0;
                        status_busy <= 1'b0;
                        current_state <= IDLE;
                    end else if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;
                        case (status_word_idx)
                            4'd0: ft601_data_out <= {24'h000000, HEADER_STATUS};
                            4'd1: ft601_data_out <= status_words[0];
                            4'd2: ft601_data_out <= status_words[1];
                            4'd3: ft601_data_out <= status_words[2];
                            4'd4: ft601_data_out <= status_words[3];
                            4'd5: ft601_data_out <= status_words[4];
                            4'd6: ft601_data_out <= status_words[5];
                            4'd7: ft601_data_out <= status_words[6];
                            4'd8: ft601_data_out <= status_words[7];
                            4'd9: ft601_data_out <= {24'h000000, FOOTER};
                            default: ;
                        endcase
                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        if (status_word_idx == 4'd9) begin
                            status_word_idx <= 4'd0;
                            current_state <= WAIT_ACK;
                        end else begin
                            status_word_idx <= status_word_idx + 1;
                        end
                    end
                end

                WAIT_ACK: begin
                    dbg_pkt_completions_r <= dbg_pkt_completions_r + 16'd1;
                    status_busy <= 1'b0;
                    current_state <= IDLE;
                end
            endcase
        end
    end
end

// ============================================================================
// FT601 clock output forwarding
// ============================================================================
`ifndef SIMULATION
ODDR #(
    .DDR_CLK_EDGE("OPPOSITE_EDGE"),
    .INIT(1'b0),
    .SRTYPE("SYNC")
) oddr_ft601_clk (
    .Q(ft601_clk_out),
    .C(ft601_clk_in),
    .CE(1'b1),
    .D1(1'b1),
    .D2(1'b0),
    .R(1'b0),
    .S(1'b0)
);
`else
// Simulation: behavioral clock forwarding
reg ft601_clk_out_sim;
always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n)
        ft601_clk_out_sim <= 1'b0;
    else
        ft601_clk_out_sim <= 1'b1;
end
assign ft601_clk_out = ft601_clk_in;
`endif

endmodule
