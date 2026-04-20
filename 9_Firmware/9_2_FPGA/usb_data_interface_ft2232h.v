`timescale 1ns / 1ps

`include "radar_params.vh"

/**
 * usb_data_interface_ft2232h.v
 *
 * FT2232H USB 2.0 Hi-Speed FIFO Interface (245 Synchronous FIFO Mode)
 * Channel A only — 8-bit data bus, 60 MHz CLKOUT from FT2232H.
 *
 * BULK PER-FRAME PROTOCOL (replaces legacy per-sample 11-byte packets):
 *
 * Frame packet (FPGA→Host): variable length, up to ~35 KB
 *   Byte 0:       0xAA (frame start header)
 *   Byte 1:       Format flags {2'b0, sparse_det, mag_only, stream_cfar, stream_doppler, stream_range}
 *   Bytes 2-3:    Frame number (16-bit, MSB first)
 *   Bytes 4-5:    Range bin count (16-bit, MSB first) = 512
 *   Bytes 6-7:    Doppler bin count (16-bit, MSB first) = 32
 *
 *   [If stream_range (bit 0):]
 *     Next 1024 bytes: Range profile, 512 × 16-bit magnitude, MSB first
 *
 *   [If stream_doppler (bit 1) AND mag_only (bit 3):]
 *     Next 32768 bytes: Doppler magnitude, 512×32 × 16-bit, row-major, MSB first
 *
 *   [If stream_doppler (bit 1) AND NOT mag_only:]
 *     Next 65536 bytes: Doppler I/Q, 512×32 × 32-bit (I16,Q16), row-major, MSB first
 *
 *   [If stream_cfar (bit 2) AND NOT sparse_det (bit 4):]
 *     Next 2048 bytes: Detection flags, 512×32 bits packed into bytes, MSB-first bit order
 *
 *   [If stream_cfar (bit 2) AND sparse_det (bit 4):]
 *     Next 2 bytes: Detection count N (16-bit, MSB first)
 *     Next N×6 bytes: Each = {range_bin[16], doppler_bin[16], magnitude[16]}, MSB first
 *
 *   Last byte:    0x55 (frame end footer)
 *
 * Status packet (FPGA→Host): 26 bytes (unchanged from legacy)
 *   Byte 0:       0xBB (status header)
 *   Bytes 1-24:   6 × 32-bit status words, MSB first
 *   Byte 25:      0x55 (footer)
 *
 * Command (Host→FPGA): 4 bytes received sequentially (unchanged)
 *   Byte 0: opcode[7:0]
 *   Byte 1: addr[7:0]
 *   Byte 2: value[15:8]
 *   Byte 3: value[7:0]
 *
 * MEMORY ARCHITECTURE:
 *   - Doppler magnitude BRAM: 512×32 = 16384 entries × 16-bit = 32 KB (~14 BRAM18)
 *     Written in clk (100 MHz) domain as Doppler cells arrive.
 *     Read in ft_clk (60 MHz) domain during USB bulk transfer.
 *   - Range profile buffer: 512 × 16-bit = 1 KB (~1 BRAM18)
 *     Written in clk domain from range_valid events.
 *   - Detection flag buffer: 512×32 = 16384 bits = 2048 bytes (~1 BRAM18)
 *     Written in clk domain from cfar_valid events.
 *   - Doppler I/Q BRAM: 16384 × 32-bit = 64 KB (~28 BRAM18) — only when mag_only=0
 *     NOTE: For 50T (75 BRAM18 total), I/Q mode may not fit alongside processing
 *     chain BRAMs. Default to mag_only=1. I/Q mode is a stretch goal.
 *
 * BANDWIDTH BUDGET (mag_only=1, all streams):
 *   Header: 8 B + Range: 1024 B + Doppler: 32768 B + CFAR: 2048 B + Footer: 1 B
 *   = 35,849 bytes/frame × 178 fps = 6.38 MB/s (80% of USB 2.0 Hi-Speed 8 MB/s)
 *
 * CDC STRATEGY:
 *   - Frame data: Written to dual-port BRAM at 100 MHz, read at 60 MHz (inherently CDC-safe)
 *   - frame_ready flag: Toggle CDC (100 MHz → 60 MHz), same as status_request
 *   - stream_control: 2-stage level sync (changes infrequently)
 *   - Commands: Read FSM in ft_clk domain, output CDC'd by consumer (unchanged)
 *
 * Clock domains:
 *   clk       = 100 MHz system clock (radar data domain)
 *   ft_clk    = 60 MHz from FT2232H CLKOUT (USB FIFO domain)
 */

module usb_data_interface_ft2232h (
    input wire clk,              // Main clock (100 MHz)
    input wire reset_n,          // System reset (clk domain)
    input wire ft_reset_n,       // FT2232H-domain synchronized reset

    // Radar data inputs (clk domain)
    input wire [31:0] range_profile,           // {range_q[15:0], range_i[15:0]}
    input wire range_valid,
    input wire [15:0] doppler_real,
    input wire [15:0] doppler_imag,
    input wire doppler_valid,
    input wire cfar_detection,
    input wire cfar_valid,

    // New inputs for bulk frame protocol (clk domain)
    input wire [`RP_RANGE_BIN_BITS-1:0] range_bin_in,   // 9-bit range bin index
    input wire [4:0]                    doppler_bin_in,  // 5-bit doppler bin index
    input wire                          frame_complete,  // 1-cycle pulse from radar_receiver_final edge detector

    // FT2232H Physical Interface (245 Synchronous FIFO mode)
    inout wire [7:0] ft_data,       // 8-bit bidirectional data bus
    input wire ft_rxf_n,            // Receive FIFO not empty (active low)
    input wire ft_txe_n,            // Transmit FIFO not full (active low)
    output reg ft_rd_n,             // Read strobe (active low)
    output reg ft_wr_n,             // Write strobe (active low)
    output reg ft_oe_n,             // Output enable (active low) — bus direction
    output reg ft_siwu,             // Send Immediate / WakeUp

    // Clock from FT2232H (directly used — no ODDR forwarding needed)
    input wire ft_clk,              // 60 MHz from FT2232H CLKOUT

    // Host command outputs (ft_clk domain — CDC'd by consumer)
    output reg [31:0] cmd_data,
    output reg cmd_valid,
    output reg [7:0] cmd_opcode,
    output reg [7:0] cmd_addr,
    output reg [15:0] cmd_value,

    // Stream control input (clk domain, CDC'd internally)
    input wire [5:0] stream_control,

    // Status readback inputs (clk domain, CDC'd internally)
    input wire status_request,
    input wire [15:0] status_cfar_threshold,
    input wire [5:0]  status_stream_ctrl,
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

    // AGC status readback
    input wire [3:0]  status_agc_current_gain,
    input wire [7:0]  status_agc_peak_magnitude,
    input wire [7:0]  status_agc_saturation_count,
    input wire        status_agc_enable
);

// ============================================================================
// CONSTANTS
// ============================================================================
localparam HEADER        = 8'hAA;
localparam FOOTER        = 8'h55;
localparam STATUS_HEADER = 8'hBB;

localparam NUM_RANGE_BINS  = `RP_NUM_RANGE_BINS;   // 512
localparam NUM_DOPPLER_BINS = `RP_NUM_DOPPLER_BINS; // 32
localparam RANGE_BIN_BITS  = `RP_RANGE_BIN_BITS;    // 9
localparam FRAME_CELLS     = NUM_RANGE_BINS * NUM_DOPPLER_BINS; // 16384

// Frame header: 8 bytes (0xAA + flags + frame_num[2] + range_bins[2] + doppler_bins[2])
localparam FRAME_HDR_BYTES = 8;
// Range profile section: 512 × 2 = 1024 bytes
localparam RANGE_SECTION_BYTES = NUM_RANGE_BINS * 2;
// Doppler mag section: 16384 × 2 = 32768 bytes
localparam DOPPLER_MAG_SECTION_BYTES = FRAME_CELLS * 2;
// Detection flag section: 16384 bits = 2048 bytes
localparam DETECT_SECTION_BYTES = FRAME_CELLS / 8;

// Status packet: 26 bytes (unchanged)
localparam STATUS_PKT_LEN = 5'd26;

// ============================================================================
// WRITE FSM STATES (FPGA → Host, ft_clk domain)
// ============================================================================
localparam [3:0] WR_IDLE          = 4'd0,
                 WR_FRAME_HDR     = 4'd1,
                 WR_RANGE_DATA    = 4'd2,
                 WR_DOPPLER_DATA  = 4'd3,
                 WR_DETECT_DATA   = 4'd4,
                 WR_FRAME_FOOTER  = 4'd5,
                 WR_STATUS_SEND   = 4'd6,
                 WR_DONE          = 4'd7;

reg [3:0] wr_state;

// ============================================================================
// READ FSM STATES (Host → FPGA, ft_clk domain — unchanged from legacy)
// ============================================================================
localparam [2:0] RD_IDLE       = 3'd0,
                 RD_OE_ASSERT  = 3'd1,
                 RD_READING    = 3'd2,
                 RD_DEASSERT   = 3'd3,
                 RD_PROCESS    = 3'd4;

reg [2:0] rd_state;
reg [1:0] rd_byte_cnt;
reg [31:0] rd_shift_reg;
reg rd_cmd_complete;  // Set when all 4 bytes received (distinguishes complete from aborted)

// ============================================================================
// DATA BUS DIRECTION CONTROL
// ============================================================================
reg [7:0] ft_data_out;
reg ft_data_oe;

assign ft_data = ft_data_oe ? ft_data_out : 8'hZZ;

// ============================================================================
// FRAME BRAM — Doppler Magnitude (clk write, ft_clk read)
// ============================================================================
// Simple dual-port BRAM: port A = write (100 MHz), port B = read (60 MHz).
// Xilinx infers true dual-port BRAM from this pattern.
// Address = {range_bin[8:0], doppler_bin[4:0]} = 14 bits, 16384 entries.
// Data = 16-bit Manhattan magnitude |I| + |Q|.

(* ram_style = "block" *) reg [15:0] doppler_mag_bram [0:FRAME_CELLS-1];

// Write port (clk domain)
reg [13:0] mag_wr_addr;
reg [15:0] mag_wr_data;
reg        mag_wr_en;

always @(posedge clk) begin
    if (mag_wr_en)
        doppler_mag_bram[mag_wr_addr] <= mag_wr_data;
end

// Read port (ft_clk domain)
reg [13:0] mag_rd_addr;
reg [15:0] mag_rd_data;

always @(posedge ft_clk) begin
    mag_rd_data <= doppler_mag_bram[mag_rd_addr];
end

// ============================================================================
// RANGE PROFILE BRAM (clk write, ft_clk read)
// ============================================================================
// 512 entries × 16-bit magnitude. Written as range bins arrive.
// range_profile input is {range_q[15:0], range_i[15:0]}.
// We store Manhattan magnitude: |I| + |Q|.

(* ram_style = "block" *) reg [15:0] range_bram [0:NUM_RANGE_BINS-1];

reg [RANGE_BIN_BITS-1:0] range_wr_addr;
reg [15:0]               range_wr_data;
reg                      range_wr_en;

always @(posedge clk) begin
    if (range_wr_en)
        range_bram[range_wr_addr] <= range_wr_data;
end

reg [RANGE_BIN_BITS-1:0] range_rd_addr;
reg [15:0]               range_rd_data;

always @(posedge ft_clk) begin
    range_rd_data <= range_bram[range_rd_addr];
end

// ============================================================================
// DETECTION FLAG BRAM (clk write, ft_clk read)
// ============================================================================
// 16384 bits stored as 2048 × 8-bit bytes.
// Write: individual bit-set on cfar_valid with cfar_detection=1.
// Clear: bulk clear on frame_complete (start of new frame).
// Address = {range_bin[8:0], doppler_bin[4:2]} = byte address (11 bits, 2048 entries)
// Bit position = doppler_bin[1:0] within sub-byte ... actually let's use
// a simpler scheme: 16384 entries × 1-bit, but that doesn't map well to BRAM.
//
// Better: Store as 2048 × 8-bit. Each byte holds 8 consecutive detection bits.
// Bit address = {range_bin, doppler_bin} = 14-bit. Byte addr = bit_addr[13:3].
// Bit position = bit_addr[2:0].
// On write: read-modify-write (set bit). On frame clear: bulk zero.
//
// For simplicity and BRAM efficiency, we use a separate approach:
// Store detections in a small register file and pack during transfer.
// With 512×32=16384 bits, that's 2048 bytes — fits in 1 BRAM18.
//
// IMPLEMENTATION: We use the BRAM in byte-write mode. On cfar_valid, we do
// a 1-cycle read then 1-cycle write-back with the bit set. This works because
// CFAR outputs arrive one cell per clock cycle (sequential scan).

(* ram_style = "block" *) reg [7:0] detect_bram [0:2047];

reg [10:0] detect_wr_addr;
reg [7:0]  detect_wr_data;
reg        detect_wr_en;

always @(posedge clk) begin
    if (detect_wr_en)
        detect_bram[detect_wr_addr] <= detect_wr_data;
end

reg [10:0] detect_rd_addr;
reg [7:0]  detect_rd_data;

always @(posedge ft_clk) begin
    detect_rd_data <= detect_bram[detect_rd_addr];
end

// Detection BRAM read-modify-write pipeline (clk domain)
reg [10:0] detect_rmw_addr;
reg [7:0]  detect_rmw_rd;
reg [2:0]  detect_rmw_bit;
reg        detect_rmw_value;
reg [1:0]  detect_rmw_state;  // 0=idle, 1=read, 2=write

// Synchronous read for RMW (clk domain, separate from ft_clk read port)
// We need a second read port for RMW. Since Xilinx BRAM is true dual-port,
// we use port A for both RMW-read and write (same clock), port B for ft_clk read.
// Port A read:
reg [7:0] detect_rmw_rddata;
always @(posedge clk) begin
    detect_rmw_rddata <= detect_bram[detect_rmw_addr];
end

// ============================================================================
// MANHATTAN MAGNITUDE COMPUTATION (combinational)
// ============================================================================
// |I| + |Q| with saturation to 16 bits.
// doppler_real and doppler_imag are signed 16-bit.

wire [15:0] abs_doppler_i = doppler_real[15] ? (~doppler_real + 16'd1) : doppler_real;
wire [15:0] abs_doppler_q = doppler_imag[15] ? (~doppler_imag + 16'd1) : doppler_imag;
wire [16:0] manhattan_sum = {1'b0, abs_doppler_i} + {1'b0, abs_doppler_q};
wire [15:0] manhattan_mag = manhattan_sum[16] ? 16'hFFFF : manhattan_sum[15:0];

// Range profile magnitude.
// Input range_profile is {16'd0, decimated_manhattan_mag} from radar_receiver_final,
// so range_profile[15:0] is already an unsigned Manhattan magnitude.
// We keep the full I/Q Manhattan path for future I/Q streaming support.
wire [15:0] range_i = range_profile[15:0];
wire [15:0] range_q = range_profile[31:16];
wire [15:0] abs_range_i = range_i[15] ? (~range_i + 16'd1) : range_i;
wire [15:0] abs_range_q = range_q[15] ? (~range_q + 16'd1) : range_q;
wire [16:0] range_manhattan = {1'b0, abs_range_i} + {1'b0, abs_range_q};
wire [15:0] range_mag = range_manhattan[16] ? 16'hFFFF : range_manhattan[15:0];

// ============================================================================
// FRAME WRITE LOGIC (clk domain, 100 MHz)
// ============================================================================
// Accumulates one full frame of data into BRAMs.
// On frame_complete: toggles frame_ready signal for CDC to ft_clk domain.

reg [15:0] frame_number;        // Incrementing frame counter
reg        frame_ready_toggle;  // Toggle CDC: frame ready for USB transfer
reg        frame_filling;       // 1 = currently accumulating frame data
reg [13:0] detect_clear_addr;   // Counter for bulk-clearing detection BRAM
reg        detect_clearing;     // 1 = bulk clear in progress

// Range bin counter for range profile writes
// range_valid arrives with range_profile data; we need range_bin_in to address it.
// However, range_valid comes from the matched filter chain BEFORE Doppler processing,
// so range_bin_in may not be valid at that time. We use a simple counter instead,
// since range bins arrive sequentially (0, 1, 2, ..., 511) within each chirp.
// Actually, range_profile from radar_receiver_final comes with associated range index
// from the matched filter. But the USB interface receives usb_range_valid which is
// just the Doppler output re-packed. Let's use the range_bin_in input which is
// the Doppler processor's range_bin output — valid when doppler_valid is asserted.
//
// For the range profile, we accumulate the magnitude across chirps (sum of |I|+|Q|
// for each range bin across all Doppler bins). This gives a range profile that
// represents total energy per range bin.
//
// SIMPLER APPROACH: Just store the range_profile data directly using a sequential
// counter. range_valid fires once per range bin per chirp; we overwrite (last chirp wins).
// This matches the legacy behavior where range_profile was sent per-sample.

reg [RANGE_BIN_BITS-1:0] range_write_counter;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        frame_number       <= 16'd0;
        frame_ready_toggle <= 1'b0;
        frame_filling      <= 1'b1;
        mag_wr_en          <= 1'b0;
        mag_wr_addr        <= 14'd0;
        mag_wr_data        <= 16'd0;
        range_wr_en        <= 1'b0;
        range_wr_addr      <= {RANGE_BIN_BITS{1'b0}};
        range_wr_data      <= 16'd0;
        detect_wr_en       <= 1'b0;
        detect_wr_addr     <= 11'd0;
        detect_wr_data     <= 8'd0;
        detect_clearing    <= 1'b0;
        detect_clear_addr  <= 14'd0;
        detect_rmw_state   <= 2'd0;
        detect_rmw_addr    <= 11'd0;
        detect_rmw_bit     <= 3'd0;
        detect_rmw_value   <= 1'b0;
        range_write_counter <= {RANGE_BIN_BITS{1'b0}};
    end else begin
        // Default: deassert write enables
        mag_wr_en    <= 1'b0;
        range_wr_en  <= 1'b0;
        detect_wr_en <= 1'b0;

        // === Detection BRAM bulk clear (runs after frame_complete) ===
        if (detect_clearing) begin
            detect_wr_en   <= 1'b1;
            detect_wr_addr <= detect_clear_addr[13:3];
            detect_wr_data <= 8'd0;
            if (detect_clear_addr[13:3] == 11'd2047) begin
                detect_clearing   <= 1'b0;
                detect_clear_addr <= 14'd0;
            end else begin
                detect_clear_addr <= detect_clear_addr + 14'd8;  // Step by 8 bits = 1 byte
            end
        end

        // === Detection RMW state machine ===
        case (detect_rmw_state)
            2'd0: begin /* idle */ end
            2'd1: begin
                // Read cycle completed (data available next cycle)
                detect_rmw_state <= 2'd2;
            end
            2'd2: begin
                // Write back with bit set/cleared
                detect_wr_en   <= 1'b1;
                detect_wr_addr <= detect_rmw_addr;
                if (detect_rmw_value)
                    detect_wr_data <= detect_rmw_rddata | (8'd1 << detect_rmw_bit);
                else
                    detect_wr_data <= detect_rmw_rddata & ~(8'd1 << detect_rmw_bit);
                detect_rmw_state <= 2'd0;
            end
            default: detect_rmw_state <= 2'd0;
        endcase

        // === Doppler magnitude write ===
        if (doppler_valid && frame_filling) begin
            mag_wr_en   <= 1'b1;
            mag_wr_addr <= {range_bin_in, doppler_bin_in};
            mag_wr_data <= manhattan_mag;
        end

        // === Range profile write ===
        if (range_valid && frame_filling) begin
            range_wr_en        <= 1'b1;
            range_wr_addr      <= range_write_counter;
            range_wr_data      <= range_mag;
            range_write_counter <= range_write_counter + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
        end

        // === CFAR detection write (read-modify-write) ===
        if (cfar_valid && frame_filling && detect_rmw_state == 2'd0 && !detect_clearing) begin
            // Start RMW: compute byte address and bit position
            // bit_addr = {range_bin_in, doppler_bin_in} = 14-bit
            // byte_addr = bit_addr[13:3], bit_pos = bit_addr[2:0]
            detect_rmw_addr  <= {range_bin_in, doppler_bin_in[4:3]};
            detect_rmw_bit   <= doppler_bin_in[2:0];
            detect_rmw_value <= cfar_detection;
            detect_rmw_state <= 2'd1;
        end

        // === Frame complete: latch frame, signal ft_clk domain ===
        if (frame_complete) begin
            frame_ready_toggle  <= ~frame_ready_toggle;
            frame_number        <= frame_number + 16'd1;
            frame_filling       <= 1'b0;  // Stop writing until USB transfer starts
            range_write_counter <= {RANGE_BIN_BITS{1'b0}};
        end

        // === Resume filling after ft_clk domain acknowledges frame transfer ===
        // (handled below via frame_ack_toggle CDC)
        // For simplicity, we resume filling immediately — the USB transfer reads
        // from the same BRAM while new data writes. Since the Doppler burst
        // (0.164 ms) is much shorter than the inter-frame gap (5.6 ms), and USB
        // transfer (0.875 ms) also fits within the gap, there's no overlap:
        //   t=0:     frame_complete pulse (1 clk cycle, from edge detector)
        //   t=0-0.87ms: USB reads BRAM (old frame data is stable)
        //   t=5.6ms: next Doppler burst starts writing
        // So single-buffer is safe. Re-enable filling after a short delay.
        if (!frame_filling && !frame_complete) begin
            // Re-enable after 1 cycle (frame_complete already deasserted)
            frame_filling   <= 1'b1;
            detect_clearing <= 1'b1;  // Clear detection BRAM for next frame
            detect_clear_addr <= 14'd0;
        end
    end
end

// ============================================================================
// TOGGLE CDC: clk (100 MHz) → ft_clk (60 MHz)
// ============================================================================

// --- Toggle registers (clk domain) ---
reg status_req_toggle;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        status_req_toggle <= 1'b0;
    end else begin
        if (status_request)
            status_req_toggle <= ~status_req_toggle;
    end
end

// --- 3-stage synchronizers (ft_clk domain) ---
(* ASYNC_REG = "TRUE" *) reg [2:0] frame_ready_sync;
(* ASYNC_REG = "TRUE" *) reg [2:0] status_toggle_sync;

reg frame_ready_prev;
reg status_toggle_prev;

wire frame_ready_ft = frame_ready_sync[2] ^ frame_ready_prev;
wire status_req_ft  = status_toggle_sync[2] ^ status_toggle_prev;

// --- Stream control CDC (6-bit, 2-stage level sync) ---
(* ASYNC_REG = "TRUE" *) reg [5:0] stream_ctrl_sync_0;
(* ASYNC_REG = "TRUE" *) reg [5:0] stream_ctrl_sync_1;

wire stream_range_en   = stream_ctrl_sync_1[0];
wire stream_doppler_en = stream_ctrl_sync_1[1];
wire stream_cfar_en    = stream_ctrl_sync_1[2];
wire stream_mag_only   = stream_ctrl_sync_1[3];
wire stream_sparse_det = stream_ctrl_sync_1[4];
// Bit 5 reserved
// NOTE: Phase 1 write FSM always sends magnitude-only range/Doppler and
// dense detection bitmap. The mag_only and sparse_det bits are included in
// the frame header for the Python parser but are not yet honored by the
// write FSM. Phase 2 will add I/Q and sparse detection paths.

// --- Frame metadata snapshot (latched in clk domain, stable for ft_clk read) ---
reg [15:0] frame_number_snapshot;
reg [5:0]  stream_flags_snapshot;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        frame_number_snapshot <= 16'd0;
        stream_flags_snapshot <= `RP_STREAM_CTRL_DEFAULT;
    end else if (frame_complete) begin
        frame_number_snapshot <= frame_number;
        stream_flags_snapshot <= stream_control;
    end
end

// --- Status snapshot (ft_clk domain) ---
reg [31:0] status_words [0:5];

// Byte counter for write FSM (needs to be wide enough for largest section)
reg [15:0] wr_byte_idx;

// BRAM read address for frame transfer
reg [13:0] bram_rd_cell;     // Cell index 0..16383 for doppler/detect
reg [RANGE_BIN_BITS-1:0] range_rd_idx;  // Range bin index 0..511
reg        wr_byte_phase;    // 0=MSB, 1=LSB for 16-bit values
reg [10:0] detect_rd_idx;    // Byte index 0..2047 for detection flags

// ============================================================================
// CLOCK-ACTIVITY WATCHDOG (clk domain)
// ============================================================================
// Detects when ft_clk stops (USB cable unplugged). A toggle register in the
// ft_clk domain flips every ft_clk edge. The clk domain synchronizes it and
// checks for transitions. If no transition is seen for 2^16 = 65536 clk
// cycles (~0.65 ms at 100 MHz), ft_clk_lost asserts.
//
// ft_clk_lost feeds into the effective reset for the ft_clk domain so that
// the write FSM and BRAM read pointers return to a clean state automatically
// when the cable is unplugged. When ft_clk resumes, a 2-stage reset
// synchronizer deasserts the effective reset cleanly in the ft_clk domain.

// Toggle register: flips every ft_clk edge (ft_clk domain).
// Uses raw ft_reset_n here (not ft_effective_reset_n) to avoid a
// combinational loop through ft_clk_lost.
reg ft_heartbeat;
always @(posedge ft_clk or negedge ft_reset_n) begin
    if (!ft_reset_n)
        ft_heartbeat <= 1'b0;
    else
        ft_heartbeat <= ~ft_heartbeat;
end

// Synchronize heartbeat into clk domain (2-stage) and watch for stalls
(* ASYNC_REG = "TRUE" *) reg [1:0] ft_hb_sync;
reg ft_hb_prev;
reg [15:0] ft_clk_timeout;
reg ft_clk_lost;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        ft_hb_sync     <= 2'b00;
        ft_hb_prev     <= 1'b0;
        ft_clk_timeout <= 16'd0;
        ft_clk_lost    <= 1'b0;
    end else begin
        ft_hb_sync <= {ft_hb_sync[0], ft_heartbeat};
        ft_hb_prev <= ft_hb_sync[1];

        if (ft_hb_sync[1] != ft_hb_prev) begin
            // ft_clk is alive — reset counter, clear lost flag
            ft_clk_timeout <= 16'd0;
            ft_clk_lost    <= 1'b0;
        end else if (!ft_clk_lost) begin
            if (ft_clk_timeout == 16'hFFFF)
                ft_clk_lost <= 1'b1;
            else
                ft_clk_timeout <= ft_clk_timeout + 16'd1;
        end
    end
end

// Effective FT-domain reset: asserted by ft_reset_n OR clock loss.
// Deassertion synchronized to ft_clk via 2-stage sync to avoid
// metastability on the recovery edge.
wire ft_reset_raw_n = ft_reset_n & ~ft_clk_lost;
(* ASYNC_REG = "TRUE" *) reg [1:0] ft_reset_sync;
always @(posedge ft_clk or negedge ft_reset_raw_n) begin
    if (!ft_reset_raw_n)
        ft_reset_sync <= 2'b00;
    else
        ft_reset_sync <= {ft_reset_sync[0], 1'b1};
end
wire ft_effective_reset_n = ft_reset_sync[1];

integer si;
always @(posedge ft_clk or negedge ft_effective_reset_n) begin
    if (!ft_effective_reset_n) begin
        frame_ready_sync   <= 3'b000;
        status_toggle_sync <= 3'b000;
        frame_ready_prev   <= 1'b0;
        status_toggle_prev <= 1'b0;
        stream_ctrl_sync_0 <= `RP_STREAM_CTRL_DEFAULT;
        stream_ctrl_sync_1 <= `RP_STREAM_CTRL_DEFAULT;
        for (si = 0; si < 6; si = si + 1)
            status_words[si] <= 32'd0;
        wr_state       <= WR_IDLE;
        wr_byte_idx    <= 16'd0;
        wr_byte_phase  <= 1'b0;
        bram_rd_cell   <= 14'd0;
        range_rd_idx   <= {RANGE_BIN_BITS{1'b0}};
        range_rd_addr  <= {RANGE_BIN_BITS{1'b0}};
        detect_rd_idx  <= 11'd0;
        detect_rd_addr <= 11'd0;
        mag_rd_addr    <= 14'd0;
        rd_state        <= RD_IDLE;
        rd_byte_cnt     <= 2'd0;
        rd_cmd_complete <= 1'b0;
        rd_shift_reg    <= 32'd0;
        ft_data_out    <= 8'd0;
        ft_data_oe     <= 1'b0;
        ft_rd_n        <= 1'b1;
        ft_wr_n        <= 1'b1;
        ft_oe_n        <= 1'b1;
        ft_siwu        <= 1'b0;
        cmd_data       <= 32'd0;
        cmd_valid      <= 1'b0;
        cmd_opcode     <= 8'd0;
        cmd_addr       <= 8'd0;
        cmd_value      <= 16'd0;
    end else begin
        cmd_valid <= 1'b0;

        // 3-stage toggle synchronizers
        frame_ready_sync   <= {frame_ready_sync[1:0], frame_ready_toggle};
        status_toggle_sync <= {status_toggle_sync[1:0], status_req_toggle};
        frame_ready_prev   <= frame_ready_sync[2];
        status_toggle_prev <= status_toggle_sync[2];

        // Stream control CDC
        stream_ctrl_sync_0 <= stream_control;
        stream_ctrl_sync_1 <= stream_ctrl_sync_0;

        // Status snapshot on request
        if (status_req_ft) begin
            // Word 0: {0xFF, mode[1:0], stream[5:0], threshold[15:0]}
            // NOTE: stream_ctrl now 6-bit. Pack as: {0xFF, mode, stream[5:3], stream[2:0], threshold}
            // Keep backward-compatible layout: {0xFF[31:24], mode[23:22], stream[21:16], threshold[15:0]}
            status_words[0] <= {8'hFF, status_radar_mode, status_stream_ctrl, status_cfar_threshold};
            status_words[1] <= {status_long_chirp, status_long_listen};
            status_words[2] <= {status_guard, status_short_chirp};
            status_words[3] <= {status_short_listen, 10'd0, status_chirps_per_elev};
            status_words[4] <= {status_agc_current_gain,
                                status_agc_peak_magnitude,
                                status_agc_saturation_count,
                                status_agc_enable,
                                9'd0,
                                status_range_mode};
            status_words[5] <= {7'd0, status_self_test_busy,
                                8'd0, status_self_test_detail,
                                3'd0, status_self_test_flags};
        end

        // ================================================================
        // READ FSM — Host → FPGA command path (unchanged from legacy)
        // ================================================================
        case (rd_state)
            RD_IDLE: begin
                if (wr_state == WR_IDLE && !ft_rxf_n) begin
                    ft_oe_n    <= 1'b0;
                    ft_data_oe <= 1'b0;
                    rd_state   <= RD_OE_ASSERT;
                end
            end

            RD_OE_ASSERT: begin
                if (!ft_rxf_n) begin
                    ft_rd_n  <= 1'b0;
                    rd_state <= RD_READING;
                end else begin
                    ft_oe_n  <= 1'b1;
                    rd_state <= RD_IDLE;
                end
            end

            RD_READING: begin
                rd_shift_reg <= {rd_shift_reg[23:0], ft_data};
                if (rd_byte_cnt == 2'd3) begin
                    ft_rd_n         <= 1'b1;
                    rd_byte_cnt     <= 2'd0;
                    rd_cmd_complete <= 1'b1;
                    rd_state        <= RD_DEASSERT;
                end else begin
                    rd_byte_cnt <= rd_byte_cnt + 2'd1;
                    if (ft_rxf_n) begin
                        // Host ran out of data mid-command — abort
                        ft_rd_n         <= 1'b1;
                        rd_byte_cnt     <= 2'd0;
                        rd_cmd_complete <= 1'b0;
                        rd_state        <= RD_DEASSERT;
                    end
                end
            end

            RD_DEASSERT: begin
                ft_oe_n <= 1'b1;
                // Only process if we received a full 4-byte command
                if (rd_cmd_complete) begin
                    rd_cmd_complete <= 1'b0;
                    rd_state        <= RD_PROCESS;
                end else begin
                    // Incomplete command — discard
                    rd_state <= RD_IDLE;
                end
            end

            RD_PROCESS: begin
                cmd_data   <= rd_shift_reg;
                cmd_opcode <= rd_shift_reg[31:24];
                cmd_addr   <= rd_shift_reg[23:16];
                cmd_value  <= rd_shift_reg[15:0];
                cmd_valid  <= 1'b1;
                rd_state   <= RD_IDLE;
            end

            default: rd_state <= RD_IDLE;
        endcase

        // ================================================================
        // WRITE FSM — Bulk per-frame transfer (ft_clk domain)
        // ================================================================
        if (rd_state == RD_IDLE) begin
            case (wr_state)
                WR_IDLE: begin
                    ft_wr_n    <= 1'b1;
                    ft_data_oe <= 1'b0;

                    // Status readback takes priority over frame data
                    if (status_req_ft && ft_rxf_n) begin
                        wr_state    <= WR_STATUS_SEND;
                        wr_byte_idx <= 16'd0;
                    end
                    // New frame ready for transfer
                    else if (frame_ready_ft && ft_rxf_n) begin
                        wr_state      <= WR_FRAME_HDR;
                        wr_byte_idx   <= 16'd0;
                        bram_rd_cell  <= 14'd0;
                        range_rd_idx  <= {RANGE_BIN_BITS{1'b0}};
                        range_rd_addr <= {RANGE_BIN_BITS{1'b0}};  // Pre-load first read addr
                        detect_rd_idx <= 11'd0;
                        detect_rd_addr <= 11'd0;
                        mag_rd_addr   <= 14'd0;
                        wr_byte_phase <= 1'b0;
                    end
                end

                // ---- Frame header: 8 bytes ----
                WR_FRAME_HDR: begin
                    if (!ft_txe_n) begin
                        ft_data_oe <= 1'b1;
                        ft_wr_n    <= 1'b0;

                        case (wr_byte_idx[2:0])
                            3'd0: ft_data_out <= HEADER;
                            3'd1: ft_data_out <= {2'b00, stream_flags_snapshot};
                            3'd2: ft_data_out <= frame_number_snapshot[15:8];
                            3'd3: ft_data_out <= frame_number_snapshot[7:0];
                            3'd4: ft_data_out <= NUM_RANGE_BINS[15:8];    // 512 >> 8 = 2
                            3'd5: ft_data_out <= NUM_RANGE_BINS[7:0];     // 512 & 0xFF = 0
                            3'd6: ft_data_out <= NUM_DOPPLER_BINS[15:8];  // 32 >> 8 = 0
                            3'd7: ft_data_out <= NUM_DOPPLER_BINS[7:0];   // 32 & 0xFF = 32
                        endcase

                        if (wr_byte_idx[2:0] == 3'd7) begin
                            wr_byte_idx   <= 16'd0;
                            wr_byte_phase <= 1'b0;
                            // Decide next section based on stream flags
                            if (stream_flags_snapshot[0])  // stream_range
                                wr_state <= WR_RANGE_DATA;
                            else if (stream_flags_snapshot[1])  // stream_doppler
                                wr_state <= WR_DOPPLER_DATA;
                            else if (stream_flags_snapshot[2])  // stream_cfar
                                wr_state <= WR_DETECT_DATA;
                            else
                                wr_state <= WR_FRAME_FOOTER;
                        end else begin
                            wr_byte_idx <= wr_byte_idx + 16'd1;
                        end
                    end
                end

                // ---- Range profile: 512 × 2 = 1024 bytes ----
                WR_RANGE_DATA: begin
                    if (!ft_txe_n) begin
                        ft_data_oe <= 1'b1;
                        ft_wr_n    <= 1'b0;

                        // BRAM read has 1-cycle latency. We pre-loaded range_rd_addr.
                        // On phase 0: output MSB of range_rd_data (read on prev cycle)
                        // On phase 1: output LSB, advance to next address
                        if (!wr_byte_phase) begin
                            ft_data_out   <= range_rd_data[15:8];
                            wr_byte_phase <= 1'b1;
                        end else begin
                            ft_data_out   <= range_rd_data[7:0];
                            wr_byte_phase <= 1'b0;
                            range_rd_idx  <= range_rd_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
                            range_rd_addr <= range_rd_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
                        end

                        wr_byte_idx <= wr_byte_idx + 16'd1;

                        if (wr_byte_idx == RANGE_SECTION_BYTES[15:0] - 16'd1) begin
                            wr_byte_idx   <= 16'd0;
                            wr_byte_phase <= 1'b0;
                            bram_rd_cell  <= 14'd0;
                            mag_rd_addr   <= 14'd0;
                            if (stream_flags_snapshot[1])
                                wr_state <= WR_DOPPLER_DATA;
                            else if (stream_flags_snapshot[2])
                                wr_state <= WR_DETECT_DATA;
                            else
                                wr_state <= WR_FRAME_FOOTER;
                        end
                    end
                end

                // ---- Doppler magnitude: 16384 × 2 = 32768 bytes (mag_only mode) ----
                // Row-major: range_bin varies slowest, doppler_bin varies fastest.
                WR_DOPPLER_DATA: begin
                    if (!ft_txe_n) begin
                        ft_data_oe <= 1'b1;
                        ft_wr_n    <= 1'b0;

                        if (!wr_byte_phase) begin
                            ft_data_out   <= mag_rd_data[15:8];
                            wr_byte_phase <= 1'b1;
                        end else begin
                            ft_data_out   <= mag_rd_data[7:0];
                            wr_byte_phase <= 1'b0;
                            bram_rd_cell  <= bram_rd_cell + 14'd1;
                            mag_rd_addr   <= bram_rd_cell + 14'd1;
                        end

                        wr_byte_idx <= wr_byte_idx + 16'd1;

                        if (wr_byte_idx == DOPPLER_MAG_SECTION_BYTES[15:0] - 16'd1) begin
                            wr_byte_idx    <= 16'd0;
                            wr_byte_phase  <= 1'b0;
                            detect_rd_idx  <= 11'd0;
                            detect_rd_addr <= 11'd0;
                            if (stream_flags_snapshot[2])
                                wr_state <= WR_DETECT_DATA;
                            else
                                wr_state <= WR_FRAME_FOOTER;
                        end
                    end
                end

                // ---- Detection flags: 2048 bytes (dense mode) ----
                WR_DETECT_DATA: begin
                    if (!ft_txe_n) begin
                        ft_data_oe <= 1'b1;
                        ft_wr_n    <= 1'b0;

                        // 1-byte per cycle (BRAM read latency handled by pre-loading addr)
                        ft_data_out    <= detect_rd_data;
                        detect_rd_idx  <= detect_rd_idx + 11'd1;
                        detect_rd_addr <= detect_rd_idx + 11'd1;

                        wr_byte_idx <= wr_byte_idx + 16'd1;

                        if (wr_byte_idx == DETECT_SECTION_BYTES[15:0] - 16'd1) begin
                            wr_byte_idx <= 16'd0;
                            wr_state    <= WR_FRAME_FOOTER;
                        end
                    end
                end

                // ---- Frame footer: 1 byte ----
                WR_FRAME_FOOTER: begin
                    if (!ft_txe_n) begin
                        ft_data_oe  <= 1'b1;
                        ft_data_out <= FOOTER;
                        ft_wr_n     <= 1'b0;
                        wr_state    <= WR_DONE;
                    end
                end

                // ---- Status packet: 26 bytes (unchanged from legacy) ----
                WR_STATUS_SEND: begin
                    if (!ft_txe_n) begin
                        ft_data_oe <= 1'b1;
                        ft_wr_n    <= 1'b0;

                        case (wr_byte_idx[4:0])
                            5'd0:  ft_data_out <= STATUS_HEADER;
                            5'd1:  ft_data_out <= status_words[0][31:24];
                            5'd2:  ft_data_out <= status_words[0][23:16];
                            5'd3:  ft_data_out <= status_words[0][15:8];
                            5'd4:  ft_data_out <= status_words[0][7:0];
                            5'd5:  ft_data_out <= status_words[1][31:24];
                            5'd6:  ft_data_out <= status_words[1][23:16];
                            5'd7:  ft_data_out <= status_words[1][15:8];
                            5'd8:  ft_data_out <= status_words[1][7:0];
                            5'd9:  ft_data_out <= status_words[2][31:24];
                            5'd10: ft_data_out <= status_words[2][23:16];
                            5'd11: ft_data_out <= status_words[2][15:8];
                            5'd12: ft_data_out <= status_words[2][7:0];
                            5'd13: ft_data_out <= status_words[3][31:24];
                            5'd14: ft_data_out <= status_words[3][23:16];
                            5'd15: ft_data_out <= status_words[3][15:8];
                            5'd16: ft_data_out <= status_words[3][7:0];
                            5'd17: ft_data_out <= status_words[4][31:24];
                            5'd18: ft_data_out <= status_words[4][23:16];
                            5'd19: ft_data_out <= status_words[4][15:8];
                            5'd20: ft_data_out <= status_words[4][7:0];
                            5'd21: ft_data_out <= status_words[5][31:24];
                            5'd22: ft_data_out <= status_words[5][23:16];
                            5'd23: ft_data_out <= status_words[5][15:8];
                            5'd24: ft_data_out <= status_words[5][7:0];
                            5'd25: ft_data_out <= FOOTER;
                            default: ft_data_out <= 8'h00;
                        endcase

                        if (wr_byte_idx[4:0] == STATUS_PKT_LEN - 5'd1) begin
                            wr_state    <= WR_DONE;
                            wr_byte_idx <= 16'd0;
                        end else begin
                            wr_byte_idx <= wr_byte_idx + 16'd1;
                        end
                    end
                end

                WR_DONE: begin
                    ft_wr_n    <= 1'b1;
                    ft_data_oe <= 1'b0;
                    wr_state   <= WR_IDLE;
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end
end

endmodule
