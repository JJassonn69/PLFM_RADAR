`timescale 1ns / 1ps

`include "radar_params.vh"

/**
 * usb_data_interface_ft2232h.v
 *
 * FT2232H USB 2.0 Hi-Speed FIFO Interface (245 Synchronous FIFO Mode)
 * Channel A only — 8-bit data bus, 60 MHz CLKOUT from FT2232H.
 *
 * BULK PER-FRAME PROTOCOL V2 (PR-G — single canonical encoding):
 *
 * Frame packet (FPGA→Host): variable length, up to 74,762 bytes
 *   Byte 0:       0xAA (frame start header)
 *   Byte 1:       0x02 (PROTOCOL VERSION — host MUST reject any other value)
 *   Byte 2:       Flags byte. Layout (PR-U / M-8 widened bits[5:3]):
 *                   bits[7:6] = 2'b00 reserved
 *                   bits[5:3] = subframe_enable[2:0] = {LONG, MEDIUM, SHORT}
 *                               (host_subframe_enable snapshot at frame_complete)
 *                   bits[2:0] = {stream_cfar, stream_doppler, stream_range}
 *   Bytes 3-4:    Frame number (uint16, MSB first)
 *   Bytes 5-6:    Range bin count   (uint16, MSB first) = `RP_NUM_RANGE_BINS`  (512)
 *   Bytes 7-8:    Doppler bin count (uint16, MSB first) = `RP_NUM_DOPPLER_BINS` (48)
 *
 *   [If stream_range (bit 0):]
 *     Next 1024 bytes: range profile, 512 × uint16 Manhattan magnitude, MSB first.
 *
 *   [If stream_doppler (bit 1):]
 *     Next 65536 bytes: doppler magnitude, 32768 cells × uint16, row-major
 *     (range_bin slowest, doppler_bin fastest), MSB first. Cells indexed
 *     [0..47] are real Doppler bins; cells [48..63] within each range are
 *     the power-of-2 padding from PR-F (always emitted as 0x0000).
 *
 *   [If stream_cfar (bit 2):]
 *     Next 8192 bytes: detect_class bitmap, 32768 cells × 2 bits, MSB-first
 *     packing. Each byte holds 4 cells:
 *       byte[N]:  bits[7:6]=cell[4*N], bits[5:4]=cell[4*N+1],
 *                 bits[3:2]=cell[4*N+2], bits[1:0]=cell[4*N+3]
 *     Cell encoding (per `RP_DETECT_*`):
 *       2'b00 = NONE      (below soft threshold)
 *       2'b01 = CANDIDATE (above soft, below confirm — host re-cues)
 *       2'b10 = CONFIRMED (above confirm threshold — track-eligible)
 *       2'b11 = RESERVED  (must not be emitted by RTL)
 *
 *   Last byte:    0x55 (frame end footer)
 *
 * Status packet (FPGA→Host): 34 bytes (M-5: was 30 / PR-G; was 26 / v1)
 *   Byte 0:       0xBB (status header)
 *   Bytes 1-32:   8 × 32-bit status words, MSB first
 *                 word[6] = {detect_count_cand[15:0], detect_threshold_soft[15:0]}  (PR-G)
 *                 word[7] = {medium_chirp[15:0], medium_listen[15:0]}               (M-5)
 *   Byte 33:      0x55 (footer)
 *
 * Command (Host→FPGA): 4 bytes received sequentially (unchanged)
 *   Byte 0: opcode[7:0]    (see RP_OP_* in radar_params.vh)
 *   Byte 1: addr[7:0]
 *   Byte 2: value[15:8]
 *   Byte 3: value[7:0]
 *
 * MEMORY ARCHITECTURE:
 *   - Doppler magnitude BRAM: 32768 entries × 16-bit = 64 KB (~28 BRAM18 on 50T)
 *     Written in clk (100 MHz) domain as Doppler cells arrive.
 *     Read in ft_clk (60 MHz) domain during USB bulk transfer.
 *   - Range profile buffer: 512 × 16-bit = 1 KB (1 BRAM18)
 *     Written in clk domain from range_valid events.
 *   - Detect-class buffer: 32768 cells × 2 bits = 65536 bits = 8192 bytes (4 BRAM18)
 *     Written in clk domain from cfar_valid events via 3-cycle RMW pipeline.
 *
 * BANDWIDTH BUDGET (PR-G v2, all streams):
 *   Header: 9 B + Range: 1024 B + Doppler: 65536 B + Detect: 8192 B + Footer: 1 B
 *   = 74,762 bytes/frame × ~119 fps (3-subframe rate post-PR-F) ≈ 8.9 MB/s
 *   FT2232H 245-Sync-FIFO conservative budget ~8 MB/s (FTDI AN_232B-04, 80%
 *   utilisation); practical sustained throughput is 30–40 MB/s on a tuned
 *   host. Sufficient headroom even with the conservative budget overshoot.
 *
 * CDC STRATEGY:
 *   - Frame data: Written to dual-port BRAM at 100 MHz, read at 60 MHz (inherently CDC-safe).
 *   - frame_ready flag: Toggle CDC (100 MHz → 60 MHz), same as status_request.
 *   - stream_control: 2-stage level sync (changes infrequently).
 *   - status_*_soft / status_*_cand: 2-stage level sync (slow-changing per-frame values).
 *   - Commands: Read FSM in ft_clk domain, output CDC'd by consumer (unchanged).
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
    // PR-G: 2-bit class replaces single cfar_detection bit.
    input wire [`RP_DETECT_CLASS_WIDTH-1:0] cfar_detect_class,
    input wire cfar_valid,

    // New inputs for bulk frame protocol (clk domain)
    // [RX-D] Widened to RP_RANGE_BIN_WIDTH_MAX (9-bit on 50T, 12-bit on 200T)
    // to match upstream pipeline. In 3 km mode only bins 0..511 are exercised
    // and the frame wire protocol still emits 512×32=16384 cells. 20 km mode
    // (4096 bins, 131072 cells) requires a wire-protocol extension before
    // bins 512..4095 can be transported to the host.
    input wire [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin_in,
    input wire [`RP_DOPPLER_BIN_WIDTH-1:0]   doppler_bin_in,  // 6-bit (PR-F): {sub_frame[1:0], bin[3:0]}
    input wire                               frame_complete,  // 1-cycle pulse from radar_receiver_final edge detector

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

    // PR-U / M-8: per-frame sub-frame enable mask (clk domain, CDC'd
    // internally, snapshotted at frame_complete). {LONG, MEDIUM, SHORT}.
    // Echoed in v2 frame byte 2 bits[5:3] so the host CRT can detect
    // when an operator disables a sub-frame and downgrade confidence
    // (default 3'b111 keeps the production 3-PRI ladder behavior).
    input wire [2:0] subframe_enable,

    // Status readback inputs (clk domain, CDC'd internally)
    input wire status_request,
    input wire [15:0] status_cfar_threshold,
    input wire [5:0]  status_stream_ctrl,
    // status_radar_mode + status_range_mode retired in PR-AB.b expanded.
    // Bits in status_words[0][23:22] and status_words[4][1:0] are reserved 0.
    input wire [15:0] status_long_chirp,
    input wire [15:0] status_long_listen,
    input wire [15:0] status_guard,
    input wire [15:0] status_short_chirp,
    input wire [15:0] status_short_listen,
    input wire [15:0] status_medium_chirp,     // M-5: status_words[7][31:16] readback
    input wire [15:0] status_medium_listen,    // M-5: status_words[7][15:0]  readback
    input wire [5:0]  status_chirps_per_elev,
    input wire        status_chirps_mismatch,  // TX-G: host requested chirps != Doppler FFT size

    // Self-test status readback
    input wire [4:0]  status_self_test_flags,
    input wire [7:0]  status_self_test_detail,
    input wire        status_self_test_busy,

    // AGC status readback
    input wire [3:0]  status_agc_current_gain,
    input wire [7:0]  status_agc_peak_magnitude,
    input wire [7:0]  status_agc_saturation_count,
    input wire        status_agc_enable,

    // AUDIT-S10: control-fault flags (clk domain). Exposed in status_words[5]
    // [6:5] so host-side telemetry can graph each fault class independently
    // of the gpio_dig7 split. 2-stage level CDC into ft_clk; sticky/slow-
    // changing source so 2-FF sync is sufficient.
    input wire        status_range_decim_watchdog,  // audit F-6.4
    input wire        status_ddc_cic_fir_overrun,   // audit F-1.2

    // PR-AB.b expanded commit 5: beam-ready handshake watchdog (clk domain).
    // Sticky in chirp_scheduler; 2-FF level CDC into ft_clk. Packed into
    // status_words[4][1] — see word-4 layout below.
    input wire        status_beam_handshake_watchdog,

    // PR-G: 2-tier CFAR telemetry (clk domain → status_words[6]).
    // Slow-changing per-frame values; 2-stage level CDC into ft_clk.
    input wire [7:0]  status_cfar_alpha_soft,       // current host_cfar_alpha_soft (Q4.4)
    input wire [16:0] status_detect_threshold_soft, // PR-G: candidate-tier threshold (last frame)
    input wire [15:0] status_detect_count_cand      // PR-G: candidate count (last frame)
);

// ============================================================================
// CONSTANTS
// ============================================================================
localparam HEADER        = 8'hAA;
localparam FOOTER        = 8'h55;
localparam STATUS_HEADER = 8'hBB;

localparam NUM_RANGE_BINS  = `RP_NUM_RANGE_BINS;   // 512
localparam NUM_DOPPLER_BINS = `RP_NUM_DOPPLER_BINS; // 48 (PR-F)
localparam RANGE_BIN_BITS  = `RP_RANGE_BIN_BITS;    // 9
localparam DOPPLER_BIN_BITS = `RP_DOPPLER_BIN_WIDTH;// 6 (PR-F)
// PR-F: pad FRAME_CELLS to next-power-of-2 along the doppler axis so the
// {range, doppler[N-1:0]} concatenation lands in a contiguous BRAM block per
// range bin. Costs ~10 extra RAMB18 vs the previous 16K-cell packing but
// avoids a per-write multiply on the 100 MHz path.
localparam FRAME_CELLS     = NUM_RANGE_BINS * (1 << DOPPLER_BIN_BITS);     // 32768 (PR-F)
// Frame-cell address widths.
localparam FRAME_ADDR_W      = RANGE_BIN_BITS + DOPPLER_BIN_BITS;          // 15

// PR-G: detect section is 2 bits/cell instead of 1 bit/cell.
// 32768 cells * 2 bits = 65536 bits = 8192 bytes; needs 13-bit byte address.
// Cell-to-byte mapping: byte_addr = bit_addr[15:3] = {range_bin[8:0], doppler_bin[5:2]}
// Sub-byte position (bits within byte) = (3 - doppler_bin[1:0]) * 2, MSB-first.
localparam DETECT_BITS_PER_CELL = `RP_DETECT_BITS_PER_CELL;                // 2
localparam DETECT_BYTE_ADDR_W   = FRAME_ADDR_W + 1 - 3;                    // 13
localparam DETECT_BYTE_LAST     = ((FRAME_CELLS * DETECT_BITS_PER_CELL) / 8) - 1;  // 8191
localparam DETECT_BIT_ADDR_W    = FRAME_ADDR_W + 1;                        // 16

// Frame header: 9 bytes (0xAA + ver + flags + frame_num[2] + range_bins[2] + doppler_bins[2])
localparam FRAME_HDR_BYTES = `RP_FRAME_HDR_BYTES;                          // 9 (PR-G)
// Range profile section: 512 × 2 = 1024 bytes
localparam RANGE_SECTION_BYTES = NUM_RANGE_BINS * 2;
// Doppler mag section: 512 range × 48 doppler × 2 = 49152 bytes (PR-G).
// FSM iterates only valid (range, doppler) cells — the next-pow-2 BRAM
// padding (doppler 48..63 per range, 8192 dead cells) is skipped on the
// wire so the body length matches the header's `doppler_bins=48` field.
localparam DOPPLER_MAG_SECTION_BYTES = NUM_RANGE_BINS * NUM_DOPPLER_BINS * 2;
// Last valid doppler index — used by WR_DOPPLER_DATA to wrap to next range.
localparam [DOPPLER_BIN_BITS-1:0] DOP_BIN_LAST = NUM_DOPPLER_BINS[DOPPLER_BIN_BITS-1:0] - 1'b1;
// Detect class section: emit only valid range × doppler cells.
// Per range bin: NUM_DOPPLER_BINS × DETECT_BITS_PER_CELL / 8 bytes (rounded
// up). For 48 doppler × 2 bits = 96 bits = 12 bytes per range. The 4 padded
// bytes per range (doppler 48..63 indices) are skipped on the wire so the
// host can compute body length deterministically from the header.
localparam VALID_DET_BYTES_PER_RANGE = (NUM_DOPPLER_BINS * DETECT_BITS_PER_CELL + 7) / 8;  // 12
localparam DETECT_SECTION_BYTES      = NUM_RANGE_BINS * VALID_DET_BYTES_PER_RANGE;          // 6144
localparam [3:0] DET_BYTE_LAST_PER_RANGE = VALID_DET_BYTES_PER_RANGE[3:0] - 4'd1;           // 11

// Status packet: 34 bytes (M-5: 8 × 32-bit words + header + footer; PR-G was 7 words / 30 B).
// Width bumped 5→6 bits because 34 doesn't fit in 5 bits.
localparam STATUS_PKT_LEN = 6'd34;

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

// AUDIT-C12 instrumentation: ft_clk → clk handshake. Toggles when WR_FSM
// completes a successful frame transfer (WR_DONE → WR_IDLE). Lets the clk
// domain detect frame drops (frame_complete arrives while previous transfer
// is still in flight). See full analysis in the clk-domain block below.
reg wr_done_toggle;

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
reg [FRAME_ADDR_W-1:0] mag_wr_addr;        // PR-F: 15-bit
reg [15:0] mag_wr_data;
reg        mag_wr_en;

always @(posedge clk) begin
    if (mag_wr_en)
        doppler_mag_bram[mag_wr_addr] <= mag_wr_data;
end

// Read port (ft_clk domain)
reg [FRAME_ADDR_W-1:0] mag_rd_addr;        // PR-F: 15-bit
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
// DETECT-CLASS BRAM (clk write, ft_clk read) — PR-G: 2 bits per cell
// ============================================================================
// FRAME_CELLS cells × 2 bits = 65536 bits stored as 8192 × 8-bit bytes.
// Each byte packs 4 consecutive cells (MSB-first):
//   byte[N] bits[7:6] = cell[4*N + 0]   (doppler_bin[1:0] = 00)
//   byte[N] bits[5:4] = cell[4*N + 1]   (doppler_bin[1:0] = 01)
//   byte[N] bits[3:2] = cell[4*N + 2]   (doppler_bin[1:0] = 10)
//   byte[N] bits[1:0] = cell[4*N + 3]   (doppler_bin[1:0] = 11)
// Cell encoding per `RP_DETECT_*` (2'b00=NONE / 2'b01=CAND / 2'b10=CONFIRM).
//
// Write path: 3-cycle read-modify-write on cfar_valid (idle → read → write).
//   Cell index within byte = doppler_bin_in[1:0]
//   MSB-first shift = (3 - cell_index) * 2  (cell 0 lands in [7:6], cell 3 in [1:0])
// Clear path: bulk byte-zero on frame_complete (steps 1 byte/cycle).

(* ram_style = "block" *) reg [7:0] detect_bram [0:DETECT_BYTE_LAST];  // PR-G: 8192 entries (was 4096)

reg [DETECT_BYTE_ADDR_W-1:0] detect_wr_addr;     // PR-G: 13-bit byte addr (was 12)
reg [7:0]  detect_wr_data;
reg        detect_wr_en;

always @(posedge clk) begin
    if (detect_wr_en)
        detect_bram[detect_wr_addr] <= detect_wr_data;
end

reg [DETECT_BYTE_ADDR_W-1:0] detect_rd_addr;     // PR-G: 13-bit byte addr (was 12)
reg [7:0]                     detect_rd_data;

always @(posedge ft_clk) begin
    detect_rd_data <= detect_bram[detect_rd_addr];
end

// Detection BRAM read-modify-write pipeline (clk domain)
reg [DETECT_BYTE_ADDR_W-1:0] detect_rmw_addr;    // PR-G: 13-bit byte addr (was 12)
reg [1:0]  detect_rmw_cell_idx;                  // PR-G: 0..3, cell within byte
reg [`RP_DETECT_CLASS_WIDTH-1:0] detect_rmw_value; // PR-G: 2-bit class (was 1-bit)
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
// PR-G: byte-counter (was 15-bit bit-counter in PR-F). 8192 bytes = 13 bits.
reg [DETECT_BYTE_ADDR_W-1:0] detect_clear_addr;
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

// Forward declaration of wr_done_pulse (driven by AUDIT-C12 block below at
// line ~575) — used by the main writer always block to retrigger
// detect_clearing after each USB transfer (PR-Z A6 Bug C fix).
(* ASYNC_REG = "TRUE" *) reg [2:0] wr_done_sync;
reg                                wr_done_prev;
wire                               wr_done_pulse = wr_done_sync[2] ^ wr_done_prev;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        frame_number       <= 16'd0;
        frame_ready_toggle <= 1'b0;
        frame_filling      <= 1'b1;
        mag_wr_en          <= 1'b0;
        mag_wr_addr        <= {FRAME_ADDR_W{1'b0}};
        mag_wr_data        <= 16'd0;
        range_wr_en        <= 1'b0;
        range_wr_addr      <= {RANGE_BIN_BITS{1'b0}};
        range_wr_data      <= 16'd0;
        detect_wr_en       <= 1'b0;
        detect_wr_addr     <= {DETECT_BYTE_ADDR_W{1'b0}};
        detect_wr_data     <= 8'd0;
        detect_clearing    <= 1'b0;
        detect_clear_addr  <= {DETECT_BYTE_ADDR_W{1'b0}};
        detect_rmw_state   <= 2'd0;
        detect_rmw_addr    <= {DETECT_BYTE_ADDR_W{1'b0}};
        detect_rmw_cell_idx <= 2'd0;
        detect_rmw_value   <= `RP_DETECT_NONE;
        range_write_counter <= {RANGE_BIN_BITS{1'b0}};
    end else begin
        // Default: deassert write enables
        mag_wr_en    <= 1'b0;
        range_wr_en  <= 1'b0;
        detect_wr_en <= 1'b0;

        // === Detect-class BRAM bulk clear (runs after frame_complete) ===
        // PR-G: 1 byte/cycle byte-counter (was 8-bits-per-cycle bit-counter).
        if (detect_clearing) begin
            detect_wr_en   <= 1'b1;
            detect_wr_addr <= detect_clear_addr;
            detect_wr_data <= 8'd0;
            if (detect_clear_addr == DETECT_BYTE_LAST[DETECT_BYTE_ADDR_W-1:0]) begin
                detect_clearing   <= 1'b0;
                detect_clear_addr <= {DETECT_BYTE_ADDR_W{1'b0}};
            end else begin
                detect_clear_addr <= detect_clear_addr + {{(DETECT_BYTE_ADDR_W-1){1'b0}}, 1'b1};
            end
        end

        // === Detect-class RMW state machine (PR-G: 2-bit pack) ===
        // Cell N within byte → MSB-first: shift = (3 - N) * 2 = {!N[1], !N[0], 1'b0}
        case (detect_rmw_state)
            2'd0: begin /* idle */ end
            2'd1: begin
                // Read cycle completed (data available next cycle)
                detect_rmw_state <= 2'd2;
            end
            2'd2: begin
                // Write back with the 2-bit class field updated.
                detect_wr_en   <= 1'b1;
                detect_wr_addr <= detect_rmw_addr;
                // Mask out the 2 bits for this cell, OR in the new class.
                // shift_amt = (3 - cell_idx) * 2 ∈ {6, 4, 2, 0}
                detect_wr_data <= (detect_rmw_rddata & ~(8'b11000000 >> ({1'b0, detect_rmw_cell_idx} << 1)))
                                | (({6'b0, detect_rmw_value} << ((3 - {1'b0, detect_rmw_cell_idx}) << 1)));
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

        // === CFAR detect-class write (read-modify-write) ===
        // PR-G: 2 bits per cell. bit_addr = {range_bin[8:0], doppler_bin[5:0]} * 2
        //        = {range_bin[8:0], doppler_bin[5:0], 1'b0} (16 bits).
        //   byte_addr = bit_addr[15:3] = {range_bin[8:0], doppler_bin[5:2]} (13 bits)
        //   cell_idx within byte = doppler_bin[1:0] (0..3, MSB-first ordering)
        if (cfar_valid && frame_filling && detect_rmw_state == 2'd0 && !detect_clearing) begin
            detect_rmw_addr     <= {range_bin_in, doppler_bin_in[DOPPLER_BIN_BITS-1:2]};
            detect_rmw_cell_idx <= doppler_bin_in[1:0];
            detect_rmw_value    <= cfar_detect_class;
            detect_rmw_state    <= 2'd1;
        end

        // === Frame complete: latch frame, signal ft_clk domain ===
        if (frame_complete) begin
            frame_ready_toggle  <= ~frame_ready_toggle;
            frame_number        <= frame_number + 16'd1;
            frame_filling       <= 1'b0;  // Stop writing until USB transfer starts
            range_write_counter <= {RANGE_BIN_BITS{1'b0}};
        end

        // === Resume filling after frame_complete ===
        // AUDIT-C12 timing analysis (corrected from earlier comment):
        //   - Frame period (178 fps): 5.62 ms
        //   - Doppler emit window: ~0.5 ms at end of frame (16384 cells × 1
        //     emission/cycle + per-range-bin 16-pt FFT compute, ~50K cycles)
        //   - USB transfer (Hi-Speed bulk @ 8 MB/s): 35849 bytes / 8 MB/s
        //     = 4.48 ms (NOT 0.875 ms as the original comment claimed)
        //   - Slack at 178 fps: 5.62 − 4.48 = 1.14 ms (~20%)
        //
        // Write/read order BOTH advance range_bin slowest, doppler_bin fastest;
        // FPGA write of frame N+1 starts at addr 0 while USB read of frame N is
        // near addr 16383, so the brief overlap (~0.16 ms at end of WR_DOPPLER)
        // never collides on the same address. No data corruption today.
        //
        // Real failure mode: at higher frame rates (or USB bandwidth shortfalls),
        // frame_complete N+1 may fire while WR_FSM is still draining frame N.
        // frame_ready_toggle's edge in the ft_clk domain is missed unless WR_FSM
        // is in WR_IDLE — frame N+1 silently dropped. The frame_drop_count
        // counter below makes this loud.
        //
        // Filling continues immediately so the next frame's data is captured
        // (it goes to the same BRAM, possibly stomping on stale frame N data
        // — but stale-stomp is fine: USB has either read those addresses
        // already, or it hasn't and will read frame N+1's data at those
        // addresses, which is what the host wants when frames drop).
        if (!frame_filling && !frame_complete) begin
            frame_filling <= 1'b1;
        end

        // PR-Z A6 (Bug C) fix: detect_clearing was previously kicked off 1
        // cycle after frame_complete (right when frame_filling resumes). At
        // 1 byte/cycle it takes 8192 clk cycles (81.92 µs) — but cfar's
        // ST_CFAR_CMP starts emitting per-cell detect_valid pulses only
        // ~520 cycles after frame_complete and runs continuously for
        // ~73000 cycles. The first ~7672 cfar pulses (≈ first 4 doppler
        // columns) overlapped the clearing pass and were silently dropped
        // by the `!detect_clearing` guard on the RMW start condition,
        // wiping cells (range, doppler 0..3) from the wire.
        //
        // Trigger clearing on wr_done_pulse instead: that fires after USB
        // has finished reading the previous frame's detect BRAM, so the
        // clear runs in the dead zone between frames (~480k cycles wide
        // at 178 fps) and finishes long before the next frame's cfar CMP
        // starts. First frame after reset relies on BRAM init=0 (Vivado
        // default; SIM init below for iverilog).
        if (!detect_clearing && wr_done_pulse) begin
            detect_clearing   <= 1'b1;
            detect_clear_addr <= {DETECT_BYTE_ADDR_W{1'b0}};
        end
    end
end

// ============================================================================
// AUDIT-C12: frame_pending + frame_drop_count (clk domain)
// ============================================================================
// Tracks whether a frame is queued for USB transfer and counts dropped frames
// (frame_complete fires while previous frame still in WR_FSM transit). The
// counter is exposed in status_words[5] for host visibility.
//
// CDC: wr_done_toggle (ft_clk) flips on every WR_DONE → WR_IDLE; we 3-stage
// sync into clk and edge-detect to clear frame_pending.
// ============================================================================
reg        frame_pending;
reg [6:0]  frame_drop_count;   // 7-bit, saturates at 127

// wr_done_sync / wr_done_prev / wr_done_pulse declared earlier (~line 430)
// because the main writer block now uses wr_done_pulse for detect_clearing
// retrigger (PR-Z A6 Bug C fix).

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        frame_pending    <= 1'b0;
        frame_drop_count <= 7'd0;
        wr_done_sync     <= 3'b000;
        wr_done_prev     <= 1'b0;
    end else begin
        wr_done_sync <= {wr_done_sync[1:0], wr_done_toggle};
        wr_done_prev <= wr_done_sync[2];

        if (frame_complete) begin
            if (frame_pending && frame_drop_count != 7'd127)
                frame_drop_count <= frame_drop_count + 7'd1;
            frame_pending <= 1'b1;
        end else if (wr_done_pulse) begin
            frame_pending <= 1'b0;
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

// --- Stream control CDC (6-bit wire, but only [2:0] used in PR-G v2; [5:3] reserved=0).
//     2-stage level sync (changes infrequently). ---
(* ASYNC_REG = "TRUE" *) reg [5:0] stream_ctrl_sync_0;
(* ASYNC_REG = "TRUE" *) reg [5:0] stream_ctrl_sync_1;

// --- PR-G: 2-tier CFAR telemetry CDC (clk → ft_clk, 2-stage level sync).
//     Slow-changing per-frame values; sufficient for status readback. ---
(* ASYNC_REG = "TRUE" *) reg [7:0]  alpha_soft_sync_0;
reg [7:0]                           alpha_soft_sync_1;
(* ASYNC_REG = "TRUE" *) reg [16:0] det_thr_soft_sync_0;
reg [16:0]                          det_thr_soft_sync_1;
(* ASYNC_REG = "TRUE" *) reg [15:0] det_count_cand_sync_0;
reg [15:0]                          det_count_cand_sync_1;

// --- AUDIT-C12: frame_drop_count CDC (slow-changing 7-bit value, 2-stage sync) ---
(* ASYNC_REG = "TRUE" *) reg [6:0] frame_drop_sync_0;
reg [6:0]                          frame_drop_sync_1;

// --- AUDIT-S10: control-fault flag CDC (clk → ft_clk, 2-stage level sync) ---
// Sticky/slow-changing in source domain so 2-FF sync is sufficient.
(* ASYNC_REG = "TRUE" *) reg range_decim_watchdog_sync_0;
reg                          range_decim_watchdog_sync_1;
(* ASYNC_REG = "TRUE" *) reg ddc_cic_fir_overrun_sync_0;
reg                          ddc_cic_fir_overrun_sync_1;
// PR-AB.b expanded commit 5: 2-FF level CDC for the beam-handshake watchdog
// sticky. Same convention as the F-6.4 / F-1.2 stickies above.
(* ASYNC_REG = "TRUE" *) reg beam_handshake_wd_sync_0;
reg                          beam_handshake_wd_sync_1;

wire stream_range_en   = stream_ctrl_sync_1[0];
wire stream_doppler_en = stream_ctrl_sync_1[1];
wire stream_cfar_en    = stream_ctrl_sync_1[2];
// Bits [5:3] reserved=0 in PR-G v2. The legacy mag_only/sparse_det/frame_decimate
// flags were retired with v1 — there is one canonical encoding now (Manhattan-mag
// doppler + 2-bit dense detect).

// --- Frame metadata snapshot (latched in clk domain, stable for ft_clk read) ---
reg [15:0] frame_number_snapshot;
reg [2:0]  stream_flags_snapshot;     // PR-G: 3 bits used (range/doppler/cfar)
// PR-U / M-8: snapshot of host_subframe_enable taken at frame_complete so the
// host parser sees the mask that was active for THIS frame (atomic per-frame).
// Stable when ft_clk reads it via the frame_ready toggle synchronizer.
reg [2:0]  subframe_enable_snapshot;  // {LONG, MEDIUM, SHORT}

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        frame_number_snapshot    <= 16'd0;
        stream_flags_snapshot    <= 3'b111;  // PR-G: all 3 streams on (range|doppler|cfar)
        subframe_enable_snapshot <= 3'b111;  // PR-U: all 3 sub-frames on (production default)
    end else if (frame_complete) begin
        frame_number_snapshot    <= frame_number;
        stream_flags_snapshot    <= stream_control[2:0];  // PR-G: ignore reserved [5:3]
        subframe_enable_snapshot <= subframe_enable;
    end
end

// --- Status snapshot (ft_clk domain) — M-5: 8 words (was 7 / PR-G; was 6 / pre-PR-G) ---
reg [31:0] status_words [0:7];

// Byte counter for write FSM (needs to be wide enough for largest section)
reg [15:0] wr_byte_idx;

// BRAM read address for frame transfer
reg [RANGE_BIN_BITS-1:0]    range_rd_idx;     // Range section: 0..511
// PR-G: nested counters for doppler section so we emit only valid cells
// (range 0..511, doppler 0..47) and skip the BRAM padding at doppler 48..63.
reg [RANGE_BIN_BITS-1:0]    dop_range_idx;    // Doppler section outer: 0..511
reg [DOPPLER_BIN_BITS-1:0]  dop_doppler_idx;  // Doppler section inner: 0..47
reg                         wr_byte_phase;    // 0=MSB, 1=LSB for 16-bit values
// PR-G: nested counters for detect section so we emit only valid bytes
// (12 per range, doppler indices 0..47) and skip the 4 padded bytes from
// doppler 48..63. detect_rd_addr is composed from these.
reg [RANGE_BIN_BITS-1:0]    det_range_idx;        // 0..511
reg [3:0]                   det_doppler_byte_idx; // 0..11 (= NUM_DOPPLER_BINS*2/8 - 1)

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
        frame_drop_sync_0  <= 7'd0;
        frame_drop_sync_1  <= 7'd0;
        // AUDIT-S10: control-fault flag CDC reset
        range_decim_watchdog_sync_0 <= 1'b0;
        range_decim_watchdog_sync_1 <= 1'b0;
        ddc_cic_fir_overrun_sync_0  <= 1'b0;
        ddc_cic_fir_overrun_sync_1  <= 1'b0;
        beam_handshake_wd_sync_0    <= 1'b0;
        beam_handshake_wd_sync_1    <= 1'b0;
        // PR-G: 2-tier CFAR telemetry CDC reset
        alpha_soft_sync_0     <= 8'd0;
        alpha_soft_sync_1     <= 8'd0;
        det_thr_soft_sync_0   <= 17'd0;
        det_thr_soft_sync_1   <= 17'd0;
        det_count_cand_sync_0 <= 16'd0;
        det_count_cand_sync_1 <= 16'd0;
        for (si = 0; si < 8; si = si + 1)
            status_words[si] <= 32'd0;
        wr_state              <= WR_IDLE;
        wr_byte_idx           <= 16'd0;
        wr_byte_phase         <= 1'b0;
        dop_range_idx         <= {RANGE_BIN_BITS{1'b0}};
        dop_doppler_idx       <= {DOPPLER_BIN_BITS{1'b0}};
        range_rd_idx          <= {RANGE_BIN_BITS{1'b0}};
        range_rd_addr         <= {RANGE_BIN_BITS{1'b0}};
        det_range_idx         <= {RANGE_BIN_BITS{1'b0}};
        det_doppler_byte_idx  <= 4'd0;
        detect_rd_addr        <= {DETECT_BYTE_ADDR_W{1'b0}};
        mag_rd_addr           <= {FRAME_ADDR_W{1'b0}};
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
        wr_done_toggle <= 1'b0;
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

        // AUDIT-C12: frame_drop_count CDC (clk → ft_clk for status read)
        frame_drop_sync_0 <= frame_drop_count;
        frame_drop_sync_1 <= frame_drop_sync_0;

        // AUDIT-S10: control-fault flag CDC (clk → ft_clk for status read)
        range_decim_watchdog_sync_0 <= status_range_decim_watchdog;
        range_decim_watchdog_sync_1 <= range_decim_watchdog_sync_0;
        ddc_cic_fir_overrun_sync_0  <= status_ddc_cic_fir_overrun;
        ddc_cic_fir_overrun_sync_1  <= ddc_cic_fir_overrun_sync_0;
        beam_handshake_wd_sync_0    <= status_beam_handshake_watchdog;
        beam_handshake_wd_sync_1    <= beam_handshake_wd_sync_0;

        // PR-G: 2-tier CFAR telemetry CDC (clk → ft_clk for status read)
        alpha_soft_sync_0     <= status_cfar_alpha_soft;
        alpha_soft_sync_1     <= alpha_soft_sync_0;
        det_thr_soft_sync_0   <= status_detect_threshold_soft;
        det_thr_soft_sync_1   <= det_thr_soft_sync_0;
        det_count_cand_sync_0 <= status_detect_count_cand;
        det_count_cand_sync_1 <= det_count_cand_sync_0;

        // Status snapshot on request
        if (status_req_ft) begin
            // Word 0: {0xFF[31:24], reserved[23:22]=0, stream[21:16], threshold[15:0]}
            // mode bits retired in PR-AB.b expanded — single-mode FSM has no mode field.
            // Layout preserved for parser compatibility; bits [23:22] always 0.
            status_words[0] <= {8'hFF, 2'd0, status_stream_ctrl, status_cfar_threshold};
            status_words[1] <= {status_long_chirp, status_long_listen};
            status_words[2] <= {status_guard, status_short_chirp};
            status_words[3] <= {status_short_listen, 10'd0, status_chirps_per_elev};
            // Word 4 layout (post PR-AB.b expanded commit 5):
            //   [31:28] agc_current_gain
            //   [27:20] agc_peak_magnitude
            //   [19:12] agc_saturation_count
            //   [11]    agc_enable
            //   [10]    chirps_mismatch (TX-G)
            //   [9:2]   alpha_soft echo (Q4.4)
            //   [1]     beam_handshake_watchdog_fired (sticky, reset_n clear)
            //   [0]     reserved 0 (range_mode bit retired PR-AB.b expanded)
            status_words[4] <= {status_agc_current_gain,
                                status_agc_peak_magnitude,
                                status_agc_saturation_count,
                                status_agc_enable,
                                status_chirps_mismatch,
                                alpha_soft_sync_1,
                                beam_handshake_wd_sync_1,
                                1'd0};
            // Word 5: {frame_drop_count[31:25], self_test_busy[24],
            //          reserved[23:16], self_test_detail[15:8], reserved[7],
            //          cic_fir_overrun[6], range_decim_watchdog[5],
            //          self_test_flags[4:0]}
            // AUDIT-C12: bits [31:25] = frame_drop_count (silent frame drops).
            // AUDIT-S10: bits [6:5] expose control-fault classes that route to
            // gpio_dig7 — gives host visibility regardless of MCU consumption.
            status_words[5] <= {frame_drop_sync_1, status_self_test_busy,
                                8'd0, status_self_test_detail,
                                1'd0,                          // [7] reserved
                                ddc_cic_fir_overrun_sync_1,    // [6] audit F-1.2
                                range_decim_watchdog_sync_1,   // [5] audit F-6.4
                                status_self_test_flags};       // [4:0]
            // PR-G word 6: {detect_count_cand[15:0], detect_threshold_soft[15:0]}.
            // detect_threshold_soft is 17-bit; saturate to 16 bits for status (top
            // bit set → emit 0xFFFF). alpha_soft (8-bit) does not need to be in
            // the status packet — host wrote it via opcode 0x2D and tracks it
            // locally; it's CDC'd here for any future readback need but is not
            // emitted by the FSM today. (Pack into reserved bits if needed in v3.)
            status_words[6] <= {det_count_cand_sync_1,
                                (det_thr_soft_sync_1[16] ? 16'hFFFF
                                                         : det_thr_soft_sync_1[15:0])};
            // M-5 word 7: {medium_chirp[15:0], medium_listen[15:0]}.
            // Host writes via 0x17/0x18; this readback closes the GUI's
            // 161-µs MEDIUM PRI visibility gap that PR-G left when status word
            // 3 ran out of reserved bits to fit a second 16-bit pair.
            // CDC-wise this follows the same convention as long_chirp /
            // long_listen / short_chirp / short_listen above (status_words[1..3]):
            // direct sample of the clk-domain register on the ft_clk-domain
            // status_req_ft strobe, accepting the same quasi-static-write
            // assumption (host writes cycles once during init, no concurrent
            // change during a status read).
            status_words[7] <= {status_medium_chirp, status_medium_listen};
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
                        wr_state             <= WR_FRAME_HDR;
                        wr_byte_idx          <= 16'd0;
                        dop_range_idx        <= {RANGE_BIN_BITS{1'b0}};
                        dop_doppler_idx      <= {DOPPLER_BIN_BITS{1'b0}};
                        range_rd_idx         <= {RANGE_BIN_BITS{1'b0}};
                        range_rd_addr        <= {RANGE_BIN_BITS{1'b0}};   // Pre-load first read addr
                        det_range_idx        <= {RANGE_BIN_BITS{1'b0}};
                        det_doppler_byte_idx <= 4'd0;
                        detect_rd_addr       <= {DETECT_BYTE_ADDR_W{1'b0}};
                        mag_rd_addr          <= {FRAME_ADDR_W{1'b0}};     // {range=0, doppler=0}
                        wr_byte_phase        <= 1'b0;
                    end
                end

                // ---- Frame header: 9 bytes (PR-G: added version byte at offset 1) ----
                WR_FRAME_HDR: begin
                    if (!ft_txe_n) begin
                        ft_data_oe <= 1'b1;
                        ft_wr_n    <= 1'b0;

                        // PR-G: 9-byte header (was 8). Byte 1 = protocol version.
                        case (wr_byte_idx[3:0])
                            4'd0: ft_data_out <= HEADER;
                            4'd1: ft_data_out <= `RP_USB_PROTOCOL_VERSION;       // 0x02
                            // PR-U / M-8: byte 2 = {2'b00, subframe_enable[2:0], stream_flags[2:0]}.
                            // Was {5'b00000, stream_flags_snapshot}; bits[5:3] now carry
                            // the per-frame sub-frame mask snapshot {LONG, MEDIUM, SHORT}.
                            4'd2: ft_data_out <= {2'b00, subframe_enable_snapshot, stream_flags_snapshot};
                            4'd3: ft_data_out <= frame_number_snapshot[15:8];
                            4'd4: ft_data_out <= frame_number_snapshot[7:0];
                            4'd5: ft_data_out <= NUM_RANGE_BINS[15:8];    // 512 >> 8 = 2
                            4'd6: ft_data_out <= NUM_RANGE_BINS[7:0];     // 512 & 0xFF = 0
                            4'd7: ft_data_out <= NUM_DOPPLER_BINS[15:8];  // 48 >> 8 = 0
                            4'd8: ft_data_out <= NUM_DOPPLER_BINS[7:0];   // 48 & 0xFF = 48
                            default: ft_data_out <= 8'h00;
                        endcase

                        if (wr_byte_idx[3:0] == 4'd8) begin
                            wr_byte_idx   <= 16'd0;
                            wr_byte_phase <= 1'b0;
                            // PR-Z A6 (Bug B) fix: BRAM read has 1-cycle latency.
                            // Pre-load detect_rd_addr=1 and det_doppler_byte_idx=1
                            // so the first WR_DETECT_DATA cycle emits bram[0]
                            // (already settled at addr 0 since WR_IDLE) while
                            // BRAM begins fetching bram[1] for the second emit.
                            // Harmless when next state is not WR_DETECT_DATA.
                            det_doppler_byte_idx <= 4'd1;
                            detect_rd_addr       <= {{(DETECT_BYTE_ADDR_W-1){1'b0}}, 1'b1};
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

                        // PR-AA: addr advance lives at end of phase 0 (MSB emit), not
                        // phase 1 (LSB emit). With BRAM 1-cycle read latency, a 2-byte
                        // pair needs 2 cycles between addr-set and the next pair's MSB
                        // read; advancing at phase 1 (1 cycle gap) leaves the next MSB
                        // reading the prior cell's high byte. See WR_DOPPLER_DATA below.
                        if (!wr_byte_phase) begin
                            ft_data_out   <= range_rd_data[15:8];
                            wr_byte_phase <= 1'b1;
                            range_rd_idx  <= range_rd_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
                            range_rd_addr <= range_rd_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
                        end else begin
                            ft_data_out   <= range_rd_data[7:0];
                            wr_byte_phase <= 1'b0;
                        end

                        wr_byte_idx <= wr_byte_idx + 16'd1;

                        if (wr_byte_idx == RANGE_SECTION_BYTES[15:0] - 16'd1) begin
                            wr_byte_idx     <= 16'd0;
                            wr_byte_phase   <= 1'b0;
                            dop_range_idx   <= {RANGE_BIN_BITS{1'b0}};
                            dop_doppler_idx <= {DOPPLER_BIN_BITS{1'b0}};
                            mag_rd_addr     <= {FRAME_ADDR_W{1'b0}};  // {range=0, doppler=0}
                            // PR-Z A6 (Bug B) fix: pre-load detect read pipeline
                            // when bypassing doppler (stream_flags[1]=0). See
                            // WR_FRAME_HDR exit comment for details.
                            det_doppler_byte_idx <= 4'd1;
                            detect_rd_addr       <= {{(DETECT_BYTE_ADDR_W-1){1'b0}}, 1'b1};
                            if (stream_flags_snapshot[1])
                                wr_state <= WR_DOPPLER_DATA;
                            else if (stream_flags_snapshot[2])
                                wr_state <= WR_DETECT_DATA;
                            else
                                wr_state <= WR_FRAME_FOOTER;
                        end
                    end
                end

                // ---- Doppler magnitude: 512 × 48 × 2 = 49152 bytes ----
                // PR-G: row-major iteration over valid (range, doppler) cells
                // only. Skips BRAM padding at doppler 48..63 by jumping to next
                // range when doppler hits DOP_BIN_LAST. Header field
                // doppler_bins=48 matches body length exactly.
                WR_DOPPLER_DATA: begin
                    if (!ft_txe_n) begin
                        ft_data_oe <= 1'b1;
                        ft_wr_n    <= 1'b0;

                        // PR-AA fix: BRAM read has 1-cycle latency. A 2-byte pair
                        // emits MSB then LSB from the SAME cell, so addr must advance
                        // at end of phase 0 (MSB) — that gives BRAM 2 cycles before
                        // the next pair's MSB needs the new cell:
                        //   cycle K   (phase 0): data=bram[addr_{K-1}]=bram[N], emit H(N), advance addr<=N+1
                        //   cycle K+1 (phase 1): data=bram[addr_K]=bram[N], emit L(N)
                        //   cycle K+2 (phase 0): data=bram[addr_{K+1}]=bram[N+1], emit H(N+1)
                        // Previous (broken) pattern advanced at phase 1, so phase 0 of
                        // the next pair re-read bram[N] and emitted H(N) again, leaving
                        // the wire pair-K = (HIGH(bram[K-1]), LOW(bram[K])).
                        if (!wr_byte_phase) begin
                            ft_data_out   <= mag_rd_data[15:8];
                            wr_byte_phase <= 1'b1;
                            // Address layout: {range[8:0], doppler[5:0]}.
                            if (dop_doppler_idx == DOP_BIN_LAST) begin
                                dop_doppler_idx <= {DOPPLER_BIN_BITS{1'b0}};
                                dop_range_idx   <= dop_range_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
                                mag_rd_addr     <= {dop_range_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1},
                                                    {DOPPLER_BIN_BITS{1'b0}}};
                            end else begin
                                dop_doppler_idx <= dop_doppler_idx + {{(DOPPLER_BIN_BITS-1){1'b0}}, 1'b1};
                                mag_rd_addr     <= {dop_range_idx,
                                                    dop_doppler_idx + {{(DOPPLER_BIN_BITS-1){1'b0}}, 1'b1}};
                            end
                        end else begin
                            ft_data_out   <= mag_rd_data[7:0];
                            wr_byte_phase <= 1'b0;
                        end

                        wr_byte_idx <= wr_byte_idx + 16'd1;

                        if (wr_byte_idx == DOPPLER_MAG_SECTION_BYTES[15:0] - 16'd1) begin
                            wr_byte_idx          <= 16'd0;
                            wr_byte_phase        <= 1'b0;
                            det_range_idx        <= {RANGE_BIN_BITS{1'b0}};
                            // PR-Z A6 (Bug B) fix: BRAM read has 1-cycle latency.
                            // Pre-load detect_rd_addr=1 and det_doppler_byte_idx=1
                            // so the first WR_DETECT_DATA cycle emits bram[0]
                            // (already settled — detect_rd_addr was 0 since
                            // WR_IDLE) while BRAM begins fetching bram[1] for
                            // the second emit. Without this pre-load the wire
                            // shifts +1 byte (= +4 doppler bins) across the
                            // entire detect section.
                            det_doppler_byte_idx <= 4'd1;
                            detect_rd_addr       <= {{(DETECT_BYTE_ADDR_W-1){1'b0}}, 1'b1};
                            if (stream_flags_snapshot[2])
                                wr_state <= WR_DETECT_DATA;
                            else
                                wr_state <= WR_FRAME_FOOTER;
                        end
                    end
                end

                // ---- Detection flags: 512 × 12 = 6144 bytes (PR-G, 2-bit dense) ----
                // PR-G: nested advance through (range 0..511, doppler_byte 0..11).
                // Skips 4 padded detect bytes per range (doppler 48..63 indices)
                // so the wire body matches host's expected size of
                // range_bins × doppler_bins × 2 / 8 = 6144 bytes.
                WR_DETECT_DATA: begin
                    if (!ft_txe_n) begin
                        ft_data_oe <= 1'b1;
                        ft_wr_n    <= 1'b0;

                        // 1-byte per cycle (BRAM read latency handled by pre-loading addr)
                        ft_data_out <= detect_rd_data;
                        if (det_doppler_byte_idx == DET_BYTE_LAST_PER_RANGE) begin
                            det_doppler_byte_idx <= 4'd0;
                            det_range_idx        <= det_range_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1};
                            detect_rd_addr       <= {det_range_idx + {{(RANGE_BIN_BITS-1){1'b0}}, 1'b1},
                                                     4'd0};
                        end else begin
                            det_doppler_byte_idx <= det_doppler_byte_idx + 4'd1;
                            detect_rd_addr       <= {det_range_idx,
                                                     det_doppler_byte_idx + 4'd1};
                        end

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

                // ---- Status packet: 34 bytes (M-5: 8 × 32-bit words) ----
                WR_STATUS_SEND: begin
                    if (!ft_txe_n) begin
                        ft_data_oe <= 1'b1;
                        ft_wr_n    <= 1'b0;

                        case (wr_byte_idx[5:0])
                            6'd0:  ft_data_out <= STATUS_HEADER;
                            6'd1:  ft_data_out <= status_words[0][31:24];
                            6'd2:  ft_data_out <= status_words[0][23:16];
                            6'd3:  ft_data_out <= status_words[0][15:8];
                            6'd4:  ft_data_out <= status_words[0][7:0];
                            6'd5:  ft_data_out <= status_words[1][31:24];
                            6'd6:  ft_data_out <= status_words[1][23:16];
                            6'd7:  ft_data_out <= status_words[1][15:8];
                            6'd8:  ft_data_out <= status_words[1][7:0];
                            6'd9:  ft_data_out <= status_words[2][31:24];
                            6'd10: ft_data_out <= status_words[2][23:16];
                            6'd11: ft_data_out <= status_words[2][15:8];
                            6'd12: ft_data_out <= status_words[2][7:0];
                            6'd13: ft_data_out <= status_words[3][31:24];
                            6'd14: ft_data_out <= status_words[3][23:16];
                            6'd15: ft_data_out <= status_words[3][15:8];
                            6'd16: ft_data_out <= status_words[3][7:0];
                            6'd17: ft_data_out <= status_words[4][31:24];
                            6'd18: ft_data_out <= status_words[4][23:16];
                            6'd19: ft_data_out <= status_words[4][15:8];
                            6'd20: ft_data_out <= status_words[4][7:0];
                            6'd21: ft_data_out <= status_words[5][31:24];
                            6'd22: ft_data_out <= status_words[5][23:16];
                            6'd23: ft_data_out <= status_words[5][15:8];
                            6'd24: ft_data_out <= status_words[5][7:0];
                            6'd25: ft_data_out <= status_words[6][31:24];   // PR-G
                            6'd26: ft_data_out <= status_words[6][23:16];   // PR-G
                            6'd27: ft_data_out <= status_words[6][15:8];    // PR-G
                            6'd28: ft_data_out <= status_words[6][7:0];     // PR-G
                            6'd29: ft_data_out <= status_words[7][31:24];   // M-5: medium_chirp[15:8]
                            6'd30: ft_data_out <= status_words[7][23:16];   // M-5: medium_chirp[7:0]
                            6'd31: ft_data_out <= status_words[7][15:8];    // M-5: medium_listen[15:8]
                            6'd32: ft_data_out <= status_words[7][7:0];     // M-5: medium_listen[7:0]
                            6'd33: ft_data_out <= FOOTER;
                            default: ft_data_out <= 8'h00;
                        endcase

                        if (wr_byte_idx[5:0] == STATUS_PKT_LEN - 6'd1) begin
                            wr_state    <= WR_DONE;
                            wr_byte_idx <= 16'd0;
                        end else begin
                            wr_byte_idx <= wr_byte_idx + 16'd1;
                        end
                    end
                end

                WR_DONE: begin
                    ft_wr_n        <= 1'b1;
                    ft_data_oe     <= 1'b0;
                    wr_done_toggle <= ~wr_done_toggle;  // AUDIT-C12: signal frame transfer complete to clk domain
                    wr_state       <= WR_IDLE;
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end
end

// ============================================================================
// SIMULATION ONLY: BRAM init
// ============================================================================
// Vivado-inferred BRAM18 initializes to all-zero by default in synthesis,
// but iverilog leaves `reg [...] mem [...]` at X. The Bug C fix (detect
// clearing now triggers on wr_done_pulse, not frame_complete + 1) means
// the first frame after reset relies on this init to give clean cells —
// otherwise wire bytes that cfar never wrote to would read X.
// ============================================================================
`ifdef SIMULATION
integer init_idx;
initial begin
    for (init_idx = 0; init_idx <= DETECT_BYTE_LAST; init_idx = init_idx + 1)
        detect_bram[init_idx] = 8'd0;
    for (init_idx = 0; init_idx < FRAME_CELLS; init_idx = init_idx + 1)
        doppler_mag_bram[init_idx] = 16'd0;
    for (init_idx = 0; init_idx < NUM_RANGE_BINS; init_idx = init_idx + 1)
        range_bram[init_idx] = 16'd0;
end
`endif

// ============================================================================
// TX-N9: payload-hold checker (simulation only)
//
// The cmd_* outputs feed a CDC sync chain in the consumer clock domain.
// Safety property: cmd_data / cmd_opcode / cmd_addr / cmd_value must only
// change on the cycle that cmd_valid rises (RD_PROCESS in this module).
// On every other cycle they must be held stable so the receiver's 2-FF
// synchronizer sees a clean payload regardless of where its sample window
// lands relative to cmd_valid.
//
// The current FSM satisfies this implicitly — the payload regs are only
// written in RD_PROCESS, never elsewhere — but until now nothing flagged
// a regression that introduced a stray write somewhere in the always
// block. This checker fires `[ASSERT FAIL]` on any payload change while
// cmd_valid is low, surfacing the violation in the simulation log
// without affecting synthesis.
// ============================================================================
`ifdef SIMULATION
reg [31:0] cmd_data_prev;
reg  [7:0] cmd_opcode_prev;
reg  [7:0] cmd_addr_prev;
reg [15:0] cmd_value_prev;
reg        cmd_valid_prev;

always @(posedge ft_clk or negedge ft_reset_n) begin
    if (!ft_reset_n) begin
        cmd_data_prev   <= 32'd0;
        cmd_opcode_prev <= 8'd0;
        cmd_addr_prev   <= 8'd0;
        cmd_value_prev  <= 16'd0;
        cmd_valid_prev  <= 1'b0;
    end else begin
        // Payload may legally change only on the cycle cmd_valid rises
        // (cmd_valid_prev=0, cmd_valid=1). Any other change is a hold
        // violation.
        if (!cmd_valid && !cmd_valid_prev) begin
            if (cmd_data   !== cmd_data_prev)
                $display("[ASSERT FAIL] TX-N9: cmd_data changed while cmd_valid=0 (%h -> %h)",
                         cmd_data_prev, cmd_data);
            if (cmd_opcode !== cmd_opcode_prev)
                $display("[ASSERT FAIL] TX-N9: cmd_opcode changed while cmd_valid=0 (%h -> %h)",
                         cmd_opcode_prev, cmd_opcode);
            if (cmd_addr   !== cmd_addr_prev)
                $display("[ASSERT FAIL] TX-N9: cmd_addr changed while cmd_valid=0 (%h -> %h)",
                         cmd_addr_prev, cmd_addr);
            if (cmd_value  !== cmd_value_prev)
                $display("[ASSERT FAIL] TX-N9: cmd_value changed while cmd_valid=0 (%h -> %h)",
                         cmd_value_prev, cmd_value);
        end
        cmd_data_prev   <= cmd_data;
        cmd_opcode_prev <= cmd_opcode;
        cmd_addr_prev   <= cmd_addr;
        cmd_value_prev  <= cmd_value;
        cmd_valid_prev  <= cmd_valid;
    end
end
`endif

// ============================================================================
// AUDIT-S22: cfar_valid-vs-RMW-busy checker (simulation only)
//
// Detection RMW (idle→read→write-back) takes 3 cycles. cfar_ca emits one
// detect_valid pulse per 3 cycles (THR/MUL/CMP pipeline). They match by
// construction today — line 469 silently rejects cfar_valid arriving when
// detect_rmw_state != 0, which never fires at the current cadence.
//
// If cfar_ca is ever optimized to <3-cycle cadence (e.g., merging MUL+CMP,
// flagged as a possible target in cfar_ca.v), the silent rejection becomes
// a silent detection-drop. This assertion makes that violation loud, so the
// regression suite catches the coupling on the day someone speeds CFAR up
// without also pipelining the RMW. Synthesis-inert.
// ============================================================================
`ifdef SIMULATION
always @(posedge clk) begin
    if (reset_n && cfar_valid && frame_filling && !detect_clearing &&
        detect_rmw_state != 2'd0) begin
        $display("[ASSERT FAIL] AUDIT-S22: cfar_valid arrived while RMW busy (state=%0d) — detection at range_bin=%0d doppler_bin=%0d dropped",
                 detect_rmw_state, range_bin_in, doppler_bin_in);
    end
end
`endif

endmodule
