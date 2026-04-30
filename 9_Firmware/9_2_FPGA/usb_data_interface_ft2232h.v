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
 *   [If stream_doppler (bit 1):]
 *     Next 32768 bytes: Doppler magnitude, 512×32 × 16-bit, row-major, MSB first
 *
 *   [If stream_cfar (bit 2):]
 *     Next 2048 bytes: Detection flags, 512×32 bits packed into bytes, MSB-first bit order
 *
 *   Last byte:    0x55 (frame end footer)
 *
 * INERT FLAGS — mag_only (bit 3) and sparse_det (bit 4) (AUDIT-C9):
 *   The wire format byte 1 reserves these two bits for future encodings:
 *     - mag_only=0 was meant to switch the doppler section to 65536 B
 *       full-I/Q (16-bit I + 16-bit Q per cell, row-major, MSB first).
 *     - sparse_det=1 was meant to switch the CFAR section to a
 *       variable-length list: 2 B count N + N×6 B (range, doppler, mag).
 *   Neither encoding is implemented in the write FSM below — the FSM
 *   always emits 32768 B mag and 2048 B dense bitmap regardless of the
 *   flag bits. To eliminate the foot-gun, `radar_system_top.v` opcode
 *   0x04 force-clamps mag_only=1 and sparse_det=0 in `host_stream_control`
 *   when USB_MODE=1. A SIMULATION-only assertion at the bottom of this
 *   module fires if either bit ever leaves its clamped value, in case a
 *   future patch adds a path that bypasses the host register clamp.
 *
 *   Reasons differ between the two:
 *     - Full-I/Q is constrained by FPGA resources: it needs a new
 *       ~28-BRAM18 I/Q buffer (16384 cells × 32-bit) which may not fit
 *       on the 50T (currently ~78% BRAM18 utilisation after wiring the
 *       Xilinx FFT IP). USB 2.0 bandwidth is also tight: 12.21 MB/s vs
 *       the conservative 8 MB/s sustained budget. Both gating items.
 *     - Sparse-list is feasible — bandwidth-wise it's smaller than the
 *       dense bitmap for any frame with fewer than ~341 detections
 *       (typical scenes produce 10-200), and memory-wise it costs
 *       ~1 BRAM18 with MAX_DETECTIONS=256. The absence is just
 *       unimplemented RTL work (a small detection-list BRAM + a new
 *       WR_DETECT_SPARSE FSM state), not a hardware constraint.
 *   See the open-defects ledger for the follow-up work items.
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
 *
 * BANDWIDTH BUDGET (current production: mag_only=1, all streams):
 *   Header: 8 B + Range: 1024 B + Doppler: 32768 B + CFAR: 2048 B + Footer: 1 B
 *   = 35,849 bytes/frame × ~178 fps = 6.38 MB/s
 *   FT2232H 245-Sync-FIFO sustained budget ~8 MB/s conservative (FTDI
 *   AN_232B-04). 80% utilisation; full-I/Q (12.21 MB/s) would not fit at
 *   the conservative budget and is why mag_only is force-clamped to 1.
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
    input wire        status_ddc_cic_fir_overrun    // audit F-1.2
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
localparam DETECT_BYTE_ADDR_W = FRAME_ADDR_W - 3;                          // 12
localparam DETECT_BYTE_LAST   = (FRAME_CELLS / 8) - 1;                     // 4095

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

(* ram_style = "block" *) reg [7:0] detect_bram [0:DETECT_BYTE_LAST];  // PR-F: 3072 entries (was 2048)

reg [DETECT_BYTE_ADDR_W-1:0] detect_wr_addr;     // PR-F: 12-bit byte addr (was 11)
reg [7:0]  detect_wr_data;
reg        detect_wr_en;

always @(posedge clk) begin
    if (detect_wr_en)
        detect_bram[detect_wr_addr] <= detect_wr_data;
end

reg [DETECT_BYTE_ADDR_W-1:0] detect_rd_addr;     // PR-F: 12-bit byte addr (was 11)
reg [7:0]                     detect_rd_data;

always @(posedge ft_clk) begin
    detect_rd_data <= detect_bram[detect_rd_addr];
end

// Detection BRAM read-modify-write pipeline (clk domain)
reg [DETECT_BYTE_ADDR_W-1:0] detect_rmw_addr;    // PR-F: 12-bit byte addr (was 11)
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
reg [FRAME_ADDR_W-1:0] detect_clear_addr;   // PR-F: 15-bit bit-counter; FRAME_CELLS = NUM_RANGE_BINS * (1<<DOPPLER_BIN_BITS) = 32768 (full 15-bit space)
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
        mag_wr_addr        <= {FRAME_ADDR_W{1'b0}};
        mag_wr_data        <= 16'd0;
        range_wr_en        <= 1'b0;
        range_wr_addr      <= {RANGE_BIN_BITS{1'b0}};
        range_wr_data      <= 16'd0;
        detect_wr_en       <= 1'b0;
        detect_wr_addr     <= {DETECT_BYTE_ADDR_W{1'b0}};
        detect_wr_data     <= 8'd0;
        detect_clearing    <= 1'b0;
        detect_clear_addr  <= {FRAME_ADDR_W{1'b0}};
        detect_rmw_state   <= 2'd0;
        detect_rmw_addr    <= {DETECT_BYTE_ADDR_W{1'b0}};
        detect_rmw_bit     <= 3'd0;
        detect_rmw_value   <= 1'b0;
        range_write_counter <= {RANGE_BIN_BITS{1'b0}};
    end else begin
        // Default: deassert write enables
        mag_wr_en    <= 1'b0;
        range_wr_en  <= 1'b0;
        detect_wr_en <= 1'b0;

        // === Detection BRAM bulk clear (runs after frame_complete) ===
        // PR-F: bit-counter is FRAME_ADDR_W (15-bit), byte addr is the upper
        // (FRAME_ADDR_W-3) bits.
        if (detect_clearing) begin
            detect_wr_en   <= 1'b1;
            detect_wr_addr <= detect_clear_addr[FRAME_ADDR_W-1:3];
            detect_wr_data <= 8'd0;
            if (detect_clear_addr[FRAME_ADDR_W-1:3] == DETECT_BYTE_LAST[DETECT_BYTE_ADDR_W-1:0]) begin
                detect_clearing   <= 1'b0;
                detect_clear_addr <= {FRAME_ADDR_W{1'b0}};
            end else begin
                detect_clear_addr <= detect_clear_addr + 15'd8;  // Step by 8 bits = 1 byte
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
        // PR-F: bit_addr = {range_bin_in[8:0], doppler_bin_in[5:0]} = 15-bit.
        // byte_addr = bit_addr[14:3] (12 bits), bit_pos = bit_addr[2:0].
        if (cfar_valid && frame_filling && detect_rmw_state == 2'd0 && !detect_clearing) begin
            detect_rmw_addr  <= {range_bin_in, doppler_bin_in[DOPPLER_BIN_BITS-1:3]};
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
            frame_filling     <= 1'b1;
            detect_clearing   <= 1'b1;  // Clear detection BRAM for next frame
            detect_clear_addr <= {FRAME_ADDR_W{1'b0}};
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

(* ASYNC_REG = "TRUE" *) reg [2:0] wr_done_sync;
reg                                wr_done_prev;
wire                               wr_done_pulse = wr_done_sync[2] ^ wr_done_prev;

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

// --- Stream control CDC (6-bit, 2-stage level sync) ---
(* ASYNC_REG = "TRUE" *) reg [5:0] stream_ctrl_sync_0;
(* ASYNC_REG = "TRUE" *) reg [5:0] stream_ctrl_sync_1;

// --- AUDIT-C12: frame_drop_count CDC (slow-changing 7-bit value, 2-stage sync) ---
(* ASYNC_REG = "TRUE" *) reg [6:0] frame_drop_sync_0;
reg [6:0]                          frame_drop_sync_1;

// --- AUDIT-S10: control-fault flag CDC (clk → ft_clk, 2-stage level sync) ---
// Sticky/slow-changing in source domain so 2-FF sync is sufficient.
(* ASYNC_REG = "TRUE" *) reg range_decim_watchdog_sync_0;
reg                          range_decim_watchdog_sync_1;
(* ASYNC_REG = "TRUE" *) reg ddc_cic_fir_overrun_sync_0;
reg                          ddc_cic_fir_overrun_sync_1;

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
reg [FRAME_ADDR_W-1:0] bram_rd_cell;     // PR-F: 15-bit cell index 0..24575
reg [RANGE_BIN_BITS-1:0] range_rd_idx;   // Range bin index 0..511
reg        wr_byte_phase;                // 0=MSB, 1=LSB for 16-bit values
reg [DETECT_BYTE_ADDR_W-1:0] detect_rd_idx; // PR-F: 12-bit byte index 0..3071

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
        for (si = 0; si < 6; si = si + 1)
            status_words[si] <= 32'd0;
        wr_state       <= WR_IDLE;
        wr_byte_idx    <= 16'd0;
        wr_byte_phase  <= 1'b0;
        bram_rd_cell   <= {FRAME_ADDR_W{1'b0}};
        range_rd_idx   <= {RANGE_BIN_BITS{1'b0}};
        range_rd_addr  <= {RANGE_BIN_BITS{1'b0}};
        detect_rd_idx  <= {DETECT_BYTE_ADDR_W{1'b0}};
        detect_rd_addr <= {DETECT_BYTE_ADDR_W{1'b0}};
        mag_rd_addr    <= {FRAME_ADDR_W{1'b0}};
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

        // Status snapshot on request
        if (status_req_ft) begin
            // Word 0: {0xFF, mode[1:0], stream[5:0], threshold[15:0]}
            // NOTE: stream_ctrl now 6-bit. Pack as: {0xFF, mode, stream[5:3], stream[2:0], threshold}
            // Keep backward-compatible layout: {0xFF[31:24], mode[23:22], stream[21:16], threshold[15:0]}
            status_words[0] <= {8'hFF, status_radar_mode, status_stream_ctrl, status_cfar_threshold};
            status_words[1] <= {status_long_chirp, status_long_listen};
            status_words[2] <= {status_guard, status_short_chirp};
            status_words[3] <= {status_short_listen, 10'd0, status_chirps_per_elev};
            status_words[4] <= {status_agc_current_gain,        // [31:28]
                                status_agc_peak_magnitude,      // [27:20]
                                status_agc_saturation_count,    // [19:12]
                                status_agc_enable,              // [11]
                                status_chirps_mismatch,         // [10] TX-G mismatch flag
                                8'd0,                           // [9:2] reserved
                                status_range_mode};             // [1:0]
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
                        bram_rd_cell  <= {FRAME_ADDR_W{1'b0}};
                        range_rd_idx  <= {RANGE_BIN_BITS{1'b0}};
                        range_rd_addr <= {RANGE_BIN_BITS{1'b0}};  // Pre-load first read addr
                        detect_rd_idx <= {DETECT_BYTE_ADDR_W{1'b0}};
                        detect_rd_addr <= {DETECT_BYTE_ADDR_W{1'b0}};
                        mag_rd_addr   <= {FRAME_ADDR_W{1'b0}};
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
                            bram_rd_cell  <= {FRAME_ADDR_W{1'b0}};
                            mag_rd_addr   <= {FRAME_ADDR_W{1'b0}};
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
                            bram_rd_cell  <= bram_rd_cell + {{(FRAME_ADDR_W-1){1'b0}}, 1'b1};
                            mag_rd_addr   <= bram_rd_cell + {{(FRAME_ADDR_W-1){1'b0}}, 1'b1};
                        end

                        wr_byte_idx <= wr_byte_idx + 16'd1;

                        if (wr_byte_idx == DOPPLER_MAG_SECTION_BYTES[15:0] - 16'd1) begin
                            wr_byte_idx    <= 16'd0;
                            wr_byte_phase  <= 1'b0;
                            detect_rd_idx  <= {DETECT_BYTE_ADDR_W{1'b0}};
                            detect_rd_addr <= {DETECT_BYTE_ADDR_W{1'b0}};
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
                        detect_rd_idx  <= detect_rd_idx + {{(DETECT_BYTE_ADDR_W-1){1'b0}}, 1'b1};
                        detect_rd_addr <= detect_rd_idx + {{(DETECT_BYTE_ADDR_W-1){1'b0}}, 1'b1};

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
// AUDIT-C9: inert-flag checker (simulation only)
//
// stream_mag_only and stream_sparse_det are documented in the wire format
// but the write FSM does not act on them — see the "INERT FLAGS" note in
// the module header. radar_system_top.v opcode 0x04 force-clamps these
// bits when USB_MODE=1 so production firmware cannot reach an unsupported
// state. This checker is the backstop: it fires `[ASSERT FAIL]` if either
// bit ever escapes its clamped value, catching any future patch that
// bypasses the host register clamp (e.g. a different opcode that writes
// stream_control directly, or a stream_control source other than the
// host). Synthesis-inert.
// ============================================================================
`ifdef SIMULATION
always @(posedge clk) begin
    if (reset_n) begin
        if (stream_mag_only !== 1'b1)
            $display("[ASSERT FAIL] AUDIT-C9: stream_mag_only=0; full-I/Q write FSM not implemented");
        if (stream_sparse_det !== 1'b0)
            $display("[ASSERT FAIL] AUDIT-C9: stream_sparse_det=1; sparse-list write FSM not implemented");
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
