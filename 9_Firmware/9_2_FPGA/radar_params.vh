// ============================================================================
// radar_params.vh — Single Source of Truth for AERIS-10 FPGA Parameters
// ============================================================================
//
// ALL modules in the FPGA processing chain MUST `include this file instead of
// hardcoding range bins, segment counts, chirp samples, or timing values.
//
// This file uses `define macros (not localparam) so it can be included at any
// scope. Each consuming module should include this file inside its body and
// optionally alias macros to localparams for readability.
//
// BOARD VARIANTS:
//   SUPPORT_LONG_RANGE = 0  (50T, USB_MODE=1)  — 3 km mode only
//   SUPPORT_LONG_RANGE = 1  (200T, USB_MODE=0) — 3 km + 20 km modes
//
// RADAR MODES (runtime, via host_radar_mode register, opcode 0x01):
//   2'b00 = STM32 pass-through (production — STM32 controls chirp timing)
//   2'b01 = Auto-scan 3 km     (FPGA-timed, short chirps only)
//   2'b10 = Single-chirp debug (one long chirp per trigger)
//   2'b11 = Reserved / idle
//
// RANGE MODES (runtime, via host_range_mode register, opcode 0x20):
//   2'b00 = 3 km   (default — pass-through treats all chirps as short)
//   2'b01 = Long-range (pass-through: first half long, second half short)
//   2'b10 = Reserved
//   2'b11 = Reserved
//
// USAGE:
//   `include "radar_params.vh"
//   Then reference `RP_FFT_SIZE, `RP_NUM_RANGE_BINS, etc.
//
// PHYSICAL CONSTANTS (derived from hardware):
//   ADC clock:           400 MSPS
//   CIC decimation:      4x
//   Processing rate:     100 MSPS (post-DDC)
//   Range per sample:    c / (2 * 100e6) = 1.5 m
//   FFT size:            2048
//   Decimation factor:   4  (2048 FFT bins -> 512 output range bins)
//   Range per dec. bin:  1.5 m * 4 = 6.0 m
//   Max range (3 km):    512 * 6.0 = 3072 m
//   Carrier frequency:   10.5 GHz
//   IF frequency:        120 MHz
//
// CHIRP BANDWIDTH (Phase 1 target — currently 20 MHz, planned 30 MHz):
//   Range resolution:    c / (2 * BW)
//     20 MHz -> 7.5 m
//     30 MHz -> 5.0 m
//   NOTE: Range resolution is independent of range-per-bin. Resolution
//   determines the minimum separation between two targets; range-per-bin
//   determines the spatial sampling grid.
// ============================================================================

`ifndef RADAR_PARAMS_VH
`define RADAR_PARAMS_VH

// ============================================================================
// BOARD VARIANT — set at synthesis time, NOT runtime
// ============================================================================
// Default to 50T (conservative). Override in top-level or synthesis script:
//   +define+SUPPORT_LONG_RANGE
// or via Vivado: set_property verilog_define {SUPPORT_LONG_RANGE} [current_fileset]

// Note: SUPPORT_LONG_RANGE is a flag define (ifdef/ifndef), not a value.
// `ifndef SUPPORT_LONG_RANGE means 50T (no long range).
// `ifdef SUPPORT_LONG_RANGE means 200T (long range supported).

// ============================================================================
// FFT AND PROCESSING CONSTANTS (fixed, both modes)
// ============================================================================

`define RP_FFT_SIZE             2048    // Range FFT points per segment
`define RP_LOG2_FFT_SIZE        11      // log2(2048)
`define RP_OVERLAP_SAMPLES      128     // Overlap between adjacent segments
`define RP_SEGMENT_ADVANCE      1920    // FFT_SIZE - OVERLAP = 2048 - 128
`define RP_DECIMATION_FACTOR    4       // Range bin decimation (2048 -> 512)
`define RP_NUM_RANGE_BINS       512     // FFT_SIZE / DECIMATION_FACTOR
`define RP_RANGE_BIN_BITS       9       // ceil(log2(512))
`define RP_DOPPLER_FFT_SIZE     16      // Per sub-frame Doppler FFT (scan mode)
`define RP_DOPPLER_FFT_SIZE_TRACK 64    // Track-mode dwell N (xfft_64, single waveform)
`define RP_CHIRPS_PER_FRAME     48      // 3 sub-frames * 16 chirps = 48 (PR-F)
`define RP_CHIRPS_PER_SUBFRAME  16      // Chirps per Doppler sub-frame
`define RP_NUM_DOPPLER_BINS     48      // 3 sub-frames * 16 bins = 48 (PR-F)
`define RP_DATA_WIDTH           16      // ADC/processing data width

// 3-ladder waveform identity (replaces 1-bit use_long_chirp rail in PR-C onward)
// `define RP_WAVE_<NAME> values are 2-bit waveform selectors carried on
// `wave_sel[1:0]` at every chirp boundary. RESERVED is a hard error.
`define RP_WAVE_SEL_WIDTH       2
`define RP_WAVE_SHORT           2'b00   // 1 µs    (3 km build workhorse)
`define RP_WAVE_MEDIUM          2'b01   // 5 µs    (mid-range fill, 0.75–8 km)
`define RP_WAVE_LONG            2'b10   // 30 µs   (legal but unused on 50T; 200T uses for 20 km)
`define RP_WAVE_RESERVED        2'b11

// Sub-frame layout. Frame = NUM_SUBFRAMES × CHIRPS_PER_SUBFRAME chirps.
// Scan mode uses 3 sub-frames (SHORT, MEDIUM, LONG), each running its own
// N=16 Doppler FFT. Track mode pins the frame to one waveform and runs N=64.
`define RP_NUM_SUBFRAMES        3
`define RP_SUBFRAME_ID_WIDTH    2       // ceil(log2(3))
`define RP_DOPPLER_BIN_WIDTH    6       // {sub_frame[1:0], bin[3:0]}

// Adaptive-escalation detection class (CFAR output — 2-class instead of 1-flag)
// Replaces detect_flag (1 bit) when PR-F lands.
`define RP_DETECT_CLASS_WIDTH   2
`define RP_DETECT_NONE          2'b00   // below soft threshold
`define RP_DETECT_CANDIDATE     2'b01   // above soft, below confirm — host re-cues
`define RP_DETECT_CONFIRMED     2'b10   // above confirm threshold — track-eligible
`define RP_DETECT_RSVD          2'b11

// ============================================================================
// 3 KM MODE PARAMETERS (both 50T and 200T)
// ============================================================================

`define RP_LONG_CHIRP_SAMPLES_3KM   3000    // 30 us at 100 MSPS
`define RP_LONG_SEGMENTS_3KM        2       // ceil((3000-2048)/1920) + 1 = 2
`define RP_SHORT_CHIRP_SAMPLES      50      // 0.5 us at 100 MSPS (same both modes)
`define RP_SHORT_SEGMENTS           1       // Single segment for short chirp

// Derived 3 km limits
`define RP_MAX_RANGE_3KM            3072    // 512 bins * 6 m = 3072 m

// ============================================================================
// 20 KM MODE PARAMETERS (200T only — Phase 2)
// ============================================================================

`define RP_LONG_CHIRP_SAMPLES_20KM  13700   // 137 us at 100 MSPS (= listen window)
`define RP_LONG_SEGMENTS_20KM       8       // 1 + ceil((13700-2048)/1920) = 1 + 7 = 8
`define RP_OUTPUT_RANGE_BINS_20KM   4096    // 8 segments * 512 dec. bins each

// Derived 20 km limits
`define RP_MAX_RANGE_20KM           24576   // 4096 bins * 6 m = 24576 m

// ============================================================================
// MAX VALUES (for sizing buffers — compile-time, based on board variant)
// ============================================================================

`ifdef SUPPORT_LONG_RANGE
  `define RP_MAX_SEGMENTS           8
  `define RP_MAX_OUTPUT_BINS        4096
  `define RP_MAX_CHIRP_SAMPLES      13700
`else
  `define RP_MAX_SEGMENTS           2
  `define RP_MAX_OUTPUT_BINS        512
  `define RP_MAX_CHIRP_SAMPLES      3000
`endif

// ============================================================================
// BIT WIDTHS (derived from MAX values)
// ============================================================================

// Segment index: ceil(log2(MAX_SEGMENTS))
//   50T:  log2(2) = 1 bit  (use 2 for safety)
//   200T: log2(8) = 3 bits
`ifdef SUPPORT_LONG_RANGE
  `define RP_SEGMENT_IDX_WIDTH      3
  `define RP_RANGE_BIN_WIDTH_MAX    12      // ceil(log2(4096))
  `define RP_DOPPLER_MEM_ADDR_W     18      // ceil(log2(4096*48)) = 18 (PR-F)
  `define RP_CFAR_MAG_ADDR_W        18      // ceil(log2(4096*48)) = 18 (PR-F)
`else
  `define RP_SEGMENT_IDX_WIDTH      2
  `define RP_RANGE_BIN_WIDTH_MAX    9       // ceil(log2(512))
  `define RP_DOPPLER_MEM_ADDR_W     15      // ceil(log2(512*48)) = 15 (PR-F)
  `define RP_CFAR_MAG_ADDR_W        15      // ceil(log2(512*48)) = 15 (PR-F)
`endif

// Derived depths (for memory declarations)
// Usage: reg [15:0] mem [0:`RP_DOPPLER_MEM_DEPTH-1];
`define RP_DOPPLER_MEM_DEPTH    (`RP_MAX_OUTPUT_BINS * `RP_CHIRPS_PER_FRAME)
`define RP_CFAR_MAG_DEPTH       (`RP_MAX_OUTPUT_BINS * `RP_NUM_DOPPLER_BINS)

// ============================================================================
// CHIRP TIMING DEFAULTS (100 MHz clock cycles)
// ============================================================================
// Reset defaults for host-configurable timing registers.
// Match radar_mode_controller.v parameters and main.cpp STM32 defaults.
//
// 3-LADDER (3 km build): SHORT 1 µs, MEDIUM 5 µs, LONG 30 µs. Same listen
// budget across waveforms (~175 µs PRI) keeps Doppler resolution uniform.
// LONG kept on 50T as legal-but-unused so 200T spin-up doesn't need a
// second wave through the codebase.

`define RP_DEF_LONG_CHIRP_CYCLES    3000    // 30 us
`define RP_DEF_LONG_LISTEN_CYCLES   13700   // 137 us
`define RP_DEF_GUARD_CYCLES         17540   // 175.4 us
`define RP_DEF_SHORT_CHIRP_CYCLES   50      // 0.5 us — LEGACY; PR-E switches to 100 (1 µs)
`define RP_DEF_SHORT_LISTEN_CYCLES  17450   // 174.5 us
`define RP_DEF_CHIRPS_PER_ELEV      32      // LEGACY; bumped to 48 in PR-F

// 3-ladder defaults — added in PR-A, consumed by chirp_scheduler in PR-D.
`define RP_DEF_SHORT_CHIRP_CYCLES_V2   100     // 1 µs at 100 MHz (was 0.5 µs)
`define RP_DEF_SHORT_LISTEN_CYCLES_V2  17400   // PRI 175 µs - chirp - guard slack
`define RP_DEF_MEDIUM_CHIRP_CYCLES     500     // 5 µs at 100 MHz
`define RP_DEF_MEDIUM_LISTEN_CYCLES    17000   // PRI 175 µs - chirp - guard slack
// LONG defaults reuse RP_DEF_LONG_CHIRP_CYCLES / RP_DEF_LONG_LISTEN_CYCLES
`define RP_DEF_CHIRPS_PER_SUBFRAME     16      // 16 per sub-frame, 3 sub-frames = 48 frame
`define RP_DEF_SUBFRAME_ENABLE         3'b111  // SHORT|MEDIUM|LONG all on by default
`define RP_DEF_TRACK_WATCHDOG_FRAMES   8'd5    // frames without track cmd before scan-fallback
`define RP_DEF_TRACK_CHIRP_COUNT       9'd64   // default track-mode dwell N
`define RP_DEF_CFAR_ALPHA_SOFT         8'h18   // 1.5 in Q4.4 — soft threshold for candidates
                                                // (Pfa_soft ≈ 10⁻⁵; confirm Pfa ≈ 10⁻⁶ at α=3.0)

// ============================================================================
// BLIND ZONE CONSTANTS (informational, for comments and GUI)
// ============================================================================
// Long chirp blind zone:  c * 30 us / 2 = 4500 m
// Short chirp blind zone: c * 0.5 us / 2 = 75 m

`define RP_LONG_BLIND_ZONE_M        4500
`define RP_SHORT_BLIND_ZONE_M       75

// ============================================================================
// PHYSICAL CONSTANTS (integer-scaled for Verilog — use in comments/assertions)
// ============================================================================
// Range per ADC sample:     1.5 m  (stored as 15 in units of 0.1 m)
// Range per decimated bin:  6.0 m  (stored as 60 in units of 0.1 m)
// Processing rate:         100 MSPS

`define RP_RANGE_PER_SAMPLE_DM      15      // 1.5 m in decimeters
`define RP_RANGE_PER_BIN_DM         60      // 6.0 m in decimeters
`define RP_PROCESSING_RATE_MHZ      100

// ============================================================================
// AGC DEFAULTS
// ============================================================================
`define RP_DEF_AGC_TARGET           200
`define RP_DEF_AGC_ATTACK           1
`define RP_DEF_AGC_DECAY            1
`define RP_DEF_AGC_HOLDOFF          4

// ============================================================================
// CFAR DEFAULTS
// ============================================================================
// alpha defaults below are calibrated for the Dolph-Chebyshev 60 dB Doppler
// window (PR-M, 2026-05-01). With the new -60 dB sidelobes, training cells
// suffer ~27 dB less leakage from strong off-Doppler returns than under the
// previous "Hamming-ish" -33 dB LUT — effective Pfa at fixed alpha drops
// accordingly. Re-measure during HW bring-up; opcode 0x23/0x2D adjusts at
// runtime. See cfar_ca.v "Doppler-window dependency" comment for details.
`define RP_DEF_CFAR_GUARD           2
`define RP_DEF_CFAR_TRAIN           8
`define RP_DEF_CFAR_ALPHA           8'h30   // 3.0 in Q4.4
`define RP_DEF_CFAR_MODE            2'b00   // CA-CFAR

// ============================================================================
// DETECTION DEFAULTS
// ============================================================================
`define RP_DEF_DETECT_THRESHOLD     10000

// ============================================================================
// RADAR MODE ENCODING (host_radar_mode, opcode 0x01)
// ============================================================================
`define RP_MODE_STM32_PASSTHROUGH   2'b00
`define RP_MODE_AUTO_3KM            2'b01
`define RP_MODE_SINGLE_DEBUG        2'b10
`define RP_MODE_RESERVED            2'b11

// ============================================================================
// RANGE MODE ENCODING (host_range_mode, opcode 0x20)
// ============================================================================
`define RP_RANGE_MODE_3KM           2'b00
`define RP_RANGE_MODE_LONG          2'b01
`define RP_RANGE_MODE_RSVD2         2'b10
`define RP_RANGE_MODE_RSVD3         2'b11

// ============================================================================
// RADAR MODE ENCODING — TRACK extension (host_radar_mode, opcode 0x01)
// ============================================================================
// Mode 11 ("RESERVED" until PR-D) becomes TRACK mode: scheduler dwells one
// beam, one waveform, host_track_chirp_count chirps. Doppler runs xfft_64.
// RP_MODE_RESERVED below is renamed in-place for clarity.
`define RP_MODE_TRACK               2'b11

// ============================================================================
// STREAM CONTROL (host_stream_control, opcode 0x04, 6-bit)
// ============================================================================
// Bits [2:0]: Stream enable mask
//   Bit 0 = range profile stream
//   Bit 1 = doppler map stream
//   Bit 2 = cfar/detection stream
// Bits [5:3]: RESERVED (must be 0). PR-G dropped the legacy inert
//   mag_only/sparse_det/frame_decimate flags — protocol v2 ships a single
//   canonical encoding (Manhattan-mag doppler + 2-bit dense detect).
`define RP_STREAM_CTRL_DEFAULT      6'b000_111  // all 3 streams on, no flags

// ============================================================================
// USB PROTOCOL V2 (PR-G — clean cutover from v1)
// ============================================================================
// Wire format (FPGA → Host bulk frame):
//   Byte 0:       0xAA (frame start)
//   Byte 1:       0x02 (PROTOCOL VERSION — pinned, host MUST reject != 0x02)
//   Byte 2:       Stream flags {5'd0, stream_cfar, stream_doppler, stream_range}
//   Bytes 3–4:    Frame number (uint16, MSB first)
//   Bytes 5–6:    Range bin count (uint16, MSB first) = `RP_NUM_RANGE_BINS`
//   Bytes 7–8:    Doppler bin count (uint16, MSB first) = `RP_NUM_DOPPLER_BINS`
//   [stream_range:]   1024 B range profile (512 × uint16, MSB first)
//   [stream_doppler:] 65536 B doppler magnitude (32768 cells × uint16, row-major)
//   [stream_cfar:]    8192 B detect bitmap (32768 cells × 2 bits, MSB-first
//                     packing: cell[N] in byte[N/4] bits [7-(N%4)*2 -: 2])
//   Last byte:    0x55 (footer)
//
// Total frame (all streams on): 9 + 1024 + 65536 + 8192 + 1 = 74762 B
// At ~119 fps (PR-F 3-subframe rate) ≈ 8.9 MB/s — within FT2232H bulk budget.
`define RP_USB_PROTOCOL_VERSION     8'h02    // pinned; host rejects mismatch
`define RP_FRAME_HDR_BYTES          9        // 0xAA + ver + flags + 2*fn + 2*rb + 2*db
`define RP_DETECT_BITS_PER_CELL     2        // PR-G: 2-bit dense (NONE/CAND/CONFIRM/RSVD)
`define RP_DETECT_CELLS_PER_BYTE    4        // 8 / RP_DETECT_BITS_PER_CELL

// ============================================================================
// USB OPCODE MAP (PR-G v2 — single source of truth for RTL & GUI parity)
// ============================================================================
`define RP_OP_RADAR_MODE            8'h01
`define RP_OP_TRIGGER_PULSE         8'h02
`define RP_OP_DETECT_THRESHOLD      8'h03
`define RP_OP_STREAM_CONTROL        8'h04
`define RP_OP_LONG_CHIRP_CYCLES     8'h10
`define RP_OP_LONG_LISTEN_CYCLES    8'h11
`define RP_OP_GUARD_CYCLES          8'h12
`define RP_OP_SHORT_CHIRP_CYCLES    8'h13
`define RP_OP_SHORT_LISTEN_CYCLES   8'h14
`define RP_OP_CHIRPS_PER_ELEV       8'h15
`define RP_OP_GAIN_SHIFT            8'h16
// PR-G G2: MEDIUM ladder timings (SHORT/LONG already at 0x10-0x14, GUARD at 0x12).
`define RP_OP_MEDIUM_CHIRP_CYCLES   8'h17
`define RP_OP_MEDIUM_LISTEN_CYCLES  8'h18
// 0x19–0x1F reserved (per-waveform guard if needed in future)
`define RP_OP_RANGE_MODE            8'h20
`define RP_OP_CFAR_GUARD            8'h21
`define RP_OP_CFAR_TRAIN            8'h22
`define RP_OP_CFAR_ALPHA            8'h23   // confirm-tier (Q4.4)
`define RP_OP_CFAR_MODE             8'h24
`define RP_OP_CFAR_ENABLE           8'h25
`define RP_OP_MTI_ENABLE            8'h26
`define RP_OP_DC_NOTCH_WIDTH        8'h27
`define RP_OP_AGC_ENABLE            8'h28
`define RP_OP_AGC_TARGET            8'h29
`define RP_OP_AGC_ATTACK            8'h2A
`define RP_OP_AGC_DECAY             8'h2B
`define RP_OP_AGC_HOLDOFF           8'h2C
`define RP_OP_CFAR_ALPHA_SOFT       8'h2D   // PR-G: candidate-tier (Q4.4)
// 0x2E–0x2F reserved
`define RP_OP_SELF_TEST_TRIGGER     8'h30
`define RP_OP_SELF_TEST_STATUS      8'h31
`define RP_OP_ADC_PWDN              8'h32
`define RP_OP_ADC_FORMAT            8'h33
`define RP_OP_STATUS_REQUEST        8'hFF

`endif // RADAR_PARAMS_VH
