`timescale 1ns / 1ps

/**
 * cfar_ca.v
 *
 * Cell-Averaging CFAR (Constant False Alarm Rate) Detector
 * for the AERIS-10 phased-array radar.
 *
 * Replaces the simple magnitude threshold detector in radar_system_top.v
 * (lines 474-514) with a proper adaptive-threshold CFAR algorithm.
 *
 * Architecture:
 *   Phase 1 (BUFFER): As Doppler processor outputs arrive, compute |I|+|Q|
 *     magnitude and store in BRAM. Address = {range_bin, doppler_bin}.
 *     When CFAR is disabled, applies simple threshold pass-through.
 *
 *   Phase 2 (CFAR): After frame_complete pulse from Doppler processor,
 *     process each Doppler column independently:
 *     a) Read 512 magnitudes from BRAM for one Doppler bin (ST_COL_LOAD)
 *     b) Compute initial sliding window sums (ST_CFAR_INIT)
 *     c) Slide CUT through all 512 range bins:
 *        - 3 sub-cycles per CUT:
 *          ST_CFAR_THR: register noise_sum (mode select + cross-multiply)
 *          ST_CFAR_MUL: compute alpha * noise_sum_reg in DSP
 *          ST_CFAR_CMP: compare CUT magnitude against threshold + update window
 *     d) Advance to next Doppler column (ST_COL_NEXT)
 *
 * CFAR Modes (cfg_cfar_mode):
 *   2'b00 = CA-CFAR:  noise = leading_sum + lagging_sum
 *   2'b01 = GO-CFAR:  pick side with greater PER-CELL AVERAGE (compare via
 *                      cross-multiply: leading_sum*lag_cnt vs lagging_sum*lead_cnt),
 *                      then return that side's RAW SUM (NOT divided by its
 *                      count — see GO/SO edge caveat in "Edge handling" below)
 *   2'b10 = SO-CFAR:  pick side with smaller per-cell average, return its raw sum
 *   2'b11 = Reserved (falls back to CA-CFAR)
 *
 * Threshold computation:
 *   threshold = (alpha * noise_sum) >> ALPHA_FRAC_BITS
 *   Host sets alpha in Q4.4 fixed-point, pre-compensated for training cell count.
 *   Example: for T=8 cells per side (16 total), desired Pfa=1e-4:
 *     alpha_statistical ≈ 4.88
 *     alpha_fpga = alpha_statistical / 16 = 0.305 → Q4.4 ≈ 0x05
 *   Or host can set alpha per training cell if it accounts for count.
 *
 * Doppler-window dependency (PR-M, 2026-05-01):
 *   CFAR slides through RANGE within each Doppler-bin column, so the
 *   training-cell statistics depend on what "noise" actually looks like
 *   in a column — which includes Doppler-window sidelobe leakage from
 *   strong returns at OTHER Doppler bins at the same range.
 *
 *   With the new Dolph-Chebyshev 60 dB window (vs the old "Hamming-ish"
 *   LUT at -33 dB peak sidelobes), sidelobe leakage drops 27 dB. Training
 *   cells are now closer to true thermal noise; effective Pfa at the
 *   shipped α defaults (RP_DEF_CFAR_ALPHA=3.0, ALPHA_SOFT=1.5) drops
 *   accordingly. Two operating-point choices for HW bring-up:
 *
 *     (a) Hold α — accept lower Pfa, get higher Pd as a free win.
 *     (b) Lower α — recover original Pfa, get even higher Pd.
 *
 *   Defaults are NOT changed in this PR. Recommended bring-up procedure:
 *     1. Collect noise-only frames (no targets in dwell).
 *     2. Measure empirical Pfa at shipped α=3.0/1.5.
 *     3. If Pfa < 0.5 × design target, lower α to spec; otherwise hold.
 *
 *   The opcodes 0x23 (RP_OP_CFAR_ALPHA) and 0x2D (RP_OP_CFAR_ALPHA_SOFT)
 *   let the host adjust α at runtime without firmware change.
 *
 * Edge handling:
 *   At range boundaries where the full window doesn't fit, only available
 *   training cells are used. The noise estimate naturally reduces, raising
 *   false alarm rate at edges — acceptable for radar (edge bins are
 *   typically clutter).
 *
 *   GO/SO edge caveat (AUDIT-C7): the cross-multiply correctly picks the
 *   side with the greater (GO) or lesser (SO) per-cell average, but the
 *   returned noise_sum is the raw SUM from the selected side, not the
 *   average. Combined with `alpha` being pre-baked for the interior
 *   training-cell count, this means at edges where the picked side has
 *   fewer than `train` cells the effective Pfa shifts by the same factor
 *   as the cell count (up to ~2x at the first/last `r_train` bins). For
 *   the typical config (r_train=8, r_guard=2) the asymmetry only affects
 *   the first/last ~10 of 512 range bins — for production 3 km mode that
 *   is 0..60 m (platform clutter) and 3012..3072 m (noise floor) where
 *   edge errors are masked by other effects.
 *
 *   The fix — divide by selected_count — is explicitly NOT applied:
 *   per-CUT integer divide is expensive in fabric and the affected
 *   bins are clutter/noise. Operators tuning Pfa at edges should either
 *   (a) accept the asymmetry, (b) host-side skip GO/SO outside
 *   r_train..NRANGE-r_train and fall back to CA there, or (c) hand-tune
 *   alpha per-mode based on observed Pfa drift.
 *
 * Timing:
 *   Phase 2 takes ~(514 + T + 3*512) * 32 ≈ 55000 cycles per frame @ 100 MHz
 *   = 0.55 ms. Frame period @ PRF=1932 Hz, 32 chirps = 16.6 ms. Fits easily.
 *   (3 cycles per CUT due to pipeline: THR → MUL → CMP)
 *
 * AUDIT-S22 — DOWNSTREAM CADENCE DEPENDENCY (DO NOT BREAK):
 *   detect_valid pulses every 3rd cycle (one per CUT triplet). The downstream
 *   consumer usb_data_interface_ft2232h.v runs a 3-cycle read-modify-write
 *   on the detection-flag BRAM (idle → read-wait → write-back) and silently
 *   drops cfar_valid arriving while RMW is busy. The two cadences match
 *   today by construction.
 *
 *   If you optimize this pipeline below 3 cycles per CUT (e.g., merging
 *   ST_CFAR_MUL+CMP into a single state, or feeding the comparator
 *   combinationally), you MUST also pipeline the RMW in
 *   usb_data_interface_ft2232h.v to keep up — otherwise every Nth
 *   detection is silently lost. A SIMULATION-only assertion in that
 *   module fires `[ASSERT FAIL] AUDIT-S22: cfar_valid arrived while RMW
 *   busy` to catch this regression in the test suite.
 *
 * Resources:
 *   - 1 BRAM36K for magnitude buffer (16384 x 17 bits)
 *   - 1 DSP48 for alpha multiply
 *   - ~300 LUTs for FSM + sliding window + comparators
 *
 * Clock domain: clk (100 MHz, same as Doppler processor)
 */

`include "radar_params.vh"

// [RX-D FIX] NUM_RANGE_BINS and range_bin port widths now scale with
// `RP_MAX_OUTPUT_BINS / `RP_RANGE_BIN_WIDTH_MAX (50T: 512/9, 200T: 4096/12).
// CFAR magnitude BRAM depth uses `RP_CFAR_MAG_DEPTH which already scales.
module cfar_ca #(
    parameter NUM_RANGE_BINS   = `RP_MAX_OUTPUT_BINS,   // 512 (50T) / 4096 (200T)
    parameter NUM_DOPPLER_BINS = `RP_NUM_DOPPLER_BINS,  // 48 (PR-F)
    parameter MAG_WIDTH        = 17,
    parameter ALPHA_WIDTH      = 8,
    parameter MAX_GUARD        = 8,
    parameter MAX_TRAIN        = 16,
    parameter DBIN_WIDTH       = `RP_DOPPLER_BIN_WIDTH  // 6 (PR-F)
) (
    input wire clk,
    input wire reset_n,

    // ========== DOPPLER PROCESSOR INPUTS ==========
    input wire [31:0] doppler_data,
    input wire        doppler_valid,
    input wire [DBIN_WIDTH-1:0] doppler_bin_in,
    input wire [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin_in,  // 9-bit (50T) / 12-bit (200T)
    input wire        frame_complete,

    // ========== CONFIGURATION ==========
    input wire [3:0]  cfg_guard_cells,
    input wire [4:0]  cfg_train_cells,
    input wire [ALPHA_WIDTH-1:0] cfg_alpha,
    input wire [ALPHA_WIDTH-1:0] cfg_alpha_soft,   // PR-F: candidate-tier threshold
    input wire [1:0]  cfg_cfar_mode,
    input wire        cfg_cfar_enable,
    input wire [15:0] cfg_simple_threshold,

    // ========== DETECTION OUTPUTS ==========
    output reg        detect_flag,        // = (detect_class != RP_DETECT_NONE)
    output reg [`RP_DETECT_CLASS_WIDTH-1:0] detect_class,  // PR-F: NONE/CANDIDATE/CONFIRMED
    output reg        detect_valid,
    output reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] detect_range,
    output reg [DBIN_WIDTH-1:0] detect_doppler,
    output reg [MAG_WIDTH-1:0] detect_magnitude,
    output reg [MAG_WIDTH-1:0] detect_threshold,        // confirmed threshold (legacy)
    output reg [MAG_WIDTH-1:0] detect_threshold_soft,   // PR-F: soft (candidate) threshold

    // ========== STATUS ==========
    output reg [15:0] detect_count,       // total detections (CONFIRMED only)
    output reg [15:0] detect_count_cand,  // PR-F: candidate-only counter
    output wire       cfar_busy,
    output reg [7:0]  cfar_status
);

// ============================================================================
// INTERNAL PARAMETERS
// ============================================================================
// Doppler-axis index width: enough bits to count 0..NUM_DOPPLER_BINS-1.
// Packed BRAM addressing pads to the next power of two so the {range,doppler}
// concatenation lands in a contiguous block per range bin (works for both
// NUM_DOPPLER_BINS=32, legacy power-of-two, and NUM_DOPPLER_BINS=48, PR-F).
function integer clog2;
    input integer v;
    integer i;
    begin
        clog2 = 0;
        for (i = v - 1; i > 0; i = i >> 1) clog2 = clog2 + 1;
    end
endfunction
localparam DBIN_INDEX_BITS  = clog2(NUM_DOPPLER_BINS);                // 5 (NUM=32) / 6 (NUM=48)
localparam DOPPLER_PAD      = (1 << DBIN_INDEX_BITS);                 // 32 / 64
localparam TOTAL_CELLS      = NUM_RANGE_BINS * DOPPLER_PAD;           // 16K (50T legacy) / 32K (50T PR-F)
localparam ADDR_WIDTH       = `RP_RANGE_BIN_WIDTH_MAX + DBIN_INDEX_BITS;
localparam COL_BITS         = DBIN_INDEX_BITS;                        // address-axis col counter
localparam ROW_BITS         = `RP_RANGE_BIN_WIDTH_MAX;                // 9 (50T) / 12 (200T)
localparam SUM_WIDTH   = MAG_WIDTH + ROW_BITS;         // 26 (50T) / 29 (200T)
localparam PROD_WIDTH  = SUM_WIDTH + ALPHA_WIDTH;  // 34 bits
localparam ALPHA_FRAC_BITS = 4;  // Q4.4

// ============================================================================
// FSM STATES
// ============================================================================
localparam [3:0] ST_IDLE       = 4'd0,
                 ST_BUFFER     = 4'd1,
                 ST_COL_LOAD   = 4'd2,
                 ST_CFAR_INIT  = 4'd3,
                 ST_CFAR_THR   = 4'd4,  // Register noise_sum (mode select + cross-multiply)
                 ST_CFAR_MUL   = 4'd8,  // Compute alpha * noise_sum_reg in DSP
                 ST_CFAR_CMP   = 4'd5,  // Compare + update window
                 ST_COL_NEXT   = 4'd6,
                 ST_DONE       = 4'd7;

reg [3:0] state;
assign cfar_busy = (state != ST_IDLE);

// ============================================================================
// MAGNITUDE COMPUTATION (combinational)
// ============================================================================
wire signed [15:0] dop_i = doppler_data[15:0];
wire signed [15:0] dop_q = doppler_data[31:16];
wire [15:0] abs_i = dop_i[15] ? (~dop_i + 16'd1) : dop_i;
wire [15:0] abs_q = dop_q[15] ? (~dop_q + 16'd1) : dop_q;
wire [MAG_WIDTH-1:0] cur_mag = {1'b0, abs_i} + {1'b0, abs_q};

// ============================================================================
// MAGNITUDE BRAM (16384 x 17 bits)
// ============================================================================
reg                    mag_we;
reg [ADDR_WIDTH-1:0]   mag_waddr;
reg [MAG_WIDTH-1:0]    mag_wdata;
reg [ADDR_WIDTH-1:0]   mag_raddr;
reg [MAG_WIDTH-1:0]    mag_rdata;

(* ram_style = "block" *) reg [MAG_WIDTH-1:0] mag_mem [0:TOTAL_CELLS-1];

always @(posedge clk) begin
    if (mag_we)
        mag_mem[mag_waddr] <= mag_wdata;
    mag_rdata <= mag_mem[mag_raddr];
end

// ============================================================================
// COLUMN LINE BUFFER (512 x 17 bits — BRAM)
// ============================================================================
reg [MAG_WIDTH-1:0] col_buf [0:NUM_RANGE_BINS-1];
reg [ROW_BITS:0] col_load_idx;

// ============================================================================
// SLIDING WINDOW STATE
// ============================================================================
reg [SUM_WIDTH-1:0] leading_sum;
reg [SUM_WIDTH-1:0] lagging_sum;
reg [ROW_BITS:0]    leading_count;
reg [ROW_BITS:0]    lagging_count;
reg [ROW_BITS:0]    cut_idx;
reg [COL_BITS-1:0]  col_idx;

// Registered config (captured at frame start)
reg [3:0]  r_guard;
reg [4:0]  r_train;
reg [ALPHA_WIDTH-1:0] r_alpha;
reg [ALPHA_WIDTH-1:0] r_alpha_soft;     // PR-F: candidate threshold multiplier
reg [1:0]  r_mode;
reg        r_enable;
reg [15:0] r_simple_thr;

// Threshold pipeline registers
reg [SUM_WIDTH-1:0]  noise_sum_reg;   // Stage 1: registered noise_sum_comb output
reg [PROD_WIDTH-1:0] noise_product;        // Stage 2: alpha      * noise_sum_reg
reg [PROD_WIDTH-1:0] noise_product_soft;   // PR-F:    alpha_soft * noise_sum_reg
reg [MAG_WIDTH-1:0]  adaptive_thr;

// Init counter for computing initial lagging sum
reg [ROW_BITS:0] init_idx;

// ============================================================================
// SLIDING WINDOW DELTA COMPUTATION (combinational)
// ============================================================================
// Compute net delta to leading_sum and lagging_sum when CUT advances by 1.
// All deltas computed combinationally, applied as a single NBA per register.

// Indices of cells entering/leaving the window when CUT moves from k to k+1:
//   Leading: new training cell at index k+1-G-1 = k-G (was closest guard cell)
//            cell falling off at index k+1-G-T-1 = k-G-T
//   Lagging: cell leaving at index k+G+1 (enters guard zone)
//            new cell entering at index k+1+G+T (at far end)

wire signed [ROW_BITS+1:0] lead_add_idx = $signed({1'b0, cut_idx}) - $signed({1'b0, r_guard});
wire signed [ROW_BITS+1:0] lead_rem_idx = $signed({1'b0, cut_idx}) - $signed({1'b0, r_guard}) - $signed({1'b0, r_train});
wire signed [ROW_BITS+1:0] lag_rem_idx  = $signed({1'b0, cut_idx}) + $signed({1'b0, r_guard}) + 1;
wire signed [ROW_BITS+1:0] lag_add_idx  = $signed({1'b0, cut_idx}) + 1 + $signed({1'b0, r_guard}) + $signed({1'b0, r_train});

wire lead_add_valid = (lead_add_idx >= 0) && (lead_add_idx < NUM_RANGE_BINS);
wire lead_rem_valid = (lead_rem_idx >= 0) && (lead_rem_idx < NUM_RANGE_BINS);
wire lag_rem_valid  = (lag_rem_idx  >= 0) && (lag_rem_idx  < NUM_RANGE_BINS);
wire lag_add_valid  = (lag_add_idx  >= 0) && (lag_add_idx  < NUM_RANGE_BINS);

// Safe col_buf read with bounds checking (combinational — feeds pipeline regs)
wire [MAG_WIDTH-1:0] lead_add_val = lead_add_valid ? col_buf[lead_add_idx[ROW_BITS-1:0]] : {MAG_WIDTH{1'b0}};
wire [MAG_WIDTH-1:0] lead_rem_val = lead_rem_valid ? col_buf[lead_rem_idx[ROW_BITS-1:0]] : {MAG_WIDTH{1'b0}};
wire [MAG_WIDTH-1:0] lag_rem_val  = lag_rem_valid  ? col_buf[lag_rem_idx[ROW_BITS-1:0]]  : {MAG_WIDTH{1'b0}};
wire [MAG_WIDTH-1:0] lag_add_val  = lag_add_valid  ? col_buf[lag_add_idx[ROW_BITS-1:0]]  : {MAG_WIDTH{1'b0}};

// ============================================================================
// PIPELINE REGISTERS: Break col_buf mux tree out of ST_CFAR_CMP critical path
// ============================================================================
// Captured in ST_CFAR_THR (col_buf indices depend only on cut_idx/r_guard/r_train,
// all stable during THR). Used in ST_CFAR_CMP for delta/sum computation.
// This removes ~6-8 logic levels (9-level mux tree) from the CMP critical path.
reg [MAG_WIDTH-1:0] lead_add_val_r, lead_rem_val_r;
reg [MAG_WIDTH-1:0] lag_rem_val_r,  lag_add_val_r;
reg                 lead_add_valid_r, lead_rem_valid_r;
reg                 lag_rem_valid_r,  lag_add_valid_r;

// Net deltas (computed from registered col_buf values — combinational in CMP)
wire signed [SUM_WIDTH:0] lead_delta = (lead_add_valid_r ? $signed({1'b0, lead_add_val_r}) : 0)
                                      - (lead_rem_valid_r ? $signed({1'b0, lead_rem_val_r}) : 0);
wire signed [1:0] lead_cnt_delta = (lead_add_valid_r ? 1 : 0) - (lead_rem_valid_r ? 1 : 0);

wire signed [SUM_WIDTH:0] lag_delta = (lag_add_valid_r ? $signed({1'b0, lag_add_val_r}) : 0)
                                     - (lag_rem_valid_r ? $signed({1'b0, lag_rem_val_r}) : 0);
wire signed [1:0] lag_cnt_delta = (lag_add_valid_r ? 1 : 0) - (lag_rem_valid_r ? 1 : 0);

// ============================================================================
// NOISE ESTIMATE COMPUTATION (combinational for CFAR mode selection)
// ============================================================================
reg [SUM_WIDTH-1:0] noise_sum_comb;

always @(*) begin
    case (r_mode)
    2'b00, 2'b11: begin  // CA-CFAR
        noise_sum_comb = leading_sum + lagging_sum;
    end
    2'b01: begin  // GO-CFAR: pick sum from side with greater average
        // AUDIT-C7: cross-multiply chooses by per-cell AVERAGE, but we return
        // the raw SUM (not divided by selected count). At range edges where
        // the picked side is truncated, effective Pfa shifts by the count
        // ratio. Trade-off accepted; per-CUT divide is too expensive in
        // 50T fabric. See module header "Edge handling / GO/SO edge caveat".
        if (leading_count > 0 && lagging_count > 0) begin
            // leading_avg > lagging_avg ↔ leading_sum * lagging_count > lagging_sum * leading_count
            if (leading_sum * lagging_count > lagging_sum * leading_count)
                noise_sum_comb = leading_sum;
            else
                noise_sum_comb = lagging_sum;
        end else if (leading_count > 0)
            noise_sum_comb = leading_sum;
        else
            noise_sum_comb = lagging_sum;
    end
    2'b10: begin  // SO-CFAR: pick sum from side with smaller average
        // AUDIT-C7: same selection-vs-normalization asymmetry as GO above.
        if (leading_count > 0 && lagging_count > 0) begin
            if (leading_sum * lagging_count < lagging_sum * leading_count)
                noise_sum_comb = leading_sum;
            else
                noise_sum_comb = lagging_sum;
        end else if (leading_count > 0)
            noise_sum_comb = leading_sum;
        else
            noise_sum_comb = lagging_sum;
    end
    default:
        noise_sum_comb = leading_sum + lagging_sum;
    endcase
end

// ============================================================================
// MAIN FSM
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state          <= ST_IDLE;
        detect_flag    <= 1'b0;
        detect_class   <= `RP_DETECT_NONE;
        detect_valid   <= 1'b0;
        detect_range   <= {ROW_BITS{1'b0}};
        detect_doppler <= {DBIN_WIDTH{1'b0}};
        detect_magnitude <= {MAG_WIDTH{1'b0}};
        detect_threshold <= {MAG_WIDTH{1'b0}};
        detect_threshold_soft <= {MAG_WIDTH{1'b0}};
        detect_count   <= 16'd0;
        detect_count_cand <= 16'd0;
        cfar_status    <= 8'd0;
        mag_we         <= 1'b0;
        mag_waddr      <= {ADDR_WIDTH{1'b0}};
        mag_wdata      <= {MAG_WIDTH{1'b0}};
        mag_raddr      <= {ADDR_WIDTH{1'b0}};
        col_load_idx   <= 0;
        col_idx        <= 0;
        cut_idx        <= 0;
        leading_sum    <= 0;
        lagging_sum    <= 0;
        leading_count  <= 0;
        lagging_count  <= 0;
        init_idx       <= 0;
        noise_sum_reg  <= 0;
        noise_product  <= 0;
        noise_product_soft <= 0;
        adaptive_thr   <= 0;
        lead_add_val_r <= 0;
        lead_rem_val_r <= 0;
        lag_rem_val_r  <= 0;
        lag_add_val_r  <= 0;
        lead_add_valid_r <= 0;
        lead_rem_valid_r <= 0;
        lag_rem_valid_r  <= 0;
        lag_add_valid_r  <= 0;
        r_guard        <= 4'd2;
        r_train        <= 5'd8;
        r_alpha        <= `RP_DEF_CFAR_ALPHA;
        r_alpha_soft   <= `RP_DEF_CFAR_ALPHA_SOFT;
        r_mode         <= 2'b00;
        r_enable       <= 1'b0;
        r_simple_thr   <= 16'd10000;
    end else begin
        // Defaults: clear one-shot outputs
        detect_valid <= 1'b0;
        detect_flag  <= 1'b0;
        detect_class <= `RP_DETECT_NONE;
        mag_we       <= 1'b0;

        case (state)
        // ================================================================
        // ST_IDLE: Wait for first Doppler output
        // ================================================================
        ST_IDLE: begin
            cfar_status <= 8'd0;

            if (doppler_valid) begin
                // Capture configuration at frame start. PR-F: per-frame counters
                // reset to 0 here (matches the AUDIT-C6 fix in ST_DONE for the
                // legacy detect_count).
                r_guard      <= cfg_guard_cells;
                r_train      <= (cfg_train_cells == 0) ? 5'd1 : cfg_train_cells;
                r_alpha      <= cfg_alpha;
                r_alpha_soft <= cfg_alpha_soft;
                r_mode       <= cfg_cfar_mode;
                r_enable     <= cfg_cfar_enable;
                r_simple_thr <= cfg_simple_threshold;

                // Buffer first sample
                mag_we    <= 1'b1;
                mag_waddr <= {range_bin_in, doppler_bin_in[DBIN_INDEX_BITS-1:0]};
                mag_wdata <= cur_mag;

                // Simple threshold pass-through when CFAR disabled.
                // Without an adaptive estimate we can't form a soft tier, so
                // detect_class collapses to NONE/CONFIRMED on the simple thr.
                if (!cfg_cfar_enable) begin
                    detect_flag  <= (cur_mag > {1'b0, cfg_simple_threshold});
                    detect_class <= (cur_mag > {1'b0, cfg_simple_threshold})
                                    ? `RP_DETECT_CONFIRMED : `RP_DETECT_NONE;
                    detect_valid <= 1'b1;
                    detect_range     <= range_bin_in;
                    detect_doppler   <= doppler_bin_in;
                    detect_magnitude <= cur_mag;
                    detect_threshold <= {1'b0, cfg_simple_threshold};
                    detect_threshold_soft <= {1'b0, cfg_simple_threshold};
                    if (cur_mag > {1'b0, cfg_simple_threshold})
                        detect_count <= detect_count + 1;
                end

                state <= ST_BUFFER;
            end
        end

        // ================================================================
        // ST_BUFFER: Store magnitudes until frame complete
        // ================================================================
        ST_BUFFER: begin
            cfar_status <= {4'd1, 4'd0};

            if (doppler_valid) begin
                mag_we    <= 1'b1;
                mag_waddr <= {range_bin_in, doppler_bin_in[DBIN_INDEX_BITS-1:0]};
                mag_wdata <= cur_mag;

                if (!r_enable) begin
                    detect_flag  <= (cur_mag > {1'b0, r_simple_thr});
                    detect_class <= (cur_mag > {1'b0, r_simple_thr})
                                    ? `RP_DETECT_CONFIRMED : `RP_DETECT_NONE;
                    detect_valid <= 1'b1;
                    detect_range     <= range_bin_in;
                    detect_doppler   <= doppler_bin_in;
                    detect_magnitude <= cur_mag;
                    detect_threshold <= {1'b0, r_simple_thr};
                    detect_threshold_soft <= {1'b0, r_simple_thr};
                    if (cur_mag > {1'b0, r_simple_thr})
                        detect_count <= detect_count + 1;
                end
            end

            if (frame_complete) begin
                if (r_enable) begin
                    col_idx      <= 0;
                    col_load_idx <= 0;
                    mag_raddr    <= {{ROW_BITS{1'b0}}, {COL_BITS{1'b0}}};
                    state        <= ST_COL_LOAD;
                end else begin
                    state <= ST_DONE;
                end
            end
        end

        // ================================================================
        // ST_COL_LOAD: Read one Doppler column from BRAM
        // ================================================================
        // BRAM has 1-cycle read latency. Pipeline: present addr cycle N,
        // capture data cycle N+1.
        ST_COL_LOAD: begin
            cfar_status <= {4'd2, 1'b0, col_idx[2:0]};

            if (col_load_idx == 0) begin
                // First address already presented, advance to range=1
                mag_raddr    <= {{{(ROW_BITS-1){1'b0}}, 1'b1}, col_idx};
                col_load_idx <= 1;
            end else if (col_load_idx <= NUM_RANGE_BINS) begin
                // Capture previous read
                col_buf[col_load_idx - 1] <= mag_rdata;

                if (col_load_idx < NUM_RANGE_BINS) begin
                    mag_raddr <= {col_load_idx[ROW_BITS-1:0] + {{(ROW_BITS-1){1'b0}}, 1'b1}, col_idx};
                end

                col_load_idx <= col_load_idx + 1;
            end

            if (col_load_idx == NUM_RANGE_BINS + 1) begin
                // Column fully loaded → initialize CFAR window
                state         <= ST_CFAR_INIT;
                init_idx      <= 0;
                leading_sum   <= 0;
                lagging_sum   <= 0;
                leading_count <= 0;
                lagging_count <= 0;
                cut_idx       <= 0;
            end
        end

        // ================================================================
        // ST_CFAR_INIT: Compute initial window sums for CUT=0
        // ================================================================
        // CUT=0 has no leading cells. Lagging cells are at
        // indices [guard+1 .. guard+train] (if they exist).
        // Iterate one training cell per cycle.
        ST_CFAR_INIT: begin
            cfar_status <= {4'd3, 1'b0, col_idx[2:0]};

            if (init_idx < r_train) begin
                if ((r_guard + 1 + init_idx) < NUM_RANGE_BINS) begin
                    lagging_sum   <= lagging_sum + col_buf[r_guard + 1 + init_idx];
                    lagging_count <= lagging_count + 1;
                end
                init_idx <= init_idx + 1;
            end else begin
                // Initial sums ready → begin CFAR sliding
                state <= ST_CFAR_THR;
            end
        end

        // ================================================================
        // ST_CFAR_THR: Register noise estimate (mode select + cross-multiply)
        // ================================================================
        // Pipeline stage 1: register the combinational noise_sum_comb
        // output. This breaks the critical path:
        //   leading_sum → cross-multiply (GO/SO) → mux → alpha*noise DSP
        // into two shorter paths:
        //   Cycle 1: leading_sum → cross-multiply → mux → noise_sum_reg
        //   Cycle 2: noise_sum_reg → alpha * noise_sum_reg → noise_product
        ST_CFAR_THR: begin
            cfar_status <= {4'd4, 1'b0, col_idx[2:0]};

            noise_sum_reg <= noise_sum_comb;

            // Pipeline: register col_buf reads for next CUT's window update.
            // Indices depend only on cut_idx/r_guard/r_train (all stable here).
            // Breaks the 9-level col_buf mux tree out of ST_CFAR_CMP.
            lead_add_val_r   <= lead_add_val;
            lead_rem_val_r   <= lead_rem_val;
            lag_rem_val_r    <= lag_rem_val;
            lag_add_val_r    <= lag_add_val;
            lead_add_valid_r <= lead_add_valid;
            lead_rem_valid_r <= lead_rem_valid;
            lag_rem_valid_r  <= lag_rem_valid;
            lag_add_valid_r  <= lag_add_valid;

            state <= ST_CFAR_MUL;
        end

        // ================================================================
        // ST_CFAR_MUL: Compute alpha * noise_sum_reg in DSP
        // ================================================================
        // Pipeline stage 2: multiply registered noise sum by alpha.
        // This is a clean registered-input → DSP path.
        ST_CFAR_MUL: begin
            cfar_status <= {4'd4, 1'b1, col_idx[2:0]};

            // Two parallel multiplies — each maps to a single DSP48 slice.
            noise_product      <= r_alpha      * noise_sum_reg;   // confirmed tier
            noise_product_soft <= r_alpha_soft * noise_sum_reg;   // candidate tier (PR-F)
            state <= ST_CFAR_CMP;
        end

        // ================================================================
        // ST_CFAR_CMP: Compare CUT against threshold + update window
        // ================================================================
        ST_CFAR_CMP: begin
            cfar_status <= {4'd5, 1'b0, col_idx[2:0]};

            // Threshold = noise_product >> ALPHA_FRAC_BITS
            // Saturate to MAG_WIDTH bits
            if (noise_product[PROD_WIDTH-1:ALPHA_FRAC_BITS+MAG_WIDTH] != 0)
                adaptive_thr <= {MAG_WIDTH{1'b1}};  // Saturate
            else
                adaptive_thr <= noise_product[ALPHA_FRAC_BITS +: MAG_WIDTH];

            // Output detection result
            detect_magnitude <= col_buf[cut_idx[ROW_BITS-1:0]];
            detect_range     <= cut_idx[ROW_BITS-1:0];
            detect_doppler   <= col_idx;
            detect_valid     <= 1'b1;

            // Compare: confirm + soft thresholds computed this cycle from
            // noise_product / noise_product_soft. detect_class encodes the
            // tier (NONE / CANDIDATE / CONFIRMED) so downstream can re-cue
            // CANDIDATEs and track CONFIRMEDs.
            begin : threshold_compare
                reg [MAG_WIDTH-1:0] thr_val;
                reg [MAG_WIDTH-1:0] thr_val_soft;
                reg [MAG_WIDTH-1:0] cur_val;

                if (noise_product[PROD_WIDTH-1:ALPHA_FRAC_BITS+MAG_WIDTH] != 0)
                    thr_val = {MAG_WIDTH{1'b1}};
                else
                    thr_val = noise_product[ALPHA_FRAC_BITS +: MAG_WIDTH];

                if (noise_product_soft[PROD_WIDTH-1:ALPHA_FRAC_BITS+MAG_WIDTH] != 0)
                    thr_val_soft = {MAG_WIDTH{1'b1}};
                else
                    thr_val_soft = noise_product_soft[ALPHA_FRAC_BITS +: MAG_WIDTH];

                detect_threshold      <= thr_val;
                detect_threshold_soft <= thr_val_soft;

                cur_val = col_buf[cut_idx[ROW_BITS-1:0]];

                if (cur_val > thr_val) begin
                    detect_flag  <= 1'b1;
                    detect_class <= `RP_DETECT_CONFIRMED;
                    detect_count <= detect_count + 1;
                end else if (cur_val > thr_val_soft) begin
                    // Above soft, below confirm — host re-cues this cell.
                    detect_flag       <= 1'b1;
                    detect_class      <= `RP_DETECT_CANDIDATE;
                    detect_count_cand <= detect_count_cand + 1;
                end
            end

            // Update sliding window for next CUT
            if (cut_idx < NUM_RANGE_BINS - 1) begin
                // Apply pre-computed deltas (single NBA per register)
                leading_sum   <= $unsigned($signed({1'b0, leading_sum}) + lead_delta);
                leading_count <= $unsigned($signed({1'b0, leading_count}) + {{(ROW_BITS){lead_cnt_delta[1]}}, lead_cnt_delta});
                lagging_sum   <= $unsigned($signed({1'b0, lagging_sum}) + lag_delta);
                lagging_count <= $unsigned($signed({1'b0, lagging_count}) + {{(ROW_BITS){lag_cnt_delta[1]}}, lag_cnt_delta});

                cut_idx <= cut_idx + 1;
                state   <= ST_CFAR_THR;
            end else begin
                state <= ST_COL_NEXT;
            end
        end

        // ================================================================
        // ST_COL_NEXT: Advance to next Doppler column or finish
        // ================================================================
        ST_COL_NEXT: begin
            if (col_idx < NUM_DOPPLER_BINS - 1) begin
                col_idx      <= col_idx + 1;
                col_load_idx <= 0;
                mag_raddr    <= {{ROW_BITS{1'b0}}, col_idx + {{(COL_BITS-1){1'b0}}, 1'b1}};
                state        <= ST_COL_LOAD;
            end else begin
                state <= ST_DONE;
            end
        end

        // ================================================================
        // ST_DONE: Frame complete, return to idle
        // ================================================================
        // AUDIT-C6 fix: reset detect_count per-frame so it represents
        // "detections this frame" instead of "total since power-on". The
        // 16-bit counter saturates after ~6500 frames at typical detection
        // rates (tens of seconds of real traffic), breaking any rate-based
        // host telemetry that reads it.
        // ================================================================
        ST_DONE: begin
            cfar_status <= 8'd0;
            state <= ST_IDLE;

            `ifdef SIMULATION
            $display("[CFAR] Frame complete: %0d confirmed, %0d candidates",
                     detect_count, detect_count_cand);
            `endif

            detect_count      <= 16'd0;
            detect_count_cand <= 16'd0;
        end

        default: state <= ST_IDLE;
        endcase
    end
end

// ============================================================================
// BRAM + LINE BUFFER INITIALIZATION (simulation only)
// ============================================================================
`ifdef SIMULATION
integer init_i;
initial begin
    for (init_i = 0; init_i < TOTAL_CELLS; init_i = init_i + 1)
        mag_mem[init_i] = 0;
    for (init_i = 0; init_i < NUM_RANGE_BINS; init_i = init_i + 1)
        col_buf[init_i] = 0;
end
`endif

endmodule
