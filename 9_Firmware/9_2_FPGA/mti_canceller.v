`timescale 1ns / 1ps

/**
 * mti_canceller.v
 *
 * Moving Target Indication (MTI) — 2-pulse canceller for ground clutter removal.
 *
 * Sits between the range bin decimator and the Doppler processor in the
 * AERIS-10 receiver chain. Subtracts the previous chirp's range profile
 * from the current chirp's profile, implementing H(z) = 1 - z^{-1} in
 * slow-time. This places a null at zero Doppler (DC), removing stationary
 * ground clutter while passing moving targets through.
 *
 * Signal chain position:
 *   Range Bin Decimator → [MTI Canceller] → Doppler Processor
 *
 * Algorithm:
 *   For each range bin r (0..NUM_RANGE_BINS-1):
 *     mti_out_i[r] = current_i[r] - previous_i[r]
 *     mti_out_q[r] = current_q[r] - previous_q[r]
 *
 * The previous chirp's 512 range bins are stored in BRAM (inferred via
 * sync-only read/write always blocks — NO async reset on memory arrays).
 * On the very first chirp after reset (or enable), there is no previous
 * data — output is zero (muted) for that first chirp.
 *
 * When mti_enable=0, the module is a transparent pass-through.
 *
 * BRAM inference note:
 *   prev_i/prev_q arrays use dedicated sync-only always blocks for read
 *   and write. This ensures Vivado infers BRAM (RAMB18) instead of fabric
 *   FFs + mux trees. The registered read adds 1 cycle of latency, which
 *   is compensated by a pipeline stage on the input data path.
 *
 * Resources (target):
 *   - 2 BRAM18 (512 x 16-bit I + 512 x 16-bit Q)
 *   - ~30 LUTs (subtract + mux + saturation)
 *   - ~80 FFs (pipeline + control)
 *   - 0 DSP48
 *
 * Clock domain: clk (100 MHz)
 */

`include "radar_params.vh"

// ----------------------------------------------------------------------------
// [RX-D FIX] NUM_RANGE_BINS and range_bin port widths now scale with
// `RP_MAX_OUTPUT_BINS and `RP_RANGE_BIN_WIDTH_MAX (conditional on
// SUPPORT_LONG_RANGE):
//   50T  (no SUPPORT_LONG_RANGE): 512 bins / 9-bit  — 3 km only
//   200T (SUPPORT_LONG_RANGE):    4096 bins / 12-bit — supports 20 km mode
// The prev-chirp BRAM buffer auto-resizes accordingly; in 20 km mode all
// 4096 range cells are stored without aliasing.
// ----------------------------------------------------------------------------
module mti_canceller #(
    parameter NUM_RANGE_BINS = `RP_MAX_OUTPUT_BINS,   // 512 (50T) / 4096 (200T)
    parameter DATA_WIDTH     = `RP_DATA_WIDTH         // 16
) (
    input wire clk,
    input wire reset_n,

    // ========== INPUT (from range bin decimator) ==========
    input wire signed [DATA_WIDTH-1:0] range_i_in,
    input wire signed [DATA_WIDTH-1:0] range_q_in,
    input wire                         range_valid_in,
    input wire [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin_in,   // 9-bit (50T) / 12-bit (200T)

    // ========== OUTPUT (to Doppler processor) ==========
    output reg signed [DATA_WIDTH-1:0] range_i_out,
    output reg signed [DATA_WIDTH-1:0] range_q_out,
    output reg                         range_valid_out,
    output reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin_out,  // 9-bit (50T) / 12-bit (200T)

    // ========== CONFIGURATION ==========
    input wire mti_enable,   // 1=MTI active, 0=pass-through

    // Current chirp's waveform selector (from radar_mode_controller). Used
    // to mute MTI output across the long↔short chirp boundary in range
    // mode 01 (long-range interleave) — without this, the first chirp of
    // a new waveform subtracts the previous waveform's range profile,
    // injecting a per-range-bin impulse into slow-time sample 0 of the
    // new Doppler sub-frame that spreads across all Doppler bins.
    input wire use_long_chirp,

    // ========== STATUS ==========
    output reg mti_first_chirp, // 1 during first chirp (output muted)

    // Audit F-6.3: count of saturated samples since last reset. Saturation
    // here produces spurious Doppler harmonics (phantom targets at ±fs/2)
    // and was previously invisible to the MCU. Saturates at 0xFF.
    output reg [7:0] mti_saturation_count
);

// ============================================================================
// PREVIOUS CHIRP BUFFER (512 x 16-bit I, 512 x 16-bit Q)
// ============================================================================
// BRAM-inferred on XC7A50T/200T (512 entries, sync-only read/write).
// Using separate I/Q arrays for clean dual-port inference.

(* ram_style = "block" *) reg signed [DATA_WIDTH-1:0] prev_i [0:NUM_RANGE_BINS-1];
(* ram_style = "block" *) reg signed [DATA_WIDTH-1:0] prev_q [0:NUM_RANGE_BINS-1];

// ============================================================================
// INPUT PIPELINE STAGE (1 cycle delay to match BRAM read latency)
// ============================================================================
// Declarations must precede the BRAM write block that references them.

reg signed [DATA_WIDTH-1:0] range_i_d1, range_q_d1;
reg                         range_valid_d1;
reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin_d1;
reg                         mti_enable_d1;
reg                         use_long_chirp_d1;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        range_i_d1        <= {DATA_WIDTH{1'b0}};
        range_q_d1        <= {DATA_WIDTH{1'b0}};
        range_valid_d1    <= 1'b0;
        range_bin_d1      <= {`RP_RANGE_BIN_WIDTH_MAX{1'b0}};
        mti_enable_d1     <= 1'b0;
        use_long_chirp_d1 <= 1'b0;
    end else begin
        range_i_d1        <= range_i_in;
        range_q_d1        <= range_q_in;
        range_valid_d1    <= range_valid_in;
        range_bin_d1      <= range_bin_in;
        mti_enable_d1     <= mti_enable;
        use_long_chirp_d1 <= use_long_chirp;
    end
end

// ============================================================================
// BRAM WRITE PORT (sync only — NO async reset for BRAM inference)
// ============================================================================
// Writes the current chirp sample into prev_i/prev_q for next chirp's
// subtraction. Uses the delayed (d1) signals so the write happens 1 cycle
// after the read address is presented, avoiding RAW hazards.

always @(posedge clk) begin
    if (range_valid_d1) begin
        prev_i[range_bin_d1] <= range_i_d1;
        prev_q[range_bin_d1] <= range_q_d1;
    end
end

// ============================================================================
// BRAM READ PORT (sync only — 1 cycle read latency)
// ============================================================================
// Address is always driven by range_bin_in (cycle 0). Read data appears
// on prev_i_rd / prev_q_rd at cycle 1, aligned with the d1 pipeline stage.

reg signed [DATA_WIDTH-1:0] prev_i_rd, prev_q_rd;

always @(posedge clk) begin
    prev_i_rd <= prev_i[range_bin_in];
    prev_q_rd <= prev_q[range_bin_in];
end

// Track whether we have valid previous data
reg has_previous;

// Waveform of the chirp whose profile currently lives in prev_i/prev_q.
// Latched on every range_valid_d1 (use_long_chirp_d1 is constant within a
// chirp, so this stays consistent inside a chirp; at the first sample of
// the *next* chirp the OLD value is still present for the combinational
// `waveform_changed` compare, then updates this cycle to the new value).
// Updating per-cycle (rather than only at the last bin) keeps the tag
// correct when range_bin_decimator early-terminates a chirp before
// `range_bin_d1` ever reaches NUM_RANGE_BINS - 1 (RX-F).
reg prev_chirp_was_long;

// ============================================================================
// CHIRP BOUNDARY DETECTION (RX-F: end-of-chirp without depending on the
// last bin index)
// ============================================================================
// `saw_nonzero_bin_in_chirp` is set on the first non-zero bin of the current
// chirp and cleared on the next bin-0. A bin-0 arrival WITH this flag set
// = "previous chirp ended, new chirp begins" = chirp_boundary. This works
// even when the decimator emits only K < NUM_RANGE_BINS bins per chirp
// (overflow guard at range_bin_decimator.v:306, watchdog at :314).
reg saw_nonzero_bin_in_chirp;

wire chirp_boundary = range_valid_d1
                   && (range_bin_d1 == 0)
                   && saw_nonzero_bin_in_chirp;

// effective_has_previous lifts has_previous=1 *for this cycle* whenever a
// chirp boundary fires, so MTI can immediately exit mute on the bin-0 of
// the next chirp instead of waiting for the (potentially never-arriving)
// last-bin arming. has_previous itself is also set at chirp_boundary so
// subsequent bins of this chirp see it directly.
wire effective_has_previous = has_previous || chirp_boundary;

wire waveform_changed = effective_has_previous
                      && (use_long_chirp_d1 != prev_chirp_was_long);

// ============================================================================
// MTI PROCESSING (operates on d1 pipeline stage + BRAM read data)
// ============================================================================

// Compute difference with saturation
// Subtraction can produce DATA_WIDTH+1 bits; saturate back to DATA_WIDTH.
wire signed [DATA_WIDTH:0] diff_i_full = {range_i_d1[DATA_WIDTH-1], range_i_d1}
                                        - {prev_i_rd[DATA_WIDTH-1], prev_i_rd};
wire signed [DATA_WIDTH:0] diff_q_full = {range_q_d1[DATA_WIDTH-1], range_q_d1}
                                        - {prev_q_rd[DATA_WIDTH-1], prev_q_rd};

// Saturate to DATA_WIDTH bits
wire signed [DATA_WIDTH-1:0] diff_i_sat;
wire signed [DATA_WIDTH-1:0] diff_q_sat;

assign diff_i_sat = (diff_i_full > $signed({{2{1'b0}}, {(DATA_WIDTH-1){1'b1}}}))
                  ? $signed({1'b0, {(DATA_WIDTH-1){1'b1}}})           // +max
                  : (diff_i_full < $signed({{2{1'b1}}, {(DATA_WIDTH-1){1'b0}}}))
                  ? $signed({1'b1, {(DATA_WIDTH-1){1'b0}}})           // -max
                  : diff_i_full[DATA_WIDTH-1:0];

assign diff_q_sat = (diff_q_full > $signed({{2{1'b0}}, {(DATA_WIDTH-1){1'b1}}}))
                  ? $signed({1'b0, {(DATA_WIDTH-1){1'b1}}})
                  : (diff_q_full < $signed({{2{1'b1}}, {(DATA_WIDTH-1){1'b0}}}))
                  ? $signed({1'b1, {(DATA_WIDTH-1){1'b0}}})
                  : diff_q_full[DATA_WIDTH-1:0];

// Saturation detection (F-6.3): the top two bits of the DATA_WIDTH+1 signed
// difference disagree iff the value exceeds the DATA_WIDTH signed range.
wire diff_i_overflow = (diff_i_full[DATA_WIDTH] != diff_i_full[DATA_WIDTH-1]);
wire diff_q_overflow = (diff_q_full[DATA_WIDTH] != diff_q_full[DATA_WIDTH-1]);

// ============================================================================
// MAIN OUTPUT LOGIC (operates on d1 pipeline stage)
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        range_i_out              <= {DATA_WIDTH{1'b0}};
        range_q_out              <= {DATA_WIDTH{1'b0}};
        range_valid_out          <= 1'b0;
        range_bin_out            <= {`RP_RANGE_BIN_WIDTH_MAX{1'b0}};
        has_previous             <= 1'b0;
        mti_first_chirp          <= 1'b1;
        prev_chirp_was_long      <= 1'b0;
        mti_saturation_count     <= 8'd0;
        saw_nonzero_bin_in_chirp <= 1'b0;
    end else begin
        // Count saturated MTI-active samples (F-6.3). Clamp at 0xFF.
        // Uses d1 pipeline stage to align with diff_i_full/diff_q_full.
        if (range_valid_d1 && mti_enable_d1 && effective_has_previous
            && (diff_i_overflow || diff_q_overflow)
            && (mti_saturation_count != 8'hFF)) begin
            mti_saturation_count <= mti_saturation_count + 8'd1;
        end
        // Default: no valid output
        range_valid_out <= 1'b0;

        if (range_valid_d1) begin
            // Track non-zero bins so chirp_boundary can fire on the next
            // bin-0 (RX-F): set on any non-zero bin, clear on bin-0.
            saw_nonzero_bin_in_chirp <= (range_bin_d1 != 0);

            // Refresh the waveform tag on every valid sample. Within a chirp
            // this is a no-op (constant). At chirp_boundary the OLD value is
            // still visible to the combinational `waveform_changed` compare
            // (read-before-write semantics), then updates this cycle to the
            // new chirp's value.
            prev_chirp_was_long <= use_long_chirp_d1;

            // Arm has_previous on either the original last-bin trigger OR a
            // chirp_boundary (RX-F). After this cycle, prev_i/prev_q holds
            // a (possibly partial) profile we can subtract against.
            // Pass-through branch below overrides this back to 0 — last
            // non-blocking assignment wins.
            if (range_bin_d1 == NUM_RANGE_BINS - 1 || chirp_boundary) begin
                has_previous    <= 1'b1;
                mti_first_chirp <= 1'b0;
            end

            // Output path — range_bin is from the delayed pipeline
            range_bin_out <= range_bin_d1;

            if (!mti_enable_d1) begin
                // Pass-through mode: no MTI processing
                range_i_out     <= range_i_d1;
                range_q_out     <= range_q_d1;
                range_valid_out <= 1'b1;
                // Reset first-chirp state when MTI is disabled — this also
                // clears saw_nonzero_bin_in_chirp so the first MTI-enabled
                // chirp after a pass-through run is correctly treated as
                // "first chirp" and muted (T7).
                has_previous             <= 1'b0;
                mti_first_chirp          <= 1'b1;
                saw_nonzero_bin_in_chirp <= 1'b0;
            end else if (!effective_has_previous || waveform_changed) begin
                // No valid previous chirp to subtract from — either the very
                // first chirp after reset/enable, or the long↔short boundary
                // in range_mode=01 where the prev buffer holds a different
                // waveform's profile. Mute output (emit zeros with valid=1
                // so Doppler still sees the expected chirp count), overwrite
                // prev_i/prev_q as this chirp streams through the write port.
                range_i_out     <= {DATA_WIDTH{1'b0}};
                range_q_out     <= {DATA_WIDTH{1'b0}};
                range_valid_out <= 1'b1;
                mti_first_chirp <= 1'b1;
            end else begin
                // Normal MTI: subtract previous from current
                range_i_out     <= diff_i_sat;
                range_q_out     <= diff_q_sat;
                range_valid_out <= 1'b1;
            end
        end
    end
end

// ============================================================================
// MEMORY INITIALIZATION (simulation only)
// ============================================================================
`ifdef SIMULATION
integer init_k;
initial begin
    for (init_k = 0; init_k < NUM_RANGE_BINS; init_k = init_k + 1) begin
        prev_i[init_k] = 0;
        prev_q[init_k] = 0;
    end
end
`endif

endmodule
