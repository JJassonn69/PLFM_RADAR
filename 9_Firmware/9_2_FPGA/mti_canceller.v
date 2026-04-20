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

module mti_canceller #(
    parameter NUM_RANGE_BINS = `RP_NUM_RANGE_BINS,    // 512
    parameter DATA_WIDTH     = `RP_DATA_WIDTH         // 16
) (
    input wire clk,
    input wire reset_n,

    // ========== INPUT (from range bin decimator) ==========
    input wire signed [DATA_WIDTH-1:0] range_i_in,
    input wire signed [DATA_WIDTH-1:0] range_q_in,
    input wire                         range_valid_in,
    input wire [`RP_RANGE_BIN_BITS-1:0] range_bin_in,   // 9-bit

    // ========== OUTPUT (to Doppler processor) ==========
    output reg signed [DATA_WIDTH-1:0] range_i_out,
    output reg signed [DATA_WIDTH-1:0] range_q_out,
    output reg                         range_valid_out,
    output reg [`RP_RANGE_BIN_BITS-1:0] range_bin_out,   // 9-bit

    // ========== CONFIGURATION ==========
    input wire mti_enable,   // 1=MTI active, 0=pass-through

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
reg [`RP_RANGE_BIN_BITS-1:0] range_bin_d1;
reg                         mti_enable_d1;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        range_i_d1     <= {DATA_WIDTH{1'b0}};
        range_q_d1     <= {DATA_WIDTH{1'b0}};
        range_valid_d1 <= 1'b0;
        range_bin_d1   <= {`RP_RANGE_BIN_BITS{1'b0}};
        mti_enable_d1  <= 1'b0;
    end else begin
        range_i_d1     <= range_i_in;
        range_q_d1     <= range_q_in;
        range_valid_d1 <= range_valid_in;
        range_bin_d1   <= range_bin_in;
        mti_enable_d1  <= mti_enable;
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
        range_i_out          <= {DATA_WIDTH{1'b0}};
        range_q_out          <= {DATA_WIDTH{1'b0}};
        range_valid_out      <= 1'b0;
        range_bin_out        <= {`RP_RANGE_BIN_BITS{1'b0}};
        has_previous         <= 1'b0;
        mti_first_chirp      <= 1'b1;
        mti_saturation_count <= 8'd0;
    end else begin
        // Count saturated MTI-active samples (F-6.3). Clamp at 0xFF.
        // Uses d1 pipeline stage to align with diff_i_full/diff_q_full.
        if (range_valid_d1 && mti_enable_d1 && has_previous
            && (diff_i_overflow || diff_q_overflow)
            && (mti_saturation_count != 8'hFF)) begin
            mti_saturation_count <= mti_saturation_count + 8'd1;
        end
        // Default: no valid output
        range_valid_out <= 1'b0;

        if (range_valid_d1) begin
            // Output path — range_bin is from the delayed pipeline
            range_bin_out <= range_bin_d1;

            if (!mti_enable_d1) begin
                // Pass-through mode: no MTI processing
                range_i_out     <= range_i_d1;
                range_q_out     <= range_q_d1;
                range_valid_out <= 1'b1;
                // Reset first-chirp state when MTI is disabled
                has_previous    <= 1'b0;
                mti_first_chirp <= 1'b1;
            end else if (!has_previous) begin
                // First chirp after enable: mute output (no subtraction possible).
                // Still emit valid=1 with zero data so Doppler processor gets
                // the expected number of samples per frame.
                range_i_out     <= {DATA_WIDTH{1'b0}};
                range_q_out     <= {DATA_WIDTH{1'b0}};
                range_valid_out <= 1'b1;

                // After last range bin of first chirp, mark previous as valid
                if (range_bin_d1 == NUM_RANGE_BINS - 1) begin
                    has_previous    <= 1'b1;
                    mti_first_chirp <= 1'b0;
                end
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
