`timescale 1ns / 1ps

/**
 * range_bin_decimator.v
 *
 * Reduces 2048 range bins from the matched filter output down to 512 bins
 * for the Doppler processor. Supports multiple decimation modes:
 *
 *   Mode 2'b00: Simple decimation (take every Nth sample)
 *   Mode 2'b01: Peak detection (select max-magnitude sample from each group)
 *   Mode 2'b10: Averaging (sum group and divide by N)
 *   Mode 2'b11: Reserved
 *
 * Interface contract (from radar_receiver_final.v):
 *   .clk, .reset_n
 *   .range_i_in, .range_q_in, .range_valid_in   <- from matched_filter output
 *   .range_i_out, .range_q_out, .range_valid_out -> to Doppler processor
 *   .range_bin_index                             -> 9-bit output bin index
 *   .decimation_mode                             <- 2-bit mode select
 *   .start_bin                                   <- 11-bit start offset
 *
 * start_bin usage:
 *   When start_bin > 0, the decimator skips the first 'start_bin' valid
 *   input samples before beginning decimation. This allows selecting a
 *   region of interest within the 2048 range bins (e.g., to focus on
 *   near-range or far-range targets). When start_bin = 0 (default),
 *   all 2048 bins are processed starting from bin 0.
 *
 * Clock domain: clk (100 MHz)
 * Decimation: 2048 -> 512 (factor of 4)
 */

`include "radar_params.vh"

module range_bin_decimator #(
    parameter INPUT_BINS        = `RP_FFT_SIZE,          // 2048
    parameter OUTPUT_BINS       = `RP_NUM_RANGE_BINS,    // 512
    parameter DECIMATION_FACTOR = `RP_DECIMATION_FACTOR  // 4
) (
    input wire clk,
    input wire reset_n,

    // Input from matched filter
    input wire signed [15:0] range_i_in,
    input wire signed [15:0] range_q_in,
    input wire range_valid_in,

    // Output to Doppler processor
    output reg signed [15:0] range_i_out,
    output reg signed [15:0] range_q_out,
    output reg range_valid_out,
    output reg [`RP_RANGE_BIN_BITS-1:0] range_bin_index,  // 9-bit

    // Configuration
    input wire [1:0] decimation_mode,  // 00=decimate, 01=peak, 10=average
    input wire [10:0] start_bin,       // First input bin to process (11-bit for 2048)

    // Diagnostics
    output reg watchdog_timeout        // Pulses high for 1 cycle on watchdog reset

`ifdef FORMAL
    ,
    output wire [2:0]  fv_state,
    output wire [10:0] fv_in_bin_count,
    output wire [1:0]  fv_group_sample_count,
    output wire [8:0]  fv_output_bin_count,
    output wire [10:0] fv_skip_count
`endif
);

// Fix 5: Watchdog timeout — if no valid input arrives for WATCHDOG_LIMIT
// clocks while in ST_PROCESS or ST_SKIP, return to ST_IDLE to prevent hang.
// 256 clocks at 100MHz = 2.56us, well beyond normal inter-sample gap.
localparam WATCHDOG_LIMIT = 10'd256;

// ============================================================================
// INTERNAL SIGNALS
// ============================================================================

// Input bin counter (0..2047)
reg [10:0] in_bin_count;

// Group tracking
reg [1:0] group_sample_count;   // 0..3 within current group of 4
reg [8:0] output_bin_count;     // 0..511 output bin index

// State machine
reg [2:0] state;
localparam ST_IDLE    = 3'd0;
localparam ST_SKIP    = 3'd1;  // Skip first start_bin samples
localparam ST_PROCESS = 3'd2;
localparam ST_EMIT    = 3'd3;
localparam ST_DONE    = 3'd4;

// Skip counter for start_bin
reg [10:0] skip_count;

// Watchdog counter — counts consecutive clocks with no range_valid_in
reg [9:0] watchdog_count;

`ifdef FORMAL
assign fv_state              = state;
assign fv_in_bin_count       = in_bin_count;
assign fv_group_sample_count = group_sample_count;
assign fv_output_bin_count   = output_bin_count;
assign fv_skip_count         = skip_count;
`endif

// ============================================================================
// PEAK DETECTION (Mode 01)
// ============================================================================
// Track the sample with the largest magnitude in the current group of 4
reg signed [15:0] peak_i, peak_q;
reg [16:0] peak_mag;  // |I| + |Q| approximation
wire [16:0] cur_mag;

// Magnitude approximation: |I| + |Q| (avoids multiplier for sqrt(I²+Q²))
wire [15:0] abs_i = range_i_in[15] ? (~range_i_in + 1) : range_i_in;
wire [15:0] abs_q = range_q_in[15] ? (~range_q_in + 1) : range_q_in;
assign cur_mag = {1'b0, abs_i} + {1'b0, abs_q};

// ============================================================================
// AVERAGING (Mode 10)
// ============================================================================
// Accumulate I and Q separately, then divide by DECIMATION_FACTOR (>>2)
reg signed [17:0] sum_i, sum_q;  // 16 + 2 guard bits for sum of 4 values

// ============================================================================
// SIMPLE DECIMATION (Mode 00)
// ============================================================================
// Just take sample at offset (group_start + DECIMATION_FACTOR/2) for center
reg signed [15:0] decim_i, decim_q;

// ============================================================================
// MAIN STATE MACHINE
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state             <= ST_IDLE;
        in_bin_count      <= 11'd0;
        group_sample_count <= 2'd0;
        output_bin_count  <= 9'd0;
        skip_count        <= 11'd0;
        watchdog_count    <= 10'd0;
        watchdog_timeout  <= 1'b0;
        range_valid_out   <= 1'b0;
        range_i_out       <= 16'd0;
        range_q_out       <= 16'd0;
        range_bin_index   <= {`RP_RANGE_BIN_BITS{1'b0}};
        peak_i            <= 16'd0;
        peak_q            <= 16'd0;
        peak_mag          <= 17'd0;
        sum_i             <= 18'd0;
        sum_q             <= 18'd0;
        decim_i           <= 16'd0;
        decim_q           <= 16'd0;
    end else begin
        // Default: output not valid, watchdog not triggered
        range_valid_out  <= 1'b0;
        watchdog_timeout <= 1'b0;

        case (state)
        // ================================================================
        // IDLE: Wait for first valid input
        // ================================================================
        ST_IDLE: begin
            in_bin_count       <= 11'd0;
            group_sample_count <= 2'd0;
            output_bin_count   <= 9'd0;
            skip_count         <= 11'd0;
            watchdog_count     <= 10'd0;
            peak_i             <= 16'd0;
            peak_q             <= 16'd0;
            peak_mag           <= 17'd0;
            sum_i              <= 18'd0;
            sum_q              <= 18'd0;

            if (range_valid_in) begin
                in_bin_count <= 11'd1;

                if (start_bin > 11'd0) begin
                    // Need to skip 'start_bin' samples first
                    skip_count <= 11'd1;
                    state      <= ST_SKIP;
                end else begin
                    // No skip — process first sample immediately
                    state              <= ST_PROCESS;
                    group_sample_count <= 2'd1;

                    // Mode-specific first sample handling
                    case (decimation_mode)
                    2'b00: begin  // Simple decimation — check if center sample
                        if (2'd0 == (DECIMATION_FACTOR / 2)) begin
                            decim_i <= range_i_in;
                            decim_q <= range_q_in;
                        end
                    end
                    2'b01: begin  // Peak detection
                        peak_i   <= range_i_in;
                        peak_q   <= range_q_in;
                        peak_mag <= cur_mag;
                    end
                    2'b10: begin  // Averaging
                        sum_i <= {{2{range_i_in[15]}}, range_i_in};
                        sum_q <= {{2{range_q_in[15]}}, range_q_in};
                    end
                    default: ;
                    endcase
                end
            end
        end

        // ================================================================
        // SKIP: Discard input samples until start_bin reached
        // ================================================================
        ST_SKIP: begin
            if (range_valid_in) begin
                watchdog_count <= 10'd0;
                in_bin_count <= in_bin_count + 1;

                if (skip_count >= start_bin) begin
                    // Done skipping — this sample is the first to process
                    state              <= ST_PROCESS;
                    group_sample_count <= 2'd1;

                    case (decimation_mode)
                    2'b00: begin
                        if (2'd0 == (DECIMATION_FACTOR / 2)) begin
                            decim_i <= range_i_in;
                            decim_q <= range_q_in;
                        end
                    end
                    2'b01: begin
                        peak_i   <= range_i_in;
                        peak_q   <= range_q_in;
                        peak_mag <= cur_mag;
                    end
                    2'b10: begin
                        sum_i <= {{2{range_i_in[15]}}, range_i_in};
                        sum_q <= {{2{range_q_in[15]}}, range_q_in};
                    end
                    default: ;
                    endcase
                end else begin
                    skip_count <= skip_count + 1;
                end
            end else begin
                // No valid input — increment watchdog
                if (watchdog_count >= WATCHDOG_LIMIT - 1) begin
                    watchdog_timeout <= 1'b1;
                    state <= ST_IDLE;
                    `ifdef SIMULATION
                    $display("[RNG_DECIM] WATCHDOG: timeout in ST_SKIP after %0d idle clocks", WATCHDOG_LIMIT);
                    `endif
                end else begin
                    watchdog_count <= watchdog_count + 1;
                end
            end
        end

        // ================================================================
        // PROCESS: Accumulate samples within each group of DECIMATION_FACTOR
        // ================================================================
        ST_PROCESS: begin
            if (range_valid_in) begin
                watchdog_count <= 10'd0;
                in_bin_count <= in_bin_count + 1;

                // Mode-specific sample processing — always process
                // the current sample before checking overflow
                case (decimation_mode)
                2'b00: begin  // Simple decimation
                    if (group_sample_count == (DECIMATION_FACTOR / 2)) begin
                        decim_i <= range_i_in;
                        decim_q <= range_q_in;
                    end
                end
                2'b01: begin  // Peak detection
                    if (cur_mag > peak_mag) begin
                        peak_i   <= range_i_in;
                        peak_q   <= range_q_in;
                        peak_mag <= cur_mag;
                    end
                end
                2'b10: begin  // Averaging
                    sum_i <= sum_i + {{2{range_i_in[15]}}, range_i_in};
                    sum_q <= sum_q + {{2{range_q_in[15]}}, range_q_in};
                end
                default: ;
                endcase

                // Check if group is complete
                if (group_sample_count == DECIMATION_FACTOR - 1) begin
                    // Group complete — emit output
                    state <= ST_EMIT;
                    group_sample_count <= 2'd0;
                end else if (in_bin_count >= INPUT_BINS - 1) begin
                    // Overflow guard: consumed all input bins but group
                    // is not yet complete. Stop to prevent corruption of
                    // downstream Doppler BRAM if matched filter emits
                    // more than INPUT_BINS valid samples.
                    state <= ST_DONE;
                end else begin
                    group_sample_count <= group_sample_count + 1;
                end
            end else begin
                // No valid input — increment watchdog
                if (watchdog_count >= WATCHDOG_LIMIT - 1) begin
                    watchdog_timeout <= 1'b1;
                    state <= ST_IDLE;
                    `ifdef SIMULATION
                    $display("[RNG_DECIM] WATCHDOG: timeout in ST_PROCESS after %0d idle clocks", WATCHDOG_LIMIT);
                    `endif
                end else begin
                    watchdog_count <= watchdog_count + 1;
                end
            end
        end

        // ================================================================
        // EMIT: Output one decimated range bin
        // ================================================================
        ST_EMIT: begin
            range_valid_out <= 1'b1;
            range_bin_index <= output_bin_count;

            case (decimation_mode)
            2'b00: begin  // Simple decimation
                range_i_out <= decim_i;
                range_q_out <= decim_q;
            end
            2'b01: begin  // Peak detection
                range_i_out <= peak_i;
                range_q_out <= peak_q;
            end
            2'b10: begin  // Averaging (sum >> 2 = divide by 4)
                range_i_out <= sum_i[17:2];
                range_q_out <= sum_q[17:2];
            end
            default: begin
                range_i_out <= 16'd0;
                range_q_out <= 16'd0;
            end
            endcase

            // Reset group accumulators
            peak_i   <= 16'd0;
            peak_q   <= 16'd0;
            peak_mag <= 17'd0;
            sum_i    <= 18'd0;
            sum_q    <= 18'd0;

            // Advance output bin
            output_bin_count <= output_bin_count + 1;

            // Check if all output bins emitted
            if (output_bin_count == OUTPUT_BINS - 1) begin
                state <= ST_DONE;
            end else begin
                // If we already have valid input waiting, process it immediately
                if (range_valid_in) begin
                    state              <= ST_PROCESS;
                    group_sample_count <= 2'd1;
                    in_bin_count       <= in_bin_count + 1;

                    case (decimation_mode)
                    2'b00: begin
                        if (2'd0 == (DECIMATION_FACTOR / 2)) begin
                            decim_i <= range_i_in;
                            decim_q <= range_q_in;
                        end
                    end
                    2'b01: begin
                        peak_i   <= range_i_in;
                        peak_q   <= range_q_in;
                        peak_mag <= cur_mag;
                    end
                    2'b10: begin
                        sum_i <= {{2{range_i_in[15]}}, range_i_in};
                        sum_q <= {{2{range_q_in[15]}}, range_q_in};
                    end
                    default: ;
                    endcase
                end else begin
                    state <= ST_PROCESS;
                    group_sample_count <= 2'd0;
                end
            end
        end

        // ================================================================
        // DONE: All 512 output bins emitted, return to idle
        // ================================================================
        ST_DONE: begin
            state <= ST_IDLE;

            `ifdef SIMULATION
            $display("[RNG_DECIM] Frame complete: %0d output bins emitted", OUTPUT_BINS);
            `endif
        end

        default: state <= ST_IDLE;
        endcase
    end
end

endmodule
