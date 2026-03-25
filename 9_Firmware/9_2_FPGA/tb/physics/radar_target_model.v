`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// radar_target_model.v — Shared synthetic radar target generator
//
// Generates physically accurate received radar echo signals for any
// combination of targets with known range, velocity, RCS, and noise floor.
//
// This module is the foundation for all physics-aware testbenches.
// It produces baseband I/Q samples that represent the matched-filter
// output (range profile) for each chirp, incorporating:
//   - Correct range-bin placement
//   - Chirp-to-chirp Doppler phase progression (slow-time)
//   - Configurable SNR via additive Gaussian-approximated noise
//   - Staggered-PRF awareness (different PRI for long vs short chirps)
//
// Usage: Instantiate one per testbench. Configure targets via parameters
//        or register interface. Clock at system rate (100 MHz).
//
// IMPORTANT: This model generates post-matched-filter range profiles
//            (complex I/Q per range bin per chirp). It does NOT model
//            the raw IF signal or ADC sampling. For DDC/matched-filter
//            testing, use radar_if_target_model.v instead.
//////////////////////////////////////////////////////////////////////////////

module radar_target_model #(
    parameter NUM_TARGETS       = 4,
    parameter NUM_RANGE_BINS    = 64,
    parameter DATA_WIDTH        = 16,
    parameter LAMBDA_UM         = 28571,     // wavelength in micrometers (2.857cm @ 10.5GHz)
    // Default target parameters (overridable per-instance)
    parameter integer TGT0_RANGE_BIN = 10,
    parameter integer TGT0_VEL_MPS_X100 = 2000,  // velocity * 100 (20.00 m/s)
    parameter integer TGT0_AMPLITUDE  = 8000,     // peak I/Q amplitude (sets SNR)
    parameter integer TGT1_RANGE_BIN = 30,
    parameter integer TGT1_VEL_MPS_X100 = -1500,  // -15.00 m/s (approaching)
    parameter integer TGT1_AMPLITUDE  = 6000,
    parameter integer TGT2_RANGE_BIN = 50,
    parameter integer TGT2_VEL_MPS_X100 = 0,      // stationary (clutter)
    parameter integer TGT2_AMPLITUDE  = 12000,
    parameter integer TGT3_RANGE_BIN = 20,
    parameter integer TGT3_VEL_MPS_X100 = 4000,   // 40.00 m/s (fast)
    parameter integer TGT3_AMPLITUDE  = 4000,
    parameter integer NOISE_AMPLITUDE = 200        // noise floor per sample
) (
    input wire clk,
    input wire reset_n,

    // Control interface
    input wire        generate_chirp,    // Pulse to generate one chirp's range profile
    input wire [5:0]  chirp_index,       // Which chirp in the frame (0..31)
    input wire        is_long_chirp,     // 1=long PRI, 0=short PRI
    input wire [31:0] pri_clocks,        // PRI in clock cycles (for Doppler phase calc)

    // Output: range profile for this chirp
    output reg signed [DATA_WIDTH-1:0] range_i_out,
    output reg signed [DATA_WIDTH-1:0] range_q_out,
    output reg                         range_valid,
    output reg [5:0]                   range_bin_out,
    output reg                         chirp_done       // Pulsed after all bins emitted
);

// ============================================================================
// INTERNAL STATE
// ============================================================================

// Phase accumulator per target (slow-time Doppler phase)
// Phase is in units of 2*pi / 2^24 (24-bit fractional turns)
reg signed [31:0] doppler_phase [0:NUM_TARGETS-1];

// Per-target config arrays (initialized from parameters)
reg [5:0]         tgt_range_bin  [0:NUM_TARGETS-1];
reg signed [31:0] tgt_vel_x100   [0:NUM_TARGETS-1]; // velocity * 100 in m/s
reg signed [15:0] tgt_amplitude  [0:NUM_TARGETS-1];

// LFSR for pseudo-random noise
reg [31:0] lfsr;

// FSM
reg [2:0] state;
localparam S_IDLE     = 3'd0;
localparam S_GENERATE = 3'd1;
localparam S_DONE     = 3'd2;

reg [5:0] bin_counter;

// ============================================================================
// INITIALIZATION
// ============================================================================
initial begin
    tgt_range_bin[0] = TGT0_RANGE_BIN;
    tgt_vel_x100[0]  = TGT0_VEL_MPS_X100;
    tgt_amplitude[0]  = TGT0_AMPLITUDE;

    tgt_range_bin[1] = TGT1_RANGE_BIN;
    tgt_vel_x100[1]  = TGT1_VEL_MPS_X100;
    tgt_amplitude[1]  = TGT1_AMPLITUDE;

    tgt_range_bin[2] = TGT2_RANGE_BIN;
    tgt_vel_x100[2]  = TGT2_VEL_MPS_X100;
    tgt_amplitude[2]  = TGT2_AMPLITUDE;

    tgt_range_bin[3] = TGT3_RANGE_BIN;
    tgt_vel_x100[3]  = TGT3_VEL_MPS_X100;
    tgt_amplitude[3]  = TGT3_AMPLITUDE;
end

// ============================================================================
// DOPPLER PHASE COMPUTATION
//
// For each target, the Doppler phase shift per chirp is:
//   delta_phi = 2*pi * 2 * v / lambda * PRI
//
// In fixed-point (2^24 = full turn):
//   delta_phi_fp = (2 * v_x100 * PRI_clocks * 2^24) / (lambda_um * f_clk_x100)
//
// We precompute this when generate_chirp is pulsed, and accumulate.
// ============================================================================

// Sine/cosine lookup (quarter-wave, 256 entries, 16-bit)
// Simplified: use a reasonable approximation for simulation
// Phase input: [23:0] where 2^24 = 2*pi
// We use the top 10 bits as a LUT index (1024-entry effective)

function signed [15:0] sin_lut;
    input [23:0] phase;
    reg [9:0] index;
    reg [1:0] quadrant;
    reg signed [15:0] base_val;
    begin
        quadrant = phase[23:22];
        index = phase[21:12];

        // Linear approximation of sin() for simulation
        // sin(x) ≈ x for small x, with proper quadrant mapping
        case (quadrant)
            2'b00: base_val = $signed({1'b0, index, 5'b0});          // 0..pi/2: positive rising
            2'b01: base_val = $signed({1'b0, ~index, 5'b0});         // pi/2..pi: positive falling
            2'b10: base_val = -$signed({1'b0, index, 5'b0});         // pi..3pi/2: negative
            2'b11: base_val = -$signed({1'b0, ~index, 5'b0});        // 3pi/2..2pi: negative rising
        endcase
        sin_lut = base_val;
    end
endfunction

function signed [15:0] cos_lut;
    input [23:0] phase;
    begin
        // cos(x) = sin(x + pi/2)
        cos_lut = sin_lut(phase + 24'h400000);
    end
endfunction

// ============================================================================
// LFSR NOISE GENERATOR (32-bit Galois LFSR)
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        lfsr <= 32'hDEADBEEF;
    end else begin
        lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
    end
end

// Noise: take top bits of LFSR, scale by NOISE_AMPLITUDE
wire signed [15:0] noise_i = ($signed(lfsr[15:0]) * NOISE_AMPLITUDE) >>> 15;
wire signed [15:0] noise_q = ($signed(lfsr[31:16]) * NOISE_AMPLITUDE) >>> 15;

// ============================================================================
// MAIN FSM — Generate range profile one bin at a time
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state       <= S_IDLE;
        range_valid <= 1'b0;
        chirp_done  <= 1'b0;
        range_i_out <= 0;
        range_q_out <= 0;
        range_bin_out <= 0;
        bin_counter <= 0;
        doppler_phase[0] <= 0;
        doppler_phase[1] <= 0;
        doppler_phase[2] <= 0;
        doppler_phase[3] <= 0;
    end else begin
        range_valid <= 1'b0;
        chirp_done  <= 1'b0;

        case (state)
        S_IDLE: begin
            if (generate_chirp) begin
                bin_counter <= 0;
                state <= S_GENERATE;
            end
        end

        S_GENERATE: begin
            // For each range bin, sum contributions from all targets
            // Target contributes only to its assigned range bin
            begin : gen_sample
                reg signed [31:0] sum_i, sum_q;
                reg signed [15:0] tgt_i, tgt_q;
                integer t;

                sum_i = 0;
                sum_q = 0;

                for (t = 0; t < NUM_TARGETS; t = t + 1) begin
                    if (bin_counter == tgt_range_bin[t]) begin
                        // Target present at this range bin
                        // I = A * cos(doppler_phase), Q = A * sin(doppler_phase)
                        // Use 32-bit intermediate to avoid 16×16 truncation bug
                        begin : tgt_mul_blk
                            reg signed [31:0] prod_i_w, prod_q_w;
                            prod_i_w = tgt_amplitude[t] * cos_lut(doppler_phase[t][23:0]);
                            prod_q_w = tgt_amplitude[t] * sin_lut(doppler_phase[t][23:0]);
                            tgt_i = prod_i_w >>> 15;
                            tgt_q = prod_q_w >>> 15;
                        end
                        sum_i = sum_i + tgt_i;
                        sum_q = sum_q + tgt_q;
                    end
                end

                // Add noise
                sum_i = sum_i + noise_i;
                sum_q = sum_q + noise_q;

                // Saturate to DATA_WIDTH
                if (sum_i > 32767) range_i_out <= 16'h7FFF;
                else if (sum_i < -32768) range_i_out <= 16'h8000;
                else range_i_out <= sum_i[15:0];

                if (sum_q > 32767) range_q_out <= 16'h7FFF;
                else if (sum_q < -32768) range_q_out <= 16'h8000;
                else range_q_out <= sum_q[15:0];
            end

            range_bin_out <= bin_counter;
            range_valid <= 1'b1;

            if (bin_counter >= NUM_RANGE_BINS - 1) begin
                state <= S_DONE;
            end else begin
                bin_counter <= bin_counter + 1;
            end
        end

        S_DONE: begin
            // Update Doppler phases for all targets
            // delta_phi (24-bit fractional turns) = 2 * v * PRI / lambda * 2^24
            // = 2 * (v_x100/100) * (pri_clocks / f_clk) / (lambda_um * 1e-6) * 2^24
            // Simplified for 100 MHz clock:
            // = v_x100 * pri_clocks * 2^25 / (100 * lambda_um * 100)
            // = v_x100 * pri_clocks * 2^25 / (10000 * lambda_um)
            // Pre-scale to avoid overflow: (v_x100 * pri_clocks) / lambda_um * (2^25 / 10000)
            // 2^25/10000 ≈ 3.3554 ≈ 3 + 0.3554
            // Use: delta = (v_x100 * pri_clocks * 3356) / (lambda_um * 1000)
            begin : update_phases
                integer t;
                reg signed [63:0] delta;
                for (t = 0; t < NUM_TARGETS; t = t + 1) begin
                    // Compute phase increment for this PRI
                    // delta_phi (24-bit frac turns) = 2 * v * PRI / lambda * 2^24
                    // = v_x100 * pri_clocks * 2^25 / (10000 * lambda_um)
                    // 2^25 / 10000 = 3355.4 ≈ 3356
                    // So: delta = v_x100 * pri_clocks * 3356 / lambda_um
                    delta = (tgt_vel_x100[t] * $signed({1'b0, pri_clocks}) * 3356)
                            / $signed(LAMBDA_UM);
                    doppler_phase[t] <= doppler_phase[t] + delta[31:0];
                end
            end

            chirp_done <= 1'b1;
            state <= S_IDLE;
        end

        default: state <= S_IDLE;
        endcase
    end
end

endmodule
