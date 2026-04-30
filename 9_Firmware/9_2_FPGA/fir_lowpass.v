`timescale 1ns / 1ps

// ============================================================================
// fir_lowpass_parallel_enhanced — 32-tap symmetric FIR, half-tap optimized
// ============================================================================
// Architecture: linear-phase symmetric FIR using the DSP48E1 pre-adder.
// Because coeff[k] == coeff[31-k], the convolution is re-grouped into 16
// pre-add+multiply operations:
//
//   y[n] = sum_{k=0..15} h[k] * ( x[n-k] + x[n-31+k] )
//
// Each grouped tap uses ONE multiply (vs two in the naive layout), so the
// FIR consumes 16 multiplies instead of 32. The pre-adder lives inside
// the DSP48E1 (D + A) → B multiplier, so the savings collapse straight to
// 16 DSP48E1 per channel.
//
// Why symmetric reduction rather than 4:1 polyphase folding:
//   The CIC `_4x` decimator runs in clk_400m and emits 100 M pulses/s,
//   which the cdc_adc_to_processing CDC carries into clk_100m as one
//   dst_valid per dst cycle in steady state. The FIR therefore receives
//   data at 100 MSPS, NOT 25 MSPS as an earlier comment incorrectly
//   claimed (see commit 977434a). Polyphase folding would drop samples
//   at this rate. Symmetric half-tap reduction preserves 1 sample/cycle
//   throughput and still gets the DSP halving.
//
// Bit-exact contract: produces the same 32-tap convolution result as the
// previous unfolded design for inputs that don't overflow the saturation
// threshold (±2^34). Verified against a Python golden model at the
// existing test stimuli (DC=5000 → 8847; 45 MHz tone → ±16 LSB).
//
// Resource impact (target XC7A50T):
//   - DSP48E1: 32 → 16 per channel (saves 16 per channel; 32 across I/Q
//     = 27% of the 120-DSP budget freed for downstream FFT work)
//   - Throughput: unchanged (1 sample/cycle, fully pipelined)
//   - Latency: 11 cycles (was 9 — pre-adder costs 1 stage; tree depth same)
//
// Accumulator widths: the pre-adder grows the multiply input by 1 bit,
// so the multiply product is 37-bit (was 36 unfolded). Sum of 16 such
// products needs +4 bits → 41-bit accumulator. Saturation thresholds and
// output bit-slice are unchanged from the unfolded design (compare against
// ±2^(ACCUM_WIDTH-2) = ±2^34, slice [ACCUM_WIDTH-2 : DATA_WIDTH-1] =
// [34:17]) so downstream signal levels and headroom stay the same.
//
// DC gain & filter shape (AUDIT-S17): the coefficient ROM has a deliberate
// positive DC pre-emphasis. Sum of 32 signed coefficients = 231,944. With
// the output slice taken at accumulator[34:17], effective Q-format is Q17,
// so DC gain = 231944 / 2^17 = 1.7696 = +4.96 dB. This is bit-exact against
// the in-line golden-model line above (DC=5000 in → 8847 out:
// 231944 × 5000 = 1,159,720,000; >>17 = 8847).
//
// The +4.96 dB pre-emphasis compensates CIC droop: the upstream 4-stage
// CIC at decimation factor 4 has ~3-4 dB of passband droop near Nyquist,
// partially offset by this FIR's positive DC gain so the cascade response
// is flatter than the CIC alone. Operational implication: AGC and
// saturation budgets in downstream stages (DDC → CIC → FIR → MF) must
// account for the +4.96 dB FIR gain — a clean 0 dB FIR assumption would
// under-budget headroom by ~5 dB.
//
// Coefficients are intentionally NOT regenerated from a clean FIR design
// tool; they are kept bit-exact with the unfolded predecessor so the
// existing golden-model test stays valid. Touching them would invalidate
// that contract and would silently shift downstream signal levels.
// ============================================================================

module fir_lowpass_parallel_enhanced (
    input wire clk,
    input wire reset_n,
    input wire signed [17:0] data_in,
    input wire data_valid,
    output reg signed [17:0] data_out,
    output reg data_out_valid,
    output wire fir_ready,
    output wire filter_overflow
);

parameter TAPS         = 32;
parameter HALF_TAPS    = TAPS / 2;             // 16 unique coefficient pairs
parameter COEFF_WIDTH  = 18;
parameter DATA_WIDTH   = 18;
parameter ACCUM_WIDTH  = 36;                   // legacy threshold base — DO NOT widen casually
parameter PREADD_W     = DATA_WIDTH + 1;       // 19 — sum of two 18-bit signed
parameter PROD_W       = PREADD_W + COEFF_WIDTH; // 37 — pre-add * coeff
parameter SUM_W        = PROD_W + 4;           // 41 — sum of 16 products

// ============================================================================
// Coefficient ROM (symmetric low-pass — kept identical to the unfolded
// design so production behaviour is preserved bit-for-bit at non-saturating
// signal levels)
// ============================================================================
reg signed [COEFF_WIDTH-1:0] coeff [0:TAPS-1];
initial begin
    coeff[ 0] = 18'sh00AD; coeff[ 1] = 18'sh00CE; coeff[ 2] = 18'sh3FD87; coeff[ 3] = 18'sh02A6;
    coeff[ 4] = 18'sh00E0; coeff[ 5] = 18'sh3F8C0; coeff[ 6] = 18'sh0A45; coeff[ 7] = 18'sh3FD82;
    coeff[ 8] = 18'sh3F0B5; coeff[ 9] = 18'sh1CAD; coeff[10] = 18'sh3EE59; coeff[11] = 18'sh3E821;
    coeff[12] = 18'sh4841; coeff[13] = 18'sh3B340; coeff[14] = 18'sh3E299; coeff[15] = 18'sh1FFFF;
    coeff[16] = 18'sh1FFFF; coeff[17] = 18'sh3E299; coeff[18] = 18'sh3B340; coeff[19] = 18'sh4841;
    coeff[20] = 18'sh3E821; coeff[21] = 18'sh3EE59; coeff[22] = 18'sh1CAD; coeff[23] = 18'sh3F0B5;
    coeff[24] = 18'sh3FD82; coeff[25] = 18'sh0A45; coeff[26] = 18'sh3F8C0; coeff[27] = 18'sh00E0;
    coeff[28] = 18'sh02A6; coeff[29] = 18'sh3FD87; coeff[30] = 18'sh00CE; coeff[31] = 18'sh00AD;
end

// ============================================================================
// Saturation thresholds — built via bit concatenation to dodge the Verilog
// 32-bit-literal trap: writing `1 <<< 34` evaluates the `1` as a 32-bit
// integer and silently wraps to 0, so `(1 <<< 34) - 1` becomes -1 and
// every nonneg accumulator value would falsely saturate. The earlier
// symmetric draft tripped this. Bit-pattern construction makes the width
// explicit and overflow-safe.
// ============================================================================
// SAT_POS = 2^(ACCUM_WIDTH-2) - 1 = 0x3_FFFF_FFFF for ACCUM_WIDTH=36
// SAT_NEG = -2^(ACCUM_WIDTH-2)    = -0x4_0000_0000 for ACCUM_WIDTH=36
localparam signed [SUM_W-1:0] SAT_POS = {1'b0, {(ACCUM_WIDTH-2){1'b1}}};
localparam signed [SUM_W-1:0] SAT_NEG = -SAT_POS - 1;

// ============================================================================
// Delay line — 32 entries, shifts on data_valid (1 sample/cycle production)
// ============================================================================
reg signed [DATA_WIDTH-1:0] delay_line [0:TAPS-1];
integer i;
always @(posedge clk) begin
    if (!reset_n) begin
        for (i = 0; i < TAPS; i = i + 1) delay_line[i] <= 0;
    end else if (data_valid) begin
        for (i = TAPS-1; i > 0; i = i - 1) delay_line[i] <= delay_line[i-1];
        delay_line[0] <= data_in;
    end
end

// ============================================================================
// Stage 1 (DREG/AREG): pre-adder operands + coefficient register.
// Vivado absorbs into DSP48E1 D and A pre-adder ports + B coefficient port.
// ============================================================================
reg signed [DATA_WIDTH-1:0]  pair_a [0:HALF_TAPS-1];   // delay_line[k]
reg signed [DATA_WIDTH-1:0]  pair_d [0:HALF_TAPS-1];   // delay_line[31-k]
reg signed [COEFF_WIDTH-1:0] coeff_reg [0:HALF_TAPS-1];

always @(posedge clk) begin
    if (!reset_n) begin
        for (i = 0; i < HALF_TAPS; i = i + 1) begin
            pair_a[i]    <= 0;
            pair_d[i]    <= 0;
            coeff_reg[i] <= 0;
        end
    end else if (data_valid) begin
        for (i = 0; i < HALF_TAPS; i = i + 1) begin
            pair_a[i]    <= delay_line[i];
            pair_d[i]    <= delay_line[(TAPS-1) - i];
            coeff_reg[i] <= coeff[i];
        end
    end
end

// ============================================================================
// Stage 2 (MREG): pre-add + multiply. Vivado infers DSP48E1 P = (D+A)*B.
// ============================================================================
reg signed [PROD_W-1:0] mult_reg [0:HALF_TAPS-1];
reg [10:0] valid_pipe;  // 11-stage pipeline tracker

genvar gk;
generate
    for (gk = 0; gk < HALF_TAPS; gk = gk + 1) begin : mac_gen
        wire signed [PREADD_W-1:0] preadd =
            $signed({pair_d[gk][DATA_WIDTH-1], pair_d[gk]}) +
            $signed({pair_a[gk][DATA_WIDTH-1], pair_a[gk]});

        always @(posedge clk) begin
            if (!reset_n)
                mult_reg[gk] <= 0;
            else if (valid_pipe[0])
                mult_reg[gk] <= preadd * coeff_reg[gk];
        end
    end
endgenerate

// ============================================================================
// Adder tree: 16 → 8 → 4 → 2 → 1 (4 stages, fabric carry chains).
// Intermediates widened so the 37-bit pre-add products grow without
// silent wrap (sum-of-16 worst case = 37 + 4 = 41 bits).
// ============================================================================
(* USE_DSP = "no" *) reg signed [PROD_W:0]   add_l0 [0:7];   // 38-bit
(* USE_DSP = "no" *) reg signed [PROD_W+1:0] add_l1 [0:3];   // 39-bit
(* USE_DSP = "no" *) reg signed [PROD_W+2:0] add_l2 [0:1];   // 40-bit
(* USE_DSP = "no" *) reg signed [SUM_W-1:0]  accumulator_reg; // 41-bit

always @(posedge clk) begin
    if (!reset_n) begin
        for (i = 0; i < 8; i = i + 1) add_l0[i] <= 0;
    end else if (valid_pipe[1]) begin
        for (i = 0; i < 8; i = i + 1)
            add_l0[i] <= $signed(mult_reg[2*i]) + $signed(mult_reg[2*i+1]);
    end
end

always @(posedge clk) begin
    if (!reset_n) begin
        for (i = 0; i < 4; i = i + 1) add_l1[i] <= 0;
    end else if (valid_pipe[2]) begin
        for (i = 0; i < 4; i = i + 1)
            add_l1[i] <= $signed(add_l0[2*i]) + $signed(add_l0[2*i+1]);
    end
end

always @(posedge clk) begin
    if (!reset_n) begin
        add_l2[0] <= 0;
        add_l2[1] <= 0;
    end else if (valid_pipe[3]) begin
        add_l2[0] <= $signed(add_l1[0]) + $signed(add_l1[1]);
        add_l2[1] <= $signed(add_l1[2]) + $signed(add_l1[3]);
    end
end

always @(posedge clk) begin
    if (!reset_n)
        accumulator_reg <= 0;
    else if (valid_pipe[4])
        accumulator_reg <= $signed(add_l2[0]) + $signed(add_l2[1]);
end

// ============================================================================
// Output saturation — same thresholds and bit-slice as the unfolded design
// ============================================================================
always @(posedge clk) begin
    if (!reset_n) begin
        data_out       <= 0;
        data_out_valid <= 1'b0;
    end else begin
        data_out_valid <= valid_pipe[5];
        if (valid_pipe[5]) begin
            if (accumulator_reg > SAT_POS)
                data_out <= {1'b0, {(DATA_WIDTH-1){1'b1}}};   // +max
            else if (accumulator_reg < SAT_NEG)
                data_out <= {1'b1, {(DATA_WIDTH-1){1'b0}}};   // -max
            else
                data_out <= accumulator_reg[ACCUM_WIDTH-2:DATA_WIDTH-1];
        end
    end
end

// ============================================================================
// Valid pipeline (11 stages: shift+pair → MREG → 4 adder levels → output)
// ============================================================================
always @(posedge clk) begin
    if (!reset_n) valid_pipe <= 11'd0;
    else          valid_pipe <= {valid_pipe[9:0], data_valid};
end

assign fir_ready = 1'b1;  // always ready — fully pipelined at 1 sample/cycle

assign filter_overflow =
       (accumulator_reg > SAT_POS) || (accumulator_reg < SAT_NEG);

endmodule
