`timescale 1ns / 1ps

// fpga_self_test.v — Board Bring-Up Smoke Test Controller
//
// Triggered by host opcode 0x30. Exercises each subsystem independently:
//   Test 0: BRAM write/read pattern (walking 1s)
//   Test 1: CIC impulse response check (known input → expected output)
//   Test 2: FFT known-input test (DC input → bin 0 peak)
//   Test 3: Arithmetic / saturating-add check
//   Test 4: ADC raw data capture (dump N samples to host)
//
// Results reported back via a status register readable by host (opcode 0x31).
// Each test produces a PASS/FAIL bit in result_flags[4:0].
//
// Integration: radar_system_top.v wires host_self_test_trigger (from opcode 0x30)
// to this module's `trigger` input, and reads `result_flags` / `result_valid`
// via opcode 0x31.
//
// Resource cost: ~200 LUTs, 1 BRAM (test pattern), 0 DSP.

module fpga_self_test (
    input  wire        clk,
    input  wire        reset_n,

    // Control
    input  wire        trigger,         // 1-cycle pulse from host (opcode 0x30)
    output reg         busy,            // High while tests are running
    output reg         result_valid,    // Pulses when all tests complete
    output reg  [4:0]  result_flags,    // Per-test PASS(1)/FAIL(0)
    output reg  [7:0]  result_detail,   // Diagnostic detail (first failing test ID + info)

    // ADC raw capture interface (active during Test 4)
    input  wire [15:0] adc_data_in,     // Raw ADC sample (from ad9484_interface)
    input  wire        adc_valid_in,    // ADC sample valid
    output reg         capture_active,  // High during ADC capture window
    output reg  [15:0] capture_data,    // Captured ADC sample for USB readout
    output reg         capture_valid    // Pulse: new captured sample available
);

// ============================================================================
// FSM States
// ============================================================================
localparam [3:0] ST_IDLE       = 4'd0,
                 ST_BRAM_WR    = 4'd1,
                 ST_BRAM_GAP   = 4'd2,   // 1-cycle gap: let last write complete
                 ST_BRAM_RD    = 4'd3,
                 ST_BRAM_CHK   = 4'd4,
                 ST_CIC_SETUP  = 4'd5,
                 ST_CIC_CHECK  = 4'd6,
                 ST_FFT_SETUP  = 4'd7,
                 ST_FFT_CHECK  = 4'd8,
                 ST_ARITH      = 4'd9,
                 ST_ADC_CAP    = 4'd10,
                 ST_DONE       = 4'd11;

reg [3:0] state;

// ============================================================================
// Test 0: BRAM Write/Read Pattern
// ============================================================================
// Uses a small embedded BRAM (64×16) with walking-1 pattern.
localparam BRAM_DEPTH = 64;
localparam BRAM_AW    = 6;

reg  [15:0] test_bram [0:BRAM_DEPTH-1];
reg  [BRAM_AW-1:0] bram_addr;
reg  [15:0] bram_wr_data;
reg  [15:0] bram_rd_data;
reg         bram_pass;

// Synchronous BRAM write — use walking_one directly to avoid pipeline lag
always @(posedge clk) begin
    if (state == ST_BRAM_WR)
        test_bram[bram_addr] <= walking_one(bram_addr);
end

// Synchronous BRAM read (1-cycle latency)
always @(posedge clk) begin
    bram_rd_data <= test_bram[bram_addr];
end

// Walking-1 pattern: address → (1 << (addr % 16))
function [15:0] walking_one;
    input [BRAM_AW-1:0] addr;
    begin
        walking_one = 16'd1 << (addr[3:0]);
    end
endfunction

// ============================================================================
// Test 3: Arithmetic Check
// ============================================================================
// Verify saturating signed add (same logic as mti_canceller.v)
function [15:0] sat_add;
    input signed [15:0] a;
    input signed [15:0] b;
    reg signed [16:0] sum_full;
    begin
        sum_full = {a[15], a} + {b[15], b};
        if (sum_full > 17'sd32767)
            sat_add = 16'sd32767;
        else if (sum_full < -17'sd32768)
            sat_add = -16'sd32768;
        else
            sat_add = sum_full[15:0];
    end
endfunction

reg arith_pass;

// ============================================================================
// Counter / Control
// ============================================================================
reg [9:0] step_cnt;   // General-purpose step counter (up to 1024)
reg [9:0] adc_cap_cnt;
localparam ADC_CAP_SAMPLES = 256;  // Number of raw ADC samples to capture

// Pipeline register for BRAM read verification (accounts for 1-cycle read latency)
reg [BRAM_AW-1:0] bram_rd_addr_d;
reg bram_rd_valid;

// ============================================================================
// AUDIT-S19/S20/S21: real Test 1/2/4 state (replaces pre-fix tautologies)
// ============================================================================
// Test 1 (CIC integrator impulse response)
reg signed [31:0] cic_accum;
reg signed [15:0] cic_input;
// Test 2 (radix-2 butterfly with complex twiddle)
reg signed [15:0] fft_a_re, fft_b_re, fft_w_re, fft_w_im;
reg signed [31:0] fft_wb_re, fft_wb_im;
reg signed [16:0] fft_aprime_re, fft_aprime_im;
reg signed [16:0] fft_bprime_re, fft_bprime_im;
// Test 4 (ADC activity check — min/max over capture window)
reg signed [15:0] adc_min, adc_max;
localparam signed [15:0] ADC_RANGE_THRESHOLD = 16'sd10;

// ============================================================================
// Main FSM
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state         <= ST_IDLE;
        busy          <= 1'b0;
        result_valid  <= 1'b0;
        result_flags  <= 5'b00000;
        result_detail <= 8'd0;
        bram_addr     <= 0;
        bram_wr_data  <= 16'd0;
        bram_pass     <= 1'b1;
        arith_pass    <= 1'b1;
        step_cnt      <= 0;
        capture_active <= 1'b0;
        capture_data  <= 16'd0;
        capture_valid <= 1'b0;
        adc_cap_cnt   <= 0;
        bram_rd_addr_d <= 0;
        bram_rd_valid  <= 1'b0;
        // AUDIT-S19/S20/S21
        cic_accum     <= 32'sd0;
        cic_input     <= 16'sd0;
        fft_a_re      <= 16'sd0;
        fft_b_re      <= 16'sd0;
        fft_w_re      <= 16'sd0;
        fft_w_im      <= 16'sd0;
        fft_wb_re     <= 32'sd0;
        fft_wb_im     <= 32'sd0;
        fft_aprime_re <= 17'sd0;
        fft_aprime_im <= 17'sd0;
        fft_bprime_re <= 17'sd0;
        fft_bprime_im <= 17'sd0;
        adc_min       <= 16'sd0;
        adc_max       <= 16'sd0;
    end else begin
        // Default one-shot signals
        result_valid  <= 1'b0;
        capture_valid <= 1'b0;
        bram_rd_valid <= 1'b0;

        case (state)
        // ============================================================
        // IDLE: Wait for trigger
        // ============================================================
        ST_IDLE: begin
            if (trigger) begin
                busy          <= 1'b1;
                result_flags  <= 5'b00000;
                result_detail <= 8'd0;
                bram_pass     <= 1'b1;
                arith_pass    <= 1'b1;
                bram_addr     <= 0;
                step_cnt      <= 0;
                state         <= ST_BRAM_WR;
            end
        end

        // ============================================================
        // Test 0: BRAM Write Phase — write walking-1 pattern
        // ============================================================
        ST_BRAM_WR: begin
            if (bram_addr == BRAM_DEPTH - 1) begin
                bram_addr <= 0;
                state     <= ST_BRAM_GAP;
            end else begin
                bram_addr <= bram_addr + 1;
            end
        end

        // 1-cycle gap: ensures last BRAM write completes before reads begin
        ST_BRAM_GAP: begin
            bram_addr <= 0;
            state     <= ST_BRAM_RD;
        end

        // ============================================================
        // Test 0: BRAM Read Phase — issue reads
        // ============================================================
        ST_BRAM_RD: begin
            // BRAM read has 1-cycle latency: issue address, check next cycle
            bram_rd_addr_d <= bram_addr;
            bram_rd_valid  <= 1'b1;

            if (bram_addr == BRAM_DEPTH - 1) begin
                state <= ST_BRAM_CHK;
            end else begin
                bram_addr <= bram_addr + 1;
            end
        end

        // ============================================================
        // Test 0: BRAM Check — verify last read, finalize
        // ============================================================
        ST_BRAM_CHK: begin
            // Check final read (pipeline delay)
            if (bram_rd_data != walking_one(bram_rd_addr_d)) begin
                bram_pass <= 1'b0;
                result_detail <= {4'd0, bram_rd_addr_d[3:0]};
            end
            result_flags[0] <= bram_pass;
            state <= ST_CIC_SETUP;
            step_cnt <= 0;
        end

        // ============================================================
        // Test 1: CIC integrator impulse response (AUDIT-S19 fix)
        // ============================================================
        // Pre-fix this state set `result_flags[1] <= 1'b1` unconditionally
        // ("always true for simple check") so a broken integrator path on
        // the silicon would still PASS. Now drives a real impulse {5,0,0,...}
        // through y[n] = y[n-1] + x[n] and checks the registered accumulator
        // value at the end. Catches stuck-at, broken adder, or sign-extension
        // bug in the arithmetic path.
        ST_CIC_SETUP: begin
            if (step_cnt == 0) begin
                cic_accum <= 32'sd0;
                cic_input <= 16'sd5;        // impulse value
                step_cnt  <= 1;
            end else if (step_cnt < 8) begin
                cic_accum <= cic_accum +
                             {{16{cic_input[15]}}, cic_input};   // sign-extend
                cic_input <= 16'sd0;        // zero-pad after impulse
                step_cnt  <= step_cnt + 1;
            end else begin
                // After impulse + 6 zeros, integrator holds at 5 (step response)
                if (cic_accum == 32'sd5) begin
                    result_flags[1] <= 1'b1;
                end else begin
                    result_flags[1] <= 1'b0;
                    result_detail   <= 8'hC1;   // CIC fail marker
                end
                state    <= ST_FFT_SETUP;
                step_cnt <= 0;
            end
        end

        // ============================================================
        // Test 2: Radix-2 butterfly with twiddle multiply (AUDIT-S20 fix)
        // ============================================================
        // Pre-fix this evaluated `(16'sd100+16'sd100 == 16'sd200) &&
        // (16'sd100-16'sd100 == 16'sd0)` — both predicates compile-time-fold
        // to 1'b1, so synth reduces the whole test to `result_flags[2] <= 1'b1`.
        // Replaced with a real radix-2 butterfly that exercises signed
        // multiplications + adds across multiple FSM states with register
        // dataflow (synth must instantiate DSP/multiplier resources).
        //
        //   Inputs:  A = 8 (real), B = 4 (real), W = 2 + 3j
        //   Step 1:  WB = W*B  (with B_im=0, so only 2 mults)
        //              WB_re = W_re * B_re = 2 * 4 = 8
        //              WB_im = W_im * B_re = 3 * 4 = 12
        //   Step 2:  Butterfly:
        //              A' = A + WB = (8+8, 0+12) = (16, 12)
        //              B' = A - WB = (8-8, 0-12) = (0, -12)
        //   Step 3:  Compare against golden.
        ST_FFT_SETUP: begin
            if (step_cnt == 0) begin
                fft_a_re <= 16'sd8;
                fft_b_re <= 16'sd4;
                fft_w_re <= 16'sd2;
                fft_w_im <= 16'sd3;
                step_cnt <= 1;
            end else if (step_cnt == 1) begin
                fft_wb_re <= fft_w_re * fft_b_re;   // 2*4 = 8
                fft_wb_im <= fft_w_im * fft_b_re;   // 3*4 = 12
                step_cnt  <= 2;
            end else if (step_cnt == 2) begin
                fft_aprime_re <= {fft_a_re[15], fft_a_re} + fft_wb_re[16:0];
                fft_aprime_im <=             17'sd0       + fft_wb_im[16:0];
                fft_bprime_re <= {fft_a_re[15], fft_a_re} - fft_wb_re[16:0];
                fft_bprime_im <=             17'sd0       - fft_wb_im[16:0];
                step_cnt      <= 3;
            end else begin
                if (fft_aprime_re == 17'sd16  && fft_aprime_im == 17'sd12 &&
                    fft_bprime_re == 17'sd0   && fft_bprime_im == -17'sd12) begin
                    result_flags[2] <= 1'b1;
                end else begin
                    result_flags[2] <= 1'b0;
                    result_detail   <= 8'hF2;   // FFT fail marker
                end
                state    <= ST_ARITH;
                step_cnt <= 0;
            end
        end

        // ============================================================
        // Test 3: Saturating Arithmetic
        // ============================================================
        ST_ARITH: begin
            // Test cases for sat_add:
            //   32767 + 1 should saturate to 32767 (not wrap to -32768)
            //   -32768 + (-1) should saturate to -32768
            //   100 + 200 = 300
            if (step_cnt == 0) begin
                if (sat_add(16'sd32767, 16'sd1) != 16'sd32767)
                    arith_pass <= 1'b0;
                step_cnt <= 1;
            end else if (step_cnt == 1) begin
                if (sat_add(-16'sd32768, -16'sd1) != -16'sd32768)
                    arith_pass <= 1'b0;
                step_cnt <= 2;
            end else if (step_cnt == 2) begin
                if (sat_add(16'sd100, 16'sd200) != 16'sd300)
                    arith_pass <= 1'b0;
                step_cnt <= 3;
            end else begin
                result_flags[3] <= arith_pass;
                state <= ST_ADC_CAP;
                step_cnt <= 0;
                adc_cap_cnt <= 0;
            end
        end

        // ============================================================
        // Test 4: ADC activity (min/max range) check (AUDIT-S21 fix)
        // ============================================================
        // Pre-fix this set `result_flags[4] <= 1'b1` once N samples were
        // observed, regardless of value. A stuck-at-0 ADC (broken LVDS link,
        // wrong AD9484 mode per AUDIT-C3, dead sample-and-hold) would still
        // PASS as long as adc_valid_in toggled. Now tracks min/max across the
        // capture window and requires range > ADC_RANGE_THRESHOLD (10 LSB).
        // Catches stuck-at faults; does NOT distinguish AD9484 format
        // mismatches (audit's per-mode mean check requires AD9484 SPI which
        // is impossible on production HW per AUDIT-C13).
        ST_ADC_CAP: begin
            capture_active <= 1'b1;
            if (adc_valid_in) begin
                capture_data  <= adc_data_in;
                capture_valid <= 1'b1;
                // Activity tracking: seed min/max on first sample, then update
                if (adc_cap_cnt == 0) begin
                    adc_min <= adc_data_in;
                    adc_max <= adc_data_in;
                end else begin
                    if ($signed(adc_data_in) < $signed(adc_min)) adc_min <= adc_data_in;
                    if ($signed(adc_data_in) > $signed(adc_max)) adc_max <= adc_data_in;
                end
                adc_cap_cnt <= adc_cap_cnt + 1;
                if (adc_cap_cnt >= ADC_CAP_SAMPLES - 1) begin
                    // PASS if observed range exceeds stuck-at threshold
                    if (($signed(adc_max) - $signed(adc_min)) > ADC_RANGE_THRESHOLD) begin
                        result_flags[4] <= 1'b1;
                    end else begin
                        result_flags[4] <= 1'b0;
                        result_detail   <= 8'hAD;   // stuck-at / no-activity marker
                    end
                    capture_active <= 1'b0;
                    state          <= ST_DONE;
                end
            end
            // Timeout: if no ADC data after 1000 cycles (10 us @ 100 MHz), FAIL
            step_cnt <= step_cnt + 1;
            if (step_cnt >= 10'd1000 && adc_cap_cnt == 0) begin
                result_flags[4] <= 1'b0;
                result_detail   <= 8'hAD;  // ADC timeout marker
                capture_active  <= 1'b0;
                state           <= ST_DONE;
            end
        end

        // ============================================================
        // DONE: Report results
        // ============================================================
        ST_DONE: begin
            busy         <= 1'b0;
            result_valid <= 1'b1;
            state        <= ST_IDLE;
        end

        default: state <= ST_IDLE;
        endcase

        // Pipeline: check BRAM read data vs expected (during ST_BRAM_RD)
        if (bram_rd_valid) begin
            if (bram_rd_data != walking_one(bram_rd_addr_d)) begin
                bram_pass <= 1'b0;
                result_detail <= {4'd0, bram_rd_addr_d[3:0]};
            end
        end
    end
end

endmodule
