`timescale 1ns / 1ps

/**
 * matched_filter_processing_chain.v
 *
 * Pulse compression processing chain for AERIS-10 FMCW radar.
 * Implements: FFT(signal) → FFT(reference) → Conjugate multiply → IFFT
 *
 * FFT is provided by fft_engine_axi_bridge — a port-compatible wrapper
 * around xfft_2048, which selects between the Xilinx LogiCORE FFT v9.1 IP
 * (synth + Vivado XSim, when FFT_USE_XILINX_IP is defined; pipelined
 * streaming, ~N+150 cycles/pass) and the in-house fft_engine batched
 * fallback (iverilog only, ~150K cycles/pass — sim coverage only).
 * Production builds set FFT_USE_XILINX_IP in scripts/{50t,200t}/build_*.tcl.
 * A single bridge instance is reused 3 times per frame; the conjugate
 * multiply between the two forward FFTs is done in
 * frequency_matched_filter.v (4-stage pipeline).
 *
 * History: an `ifdef SIMULATION inline behavioural FFT was removed in
 * RX-NEW-1 (2026-04-23) — wrong-bin peaks masked real checks. The
 * LogiCORE swap landed later under RX-NEW-3 to close the PRI budget by
 * replacing the iterative in-house engine with pipelined streaming.
 *
 * Interface contract (from matched_filter_multi_segment.v line 361):
 *   .clk, .reset_n
 *   .adc_data_i, .adc_data_q, .adc_valid      <- from input buffer
 *   .ref_chirp_real/imag                     <- reference (time-domain)
 *   .range_profile_i, .range_profile_q, .range_profile_valid -> output
 *   .chain_state                                -> 4-bit status
 *
 * Clock domain: clk (100 MHz system clock)
 * Data format:  16-bit signed (Q15 fixed-point)
 * FFT size:     2048 points (parameterized via radar_params.vh)
 *
 * Pipeline states (FSM at lines 98-108):
 *   IDLE -> COLLECT  (capture FFT_SIZE ADC + reference samples)
 *        -> SIG_FFT  (feed signal into bridge, run forward FFT)
 *        -> SIG_CAP  (capture signal FFT output into sig_buf)
 *        -> REF_FFT  (feed reference, run forward FFT)
 *        -> REF_CAP  (capture reference FFT output into ref_buf)
 *        -> MULTIPLY (conjugate multiply, into prod_buf)
 *        -> INV_FFT  (feed product, run inverse FFT)
 *        -> INV_CAP  (capture IFFT output into prod_buf)
 *        -> OUTPUT   (stream FFT_SIZE pulse-compressed samples)
 *        -> DONE -> IDLE
 *
 *   Bit-reversal is handled inside the FFT IP (and the iverilog fallback);
 *   no explicit bit-reverse stage in this FSM.
 */

`include "radar_params.vh"

module matched_filter_processing_chain (
    input wire clk,
    input wire reset_n,

    // Input ADC data (from matched_filter_multi_segment buffer)
    input wire [15:0] adc_data_i,
    input wire [15:0] adc_data_q,
    input wire adc_valid,

    // RX-A1 (closed 2026-04-27): chirp_counter port removed — never read
    // inside the chain. multi_segment passed it through to nothing.

    // Reference chirp (time-domain, latency-aligned by upstream buffer)
    // Upstream chirp_reference_rom selects SHORT/MEDIUM/LONG reference
    // via wave_sel[1:0] — this single pair carries whichever is active.
    input wire [15:0] ref_chirp_real,
    input wire [15:0] ref_chirp_imag,

    // Output: range profile (pulse-compressed)
    output wire signed [15:0] range_profile_i,
    output wire signed [15:0] range_profile_q,
    output wire range_profile_valid,

    // Status
    output wire [3:0] chain_state
);

// ============================================================================
// IMPLEMENTATION — freq-domain MF via fft_engine_axi_bridge → xfft_2048
// ============================================================================
// One fft_engine_axi_bridge (2048-pt) instance, reused 3 times per frame:
//   1. Forward FFT of signal     (inverse=0)
//   2. Forward FFT of reference  (inverse=0)
//   3. Inverse FFT of product    (inverse=1)
// Bridge wraps xfft_2048, which selects LogiCORE FFT v9.1 in synth/XSim
// (FFT_USE_XILINX_IP defined) or the in-house fft_engine fallback in
// iverilog. Conjugate multiply in frequency_matched_filter (4-stage pipeline).
//
// Buffer scheme (BRAM-inferrable):
//   sig_buf[2048]:  ADC input -> signal FFT output
//   ref_buf[2048]:  Reference input -> reference FFT output
//   prod_buf[2048]: Conjugate multiply output -> IFFT output
//
// Memory access is INSIDE always @(posedge clk) blocks (no async reset)
// using local blocking variables. Eliminates NBA race conditions and
// enables Vivado BRAM inference.
//
// BRAM read latency (1 cycle) is handled by "primed" flags:
//   feed_primed  — for FFT feed operations
//   mult_primed  — for conjugate multiply feed
//   out_primed   — for output streaming
// ============================================================================

localparam FFT_SIZE  = `RP_FFT_SIZE;    // 2048
localparam ADDR_BITS = `RP_LOG2_FFT_SIZE; // 11

// State encoding
localparam [3:0] ST_IDLE     = 4'd0,
                 ST_COLLECT  = 4'd1,   // Collect FFT_SIZE ADC + ref samples
                 ST_SIG_FFT  = 4'd2,   // Forward FFT of signal
                 ST_SIG_CAP  = 4'd3,   // Capture signal FFT output
                 ST_REF_FFT  = 4'd4,   // Forward FFT of reference
                 ST_REF_CAP  = 4'd5,   // Capture reference FFT output
                 ST_MULTIPLY = 4'd6,   // Conjugate multiply (pipelined)
                 ST_INV_FFT  = 4'd7,   // Inverse FFT of product
                 ST_INV_CAP  = 4'd8,   // Capture IFFT output
                 ST_OUTPUT   = 4'd9,   // Stream FFT_SIZE results
                 ST_DONE     = 4'd10;

reg [3:0] state;

// ============================================================================
// DATA BUFFERS (block RAM) — declared here, accessed in BRAM port blocks
// ============================================================================
(* ram_style = "block" *) reg signed [15:0] sig_buf_i [0:FFT_SIZE-1];
(* ram_style = "block" *) reg signed [15:0] sig_buf_q [0:FFT_SIZE-1];
(* ram_style = "block" *) reg signed [15:0] ref_buf_i [0:FFT_SIZE-1];
(* ram_style = "block" *) reg signed [15:0] ref_buf_q [0:FFT_SIZE-1];
(* ram_style = "block" *) reg signed [15:0] prod_buf_i [0:FFT_SIZE-1];
(* ram_style = "block" *) reg signed [15:0] prod_buf_q [0:FFT_SIZE-1];

// BRAM read data (registered outputs from port blocks)
reg signed [15:0] sig_rdata_i, sig_rdata_q;
reg signed [15:0] ref_rdata_i, ref_rdata_q;
reg signed [15:0] prod_rdata_i, prod_rdata_q;

// ============================================================================
// COUNTERS
// ============================================================================
reg [ADDR_BITS:0] collect_count;   // 0..FFT_SIZE for sample collection
reg [ADDR_BITS:0] feed_count;      // 0..FFT_SIZE for feeding FFT engine
reg [ADDR_BITS:0] cap_count;       // 0..FFT_SIZE for capturing FFT output
reg [ADDR_BITS:0] mult_count;      // 0..FFT_SIZE for multiply feeding
reg [ADDR_BITS:0] out_count;       // 0..FFT_SIZE for output streaming

// BRAM read latency pipeline flags
reg feed_primed;   // 1 = BRAM rdata valid for feed operations
reg mult_primed;   // 1 = BRAM rdata valid for multiply reads
reg out_primed;    // 1 = BRAM rdata valid for output reads

// ============================================================================
// FFT ENGINE INTERFACE (single instance, reused 3 times)
// ============================================================================
reg fft_start;
reg fft_inverse;
reg signed [15:0] fft_din_re, fft_din_im;
reg fft_din_valid;
wire signed [15:0] fft_dout_re, fft_dout_im;
wire fft_dout_valid;
wire fft_busy;
wire fft_done;

// xfft_2048 (Xilinx LogiCORE FFT v9.1) via fft_engine_axi_bridge — preserves
// the legacy fft_engine port surface so this call site stays a 1-line swap.
// In synth + remote XSim: real Pipelined Streaming IP (~N + 150 cycles/pass,
// closes RX-NEW-3 PRI budget). In iverilog: bridge falls through to the
// in-house fft_engine batched fallback inside xfft_2048.v (~150K cycles/pass,
// for unit coverage only — receiver-integration timing is meaningful only in
// XSim with the real IP).
fft_engine_axi_bridge #(
    .N(FFT_SIZE),
    .LOG2N(ADDR_BITS),
    .DATA_W(16),
    .INTERNAL_W(32),
    .TWIDDLE_W(16),
    .TWIDDLE_FILE("fft_twiddle_2048.mem")
) fft_inst (
    .clk(clk),
    .reset_n(reset_n),
    .start(fft_start),
    .inverse(fft_inverse),
    .din_re(fft_din_re),
    .din_im(fft_din_im),
    .din_valid(fft_din_valid),
    .dout_re(fft_dout_re),
    .dout_im(fft_dout_im),
    .dout_valid(fft_dout_valid),
    .busy(fft_busy),
    .done(fft_done)
);

// ============================================================================
// CONJUGATE MULTIPLY INTERFACE (frequency_matched_filter)
// ============================================================================
reg signed [15:0] mf_sig_re, mf_sig_im;
reg signed [15:0] mf_ref_re, mf_ref_im;
reg mf_valid_in;
wire signed [15:0] mf_out_re, mf_out_im;
wire mf_valid_out;

frequency_matched_filter mf_inst (
    .clk(clk),
    .reset_n(reset_n),
    .fft_real_in(mf_sig_re),
    .fft_imag_in(mf_sig_im),
    .fft_valid_in(mf_valid_in),
    .ref_chirp_real(mf_ref_re),
    .ref_chirp_imag(mf_ref_im),
    .filtered_real(mf_out_re),
    .filtered_imag(mf_out_im),
    .filtered_valid(mf_valid_out),
    .state()
);

// Pipeline flush counter for matched filter (4-stage pipeline)
reg [2:0] mf_flush_count;

// ============================================================================
// OUTPUT REGISTERS
// ============================================================================
reg out_valid_reg;
reg signed [15:0] out_i_reg, out_q_reg;

// ============================================================================
// BRAM PORT: sig_buf — all address/we/wdata computed inline (race-free)
// ============================================================================
// Handles: IDLE/COLLECT writes, SIG_FFT/SIG_CAP capture writes,
//          SIG_FFT feed reads, MULTIPLY signal reads
// No async reset in sensitivity list — enables Vivado BRAM inference.
// ============================================================================
always @(posedge clk) begin : sig_bram_port
    reg                    we;
    reg  [ADDR_BITS-1:0]   addr;
    reg  signed [15:0]     wdata_i, wdata_q;

    // Defaults
    we      = 1'b0;
    addr    = 0;
    wdata_i = 0;
    wdata_q = 0;

    case (state)
    ST_IDLE: begin
        if (adc_valid) begin
            we      = 1'b1;
            addr    = 0;
            wdata_i = $signed(adc_data_i);
            wdata_q = $signed(adc_data_q);
        end
    end
    ST_COLLECT: begin
        if (adc_valid && collect_count < FFT_SIZE) begin
            we      = 1'b1;
            addr    = collect_count[ADDR_BITS-1:0];
            wdata_i = $signed(adc_data_i);
            wdata_q = $signed(adc_data_q);
        end
    end
    ST_SIG_FFT: begin
        if (feed_count < FFT_SIZE && !feed_primed) begin
            // Pre-read cycle: present address, no write
            addr = feed_count[ADDR_BITS-1:0];
        end else if (feed_count <= FFT_SIZE && feed_primed) begin
            // Primed: read address for NEXT sample (or hold last)
            if (feed_count < FFT_SIZE)
                addr = feed_count[ADDR_BITS-1:0];
            else
                addr = 0; // don't care, past last sample
        end
        // Capture FFT output (write) — happens after feeding is done
        if (fft_dout_valid && cap_count < FFT_SIZE) begin
            we      = 1'b1;
            addr    = cap_count[ADDR_BITS-1:0];
            wdata_i = fft_dout_re;
            wdata_q = fft_dout_im;
        end
    end
    ST_SIG_CAP: begin
        if (fft_dout_valid && cap_count < FFT_SIZE) begin
            we      = 1'b1;
            addr    = cap_count[ADDR_BITS-1:0];
            wdata_i = fft_dout_re;
            wdata_q = fft_dout_im;
        end
    end
    ST_MULTIPLY: begin
        // Read signal FFT results for conjugate multiply
        if (mult_count < FFT_SIZE && !mult_primed) begin
            addr = mult_count[ADDR_BITS-1:0];
        end else if (mult_count <= FFT_SIZE && mult_primed) begin
            if (mult_count < FFT_SIZE)
                addr = mult_count[ADDR_BITS-1:0];
            else
                addr = 0;
        end
    end
    default: begin
        // keep defaults
    end
    endcase

    // BRAM write
    if (we) begin
        sig_buf_i[addr] <= wdata_i;
        sig_buf_q[addr] <= wdata_q;
    end
    // BRAM read (1-cycle latency)
    sig_rdata_i <= sig_buf_i[addr];
    sig_rdata_q <= sig_buf_q[addr];
end

// ============================================================================
// BRAM PORT: ref_buf — all address/we/wdata computed inline (race-free)
// ============================================================================
// Handles: IDLE/COLLECT writes, REF_FFT/REF_CAP capture writes,
//          REF_FFT feed reads, MULTIPLY reference reads
// ============================================================================
always @(posedge clk) begin : ref_bram_port
    reg                    we;
    reg  [ADDR_BITS-1:0]   addr;
    reg  signed [15:0]     wdata_i, wdata_q;

    // Defaults
    we      = 1'b0;
    addr    = 0;
    wdata_i = 0;
    wdata_q = 0;

    case (state)
    ST_IDLE: begin
        if (adc_valid) begin
            we      = 1'b1;
            addr    = 0;
            wdata_i = $signed(ref_chirp_real);
            wdata_q = $signed(ref_chirp_imag);
        end
    end
    ST_COLLECT: begin
        if (adc_valid && collect_count < FFT_SIZE) begin
            we      = 1'b1;
            addr    = collect_count[ADDR_BITS-1:0];
            wdata_i = $signed(ref_chirp_real);
            wdata_q = $signed(ref_chirp_imag);
        end
    end
    ST_REF_FFT: begin
        if (feed_count < FFT_SIZE && !feed_primed) begin
            addr = feed_count[ADDR_BITS-1:0];
        end else if (feed_count <= FFT_SIZE && feed_primed) begin
            if (feed_count < FFT_SIZE)
                addr = feed_count[ADDR_BITS-1:0];
            else
                addr = 0;
        end
        // Capture FFT output
        if (fft_dout_valid && cap_count < FFT_SIZE) begin
            we      = 1'b1;
            addr    = cap_count[ADDR_BITS-1:0];
            wdata_i = fft_dout_re;
            wdata_q = fft_dout_im;
        end
    end
    ST_REF_CAP: begin
        if (fft_dout_valid && cap_count < FFT_SIZE) begin
            we      = 1'b1;
            addr    = cap_count[ADDR_BITS-1:0];
            wdata_i = fft_dout_re;
            wdata_q = fft_dout_im;
        end
    end
    ST_MULTIPLY: begin
        // Read reference FFT results for conjugate multiply
        if (mult_count < FFT_SIZE && !mult_primed) begin
            addr = mult_count[ADDR_BITS-1:0];
        end else if (mult_count <= FFT_SIZE && mult_primed) begin
            if (mult_count < FFT_SIZE)
                addr = mult_count[ADDR_BITS-1:0];
            else
                addr = 0;
        end
    end
    default: begin
        // keep defaults
    end
    endcase

    // BRAM write
    if (we) begin
        ref_buf_i[addr] <= wdata_i;
        ref_buf_q[addr] <= wdata_q;
    end
    // BRAM read (1-cycle latency)
    ref_rdata_i <= ref_buf_i[addr];
    ref_rdata_q <= ref_buf_q[addr];
end

// ============================================================================
// BRAM PORT: prod_buf — all address/we/wdata computed inline (race-free)
// ============================================================================
// Handles: MULTIPLY capture writes, INV_FFT/INV_CAP capture writes,
//          INV_FFT feed reads, OUTPUT reads
// ============================================================================
always @(posedge clk) begin : prod_bram_port
    reg                    we;
    reg  [ADDR_BITS-1:0]   addr;
    reg  signed [15:0]     wdata_i, wdata_q;

    // Defaults
    we      = 1'b0;
    addr    = 0;
    wdata_i = 0;
    wdata_q = 0;

    case (state)
    ST_MULTIPLY: begin
        // Capture conjugate multiply output
        if (mf_valid_out && cap_count < FFT_SIZE) begin
            we      = 1'b1;
            addr    = cap_count[ADDR_BITS-1:0];
            wdata_i = mf_out_re;
            wdata_q = mf_out_im;
        end
    end
    ST_INV_FFT: begin
        if (feed_count < FFT_SIZE && !feed_primed) begin
            addr = feed_count[ADDR_BITS-1:0];
        end else if (feed_count <= FFT_SIZE && feed_primed) begin
            if (feed_count < FFT_SIZE)
                addr = feed_count[ADDR_BITS-1:0];
            else
                addr = 0;
        end
        // Capture IFFT output
        if (fft_dout_valid && cap_count < FFT_SIZE) begin
            we      = 1'b1;
            addr    = cap_count[ADDR_BITS-1:0];
            wdata_i = fft_dout_re;
            wdata_q = fft_dout_im;
        end
    end
    ST_INV_CAP: begin
        if (fft_dout_valid && cap_count < FFT_SIZE) begin
            we      = 1'b1;
            addr    = cap_count[ADDR_BITS-1:0];
            wdata_i = fft_dout_re;
            wdata_q = fft_dout_im;
        end
    end
    ST_OUTPUT: begin
        // Read product buffer for output streaming
        if (out_count < FFT_SIZE && !out_primed) begin
            addr = out_count[ADDR_BITS-1:0];
        end else if (out_count <= FFT_SIZE && out_primed) begin
            if (out_count < FFT_SIZE)
                addr = out_count[ADDR_BITS-1:0];
            else
                addr = 0;
        end
    end
    default: begin
        // keep defaults
    end
    endcase

    // BRAM write
    if (we) begin
        prod_buf_i[addr] <= wdata_i;
        prod_buf_q[addr] <= wdata_q;
    end
    // BRAM read (1-cycle latency)
    prod_rdata_i <= prod_buf_i[addr];
    prod_rdata_q <= prod_buf_q[addr];
end

// ============================================================================
// MAIN FSM — no buffer array accesses here (all via BRAM ports above)
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state          <= ST_IDLE;
        collect_count  <= 0;
        feed_count     <= 0;
        cap_count      <= 0;
        mult_count     <= 0;
        out_count      <= 0;
        feed_primed    <= 1'b0;
        mult_primed    <= 1'b0;
        out_primed     <= 1'b0;
        fft_start      <= 1'b0;
        fft_inverse    <= 1'b0;
        fft_din_re     <= 0;
        fft_din_im     <= 0;
        fft_din_valid  <= 1'b0;
        mf_sig_re      <= 0;
        mf_sig_im      <= 0;
        mf_ref_re      <= 0;
        mf_ref_im      <= 0;
        mf_valid_in    <= 1'b0;
        mf_flush_count <= 0;
        out_valid_reg  <= 1'b0;
        out_i_reg      <= 0;
        out_q_reg      <= 0;
    end else begin
        // Defaults
        fft_start     <= 1'b0;
        fft_din_valid <= 1'b0;
        mf_valid_in   <= 1'b0;
        out_valid_reg <= 1'b0;

        case (state)

        // ================================================================
        ST_IDLE: begin
            collect_count <= 0;
            feed_primed   <= 1'b0;
            mult_primed   <= 1'b0;
            out_primed    <= 1'b0;
            if (adc_valid) begin
                // First sample written by sig/ref BRAM ports (they see
                // state==ST_IDLE && adc_valid)
                collect_count <= 1;
                state <= ST_COLLECT;
            end
        end

        // ================================================================
        // COLLECT: Gather 2048 ADC + reference samples
        // Writes happen in sig/ref BRAM ports (they see state==ST_COLLECT)
        // ================================================================
        ST_COLLECT: begin
            if (adc_valid && collect_count < FFT_SIZE) begin
                collect_count <= collect_count + 1;
            end

            if (collect_count == FFT_SIZE) begin
                // All 2048 samples collected — start signal FFT
                state       <= ST_SIG_FFT;
                fft_start   <= 1'b1;
                fft_inverse <= 1'b0;  // Forward FFT
                feed_count  <= 0;
                cap_count   <= 0;
                feed_primed <= 1'b0;
            end
        end

        // ================================================================
        // SIG_FFT: Feed signal buffer to FFT engine (forward)
        // BRAM read has 1-cycle latency: address presented in BRAM port,
        // data available in sig_rdata_i/q next cycle.
        // ================================================================
        ST_SIG_FFT: begin
            // Feed phase: read sig_buf -> fft_din
            if (feed_count < FFT_SIZE) begin
                if (!feed_primed) begin
                    // Pre-read cycle: address presented to BRAM, wait 1 cycle
                    feed_primed <= 1'b1;
                    feed_count  <= feed_count + 1;
                    // fft_din_valid stays 0 (default)
                end else begin
                    // Primed: BRAM rdata is valid for previous address
                    fft_din_re    <= sig_rdata_i;
                    fft_din_im    <= sig_rdata_q;
                    fft_din_valid <= 1'b1;
                    feed_count    <= feed_count + 1;
                end
            end else if (feed_count == FFT_SIZE && feed_primed) begin
                // Last sample: BRAM rdata has data for address 1023
                fft_din_re    <= sig_rdata_i;
                fft_din_im    <= sig_rdata_q;
                fft_din_valid <= 1'b1;
                feed_count    <= feed_count + 1; // -> 1025, stops feeding
            end

            // Capture FFT output (writes happen in BRAM port)
            if (fft_dout_valid && cap_count < FFT_SIZE) begin
                cap_count <= cap_count + 1;
            end

            if (fft_done) begin
                state <= ST_SIG_CAP;
            end
        end

        // ================================================================
        // SIG_CAP: Ensure all signal FFT outputs captured
        // ================================================================
        ST_SIG_CAP: begin
            if (fft_dout_valid && cap_count < FFT_SIZE) begin
                cap_count <= cap_count + 1;
            end

            // Start reference FFT
            state       <= ST_REF_FFT;
            fft_start   <= 1'b1;
            fft_inverse <= 1'b0;  // Forward FFT
            feed_count  <= 0;
            cap_count   <= 0;
            feed_primed <= 1'b0;
        end

        // ================================================================
        // REF_FFT: Feed reference buffer to FFT engine (forward)
        // ================================================================
        ST_REF_FFT: begin
            // Feed phase: read ref_buf -> fft_din
            if (feed_count < FFT_SIZE) begin
                if (!feed_primed) begin
                    feed_primed <= 1'b1;
                    feed_count  <= feed_count + 1;
                end else begin
                    fft_din_re    <= ref_rdata_i;
                    fft_din_im    <= ref_rdata_q;
                    fft_din_valid <= 1'b1;
                    feed_count    <= feed_count + 1;
                end
            end else if (feed_count == FFT_SIZE && feed_primed) begin
                fft_din_re    <= ref_rdata_i;
                fft_din_im    <= ref_rdata_q;
                fft_din_valid <= 1'b1;
                feed_count    <= feed_count + 1;
            end

            if (fft_dout_valid && cap_count < FFT_SIZE) begin
                cap_count <= cap_count + 1;
            end

            if (fft_done) begin
                state <= ST_REF_CAP;
            end
        end

        // ================================================================
        // REF_CAP: Ensure all ref FFT outputs captured
        // ================================================================
        ST_REF_CAP: begin
            if (fft_dout_valid && cap_count < FFT_SIZE) begin
                cap_count <= cap_count + 1;
            end

            state          <= ST_MULTIPLY;
            mult_count     <= 0;
            cap_count      <= 0;
            mf_flush_count <= 0;
            mult_primed    <= 1'b0;
        end

        // ================================================================
        // MULTIPLY: Stream sig FFT and ref FFT through freq_matched_filter
        // Both sig_buf and ref_buf are read simultaneously (separate BRAM
        // ports). Pipeline latency = 4 clocks. Feed 2048 pairs, then flush.
        // ================================================================
        ST_MULTIPLY: begin
            if (mult_count < FFT_SIZE) begin
                if (!mult_primed) begin
                    // Pre-read cycle
                    mult_primed <= 1'b1;
                    mult_count  <= mult_count + 1;
                end else begin
                    mf_sig_re   <= sig_rdata_i;
                    mf_sig_im   <= sig_rdata_q;
                    mf_ref_re   <= ref_rdata_i;
                    mf_ref_im   <= ref_rdata_q;
                    mf_valid_in <= 1'b1;
                    mult_count  <= mult_count + 1;
                end
            end else if (mult_count == FFT_SIZE && mult_primed) begin
                // Last sample
                mf_sig_re   <= sig_rdata_i;
                mf_sig_im   <= sig_rdata_q;
                mf_ref_re   <= ref_rdata_i;
                mf_ref_im   <= ref_rdata_q;
                mf_valid_in <= 1'b1;
                mult_count  <= mult_count + 1;
            end else begin
                // Pipeline flush — wait for remaining outputs
                mf_flush_count <= mf_flush_count + 1;
            end

            // Capture multiply outputs (writes happen in BRAM port)
            if (mf_valid_out && cap_count < FFT_SIZE) begin
                cap_count <= cap_count + 1;
            end

            // Done when all outputs captured
            if (cap_count == FFT_SIZE) begin
                state       <= ST_INV_FFT;
                fft_start   <= 1'b1;
                fft_inverse <= 1'b1;  // Inverse FFT
                feed_count  <= 0;
                cap_count   <= 0;
                feed_primed <= 1'b0;
            end
        end

        // ================================================================
        // INV_FFT: Feed product buffer to FFT engine (inverse)
        // ================================================================
        ST_INV_FFT: begin
            if (feed_count < FFT_SIZE) begin
                if (!feed_primed) begin
                    feed_primed <= 1'b1;
                    feed_count  <= feed_count + 1;
                end else begin
                    fft_din_re    <= prod_rdata_i;
                    fft_din_im    <= prod_rdata_q;
                    fft_din_valid <= 1'b1;
                    feed_count    <= feed_count + 1;
                end
            end else if (feed_count == FFT_SIZE && feed_primed) begin
                fft_din_re    <= prod_rdata_i;
                fft_din_im    <= prod_rdata_q;
                fft_din_valid <= 1'b1;
                feed_count    <= feed_count + 1;
            end

            if (fft_dout_valid && cap_count < FFT_SIZE) begin
                cap_count <= cap_count + 1;
            end

            if (fft_done) begin
                state <= ST_INV_CAP;
            end
        end

        // ================================================================
        // INV_CAP: Ensure all IFFT outputs captured
        // ================================================================
        ST_INV_CAP: begin
            if (fft_dout_valid && cap_count < FFT_SIZE) begin
                cap_count <= cap_count + 1;
            end

            state      <= ST_OUTPUT;
            out_count  <= 0;
            out_primed <= 1'b0;
        end

        // ================================================================
        // OUTPUT: Stream 2048 range profile samples
        // BRAM read latency: present address, data valid next cycle.
        // ================================================================
        ST_OUTPUT: begin
            if (out_count < FFT_SIZE) begin
                if (!out_primed) begin
                    // Pre-read cycle
                    out_primed <= 1'b1;
                    out_count  <= out_count + 1;
                end else begin
                    out_i_reg     <= prod_rdata_i;
                    out_q_reg     <= prod_rdata_q;
                    out_valid_reg <= 1'b1;
                    out_count     <= out_count + 1;
                end
            end else if (out_count == FFT_SIZE && out_primed) begin
                // Last sample
                out_i_reg     <= prod_rdata_i;
                out_q_reg     <= prod_rdata_q;
                out_valid_reg <= 1'b1;
                out_count     <= out_count + 1;
            end else begin
                state <= ST_DONE;
            end
        end

        // ================================================================
        // DONE: Return to idle
        // ================================================================
        ST_DONE: begin
            state <= ST_IDLE;
        end

        default: state <= ST_IDLE;

        endcase
    end
end

// ============================================================================
// OUTPUT ASSIGNMENTS
// ============================================================================
assign range_profile_i     = out_i_reg;
assign range_profile_q     = out_q_reg;
assign range_profile_valid = out_valid_reg;
assign chain_state         = state;

// ============================================================================
// BUFFER INIT (for simulation — Vivado ignores initial blocks on arrays)
// ============================================================================
integer init_idx;
initial begin
    for (init_idx = 0; init_idx < FFT_SIZE; init_idx = init_idx + 1) begin
        sig_buf_i[init_idx]  = 0;
        sig_buf_q[init_idx]  = 0;
        ref_buf_i[init_idx]  = 0;
        ref_buf_q[init_idx]  = 0;
        prod_buf_i[init_idx] = 0;
        prod_buf_q[init_idx] = 0;
    end
end


endmodule
