`timescale 1ns / 1ps
// matched_filter_multi_segment.v

`include "radar_params.vh"

module matched_filter_multi_segment (
    input wire clk,           // 100MHz
    input wire reset_n,
    
    // Input from DDC (100 MSPS)
    input wire signed [17:0] ddc_i,
    input wire signed [17:0] ddc_q,
    input wire ddc_valid,
    
    // Chirp control (from chirp_scheduler — chirp-v2 wave_sel rail)
    input wire [1:0] wave_sel,        // 00=SHORT, 01=MEDIUM, 10=LONG
    input wire [5:0] chirp_counter,

    // Chirp boundary — 1-cycle pulse from chirp_scheduler. Replaces the old
    // mc_new_chirp toggle + XOR edge detector; mc_new_elevation/azimuth are
    // gone (they were dead — no consumer in this module).
    input wire chirp_pulse,

    // Reference chirp (chirp_reference_rom selects waveform via wave_sel)
    input wire [15:0] ref_chirp_real,
    input wire [15:0] ref_chirp_imag,
    
    // Memory system interface
    output reg [1:0] segment_request,
    output wire [10:0] sample_addr_out,  // Tell memory which sample we need (11-bit for 2048)
    output reg mem_request,
    input wire mem_ready,
    
    // Output: Pulse compressed
    output wire signed [15:0] pc_i_w,
    output wire signed [15:0] pc_q_w,
    output wire pc_valid_w,
    
    // Status
    output reg [3:0] status
);

// ========== FIXED PARAMETERS ==========
parameter BUFFER_SIZE = `RP_FFT_SIZE;              // 2048 (FFT pass size)
parameter LONG_CHIRP_SAMPLES   = 3000;             // 30 us @ 100 MHz
parameter MEDIUM_CHIRP_SAMPLES = 500;              // 5 us @ 100 MHz (chirp-v2)
parameter SHORT_CHIRP_SAMPLES  = 100;              // 1 us @ 100 MHz (chirp-v2; was 50)
parameter OVERLAP_SAMPLES = `RP_OVERLAP_SAMPLES;   // 128
parameter SEGMENT_ADVANCE = `RP_SEGMENT_ADVANCE;   // 2048 - 128 = 1920 samples
parameter DEBUG = 1;                               // Debug output control

// Segment counts. LONG spans 2 segments; SHORT and MEDIUM both fit in a
// single 2048-sample window with zero-pad.
parameter LONG_SEGMENTS  = `RP_LONG_SEGMENTS_3KM;  // 2 segments (30 us)
parameter SHORT_SEGMENTS = 1;                      // SHORT or MEDIUM, single segment

// PR-J.2: pre-collect-then-slide ingestion. The full chirp is captured into
// INPUT_BUF_DEPTH-deep BRAM during ST_COLLECT_DATA, then segments are read
// out non-destructively as 2048-sample windows starting at
// current_segment * SEGMENT_ADVANCE. This replaces the original overlap-save
// mechanism, which assumed the input ddc stream stayed live across segment
// processing — a contract that breaks because chain processing (~70 us at
// production xfft_2048 timing) outlasts the LONG chirp duration (30 us),
// dropping segment-N>0 input samples.
//
// Required depth: (LONG_SEGMENTS - 1) * SEGMENT_ADVANCE + BUFFER_SIZE
//                 50T  -> 1*1920 + 2048 = 3968  (round up to 4096)
//                 200T -> 7*1920 + 2048 = 15488 (would need 16384)
// 50T-only here; 200T variant must bump INPUT_BUF_DEPTH when LONG_SEGMENTS
// changes via SUPPORT_LONG_RANGE.
parameter INPUT_BUF_DEPTH  = 4096;
parameter INPUT_BUF_ADDR_W = 12;

// Total samples needed in buffer to cover LONG_SEGMENTS segments with
// overlap-save indexing. Last segment ends at (LONG_SEGMENTS-1)*ADVANCE +
// BUFFER_SIZE = 3968 (50T). ST_ZERO_PAD pads up to this point for LONG
// chirps so segment 1's read window is well-defined even when the chirp
// only delivers 3000 real samples.
parameter LONG_FILL_END = (LONG_SEGMENTS - 1) * SEGMENT_ADVANCE + BUFFER_SIZE;

// Convenience nets so the FSM body reads cleanly.
wire is_long   = (wave_sel == `RP_WAVE_LONG);
wire is_medium = (wave_sel == `RP_WAVE_MEDIUM);

// ========== FIXED INTERNAL SIGNALS ==========
reg signed [31:0] pc_i, pc_q;
reg pc_valid;

// Pre-collect input buffer (PR-J.2). Sized to hold the full LONG chirp plus
// segment-1's read tail; SHORT/MEDIUM only use the first 2048 entries.
(* ram_style = "block" *) reg signed [15:0] input_buffer_i [0:INPUT_BUF_DEPTH-1];
(* ram_style = "block" *) reg signed [15:0] input_buffer_q [0:INPUT_BUF_DEPTH-1];
reg [INPUT_BUF_ADDR_W-1:0] buffer_write_ptr;
reg [INPUT_BUF_ADDR_W-1:0] buffer_read_ptr;       // global address into input_buffer
reg buffer_has_data;
reg buffer_processing;
reg [15:0] chirp_samples_collected;

// Per-segment offset and feed counter — segment N reads buffer[segment_offset
// .. segment_offset + FFT_SIZE - 1], where segment_offset advances by
// SEGMENT_ADVANCE each ST_NEXT_SEGMENT.
reg [INPUT_BUF_ADDR_W-1:0] segment_offset;
reg [11:0]                 samples_fed;            // 0..BUFFER_SIZE within a segment

// BRAM write port signals
reg                        buf_we;
reg [INPUT_BUF_ADDR_W-1:0] buf_waddr;
reg signed [15:0]          buf_wdata_i, buf_wdata_q;

// BRAM read port signals
reg [INPUT_BUF_ADDR_W-1:0] buf_raddr;
reg signed [15:0]          buf_rdata_i, buf_rdata_q;

// State machine
reg [3:0] state;
localparam ST_IDLE = 0;
localparam ST_COLLECT_DATA = 1;
localparam ST_ZERO_PAD = 2;
localparam ST_WAIT_REF = 3;
localparam ST_PROCESSING = 4;
localparam ST_WAIT_FFT = 5;
localparam ST_OUTPUT = 6;
localparam ST_NEXT_SEGMENT = 7;
// State 8 (ST_OVERLAP_COPY) retired in PR-J.2 — pre-collected buffer is
// stable across segments, so no overlap copy is needed.

// Segment tracking
reg [2:0] current_segment;        // 0-3
reg [2:0] total_segments;
reg segment_done;
reg chirp_complete;
reg saw_chain_output;             // Flag: chain started producing output

// Processing chain signals
wire [15:0] fft_pc_i, fft_pc_q;
wire fft_pc_valid;
wire [3:0] fft_chain_state;

// Buffer for FFT input
reg [15:0] fft_input_i, fft_input_q;
reg fft_input_valid;
reg fft_start;

// ========== SAMPLE ADDRESS OUTPUT ==========
// chirp_reference_rom expects a per-segment 0..2047 address (the reference
// window is segment-scoped, FFT_SIZE-deep). samples_fed counts FFT-input
// position within the current segment, which is exactly that.
assign sample_addr_out = samples_fed[10:0];

// ========== BUFFER INITIALIZATION ==========
integer buf_init;
initial begin
    for (buf_init = 0; buf_init < INPUT_BUF_DEPTH; buf_init = buf_init + 1) begin
        input_buffer_i[buf_init] = 16'd0;
        input_buffer_q[buf_init] = 16'd0;
    end
end

// ========== BRAM WRITE PORT (synchronous, no async reset) ==========
always @(posedge clk) begin
    if (buf_we) begin
        input_buffer_i[buf_waddr] <= buf_wdata_i;
        input_buffer_q[buf_waddr] <= buf_wdata_q;
    end
end

// ========== BRAM READ PORT (synchronous, no async reset) ==========
always @(posedge clk) begin
    buf_rdata_i <= input_buffer_i[buf_raddr];
    buf_rdata_q <= input_buffer_q[buf_raddr];
end

// ========== FIXED STATE MACHINE WITH OVERLAP-SAVE ==========
integer i;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state <= ST_IDLE;
        buffer_write_ptr <= 0;
        buffer_read_ptr <= 0;
        buffer_has_data <= 0;
        buffer_processing <= 0;
        current_segment <= 0;
        segment_done <= 0;
        segment_request <= 0;
        segment_offset <= 0;
        samples_fed <= 0;
        mem_request <= 0;
        pc_valid <= 0;
        status <= 0;
        chirp_samples_collected <= 0;
        chirp_complete <= 0;
        saw_chain_output <= 0;
        fft_input_valid <= 0;
        fft_start <= 0;
        buf_we <= 0;
        buf_waddr <= 0;
        buf_wdata_i <= 0;
        buf_wdata_q <= 0;
        buf_raddr <= 0;
    end else begin
        pc_valid <= 0;
        mem_request <= 0;
        fft_input_valid <= 0;
        buf_we <= 0;  // Default: no write
        
        case (state)
            ST_IDLE: begin
                // Reset for new chirp
                buffer_write_ptr <= 0;
                buffer_read_ptr <= 0;
                buffer_has_data <= 0;
                buffer_processing <= 0;
                current_segment <= 0;
                segment_offset <= 0;
                samples_fed <= 0;
                segment_done <= 0;
                chirp_samples_collected <= 0;
                chirp_complete <= 0;
                saw_chain_output <= 0;

                // Wait for chirp start (1-cycle pulse from chirp_scheduler)
                if (chirp_pulse) begin
                    state <= ST_COLLECT_DATA;
                    total_segments <= is_long ? LONG_SEGMENTS[2:0] : SHORT_SEGMENTS[2:0];

                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Starting %s chirp, segments: %d",
                             is_long ? "LONG" : (is_medium ? "MEDIUM" : "SHORT"),
                             is_long ? LONG_SEGMENTS : SHORT_SEGMENTS);
                    $display("[MULTI_SEG_FIXED] Overlap: %d samples, Advance: %d samples",
                             OVERLAP_SAMPLES, SEGMENT_ADVANCE);
                    `endif
                end
            end
            
            ST_COLLECT_DATA: begin
                // PR-J.2: pre-collect entire chirp into the input buffer.
                // No mid-stream segment processing; segments slide over the
                // stable buffer once collection + zero-pad complete.
                if (ddc_valid && buffer_write_ptr < INPUT_BUF_DEPTH) begin
                    buf_we <= 1;
                    buf_waddr <= buffer_write_ptr;
                    // [RX-A FIX] ddc_i = {{2{gc_i[15]}}, gc_i} — top 2 bits
                    // are sign-extension; full 16-bit input is safe because
                    // fft_engine has INTERNAL_W=32 with saturating 16-bit
                    // output (no bit-growth overflow risk).
                    buf_wdata_i <= ddc_i[15:0];
                    buf_wdata_q <= ddc_q[15:0];

                    buffer_write_ptr <= buffer_write_ptr + 1;
                    chirp_samples_collected <= chirp_samples_collected + 1;

                    if (chirp_samples_collected < 10) begin
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] Store[%0d]: I=%h Q=%h",
                                 buffer_write_ptr, ddc_i[15:0], ddc_q[15:0]);
                        `endif
                    end
                end

                // Chirp-complete check (each-clock, not gated on ddc_valid,
                // so the transition fires the cycle after the last write).
                if (!chirp_complete) begin
                    if (is_long &&
                        chirp_samples_collected >= LONG_CHIRP_SAMPLES) begin
                        chirp_complete <= 1;
                        state <= ST_ZERO_PAD;
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] LONG chirp: collected %0d samples, padding to %0d",
                                 chirp_samples_collected, LONG_FILL_END);
                        `endif
                    end else if (is_medium &&
                                 chirp_samples_collected >= MEDIUM_CHIRP_SAMPLES) begin
                        chirp_complete <= 1;
                        state <= ST_ZERO_PAD;
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] MEDIUM chirp: collected %0d samples, padding to %0d",
                                 chirp_samples_collected, BUFFER_SIZE);
                        `endif
                    end else if (!is_long && !is_medium &&
                                 chirp_samples_collected >= SHORT_CHIRP_SAMPLES) begin
                        chirp_complete <= 1;
                        state <= ST_ZERO_PAD;
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] SHORT chirp: collected %0d samples, padding to %0d",
                                 chirp_samples_collected, BUFFER_SIZE);
                        `endif
                    end
                end
            end
            
            ST_ZERO_PAD: begin
                // Zero-pad remaining buffer via BRAM write port. LONG pads
                // to LONG_FILL_END (covers all segments' read windows).
                // SHORT/MEDIUM pads to BUFFER_SIZE (single-segment).
                buf_we <= 1;
                buf_waddr <= buffer_write_ptr;
                buf_wdata_i <= 16'd0;
                buf_wdata_q <= 16'd0;
                buffer_write_ptr <= buffer_write_ptr + 1;

                if ((is_long  && buffer_write_ptr >= LONG_FILL_END - 1) ||
                    (!is_long && buffer_write_ptr >= BUFFER_SIZE   - 1)) begin
                    buffer_has_data <= 1;
                    state <= ST_WAIT_REF;
                    segment_request <= 2'd0;
                    mem_request <= 1;
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Zero-pad complete, requesting segment 0 reference");
                    `endif
                end
            end
            
            ST_WAIT_REF: begin
                // Pre-present this segment's first read address so buf_rdata
                // holds buffer[segment_offset] on the first ST_PROCESSING clk.
                buf_raddr <= segment_offset;
                if (mem_ready) begin
                    buffer_processing <= 1;
                    buffer_read_ptr   <= segment_offset;
                    samples_fed       <= 0;
                    fft_start         <= 1;
                    state             <= ST_PROCESSING;

                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Reference ready, starting processing segment %0d (offset=%0d)",
                             current_segment, segment_offset);
                    `endif
                end
            end

            ST_PROCESSING: begin
                // Feed BUFFER_SIZE (FFT_SIZE) samples from buffer[segment_offset
                // .. segment_offset+FFT_SIZE-1] into the chain. samples_fed
                // counts within the segment; buffer_read_ptr is the global
                // BRAM address (= segment_offset + samples_fed).
                if (buffer_processing && samples_fed < BUFFER_SIZE) begin
                    fft_input_i      <= buf_rdata_i;
                    fft_input_q      <= buf_rdata_q;
                    fft_input_valid  <= 1;

                    // Request corresponding reference sample (per-segment
                    // address; sample_addr_out is samples_fed[10:0]).
                    mem_request <= 1'b1;

                    if (samples_fed % 100 == 0) begin
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] Processing seg=%0d [%0d/%0d]: ADC I=%h Q=%h",
                                 current_segment, samples_fed, BUFFER_SIZE,
                                 buf_rdata_i, buf_rdata_q);
                        `endif
                    end

                    // Present NEXT read address for next cycle.
                    buf_raddr        <= buffer_read_ptr + 1;
                    buffer_read_ptr  <= buffer_read_ptr + 1;
                    samples_fed      <= samples_fed + 1;

                end else if (samples_fed >= BUFFER_SIZE) begin
                    // Done feeding this segment.
                    fft_input_valid   <= 0;
                    mem_request       <= 0;
                    buffer_processing <= 0;
                    saw_chain_output  <= 0;
                    state             <= ST_WAIT_FFT;

                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Finished feeding %0d samples for seg=%0d, waiting on FFT chain",
                             BUFFER_SIZE, current_segment);
                    `endif
                end
            end
            
            ST_WAIT_FFT: begin
                // Wait for the processing chain to complete ALL outputs.
                // The chain streams FFT_SIZE samples (fft_pc_valid=1 for FFT_SIZE clocks),
                // then transitions to ST_DONE (9) -> ST_IDLE (0).
                // We track when output starts (saw_chain_output) and only
                // proceed once the chain returns to idle after outputting.
                if (fft_pc_valid) begin
                    saw_chain_output <= 1;
                end
                
                if (saw_chain_output && fft_chain_state == 4'd0) begin
                    // Chain has returned to idle after completing all output
                    saw_chain_output <= 0;
                    state <= ST_OUTPUT;
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Chain complete for segment %d, entering ST_OUTPUT",
                             current_segment);
                    `endif
                end
            end
            
            ST_OUTPUT: begin
                // Store FFT output
                pc_i <= fft_pc_i;
                pc_q <= fft_pc_q;
                pc_valid <= 1;
                segment_done <= 1;
                
                `ifdef SIMULATION
                $display("[MULTI_SEG_FIXED] Output segment %d: I=%h Q=%h",
                         current_segment, fft_pc_i, fft_pc_q);
                `endif
                
                // Check if we need more segments
                if (current_segment < total_segments - 1 || !chirp_complete) begin
                    state <= ST_NEXT_SEGMENT;
                end else begin
                    // All segments complete
                    state <= ST_IDLE;
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] All %d segments complete",
                             total_segments);
                    `endif
                end
            end
            
            ST_NEXT_SEGMENT: begin
                // Bump segment counter and slide the read window forward by
                // SEGMENT_ADVANCE. Buffer is stable across segments, so we
                // go straight to ST_WAIT_REF for the next reference window.
                current_segment <= current_segment + 1;
                segment_offset  <= segment_offset + SEGMENT_ADVANCE;
                segment_done    <= 0;
                segment_request <= current_segment + 1;
                mem_request     <= 1;
                state           <= ST_WAIT_REF;

                `ifdef SIMULATION
                $display("[MULTI_SEG_FIXED] Next segment: %0d (offset will be %0d)",
                         current_segment + 1, segment_offset + SEGMENT_ADVANCE);
                `endif
            end

            default: begin
                state <= ST_IDLE;
            end
        endcase
        
        // Update status — bit 0 echoes is_long for legacy probes; full
        // wave_sel is consumed at the module boundary.
        status <= {state[2:0], is_long};
    end
end

// ========== PROCESSING CHAIN INSTANTIATION ==========
matched_filter_processing_chain m_f_p_c(
    .clk(clk),
    .reset_n(reset_n),
    
    // Input ADC Data
    .adc_data_i(fft_input_i),
    .adc_data_q(fft_input_q),
    .adc_valid(fft_input_valid),// && buffer_processing),

    // RX-A1: chain.chirp_counter removed (was unused inside the chain).
    // multi_segment.chirp_counter input is now formally unused but kept
    // on the port list for potential future per-chirp sequencing.

    // Reference Chirp Memory Interface (single pair — upstream selects long/short)
    .ref_chirp_real(ref_chirp_real),
    .ref_chirp_imag(ref_chirp_imag),
    
    // Output
    .range_profile_i(fft_pc_i),
    .range_profile_q(fft_pc_q),
    .range_profile_valid(fft_pc_valid),
    
    // Status
    .chain_state(fft_chain_state)
);

// ========== DEBUG MONITOR ==========
`ifdef SIMULATION
reg [31:0] dbg_cycles;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        dbg_cycles <= 0;
    end else begin
        dbg_cycles <= dbg_cycles + 1;
        
        // Monitor state transitions
        if (dbg_cycles % 1000 == 0 && state != ST_IDLE) begin
            $display("[MULTI_SEG_MONITOR @%0d] state=%0d, segment=%0d/%0d, samples=%0d",
                     dbg_cycles, state, current_segment, total_segments,
                     chirp_samples_collected);
        end
    end
end
`endif

// ========== OUTPUT CONNECTIONS ==========
assign pc_i_w = fft_pc_i;
assign pc_q_w = fft_pc_q;
assign pc_valid_w = fft_pc_valid;

endmodule