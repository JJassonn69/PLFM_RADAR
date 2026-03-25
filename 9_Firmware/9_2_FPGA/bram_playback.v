`timescale 1ns / 1ps
//
// bram_playback.v — BRAM Playback Engine for AERIS-10 FPGA
//
// Pre-loads fullchain_range_input.hex (32768 x 32-bit packed {Q[31:16], I[15:0]})
// into BRAM at synthesis time via $readmemh, then streams it through the
// pipeline with correct chirp-by-chirp timing.
//
// Data structure: 32 chirps × 1024 range bins = 32768 words
//
// Timing protocol (matches tb_fullchain_realdata.v):
//   1. Wait for playback_start pulse
//   2. Assert new_chirp_frame for 2 cycles before first chirp
//   3. For each chirp:
//      a. Stream 1024 samples with data_valid=1 (one per clock)
//      b. Deassert data_valid
//      c. Wait for decimator to finish (INTER_CHIRP_GAP cycles)
//   4. Assert playback_done when complete
//
// The BRAM uses 32768 × 32-bit = 128 KiB ≈ 57 BRAM18 blocks.
// XC7A200T has ~730 BRAM18 available, only ~17 currently used.
//

module bram_playback #(
    parameter NUM_CHIRPS       = 32,
    parameter SAMPLES_PER_CHIRP = 1024,
    parameter INTER_CHIRP_GAP  = 200,    // Cycles between chirps for decimator to finish
    parameter FRAME_START_GAP  = 4       // Cycles after new_chirp_frame before first sample
) (
    input  wire        clk,
    input  wire        reset_n,

    // Control
    input  wire        playback_start,    // Pulse to begin playback

    // Output data stream
    output reg  [31:0] data_out,          // Packed {Q[31:16], I[15:0]}
    output reg         data_valid,
    output reg         new_chirp_frame,   // Pulse before first chirp

    // Status
    output reg         playback_done,     // Asserted when all chirps streamed
    output reg         playback_active,   // High during playback
    output reg  [5:0]  chirp_count        // Current chirp index (0..31)
);

// =========================================================================
// BRAM: 32768 × 32-bit, loaded at synthesis from hex file
// =========================================================================
localparam TOTAL_SAMPLES = NUM_CHIRPS * SAMPLES_PER_CHIRP; // 32768
localparam ADDR_WIDTH    = 15;                              // log2(32768)

(* ram_style = "block" *) reg [31:0] bram [0:TOTAL_SAMPLES-1];

initial begin
    `ifdef SIMULATION
        $readmemh("tb/cosim/real_data/hex/fullchain_range_input.hex", bram);
    `else
        $readmemh("fullchain_range_input.hex", bram);
    `endif
end

// Registered read output for timing closure
reg [31:0] bram_rdata;
reg [ADDR_WIDTH-1:0] bram_addr;

always @(posedge clk) begin
    bram_rdata <= bram[bram_addr];
end

// =========================================================================
// Playback State Machine
// =========================================================================
localparam [2:0] ST_IDLE        = 3'd0,
                 ST_FRAME_PULSE = 3'd1,  // Assert new_chirp_frame
                 ST_FRAME_GAP   = 3'd2,  // Gap after frame pulse
                 ST_STREAM      = 3'd3,  // Streaming samples
                 ST_CHIRP_GAP   = 3'd4,  // Inter-chirp gap
                 ST_CHIRP_PRIME = 3'd6,  // 1-cycle BRAM priming after chirp gap
                 ST_DONE        = 3'd5;

reg [2:0]  state;
reg [9:0]  sample_idx;     // 0..1023 within current chirp
reg [7:0]  gap_counter;    // Gap cycle counter
reg [2:0]  frame_gap_cnt;  // Frame start gap counter

// BRAM read pipeline model:
//   Posedge N  : bram_addr = A  (set by FSM via non-blocking assign)
//   Posedge N+1: bram_rdata = BRAM[A]  (registered read in separate always block)
//
// For STREAM to emit sample N on cycle T:
//   - bram_addr must have been N on posedge T-1
//   - Then bram_rdata = BRAM[N] at posedge T
//   - STREAM: data_out <= bram_rdata captures BRAM[N]
//   - STREAM: bram_addr <= N+1 (for next cycle)
//
// This means we only need bram_addr set to 0 one cycle before STREAM starts.
// No pre-fetch pipeline complexity needed.
reg [ADDR_WIDTH-1:0] read_ptr;  // Next address to issue to BRAM

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state          <= ST_IDLE;
        bram_addr      <= {ADDR_WIDTH{1'b0}};
        read_ptr       <= {ADDR_WIDTH{1'b0}};
        data_out       <= 32'd0;
        data_valid     <= 1'b0;
        new_chirp_frame <= 1'b0;
        playback_done  <= 1'b0;
        playback_active <= 1'b0;
        chirp_count    <= 6'd0;
        sample_idx     <= 10'd0;
        gap_counter    <= 8'd0;
        frame_gap_cnt  <= 3'd0;
    end else begin
        // Defaults
        new_chirp_frame <= 1'b0;
        data_valid      <= 1'b0;

        case (state)
            // =============================================================
            ST_IDLE: begin
                playback_done   <= 1'b0;
                playback_active <= 1'b0;
                chirp_count     <= 6'd0;
                if (playback_start) begin
                    state           <= ST_FRAME_PULSE;
                    playback_active <= 1'b1;
                    read_ptr        <= {ADDR_WIDTH{1'b0}};
                end
            end

            // =============================================================
            // Assert new_chirp_frame for 2 cycles (matches testbench behavior)
            ST_FRAME_PULSE: begin
                new_chirp_frame <= 1'b1;
                frame_gap_cnt   <= frame_gap_cnt + 3'd1;
                if (frame_gap_cnt == 3'd1) begin
                    // After 2 cycles of new_chirp_frame, move to FRAME_GAP.
                    // Set bram_addr = 0 so BRAM reads address 0 on the next
                    // posedge, making bram_rdata = BRAM[0] available when
                    // STREAM starts.
                    new_chirp_frame <= 1'b0;
                    frame_gap_cnt   <= 3'd0;
                    state           <= ST_FRAME_GAP;
                    bram_addr       <= read_ptr;     // read_ptr = 0
                end
            end

            // =============================================================
            // 1-cycle wait: BRAM is reading addr 0 (set in FRAME_PULSE).
            // On the NEXT posedge (STREAM cycle 0), bram_rdata = BRAM[0].
            ST_FRAME_GAP: begin
                // Transition to STREAM. Set bram_addr = 1 so that on
                // STREAM cycle 1, bram_rdata = BRAM[1].
                read_ptr   <= read_ptr + 1;  // 0 -> 1
                bram_addr  <= read_ptr + 1;  // addr = 1
                sample_idx <= 10'd0;
                state      <= ST_STREAM;
            end

            // =============================================================
            // Stream 1024 samples per chirp, one per clock.
            //
            // Invariant at each STREAM posedge:
            //   bram_rdata = BRAM[current_sample]
            //   bram_addr was set to current_sample+1 on previous posedge
            //
            // We emit bram_rdata and advance bram_addr for the next sample.
            ST_STREAM: begin
                data_out   <= bram_rdata;
                data_valid <= 1'b1;

                sample_idx <= sample_idx + 10'd1;

                if (sample_idx == SAMPLES_PER_CHIRP - 1) begin
                    // Last sample of this chirp.
                    // Do NOT increment read_ptr here — it already points
                    // to the first sample of the next chirp.
                    chirp_count <= chirp_count + 6'd1;

                    if (chirp_count == NUM_CHIRPS - 1) begin
                        // All chirps done
                        state <= ST_DONE;
                    end else begin
                        // Inter-chirp gap for decimator to finish
                        state       <= ST_CHIRP_GAP;
                        gap_counter <= 8'd0;
                    end
                end else begin
                    // Advance read pointer and set bram_addr for next
                    // sample's registered read
                    read_ptr  <= read_ptr + 1;
                    bram_addr <= read_ptr + 1;
                end
            end

            // =============================================================
            // Gap between chirps — decimator processes last samples.
            // read_ptr already points to the first sample of the next chirp.
            ST_CHIRP_GAP: begin
                gap_counter <= gap_counter + 8'd1;
                if (gap_counter >= INTER_CHIRP_GAP - 1) begin
                    // Set bram_addr = first sample of next chirp.
                    // After 1 cycle in CHIRP_PRIME, bram_rdata will be valid.
                    bram_addr <= read_ptr;
                    state     <= ST_CHIRP_PRIME;
                end
            end

            // =============================================================
            // 1-cycle BRAM priming: bram_addr was set to read_ptr in
            // CHIRP_GAP. On this posedge, BRAM is reading that address.
            // Set bram_addr to read_ptr+1 for the second sample.
            // On the NEXT posedge (STREAM cycle 0), bram_rdata = first sample.
            ST_CHIRP_PRIME: begin
                bram_addr  <= read_ptr + 1;
                read_ptr   <= read_ptr + 1;
                sample_idx <= 10'd0;
                state      <= ST_STREAM;
            end

            // =============================================================
            // ST_DONE: Hold done flag until re-triggered.
            // On playback_start, go directly to ST_FRAME_PULSE with all
            // state reset (same as ST_IDLE path). This allows a single
            // trigger pulse to restart playback without needing two pulses.
            ST_DONE: begin
                playback_done   <= 1'b1;
                playback_active <= 1'b0;
                if (playback_start) begin
                    state           <= ST_FRAME_PULSE;
                    playback_done   <= 1'b0;
                    playback_active <= 1'b1;
                    chirp_count     <= 6'd0;
                    read_ptr        <= {ADDR_WIDTH{1'b0}};
                    frame_gap_cnt   <= 3'd0;
                end
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
