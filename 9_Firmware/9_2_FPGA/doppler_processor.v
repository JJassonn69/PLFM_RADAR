`timescale 1ns / 1ps

// ============================================================================
// doppler_processor.v — Multi-subframe Doppler Processor (chirp-v2 PR-F)
// ============================================================================
//
// ARCHITECTURE:
//   Processes NUM_SUBFRAMES = CHIRPS_PER_FRAME / CHIRPS_PER_SUBFRAME independent
//   16-point FFTs per range bin. The chirp-v2 production build runs three
//   sub-frames (SHORT, MEDIUM, LONG) at 16 chirps each = 48 chirps per frame:
//
//     Sub-frame 0: chirps  0..15 → 16-pt windowed FFT  (SHORT in chirp-v2)
//     Sub-frame 1: chirps 16..31 → 16-pt windowed FFT  (MEDIUM in chirp-v2)
//     Sub-frame 2: chirps 32..47 → 16-pt windowed FFT  (LONG  in chirp-v2)
//
//   Each sub-frame produces 16 Doppler bins per range bin. Outputs are tagged
//   with the 2-bit sub_frame index and the 4-bit bin index is packed into the
//   6-bit doppler_bin port as {sub_frame[1:0], bin[3:0]}.
//
//   Legacy 2-subframe golden-vector tests (tb_doppler_realdata,
//   tb_fullchain_realdata) override CHIRPS_PER_FRAME=32 + CHIRPS_PER_SUBFRAME=16
//   to make NUM_SUBFRAMES=2; the FSM generalises cleanly. doppler_bin still
//   reports 6 bits there with the high bit always zero.
//
//   Staggered-PRF ambiguity resolution downstream picks the matching Doppler
//   bin from the SHORT vs MEDIUM vs LONG sub-frame to resolve velocity.
//
// WINDOW:
//   16-point Hamming window (Q15), symmetric. Computed as:
//     w[n] = 0.54 - 0.46 * cos(2*pi*n/15), n=0..15
// ============================================================================

`include "radar_params.vh"

// ----------------------------------------------------------------------------
// [RX-D FIX] RANGE_BINS and range_bin port now scale with `RP_MAX_OUTPUT_BINS
// and `RP_RANGE_BIN_WIDTH_MAX (auto-conditional on SUPPORT_LONG_RANGE).
//   50T  (no SUPPORT_LONG_RANGE): 512 bins / 9-bit  — 3 km only
//   200T (SUPPORT_LONG_RANGE):    4096 bins / 12-bit — 3 km and 20 km
// In 3 km mode the upstream produces 512 bins (uses bins 0..511 only on 200T).
// In 20 km mode the upstream produces 4096 bins, which the BRAMs and counters
// can now represent without aliasing.
// ----------------------------------------------------------------------------
module doppler_processor_optimized #(
    parameter DOPPLER_FFT_SIZE   = `RP_DOPPLER_FFT_SIZE,    // 16
    parameter RANGE_BINS         = `RP_MAX_OUTPUT_BINS,     // 512 (50T) / 4096 (200T)
    parameter CHIRPS_PER_FRAME   = `RP_CHIRPS_PER_FRAME,    // 48 (PR-F); legacy TBs override to 32
    parameter CHIRPS_PER_SUBFRAME = `RP_CHIRPS_PER_SUBFRAME, // 16
    parameter WINDOW_TYPE        = 0,      // 0=Hamming, 1=Rectangular
    parameter DATA_WIDTH         = `RP_DATA_WIDTH           // 16
)(
    input wire clk,
    input wire reset_n,
    input wire [31:0] range_data,
    input wire data_valid,
    input wire new_chirp_frame,
    output reg [31:0] doppler_output,
    output reg doppler_valid,
    output reg [`RP_DOPPLER_BIN_WIDTH-1:0] doppler_bin,    // 6-bit {sub_frame[1:0], bin[3:0]}
    output reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin,    // 9-bit (50T) / 12-bit (200T)
    output reg [`RP_SUBFRAME_ID_WIDTH-1:0] sub_frame,      // 2-bit subframe index
    output wire processing_active,
    output wire frame_complete,
    output reg [3:0] status

`ifdef FORMAL
    ,
    output wire [2:0]  fv_state,
    output wire [`RP_DOPPLER_MEM_ADDR_W-1:0] fv_mem_write_addr,
    output wire [`RP_DOPPLER_MEM_ADDR_W-1:0] fv_mem_read_addr,
    output wire [`RP_RANGE_BIN_WIDTH_MAX-1:0]     fv_write_range_bin,
    output wire [5:0]  fv_write_chirp_index,
    output wire [`RP_RANGE_BIN_WIDTH_MAX-1:0]     fv_read_range_bin,
    output wire [5:0]  fv_read_doppler_index,
    output wire [9:0]  fv_processing_timeout,
    output wire        fv_frame_buffer_full,
    output wire        fv_mem_we,
    output wire [`RP_DOPPLER_MEM_ADDR_W-1:0] fv_mem_waddr_r
`endif
);

// Derived: number of sub-frames in the current configuration. Production
// build = 3 (SHORT/MEDIUM/LONG @ 16 chirps each = 48 frame). Legacy TBs
// override CHIRPS_PER_FRAME=32 to get NUM_SUBFRAMES=2 for golden compat.
localparam NUM_SUBFRAMES = CHIRPS_PER_FRAME / CHIRPS_PER_SUBFRAME;

// ==============================================
// Window Coefficients — 16-point Hamming (Q15)
// ==============================================
// w[n] = 0.54 - 0.46 * cos(2*pi*n/15), n=0..15
// Symmetric: w[n] = w[15-n]
reg [DATA_WIDTH-1:0] window_coeff [0:15];

integer w;
initial begin
    if (WINDOW_TYPE == 0) begin
        // 16-point Hamming window, Q15 format
        // Computed: round(32767 * (0.54 - 0.46*cos(2*pi*n/15)))
        window_coeff[0]  = 16'h0A3D;  // 0.0800 * 32767 = 2621
        window_coeff[1]  = 16'h0E5C;  // 0.1116 * 32767 = 3676
        window_coeff[2]  = 16'h1B6D;  // 0.2138 * 32767 = 7021
        window_coeff[3]  = 16'h3088;  // 0.3790 * 32767 = 12424
        window_coeff[4]  = 16'h4B33;  // 0.5868 * 32767 = 19251
        window_coeff[5]  = 16'h6573;  // 0.7930 * 32767 = 25971
        window_coeff[6]  = 16'h7642;  // 0.9245 * 32767 = 30274
        window_coeff[7]  = 16'h7F62;  // 0.9932 * 32767 = 32610
        window_coeff[8]  = 16'h7F62;  // symmetric
        window_coeff[9]  = 16'h7642;
        window_coeff[10] = 16'h6573;
        window_coeff[11] = 16'h4B33;
        window_coeff[12] = 16'h3088;
        window_coeff[13] = 16'h1B6D;
        window_coeff[14] = 16'h0E5C;
        window_coeff[15] = 16'h0A3D;
    end else begin
        for (w = 0; w < 16; w = w + 1) begin
            window_coeff[w] = 16'h7FFF;
        end
    end
end

// ==============================================
// Memory Declaration - FIXED SIZE
// ==============================================
localparam MEM_DEPTH = RANGE_BINS * CHIRPS_PER_FRAME;
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] doppler_i_mem [0:MEM_DEPTH-1];
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] doppler_q_mem [0:MEM_DEPTH-1];

// ==============================================
// Control Registers
// ==============================================
reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] write_range_bin;
reg [5:0] write_chirp_index;          // 6-bit: 0..47 (PR-F)
reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] read_range_bin;
reg [5:0] read_doppler_index;         // 6-bit (PR-F)
reg frame_buffer_full;
reg [9:0] chirps_received;
reg [1:0] chirp_state;

// AUDIT-S3 fix: arm-on-frame-start gating. Set when frame_start_pulse arrives
// in S_IDLE; cleared when the FSM transitions to S_ACCUMULATE. Prevents stale
// data_valid from prior MF pipeline residue from advancing S_IDLE → S_ACCUMULATE
// before the new frame is officially started, which would write the first
// sample(s) into addr 0 of the previous frame's buffer if write_chirp_index
// happened to be non-zero. The pointer-reset invariant (line 287-288 always
// zeros pointers at end of S_ACCUMULATE) makes this race benign in current
// operation, but the gate makes the FSM robust against future code paths
// that might leave pointers stale on entry to S_IDLE.
reg frame_armed;

// Sub-frame tracking
reg [`RP_SUBFRAME_ID_WIDTH-1:0] current_sub_frame;  // 2-bit (PR-F): 0..NUM_SUBFRAMES-1

// ==============================================
// FFT Interface
// ==============================================
reg fft_start;
wire fft_ready;
reg [DATA_WIDTH-1:0] fft_input_i;
reg [DATA_WIDTH-1:0] fft_input_q;
reg signed [31:0] mult_i, mult_q;
reg signed [DATA_WIDTH-1:0] window_val_reg;
reg signed [31:0] mult_i_raw, mult_q_raw;

reg fft_input_valid;
reg fft_input_last;
wire [DATA_WIDTH-1:0] fft_output_i;
wire [DATA_WIDTH-1:0] fft_output_q;
wire fft_output_valid;
wire fft_output_last;

// ==============================================
// Addressing
// ==============================================
wire [`RP_DOPPLER_MEM_ADDR_W-1:0] mem_write_addr;
wire [`RP_DOPPLER_MEM_ADDR_W-1:0] mem_read_addr;

assign mem_write_addr = (write_chirp_index * RANGE_BINS) + write_range_bin;
assign mem_read_addr = (read_doppler_index * RANGE_BINS) + read_range_bin;

// ==============================================
// State Machine
// ==============================================
reg [2:0] state;
localparam S_IDLE       = 3'b000;
localparam S_ACCUMULATE = 3'b001;
localparam S_PRE_READ   = 3'b101;
localparam S_LOAD_FFT   = 3'b010;
localparam S_FFT_WAIT   = 3'b011;
localparam S_OUTPUT     = 3'b100;

// Frame sync detection
reg new_chirp_frame_d1;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) new_chirp_frame_d1 <= 0;
    else new_chirp_frame_d1 <= new_chirp_frame;
end
wire frame_start_pulse = new_chirp_frame & ~new_chirp_frame_d1;

// ==============================================
// Main State Machine
// ==============================================
reg [4:0] fft_sample_counter;  // Reduced: only need 0..17 for 16-pt FFT
reg [9:0] processing_timeout;

// Memory write enable and data signals
reg mem_we;
reg [`RP_DOPPLER_MEM_ADDR_W-1:0] mem_waddr_r;
reg [DATA_WIDTH-1:0] mem_wdata_i, mem_wdata_q;

// Memory read data
reg [DATA_WIDTH-1:0] mem_rdata_i, mem_rdata_q;

`ifdef FORMAL
assign fv_state              = state;
assign fv_mem_write_addr     = mem_write_addr;
assign fv_mem_read_addr      = mem_read_addr;
assign fv_write_range_bin    = write_range_bin;
assign fv_write_chirp_index  = write_chirp_index;
assign fv_read_range_bin     = read_range_bin;
assign fv_read_doppler_index = read_doppler_index;
assign fv_processing_timeout = processing_timeout;
assign fv_frame_buffer_full  = frame_buffer_full;
assign fv_mem_we             = mem_we;
assign fv_mem_waddr_r        = mem_waddr_r;
`endif

// ----------------------------------------------------------
// Separate always block for memory writes — NO async reset
// ----------------------------------------------------------
always @(posedge clk) begin
    if (mem_we) begin
        doppler_i_mem[mem_waddr_r] <= mem_wdata_i;
        doppler_q_mem[mem_waddr_r] <= mem_wdata_q;
    end
    mem_rdata_i <= doppler_i_mem[mem_read_addr];
    mem_rdata_q <= doppler_q_mem[mem_read_addr];
end

// ----------------------------------------------------------
// Block 1: FSM / Control — async reset
// ----------------------------------------------------------
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state <= S_IDLE;
        write_range_bin <= 0;
        write_chirp_index <= 0;
        frame_buffer_full <= 0;
        doppler_valid <= 0;
        fft_start <= 0;
        fft_input_valid <= 0;
        fft_input_last <= 0;
        fft_sample_counter <= 0;
        processing_timeout <= 0;
        status <= 0;
        chirps_received <= 0;
        chirp_state <= 0;
        doppler_output <= 0;
        doppler_bin <= 0;
        range_bin <= 0;
        sub_frame <= 0;
        current_sub_frame <= 0;
        frame_armed <= 0;
    end else begin
        doppler_valid <= 0;
        fft_input_valid <= 0;
        fft_input_last <= 0;
        
        if (processing_timeout > 0) begin
            processing_timeout <= processing_timeout - 1;
        end
        
        case (state)
            S_IDLE: begin
                if (frame_start_pulse) begin
                    write_chirp_index <= 0;
                    write_range_bin <= 0;
                    frame_buffer_full <= 0;
                    chirps_received <= 0;
                    frame_armed <= 1;     // AUDIT-S3: arm on frame_start_pulse
                end

                // AUDIT-S3 fix: only transition to S_ACCUMULATE when armed,
                // i.e., when this frame has been officially started by a
                // frame_start_pulse. Pre-fix code accepted any data_valid in
                // S_IDLE and could race with a missing/late frame_start_pulse.
                // (frame_start_pulse || frame_armed) admits the same-cycle case
                // where both pulse and data_valid arrive together — write to
                // addr 0 still resolves correctly because the BRAM write block
                // uses the same gate.
                if ((frame_start_pulse || frame_armed) && data_valid && !frame_buffer_full) begin
                    state <= S_ACCUMULATE;
                    write_range_bin <= 1;
                    frame_armed <= 0;     // disarm; S_ACCUMULATE handles its own pointers
                end
            end

            S_ACCUMULATE: begin
                if (data_valid) begin
                    if (write_range_bin < RANGE_BINS - 1) begin
                        write_range_bin <= write_range_bin + 1;
                    end else begin
                        write_range_bin <= 0;
                        write_chirp_index <= write_chirp_index + 1;
                        chirps_received <= chirps_received + 1;
                        
                        if (write_chirp_index >= CHIRPS_PER_FRAME - 1) begin
                            frame_buffer_full <= 1;
                            chirp_state <= 0;
                            state <= S_PRE_READ;
                            fft_sample_counter <= 0;
                            write_chirp_index <= 0;
                            write_range_bin <= 0;
                            // Start with sub-frame 0 (long PRI chirps 0..15)
                            current_sub_frame <= 0;
                        end
                    end
                end 
            end
            
            S_PRE_READ: begin
                // Prime BRAM pipeline for current sub-frame
                // read_doppler_index already set in Block 2 to sub-frame base
                fft_start <= 1;
                state <= S_LOAD_FFT;
            end

            S_LOAD_FFT: begin
                fft_start <= 0;
                
                // Pipeline: 2 priming cycles + CHIRPS_PER_SUBFRAME data cycles
                if (fft_sample_counter <= 1) begin
                    fft_sample_counter <= fft_sample_counter + 1;
                end else if (fft_sample_counter <= CHIRPS_PER_SUBFRAME + 1) begin
                    fft_input_valid <= 1;

                    if (fft_sample_counter == CHIRPS_PER_SUBFRAME + 1) begin
                        fft_input_last <= 1;
                        state <= S_FFT_WAIT;
                        fft_sample_counter <= 0;
                        processing_timeout <= 1000;
                    end else begin
                        fft_sample_counter <= fft_sample_counter + 1;
                    end
                end
            end
            
            S_FFT_WAIT: begin
                if (fft_output_valid) begin
                    doppler_output <= {fft_output_q[15:0], fft_output_i[15:0]};
                    // Pack: {sub_frame, bin[3:0]}
                    doppler_bin <= {current_sub_frame, fft_sample_counter[3:0]};
                    range_bin <= read_range_bin;
                    sub_frame <= current_sub_frame;
                    doppler_valid <= 1;
                    
                    fft_sample_counter <= fft_sample_counter + 1;
                    
                    if (fft_output_last) begin
                        state <= S_OUTPUT;
                        fft_sample_counter <= 0;
                    end
                end
                
                if (processing_timeout == 0) begin
                    state <= S_OUTPUT;
                end
            end
            
            S_OUTPUT: begin
                if (current_sub_frame < NUM_SUBFRAMES - 1) begin
                    // Advance to next sub-frame; same range bin, next FFT
                    current_sub_frame <= current_sub_frame + 1;
                    fft_sample_counter <= 0;
                    state <= S_PRE_READ;
                end else begin
                    // Finished all NUM_SUBFRAMES for this range bin
                    current_sub_frame <= 0;
                    if (read_range_bin < RANGE_BINS - 1) begin
                        fft_sample_counter <= 0;
                        state <= S_PRE_READ;
                    end else begin
                        state <= S_IDLE;
                        frame_buffer_full <= 0;
                    end
                end
            end
            
        endcase
        
        status <= {state, frame_buffer_full};
    end
end

// ----------------------------------------------------------
// Block 2: BRAM address/data & DSP datapath — synchronous reset
// ----------------------------------------------------------
always @(posedge clk) begin
    if (!reset_n) begin
        mem_we      <= 0;
        mem_waddr_r <= 0;
        mem_wdata_i <= 0;
        mem_wdata_q <= 0;
        mult_i      <= 0;
        mult_q      <= 0;
        mult_i_raw     <= 0;
        mult_q_raw     <= 0;
        window_val_reg <= 0;
        fft_input_i <= 0;
        fft_input_q <= 0;
        read_range_bin     <= 0;
        read_doppler_index <= 0;
    end else begin
        mem_we <= 0;
        
        case (state)
            S_IDLE: begin
                // AUDIT-S3 fix: gate BRAM write on frame_armed so stale
                // data_valid arriving before frame_start_pulse cannot
                // overwrite addr 0 of the buffer. Same gate as the FSM's
                // S_IDLE → S_ACCUMULATE transition above, so the two blocks
                // stay coherent.
                if ((frame_start_pulse || frame_armed) && data_valid && !frame_buffer_full) begin
                    mem_we      <= 1;
                    mem_waddr_r <= mem_write_addr;
                    mem_wdata_i <= range_data[15:0];
                    mem_wdata_q <= range_data[31:16];
                end
            end

            S_ACCUMULATE: begin
                if (data_valid) begin
                    mem_we      <= 1;
                    mem_waddr_r <= mem_write_addr;
                    mem_wdata_i <= range_data[15:0];
                    mem_wdata_q <= range_data[31:16];

                    if (write_range_bin >= RANGE_BINS - 1 &&
                        write_chirp_index >= CHIRPS_PER_FRAME - 1) begin
                        read_range_bin     <= 0;
                        // Start reading from chirp 0 (long PRI sub-frame)
                        read_doppler_index <= 0;
                    end
                end
            end
            
            S_PRE_READ: begin
                // First chirp of current sub-frame + 1 (address-then-data pipe).
                // Generalised: chirp_base = current_sub_frame * CHIRPS_PER_SUBFRAME.
                read_doppler_index <= current_sub_frame * CHIRPS_PER_SUBFRAME + 6'd1;

                // BREG priming: window coeff for sample 0
                window_val_reg <= $signed(window_coeff[0]);
            end

            S_LOAD_FFT: begin
                if (fft_sample_counter == 0) begin
                    // Pipe stage 1: multiply using pre-registered BREG value
                    mult_i_raw <= $signed(mem_rdata_i) * window_val_reg;
                    mult_q_raw <= $signed(mem_rdata_q) * window_val_reg;
                    window_val_reg <= $signed(window_coeff[1]);
                    // Advance to chirp base+2
                    read_doppler_index <= current_sub_frame * CHIRPS_PER_SUBFRAME + 6'd2;
                end else if (fft_sample_counter == 1) begin
                    mult_i <= mult_i_raw;
                    mult_q <= mult_q_raw;
                    mult_i_raw <= $signed(mem_rdata_i) * window_val_reg;
                    mult_q_raw <= $signed(mem_rdata_q) * window_val_reg;
                    if (2 < CHIRPS_PER_SUBFRAME)
                        window_val_reg <= $signed(window_coeff[2]);
                    // Advance to chirp base+3
                    read_doppler_index <= current_sub_frame * CHIRPS_PER_SUBFRAME + 6'd3;
                end else if (fft_sample_counter <= CHIRPS_PER_SUBFRAME + 1) begin
                    // Steady state
                    fft_input_i <= (mult_i + (1 << 14)) >>> 15;
                    fft_input_q <= (mult_q + (1 << 14)) >>> 15;
                    mult_i <= mult_i_raw;
                    mult_q <= mult_q_raw;

                    if (fft_sample_counter <= CHIRPS_PER_SUBFRAME - 1) begin
                        mult_i_raw <= $signed(mem_rdata_i) * window_val_reg;
                        mult_q_raw <= $signed(mem_rdata_q) * window_val_reg;
                        // Window coeff index within sub-frame
                        begin : advance_window
                            reg [4:0] win_idx;
                            win_idx = fft_sample_counter[3:0] + 1;
                            if (win_idx < CHIRPS_PER_SUBFRAME)
                                window_val_reg <= $signed(window_coeff[win_idx]);
                        end
                        // Advance BRAM read: chirp_base + (counter + 2).
                        // The last useful read is data[chirp_base + CPS-1], needed
                        // by mult_i_raw at counter=CPS-1. Working back through the
                        // 2-cycle BRAM-then-multiply pipeline, the last NBA that
                        // matters is at counter = CPS-3 (= 13 for CPS=16) which
                        // schedules read of base+CPS-1. After that, advancing
                        // would address chirp base+CPS or base+CPS+1 — past the
                        // end of the highest sub-frame's data window (e.g. chirps
                        // 48 / 49 with sub_frame=2 in a 48-chirp frame), which is
                        // outside MEM_DEPTH = RANGE_BINS * CHIRPS_PER_FRAME. The
                        // would-be values are never consumed, but the reads
                        // would still drive an out-of-range mem_read_addr. Stop
                        // the read pointer at the last useful chirp instead.
                        if (fft_sample_counter <= CHIRPS_PER_SUBFRAME - 3) begin
                            read_doppler_index <= current_sub_frame * CHIRPS_PER_SUBFRAME
                                                  + {2'd0, fft_sample_counter[3:0]} + 6'd2;
                        end
                    end

                    if (fft_sample_counter == CHIRPS_PER_SUBFRAME + 1) begin
                        // Reset read index for the next sub-frame (or wrap to 0
                        // when we've finished all NUM_SUBFRAMES).
                        if (current_sub_frame < NUM_SUBFRAMES - 1)
                            read_doppler_index <= (current_sub_frame + 6'd1) * CHIRPS_PER_SUBFRAME;
                        else
                            read_doppler_index <= 6'd0;
                    end
                end
            end

            S_OUTPUT: begin
                if (current_sub_frame < NUM_SUBFRAMES - 1) begin
                    // Transitioning to next sub-frame for the same range bin.
                    read_doppler_index <= (current_sub_frame + 6'd1) * CHIRPS_PER_SUBFRAME;
                end else begin
                    // All sub-frames done for this range bin
                    if (read_range_bin < RANGE_BINS - 1) begin
                        read_range_bin     <= read_range_bin + 1;
                        read_doppler_index <= 6'd0;  // Next range bin starts with sub-frame 0
                    end
                end
            end

            default: begin
                // S_FFT_WAIT: no BRAM-write or address operations needed
            end
        endcase
    end
end

// ==============================================
// FFT Module — 16-point
// ==============================================
xfft_16 fft_inst (
    .aclk(clk),
    .aresetn(reset_n),
    .s_axis_config_tdata(8'h01),
    .s_axis_config_tvalid(fft_start),
    .s_axis_config_tready(fft_ready),
    .s_axis_data_tdata({fft_input_q, fft_input_i}),
    .s_axis_data_tvalid(fft_input_valid),
    .s_axis_data_tlast(fft_input_last),
    .m_axis_data_tdata({fft_output_q, fft_output_i}),
    .m_axis_data_tvalid(fft_output_valid),
    .m_axis_data_tlast(fft_output_last),
    .m_axis_data_tready(1'b1)
);

// ==============================================
// Status Outputs
// ==============================================
assign processing_active = (state != S_IDLE);
// NOTE: frame_complete is a LEVEL, not a pulse. It is high whenever the
// doppler processor is idle with no buffered frame. radar_receiver_final.v
// converts this to a single-cycle rising-edge pulse before routing to
// downstream consumers (USB FT2232H, AGC, CFAR). Do NOT connect this
// level output directly to modules that expect a pulse.
assign frame_complete = (state == S_IDLE && frame_buffer_full == 0);

endmodule
