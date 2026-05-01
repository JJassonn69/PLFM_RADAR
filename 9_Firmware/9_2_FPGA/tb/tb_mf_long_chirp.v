`timescale 1ns/1ps
`include "radar_params.vh"

// ============================================================================
// tb_mf_long_chirp.v
//
// PR-J.1 — focused diagnostic for the matched_filter_multi_segment LONG-chirp
// hang. Drives a single chirp (selectable wave_sel via plusarg) into the
// production multi_segment + processing_chain + chirp_reference_rom stack,
// and logs every state transition of:
//
//   ms_state          — matched_filter_multi_segment FSM
//   ch_state          — matched_filter_processing_chain FSM
//   pc_valid          — chain output stream
//   mem_request /     — multi_seg <-> chirp_reference_rom handshake
//   mem_ready
//   segment_request,  — current segment being processed
//   current_segment
//
// Run as:
//   vvp tb_mf_long_chirp.vvp                  # default LONG (wave_sel=2)
//   vvp tb_mf_long_chirp.vvp +WAVE=0          # SHORT (control: known-good)
//   vvp tb_mf_long_chirp.vvp +WAVE=1          # MEDIUM (control: known-good)
//
// Pass criterion (per-segment):
//   - chirp_pulse seen
//   - mem_request → mem_ready handshake completes for each segment
//   - chain enters non-IDLE state, emits FFT_SIZE pc_valid pulses
//   - multi_seg observes pc_valid, transitions ST_WAIT_FFT → ST_OUTPUT
//   - all segments emit; FSM returns to ST_IDLE
//
// Observed: LONG hangs in ST_WAIT_FFT (state=5) on segment 0. This TB
// captures the cycle-accurate signature so PR-J.2 can fix the right thing.
// ============================================================================

module tb_mf_long_chirp;

    localparam CLK_PERIOD = 10.0;       // 100 MHz
    localparam FFT_SIZE   = `RP_FFT_SIZE; // 2048
    localparam SHORT_LEN  = 100;        // RP_DEF_SHORT_CHIRP_CYCLES_V2
    localparam MEDIUM_LEN = 500;        // RP_DEF_MEDIUM_CHIRP_CYCLES
    localparam LONG_LEN   = `RP_LONG_CHIRP_SAMPLES_3KM; // 3000
    localparam HARD_BUDGET_CYCLES = 200_000;

    reg                 clk;
    reg                 reset_n;
    reg  signed [17:0]  ddc_i;
    reg  signed [17:0]  ddc_q;
    reg                 ddc_valid;
    reg  [1:0]          wave_sel_r;
    reg  [5:0]          chirp_counter;
    reg                 chirp_pulse;

    wire [1:0]          segment_request;
    wire [10:0]         sample_addr_out;
    wire                mem_request;
    wire                mem_ready_loader;
    wire [15:0]         ref_i_raw;
    wire [15:0]         ref_q_raw;

    wire signed [15:0]  pc_i;
    wire signed [15:0]  pc_q;
    wire                pc_valid;
    wire [3:0]          ms_status;

    wire [1:0]          wave_sel = wave_sel_r;

    // ----- Chirp reference ROM (production wiring) -----
    chirp_reference_rom chirp_rom (
        .clk            (clk),
        .reset_n        (reset_n),
        .wave_sel       (wave_sel),
        .segment_select (segment_request),
        .mem_request    (mem_request),
        .sample_addr    (sample_addr_out),
        .ref_i          (ref_i_raw),
        .ref_q          (ref_q_raw),
        .mem_ready      (mem_ready_loader)
    );

    // 1-FF align register, mirrors radar_receiver_final.v
    reg [15:0] ref_i_d, ref_q_d;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ref_i_d <= 16'd0;
            ref_q_d <= 16'd0;
        end else begin
            ref_i_d <= ref_i_raw;
            ref_q_d <= ref_q_raw;
        end
    end

    // ----- multi_segment DUT -----
    matched_filter_multi_segment ms_dut (
        .clk              (clk),
        .reset_n          (reset_n),
        .ddc_i            (ddc_i),
        .ddc_q            (ddc_q),
        .ddc_valid        (ddc_valid),
        .wave_sel         (wave_sel),
        .chirp_counter    (chirp_counter),
        .chirp_pulse      (chirp_pulse),
        .ref_chirp_real   (ref_i_d),
        .ref_chirp_imag   (ref_q_d),
        .segment_request  (segment_request),
        .sample_addr_out  (sample_addr_out),
        .mem_request      (mem_request),
        .mem_ready        (mem_ready_loader),
        .pc_i_w           (pc_i),
        .pc_q_w           (pc_q),
        .pc_valid_w       (pc_valid),
        .status           (ms_status)
    );

    always #(CLK_PERIOD/2.0) clk = ~clk;

    // ----- Hierarchical probes (read-only) -----
    wire [3:0] ms_state = ms_dut.state;
    wire [3:0] ch_state = ms_dut.fft_chain_state;
    wire [2:0] curr_seg = ms_dut.current_segment;

    // ----- Transition logger -----
    reg [3:0] ms_state_prev;
    reg [3:0] ch_state_prev;
    reg       mem_req_prev;
    reg       mem_rdy_prev;
    reg       pc_valid_prev;
    integer   cycle_count;
    integer   pc_pulse_count;
    integer   first_pc_cycle;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ms_state_prev  <= 4'h0;
            ch_state_prev  <= 4'h0;
            mem_req_prev   <= 1'b0;
            mem_rdy_prev   <= 1'b0;
            pc_valid_prev  <= 1'b0;
            cycle_count    <= 0;
            pc_pulse_count <= 0;
            first_pc_cycle <= -1;
        end else begin
            cycle_count <= cycle_count + 1;

            if (ms_state != ms_state_prev) begin
                $display("[%6d] ms_state: %0d -> %0d   (curr_seg=%0d, segment_request=%0d)",
                         cycle_count, ms_state_prev, ms_state, curr_seg, segment_request);
                ms_state_prev <= ms_state;
            end
            if (ch_state != ch_state_prev) begin
                $display("[%6d] ch_state: %0d -> %0d",
                         cycle_count, ch_state_prev, ch_state);
                ch_state_prev <= ch_state;
            end
            if (mem_request != mem_req_prev) begin
                $display("[%6d] mem_request: %0d -> %0d",
                         cycle_count, mem_req_prev, mem_request);
                mem_req_prev <= mem_request;
            end
            if (mem_ready_loader != mem_rdy_prev) begin
                $display("[%6d] mem_ready:   %0d -> %0d",
                         cycle_count, mem_rdy_prev, mem_ready_loader);
                mem_rdy_prev <= mem_ready_loader;
            end
            if (pc_valid && !pc_valid_prev) begin
                if (first_pc_cycle == -1) first_pc_cycle <= cycle_count;
                $display("[%6d] pc_valid rising  (count so far=%0d)",
                         cycle_count, pc_pulse_count);
            end
            if (!pc_valid && pc_valid_prev) begin
                $display("[%6d] pc_valid falling (count so far=%0d)",
                         cycle_count, pc_pulse_count);
            end
            pc_valid_prev <= pc_valid;
            if (pc_valid) pc_pulse_count <= pc_pulse_count + 1;
        end
    end

    // ----- Stimulus driver -----
    integer chirp_len;
    integer total_segments_expected;
    reg [255:0] wave_name;

    task feed_chirp;
        input integer n_samples;
        integer j;
        begin
            // DC-ish ramp pattern, just to give the FSM something to process.
            for (j = 0; j < n_samples; j = j + 1) begin
                ddc_i     <= $signed(j[7:0]);          // small 8-bit ramp, sign-extended
                ddc_q     <= -$signed(j[7:0]);
                ddc_valid <= 1'b1;
                @(posedge clk);
            end
            ddc_valid <= 1'b0;
            ddc_i     <= 18'd0;
            ddc_q     <= 18'd0;
        end
    endtask

    integer wave_arg;

    initial begin
        // Default: LONG. Override via +WAVE=N (0=SHORT, 1=MEDIUM, 2=LONG).
        if (!$value$plusargs("WAVE=%d", wave_arg)) begin
            wave_arg = 2;
        end

        case (wave_arg)
            0: begin wave_sel_r = `RP_WAVE_SHORT;  chirp_len = SHORT_LEN;  total_segments_expected = 1; wave_name = "SHORT"; end
            1: begin wave_sel_r = `RP_WAVE_MEDIUM; chirp_len = MEDIUM_LEN; total_segments_expected = 1; wave_name = "MEDIUM"; end
            default: begin wave_sel_r = `RP_WAVE_LONG; chirp_len = LONG_LEN; total_segments_expected = `RP_LONG_SEGMENTS_3KM; wave_name = "LONG"; end
        endcase

        clk           = 0;
        reset_n       = 0;
        ddc_i         = 0;
        ddc_q         = 0;
        ddc_valid     = 0;
        chirp_counter = 6'd0;
        chirp_pulse   = 1'b0;

        repeat (8) @(posedge clk);
        reset_n = 1;
        repeat (8) @(posedge clk);

        $display("============================================================");
        $display("  tb_mf_long_chirp — wave=%0s, chirp_len=%0d samples, expected segments=%0d",
                 wave_name, chirp_len, total_segments_expected);
        $display("============================================================");
        $display("[%6d] reset released, starting", cycle_count);

        // Pulse chirp_pulse for one cycle
        @(posedge clk);
        #1 chirp_pulse = 1'b1;
        @(posedge clk);
        #1 chirp_pulse = 1'b0;

        feed_chirp(chirp_len);

        // Wait for FFT_SIZE outputs OR hard budget
        begin : wait_loop
            integer w;
            for (w = 0; w < HARD_BUDGET_CYCLES; w = w + 1) begin
                @(posedge clk);
                if (pc_pulse_count >= FFT_SIZE * total_segments_expected) begin
                    $display("[%6d] === Reached %0d pc_valid pulses (expected %0d) ===",
                             cycle_count, pc_pulse_count, FFT_SIZE * total_segments_expected);
                    disable wait_loop;
                end
            end
        end

        $display("\n============================================================");
        $display("  FINAL STATE for wave=%0s", wave_name);
        $display("============================================================");
        $display("  cycle_count        : %0d", cycle_count);
        $display("  ms_state           : %0d", ms_state);
        $display("  ch_state           : %0d", ch_state);
        $display("  current_segment    : %0d", curr_seg);
        $display("  segment_request    : %0d", segment_request);
        $display("  pc_pulse_count     : %0d (expected %0d)",
                 pc_pulse_count, FFT_SIZE * total_segments_expected);
        $display("  first_pc_cycle     : %0d", first_pc_cycle);
        $display("  ms_status          : %b", ms_status);
        $display("  mem_request        : %0d", mem_request);
        $display("  mem_ready          : %0d", mem_ready_loader);
        if (pc_pulse_count >= FFT_SIZE * total_segments_expected)
            $display("  [PASS] All segments produced expected output count");
        else
            $display("  [FAIL] Hung — %0d pc_valid pulses short", FFT_SIZE * total_segments_expected - pc_pulse_count);

        $finish;
    end

    // Hard sim timeout
    initial begin
        #50_000_000; // 50 ms wall-equivalent
        $display("[ERROR] Hard simulation timeout");
        $finish;
    end

endmodule
