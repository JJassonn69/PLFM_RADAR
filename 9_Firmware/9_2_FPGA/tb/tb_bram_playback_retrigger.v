`timescale 1ns / 1ps
//
// tb_bram_playback_retrigger.v — Focused test for bram_playback re-trigger bug fix
//
// Verifies that:
//   1. First playback completes normally (ST_DONE reached, playback_done=1)
//   2. A second playback_start pulse transitions FSM out of ST_DONE
//   3. Second playback runs to completion with correct sample/chirp counts
//   4. playback_done deasserts during second run, then reasserts at completion
//
// Uses small parameters for fast simulation (4 chirps x 16 samples).
//
// Compile:
//   iverilog -Wall -DSIMULATION -g2012 \
//     -o tb/tb_bram_playback_retrigger.vvp \
//     tb/tb_bram_playback_retrigger.v bram_playback.v
//
// Run from: 9_Firmware/9_2_FPGA/
//   vvp tb/tb_bram_playback_retrigger.vvp
//

module tb_bram_playback_retrigger;

localparam CLK_PERIOD = 10.0;
localparam NUM_CHIRPS = 4;
localparam SAMPLES_PER_CHIRP = 16;
localparam INTER_CHIRP_GAP = 10;
localparam FRAME_START_GAP = 4;
localparam TOTAL_SAMPLES = NUM_CHIRPS * SAMPLES_PER_CHIRP;  // 64
localparam MAX_CYCLES = 50_000;

// ============================================================================
// Clock and Reset
// ============================================================================
reg clk = 0;
reg reset_n = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// ============================================================================
// DUT
// ============================================================================
reg playback_start;
wire [31:0] data_out;
wire data_valid;
wire new_chirp_frame;
wire playback_done;
wire playback_active;
wire [5:0] chirp_count;

bram_playback #(
    .NUM_CHIRPS(NUM_CHIRPS),
    .SAMPLES_PER_CHIRP(SAMPLES_PER_CHIRP),
    .INTER_CHIRP_GAP(INTER_CHIRP_GAP),
    .FRAME_START_GAP(FRAME_START_GAP)
) dut (
    .clk(clk),
    .reset_n(reset_n),
    .playback_start(playback_start),
    .data_out(data_out),
    .data_valid(data_valid),
    .new_chirp_frame(new_chirp_frame),
    .playback_done(playback_done),
    .playback_active(playback_active),
    .chirp_count(chirp_count)
);

// ============================================================================
// Counters
// ============================================================================
integer valid_count_run1 = 0;
integer valid_count_run2 = 0;
integer run = 0;  // 0 = before start, 1 = first run, 2 = second run

always @(posedge clk) begin
    if (data_valid) begin
        if (run == 1) valid_count_run1 <= valid_count_run1 + 1;
        if (run == 2) valid_count_run2 <= valid_count_run2 + 1;
    end
end

// ============================================================================
// Pass/Fail
// ============================================================================
integer pass_count = 0;
integer fail_count = 0;
integer test_count = 0;

task check;
    input cond;
    input [511:0] label;
    begin
        test_count = test_count + 1;
        if (cond)
            pass_count = pass_count + 1;
        else begin
            $display("  [FAIL] %0s", label);
            fail_count = fail_count + 1;
        end
    end
endtask

// ============================================================================
// Main Test Sequence
// ============================================================================
integer cycle_count;

initial begin
    playback_start = 1'b0;

    // Reset
    #(CLK_PERIOD * 5);
    reset_n = 1'b1;
    #(CLK_PERIOD * 5);

    $display("============================================================");
    $display("  bram_playback Re-Trigger Bug Fix Test");
    $display("  %0d chirps x %0d samples, gap=%0d", NUM_CHIRPS, SAMPLES_PER_CHIRP, INTER_CHIRP_GAP);
    $display("============================================================");

    // ---- Pre-trigger checks ----
    check(!playback_active, "Not active before trigger");
    check(!playback_done,   "Not done before trigger");

    // ==================================================================
    // RUN 1: First playback
    // ==================================================================
    $display("\n--- Run 1: First playback ---");
    run = 1;
    @(posedge clk);
    playback_start <= 1'b1;
    @(posedge clk);
    playback_start <= 1'b0;

    // Wait for playback to start
    cycle_count = 0;
    while (!playback_active && cycle_count < 100) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
    end
    check(playback_active, "Run 1: playback_active asserted");

    // Wait for playback to complete
    cycle_count = 0;
    while (!playback_done && cycle_count < MAX_CYCLES) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
    end
    check(playback_done, "Run 1: playback_done asserted");
    check(!playback_active, "Run 1: playback_active deasserted at done");
    check(chirp_count == NUM_CHIRPS, "Run 1: chirp_count == NUM_CHIRPS");
    $display("  Run 1 completed: %0d valid samples, chirp_count=%0d", valid_count_run1, chirp_count);
    check(valid_count_run1 == TOTAL_SAMPLES, "Run 1: correct sample count");

    // Stay in ST_DONE for a few cycles to verify it holds
    repeat(20) @(posedge clk);
    check(playback_done, "Run 1: still done after 20 cycles");

    // ==================================================================
    // RUN 2: Re-trigger (this was the bug — ST_DONE was a dead end)
    // ==================================================================
    $display("\n--- Run 2: Re-trigger from ST_DONE ---");
    run = 2;
    @(posedge clk);
    playback_start <= 1'b1;
    @(posedge clk);
    playback_start <= 1'b0;

    // The FSM should go ST_DONE → ST_FRAME_PULSE directly (single pulse).
    // Wait for playback_done to deassert (proves FSM left ST_DONE).
    cycle_count = 0;
    while (playback_done && cycle_count < 100) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
    end
    check(!playback_done, "Run 2: playback_done deasserted after re-trigger");
    $display("  playback_done deasserted after %0d cycles", cycle_count);

    // Wait for playback_active
    cycle_count = 0;
    while (!playback_active && cycle_count < 100) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
    end
    check(playback_active, "Run 2: playback_active asserted");

    // Wait for completion
    cycle_count = 0;
    while (!playback_done && cycle_count < MAX_CYCLES) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
    end
    check(playback_done, "Run 2: playback_done asserted");
    check(!playback_active, "Run 2: playback_active deasserted at done");
    check(chirp_count == NUM_CHIRPS, "Run 2: chirp_count == NUM_CHIRPS");
    $display("  Run 2 completed: %0d valid samples, chirp_count=%0d", valid_count_run2, chirp_count);
    check(valid_count_run2 == TOTAL_SAMPLES, "Run 2: correct sample count");

    // ==================================================================
    // Summary
    // ==================================================================
    $display("");
    $display("============================================================");
    $display("  SUMMARY: bram_playback Re-Trigger Test");
    $display("============================================================");
    $display("  Run 1 samples: %0d (expected %0d)", valid_count_run1, TOTAL_SAMPLES);
    $display("  Run 2 samples: %0d (expected %0d)", valid_count_run2, TOTAL_SAMPLES);
    $display("  Pass: %0d  Fail: %0d", pass_count, fail_count);
    $display("============================================================");

    if (fail_count == 0)
        $display("RESULT: ALL TESTS PASSED (%0d/%0d)", pass_count, test_count);
    else
        $display("RESULT: %0d TESTS FAILED", fail_count);
    $display("============================================================");

    #(CLK_PERIOD * 10);
    $finish;
end

// Watchdog
initial begin
    #(CLK_PERIOD * MAX_CYCLES * 4);
    $display("[TIMEOUT] Simulation exceeded maximum time");
    $display("SOME TESTS FAILED");
    $finish;
end

endmodule
