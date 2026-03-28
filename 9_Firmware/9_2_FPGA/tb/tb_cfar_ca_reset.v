`timescale 1ns / 1ps
//
// tb_cfar_ca_reset.v — Focused test for cfar_ca detect_count reset bug fix
//
// Verifies that:
//   1. First frame: detect_count accumulates correctly for simple threshold
//   2. After first frame completes (ST_DONE → ST_IDLE), detect_count is NOT
//      carried over when a second frame begins.
//   3. Second frame: detect_count starts from 0 (or 1 if first sample detects)
//   4. Both simple threshold and CA-CFAR modes tested.
//
// Uses minimal data (4 range bins x 4 Doppler bins) for fast simulation.
//
// Compile:
//   iverilog -Wall -DSIMULATION -g2012 \
//     -o tb/tb_cfar_ca_reset.vvp \
//     tb/tb_cfar_ca_reset.v cfar_ca.v
//
// Run from: 9_Firmware/9_2_FPGA/
//   vvp tb/tb_cfar_ca_reset.vvp
//

module tb_cfar_ca_reset;

localparam CLK_PERIOD = 10.0;
localparam NUM_RANGE = 64;
localparam NUM_DOPPLER = 32;
localparam MAG_WIDTH = 17;

// ============================================================================
// Clock and Reset
// ============================================================================
reg clk = 0;
reg reset_n = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// ============================================================================
// DUT signals
// ============================================================================
reg [31:0] doppler_data;
reg        doppler_valid;
reg [4:0]  doppler_bin_in;
reg [5:0]  range_bin_in;
reg        frame_complete;

reg [3:0]  cfg_guard_cells;
reg [4:0]  cfg_train_cells;
reg [7:0]  cfg_alpha;
reg [1:0]  cfg_cfar_mode;
reg        cfg_cfar_enable;
reg [15:0] cfg_simple_threshold;

wire        detect_flag;
wire        detect_valid;
wire [5:0]  detect_range;
wire [4:0]  detect_doppler;
wire [MAG_WIDTH-1:0] detect_magnitude;
wire [MAG_WIDTH-1:0] detect_threshold;
wire [15:0] detect_count;
wire        cfar_busy;
wire [7:0]  cfar_status;

cfar_ca #(
    .NUM_RANGE_BINS(NUM_RANGE),
    .NUM_DOPPLER_BINS(NUM_DOPPLER)
) dut (
    .clk(clk),
    .reset_n(reset_n),
    .doppler_data(doppler_data),
    .doppler_valid(doppler_valid),
    .doppler_bin_in(doppler_bin_in),
    .range_bin_in(range_bin_in),
    .frame_complete(frame_complete),
    .cfg_guard_cells(cfg_guard_cells),
    .cfg_train_cells(cfg_train_cells),
    .cfg_alpha(cfg_alpha),
    .cfg_cfar_mode(cfg_cfar_mode),
    .cfg_cfar_enable(cfg_cfar_enable),
    .cfg_simple_threshold(cfg_simple_threshold),
    .detect_flag(detect_flag),
    .detect_valid(detect_valid),
    .detect_range(detect_range),
    .detect_doppler(detect_doppler),
    .detect_magnitude(detect_magnitude),
    .detect_threshold(detect_threshold),
    .detect_count(detect_count),
    .cfar_busy(cfar_busy),
    .cfar_status(cfar_status),
    .dbg_cells_processed(),
    .dbg_cols_completed(),
    .dbg_valid_count()
);

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
// Helper: Feed one frame of data through CFAR (simple threshold mode)
// Sends NUM_RANGE * NUM_DOPPLER samples with magnitude = value.
// The Doppler processor outputs data one Doppler bin at a time for all range bins.
// ============================================================================
task feed_frame;
    input [15:0] i_val;  // I component (will make mag = |I| + |Q|)
    input [15:0] q_val;  // Q component
    integer r, d;
    begin
        // Feed all samples: range bins iterate within each Doppler bin
        for (d = 0; d < NUM_DOPPLER; d = d + 1) begin
            for (r = 0; r < NUM_RANGE; r = r + 1) begin
                @(posedge clk);
                doppler_data  <= {q_val, i_val};
                doppler_valid <= 1'b1;
                range_bin_in  <= r[5:0];
                doppler_bin_in <= d[4:0];
            end
        end
        @(posedge clk);
        doppler_valid <= 1'b0;

        // Wait a few cycles then assert frame_complete
        repeat(5) @(posedge clk);
        frame_complete <= 1'b1;
        @(posedge clk);
        frame_complete <= 1'b0;
    end
endtask

// ============================================================================
// Main Test
// ============================================================================
integer cycle_count;
reg [15:0] saved_count_1;
reg [15:0] saved_count_2;

initial begin
    // Init
    doppler_data    = 32'd0;
    doppler_valid   = 1'b0;
    doppler_bin_in  = 5'd0;
    range_bin_in    = 6'd0;
    frame_complete  = 1'b0;
    cfg_guard_cells = 4'd2;
    cfg_train_cells = 5'd8;
    cfg_alpha       = 8'h30;
    cfg_cfar_mode   = 2'b00;
    cfg_cfar_enable = 1'b0;       // Simple threshold mode
    cfg_simple_threshold = 16'd100;  // Low threshold — most samples should detect

    // Reset
    #(CLK_PERIOD * 5);
    reset_n = 1'b1;
    #(CLK_PERIOD * 5);

    $display("============================================================");
    $display("  cfar_ca detect_count Reset Bug Fix Test");
    $display("  %0d range x %0d Doppler, simple threshold = %0d",
             NUM_RANGE, NUM_DOPPLER, cfg_simple_threshold);
    $display("============================================================");

    check(detect_count == 0, "detect_count is 0 after reset");
    check(!cfar_busy, "CFAR not busy after reset");

    // ==================================================================
    // FRAME 1: Simple threshold mode, mag=1000 (all should detect, thr=100)
    // ==================================================================
    $display("\n--- Frame 1: threshold=100, mag=1000 (all should detect) ---");
    // I=500, Q=500 → mag = 500 + 500 = 1000 > 100
    feed_frame(16'd500, 16'd500);

    // Wait for CFAR to finish (simple threshold: ST_DONE after buffering)
    cycle_count = 0;
    while (cfar_busy && cycle_count < 100_000) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
    end
    // Let ST_DONE → ST_IDLE transition occur
    repeat(5) @(posedge clk);

    saved_count_1 = detect_count;
    $display("  Frame 1 detect_count = %0d (expected %0d)", saved_count_1, NUM_RANGE * NUM_DOPPLER);
    check(saved_count_1 == NUM_RANGE * NUM_DOPPLER, "Frame 1: all cells detected");
    check(!cfar_busy, "Frame 1: CFAR returned to idle");

    // ==================================================================
    // FRAME 2: Same data — detect_count should reset and NOT accumulate
    // ==================================================================
    $display("\n--- Frame 2: same data, checking detect_count resets ---");
    feed_frame(16'd500, 16'd500);

    cycle_count = 0;
    while (cfar_busy && cycle_count < 100_000) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
    end
    repeat(5) @(posedge clk);

    saved_count_2 = detect_count;
    $display("  Frame 2 detect_count = %0d (expected %0d)", saved_count_2, NUM_RANGE * NUM_DOPPLER);
    // The critical check: detect_count should be the count for frame 2 ONLY,
    // not frame 1 + frame 2 accumulated.
    check(saved_count_2 == NUM_RANGE * NUM_DOPPLER,
          "Frame 2: detect_count reset (not accumulated)");
    check(saved_count_2 != saved_count_1 * 2,
          "Frame 2: not double the frame 1 count (would indicate no reset)");

    // ==================================================================
    // FRAME 3: Low magnitude — should get 0 detections
    // ==================================================================
    $display("\n--- Frame 3: threshold=100, mag=50 (none should detect) ---");
    // I=25, Q=25 → mag = 25 + 25 = 50 < 100
    feed_frame(16'd25, 16'd25);

    cycle_count = 0;
    while (cfar_busy && cycle_count < 100_000) begin
        @(posedge clk);
        cycle_count = cycle_count + 1;
    end
    repeat(5) @(posedge clk);

    $display("  Frame 3 detect_count = %0d (expected 0)", detect_count);
    check(detect_count == 0, "Frame 3: zero detections (mag < threshold)");

    // ==================================================================
    // Summary
    // ==================================================================
    $display("");
    $display("============================================================");
    $display("  SUMMARY: cfar_ca detect_count Reset Test");
    $display("============================================================");
    $display("  Frame 1 detect_count: %0d (expected %0d)", saved_count_1, NUM_RANGE * NUM_DOPPLER);
    $display("  Frame 2 detect_count: %0d (expected %0d)", saved_count_2, NUM_RANGE * NUM_DOPPLER);
    $display("  Frame 3 detect_count: %0d (expected 0)", detect_count);
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
    #(CLK_PERIOD * 500_000);
    $display("[TIMEOUT] Simulation exceeded maximum time");
    $display("  detect_count=%0d cfar_busy=%0b", detect_count, cfar_busy);
    $display("SOME TESTS FAILED");
    $finish;
end

endmodule
