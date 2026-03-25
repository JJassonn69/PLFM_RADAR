`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// tb_cfar_physics.v - Physics-aware CFAR detector testbench (AERIS-10)
//
// Simulation command:
//   iverilog -DSIMULATION -o tb_cfar_physics.vvp tb/physics/tb_cfar_physics.v cfar_ca.v && vvp tb_cfar_physics.vvp
//
// Test coverage:
//   C1: Known target detection (CA-CFAR)
//   C2: Noise-only false alarm rate
//   C3: Clutter edge behavior (CA vs SO)
//   C4: CFAR disabled simple-threshold pass-through
//   C5: Edge-bin target behavior
//   C6: Multiple targets in same Doppler column
//   C7: Alpha sensitivity
//////////////////////////////////////////////////////////////////////////////

module tb_cfar_physics;

localparam NUM_RANGE_BINS   = 64;
localparam NUM_DOPPLER_BINS = 32;
localparam MAG_WIDTH        = 17;
localparam ALPHA_WIDTH      = 8;
localparam TOTAL_CELLS      = NUM_RANGE_BINS * NUM_DOPPLER_BINS;

localparam [5:0] C1_TGT_RANGE = 6'd20;
localparam [4:0] C1_TGT_DOP   = 5'd13;

localparam [5:0] C3_TGT_RANGE = 6'd33;
localparam [4:0] C3_TGT_DOP   = 5'd7;

localparam [4:0] C5_TGT_DOP   = 5'd11;
localparam [4:0] C6_TGT_DOP   = 5'd9;

// ==========================================================================
// CLOCK / RESET
// ==========================================================================
reg clk;
reg reset_n;

initial clk = 1'b0;
always #5 clk = ~clk;  // 100 MHz

// ==========================================================================
// TEST INFRASTRUCTURE
// ==========================================================================
integer pass_count;
integer fail_count;
integer test_num;

task check;
    input condition;
    input [256*8-1:0] msg;
    begin
        if (condition) begin
            $display("[PASS] C%0d: %0s", test_num, msg);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] C%0d: %0s", test_num, msg);
            fail_count = fail_count + 1;
        end
    end
endtask

// ==========================================================================
// DUT I/O
// ==========================================================================
reg [31:0] doppler_data;
reg        doppler_valid;
reg [4:0]  doppler_bin_in;
reg [5:0]  range_bin_in;
reg        frame_complete;

reg [3:0]  cfg_guard_cells;
reg [4:0]  cfg_train_cells;
reg [ALPHA_WIDTH-1:0] cfg_alpha;
reg [1:0]  cfg_cfar_mode;
reg        cfg_cfar_enable;
reg [15:0] cfg_simple_threshold;

wire       detect_flag;
wire       detect_valid;
wire [5:0] detect_range;
wire [4:0] detect_doppler;
wire [MAG_WIDTH-1:0] detect_magnitude;
wire [MAG_WIDTH-1:0] detect_threshold;
wire [15:0] detect_count;
wire       cfar_busy;
wire [7:0] cfar_status;

cfar_ca dut (
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
    .cfar_status(cfar_status)
);

// ==========================================================================
// CAPTURE / SCOREBOARD
// ==========================================================================
reg capture_enable;
reg detected_map [0:NUM_RANGE_BINS-1][0:NUM_DOPPLER_BINS-1];
integer cap_detect_valid_count;
integer cap_detect_flag_count;

always @(posedge clk) begin
    if (capture_enable && detect_valid) begin
        cap_detect_valid_count <= cap_detect_valid_count + 1;
        if (detect_flag) begin
            cap_detect_flag_count <= cap_detect_flag_count + 1;
            detected_map[detect_range][detect_doppler] <= 1'b1;
        end
    end
end

task clear_scoreboard;
    integer r;
    integer d;
    begin
        cap_detect_valid_count = 0;
        cap_detect_flag_count = 0;
        for (r = 0; r < NUM_RANGE_BINS; r = r + 1) begin
            for (d = 0; d < NUM_DOPPLER_BINS; d = d + 1)
                detected_map[r][d] = 1'b0;
        end
    end
endtask

task reset_dut;
    begin
        reset_n = 1'b0;
        doppler_data = 32'd0;
        doppler_valid = 1'b0;
        doppler_bin_in = 5'd0;
        range_bin_in = 6'd0;
        frame_complete = 1'b0;
        capture_enable = 1'b0;
        clear_scoreboard;
        repeat (12) @(posedge clk);
        reset_n = 1'b1;
        repeat (6) @(posedge clk);
    end
endtask

task set_cfar_cfg;
    input en;
    input [1:0] mode;
    input [3:0] guard_cells;
    input [4:0] train_cells;
    input [7:0] alpha_q44;
    input [15:0] simple_thr;
    begin
        cfg_cfar_enable = en;
        cfg_cfar_mode = mode;
        cfg_guard_cells = guard_cells;
        cfg_train_cells = train_cells;
        cfg_alpha = alpha_q44;
        cfg_simple_threshold = simple_thr;
    end
endtask

function [15:0] pattern_mag;
    input integer pattern_id;
    input [5:0] rbin;
    input [4:0] dbin;
    begin
        pattern_mag = 16'd100;
        case (pattern_id)
            0: begin
                pattern_mag = 16'd100;
            end

            // C1/C7 baseline: one strong target
            1: begin
                if (rbin == C1_TGT_RANGE && dbin == C1_TGT_DOP)
                    pattern_mag = 16'd5000;
            end

            // C3 clutter edge: bins 0..31 clutter, 32..63 noise, target at 33
            2: begin
                if (rbin <= 6'd31)
                    pattern_mag = 16'd3000;
                else
                    pattern_mag = 16'd100;

                if (rbin == C3_TGT_RANGE && dbin == C3_TGT_DOP)
                    pattern_mag = 16'd3000;
            end

            // C4 pass-through threshold checks
            3: begin
                pattern_mag = 16'd100;
                if (rbin == 6'd5  && dbin == 5'd3)  pattern_mag = 16'd2500; // above
                if (rbin == 6'd40 && dbin == 5'd20) pattern_mag = 16'd3000; // above
                if (rbin == 6'd6  && dbin == 5'd3)  pattern_mag = 16'd1500; // below
                if (rbin == 6'd41 && dbin == 5'd20) pattern_mag = 16'd1999; // below
            end

            // C5 edge-bin targets
            4: begin
                if (rbin == 6'd0  && dbin == C5_TGT_DOP) pattern_mag = 16'd5000;
                if (rbin == 6'd63 && dbin == C5_TGT_DOP) pattern_mag = 16'd5000;
            end

            // C6 multiple targets in one Doppler column
            5: begin
                if (dbin == C6_TGT_DOP) begin
                    if (rbin == 6'd10 || rbin == 6'd30 || rbin == 6'd50)
                        pattern_mag = 16'd5000;
                end
            end

            default: begin
                pattern_mag = 16'd100;
            end
        endcase
    end
endfunction

task feed_frame;
    input integer pattern_id;
    integer d;
    integer r;
    reg [15:0] mag_i;
    begin
        for (d = 0; d < NUM_DOPPLER_BINS; d = d + 1) begin
            for (r = 0; r < NUM_RANGE_BINS; r = r + 1) begin
                mag_i = pattern_mag(pattern_id, r[5:0], d[4:0]);
                @(posedge clk);
                doppler_valid  <= 1'b1;
                doppler_bin_in <= d[4:0];
                range_bin_in   <= r[5:0];
                doppler_data   <= {16'd0, mag_i};
                frame_complete <= 1'b0;
            end
        end

        @(posedge clk);
        doppler_valid  <= 1'b0;
        frame_complete <= 1'b1;

        @(posedge clk);
        frame_complete <= 1'b0;
    end
endtask

task run_frame_and_wait;
    input integer pattern_id;
    integer guard_cycles;
    begin
        feed_frame(pattern_id);

        // Wait for frame processing to complete
        guard_cycles = 0;
        while (cfar_busy && guard_cycles < 200000) begin
            @(posedge clk);
            guard_cycles = guard_cycles + 1;
        end

        // Small drain period for last valid pulse visibility
        repeat (6) @(posedge clk);
    end
endtask

// ==========================================================================
// MAIN TEST SEQUENCE
// ==========================================================================
initial begin
    $dumpfile("tb_cfar_physics.vcd");
    $dumpvars(0, tb_cfar_physics);

    pass_count = 0;
    fail_count = 0;

    // Default drives
    reset_n = 1'b0;
    doppler_data = 32'd0;
    doppler_valid = 1'b0;
    doppler_bin_in = 5'd0;
    range_bin_in = 6'd0;
    frame_complete = 1'b0;
    cfg_guard_cells = 4'd2;
    cfg_train_cells = 5'd8;
    cfg_alpha = 8'h30;
    cfg_cfar_mode = 2'b00;
    cfg_cfar_enable = 1'b1;
    cfg_simple_threshold = 16'd2000;
    capture_enable = 1'b0;

    $display("");
    $display("==========================================================");
    $display("  AERIS-10 CFAR Physics Testbench");
    $display("  Range bins=%0d Doppler bins=%0d", NUM_RANGE_BINS, NUM_DOPPLER_BINS);
    $display("==========================================================");

    // ----------------------------------------------------------------------
    // C1: Known target detection (CA-CFAR)
    // ----------------------------------------------------------------------
    test_num = 1;
    $display("");
    $display("--- C1: Known target detection (CA-CFAR) ---");

    reset_dut;
    set_cfar_cfg(1'b1, 2'b00, 4'd2, 5'd8, 8'h30, 16'd2000);
    clear_scoreboard;
    capture_enable = 1'b1;
    run_frame_and_wait(1);
    capture_enable = 1'b0;

    check(detected_map[C1_TGT_RANGE][C1_TGT_DOP],
          "Strong target is detected at expected range/doppler cell");
    check(cap_detect_flag_count >= 1,
          "At least one detection exists for strong-target scenario");

    // ----------------------------------------------------------------------
    // C2: Noise-only false alarm rate
    // ----------------------------------------------------------------------
    test_num = 2;
    $display("");
    $display("--- C2: Noise-only false alarm rate ---");

    reset_dut;
    set_cfar_cfg(1'b1, 2'b00, 4'd2, 5'd8, 8'h30, 16'd2000);
    clear_scoreboard;
    capture_enable = 1'b1;
    run_frame_and_wait(0);
    capture_enable = 1'b0;

    check(cap_detect_flag_count == 0,
          "Uniform noise floor yields zero CFAR detections");
    check(cap_detect_flag_count < ((TOTAL_CELLS * 5) / 100),
          "False alarms remain below 5% of frame cells");

    // ----------------------------------------------------------------------
    // C3: GO/SO clutter-edge handling focus (verify CA miss and SO hit)
    // ----------------------------------------------------------------------
    test_num = 3;
    $display("");
    $display("--- C3: Clutter edge behavior (CA vs SO) ---");

    begin : c3_run
        reg c3_ca_hit;
        reg c3_so_hit;

        // CA-CFAR run
        reset_dut;
        set_cfar_cfg(1'b1, 2'b00, 4'd2, 5'd8, 8'h30, 16'd2000);
        clear_scoreboard;
        capture_enable = 1'b1;
        run_frame_and_wait(2);
        capture_enable = 1'b0;
        c3_ca_hit = detected_map[C3_TGT_RANGE][C3_TGT_DOP];

        // SO-CFAR run
        reset_dut;
        set_cfar_cfg(1'b1, 2'b10, 4'd2, 5'd8, 8'h30, 16'd2000);
        clear_scoreboard;
        capture_enable = 1'b1;
        run_frame_and_wait(2);
        capture_enable = 1'b0;
        c3_so_hit = detected_map[C3_TGT_RANGE][C3_TGT_DOP];

        check(!c3_ca_hit,
              "CA-CFAR does not detect clutter-edge target at range 33");
        check(c3_so_hit,
              "SO-CFAR detects clutter-edge target at range 33");
    end

    // ----------------------------------------------------------------------
    // C4: CFAR disabled simple-threshold pass-through
    // ----------------------------------------------------------------------
    test_num = 4;
    $display("");
    $display("--- C4: CFAR disable pass-through ---");

    reset_dut;
    set_cfar_cfg(1'b0, 2'b00, 4'd2, 5'd8, 8'h30, 16'd2000);
    clear_scoreboard;
    capture_enable = 1'b1;
    run_frame_and_wait(3);
    capture_enable = 1'b0;

    check(detected_map[6'd5][5'd3] && detected_map[6'd40][5'd20],
          "Samples above simple threshold assert detect_flag in buffer phase");
    check(!detected_map[6'd6][5'd3] && !detected_map[6'd41][5'd20],
          "Samples below simple threshold do not assert detect_flag");

    // ----------------------------------------------------------------------
    // C5: Edge bin behavior
    // ----------------------------------------------------------------------
    test_num = 5;
    $display("");
    $display("--- C5: Edge-bin target detection ---");

    reset_dut;
    set_cfar_cfg(1'b1, 2'b00, 4'd2, 5'd8, 8'h30, 16'd2000);
    clear_scoreboard;
    capture_enable = 1'b1;
    run_frame_and_wait(4);
    capture_enable = 1'b0;

    check(detected_map[6'd0][C5_TGT_DOP],
          "Target at first range bin (0) is detected");
    check(detected_map[6'd63][C5_TGT_DOP],
          "Target at last range bin (63) is detected");

    // ----------------------------------------------------------------------
    // C6: Multiple targets in same Doppler column
    // ----------------------------------------------------------------------
    test_num = 6;
    $display("");
    $display("--- C6: Multiple targets in one Doppler column ---");

    reset_dut;
    set_cfar_cfg(1'b1, 2'b00, 4'd2, 5'd8, 8'h30, 16'd2000);
    clear_scoreboard;
    capture_enable = 1'b1;
    run_frame_and_wait(5);
    capture_enable = 1'b0;

    check(detected_map[6'd10][C6_TGT_DOP] && detected_map[6'd30][C6_TGT_DOP] && detected_map[6'd50][C6_TGT_DOP],
          "All three targets (10,30,50) are detected in same Doppler column");
    check(cap_detect_flag_count >= 3,
          "Total detections include at least three target hits");

    // ----------------------------------------------------------------------
    // C7: Alpha sensitivity
    // ----------------------------------------------------------------------
    test_num = 7;
    $display("");
    $display("--- C7: Alpha sensitivity ---");

    begin : c7_run
        reg low_alpha_hit;
        reg high_alpha_hit;

        // Lower alpha -> lower threshold -> should detect
        reset_dut;
        set_cfar_cfg(1'b1, 2'b00, 4'd2, 5'd8, 8'h10, 16'd2000);
        clear_scoreboard;
        capture_enable = 1'b1;
        run_frame_and_wait(1);
        capture_enable = 1'b0;
        low_alpha_hit = detected_map[C1_TGT_RANGE][C1_TGT_DOP];

        // Very high alpha -> very high threshold -> should not detect
        reset_dut;
        set_cfar_cfg(1'b1, 2'b00, 4'd2, 5'd8, 8'hFF, 16'd2000);
        clear_scoreboard;
        capture_enable = 1'b1;
        run_frame_and_wait(1);
        capture_enable = 1'b0;
        high_alpha_hit = detected_map[C1_TGT_RANGE][C1_TGT_DOP];

        check(low_alpha_hit,
              "alpha=0x10 (1.0) detects strong target");
        check(!high_alpha_hit,
              "alpha=0xFF (15.9375) suppresses target detection");
    end

    // ----------------------------------------------------------------------
    // Summary
    // ----------------------------------------------------------------------
    $display("");
    $display("==========================================================");
    $display("  CFAR PHYSICS TESTBENCH RESULTS");
    $display("  Passed: %0d  Failed: %0d  Total: %0d",
             pass_count, fail_count, pass_count + fail_count);
    $display("==========================================================");

    if (fail_count > 0)
        $display("  >>> FAILURES DETECTED <<<");
    else
        $display("  All CFAR physics tests passed.");

    $display("");
    $finish;
end

// Timeout watchdog
initial begin
    #50_000_000;  // 50 ms
    $display("[FAIL] C0: TIMEOUT - Simulation exceeded 50ms");
    $finish;
end

endmodule
