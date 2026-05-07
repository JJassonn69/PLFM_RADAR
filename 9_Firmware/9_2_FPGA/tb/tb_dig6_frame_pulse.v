// ============================================================================
// tb_dig6_frame_pulse.v
//
// PR-AB.b: gpio_dig6 (PD14) carries the chirp_scheduler frame_pulse, stretched
// to ~100 ns so the STM32 EXTI on PD14 can latch the rising edge reliably.
// The MCU dwell loop (runRadarPulseSequence) replaces
// HAL_Delay(BEAM_PATTERN_DWELL_MS) with osSemaphoreAcquire so per-pattern
// dwell tracks the actual ladder length — drift-free, mask-aware.
//
// Companion to tb_status_words_stickies.v which covers the gpio_dig7 fault-OR
// (watchdog | CDC overrun) semantic and the status_words[5][6:5] CDC packing.
//
// This TB mirrors the production stretcher fragment from radar_system_top.v
// and asserts:
//
//   T1  Reset → count=0, dig6=0
//   T2  Single 1-cycle pulse → dig6 high for exactly 10 cycles, low on 11
//   T3  Pulse during stretch → counter reloads to 10 (longer total high time)
//   T4  Two pulses spaced > 10 cycles → two clean rising edges
//   T5  Pulse asserted continuously for many cycles → counter pinned high
//   T6  No pulse activity → dig6 stays low forever
//   T7  Reset mid-stretch → counter and dig6 drop immediately
// ============================================================================
`timescale 1ns/1ps

module tb_dig6_frame_pulse;

    reg clk_100m   = 1'b0;
    reg sys_reset_n = 1'b0;
    reg frame_pulse = 1'b0;
    wire dig6;

    frame_pulse_stretcher_block dut (
        .clk_100m_buf  (clk_100m),
        .sys_reset_n   (sys_reset_n),
        .frame_pulse_in(frame_pulse),
        .gpio_dig6     (dig6)
    );

    // 100 MHz clock (10 ns period)
    always #5 clk_100m = ~clk_100m;

    integer pass = 0;
    integer fail = 0;

    task check (input [127:0] label, input expected);
        begin
            #1;
            if (dig6 === expected) begin
                $display("  [PASS] %0s: dig6=%b", label, dig6);
                pass = pass + 1;
            end else begin
                $display("  [FAIL] %0s: dig6=%b (exp %b)", label, dig6, expected);
                fail = fail + 1;
            end
        end
    endtask

    task fire_pulse;
        begin
            @(posedge clk_100m); #1;
            frame_pulse = 1'b1;
            @(posedge clk_100m); #1;
            frame_pulse = 1'b0;
        end
    endtask

    integer i;

    initial begin
        $display("============================================================");
        $display("PR-AB.b: gpio_dig6 = stretched(frame_pulse)");
        $display("============================================================");

        // ---- T1: reset state ----
        sys_reset_n = 1'b0;
        repeat (4) @(posedge clk_100m);
        check("T1 reset asserted, dig6 low", 1'b0);
        @(posedge clk_100m); #1;
        sys_reset_n = 1'b1;
        @(posedge clk_100m); #1;
        check("T1b after deassert, dig6 still low", 1'b0);

        // ---- T2: single pulse → exactly 10 cycles high ----
        fire_pulse();
        // After fire_pulse, frame_pulse was high for 1 clk; the always block
        // sampled it on that posedge and loaded count=10. dig6 is high while
        // count != 0. Count decrements each cycle: 10,9,...,1,0. So dig6 is
        // high for 10 posedges starting on the cycle the load took effect.
        for (i = 0; i < 10; i = i + 1) begin
            #1;
            if (dig6 !== 1'b1) begin
                $display("  [FAIL] T2 cycle %0d: dig6=%b (exp 1)", i, dig6);
                fail = fail + 1;
            end
            @(posedge clk_100m);
        end
        // 11th cycle: count has wrapped to 0, dig6 should be low.
        #1;
        check("T2 dig6 low on 11th cycle", 1'b0);
        // Make sure it stays low.
        repeat (5) @(posedge clk_100m);
        check("T2 dig6 stays low after stretch", 1'b0);

        // ---- T3: pulse during stretch reloads counter ----
        fire_pulse();
        @(posedge clk_100m); #1;  // count = 9
        @(posedge clk_100m); #1;  // count = 8
        @(posedge clk_100m); #1;  // count = 7
        // Reload by firing another pulse mid-stretch.
        @(posedge clk_100m); #1;
        frame_pulse = 1'b1;
        @(posedge clk_100m); #1;
        frame_pulse = 1'b0;
        // Counter is now reloaded to 10. dig6 stays high for another 10 cycles.
        for (i = 0; i < 10; i = i + 1) begin
            #1;
            if (dig6 !== 1'b1) begin
                $display("  [FAIL] T3 reload cycle %0d: dig6=%b (exp 1)", i, dig6);
                fail = fail + 1;
            end
            @(posedge clk_100m);
        end
        #1;
        check("T3 dig6 low after reloaded stretch", 1'b0);

        // ---- T4: spaced pulses produce two distinct rising edges ----
        fire_pulse();
        repeat (15) @(posedge clk_100m);  // > 10 cycles, dig6 must be 0
        check("T4a dig6 low between pulses", 1'b0);
        fire_pulse();
        #1;
        check("T4b dig6 high after second pulse", 1'b1);
        repeat (10) @(posedge clk_100m);
        #1;
        check("T4c dig6 low after second stretch", 1'b0);

        // ---- T5: continuous pulse pins counter at 10 ----
        @(posedge clk_100m); #1;
        frame_pulse = 1'b1;
        repeat (50) @(posedge clk_100m);
        #1;
        check("T5 dig6 high under continuous pulse", 1'b1);
        @(posedge clk_100m); #1;
        frame_pulse = 1'b0;
        // After deassert, dig6 should drain over 10 cycles.
        repeat (10) @(posedge clk_100m);
        #1;
        check("T5 dig6 drains after pulse deassert", 1'b0);

        // ---- T6: idle line stays low ----
        repeat (200) @(posedge clk_100m);
        check("T6 dig6 idle stays low", 1'b0);

        // ---- T7: reset mid-stretch drops dig6 immediately ----
        fire_pulse();
        @(posedge clk_100m); #1;
        @(posedge clk_100m); #1;
        check("T7a dig6 high pre-reset", 1'b1);
        sys_reset_n = 1'b0;
        @(posedge clk_100m); #1;
        check("T7b reset drops dig6", 1'b0);
        sys_reset_n = 1'b1;
        @(posedge clk_100m); #1;

        $display("============================================================");
        $display("DIG6 FRAME PULSE RESULTS: pass=%0d fail=%0d", pass, fail);
        $display("============================================================");
        if (fail == 0) $display("[OVERALL] PASS");
        else           $display("[OVERALL] FAIL");
        $finish;
    end

    initial begin
        #1_000_000;
        $display("[FATAL] timeout");
        $finish;
    end

endmodule

// ============================================================================
// frame_pulse_stretcher_block — mirrors the production stretcher in
// radar_system_top.v (PR-AB.b). 1-cycle frame_pulse_in loads count=10;
// counter decrements each clock; gpio_dig6 = (count != 0). reset_n async
// clear. Reloads on subsequent pulses. Result: dig6 high for 10 clk_100m
// cycles = 100 ns starting on the cycle after the input pulse is sampled.
// ============================================================================
module frame_pulse_stretcher_block (
    input  wire clk_100m_buf,
    input  wire sys_reset_n,
    input  wire frame_pulse_in,
    output wire gpio_dig6
);
    reg [3:0] frame_pulse_stretch_count;
    always @(posedge clk_100m_buf or negedge sys_reset_n) begin
        if (!sys_reset_n)
            frame_pulse_stretch_count <= 4'd0;
        else if (frame_pulse_in)
            frame_pulse_stretch_count <= 4'd10;
        else if (frame_pulse_stretch_count != 4'd0)
            frame_pulse_stretch_count <= frame_pulse_stretch_count - 4'd1;
    end
    assign gpio_dig6 = (frame_pulse_stretch_count != 4'd0);
endmodule
