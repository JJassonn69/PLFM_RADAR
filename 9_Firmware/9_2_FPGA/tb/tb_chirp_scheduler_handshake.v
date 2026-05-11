`timescale 1ns / 1ps
// ============================================================================
// tb_chirp_scheduler_handshake.v — PR-AB.b expanded commit 5 unit TB
//
// Exercises the beam-ready handshake added to chirp_scheduler.v:
//   - S_BEAM_WAIT entered on frame_pulse when host_handshake_enable=1
//   - Exit on any beam_ready_async edge (toggle semantic)
//   - Watchdog auto-advances + sets beam_handshake_watchdog_fired sticky
//   - host_handshake_enable=0 keeps legacy open-loop cadence
//   - Mid-wait disable releases the FSM
//   - Reset clears the sticky
//
// Uses compressed cycle counts so a full 48-chirp frame completes in <1 ms
// of sim time. The production cycle counts (175/161/167 µs PRIs) would push
// the sim past the iverilog regression budget; the FSM logic exercised here
// is independent of the cycle-count values.
// ============================================================================
`include "radar_params.vh"

module tb_chirp_scheduler_handshake;

// ---- Clock (100 MHz, 10 ns) ----
reg clk = 1'b0;
always #5 clk = ~clk;

// ---- Compressed timing — 6 chirp/listen/guard cycles each per PRI ----
localparam [15:0] T_CHIRP  = 16'd6;
localparam [15:0] T_LISTEN = 16'd6;
localparam [15:0] T_GUARD  = 16'd6;
// Single chirp + listen + guard ≈ 20 cycles. With chirps_per_subframe=2 and
// 3 sub-frames active, a frame lands at ~120 cycles → 1.2 µs/frame in sim.
localparam [5:0]  CHIRPS_PER_SUBFRAME = 6'd2;

// ---- DUT signals ----
reg         reset_n          = 1'b0;
reg         mixers_enable    = 1'b0;
reg  [2:0]  subframe_enable  = 3'b111;
reg         beam_ready_async = 1'b0;
reg         handshake_enable = 1'b0;

wire [1:0]  wave_sel;
wire        chirp_pulse;
wire        subframe_pulse;
wire        frame_pulse;
wire [5:0]  chirp_counter;
wire [1:0]  subframe_id;
wire [15:0] cfg_chirp_cycles;
wire [15:0] cfg_listen_cycles;
wire [15:0] cfg_guard_cycles;
wire        watchdog_fired;

chirp_scheduler dut (
    .clk                          (clk),
    .reset_n                      (reset_n),
    .host_subframe_enable         (subframe_enable),
    .host_short_chirp_cycles      (T_CHIRP),
    .host_short_listen_cycles     (T_LISTEN),
    .host_medium_chirp_cycles     (T_CHIRP),
    .host_medium_listen_cycles    (T_LISTEN),
    .host_long_chirp_cycles       (T_CHIRP),
    .host_long_listen_cycles      (T_LISTEN),
    .host_guard_cycles            (T_GUARD),
    .host_chirps_per_subframe     (CHIRPS_PER_SUBFRAME),
    .mixers_enable                (mixers_enable),
    .beam_ready_async             (beam_ready_async),
    .host_handshake_enable        (handshake_enable),
    .wave_sel                     (wave_sel),
    .chirp_pulse                  (chirp_pulse),
    .subframe_pulse               (subframe_pulse),
    .frame_pulse                  (frame_pulse),
    .chirp_counter                (chirp_counter),
    .subframe_id                  (subframe_id),
    .cfg_chirp_cycles             (cfg_chirp_cycles),
    .cfg_listen_cycles            (cfg_listen_cycles),
    .cfg_guard_cycles             (cfg_guard_cycles),
    .beam_handshake_watchdog_fired(watchdog_fired)
);

// ---- Bookkeeping ----
integer pass = 0;
integer fail = 0;
integer frame_pulse_count = 0;
always @(posedge clk) if (frame_pulse) frame_pulse_count = frame_pulse_count + 1;

task check;
    input [255:0] label;
    input cond;
    begin
        if (cond) begin
            $display("  [PASS] %0s", label);
            pass = pass + 1;
        end else begin
            $display("  [FAIL] %0s", label);
            fail = fail + 1;
        end
    end
endtask

// Wait for the FSM to enter S_BEAM_WAIT (state == 3'd5).
task wait_for_beam_wait;
    input integer timeout_cycles;
    integer i;
    begin
        i = 0;
        while (dut.state !== 3'd5 && i < timeout_cycles) begin
            @(posedge clk);
            i = i + 1;
        end
    end
endtask

// Wait for state to leave S_BEAM_WAIT (timeout reports failure to caller).
task wait_for_beam_wait_exit;
    input integer timeout_cycles;
    integer i;
    begin
        i = 0;
        while (dut.state === 3'd5 && i < timeout_cycles) begin
            @(posedge clk);
            i = i + 1;
        end
    end
endtask

// Wait for at least N frames to complete (frame_pulse_count >= target).
task wait_frames;
    input integer target;
    input integer timeout_cycles;
    integer i;
    begin
        i = 0;
        while (frame_pulse_count < target && i < timeout_cycles) begin
            @(posedge clk);
            i = i + 1;
        end
    end
endtask

// =========================================================================
// MAIN
// =========================================================================
initial begin
    $dumpfile("tb_chirp_scheduler_handshake.vcd");
    $dumpvars(0, tb_chirp_scheduler_handshake);

    $display("============================================================");
    $display("  CHIRP_SCHEDULER beam-ready handshake (PR-AB.b expanded c5)");
    $display("============================================================");

    // Reset
    reset_n          = 1'b0;
    mixers_enable    = 1'b0;
    handshake_enable = 1'b0;
    beam_ready_async = 1'b0;
    repeat (4) @(posedge clk);
    reset_n = 1'b1;
    @(posedge clk);
    check("T1: post-reset watchdog sticky low", watchdog_fired == 1'b0);

    // ====================================================================
    // T2: Legacy mode (handshake_enable=0) — frames advance back-to-back,
    //     S_BEAM_WAIT must never be visited.
    // ====================================================================
    $display("--- T2: legacy open-loop (handshake_enable=0) ---");
    mixers_enable = 1'b1;
    frame_pulse_count = 0;
    wait_frames(3, 5000);
    check("T2: at least 3 frames fired without handshake", frame_pulse_count >= 3);
    check("T2: watchdog sticky still low", watchdog_fired == 1'b0);

    // Park the scheduler in IDLE so the next test starts clean.
    mixers_enable = 1'b0;
    repeat (5) @(posedge clk);
    frame_pulse_count = 0;

    // ====================================================================
    // T3: Handshake enabled — scheduler should enter S_BEAM_WAIT after
    //     the next frame_pulse and only exit on a beam_ready edge.
    // ====================================================================
    $display("--- T3: handshake enabled, MCU toggles before watchdog ---");
    handshake_enable = 1'b1;
    mixers_enable    = 1'b1;
    wait_for_beam_wait(5000);
    check("T3a: FSM entered S_BEAM_WAIT after a frame_pulse",
          dut.state == 3'd5);

    // Sit in S_BEAM_WAIT for a deliberate number of cycles; the scheduler
    // must stay parked until we toggle beam_ready_async.
    repeat (200) @(posedge clk);
    check("T3b: FSM still in S_BEAM_WAIT after 200 idle cycles",
          dut.state == 3'd5);
    check("T3b: watchdog has not fired",  watchdog_fired == 1'b0);

    // Toggle beam_ready_async and verify exit within a handful of clk
    // edges (2-FF sync + 1-cycle edge latch + S_BEAM_WAIT → S_CHIRP).
    @(posedge clk); beam_ready_async = 1'b1;
    wait_for_beam_wait_exit(50);
    check("T3c: FSM left S_BEAM_WAIT after PD8 toggle (rising)",
          dut.state != 3'd5);

    // ====================================================================
    // T4: Second toggle exits a later wait (verifies edge-detect handles
    //     falling edges symmetrically — HAL_GPIO_TogglePin gives both).
    // ====================================================================
    $display("--- T4: second wait, falling-edge ack ---");
    wait_for_beam_wait(5000);
    check("T4a: FSM re-entered S_BEAM_WAIT on next frame", dut.state == 3'd5);
    @(posedge clk); beam_ready_async = 1'b0;  // falling edge
    wait_for_beam_wait_exit(50);
    check("T4b: FSM left S_BEAM_WAIT after PD8 toggle (falling)",
          dut.state != 3'd5);

    // ====================================================================
    // T5: Watchdog timeout. The real 23-bit terminal value (8M cycles ≈
    //     80 ms) is unreachable in iverilog sim time, so we force the
    //     FSM's counter to the terminal value and let one clk edge
    //     resolve the (counter >= BEAM_WATCHDOG_MAX) branch.
    // ====================================================================
    $display("--- T5: watchdog auto-advance + sticky latch ---");
    wait_for_beam_wait(5000);
    check("T5a: FSM in S_BEAM_WAIT (pre-force)", dut.state == 3'd5);
    // Force the counter to BEAM_WATCHDOG_MAX so the FSM's >= comparison
    // trips on the next posedge; then release immediately so the always
    // block can drive normally on the same edge.
    @(negedge clk);
    force dut.beam_watchdog = 23'd8_000_000;
    @(posedge clk); #1;
    release dut.beam_watchdog;
    check("T5b: watchdog sticky latched after timeout",
          watchdog_fired == 1'b1);
    check("T5c: FSM left S_BEAM_WAIT after watchdog",
          dut.state != 3'd5);

    // ====================================================================
    // T6: Sticky survives mixers_enable=0 cycle (only reset_n clears it).
    // ====================================================================
    $display("--- T6: sticky watchdog is reset-only ---");
    mixers_enable = 1'b0;
    repeat (20) @(posedge clk);
    check("T6a: watchdog sticky stays high across mixers_enable=0",
          watchdog_fired == 1'b1);
    mixers_enable = 1'b1;

    // ====================================================================
    // T7: Mid-wait disable releases the FSM (handshake_enable→0).
    // ====================================================================
    $display("--- T7: mid-wait host_handshake_enable=0 releases FSM ---");
    wait_for_beam_wait(5000);
    check("T7a: FSM in S_BEAM_WAIT for disable test", dut.state == 3'd5);
    @(posedge clk); handshake_enable = 1'b0;
    wait_for_beam_wait_exit(20);
    check("T7b: FSM left S_BEAM_WAIT after handshake disable",
          dut.state != 3'd5);

    // ====================================================================
    // T8: Full reset clears sticky.
    // ====================================================================
    $display("--- T8: reset_n clears watchdog sticky ---");
    reset_n = 1'b0;
    repeat (4) @(posedge clk);
    check("T8: watchdog sticky cleared by reset_n", watchdog_fired == 1'b0);
    reset_n = 1'b1;
    repeat (4) @(posedge clk);

    $display("============================================================");
    $display("RESULTS: pass=%0d fail=%0d", pass, fail);
    $display("============================================================");
    if (fail == 0) $display("[OVERALL] PASS");
    else           $display("[OVERALL] FAIL");
    $finish;
end

initial begin
    #1_000_000;  // 1 ms wall-clock safety
    $display("[FATAL] timeout");
    $finish;
end

endmodule
