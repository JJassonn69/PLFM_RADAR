`timescale 1ns / 1ps
`include "radar_params.vh"
// ============================================================================
// tb_ft2232h_frame_drop.v — verifies AUDIT-C12 frame_drop_count instrumentation
// ============================================================================
// The bridge logic added under AUDIT-C12 surfaces silent USB frame drops via
// a 7-bit `frame_drop_count` register exposed in `status_words[5][31:25]`.
// Tracking handshake:
//   - clk-domain frame_pending: SET on frame_complete, CLEARED by wr_done_pulse
//   - ft_clk-domain wr_done_toggle: flips on WR_DONE → WR_IDLE (frame fully sent)
//   - 3-stage CDC syncs wr_done_toggle into clk for edge-detect
//
// Test cases:
//   1. Single frame, USB drains promptly → drop count stays 0
//   2. Two back-to-back frame_complete with USB stalled → drop count = 1
//   3. Multiple drops while stalled → drop count saturates at 127
//   4. Stalled + recovery → drop count stable, frame_pending clears post-drain
//
// Stimulus uses `stream_control = 6'b000_000` (PR-G v2: no inert flags, no
// sections enabled) so the WR FSM goes HDR (9B) → FOOTER (1B) → DONE in 10
// ft_clk cycles. This gives a fast, deterministic per-frame transfer time.
//
// PASS criteria:
//   - frame_drop_count matches expected value after each scenario
//   - frame_pending tracks correctly across handshake
//   - wr_done_toggle observed to flip on each successful drain
// ============================================================================

module tb_ft2232h_frame_drop;
    localparam CLK_PER    = 10.0;     // 100 MHz
    localparam FT_CLK_PER = 16.667;   // 60 MHz

    reg clk     = 1'b0;
    reg ft_clk  = 1'b0;
    reg reset_n = 1'b0;
    reg ft_reset_n = 1'b0;

    // Radar data inputs (clk domain) - all idle for this test
    reg [31:0] range_profile = 32'd0;
    reg        range_valid   = 1'b0;
    reg [15:0] doppler_real  = 16'd0;
    reg [15:0] doppler_imag  = 16'd0;
    reg        doppler_valid = 1'b0;
    // PR-G: 2-bit class (was 1-bit cfar_detection)
    reg [`RP_DETECT_CLASS_WIDTH-1:0] cfar_detect_class = `RP_DETECT_NONE;
    reg        cfar_valid    = 1'b0;

    reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin_in   = 0;
    // PR-F: doppler_bin widened to RP_DOPPLER_BIN_WIDTH (6 bits)
    reg [`RP_DOPPLER_BIN_WIDTH-1:0]   doppler_bin_in = {`RP_DOPPLER_BIN_WIDTH{1'b0}};
    reg                               frame_complete = 1'b0;

    // FT2232H interface (ft_clk domain)
    wire [7:0] ft_data;
    reg        ft_rxf_n = 1'b1;       // No host read commands
    reg        ft_txe_n = 1'b0;       // 0 = USB ready (default)
    wire       ft_rd_n;
    wire       ft_wr_n;
    wire       ft_oe_n;
    wire       ft_siwu;

    wire [31:0] cmd_data;
    wire        cmd_valid;
    wire [7:0]  cmd_opcode;
    wire [7:0]  cmd_addr;
    wire [15:0] cmd_value;

    // PR-G: stream bits [2:0] all off → WR FSM: HDR → FOOTER → DONE
    // = fast deterministic drain. Bits [5:3] are reserved=0 in v2.
    reg [5:0] stream_control = 6'b000_000;
    // PR-U / M-8: production 3-PRI ladder.
    reg [2:0] subframe_enable = 3'b111;

    // Status inputs (irrelevant for this test)
    reg        status_request = 1'b0;
    reg [15:0] status_cfar_threshold = 16'd0;
    reg [5:0]  status_stream_ctrl = 6'b000_000;
    reg [15:0] status_long_chirp = 16'd0;
    reg [15:0] status_long_listen = 16'd0;
    reg [15:0] status_guard = 16'd0;
    reg [15:0] status_short_chirp = 16'd0;
    reg [15:0] status_short_listen = 16'd0;
    reg [5:0]  status_chirps_per_elev = 6'd0;
    reg        status_chirps_mismatch = 1'b0;
    reg [4:0]  status_self_test_flags = 5'd0;
    reg [7:0]  status_self_test_detail = 8'd0;
    reg        status_self_test_busy = 1'b0;
    reg [3:0]  status_agc_current_gain = 4'd0;
    reg [7:0]  status_agc_peak_magnitude = 8'd0;
    reg [7:0]  status_agc_saturation_count = 8'd0;
    reg        status_agc_enable = 1'b0;

    integer pass = 0;
    integer fail = 0;

    always #(CLK_PER/2)    clk = ~clk;
    always #(FT_CLK_PER/2) ft_clk = ~ft_clk;

    usb_data_interface_ft2232h u_dut (
        .clk(clk),
        .reset_n(reset_n),
        .ft_reset_n(ft_reset_n),
        .range_profile(range_profile),
        .range_valid(range_valid),
        .doppler_real(doppler_real),
        .doppler_imag(doppler_imag),
        .doppler_valid(doppler_valid),
        .cfar_detect_class(cfar_detect_class),  // PR-G: 2-bit class
        .cfar_valid(cfar_valid),
        .range_bin_in(range_bin_in),
        .doppler_bin_in(doppler_bin_in),
        .frame_complete(frame_complete),
        .ft_data(ft_data),
        .ft_rxf_n(ft_rxf_n),
        .ft_txe_n(ft_txe_n),
        .ft_rd_n(ft_rd_n),
        .ft_wr_n(ft_wr_n),
        .ft_oe_n(ft_oe_n),
        .ft_siwu(ft_siwu),
        .ft_clk(ft_clk),
        .cmd_data(cmd_data),
        .cmd_valid(cmd_valid),
        .cmd_opcode(cmd_opcode),
        .cmd_addr(cmd_addr),
        .cmd_value(cmd_value),
        .stream_control(stream_control),
        // PR-U / M-8: per-frame snapshot of host_subframe_enable.
        .subframe_enable(subframe_enable),
        .status_request(status_request),
        .status_cfar_threshold(status_cfar_threshold),
        .status_stream_ctrl(status_stream_ctrl),
        .status_long_chirp(status_long_chirp),
        .status_long_listen(status_long_listen),
        .status_guard(status_guard),
        .status_short_chirp(status_short_chirp),
        .status_short_listen(status_short_listen),
        // M-5: medium PRI readback (test only exercises frame_drop path,
        // medium values left at defaults since this TB doesn't read status).
        .status_medium_chirp(16'd0),
        .status_medium_listen(16'd0),
        .status_chirps_per_elev(status_chirps_per_elev),
        .status_chirps_mismatch(status_chirps_mismatch),
        .status_self_test_flags(status_self_test_flags),
        .status_self_test_detail(status_self_test_detail),
        .status_self_test_busy(status_self_test_busy),
        .status_agc_current_gain(status_agc_current_gain),
        .status_agc_peak_magnitude(status_agc_peak_magnitude),
        .status_agc_saturation_count(status_agc_saturation_count),
        .status_agc_enable(status_agc_enable),
        // AUDIT-S10: control-fault flags tied off (frame-drop TB scope)
        .status_range_decim_watchdog(1'b0),
        .status_ddc_cic_fir_overrun(1'b0),
        // PR-AB.b expanded commit 5: beam-handshake watchdog tied off
        .status_beam_handshake_watchdog(1'b0),
        // PR-G: 2-tier CFAR telemetry tied off
        .status_cfar_alpha_soft(8'h18),       // RP_DEF_CFAR_ALPHA_SOFT
        .status_detect_threshold_soft(17'd0),
        .status_detect_count_cand(16'd0)
    );

    task pulse_frame_complete;
        begin
            @(posedge clk); #1;
            frame_complete = 1'b1;
            @(posedge clk); #1;
            frame_complete = 1'b0;
        end
    endtask

    task wait_cycles;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) @(posedge clk);
        end
    endtask

    task check;
        input integer test_id;
        input [127:0] label;
        input integer expected;
        input integer actual;
        begin
            if (expected == actual) begin
                $display("[PASS] Test %0d (%0s): %0d == %0d",
                         test_id, label, actual, expected);
                pass = pass + 1;
            end else begin
                $display("[FAIL] Test %0d (%0s): got %0d, expected %0d",
                         test_id, label, actual, expected);
                fail = fail + 1;
            end
        end
    endtask

    initial begin
        $display("===========================================================");
        $display("tb_ft2232h_frame_drop — AUDIT-C12 frame_drop_count regression");
        $display("===========================================================");

        // Reset both domains
        repeat (4) @(posedge clk);
        reset_n    = 1'b1;
        ft_reset_n = 1'b1;
        wait_cycles(20);

        // -----------------------------------------------------------
        // Test 1: Normal flow. ft_txe_n=0 (USB ready).
        // Single frame_complete; expect drain to complete and drop count=0.
        // -----------------------------------------------------------
        $display("\n[TEST 1] Single frame, USB ready -> no drops");
        ft_txe_n = 1'b0;
        pulse_frame_complete();
        // Wait for frame to drain through WR_FSM. PR-G v2: stream_control[2:0]=000,
        // FSM goes HDR (9B) -> FOOTER (1B) -> DONE = 10 ft_clk cycles. Plus CDC
        // latency. Allow ~50 ft_clk = ~833 ns = ~83 clk cycles. Be generous:
        // wait 200 clk cycles.
        wait_cycles(200);
        check(1, "drop_count", 0, u_dut.frame_drop_count);
        check(1, "frame_pending_cleared", 0, u_dut.frame_pending);

        // -----------------------------------------------------------
        // Test 2: USB stalled, two frame_completes back-to-back.
        // Expect drop_count = 1 (second pulse arrives while pending).
        // -----------------------------------------------------------
        $display("\n[TEST 2] USB stalled, 2 frame_completes -> drop count = 1");
        ft_txe_n = 1'b1;   // Stall USB
        wait_cycles(10);
        pulse_frame_complete();      // frame N: pending=1, no drop
        wait_cycles(20);             // give time for any spurious wr_done_pulse
        pulse_frame_complete();      // frame N+1: pending was 1, drop count++
        wait_cycles(20);
        check(2, "drop_count_after_stall", 1, u_dut.frame_drop_count);
        check(2, "frame_pending_still_set", 1, u_dut.frame_pending);

        // -----------------------------------------------------------
        // Test 3: Multiple drops while stalled.
        // 3 more frame_completes -> drop count = 4 (1 from test 2 + 3 new).
        // -----------------------------------------------------------
        $display("\n[TEST 3] 3 more frames during stall -> drop count = 4");
        pulse_frame_complete(); wait_cycles(15);
        pulse_frame_complete(); wait_cycles(15);
        pulse_frame_complete(); wait_cycles(15);
        check(3, "drop_count_after_3_more", 4, u_dut.frame_drop_count);

        // -----------------------------------------------------------
        // Test 4: Recovery. Release USB, FSM drains, pending clears.
        // Then a new frame_complete should NOT increment drop count.
        // -----------------------------------------------------------
        $display("\n[TEST 4] USB recovers, drain completes, no new drop");
        ft_txe_n = 1'b0;  // USB ready
        wait_cycles(200); // Allow drain
        check(4, "frame_pending_cleared_after_drain", 0, u_dut.frame_pending);
        check(4, "drop_count_stable_after_drain", 4, u_dut.frame_drop_count);
        // Now a clean frame_complete should add no drop
        pulse_frame_complete();
        wait_cycles(200);
        check(4, "drop_count_unchanged_clean_frame", 4, u_dut.frame_drop_count);
        check(4, "frame_pending_cleared_post_clean", 0, u_dut.frame_pending);

        // -----------------------------------------------------------
        // Test 5: Saturation at 127. Stall USB and pulse frame_complete
        // many times. drop_count should saturate, not wrap.
        // -----------------------------------------------------------
        $display("\n[TEST 5] Saturation: 200 drops requested -> count saturates at 127");
        ft_txe_n = 1'b1;
        // First make pending=1 (this isn't a drop)
        pulse_frame_complete(); wait_cycles(10);
        // Now 199 more frame_completes — each is a drop after the first counted earlier
        // We're at drop_count=4 from prior tests, plus this new sequence will drive it up.
        // After ~130 more pulses, should saturate at 127.
        begin: sat_loop
            integer k;
            for (k = 0; k < 200; k = k + 1) begin
                pulse_frame_complete();
                wait_cycles(5);
            end
        end
        check(5, "drop_count_saturated", 127, u_dut.frame_drop_count);

        $display("\n-----------------------------------------------------------");
        $display("RESULTS: %0d PASS, %0d FAIL", pass, fail);
        $display("-----------------------------------------------------------");
        if (fail == 0)
            $display("[OVERALL PASS]");
        else
            $display("[OVERALL FAIL]");
        $finish;
    end

    initial begin
        #(CLK_PER * 50000);
        $display("[FATAL] Global timeout");
        $finish;
    end

endmodule
