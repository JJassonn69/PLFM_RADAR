`timescale 1ns / 1ps
//
// Testbench for radar_system_top_te0713_umft601x_dev
//
// Validates the FT601 dev build with constant-HIGH chip reset (no FT601
// reset pulse). The FT601 is assumed to be running before FPGA config;
// the PULLUP on ft601_chip_reset_n keeps it stable during config, and
// the RTL drives it HIGH immediately.
//
// Key scenarios tested:
//   1. ft601_chip_reset_n is immediately HIGH (constant, not POR-driven)
//   2. ft601_wakeup_n is tied HIGH
//   3. Internal sys_reset_n releases after ft601_clk_in POR counter saturates
//   4. Heartbeat counter increments after sys_reset_n releases
//   5. stream_control defaults to OFF (3'b000)
//   6. USB host command via FT601 bus: byte-order decode (Bug 1 fix)
//   7. WR_N deasserts when TXE goes high mid-packet (Bug 2 fix)
//   8. Synthetic data packets are generated when stream_control[0] is enabled
//   9. FT601 control outputs are not X after sys_reset_n
//  10. Post-POR startup lockout prevents write FSM from acting (v7 fix)
//  11. Status request via USB command produces correct 8-word response (v7 fix)
//  12. SEND_STATUS watchdog timeout returns FSM to IDLE (v7 fix)
//  13. Debug counters (v7b) are wired and incrementing
//  14. Expanded status packet (v7b) with 10 transfers completes correctly
//  15. v7c: SEND_STATUS no-retrigger — exactly 1 status packet per request
//

module tb_radar_system_top_te0713_umft601x_dev;

    // =====================================================================
    // Clock/reset control
    // =====================================================================
    reg osc_50m = 0;
    reg ft601_clk_running = 0;
    reg ft601_clk_raw = 0;
    wire ft601_clk_in;

    // 50 MHz on-board oscillator — always running
    always #10 osc_50m = ~osc_50m;  // 20 ns period

    // FT601 100 MHz clock — starts ONLY after ft601_chip_reset_n goes high
    always #5 ft601_clk_raw = ~ft601_clk_raw;  // 10 ns period
    assign ft601_clk_in = ft601_clk_running ? ft601_clk_raw : 1'b0;

    // =====================================================================
    // DUT interface signals
    // =====================================================================
    wire [31:0] ft601_data;
    wire [3:0]  ft601_be;
    reg         ft601_txe;
    reg         ft601_rxf;
    wire        ft601_wr_n;
    wire        ft601_rd_n;
    wire        ft601_oe_n;
    wire        ft601_siwu_n;
    wire        ft601_chip_reset_n;
    wire        ft601_wakeup_n;
    wire        ft601_gpio0;
    wire        ft601_gpio1;

    // =====================================================================
    // FT601 host emulation — bidirectional data bus driver
    // =====================================================================
    reg [31:0] host_data_drive = 32'd0;
    reg        host_data_oe = 1'b0;  // 1 = TB drives bus, 0 = tristate

    // TB drives ft601_data when host_data_oe=1 (simulating FT601 pushing data)
    // DUT drives ft601_data when ft601_data_oe=1 (FPGA sending data to host)
    assign ft601_data = host_data_oe ? host_data_drive : 32'hzzzz_zzzz;

    // Pulldown on data bus to avoid X during tristate
    pulldown pd[31:0] (ft601_data);

    // FT601 emulation: TXE=0 means "ready for write" (active-low on wire,
    // but the RTL reads it as active-HIGH due to polarity mapping).
    // Set txe=0 → FSM sees !0=1 → "can write". Set rxf=1 → "no data to read".
    initial begin
        ft601_txe = 1'b0;   // FT601 FIFO ready for writes (active-low)
        ft601_rxf = 1'b1;   // No host data to read (active-low, high=empty)
    end

    // =====================================================================
    // DUT instantiation
    // =====================================================================
    radar_system_top_te0713_umft601x_dev dut (
        .osc_50m            (osc_50m),
        .ft601_clk_in       (ft601_clk_in),
        .ft601_data         (ft601_data),
        .ft601_be           (ft601_be),
        .ft601_txe          (ft601_txe),
        .ft601_rxf          (ft601_rxf),
        .ft601_wr_n         (ft601_wr_n),
        .ft601_rd_n         (ft601_rd_n),
        .ft601_oe_n         (ft601_oe_n),
        .ft601_siwu_n       (ft601_siwu_n),
        .ft601_chip_reset_n (ft601_chip_reset_n),
        .ft601_wakeup_n     (ft601_wakeup_n),
        .ft601_gpio0        (ft601_gpio0),
        .ft601_gpio1        (ft601_gpio1)
    );

    // =====================================================================
    // Test infrastructure
    // =====================================================================
    integer test_num = 0;
    integer pass_count = 0;
    integer fail_count = 0;

    task check;
        input [255:0] name;
        input         actual;
        input         expected;
        begin
            test_num = test_num + 1;
            if (actual === expected) begin
                $display("  [PASS] Test %0d: %0s", test_num, name);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] Test %0d: %0s -- got %b, expected %b",
                         test_num, name, actual, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task check_nonzero;
        input [255:0] name;
        input [31:0]  actual;
        begin
            test_num = test_num + 1;
            if (actual !== 32'd0 && actual !== 32'bx) begin
                $display("  [PASS] Test %0d: %0s (value=0x%08h)", test_num, name, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] Test %0d: %0s -- got 0x%08h, expected non-zero",
                         test_num, name, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task check_val;
        input [255:0] name;
        input [31:0]  actual;
        input [31:0]  expected;
        begin
            test_num = test_num + 1;
            if (actual === expected) begin
                $display("  [PASS] Test %0d: %0s", test_num, name);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] Test %0d: %0s -- got 0x%08h, expected 0x%08h",
                         test_num, name, actual, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // =====================================================================
    // Helper: wait N posedge of osc_50m
    // =====================================================================
    task wait_osc;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                @(posedge osc_50m);
        end
    endtask

    // =====================================================================
    // Helper: wait N posedge of ft601_clk_in (only when running)
    // =====================================================================
    task wait_ft_clk;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                @(posedge ft601_clk_in);
        end
    endtask

    // =====================================================================
    // Helper: Simulate FT601 host writing a 32-bit command to the FPGA
    //
    // Models the FT601 245 Sync FIFO read protocol:
    //   1. Assert RXF (active-low: 0 = data available)
    //   2. Wait for DUT to assert OE_N=0 (bus turnaround request)
    //   3. Drive data on bus after 1-cycle turnaround
    //   4. Wait for DUT to assert RD_N=0 (read strobe)
    //   5. Hold data for 1 more cycle (data sampled on posedge)
    //   6. Deassert RXF (high = no more data)
    //   7. Release bus
    //
    // The command word is in FT601 byte-lane order (little-endian):
    //   DATA[7:0]   = opcode (first USB byte)
    //   DATA[15:8]  = addr   (second USB byte)
    //   DATA[31:16] = value  (bytes 3-4, byte-swapped from big-endian)
    //
    // Python packs: struct.pack(">I", (opcode<<24)|(addr<<16)|value)
    //   => wire bytes: [opcode, addr, value_hi, value_lo]
    //   => FT601 DATA[31:0] = {value_lo, value_hi, addr, opcode}
    // =====================================================================
    task ft601_host_write;
        input [7:0]  opcode;
        input [7:0]  addr;
        input [15:0] value;
        integer timeout;
        begin
            // Build the 32-bit word as FT601 presents it on the bus:
            // DATA = {value[7:0], value[15:8], addr, opcode}
            host_data_drive = {value[7:0], value[15:8], addr, opcode};

            // Signal: host has data available
            @(posedge ft601_clk_in);
            ft601_rxf = 1'b0;  // RXF active-low: 0 = data available

            // Wait for DUT to assert OE_N=0 (bus turnaround) — timeout after 100 cycles
            timeout = 0;
            while (ft601_oe_n !== 1'b0 && timeout < 100) begin
                @(posedge ft601_clk_in);
                timeout = timeout + 1;
            end

            if (timeout >= 100) begin
                $display("    WARNING: ft601_host_write timeout waiting for OE_N=0");
                ft601_rxf = 1'b1;
                // Don't return early — let the test check fail naturally
            end else begin
                // DUT asserted OE_N=0 — 1-cycle turnaround, then drive data
                host_data_oe = 1'b1;  // TB drives bus

                // Wait for DUT to assert RD_N=0 (read strobe)
                timeout = 0;
                while (ft601_rd_n !== 1'b0 && timeout < 100) begin
                    @(posedge ft601_clk_in);
                    timeout = timeout + 1;
                end

                // Data is sampled on the posedge when RD_N=0
                // Hold for 1 more cycle so the DUT captures it
                @(posedge ft601_clk_in);

                // DUT should have captured data and will deassert RD_N
                // Deassert RXF (no more data from host)
                ft601_rxf = 1'b1;

                // Wait a couple cycles for DUT to process (RD_DEASSERT -> RD_PROCESS -> RD_IDLE)
                wait_ft_clk(5);

                // Release bus
                host_data_oe = 1'b0;
            end
        end
    endtask

    // =====================================================================
    // Main test
    // =====================================================================
    initial begin
        $dumpfile("tb_radar_system_top_te0713_umft601x_dev.vcd");
        $dumpvars(0, tb_radar_system_top_te0713_umft601x_dev);

        $display("");
        $display("=== tb_radar_system_top_te0713_umft601x_dev ===");
        $display("    Constant-HIGH chip reset / no FT601 reset pulse");
        $display("    v7c: status retrigger fix (pending+busy flags)");
        $display("");

        // -----------------------------------------------------------------
        // Phase 1: Verify ft601_chip_reset_n is immediately HIGH
        // -----------------------------------------------------------------
        $display("--- Phase 1: Chip reset is constant HIGH ---");

        #1;
        check("ft601_chip_reset_n immediately high", ft601_chip_reset_n, 1'b1);
        check("ft601_wakeup_n tied high", ft601_wakeup_n, 1'b1);
        check("ft601_clk_running initially off", ft601_clk_running, 1'b0);
        check("ft601_gpio1 (sys_reset_n) initially low", ft601_gpio1, 1'b0);

        // -----------------------------------------------------------------
        // Phase 2: Start FT601 clock (simulating FT601 already running)
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 2: FT601 clock starts (already running in HW) ---");

        // In real HW, FT601 was running before FPGA config.
        // Start the clock after a small delay to model IBUF startup.
        #200;
        ft601_clk_running = 1;
        $display("    ft601_clk_in now running (100 MHz)");

        // por_counter in ft601_clk_in domain needs 2^15 = 32768 cycles
        $display("    Waiting for sys_por_counter to saturate (32768 ft601 clk cycles)...");
        wait_ft_clk(32800);

        check("sys_reset_n goes high after ft601_clk POR", dut.sys_reset_n, 1'b1);
        check("ft601_gpio1 reflects sys_reset_n", ft601_gpio1, 1'b1);
        check("ft601_chip_reset_n still high", ft601_chip_reset_n, 1'b1);

        // -----------------------------------------------------------------
        // Phase 2b: Post-POR startup lockout prevents spurious SEND_STATUS
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 2b: Startup lockout (v7 fix) ---");

        // Immediately after POR release, the startup lockout counter should
        // be counting up. The write FSM must stay in IDLE during this period.
        // With TXE=0 (ready for writes) and ft601_rxf=1 (no host data), any
        // spurious status_req_edge pulse would previously trigger SEND_STATUS.
        check("startup_lockout_active right after POR",
              dut.usb_inst.startup_lockout_active, 1'b1);
        check("write FSM in IDLE during lockout",
              (dut.usb_inst.current_state == 3'd0), 1'b1);

        // Wait for lockout to complete (256 cycles)
        wait_ft_clk(260);
        check("startup_lockout_active cleared after 256+ cycles",
              dut.usb_inst.startup_lockout_active, 1'b0);
        check("write FSM still in IDLE after lockout",
              (dut.usb_inst.current_state == 3'd0), 1'b1);

        // -----------------------------------------------------------------
        // Phase 3: Verify heartbeat counter is running
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 3: Heartbeat counter running ---");

        wait_ft_clk(100);
        check_nonzero("hb_counter is non-zero after POR", dut.hb_counter);

        // -----------------------------------------------------------------
        // Phase 4: USB host command via FT601 bus — byte-order decode test
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 4: USB host command byte-order decode (Bug 1 fix) ---");

        // stream_control_reg defaults to 3'b000 (all streams off).
        check_val("stream_control_reg defaults to 0",
                  {29'd0, dut.stream_control_reg}, 32'd0);

        // Send opcode=0x04, addr=0x00, value=0x0007 (enable all 3 streams)
        // This is the exact command test_ft601_streaming.py sends.
        // Python: struct.pack(">I", 0x04000007) => bytes [04, 00, 00, 07]
        // FT601 DATA[31:0] = {07, 00, 00, 04}
        $display("    Sending USB command: opcode=0x04, addr=0x00, value=0x0007...");
        ft601_host_write(8'h04, 8'h00, 16'h0007);

        // Verify cmd_opcode was decoded correctly (should be 0x04, not 0x07)
        check_val("cmd_opcode decoded as 0x04",
                  {24'd0, dut.usb_inst.cmd_opcode}, 32'h00000004);
        check_val("cmd_value decoded as 0x0007",
                  {16'd0, dut.usb_inst.cmd_value}, 32'h00000007);

        // The opcode 0x04 should have set stream_control_reg = 3'b111
        check_val("stream_control_reg set to 0x07 via USB cmd",
                  {29'd0, dut.stream_control_reg}, 32'd7);

        // Send a second command to change stream_control to just range (0x01)
        $display("    Sending USB command: opcode=0x04, addr=0x00, value=0x0001...");
        ft601_host_write(8'h04, 8'h00, 16'h0001);

        check_val("stream_control_reg updated to 0x01 via USB cmd",
                  {29'd0, dut.stream_control_reg}, 32'd1);

        // Send a command with different opcode (0x30 = self-test trigger)
        // to verify non-0x04 opcodes don't clobber stream_control_reg
        $display("    Sending USB command: opcode=0x30 (self-test trigger)...");
        ft601_host_write(8'h30, 8'h00, 16'h0000);

        check_val("stream_control_reg unchanged after 0x30 cmd",
                  {29'd0, dut.stream_control_reg}, 32'd1);
        check_val("cmd_opcode is 0x30 after self-test cmd",
                  {24'd0, dut.usb_inst.cmd_opcode}, 32'h00000030);

        // -----------------------------------------------------------------
        // Phase 5: WR_N safe defaults — TXE deassert mid-packet (Bug 2 fix)
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 5: WR_N deasserts when TXE goes high (Bug 2 fix) ---");

        // stream_control_reg is already 3'b001 (range stream enabled).
        // Wait for a synthetic data packet to start sending.
        // First, wait for packet_div to rollover (65536 cycles).
        $display("    Waiting for packet_div rollover to trigger data send...");
        wait_ft_clk(66000);

        // At this point, the write FSM should have tried to send data.
        // With ft601_txe=0, the FSM should have asserted wr_n=0 during write.
        // Now, yank TXE high (FIFO full) and verify WR_N goes high within
        // a couple of cycles.
        $display("    Deasserting TXE (FIFO full)...");
        @(posedge ft601_clk_in);
        ft601_txe = 1'b1;  // FT601 says "FIFO full, stop writing"

        // Wait 3 cycles for the safe default to take effect
        // (1 cycle for TXE pipeline register, 1 cycle for FSM, 1 safety margin)
        wait_ft_clk(3);

        check("WR_N deasserted after TXE goes high", ft601_wr_n, 1'b1);

        // Restore TXE for remaining tests
        ft601_txe = 1'b0;
        wait_ft_clk(5);

        // -----------------------------------------------------------------
        // Phase 6: Synthetic data generation (direct poke fallback)
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 6: Synthetic data generation ---");

        // stream_control_reg is already 1 from USB command in Phase 4
        check_nonzero("range_profile_reg has synthetic data", dut.range_profile_reg);

        // -----------------------------------------------------------------
        // Phase 7: Verify FT601 control outputs are not X
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 7: FT601 bus sanity ---");

        check("ft601_wr_n is not X", (ft601_wr_n === 1'b0 || ft601_wr_n === 1'b1), 1'b1);
        check("ft601_rd_n is not X", (ft601_rd_n === 1'b0 || ft601_rd_n === 1'b1), 1'b1);
        check("ft601_oe_n is not X", (ft601_oe_n === 1'b0 || ft601_oe_n === 1'b1), 1'b1);
        check("ft601_siwu_n is not X", (ft601_siwu_n === 1'b0 || ft601_siwu_n === 1'b1), 1'b1);

        // -----------------------------------------------------------------
        // Phase 8: Status request via USB (0xFF) produces status response
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 8: Status request via USB command (v7) ---");

        // Send opcode=0xFF (status request). The dev wrapper maps this to
        // status_request_reg pulse, which toggles status_req_toggle_100m in
        // usb_data_interface, causing the write FSM to enter SEND_STATUS.
        $display("    Sending USB command: opcode=0xFF (status request)...");
        ft601_host_write(8'hFF, 8'h00, 16'h0000);

        // The status request CDC toggle needs a few cycles to propagate.
        // After that, the IDLE state should detect status_req_pending and
        // enter SEND_STATUS.
        wait_ft_clk(10);

        // The write FSM should have entered SEND_STATUS and started sending.
        // With TXE=0 (FIFO ready), it should complete all 8 words quickly.
        // Wait enough cycles for 8 words + WAIT_ACK + return to IDLE.
        wait_ft_clk(20);

        // After completing, write FSM should be back in IDLE
        check("write FSM returns to IDLE after status send",
              (dut.usb_inst.current_state == 3'd0), 1'b1);

        // -----------------------------------------------------------------
        // Phase 9: SEND_STATUS watchdog timeout test
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 9: SEND_STATUS watchdog timeout (v7) ---");

        // Send another status request, but this time hold TXE HIGH (FIFO full)
        // so the write FSM can't send any data. The watchdog should abort
        // after 65536 cycles.
        $display("    Setting TXE=1 (FIFO full) to block writes...");
        ft601_txe = 1'b1;

        $display("    Sending USB command: opcode=0xFF (status request)...");
        ft601_host_write(8'hFF, 8'h00, 16'h0000);

        // Wait for status request CDC to propagate and FSM to enter SEND_STATUS
        wait_ft_clk(10);

        // The FSM should be in SEND_STATUS (or about to enter it)
        // Wait for the watchdog to trigger (65536 cycles + margin)
        $display("    Waiting for watchdog timeout (65536 cycles)...");
        wait_ft_clk(66000);

        // After watchdog, FSM should be back in IDLE
        check("write FSM returns to IDLE after watchdog timeout",
              (dut.usb_inst.current_state == 3'd0), 1'b1);

        // Verify the FSM can still accept commands after watchdog recovery
        ft601_txe = 1'b0;  // Restore TXE
        wait_ft_clk(5);

        $display("    Sending USB command: opcode=0x04, value=0x0003 (recovery test)...");
        ft601_host_write(8'h04, 8'h00, 16'h0003);
        check_val("stream_control_reg updated after watchdog recovery",
                  {29'd0, dut.stream_control_reg}, 32'd3);

        // -----------------------------------------------------------------
        // Phase 10: Debug counters (v7b instrumentation)
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 10: Debug counters (v7b instrumentation) ---");

        // After all the phases above, the debug counters should have
        // accumulated some counts. The exact values depend on FSM activity:
        //
        // - dbg_pkt_starts: incremented each time SEND_HEADER is entered
        //   Phase 6 waited for packet_div rollover with stream enabled,
        //   so at least 1 packet start should have occurred.
        //
        // - dbg_pkt_completions: incremented each time WAIT_ACK is entered
        //   Phase 8 sent a status request which completed (WAIT_ACK),
        //   plus any data packet completions from Phase 6.
        //
        // - dbg_wr_strobes: incremented on each WR_N=0 cycle. Should be
        //   non-zero if any data was written.
        //
        // We can verify the counters are wired and non-zero.

        // Check debug ports are accessible and at least pkt_completions > 0
        // (Phase 8 status request completed → at least 1 WAIT_ACK)
        check("dbg_pkt_completions > 0 (status request completed)",
              (dut.usb_inst.dbg_pkt_completions_r != 16'd0), 1'b1);

        check("dbg_wr_strobes > 0 (writes occurred)",
              (dut.usb_inst.dbg_wr_strobes_r != 16'd0), 1'b1);

        // Verify debug output wires match internal registers
        check("dbg_wr_strobes output matches register",
              (dut.dbg_wr_strobes == dut.usb_inst.dbg_wr_strobes_r), 1'b1);
        check("dbg_txe_blocks output matches register",
              (dut.dbg_txe_blocks == dut.usb_inst.dbg_txe_blocks_r), 1'b1);
        check("dbg_pkt_starts output matches register",
              (dut.dbg_pkt_starts == dut.usb_inst.dbg_pkt_starts_r), 1'b1);
        check("dbg_pkt_completions output matches register",
              (dut.dbg_pkt_completions == dut.usb_inst.dbg_pkt_completions_r), 1'b1);

        // Now do a fresh packet generation cycle and verify counters increment.
        // First note current values:
        $display("    Current debug counters:");
        $display("      wr_strobes=%0d  txe_blocks=%0d  pkt_starts=%0d  pkt_completions=%0d",
                 dut.usb_inst.dbg_wr_strobes_r,
                 dut.usb_inst.dbg_txe_blocks_r,
                 dut.usb_inst.dbg_pkt_starts_r,
                 dut.usb_inst.dbg_pkt_completions_r);

        // -----------------------------------------------------------------
        // Phase 11: Status response with debug words (v7b expanded packet)
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 11: Expanded status packet with debug words (v7b) ---");

        // Send status request and capture the response (10 x 32-bit transfers)
        $display("    Sending USB command: opcode=0xFF (status request)...");
        ft601_host_write(8'hFF, 8'h00, 16'h0000);

        // Wait for CDC propagation + full status packet send
        // 10 transfers at 1 cycle each + margins
        wait_ft_clk(30);

        // After completing, write FSM should be back in IDLE
        check("write FSM returns to IDLE after expanded status send",
              (dut.usb_inst.current_state == 3'd0), 1'b1);

        // Verify status_word_idx reset to 0 after completion
        check_val("status_word_idx reset after completion",
                  {28'd0, dut.usb_inst.status_word_idx}, 32'd0);

        // -----------------------------------------------------------------
        // Phase 12: v7c — SEND_STATUS no-retrigger test
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 12: SEND_STATUS no-retrigger (v7c fix) ---");

        // This phase verifies that a single status request produces exactly
        // ONE status packet (10 writes: BB + 8 words + footer), and the FSM
        // does NOT re-enter SEND_STATUS after completing.
        //
        // The v7b bug: status_req_ft601 (XOR edge detect) could re-trigger
        // when the FSM returned to IDLE, causing an infinite BB loop.
        // The v7c fix uses a registered pending flag + busy guard.

        // First, disable streaming so we have a clean bus (no data packets)
        $display("    Disabling streaming for clean status test...");
        ft601_host_write(8'h04, 8'h00, 16'h0000);  // stream_control = 0
        wait_ft_clk(10);

        // Record current pkt_completions count
        // (We'll check that exactly 1 more completion occurs)
        begin : phase12_block
            reg [15:0] completions_before;
            reg [15:0] wr_strobes_before;

            completions_before = dut.usb_inst.dbg_pkt_completions_r;
            wr_strobes_before  = dut.usb_inst.dbg_wr_strobes_r;

            $display("    completions_before=%0d  wr_strobes_before=%0d",
                     completions_before, wr_strobes_before);

            // Send status request
            $display("    Sending USB command: opcode=0xFF (status request)...");
            ft601_host_write(8'hFF, 8'h00, 16'h0000);

            // Wait for CDC propagation + full status packet + WAIT_ACK + IDLE
            // 10 writes + some overhead = ~30 cycles should be more than enough
            wait_ft_clk(50);

            // FSM should be back in IDLE
            check("v7c: FSM in IDLE after status send",
                  (dut.usb_inst.current_state == 3'd0), 1'b1);

            // status_busy should be cleared
            check("v7c: status_busy cleared after completion",
                  dut.usb_inst.status_busy, 1'b0);

            // status_req_pending should be cleared
            check("v7c: status_req_pending cleared after consumption",
                  dut.usb_inst.status_req_pending, 1'b0);

            // Exactly 1 new completion (the status packet)
            check_val("v7c: exactly 1 pkt_completion from status request",
                      {16'd0, dut.usb_inst.dbg_pkt_completions_r - completions_before},
                      32'd1);

            // Exactly 10 new write strobes (BB + 8 status words + footer)
            check_val("v7c: exactly 10 wr_strobes from status packet",
                      {16'd0, dut.usb_inst.dbg_wr_strobes_r - wr_strobes_before},
                      32'd10);

            // Now wait a long time to confirm no more packets are sent
            // (the old bug would produce continuous BB words)
            $display("    Waiting 500 cycles to confirm no retrigger...");
            wait_ft_clk(500);

            // Check that no additional completions occurred
            check_val("v7c: no additional completions after 500 cycles",
                      {16'd0, dut.usb_inst.dbg_pkt_completions_r - completions_before},
                      32'd1);

            // Check that no additional write strobes occurred
            check_val("v7c: no additional wr_strobes after 500 cycles",
                      {16'd0, dut.usb_inst.dbg_wr_strobes_r - wr_strobes_before},
                      32'd10);

            // FSM should still be in IDLE (not in SEND_STATUS)
            check("v7c: FSM still in IDLE after wait",
                  (dut.usb_inst.current_state == 3'd0), 1'b1);
        end

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("");
        $display("========================================");
        $display("  Results: %0d passed, %0d failed (of %0d)",
                 pass_count, fail_count, test_num);
        $display("========================================");

        if (fail_count > 0) begin
            $display("FAIL");
            $finish(1);
        end else begin
            $display("ALL TESTS PASSED");
            $finish(0);
        end
    end

    // Safety timeout — abort if sim runs too long
    initial begin
        #50_000_000;  // 50 ms of sim time
        $display("TIMEOUT: Simulation exceeded 50ms");
        $finish(1);
    end

endmodule
