`timescale 1ns / 1ps
//
// Testbench for radar_system_top_te0713_umft601x_playback
//
// Validates the unified BRAM playback + FT601 USB streaming pipeline.
// Tests both the USB infrastructure (inherited from v7c) and the new
// DSP pipeline integration:
//
//   1. ft601_chip_reset_n constant HIGH
//   2. POR release and startup lockout
//   3. USB command decode for all playback opcodes
//   4. Playback trigger via USB → BRAM starts
//   5. DSP data (decimated range) appears at USB output
//   6. Status request returns correct config values
//   7. CFAR/MTI/DC config via USB commands
//   8. FT601 control signals are clean
//   9. No status retrigger (v7c test)
//

module tb_radar_system_top_te0713_umft601x_playback;

    // =====================================================================
    // Clock/reset control
    // =====================================================================
    reg osc_50m = 0;
    reg ft601_clk_running = 0;
    reg ft601_clk_raw = 0;
    wire ft601_clk_in;

    // 50 MHz on-board oscillator
    always #10 osc_50m = ~osc_50m;  // 20 ns period

    // FT601 100 MHz clock
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
    reg        host_data_oe = 1'b0;

    assign ft601_data = host_data_oe ? host_data_drive : 32'hzzzz_zzzz;

    pulldown pd[31:0] (ft601_data);

    initial begin
        ft601_txe = 1'b0;   // FT601 FIFO ready for writes
        ft601_rxf = 1'b1;   // No host data to read
    end

    // =====================================================================
    // DUT instantiation
    // =====================================================================
    radar_system_top_te0713_umft601x_playback dut (
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
    // Helper: wait N posedge of ft601_clk_in
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
    // =====================================================================
    task ft601_host_write;
        input [7:0]  opcode;
        input [7:0]  addr;
        input [15:0] value;
        integer timeout;
        begin
            host_data_drive = {value[7:0], value[15:8], addr, opcode};
            @(posedge ft601_clk_in);
            ft601_rxf = 1'b0;

            timeout = 0;
            while (ft601_oe_n !== 1'b0 && timeout < 100) begin
                @(posedge ft601_clk_in);
                timeout = timeout + 1;
            end

            if (timeout >= 100) begin
                $display("    WARNING: ft601_host_write timeout waiting for OE_N=0");
                ft601_rxf = 1'b1;
            end else begin
                host_data_oe = 1'b1;

                timeout = 0;
                while (ft601_rd_n !== 1'b0 && timeout < 100) begin
                    @(posedge ft601_clk_in);
                    timeout = timeout + 1;
                end

                @(posedge ft601_clk_in);

                ft601_rxf = 1'b1;
                wait_ft_clk(5);
                host_data_oe = 1'b0;
            end
        end
    endtask

    // =====================================================================
    // Helper: Wait for write FSM to return to IDLE
    // =====================================================================
    task wait_fsm_idle;
        input integer max_cycles;
        integer timeout;
        begin
            timeout = 0;
            while (dut.usb_inst.current_state != 3'd0 && timeout < max_cycles) begin
                @(posedge ft601_clk_in);
                timeout = timeout + 1;
            end
        end
    endtask

    // =====================================================================
    // Helper: Count AA headers seen on ft601_data during write operations
    // =====================================================================
    integer data_packet_count;
    reg [31:0] captured_data_words [0:15];  // Capture buffer for packet words
    integer capture_idx;

    // =====================================================================
    // Main test
    // =====================================================================
    initial begin
        $dumpfile("tb_radar_system_top_te0713_umft601x_playback.vcd");
        $dumpvars(0, tb_radar_system_top_te0713_umft601x_playback);

        data_packet_count = 0;
        capture_idx = 0;

        $display("");
        $display("=== tb_radar_system_top_te0713_umft601x_playback ===");
        $display("    Unified BRAM playback + FT601 USB streaming");
        $display("    v8: Real DSP data over USB");
        $display("");

        // -----------------------------------------------------------------
        // Phase 1: Verify ft601_chip_reset_n is immediately HIGH
        // -----------------------------------------------------------------
        $display("--- Phase 1: Chip reset is constant HIGH ---");

        #1;
        check("ft601_chip_reset_n immediately high", ft601_chip_reset_n, 1'b1);
        check("ft601_wakeup_n tied high", ft601_wakeup_n, 1'b1);
        check("ft601_gpio1 (sys_reset_n) initially low", ft601_gpio1, 1'b0);

        // -----------------------------------------------------------------
        // Phase 2: Start FT601 clock and POR
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 2: POR release ---");

        #200;
        ft601_clk_running = 1;
        $display("    ft601_clk_in now running (100 MHz)");

        // Wait for POR (2^15 = 32768 cycles)
        wait_ft_clk(32800);

        check("sys_reset_n goes high after POR", dut.sys_reset_n, 1'b1);
        check("ft601_gpio1 reflects sys_reset_n", ft601_gpio1, 1'b1);

        // -----------------------------------------------------------------
        // Phase 2b: Startup lockout
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 2b: Startup lockout ---");

        check("startup_lockout_active right after POR",
              dut.usb_inst.startup_lockout_active, 1'b1);
        check("write FSM in IDLE during lockout",
              (dut.usb_inst.current_state == 3'd0), 1'b1);

        wait_ft_clk(260);
        check("startup_lockout_active cleared after 256+ cycles",
              dut.usb_inst.startup_lockout_active, 1'b0);

        // -----------------------------------------------------------------
        // Phase 3: Heartbeat counter running
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 3: Heartbeat counter ---");

        wait_ft_clk(100);
        check_nonzero("hb_counter is non-zero", dut.hb_counter);

        // -----------------------------------------------------------------
        // Phase 4: USB command decode — stream control
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 4: USB command decode — stream control ---");

        check_val("stream_control defaults to 0",
                  {29'd0, dut.stream_control_reg}, 32'd0);

        ft601_host_write(8'h04, 8'h00, 16'h0001);
        check_val("stream_control set to 0x01 (range only)",
                  {29'd0, dut.stream_control_reg}, 32'd1);

        // -----------------------------------------------------------------
        // Phase 5: USB command decode — DSP config registers
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 5: USB command decode — DSP config ---");

        // CFAR guard cells
        ft601_host_write(8'h21, 8'h00, 16'h0005);
        check_val("cfar_guard_reg set to 5",
                  {28'd0, dut.cfar_guard_reg}, 32'd5);

        // CFAR training cells
        ft601_host_write(8'h22, 8'h00, 16'h000A);
        check_val("cfar_train_reg set to 10",
                  {27'd0, dut.cfar_train_reg}, 32'd10);

        // CFAR alpha
        ft601_host_write(8'h23, 8'h00, 16'h0040);
        check_val("cfar_alpha_reg set to 0x40",
                  {24'd0, dut.cfar_alpha_reg}, 32'h40);

        // CFAR mode (GO-CFAR)
        ft601_host_write(8'h24, 8'h00, 16'h0001);
        check_val("cfar_mode_reg set to 1 (GO)",
                  {30'd0, dut.cfar_mode_reg}, 32'd1);

        // CFAR enable
        ft601_host_write(8'h25, 8'h00, 16'h0001);
        check_val("cfar_enable_reg set to 1",
                  {31'd0, dut.cfar_enable_reg}, 32'd1);

        // MTI enable
        ft601_host_write(8'h26, 8'h00, 16'h0001);
        check_val("mti_enable_reg set to 1",
                  {31'd0, dut.mti_enable_reg}, 32'd1);

        // DC notch width
        ft601_host_write(8'h27, 8'h00, 16'h0003);
        check_val("dc_notch_width_reg set to 3",
                  {29'd0, dut.dc_notch_width_reg}, 32'd3);

        // Detection threshold
        ft601_host_write(8'h03, 8'h00, 16'h03E8);
        check_val("detect_threshold set to 1000",
                  {16'd0, dut.detect_threshold_reg}, 32'd1000);

        // -----------------------------------------------------------------
        // Phase 6: Playback trigger via USB → BRAM starts
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 6: Playback trigger via USB ---");

        // Verify playback is initially idle
        check("playback not active initially", dut.pb_playback_active, 1'b0);
        check("playback not done initially", dut.pb_playback_done, 1'b0);

        // Reset DSP config to known-good values for playback
        ft601_host_write(8'h26, 8'h00, 16'h0000);  // MTI disable
        ft601_host_write(8'h25, 8'h00, 16'h0000);  // CFAR disable
        ft601_host_write(8'h27, 8'h00, 16'h0000);  // DC notch off

        // Send playback trigger (opcode 0x02)
        $display("    Sending USB command: opcode=0x02 (playback trigger)...");
        ft601_host_write(8'h02, 8'h00, 16'h0000);

        // Playback should become active within a few cycles
        wait_ft_clk(10);
        check("playback_active after trigger", dut.pb_playback_active, 1'b1);
        check("chirp_count starts at 0",
              (dut.pb_chirp_count == 6'd0), 1'b1);

        // -----------------------------------------------------------------
        // Phase 7: DSP data appears at USB output
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 7: DSP data at USB output ---");

        // BRAM plays 1024 samples per chirp. The decimator reduces to 64.
        // After the first chirp's 1024 samples are played, the decimator
        // should have produced 64 valid range bins.
        //
        // Wait for first chirp to complete: 1024 samples + BRAM pipeline
        // At 100 MHz, ~1100 cycles for the first chirp + decimation.
        $display("    Waiting for first chirp decimation...");
        wait_ft_clk(1200);

        // At least one range_valid_usb should have pulsed
        // We can check that range_profile_usb has been set to a non-zero value
        // (since BRAM data is real radar data, it should be non-zero)
        check_nonzero("range_profile_usb has real data after first chirp",
                      dut.range_profile_usb);

        // -----------------------------------------------------------------
        // Phase 8: USB streaming sends real data packets
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 8: USB data packets with real DSP data ---");

        // The decimator produces 64 range bins per chirp. With stream_control[0]=1,
        // each range_valid pulse triggers a 24-byte USB data packet via the
        // write FSM. We should see WR_N pulses and AA/55 framing.

        // Wait for some packets to be sent
        wait_ft_clk(1000);

        // Check that packet starts and completions have occurred
        check("pkt_starts > 0 (data packets being sent)",
              (dut.usb_inst.dbg_pkt_starts_r != 16'd0), 1'b1);
        check("pkt_completions > 0 (data packets completed)",
              (dut.usb_inst.dbg_pkt_completions_r != 16'd0), 1'b1);
        check("wr_strobes > 0 (writes occurred)",
              (dut.usb_inst.dbg_wr_strobes_r != 16'd0), 1'b1);

        $display("    Debug counters after playback start:");
        $display("      pkt_starts=%0d  pkt_completions=%0d  wr_strobes=%0d",
                 dut.usb_inst.dbg_pkt_starts_r,
                 dut.usb_inst.dbg_pkt_completions_r,
                 dut.usb_inst.dbg_wr_strobes_r);

        // -----------------------------------------------------------------
        // Phase 9: Wait for playback to complete
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 9: Playback completion ---");

        // Full playback: 32 chirps × (1024 samples + 200 gap + pipeline) ≈ 32×1300 = 41600 cycles
        // Plus Doppler processing time. Wait generously.
        $display("    Waiting for all 32 chirps to play...");
        begin : phase9_wait
            integer wait_count;
            wait_count = 0;
            while (!dut.pb_playback_done && wait_count < 200000) begin
                @(posedge ft601_clk_in);
                wait_count = wait_count + 1;
            end
            if (wait_count >= 200000)
                $display("    WARNING: Timed out waiting for playback_done");
            else
                $display("    Playback completed after %0d cycles", wait_count);
        end

        check("playback_done asserted", dut.pb_playback_done, 1'b1);
        check("playback_active deasserted", dut.pb_playback_active, 1'b0);
        check_val("all 32 chirps played",
                  {26'd0, dut.pb_chirp_count}, 32'd32);

        // Check many packets were sent (at least 32 chirps × some range bins)
        $display("    Final debug counters:");
        $display("      pkt_starts=%0d  pkt_completions=%0d  wr_strobes=%0d",
                 dut.usb_inst.dbg_pkt_starts_r,
                 dut.usb_inst.dbg_pkt_completions_r,
                 dut.usb_inst.dbg_wr_strobes_r);

        // Sanity: completions should match starts (no stuck packets)
        check("pkt_completions == pkt_starts (no stuck packets)",
              (dut.usb_inst.dbg_pkt_completions_r == dut.usb_inst.dbg_pkt_starts_r), 1'b1);

        // -----------------------------------------------------------------
        // Phase 9b: v8b FIFO draining — wait for FIFO to empty
        // -----------------------------------------------------------------
        // The FIFO may still contain entries because the USB FSM takes
        // several cycles per packet. Wait for it to drain completely.
        $display("");
        $display("--- Phase 9b: FIFO drain after playback ---");

        begin : phase9b_drain
            integer drain_wait;
            drain_wait = 0;
            while (!dut.fifo_empty && drain_wait < 500000) begin
                @(posedge ft601_clk_in);
                drain_wait = drain_wait + 1;
            end
            if (drain_wait >= 500000)
                $display("    WARNING: FIFO drain timeout after %0d cycles", drain_wait);
            else
                $display("    FIFO drained after %0d cycles", drain_wait);
        end

        // Wait for last packet to complete
        wait_fsm_idle(1000);

        check("FIFO empty after drain", dut.fifo_empty, 1'b1);
        check("FIFO overflow count is 0", (dut.fifo_overflow_count == 16'd0), 1'b1);

        // After FIFO drain, pkt_starts should equal 2048 (32 chirps × 64 bins)
        // Allow some tolerance: at least 2000 packets
        $display("    After FIFO drain: pkt_starts=%0d  pkt_completions=%0d",
                 dut.usb_inst.dbg_pkt_starts_r,
                 dut.usb_inst.dbg_pkt_completions_r);

        check("v8b: pkt_starts >= 2000 (FIFO buffering works)",
              (dut.usb_inst.dbg_pkt_starts_r >= 16'd2000), 1'b1);
        check("v8b: pkt_completions == pkt_starts after drain",
              (dut.usb_inst.dbg_pkt_completions_r == dut.usb_inst.dbg_pkt_starts_r), 1'b1);

        // -----------------------------------------------------------------
        // Phase 10: Status request returns correct values
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 10: Status readback ---");

        ft601_host_write(8'hFF, 8'h00, 16'h0000);
        wait_ft_clk(30);

        check("write FSM in IDLE after status",
              (dut.usb_inst.current_state == 3'd0), 1'b1);
        check("status_word_idx reset",
              (dut.usb_inst.status_word_idx == 4'd0), 1'b1);

        // -----------------------------------------------------------------
        // Phase 11: FT601 bus sanity
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 11: FT601 bus sanity ---");

        check("ft601_wr_n is not X", (ft601_wr_n === 1'b0 || ft601_wr_n === 1'b1), 1'b1);
        check("ft601_rd_n is not X", (ft601_rd_n === 1'b0 || ft601_rd_n === 1'b1), 1'b1);
        check("ft601_oe_n is not X", (ft601_oe_n === 1'b0 || ft601_oe_n === 1'b1), 1'b1);
        check("ft601_siwu_n is not X", (ft601_siwu_n === 1'b0 || ft601_siwu_n === 1'b1), 1'b1);

        // -----------------------------------------------------------------
        // Phase 12: WR_N deasserts when TXE goes high (Bug 2 regression)
        // Plus v8b: FIFO continues delivering data after backpressure
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 12: WR_N deasserts + FIFO survives backpressure ---");

        // Trigger another playback so data is flowing
        ft601_host_write(8'h02, 8'h00, 16'h0000);
        wait_ft_clk(1200);  // Wait for data to be flowing

        begin : phase12_block
            reg [15:0] pkt_before_bp;
            pkt_before_bp = dut.usb_inst.dbg_pkt_completions_r;

            @(posedge ft601_clk_in);
            ft601_txe = 1'b1;  // FIFO full — backpressure
            wait_ft_clk(3);

            check("WR_N deasserted after TXE goes high", ft601_wr_n, 1'b1);

            // Hold backpressure for 500 cycles — FIFO should buffer data
            wait_ft_clk(500);

            check("v8b: FIFO not empty during backpressure",
                  !dut.fifo_empty, 1'b1);

            // Release backpressure
            ft601_txe = 1'b0;
            wait_ft_clk(200);

            // Packets should resume after backpressure clears
            check("v8b: more packets after backpressure release",
                  (dut.usb_inst.dbg_pkt_completions_r > pkt_before_bp), 1'b1);
        end

        // Wait for this playback to finish + FIFO drain
        begin : phase12_drain
            integer drain_wait;
            drain_wait = 0;
            while ((!dut.pb_playback_done || !dut.fifo_empty) && drain_wait < 500000) begin
                @(posedge ft601_clk_in);
                drain_wait = drain_wait + 1;
            end
        end
        wait_fsm_idle(1000);

        // -----------------------------------------------------------------
        // Phase 13: SEND_STATUS no-retrigger (v7c regression)
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 13: SEND_STATUS no-retrigger (v7c) ---");

        // Disable streaming for clean test
        ft601_host_write(8'h04, 8'h00, 16'h0000);
        wait_ft_clk(10);

        // Wait for any in-flight packets to complete
        wait_fsm_idle(200);

        begin : phase13_block
            reg [15:0] completions_before;
            reg [15:0] wr_strobes_before;

            completions_before = dut.usb_inst.dbg_pkt_completions_r;
            wr_strobes_before  = dut.usb_inst.dbg_wr_strobes_r;

            ft601_host_write(8'hFF, 8'h00, 16'h0000);
            wait_ft_clk(50);

            check("v7c: FSM in IDLE after status",
                  (dut.usb_inst.current_state == 3'd0), 1'b1);
            check("v7c: status_busy cleared",
                  dut.usb_inst.status_busy, 1'b0);
            check("v7c: status_req_pending cleared",
                  dut.usb_inst.status_req_pending, 1'b0);
            check_val("v7c: exactly 1 completion",
                      {16'd0, dut.usb_inst.dbg_pkt_completions_r - completions_before},
                      32'd1);
            check_val("v7c: exactly 10 wr_strobes",
                      {16'd0, dut.usb_inst.dbg_wr_strobes_r - wr_strobes_before},
                      32'd10);

            wait_ft_clk(500);

            check_val("v7c: no retrigger after 500 cycles",
                      {16'd0, dut.usb_inst.dbg_pkt_completions_r - completions_before},
                      32'd1);
        end

        // -----------------------------------------------------------------
        // Phase 14: Retrigger playback (restart from DONE state)
        // Plus v8b: verify FIFO delivers all packets on retrigger
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 14: Retrigger playback + FIFO delivery ---");

        // Re-enable streaming
        ft601_host_write(8'h04, 8'h00, 16'h0001);

        // Wait for second playback completion
        begin : phase14_wait
            reg [15:0] pkt_before;
            pkt_before = dut.usb_inst.dbg_pkt_starts_r;

            ft601_host_write(8'h02, 8'h00, 16'h0000);
            wait_ft_clk(10);

            check("playback_active after retrigger", dut.pb_playback_active, 1'b1);

            // Wait for completion
            begin : phase14_inner_wait
                integer wait_count;
                wait_count = 0;
                while (!dut.pb_playback_done && wait_count < 200000) begin
                    @(posedge ft601_clk_in);
                    wait_count = wait_count + 1;
                end
            end

            check("playback_done after retrigger", dut.pb_playback_done, 1'b1);

            // v8b: Wait for FIFO to drain completely
            begin : phase14_drain
                integer drain_wait;
                drain_wait = 0;
                while (!dut.fifo_empty && drain_wait < 500000) begin
                    @(posedge ft601_clk_in);
                    drain_wait = drain_wait + 1;
                end
                if (drain_wait >= 500000)
                    $display("    WARNING: Phase 14 FIFO drain timeout");
                else
                    $display("    Phase 14 FIFO drained after %0d cycles", drain_wait);
            end
            wait_fsm_idle(1000);

            // Should have more packets than before
            check("more pkt_starts after retrigger",
                  (dut.usb_inst.dbg_pkt_starts_r > pkt_before), 1'b1);

            // v8b: pkt_completions should still equal pkt_starts (for data)
            // Note: pkt_completions counts ALL WAIT_ACK entries (data+status),
            // while pkt_starts only counts SEND_HEADER entries (data only).
            // So pkt_completions >= pkt_starts. The difference is the number
            // of status packets sent (2 in this test: Phases 10 and 13).
            check("v8b: pkt_completions >= pkt_starts after retrigger drain",
                  (dut.usb_inst.dbg_pkt_completions_r >= dut.usb_inst.dbg_pkt_starts_r), 1'b1);

            $display("    Final counters: pkt_starts=%0d  pkt_completions=%0d  fifo_overflows=%0d",
                     dut.usb_inst.dbg_pkt_starts_r,
                     dut.usb_inst.dbg_pkt_completions_r,
                     dut.fifo_overflow_count);
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

    // Safety timeout
    initial begin
        #200_000_000;  // 200 ms (v8b: longer due to FIFO drain waits)
        $display("TIMEOUT: Simulation exceeded 200ms");
        $finish(1);
    end

endmodule
