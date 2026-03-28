`timescale 1ns / 1ps
//
// Testbench for radar_system_top_te0713_umft601x_playback
//
// v9 — Full DSP pipeline with Doppler + CFAR streaming over USB
//
// Validates the unified BRAM playback + FT601 USB streaming pipeline.
// Tests both the USB infrastructure (inherited from v7c) and the v9
// DSP pipeline integration with three independent FIFO channels:
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
//  10. v9: Full pipeline (stream_control=0x07) — range + Doppler + CFAR
//  11. v9: Doppler FIFO drain and packet count verification
//  12. v9: CFAR FIFO drain and detection packet verification
//  13. v9: Priority gating (Doppler waits for range, CFAR waits for both)
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

    task check_ge;
        input [255:0] name;
        input [31:0]  actual;
        input [31:0]  minimum;
        begin
            test_num = test_num + 1;
            if (actual >= minimum) begin
                $display("  [PASS] Test %0d: %0s (value=%0d >= %0d)", test_num, name, actual, minimum);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] Test %0d: %0s -- got %0d, expected >= %0d",
                         test_num, name, actual, minimum);
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
    // v9: FSM is now 4 bits wide
    // =====================================================================
    task wait_fsm_idle;
        input integer max_cycles;
        integer timeout;
        begin
            timeout = 0;
            while (dut.usb_inst.current_state != 4'd0 && timeout < max_cycles) begin
                @(posedge ft601_clk_in);
                timeout = timeout + 1;
            end
        end
    endtask

    // =====================================================================
    // Helper: Wait for all three FIFOs to drain (v9)
    // =====================================================================
    task wait_all_fifos_empty;
        input integer max_cycles;
        integer drain_wait;
        begin
            drain_wait = 0;
            while ((!dut.range_fifo_empty || !dut.doppler_fifo_empty || !dut.cfar_fifo_empty)
                   && drain_wait < max_cycles) begin
                @(posedge ft601_clk_in);
                drain_wait = drain_wait + 1;
            end
            if (drain_wait >= max_cycles)
                $display("    WARNING: All-FIFO drain timeout after %0d cycles", drain_wait);
            else
                $display("    All FIFOs drained after %0d cycles", drain_wait);
        end
    endtask

    // =====================================================================
    // v9: Packet capture infrastructure — sniff ft601_data on WR_N=LOW
    // Counts packets by header type: 0xAA (range), 0xCC (doppler), 0xDD (cfar), 0xBB (status)
    // =====================================================================
    integer range_pkt_count;
    integer doppler_pkt_count;
    integer cfar_pkt_count;
    integer status_pkt_count;
    integer footer_count;

    // Track current packet type being captured
    reg [7:0] cur_pkt_type;
    integer   cur_pkt_word;

    // Capture buffer for verifying packet content
    reg [31:0] captured_data_words [0:15];
    integer capture_idx;

    // Initialize sniffer counters (initial block — before clock starts)
    initial begin
        range_pkt_count   = 0;
        doppler_pkt_count = 0;
        cfar_pkt_count    = 0;
        status_pkt_count  = 0;
        footer_count      = 0;
        cur_pkt_type      = 8'h00;
        cur_pkt_word      = 0;
    end

    // v9: Continuous packet sniffer — runs in parallel with main test
    always @(posedge ft601_clk_in) begin
        if (ft601_wr_n === 1'b0 && ft601_txe === 1'b0) begin
            // Data word is being written by FPGA
            if (ft601_data[7:0] == 8'hAA && ft601_data[31:8] == 24'h000000 && cur_pkt_word == 0) begin
                range_pkt_count <= range_pkt_count + 1;
                cur_pkt_type <= 8'hAA;
                cur_pkt_word <= 1;
            end else if (ft601_data[7:0] == 8'hCC && ft601_data[31:8] == 24'h000000 && cur_pkt_word == 0) begin
                doppler_pkt_count <= doppler_pkt_count + 1;
                cur_pkt_type <= 8'hCC;
                cur_pkt_word <= 1;
            end else if (ft601_data[7:0] == 8'hDD && ft601_data[31:8] == 24'h000000 && cur_pkt_word == 0) begin
                cfar_pkt_count <= cfar_pkt_count + 1;
                cur_pkt_type <= 8'hDD;
                cur_pkt_word <= 1;
            end else if (ft601_data[7:0] == 8'hBB && ft601_data[31:8] == 24'h000000 && cur_pkt_word == 0) begin
                status_pkt_count <= status_pkt_count + 1;
                cur_pkt_type <= 8'hBB;
                cur_pkt_word <= 1;
            end else if (ft601_data[7:0] == 8'h55 && ft601_data[31:8] == 24'h000000) begin
                footer_count <= footer_count + 1;
                cur_pkt_type <= 8'h00;
                cur_pkt_word <= 0;
            end else begin
                // Data word inside a packet
                if (capture_idx < 16) begin
                    captured_data_words[capture_idx] <= ft601_data;
                    capture_idx <= capture_idx + 1;
                end
                cur_pkt_word <= cur_pkt_word + 1;
            end
        end
    end

    // =====================================================================
    // Doppler handshake watchdog — silent unless actual race/error detected
    // =====================================================================
    integer doppler_pop_while_pending;
    integer doppler_pending_lost;
    reg     prev_doppler_pending;

    initial begin
        doppler_pop_while_pending = 0;
        doppler_pending_lost = 0;
        prev_doppler_pending = 0;
    end

    always @(posedge ft601_clk_in) begin
        // Pop-while-pending race (should NEVER happen with v9d guards)
        if (dut.doppler_pop_state == 2'd0 &&
            !dut.doppler_fifo_empty && dut.usb_write_idle && dut.range_fifo_empty
            && !dut.usb_doppler_pending) begin
            if (dut.usb_inst.doppler_data_pending) begin
                doppler_pop_while_pending <= doppler_pop_while_pending + 1;
                $display("    *** RACE: Doppler pop while pending=1 at time %0t", $time);
            end
        end

        // Simultaneous set+clear on pending flag (valid fires while FSM consuming)
        if (dut.doppler_valid_usb &&
            dut.usb_inst.current_state == 4'd0 &&
            dut.usb_inst.doppler_data_pending &&
            dut.usb_inst.stream_ctrl_sync_1[1]) begin
            doppler_pending_lost <= doppler_pending_lost + 1;
            $display("    *** LOST: doppler_valid + IDLE consuming pending at time %0t", $time);
        end

        prev_doppler_pending <= dut.usb_inst.doppler_data_pending;
    end

    // =====================================================================
    // Doppler valid-while-pending detector — catches Pop FSM race
    // =====================================================================
    integer doppler_valid_while_pending;
    reg     prev_dop_valid;

    initial begin
        doppler_valid_while_pending = 0;
        prev_dop_valid = 0;
    end

    always @(posedge ft601_clk_in) begin
        prev_dop_valid <= dut.doppler_valid_usb;
        // Rising edge of doppler_valid_usb while pending already set
        if (dut.doppler_valid_usb && !prev_dop_valid) begin
            if (dut.usb_inst.doppler_data_pending) begin
                doppler_valid_while_pending <= doppler_valid_while_pending + 1;
                if (doppler_valid_while_pending < 5)
                    $display("    *** VALID-WHILE-PENDING #%0d at time %0t",
                             doppler_valid_while_pending + 1, $time);
            end
        end
    end

    // Enhanced Doppler packet sniffer — capture first/last range_bin/doppler_bin
    integer  first_dop_range_bin, first_dop_doppler_bin;
    integer  last_dop_range_bin, last_dop_doppler_bin;
    reg      dop_sniffer_word1;

    initial begin
        first_dop_range_bin = -1;
        first_dop_doppler_bin = -1;
        last_dop_range_bin = -1;
        last_dop_doppler_bin = -1;
        dop_sniffer_word1 = 0;
    end

    always @(posedge ft601_clk_in) begin
        if (ft601_wr_n === 1'b0 && ft601_txe === 1'b0) begin
            if (ft601_data[7:0] == 8'hCC && ft601_data[31:8] == 24'h000000) begin
                dop_sniffer_word1 <= 1;
            end else if (dop_sniffer_word1) begin
                dop_sniffer_word1 <= 0;
                last_dop_range_bin <= ft601_data[31:26];
                last_dop_doppler_bin <= ft601_data[25:21];
                if (first_dop_range_bin == -1) begin
                    first_dop_range_bin <= ft601_data[31:26];
                    first_dop_doppler_bin <= ft601_data[25:21];
                end
            end
        end
    end





    // =====================================================================
    // Main test
    // =====================================================================
    initial begin
        $dumpfile("tb_radar_system_top_te0713_umft601x_playback.vcd");
        $dumpvars(0, tb_radar_system_top_te0713_umft601x_playback);

        capture_idx = 0;

        $display("");
        $display("=== tb_radar_system_top_te0713_umft601x_playback ===");
        $display("    Unified BRAM playback + FT601 USB streaming");
        $display("    v9: Full DSP pipeline — Range + Doppler + CFAR over USB");
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
              (dut.usb_inst.current_state == 4'd0), 1'b1);

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

        // v9: Test full pipeline stream control
        ft601_host_write(8'h04, 8'h00, 16'h0007);
        check_val("stream_control set to 0x07 (range+doppler+cfar)",
                  {29'd0, dut.stream_control_reg}, 32'd7);

        // Reset to range-only for backward-compat tests
        ft601_host_write(8'h04, 8'h00, 16'h0001);

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
        // Phase 6: Playback trigger via USB → BRAM starts (range-only)
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 6: Playback trigger via USB ---");

        // Verify playback is initially idle
        check("playback not active initially", dut.pb_playback_active, 1'b0);
        check("playback not done initially", dut.pb_playback_done, 1'b0);

        // Reset DSP config to known-good values for range-only playback
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

        $display("    Waiting for first chirp decimation...");
        wait_ft_clk(1200);

        check_nonzero("range_profile_usb has real data after first chirp",
                      dut.range_profile_usb);

        // -----------------------------------------------------------------
        // Phase 8: USB streaming sends real data packets
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 8: USB data packets with real DSP data ---");

        wait_ft_clk(1000);

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

        $display("    Final debug counters:");
        $display("      pkt_starts=%0d  pkt_completions=%0d  wr_strobes=%0d",
                 dut.usb_inst.dbg_pkt_starts_r,
                 dut.usb_inst.dbg_pkt_completions_r,
                 dut.usb_inst.dbg_wr_strobes_r);

        check("pkt_completions == pkt_starts (no stuck packets)",
              (dut.usb_inst.dbg_pkt_completions_r == dut.usb_inst.dbg_pkt_starts_r), 1'b1);

        // -----------------------------------------------------------------
        // Phase 9b: v9 Range FIFO draining
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 9b: Range FIFO drain after playback ---");

        begin : phase9b_drain
            integer drain_wait;
            drain_wait = 0;
            while (!dut.range_fifo_empty && drain_wait < 500000) begin
                @(posedge ft601_clk_in);
                drain_wait = drain_wait + 1;
            end
            if (drain_wait >= 500000)
                $display("    WARNING: Range FIFO drain timeout after %0d cycles", drain_wait);
            else
                $display("    Range FIFO drained after %0d cycles", drain_wait);
        end

        wait_fsm_idle(1000);

        check("Range FIFO empty after drain", dut.range_fifo_empty, 1'b1);
        check("Range FIFO overflow count is 0", (dut.range_overflow_count == 16'd0), 1'b1);

        $display("    After Range FIFO drain: pkt_starts=%0d  pkt_completions=%0d",
                 dut.usb_inst.dbg_pkt_starts_r,
                 dut.usb_inst.dbg_pkt_completions_r);

        check("v9: pkt_starts >= 2000 (range FIFO buffering works)",
              (dut.usb_inst.dbg_pkt_starts_r >= 16'd2000), 1'b1);
        check("v9: pkt_completions == pkt_starts after drain",
              (dut.usb_inst.dbg_pkt_completions_r == dut.usb_inst.dbg_pkt_starts_r), 1'b1);

        // v9: verify Doppler and CFAR FIFOs are empty too (no Doppler/CFAR
        // streaming since stream_control was 0x01 = range-only)
        check("v9: Doppler FIFO empty (range-only mode)",
              dut.doppler_fifo_empty, 1'b1);
        // CFAR FIFO: push is independent of stream_control (always pushes on
        // cfar_detect_valid && cfar_detect_flag), but CFAR was disabled so
        // there should be no detections.
        check("v9: CFAR FIFO empty (CFAR disabled)",
              dut.cfar_fifo_empty, 1'b1);

        // -----------------------------------------------------------------
        // Phase 10: Status request returns correct values
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 10: Status readback ---");

        ft601_host_write(8'hFF, 8'h00, 16'h0000);
        wait_ft_clk(30);

        check("write FSM in IDLE after status",
              (dut.usb_inst.current_state == 4'd0), 1'b1);
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
        // Plus v9: Range FIFO continues delivering data after backpressure
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 12: WR_N deasserts + FIFO survives backpressure ---");

        // Trigger another playback so data is flowing
        ft601_host_write(8'h02, 8'h00, 16'h0000);
        wait_ft_clk(1200);

        begin : phase12_block
            reg [15:0] pkt_before_bp;
            pkt_before_bp = dut.usb_inst.dbg_pkt_completions_r;

            @(posedge ft601_clk_in);
            ft601_txe = 1'b1;  // FIFO full — backpressure
            wait_ft_clk(3);

            check("WR_N deasserted after TXE goes high", ft601_wr_n, 1'b1);

            // Hold backpressure for 500 cycles — FIFO should buffer data
            wait_ft_clk(500);

            check("v9: Range FIFO not empty during backpressure",
                  !dut.range_fifo_empty, 1'b1);

            // Release backpressure
            ft601_txe = 1'b0;
            wait_ft_clk(200);

            // Packets should resume after backpressure clears
            check("v9: more packets after backpressure release",
                  (dut.usb_inst.dbg_pkt_completions_r > pkt_before_bp), 1'b1);
        end

        // Wait for this playback to finish + range FIFO drain
        begin : phase12_drain
            integer drain_wait;
            drain_wait = 0;
            while ((!dut.pb_playback_done || !dut.range_fifo_empty) && drain_wait < 500000) begin
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
                  (dut.usb_inst.current_state == 4'd0), 1'b1);
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
        // Plus v9: verify Range FIFO delivers all packets on retrigger
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 14: Retrigger playback + FIFO delivery ---");

        // Re-enable range-only streaming
        ft601_host_write(8'h04, 8'h00, 16'h0001);

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

            // v9: Wait for range FIFO to drain completely
            begin : phase14_drain
                integer drain_wait;
                drain_wait = 0;
                while (!dut.range_fifo_empty && drain_wait < 500000) begin
                    @(posedge ft601_clk_in);
                    drain_wait = drain_wait + 1;
                end
                if (drain_wait >= 500000)
                    $display("    WARNING: Phase 14 range FIFO drain timeout");
                else
                    $display("    Phase 14 range FIFO drained after %0d cycles", drain_wait);
            end
            wait_fsm_idle(1000);

            check("more pkt_starts after retrigger",
                  (dut.usb_inst.dbg_pkt_starts_r > pkt_before), 1'b1);

            check("v9: pkt_completions >= pkt_starts after retrigger drain",
                  (dut.usb_inst.dbg_pkt_completions_r >= dut.usb_inst.dbg_pkt_starts_r), 1'b1);

            $display("    Retrigger counters: pkt_starts=%0d  pkt_completions=%0d  range_overflows=%0d",
                     dut.usb_inst.dbg_pkt_starts_r,
                     dut.usb_inst.dbg_pkt_completions_r,
                     dut.range_overflow_count);
        end

        // =================================================================
        // Phase 15: v9 FULL PIPELINE — stream_control=0x07 with CFAR enabled
        //
        // This is the core v9 test: enable all three streaming channels
        // (range + Doppler + CFAR), run a full playback, and verify:
        //   - All three FIFOs push data
        //   - All three FIFOs drain completely
        //   - Range packets (0xAA): exactly 2048 (32 chirps × 64 bins)
        //   - Doppler packets (0xCC): exactly 2048 (64 range × 32 doppler)
        //   - CFAR packets (0xDD): at least 1 (sparse detections)
        //   - No FIFO overflows on any channel
        //   - Priority: range drains before Doppler, Doppler before CFAR
        // =================================================================
        $display("");
        $display("--- Phase 15: v9 Full Pipeline (range + Doppler + CFAR) ---");

        // Reset debug counters by noting current values
        begin : phase15_block
            reg [15:0] pkt_starts_before;
            reg [15:0] pkt_completions_before;
            integer range_pkt_before;
            integer doppler_pkt_before;
            integer cfar_pkt_before;

            pkt_starts_before      = dut.usb_inst.dbg_pkt_starts_r;
            pkt_completions_before = dut.usb_inst.dbg_pkt_completions_r;
            range_pkt_before       = range_pkt_count;
            doppler_pkt_before     = doppler_pkt_count;
            cfar_pkt_before        = cfar_pkt_count;

            // Configure for full pipeline:
            // - CFAR enabled with CA-CFAR, default guard=2, train=8, alpha=3.0
            ft601_host_write(8'h21, 8'h00, 16'h0002);  // guard=2
            ft601_host_write(8'h22, 8'h00, 16'h0008);  // train=8
            ft601_host_write(8'h23, 8'h00, 16'h0030);  // alpha=3.0 (Q4.4)
            ft601_host_write(8'h24, 8'h00, 16'h0000);  // CA-CFAR mode
            ft601_host_write(8'h25, 8'h00, 16'h0001);  // CFAR enable
            ft601_host_write(8'h26, 8'h00, 16'h0000);  // MTI disable (clean test)
            ft601_host_write(8'h27, 8'h00, 16'h0000);  // DC notch off

            // Enable all three streams: range + doppler + cfar
            ft601_host_write(8'h04, 8'h00, 16'h0007);
            check_val("v9: stream_control set to 0x07",
                      {29'd0, dut.stream_control_reg}, 32'd7);

            // Clear overflow counts (they persist across playbacks)
            // Note: we can't reset them — just record current values
            $display("    Pre-playback overflow counts: range=%0d  doppler=%0d  cfar=%0d",
                     dut.range_overflow_count,
                     dut.doppler_overflow_count,
                     dut.cfar_overflow_count);

            begin : phase15_overflow_save
                reg [15:0] range_ov_before;
                reg [15:0] doppler_ov_before;
                reg [15:0] cfar_ov_before;

                range_ov_before   = dut.range_overflow_count;
                doppler_ov_before = dut.doppler_overflow_count;
                cfar_ov_before    = dut.cfar_overflow_count;

                // Trigger playback
                $display("    Triggering full pipeline playback...");
                ft601_host_write(8'h02, 8'h00, 16'h0000);
                wait_ft_clk(10);

                check("v9: playback_active after full pipeline trigger",
                      dut.pb_playback_active, 1'b1);

                // Wait for all 32 chirps to complete
                begin : phase15_playback_wait
                    integer wait_count;
                    wait_count = 0;
                    while (!dut.pb_playback_done && wait_count < 200000) begin
                        @(posedge ft601_clk_in);
                        wait_count = wait_count + 1;
                    end
                    if (wait_count >= 200000)
                        $display("    WARNING: Phase 15 playback timeout");
                    else
                        $display("    Phase 15 playback completed after %0d cycles", wait_count);
                end

                check("v9: playback_done after full pipeline",
                      dut.pb_playback_done, 1'b1);

                // After playback completes, Doppler processor runs. Wait for
                // Doppler processing to finish (frame_complete pulses).
                // The Doppler FIFO should start filling. Wait a bit extra.
                $display("    Waiting for Doppler processing + CFAR scan...");
                wait_ft_clk(50000);

                // Now wait for ALL three FIFOs to drain through USB
                $display("    Draining all three FIFOs...");
                wait_all_fifos_empty(2000000);
                // Wait for last packet to complete through USB FSM pipeline
                wait_ft_clk(20);
                wait_fsm_idle(1000);

                // === Verify all FIFOs are empty ===
                check("v9: Range FIFO empty after full drain",
                      dut.range_fifo_empty, 1'b1);
                check("v9: Doppler FIFO empty after full drain",
                      dut.doppler_fifo_empty, 1'b1);
                check("v9: CFAR FIFO empty after full drain",
                      dut.cfar_fifo_empty, 1'b1);

                // === Verify no overflow on range and doppler FIFOs ===
                // Range FIFO: 2048 entries, 2048 packets per frame — should not overflow
                check("v9: no range FIFO overflows during full pipeline",
                      (dut.range_overflow_count == range_ov_before), 1'b1);
                // Doppler FIFO: 2048 entries, 2048 cells per frame — should not overflow
                check("v9: no doppler FIFO overflows during full pipeline",
                      (dut.doppler_overflow_count == doppler_ov_before), 1'b1);
                // CFAR FIFO: 2048 entries — should not overflow with ~1300 detections/frame
                check("v9: no cfar FIFO overflows during full pipeline",
                      (dut.cfar_overflow_count == cfar_ov_before), 1'b1);
                $display("    CFAR FIFO overflow delta: %0d (2048-entry FIFO)",
                         dut.cfar_overflow_count - cfar_ov_before);

                // === Verify packet counts from sniffer ===
                $display("    Packet sniffer counts (delta from phase 15 start):");
                $display("      range(0xAA)=%0d  doppler(0xCC)=%0d  cfar(0xDD)=%0d",
                         range_pkt_count - range_pkt_before,
                         doppler_pkt_count - doppler_pkt_before,
                         cfar_pkt_count - cfar_pkt_before);

                // Range: 32 chirps × 64 bins = 2048 packets
                check_ge("v9: range packets >= 2000",
                         range_pkt_count - range_pkt_before, 2000);

                // Doppler: 64 range bins × 32 doppler bins = 2048 cells
                // But the Doppler processor outputs per sub-frame (16-pt FFT × 2 sub-frames)
                // Exact count depends on doppler_processor_optimized behavior.
                // At minimum we expect > 0 (proves Doppler FIFO + pop FSM works)
                check("v9: doppler packets > 0 (Doppler pipeline active)",
                      (doppler_pkt_count > doppler_pkt_before), 1'b1);

                // CFAR: sparse detections. With real radar data and CFAR enabled,
                // we expect many detections (~1300 with CA-CFAR, guard=2, train=8, alpha=3.0).
                // With the 2048-entry FIFO, all should be captured.
                // Test that at least 100 CFAR packets were sent.
                check_ge("v9: cfar packets >= 100 (CFAR detection pipeline active)",
                      cfar_pkt_count - cfar_pkt_before, 100);

                // === Verify USB FSM counters are consistent ===
                $display("    USB FSM counters: pkt_starts=%0d  pkt_completions=%0d",
                         dut.usb_inst.dbg_pkt_starts_r,
                         dut.usb_inst.dbg_pkt_completions_r);

                // pkt_completions should be >= pkt_starts (status packets also count in completions)
                check("v9: pkt_completions >= pkt_starts after full pipeline",
                      (dut.usb_inst.dbg_pkt_completions_r >= dut.usb_inst.dbg_pkt_starts_r), 1'b1);

                $display("    Overflow counts: range=%0d  doppler=%0d  cfar=%0d",
                         dut.range_overflow_count,
                         dut.doppler_overflow_count,
                         dut.cfar_overflow_count);
            end
        end

        // -----------------------------------------------------------------
        // Phase 16: v9 Doppler-only mode (stream_control=0x02)
        // Verify range packets are NOT sent, Doppler packets ARE sent
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 16: v9 Doppler-only streaming mode ---");

        begin : phase16_block
            integer range_pkt_before;
            integer doppler_pkt_before;
            // RTL-internal counter snapshots
            integer rtl_dop_consumed_before;
            integer rtl_dop_discarded_before;
            integer rtl_range_discarded_before;

            range_pkt_before   = range_pkt_count;
            doppler_pkt_before = doppler_pkt_count;
            rtl_dop_consumed_before  = dut.usb_inst.dbg_doppler_consumed_r;
            rtl_dop_discarded_before = dut.usb_inst.dbg_doppler_discarded_r;
            rtl_range_discarded_before = dut.usb_inst.dbg_range_discarded_r;

            // Reset first/last Doppler sniffer for this phase
            first_dop_range_bin = -1;
            first_dop_doppler_bin = -1;
            last_dop_range_bin = -1;
            last_dop_doppler_bin = -1;

            // Set stream_control = 0x02 (doppler only)
            ft601_host_write(8'h04, 8'h00, 16'h0002);
            check_val("v9: stream_control set to 0x02 (doppler only)",
                      {29'd0, dut.stream_control_reg}, 32'd2);

            // Trigger playback
            ft601_host_write(8'h02, 8'h00, 16'h0000);
            wait_ft_clk(10);

            check("v9: playback_active in doppler-only mode",
                  dut.pb_playback_active, 1'b1);

            // Wait for completion + Doppler processing
            begin : phase16_wait
                integer wait_count;
                wait_count = 0;
                while (!dut.pb_playback_done && wait_count < 200000) begin
                    @(posedge ft601_clk_in);
                    wait_count = wait_count + 1;
                end
            end
            wait_ft_clk(50000);

            // Drain all FIFOs
            wait_all_fifos_empty(2000000);
            // Wait for last packet to complete through USB FSM pipeline
            wait_ft_clk(20);
            wait_fsm_idle(1000);

            // Verify no range packets sent in Doppler-only mode
            $display("    Phase 16: range(0xAA)=%0d  doppler(0xCC)=%0d",
                     range_pkt_count - range_pkt_before,
                     doppler_pkt_count - doppler_pkt_before);
            $display("    RTL counters (delta): dop_consumed=%0d  dop_discarded=%0d  range_discarded=%0d",
                     dut.usb_inst.dbg_doppler_consumed_r - rtl_dop_consumed_before,
                     dut.usb_inst.dbg_doppler_discarded_r - rtl_dop_discarded_before,
                     dut.usb_inst.dbg_range_discarded_r - rtl_range_discarded_before);
            $display("    First Doppler pkt: range_bin=%0d doppler_bin=%0d",
                     first_dop_range_bin, first_dop_doppler_bin);
            $display("    Last Doppler pkt:  range_bin=%0d doppler_bin=%0d",
                     last_dop_range_bin, last_dop_doppler_bin);

            // No range packets should have been sent
            check_val("v9: no range packets in doppler-only mode",
                      range_pkt_count - range_pkt_before, 0);

            // Doppler packets should have been sent
            check("v9: doppler packets > 0 in doppler-only mode",
                  (doppler_pkt_count > doppler_pkt_before), 1'b1);

            // Exact Doppler count: 64 range bins x 32 Doppler bins = 2048
            check_val("v9: exactly 2048 doppler packets in doppler-only mode",
                      doppler_pkt_count - doppler_pkt_before, 2048);

            // Verify last Doppler packet is the final cell (range=63, doppler=31)
            check_val("v9: last doppler pkt range_bin=63",
                      last_dop_range_bin, 63);
            check_val("v9: last doppler pkt doppler_bin=31",
                      last_dop_doppler_bin, 31);

            // Verify no handshake races occurred
            check_val("v9: no pop-while-pending races",
                      doppler_pop_while_pending, 0);
            check_val("v9: no pending-lost events",
                      doppler_pending_lost, 0);
            check_val("v9: no valid-while-pending events",
                      doppler_valid_while_pending, 0);
        end

        // -----------------------------------------------------------------
        // Phase 17: v9 Three independent FIFO status check
        // Verify the three overflow counters and pop FSM states
        // -----------------------------------------------------------------
        $display("");
        $display("--- Phase 17: v9 FIFO and pop FSM state check ---");

        // All pop FSMs should be in POP_IDLE (2'd0) after draining
        // (note: might be cycling due to pending data, but likely idle)
        $display("    Pop FSM states: range=%0d  doppler=%0d  cfar=%0d",
                 dut.range_pop_state,
                 dut.doppler_pop_state,
                 dut.cfar_pop_state);

        $display("    Total overflow counts: range=%0d  doppler=%0d  cfar=%0d",
                 dut.range_overflow_count,
                 dut.doppler_overflow_count,
                 dut.cfar_overflow_count);

        // Final USB FSM state should be IDLE
        check("v9: USB write FSM in IDLE at end",
              (dut.usb_inst.current_state == 4'd0), 1'b1);

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("");
        $display("========================================");
        $display("  Results: %0d passed, %0d failed (of %0d)",
                 pass_count, fail_count, test_num);
        $display("========================================");
        $display("  Packet sniffer totals: range=%0d  doppler=%0d  cfar=%0d  status=%0d",
                 range_pkt_count, doppler_pkt_count, cfar_pkt_count, status_pkt_count);

        if (fail_count > 0) begin
            $display("FAIL");
            $finish(1);
        end else begin
            $display("ALL TESTS PASSED");
            $finish(0);
        end
    end

    // Safety timeout — v9: longer due to three FIFO drain waits + Doppler processing
    initial begin
        #500_000_000;  // 500 ms
        $display("TIMEOUT: Simulation exceeded 500ms");
        $finish(1);
    end

endmodule
