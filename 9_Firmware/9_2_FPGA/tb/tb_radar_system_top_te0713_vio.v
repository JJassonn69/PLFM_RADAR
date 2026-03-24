`timescale 1ns / 1ps
//
// Testbench for radar_system_top_te0713_vio
//
// Tests the self-test controller + VIO readback logic in simulation.
// VIO IP is stubbed out via `SIMULATION define; testbench drives
// the sim_vio_trigger and sim_vio_status signals directly.
//

`define SIMULATION

module tb_radar_system_top_te0713_vio;

    // =====================================================================
    // DUT signals
    // =====================================================================
    reg        clk = 0;
    wire [3:0] user_led;
    wire [3:0] system_status;

    // Clock: 50 MHz (20 ns period)
    always #10 clk = ~clk;

    // =====================================================================
    // DUT instantiation
    // =====================================================================
    radar_system_top_te0713_vio dut (
        .clk_100m       (clk),
        .user_led       (user_led),
        .system_status  (system_status)
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
                $display("  [FAIL] Test %0d: %0s — got %b, expected %b",
                         test_num, name, actual, expected);
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
                $display("  [FAIL] Test %0d: %0s — got 0x%08h, expected 0x%08h",
                         test_num, name, actual, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // =====================================================================
    // Helper: wait N clock cycles
    // =====================================================================
    task wait_clks;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                @(posedge clk);
        end
    endtask

    // =====================================================================
    // Helper: wait for self-test to complete (with timeout)
    // =====================================================================
    task wait_test_done;
        input integer timeout_clks;
        integer cnt;
        begin
            cnt = 0;
            while (!dut.test_done_latched && cnt < timeout_clks) begin
                @(posedge clk);
                cnt = cnt + 1;
            end
            if (cnt >= timeout_clks)
                $display("  WARNING: Timeout waiting for test_done_latched");
        end
    endtask

    // =====================================================================
    // Main test sequence
    // =====================================================================
    initial begin
        $display("");
        $display("==========================================================");
        $display("  TB: radar_system_top_te0713_vio");
        $display("==========================================================");
        $display("");

        // ---------------------------------------------------------------
        // Test group 1: Power-on reset and auto-test
        // ---------------------------------------------------------------
        $display("--- Group 1: Power-on Reset & Auto-Test ---");

        // Verify initial state
        check("reset_n starts low", dut.reset_n, 1'b0);
        check("test_done_latched starts low", dut.test_done_latched, 1'b0);
        check("all_pass_latched starts low", dut.all_pass_latched, 1'b0);

        // Wait for POR to complete
        // POR counter is 25 bits, but in simulation we can override or just wait.
        // At 50MHz, 2^24 = 16.7M cycles = 335ms. That's 335M ns.
        // For simulation, let's force an early reset release.
        // Force POR counter to near-end
        wait_clks(10);
        force dut.por_cnt = 25'h1FF_FFFE;
        wait_clks(5);
        release dut.por_cnt;

        // Wait for reset to release and auto-trigger to fire
        wait_clks(10);
        check("reset_n is now high", dut.reset_n, 1'b1);

        // Wait for self-test to complete (BRAM + CIC + FFT + ARITH + ADC timeout)
        // ADC timeout is ~1000 cycles
        wait_test_done(2000);

        check("test_done_latched after auto-test", dut.test_done_latched, 1'b1);
        check("self_test_busy is low after test", dut.self_test_busy, 1'b0);

        // Check result_flags_latched: BRAM(0)=PASS, CIC(1)=PASS, FFT(2)=PASS, 
        // ARITH(3)=PASS, ADC(4)=FAIL (no ADC data in sim)
        // Expected: 5'b01111 = 0x0F
        check("BRAM test passed", dut.result_flags_latched[0], 1'b1);
        check("CIC test passed", dut.result_flags_latched[1], 1'b1);
        check("FFT test passed", dut.result_flags_latched[2], 1'b1);
        check("ARITH test passed", dut.result_flags_latched[3], 1'b1);
        check("ADC test failed (no ADC)", dut.result_flags_latched[4], 1'b0);

        // all_pass should be 0 because ADC failed
        check("all_pass_latched is 0 (ADC failed)", dut.all_pass_latched, 1'b0);

        // ---------------------------------------------------------------
        // Test group 2: VIO trigger (re-trigger self-test)
        // ---------------------------------------------------------------
        $display("");
        $display("--- Group 2: VIO Trigger Re-test ---");

        // Set trigger high RIGHT AFTER a clock edge, so the edge detector
        // sees the 0→1 transition at the NEXT posedge.
        @(posedge clk);
        #1;  // small delay after clock edge
        dut.sim_vio_trigger = 1'b1;

        // Next posedge: vio_trigger_test=1, vio_trigger_d=0 → pulse=1
        @(posedge clk);
        // Next posedge: FSM responds to trigger, latches clear
        @(posedge clk);

        check("test_done_latched cleared on re-trigger", dut.test_done_latched, 1'b0);
        check("self_test_busy during re-test", dut.self_test_busy, 1'b1);

        // Release trigger
        #1;
        dut.sim_vio_trigger = 1'b0;

        // Wait for completion
        wait_test_done(2000);
        check("test_done_latched after re-trigger", dut.test_done_latched, 1'b1);
        check("result_flags same after re-test", dut.result_flags_latched, 5'b01111);

        // ---------------------------------------------------------------
        // Test group 3: Heartbeat counter is incrementing
        // ---------------------------------------------------------------
        $display("");
        $display("--- Group 3: Heartbeat & Status ---");

        begin : hb_check
            reg [31:0] hb1, hb2;
            hb1 = dut.hb_counter;
            wait_clks(100);
            hb2 = dut.hb_counter;
            check("heartbeat counter increments", (hb2 > hb1) ? 1'b1 : 1'b0, 1'b1);
        end

        // Check status outputs
        // system_status[0] = self_test_busy (should be 0)
        // system_status[1] = test_done_latched (should be 1)
        // system_status[2] = all_pass_latched (should be 0)
        check("status[0] = busy (low)", system_status[0], 1'b0);
        check("status[1] = done (high)", system_status[1], 1'b1);
        check("status[2] = all_pass (low)", system_status[2], 1'b0);

        // ---------------------------------------------------------------
        // Test group 4: Version readback
        // ---------------------------------------------------------------
        $display("");
        $display("--- Group 4: Version ---");

        // VERSION_MAJOR=0, VERSION_MINOR=3 → packed = {0[3:0], 3[3:0]} = 8'h03
        check_val("version packed byte", {dut.VERSION_MAJOR[3:0], dut.VERSION_MINOR[3:0]}, 8'h03);

        // ---------------------------------------------------------------
        // Test group 5: Edge detection for VIO trigger
        // ---------------------------------------------------------------
        $display("");
        $display("--- Group 5: VIO Edge Detection ---");

        // Hold trigger high — should only produce one pulse
        dut.sim_vio_trigger = 1'b1;
        wait_clks(2);
        // vio_trigger_pulse should be high for exactly 1 cycle
        // After the first cycle, vio_trigger_d catches up
        check("vio_trigger_d follows after 2 clks", dut.vio_trigger_d, 1'b1);
        // Now pulse should be gone (level=1, delayed=1 → no edge)
        check("no repeated pulse while held high", dut.vio_trigger_pulse, 1'b0);

        dut.sim_vio_trigger = 1'b0;
        wait_clks(2);

        // ---------------------------------------------------------------
        // Summary
        // ---------------------------------------------------------------
        $display("");
        $display("==========================================================");
        $display("  Results: %0d passed, %0d failed / %0d total",
                 pass_count, fail_count, test_num);
        $display("==========================================================");

        if (fail_count > 0)
            $display("  *** FAILURES DETECTED ***");
        else
            $display("  ALL TESTS PASSED");

        $display("");
        $finish;
    end

    // Timeout watchdog
    initial begin
        #5_000_000;  // 5ms
        $display("ERROR: Simulation timeout!");
        $finish;
    end

    // VCD dump for waveform viewing
    initial begin
        $dumpfile("tb_radar_system_top_te0713_vio.vcd");
        $dumpvars(0, tb_radar_system_top_te0713_vio);
    end

endmodule
