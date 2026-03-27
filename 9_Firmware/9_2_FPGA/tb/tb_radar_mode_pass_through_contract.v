`timescale 1ns / 1ps

module tb_radar_mode_pass_through_contract;

    localparam CLK_PERIOD = 10;
    localparam CHIRPS_PER_ELEVATION = 32;
    localparam ELEVATIONS_PER_AZIMUTH = 31;
    localparam AZIMUTHS_PER_SCAN = 50;

    reg clk;
    reg reset_n;
    reg [1:0] mode;
    reg stm32_new_chirp;
    reg stm32_new_elevation;
    reg stm32_new_azimuth;
    reg trigger;

    reg [15:0] cfg_long_chirp_cycles;
    reg [15:0] cfg_long_listen_cycles;
    reg [15:0] cfg_guard_cycles;
    reg [15:0] cfg_short_chirp_cycles;
    reg [15:0] cfg_short_listen_cycles;
    reg [5:0] cfg_chirps_per_elev;

    wire use_long_chirp;
    wire mc_new_chirp;
    wire mc_new_elevation;
    wire mc_new_azimuth;
    wire [5:0] chirp_count;
    wire [5:0] elevation_count;
    wire [5:0] azimuth_count;
    wire scanning;
    wire scan_complete;

    integer pass_count;
    integer fail_count;
    integer test_num;

    integer i;
    integer j;
    integer k;

    reg mc_new_chirp_prev;
    reg mc_new_elevation_prev;
    reg mc_new_azimuth_prev;
    integer chirp_out_toggles;
    integer elev_out_toggles;
    integer az_out_toggles;
    integer scan_complete_pulses;

    always #(CLK_PERIOD/2) clk = ~clk;

    radar_mode_controller #(
        .CHIRPS_PER_ELEVATION(CHIRPS_PER_ELEVATION),
        .ELEVATIONS_PER_AZIMUTH(ELEVATIONS_PER_AZIMUTH),
        .AZIMUTHS_PER_SCAN(AZIMUTHS_PER_SCAN),
        .LONG_CHIRP_CYCLES(16'd10),
        .LONG_LISTEN_CYCLES(16'd10),
        .GUARD_CYCLES(16'd10),
        .SHORT_CHIRP_CYCLES(16'd2),
        .SHORT_LISTEN_CYCLES(16'd10)
    ) dut (
        .clk(clk),
        .reset_n(reset_n),
        .mode(mode),
        .stm32_new_chirp(stm32_new_chirp),
        .stm32_new_elevation(stm32_new_elevation),
        .stm32_new_azimuth(stm32_new_azimuth),
        .trigger(trigger),
        .cfg_long_chirp_cycles(cfg_long_chirp_cycles),
        .cfg_long_listen_cycles(cfg_long_listen_cycles),
        .cfg_guard_cycles(cfg_guard_cycles),
        .cfg_short_chirp_cycles(cfg_short_chirp_cycles),
        .cfg_short_listen_cycles(cfg_short_listen_cycles),
        .cfg_chirps_per_elev(cfg_chirps_per_elev),
        .use_long_chirp(use_long_chirp),
        .mc_new_chirp(mc_new_chirp),
        .mc_new_elevation(mc_new_elevation),
        .mc_new_azimuth(mc_new_azimuth),
        .chirp_count(chirp_count),
        .elevation_count(elevation_count),
        .azimuth_count(azimuth_count),
        .scanning(scanning),
        .scan_complete(scan_complete)
    );

    task check;
        input condition;
        input [255:0] label;
        begin
            test_num = test_num + 1;
            if (condition) begin
                pass_count = pass_count + 1;
                $display("[PASS] %0d: %0s", test_num, label);
            end else begin
                fail_count = fail_count + 1;
                $display("[FAIL] %0d: %0s", test_num, label);
            end
        end
    endtask

    task do_chirp_toggle;
        begin
            stm32_new_chirp = ~stm32_new_chirp;
            @(posedge clk);
            @(posedge clk);
            #1;
        end
    endtask

    task do_elevation_toggle;
        begin
            stm32_new_elevation = ~stm32_new_elevation;
            @(posedge clk);
            @(posedge clk);
            #1;
        end
    endtask

    task do_azimuth_toggle;
        begin
            stm32_new_azimuth = ~stm32_new_azimuth;
            @(posedge clk);
            @(posedge clk);
            #1;
        end
    endtask

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            mc_new_chirp_prev <= 1'b0;
            mc_new_elevation_prev <= 1'b0;
            mc_new_azimuth_prev <= 1'b0;
        end else begin
            if (mc_new_chirp !== mc_new_chirp_prev)
                chirp_out_toggles <= chirp_out_toggles + 1;
            if (mc_new_elevation !== mc_new_elevation_prev)
                elev_out_toggles <= elev_out_toggles + 1;
            if (mc_new_azimuth !== mc_new_azimuth_prev)
                az_out_toggles <= az_out_toggles + 1;
            if (scan_complete)
                scan_complete_pulses <= scan_complete_pulses + 1;

            mc_new_chirp_prev <= mc_new_chirp;
            mc_new_elevation_prev <= mc_new_elevation;
            mc_new_azimuth_prev <= mc_new_azimuth;
        end
    end

    initial begin
        $dumpfile("tb_radar_mode_pass_through_contract.vcd");
        $dumpvars(0, tb_radar_mode_pass_through_contract);

        clk = 0;
        reset_n = 0;
        mode = 2'b11;
        stm32_new_chirp = 0;
        stm32_new_elevation = 0;
        stm32_new_azimuth = 0;
        trigger = 0;

        cfg_long_chirp_cycles = 16'd10;
        cfg_long_listen_cycles = 16'd10;
        cfg_guard_cycles = 16'd10;
        cfg_short_chirp_cycles = 16'd2;
        cfg_short_listen_cycles = 16'd10;
        cfg_chirps_per_elev = CHIRPS_PER_ELEVATION;

        pass_count = 0;
        fail_count = 0;
        test_num = 0;

        chirp_out_toggles = 0;
        elev_out_toggles = 0;
        az_out_toggles = 0;
        scan_complete_pulses = 0;

        repeat (4) @(posedge clk);
        reset_n = 1;
        @(posedge clk);
        #1;

        mode = 2'b00;
        @(posedge clk);
        #1;

        // One azimuth worth of pass-through traffic: 31 elevations x 32 chirps
        for (i = 0; i < ELEVATIONS_PER_AZIMUTH; i = i + 1) begin
            for (j = 0; j < CHIRPS_PER_ELEVATION; j = j + 1) begin
                do_chirp_toggle();
            end

            check(chirp_count == 6'd0,
                  "chirp_count wraps to 0 after 32 chirp toggles");

            do_elevation_toggle();
        end

        check(elev_out_toggles == ELEVATIONS_PER_AZIMUTH,
              "mc_new_elevation toggles exactly 31 times per azimuth");
        check(chirp_out_toggles == CHIRPS_PER_ELEVATION * ELEVATIONS_PER_AZIMUTH,
              "mc_new_chirp toggles exactly 32*31 times per azimuth");
        check(elevation_count == 6'd0,
              "elevation_count wraps to 0 after 31 elevation toggles");

        do_azimuth_toggle();
        check(azimuth_count == 6'd1,
              "azimuth_count increments once after one azimuth toggle");

        // Full-scan contract: 50 azimuth toggles should generate one scan_complete
        // Re-apply reset to start from a known zeroed azimuth_count.
        stm32_new_chirp = 0;
        stm32_new_elevation = 0;
        stm32_new_azimuth = 0;
        reset_n = 0;
        repeat (4) @(posedge clk);
        reset_n = 1;
        @(posedge clk);
        @(posedge clk);
        #1;
        mode = 2'b00;

        chirp_out_toggles = 0;
        elev_out_toggles = 0;
        az_out_toggles = 0;
        scan_complete_pulses = 0;

        for (k = 0; k < AZIMUTHS_PER_SCAN; k = k + 1) begin
            for (i = 0; i < ELEVATIONS_PER_AZIMUTH; i = i + 1) begin
                for (j = 0; j < CHIRPS_PER_ELEVATION; j = j + 1) begin
                    do_chirp_toggle();
                end
                do_elevation_toggle();
            end
            do_azimuth_toggle();
        end

        @(posedge clk);
        #1;

        check(chirp_out_toggles == AZIMUTHS_PER_SCAN * ELEVATIONS_PER_AZIMUTH * CHIRPS_PER_ELEVATION,
              "full scan chirp toggle count matches 50*31*32");
        check(elev_out_toggles == AZIMUTHS_PER_SCAN * ELEVATIONS_PER_AZIMUTH,
              "full scan elevation toggle count matches 50*31");
        check(az_out_toggles == AZIMUTHS_PER_SCAN,
              "full scan azimuth toggle count matches 50");
        check(scan_complete_pulses == 1,
              "scan_complete pulses exactly once per full scan");
        check(azimuth_count == 6'd0,
              "azimuth_count wraps to 0 after full scan");

        $display("========================================");
        $display(" PASS-THROUGH CONTRACT RESULTS");
        $display(" PASSED: %0d / %0d", pass_count, test_num);
        $display(" FAILED: %0d / %0d", fail_count, test_num);
        if (fail_count == 0)
            $display(" ** ALL TESTS PASSED **");
        else
            $display(" ** SOME TESTS FAILED **");
        $display("========================================");

        #50;
        $finish;
    end

endmodule
