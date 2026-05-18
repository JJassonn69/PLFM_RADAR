`timescale 1ns / 1ps
`include "radar_params.vh"

// ============================================================================
// tb_system_opcodes_ft601.v  (PR-AD AD.3 — FT601 sibling of tb_system_opcodes)
//
// Verifies host opcode dispatch through the FT601 USB 3.0 path.
// radar_system_top is instantiated with USB_MODE=0; ft601_data / ft601_rxf
// are driven by a BFM. The FT601 RX FSM reads one 32-bit word per command
// transaction (vs FT2232H's 4-byte shift), so the BFM is simpler than its
// FT2232H sibling.
//
// Each test sends a command word {opcode[31:24], addr[23:16], value[15:0]}
// and verifies the corresponding dut.host_* register updates after CDC
// propagation.
//
// Mirrors the same test groups as tb_system_opcodes.v (G6 / G7 / G13 / G14
// / G17) so any opcode-dispatch regression caught on the FT2232H path is
// also caught on the FT601 path. This guards against future drift between
// the two USB drivers' command paths.
// ============================================================================

module tb_system_opcodes_ft601;

// ----------------------------------------------------------------------------
// Clocks
// ----------------------------------------------------------------------------
localparam CLK_100M_PERIOD  = 10.0;    // 100 MHz radar clock
localparam CLK_120M_PERIOD  = 8.333;   // 120 MHz DAC clock
localparam FT601_CLK_PERIOD = 10.0;    // 100 MHz FT601 clock (asynchronous to clk_100m)
localparam ADC_DCO_PERIOD   = 2.5;

reg clk_100m     = 1'b0;
reg clk_120m_dac = 1'b0;
reg ft601_clk_in = 1'b0;
reg adc_dco_p    = 1'b0;
reg adc_dco_n    = 1'b1;

always #(CLK_100M_PERIOD/2)  clk_100m     = ~clk_100m;
always #(CLK_120M_PERIOD/2)  clk_120m_dac = ~clk_120m_dac;
always #(FT601_CLK_PERIOD/2) ft601_clk_in = ~ft601_clk_in;
always #(ADC_DCO_PERIOD/2)   begin adc_dco_p = ~adc_dco_p; adc_dco_n = ~adc_dco_n; end

// ----------------------------------------------------------------------------
// DUT signals
// ----------------------------------------------------------------------------
reg         reset_n = 1'b0;

reg [7:0]   adc_d_p = 8'h80;
reg [7:0]   adc_d_n = 8'h7F;

reg         stm32_mixers_enable = 1'b0;
reg         stm32_sclk_3v3 = 1'b0;
reg         stm32_mosi_3v3 = 1'b0;
wire        stm32_miso_3v3;
reg         stm32_cs_adar1_3v3 = 1'b1, stm32_cs_adar2_3v3 = 1'b1;
reg         stm32_cs_adar3_3v3 = 1'b1, stm32_cs_adar4_3v3 = 1'b1;
wire        stm32_sclk_1v8, stm32_mosi_1v8;
reg         stm32_miso_1v8 = 1'b0;
wire        stm32_cs_adar1_1v8, stm32_cs_adar2_1v8;
wire        stm32_cs_adar3_1v8, stm32_cs_adar4_1v8;

wire [7:0]  dac_data;
wire        dac_clk;
wire        dac_sleep;
wire        fpga_rf_switch;
wire        rx_mixer_en, tx_mixer_en;
wire        adc_pwdn;

wire        adar_tx_load_1, adar_rx_load_1;
wire        adar_tx_load_2, adar_rx_load_2;
wire        adar_tx_load_3, adar_rx_load_3;
wire        adar_tx_load_4, adar_rx_load_4;
wire        adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4;

// FT601 ports — DRIVEN BY THIS TB
wire [31:0] ft601_data;
wire [3:0]  ft601_be;
wire        ft601_txe_n;
wire        ft601_rxf_n;
reg         ft601_txe = 1'b0;
reg         ft601_rxf = 1'b1;
wire        ft601_wr_n;
wire        ft601_rd_n;
wire        ft601_oe_n;
wire        ft601_siwu_n;
reg  [1:0]  ft601_srb = 2'b00;
reg  [1:0]  ft601_swb = 2'b00;
wire        ft601_clk_out;

// TB-side bus driver: drive ft601_data on host->FPGA, release otherwise.
reg [31:0]  ft601_data_drive    = 32'd0;
reg         ft601_data_drive_en = 1'b0;
assign ft601_data = ft601_data_drive_en ? ft601_data_drive : 32'hzzzz_zzzz;
pulldown pd[31:0] (ft601_data);

// FT2232H ports — unused in USB_MODE=0; tie inputs, ignore outputs
wire [7:0]  ft_data;
reg         ft_rxf_n = 1'b1;
reg         ft_txe_n = 1'b1;
wire        ft_rd_n;
wire        ft_wr_n;
wire        ft_oe_n;
wire        ft_siwu;

wire [5:0]  current_chirp;
wire        new_chirp_frame;
wire [31:0] dbg_doppler_data;
wire        dbg_doppler_valid;
wire [`RP_DOPPLER_BIN_WIDTH-1:0]   dbg_doppler_bin;
wire [`RP_RANGE_BIN_WIDTH_MAX-1:0] dbg_range_bin;
wire [3:0]  system_status;
wire        gpio_dig5, gpio_dig6, gpio_dig7;

// ----------------------------------------------------------------------------
// DUT — radar_system_top with USB_MODE=0 (FT601 path, 200T premium board)
// ----------------------------------------------------------------------------
radar_system_top #(
    .USB_MODE(0)
) dut (
    .clk_100m(clk_100m),
    .clk_120m_dac(clk_120m_dac),
    .ft601_clk_in(ft601_clk_in),
    .reset_n(reset_n),

    .dac_data(dac_data), .dac_clk(dac_clk), .dac_sleep(dac_sleep),
    .fpga_rf_switch(fpga_rf_switch),
    .rx_mixer_en(rx_mixer_en), .tx_mixer_en(tx_mixer_en),

    .adar_tx_load_1(adar_tx_load_1), .adar_rx_load_1(adar_rx_load_1),
    .adar_tx_load_2(adar_tx_load_2), .adar_rx_load_2(adar_rx_load_2),
    .adar_tx_load_3(adar_tx_load_3), .adar_rx_load_3(adar_rx_load_3),
    .adar_tx_load_4(adar_tx_load_4), .adar_rx_load_4(adar_rx_load_4),
    .adar_tr_1(adar_tr_1), .adar_tr_2(adar_tr_2),
    .adar_tr_3(adar_tr_3), .adar_tr_4(adar_tr_4),

    .stm32_sclk_3v3(stm32_sclk_3v3),
    .stm32_mosi_3v3(stm32_mosi_3v3),
    .stm32_miso_3v3(stm32_miso_3v3),
    .stm32_cs_adar1_3v3(stm32_cs_adar1_3v3),
    .stm32_cs_adar2_3v3(stm32_cs_adar2_3v3),
    .stm32_cs_adar3_3v3(stm32_cs_adar3_3v3),
    .stm32_cs_adar4_3v3(stm32_cs_adar4_3v3),
    .stm32_sclk_1v8(stm32_sclk_1v8),
    .stm32_mosi_1v8(stm32_mosi_1v8),
    .stm32_miso_1v8(stm32_miso_1v8),
    .stm32_cs_adar1_1v8(stm32_cs_adar1_1v8),
    .stm32_cs_adar2_1v8(stm32_cs_adar2_1v8),
    .stm32_cs_adar3_1v8(stm32_cs_adar3_1v8),
    .stm32_cs_adar4_1v8(stm32_cs_adar4_1v8),

    .adc_d_p(adc_d_p), .adc_d_n(adc_d_n),
    .adc_dco_p(adc_dco_p), .adc_dco_n(adc_dco_n),
    .adc_or_p(1'b0), .adc_or_n(1'b1),
    .adc_pwdn(adc_pwdn),

    .stm32_beam_ready(1'b0),
    .stm32_mixers_enable(stm32_mixers_enable),

    // FT601 ports — driven by this TB
    .ft601_data(ft601_data),
    .ft601_be(ft601_be),
    .ft601_txe_n(ft601_txe_n),
    .ft601_rxf_n(ft601_rxf_n),
    .ft601_txe(ft601_txe),
    .ft601_rxf(ft601_rxf),
    .ft601_wr_n(ft601_wr_n),
    .ft601_rd_n(ft601_rd_n),
    .ft601_oe_n(ft601_oe_n),
    .ft601_siwu_n(ft601_siwu_n),
    .ft601_srb(ft601_srb),
    .ft601_swb(ft601_swb),
    .ft601_clk_out(ft601_clk_out),

    // FT2232H ports — tied off in USB_MODE=0
    .ft_data(ft_data),
    .ft_rxf_n(ft_rxf_n),
    .ft_txe_n(ft_txe_n),
    .ft_rd_n(ft_rd_n),
    .ft_wr_n(ft_wr_n),
    .ft_oe_n(ft_oe_n),
    .ft_siwu(ft_siwu),

    .current_chirp(current_chirp),
    .new_chirp_frame(new_chirp_frame),
    .dbg_doppler_data(dbg_doppler_data),
    .dbg_doppler_valid(dbg_doppler_valid),
    .dbg_doppler_bin(dbg_doppler_bin),
    .dbg_range_bin(dbg_range_bin),
    .system_status(system_status),
    .gpio_dig5(gpio_dig5),
    .gpio_dig6(gpio_dig6),
    .gpio_dig7(gpio_dig7)
);

// ----------------------------------------------------------------------------
// BFM — FT601 RX FSM: 32-bit single-word read
// FSM advances RD_IDLE -> RD_OE_ASSERT -> RD_READING -> RD_DEASSERT ->
// RD_PROCESS over 4 ft601_clk cycles, sampling ft601_data once at RD_READING.
// Command word format: {opcode[31:24], addr[23:16], value[15:0]}.
// ----------------------------------------------------------------------------
task wait_clk;
    input integer n;
    integer i;
    begin
        for (i = 0; i < n; i = i + 1) @(posedge clk_100m);
    end
endtask

task send_cmd;
    input [7:0]  op;
    input [7:0]  addr;
    input [15:0] val;
    begin
        @(posedge ft601_clk_in); #1;
        ft601_rxf           = 1'b0;
        ft601_data_drive    = {op, addr, val};
        ft601_data_drive_en = 1'b1;
        @(posedge ft601_clk_in); #1;   // RD_IDLE -> RD_OE_ASSERT
        @(posedge ft601_clk_in); #1;   // RD_OE_ASSERT -> RD_READING
        @(posedge ft601_clk_in); #1;   // RD_READING samples ft601_data
        @(posedge ft601_clk_in); #1;   // RD_DEASSERT
        @(posedge ft601_clk_in); #1;   // RD_PROCESS pulses cmd_valid
        ft601_rxf           = 1'b1;
        ft601_data_drive_en = 1'b0;
        wait_clk(40);                  // CDC ft601_clk_in -> clk_100m
    end
endtask

// ----------------------------------------------------------------------------
// Test infrastructure
// ----------------------------------------------------------------------------
integer pass_count = 0;
integer fail_count = 0;
integer test_num   = 0;

task check;
    input         cond;
    input [80*8-1:0] msg;
    begin
        test_num = test_num + 1;
        if (cond) begin
            $display("  [PASS] %0d: %0s", test_num, msg);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] %0d: %0s", test_num, msg);
            fail_count = fail_count + 1;
        end
    end
endtask

// ----------------------------------------------------------------------------
// Main test sequence — mirrors tb_system_opcodes.v groups
// ----------------------------------------------------------------------------
initial begin
    $display("============================================================");
    $display("  tb_system_opcodes_ft601 — opcode dispatch via FT601");
    $display("============================================================");

    reset_n = 1'b0;
    wait_clk(20);
    reset_n = 1'b1;
    wait_clk(50);

    // ====================================================================
    // Group 6: USB Command Decode
    // ====================================================================
    $display("\n--- Group 6: USB Command Decode (FT601) ---");

    send_cmd(8'h03, 8'h00, 16'h1234);
    check(dut.host_detect_threshold == 16'h1234,
          "G6.2: 0x03 -> host_detect_threshold = 0x1234");

    send_cmd(8'h04, 8'h00, 16'h0005);
    check(dut.host_stream_control == 6'b000_101,
          "G6.3: 0x04 -> host_stream_control[2:0] = 3'b101");

    send_cmd(8'h10, 8'h00, 16'd2000);
    check(dut.host_long_chirp_cycles == 16'd2000,
          "G6.4: 0x10 -> host_long_chirp_cycles = 2000");

    send_cmd(8'h15, 8'h00, 16'd48);
    check(dut.host_chirps_per_elev == 6'd48,
          "G6.5: 0x15 -> host_chirps_per_elev = 48");
    check(dut.chirps_mismatch_error == 1'b0,
          "G6.5b: chirps_mismatch_error clear when chirps==48");

    // ====================================================================
    // Group 7: CDC Integrity
    // ====================================================================
    $display("\n--- Group 7: USB Command CDC Integrity ---");

    send_cmd(8'h03, 8'h00, 16'hAAAA);
    send_cmd(8'h03, 8'h00, 16'hBBBB);
    send_cmd(8'h03, 8'h00, 16'hCCCC);
    check(dut.host_detect_threshold == 16'hCCCC,
          "G7.2: Last of 3 rapid commands applied (0xCCCC)");
    check(dut.host_detect_threshold == 16'hCCCC,
          "G7.4: CDC-transferred threshold bit-exact");

    // ====================================================================
    // Group 13: Chirps/Doppler Mismatch Protection
    // ====================================================================
    $display("\n--- Group 13: Chirps/Doppler Mismatch Protection ---");

    send_cmd(8'h15, 8'h00, 16'd48);
    check(dut.host_chirps_per_elev == 6'd48,
          "G13.1: chirps_per_elev=48 accepted");
    check(dut.chirps_mismatch_error == 1'b0,
          "G13.2: Mismatch clear when chirps==48");

    send_cmd(8'h15, 8'h00, 16'd56);
    check(dut.host_chirps_per_elev == 6'd48,
          "G13.3: chirps=56 clamped to 48");
    check(dut.chirps_mismatch_error == 1'b1,
          "G13.4: Mismatch set when chirps>48");

    send_cmd(8'h15, 8'h00, 16'd0);
    check(dut.host_chirps_per_elev == 6'd48,
          "G13.5: chirps=0 clamped to 48");

    send_cmd(8'h15, 8'h00, 16'd16);
    check(dut.host_chirps_per_elev == 6'd16,
          "G13.6: chirps_per_elev=16 accepted (not clamped)");
    check(dut.chirps_mismatch_error == 1'b1,
          "G13.7: Mismatch set when chirps<48");

    send_cmd(8'h15, 8'h00, 16'd48);
    check(dut.chirps_mismatch_error == 1'b0,
          "G13.8: Mismatch clears when restored to 48");

    // ====================================================================
    // Group 14: CFAR Opcodes
    // ====================================================================
    $display("\n--- Group 14: CFAR Opcodes ---");

    send_cmd(8'h21, 8'h00, 16'h0004);
    check(dut.host_cfar_guard == 4'd4,  "G14.4: 0x21 -> host_cfar_guard = 4");
    send_cmd(8'h21, 8'h00, 16'h0000);
    check(dut.host_cfar_guard == 4'd0,  "G14.5: 0x21 -> host_cfar_guard = 0");

    send_cmd(8'h22, 8'h00, 16'h0010);
    check(dut.host_cfar_train == 5'd16, "G14.6: 0x22 -> host_cfar_train = 16");
    send_cmd(8'h22, 8'h00, 16'h0001);
    check(dut.host_cfar_train == 5'd1,  "G14.7: 0x22 -> host_cfar_train = 1");

    send_cmd(8'h23, 8'h00, 16'h0048);
    check(dut.host_cfar_alpha == 8'h48, "G14.8: 0x23 -> host_cfar_alpha = 0x48");
    send_cmd(8'h23, 8'h00, 16'h0010);
    check(dut.host_cfar_alpha == 8'h10, "G14.9: 0x23 -> host_cfar_alpha = 0x10");

    send_cmd(8'h24, 8'h00, 16'h0001);
    check(dut.host_cfar_mode == 2'b01, "G14.10: 0x24 -> host_cfar_mode = GO-CFAR");
    send_cmd(8'h24, 8'h00, 16'h0002);
    check(dut.host_cfar_mode == 2'b10, "G14.11: 0x24 -> host_cfar_mode = SO-CFAR");

    send_cmd(8'h25, 8'h00, 16'h0001);
    check(dut.host_cfar_enable == 1'b1, "G14.12: 0x25 -> host_cfar_enable = 1");
    send_cmd(8'h25, 8'h00, 16'h0000);
    check(dut.host_cfar_enable == 1'b0, "G14.13: 0x25 -> host_cfar_enable = 0");

    // ====================================================================
    // Group 17: PR-G additions (MEDIUM ladder + alpha_soft)
    // ====================================================================
    $display("\n--- Group 17: PR-G MEDIUM ladder + alpha_soft ---");

    send_cmd(`RP_OP_MEDIUM_CHIRP_CYCLES, 8'h00, 16'd750);
    check(dut.host_medium_chirp_cycles == 16'd750,
          "G17.1: 0x17 -> host_medium_chirp_cycles = 750");

    send_cmd(`RP_OP_MEDIUM_LISTEN_CYCLES, 8'h00, 16'd16500);
    check(dut.host_medium_listen_cycles == 16'd16500,
          "G17.2: 0x18 -> host_medium_listen_cycles = 16500");

    send_cmd(`RP_OP_CFAR_ALPHA_SOFT, 8'h00, 16'h0024);
    check(dut.host_cfar_alpha_soft == 8'h24,
          "G17.3: 0x2D -> host_cfar_alpha_soft = 0x24");

    $display("\n============================================================");
    $display("  RESULTS: %0d passed, %0d failed / %0d total",
             pass_count, fail_count, test_num);
    $display("============================================================");
    if (fail_count == 0) $display("  *** ALL TESTS PASSED ***");
    else                 $display("  *** %0d TEST(S) FAILED ***", fail_count);

    $finish;
end

initial begin
    #2_000_000;
    $display("[WATCHDOG] tb_system_opcodes_ft601 timeout");
    $display("  Tests: %0d, Pass: %0d, Fail: %0d",
             test_num, pass_count, fail_count);
    $finish;
end

endmodule
