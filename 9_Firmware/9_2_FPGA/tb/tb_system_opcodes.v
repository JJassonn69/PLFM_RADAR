`timescale 1ns / 1ps
`include "radar_params.vh"

// ============================================================================
// tb_system_opcodes.v  (PR-I, replaces tb_system_e2e G6/G7.2/G7.4/G13/G14)
//
// Verifies host opcode dispatch through the production FT2232H USB path.
// radar_system_top is instantiated with USB_MODE=1; ft_data / ft_rxf_n / etc.
// are driven by a BFM modeled on tb_usb_protocol_v2's send_cmd task (the only
// stimulus pattern proven to align with the FT2232H 4-cycle read FSM).
//
// Each test sends a 4-byte command (op, addr, val_hi, val_lo) and verifies
// the corresponding dut.host_* register updates after CDC propagation.
//
// Sim budget: ~1 ms — opcode dispatch only, no chirp pipeline.
// ============================================================================

module tb_system_opcodes;

// ----------------------------------------------------------------------------
// Clocks
// ----------------------------------------------------------------------------
localparam CLK_100M_PERIOD  = 10.0;     // 100 MHz radar clock
localparam CLK_120M_PERIOD  = 8.333;    // 120 MHz DAC clock
localparam FT_CLK_PERIOD    = 16.667;   // 60 MHz FT2232H clock
localparam ADC_DCO_PERIOD   = 2.5;      // 400 MHz ADC DCO

reg clk_100m     = 1'b0;
reg clk_120m_dac = 1'b0;
reg ft601_clk_in = 1'b0;
reg adc_dco_p    = 1'b0;
reg adc_dco_n    = 1'b1;

always #(CLK_100M_PERIOD/2) clk_100m     = ~clk_100m;
always #(CLK_120M_PERIOD/2) clk_120m_dac = ~clk_120m_dac;
always #(FT_CLK_PERIOD/2)   ft601_clk_in = ~ft601_clk_in;
always #(ADC_DCO_PERIOD/2)  begin adc_dco_p = ~adc_dco_p; adc_dco_n = ~adc_dco_n; end

// ----------------------------------------------------------------------------
// DUT signals (FT2232H production path)
// ----------------------------------------------------------------------------
reg         reset_n = 1'b0;

// ADC quiescent (no chirp data needed for opcode tests)
reg [7:0]   adc_d_p = 8'h80;
reg [7:0]   adc_d_n = 8'h7F;

// STM32 control — tied off
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

// DAC outputs (ignored)
wire [7:0]  dac_data;
wire        dac_clk;
wire        dac_sleep;

// RF control (ignored)
wire        fpga_rf_switch;
wire        rx_mixer_en, tx_mixer_en;
wire        adc_pwdn;

// ADAR (ignored)
wire        adar_tx_load_1, adar_rx_load_1;
wire        adar_tx_load_2, adar_rx_load_2;
wire        adar_tx_load_3, adar_rx_load_3;
wire        adar_tx_load_4, adar_rx_load_4;
wire        adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4;

// FT601 ports — unused in USB_MODE=1; tie inputs and ignore outputs
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

// FT2232H ports — DRIVEN BY THIS TB
wire [7:0]  ft_data;     // bidirectional
reg         ft_rxf_n = 1'b1;
reg         ft_txe_n = 1'b0;
wire        ft_rd_n;
wire        ft_wr_n;
wire        ft_oe_n;
wire        ft_siwu;

// TB-side bus driver: drive ft_data while sending command bytes,
// release to high-Z otherwise (DUT may drive on writes, but opcode
// dispatch tests don't trigger writes).
reg [7:0]   ft_data_drive    = 8'h00;
reg         ft_data_drive_en = 1'b0;
assign ft_data = ft_data_drive_en ? ft_data_drive : 8'hzz;
pulldown pd[7:0] (ft_data);

// Status / debug outputs (mostly ignored)
wire [5:0]  current_chirp;
wire        new_chirp_frame;
wire [31:0] dbg_doppler_data;
wire        dbg_doppler_valid;
wire [`RP_DOPPLER_BIN_WIDTH-1:0]   dbg_doppler_bin;
wire [`RP_RANGE_BIN_WIDTH_MAX-1:0] dbg_range_bin;
wire [3:0]  system_status;
wire        gpio_dig5, gpio_dig6, gpio_dig7;

// ----------------------------------------------------------------------------
// DUT — radar_system_top with USB_MODE=1 (production FT2232H path)
// ----------------------------------------------------------------------------
radar_system_top #(
    .USB_MODE(1)
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

    .stm32_beam_ready(1'b0),  // commit 5: handshake gated off by host_handshake_enable cold-reset = 0
    .stm32_mixers_enable(stm32_mixers_enable),

    // FT601 ports — tied off / unused in USB_MODE=1
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

    // FT2232H ports — driven by this TB
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
// BFM — proven send_cmd from tb_usb_protocol_v2 (4-cycle FT2232H read FSM)
// ----------------------------------------------------------------------------
task wait_clk;
    input integer n;
    integer i;
    begin
        for (i = 0; i < n; i = i + 1) @(posedge clk_100m);
    end
endtask

task wait_ft;
    input integer n;
    integer i;
    begin
        for (i = 0; i < n; i = i + 1) @(posedge ft601_clk_in);
    end
endtask

task send_cmd;
    input [7:0]  op;
    input [7:0]  addr;
    input [15:0] val;
    begin
        @(posedge ft601_clk_in); #1;
        ft_rxf_n         = 1'b0;
        ft_data_drive    = op;
        ft_data_drive_en = 1'b1;
        @(posedge ft601_clk_in); #1;   // RD_IDLE -> RD_OE_ASSERT (NBA)
        @(posedge ft601_clk_in); #1;   // RD_OE_ASSERT -> RD_READING
        @(posedge ft601_clk_in); #1;   // RD_READING samples op
        ft_data_drive    = addr;
        @(posedge ft601_clk_in); #1;   // samples addr
        ft_data_drive    = val[15:8];
        @(posedge ft601_clk_in); #1;   // samples val_hi
        ft_data_drive    = val[7:0];
        @(posedge ft601_clk_in); #1;   // samples val_lo
        ft_rxf_n         = 1'b1;
        ft_data_drive_en = 1'b0;
        wait_clk(40);                  // CDC ft_clk -> clk_100m + dispatch register
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
// Main test sequence
// ----------------------------------------------------------------------------
initial begin
    $display("============================================================");
    $display("  tb_system_opcodes — opcode dispatch via FT2232H");
    $display("============================================================");

    // Reset
    reset_n = 1'b0;
    wait_clk(20);
    reset_n = 1'b1;
    wait_clk(50);

    // ====================================================================
    // GROUP 6: USB COMMAND DECODE (was tb_system_e2e G6;
    // G6.1 radar_mode / G6.6 trigger_pulse retired in PR-AB.b expanded)
    // ====================================================================
    $display("\n--- Group 6: USB Command Decode ---");

    // G6.2: Set detection threshold (0x03) -> host_detect_threshold
    send_cmd(8'h03, 8'h00, 16'h1234);
    check(dut.host_detect_threshold == 16'h1234,
          "G6.2: 0x03 -> host_detect_threshold = 0x1234");

    // G6.3: Set stream control (0x04) -> host_stream_control[2:0]
    // Bits [5:3] are reserved (forced to 000 by dispatch logic)
    send_cmd(8'h04, 8'h00, 16'h0005);
    check(dut.host_stream_control == 6'b000_101,
          "G6.3: 0x04 -> host_stream_control[2:0] = 3'b101");

    // G6.4: Long chirp cycles (0x10) -> host_long_chirp_cycles
    send_cmd(8'h10, 8'h00, 16'd2000);
    check(dut.host_long_chirp_cycles == 16'd2000,
          "G6.4: 0x10 -> host_long_chirp_cycles = 2000");

    // G6.5: chirps_per_elev (0x15). Production frame is 48 chirps; value
    // matching DOPPLER_FRAME_CHIRPS clears the mismatch flag.
    send_cmd(8'h15, 8'h00, 16'd48);
    check(dut.host_chirps_per_elev == 6'd48,
          "G6.5: 0x15 -> host_chirps_per_elev = 48");
    check(dut.chirps_mismatch_error == 1'b0,
          "G6.5b: chirps_mismatch_error clear when chirps==48");

    // ====================================================================
    // GROUP 7: USB COMMAND CDC INTEGRITY (was G7.2 / G7.4)
    // ====================================================================
    $display("\n--- Group 7: USB Command CDC Integrity ---");

    // G7.2: Three rapid USB commands; verify the last one is applied.
    send_cmd(8'h03, 8'h00, 16'hAAAA);
    send_cmd(8'h03, 8'h00, 16'hBBBB);
    send_cmd(8'h03, 8'h00, 16'hCCCC);
    check(dut.host_detect_threshold == 16'hCCCC,
          "G7.2: Last of 3 rapid USB commands applied (0xCCCC)");

    // G7.4: CDC carries value bit-exact (no corruption).
    check(dut.host_detect_threshold == 16'hCCCC,
          "G7.4: CDC-transferred detect threshold bit-exact");

    // ====================================================================
    // GROUP 13: DOPPLER/CHIRPS MISMATCH PROTECTION (PR-F: 48-chirp aware)
    // ====================================================================
    $display("\n--- Group 13: Chirps/Doppler Mismatch Protection ---");

    // G13.1: chirps==48 (matches DOPPLER_FRAME_CHIRPS) -> mismatch clear
    send_cmd(8'h15, 8'h00, 16'd48);
    check(dut.host_chirps_per_elev == 6'd48,
          "G13.1: chirps_per_elev=48 accepted (matches frame size)");
    check(dut.chirps_mismatch_error == 1'b0,
          "G13.2: Mismatch clear when chirps==48");

    // G13.3: chirps>48 clamped to 48, error set
    send_cmd(8'h15, 8'h00, 16'd56);
    check(dut.host_chirps_per_elev == 6'd48,
          "G13.3: chirps=56 clamped to 48");
    check(dut.chirps_mismatch_error == 1'b1,
          "G13.4: Mismatch set when chirps>48 (was 56)");

    // G13.5: chirps==0 clamped to 48
    send_cmd(8'h15, 8'h00, 16'd0);
    check(dut.host_chirps_per_elev == 6'd48,
          "G13.5: chirps=0 clamped to 48");

    // G13.6: chirps<48 accepted but flagged
    send_cmd(8'h15, 8'h00, 16'd16);
    check(dut.host_chirps_per_elev == 6'd16,
          "G13.6: chirps_per_elev=16 accepted (not clamped)");
    check(dut.chirps_mismatch_error == 1'b1,
          "G13.7: Mismatch set when chirps<48 (was 16)");

    // G13.8: Restore chirps=48, error clears
    send_cmd(8'h15, 8'h00, 16'd48);
    check(dut.chirps_mismatch_error == 1'b0,
          "G13.8: Mismatch clears when restored to 48");

    // ====================================================================
    // GROUP 14: CFAR OPCODES  (G14.1/.2/.3 range_mode retired in PR-AB.b
    // expanded — opcode 0x20 / host_range_mode are gone)
    // ====================================================================
    $display("\n--- Group 14: CFAR Opcodes ---");

    // G14.4-5: CFAR guard cells (0x21)
    send_cmd(8'h21, 8'h00, 16'h0004);
    check(dut.host_cfar_guard == 4'd4,  "G14.4: 0x21 -> host_cfar_guard = 4");
    send_cmd(8'h21, 8'h00, 16'h0000);
    check(dut.host_cfar_guard == 4'd0,  "G14.5: 0x21 -> host_cfar_guard = 0");

    // G14.6-7: CFAR training cells (0x22)
    send_cmd(8'h22, 8'h00, 16'h0010);
    check(dut.host_cfar_train == 5'd16, "G14.6: 0x22 -> host_cfar_train = 16");
    send_cmd(8'h22, 8'h00, 16'h0001);
    check(dut.host_cfar_train == 5'd1,  "G14.7: 0x22 -> host_cfar_train = 1");

    // G14.8-9: CFAR alpha (0x23, Q4.4)
    send_cmd(8'h23, 8'h00, 16'h0048);
    check(dut.host_cfar_alpha == 8'h48, "G14.8: 0x23 -> host_cfar_alpha = 0x48");
    send_cmd(8'h23, 8'h00, 16'h0010);
    check(dut.host_cfar_alpha == 8'h10, "G14.9: 0x23 -> host_cfar_alpha = 0x10");

    // G14.10-11: CFAR mode (0x24)
    send_cmd(8'h24, 8'h00, 16'h0001);
    check(dut.host_cfar_mode == 2'b01, "G14.10: 0x24 -> host_cfar_mode = GO-CFAR");
    send_cmd(8'h24, 8'h00, 16'h0002);
    check(dut.host_cfar_mode == 2'b10, "G14.11: 0x24 -> host_cfar_mode = SO-CFAR");

    // G14.12-13: CFAR enable (0x25)
    send_cmd(8'h25, 8'h00, 16'h0001);
    check(dut.host_cfar_enable == 1'b1, "G14.12: 0x25 -> host_cfar_enable = 1");
    send_cmd(8'h25, 8'h00, 16'h0000);
    check(dut.host_cfar_enable == 1'b0, "G14.13: 0x25 -> host_cfar_enable = 0");

    // ====================================================================
    // GROUP 17: PR-G additions (0x17 / 0x18 MEDIUM ladder, 0x2D alpha_soft)
    // ====================================================================
    $display("\n--- Group 17: PR-G MEDIUM ladder + alpha_soft ---");

    // 0x17 / 0x18 MEDIUM ladder (PR-G G2)
    send_cmd(`RP_OP_MEDIUM_CHIRP_CYCLES, 8'h00, 16'd750);
    check(dut.host_medium_chirp_cycles == 16'd750,
          "G17.1: 0x17 -> host_medium_chirp_cycles = 750");

    send_cmd(`RP_OP_MEDIUM_LISTEN_CYCLES, 8'h00, 16'd16500);
    check(dut.host_medium_listen_cycles == 16'd16500,
          "G17.2: 0x18 -> host_medium_listen_cycles = 16500");

    // 0x2D cfar_alpha_soft (PR-G G1)
    send_cmd(`RP_OP_CFAR_ALPHA_SOFT, 8'h00, 16'h0024);
    check(dut.host_cfar_alpha_soft == 8'h24,
          "G17.3: 0x2D -> host_cfar_alpha_soft = 0x24");

    // ====================================================================
    // SUMMARY
    // ====================================================================
    $display("\n============================================================");
    $display("  RESULTS: %0d passed, %0d failed / %0d total",
             pass_count, fail_count, test_num);
    $display("============================================================");
    if (fail_count == 0) $display("  *** ALL TESTS PASSED ***");
    else                 $display("  *** %0d TEST(S) FAILED ***", fail_count);

    $finish;
end

// ----------------------------------------------------------------------------
// Watchdog
// ----------------------------------------------------------------------------
initial begin
    #2_000_000;   // 2 ms — plenty for ~30 send_cmd calls
    $display("[WATCHDOG] tb_system_opcodes timeout");
    $display("  Tests: %0d, Pass: %0d, Fail: %0d",
             test_num, pass_count, fail_count);
    $finish;
end

endmodule
