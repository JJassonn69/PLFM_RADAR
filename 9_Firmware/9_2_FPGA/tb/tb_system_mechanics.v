`timescale 1ns / 1ps
`include "radar_params.vh"

// ============================================================================
// tb_system_mechanics.v  (PR-I, replaces tb_system_e2e G1/G2/G3 + G7.1/G7.3)
//
// Verifies low-level chirp/RF/safety/CDC mechanics that don't require the
// 48-chirp Doppler accumulation. Sim runs at production timing (~1 ms).
//
// Coverage:
//   G1  Reset & initialization (system_status, ft601_wr_n, adc_pwdn)
//   G2  Transmitter chain (DAC chirp, RF switch, TX/RX mixer)
//        — G2.2 (new_chirp_frame at 48-chirp boundary) is exercised by
//          tb_e2e_dsp_to_host (PR-Z A6) end-to-end.
//   G3  Safety architecture (TX/RX mixer mutual exclusion, ADC pwdn, ADAR TR,
//        mixer-disable propagation)
//   G7.3 TX chirp counter CDC (120MHz -> 100MHz)
//        — G7.1 (STM32→FPGA chirp toggle CDC stress) retired in PR-AB.b
//          expanded; the stm32_new_chirp port was renamed to
//          stm32_beam_ready in commit 5 (beam-ready handshake) and is
//          tied 1'b0 here so the cold-reset default of host_handshake_enable
//          (=0) keeps the FSM out of S_BEAM_WAIT and the open-loop cadence
//          intact.
//
// DUT is radar_system_top with USB_MODE=1 (production FT2232H path); the
// FT2232H ports are wired so a stream_control opcode (0x04) can be sent at
// start to give the USB write FSM a clean exit from IDLE.
// ============================================================================

module tb_system_mechanics;

// ----------------------------------------------------------------------------
// Clocks (match production)
// ----------------------------------------------------------------------------
localparam CLK_100M_PERIOD  = 10.0;     // 100 MHz
localparam CLK_120M_PERIOD  = 8.333;    // 120 MHz DAC
localparam FT_CLK_PERIOD    = 16.667;   // 60 MHz FT2232H
localparam ADC_DCO_PERIOD   = 2.5;      // 400 MHz ADC

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

// FT601 ports — unused in USB_MODE=1
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

// FT2232H ports — used for the (single) opcode we may send to set stream_control
wire [7:0]  ft_data;
reg         ft_rxf_n = 1'b1;
reg         ft_txe_n = 1'b0;
wire        ft_rd_n;
wire        ft_wr_n;
wire        ft_oe_n;
wire        ft_siwu;

reg [7:0]   ft_data_drive    = 8'h00;
reg         ft_data_drive_en = 1'b0;
assign ft_data = ft_data_drive_en ? ft_data_drive : 8'hzz;
pulldown pd[7:0] (ft_data);

wire [5:0]  current_chirp;
wire        new_chirp_frame;
wire [31:0] dbg_doppler_data;
wire        dbg_doppler_valid;
wire [`RP_DOPPLER_BIN_WIDTH-1:0]   dbg_doppler_bin;
wire [`RP_RANGE_BIN_WIDTH_MAX-1:0] dbg_range_bin;
wire [3:0]  system_status;
wire        gpio_dig5, gpio_dig6, gpio_dig7;

// ----------------------------------------------------------------------------
// DUT
// ----------------------------------------------------------------------------
radar_system_top #(.USB_MODE(1)) dut (
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

    .stm32_beam_ready(1'b0),  // commit 5: handshake disabled by cold-reset default
    .stm32_mixers_enable(stm32_mixers_enable),

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

// ADC stimulus: ramp around mid-scale (matches tb_system_e2e pattern)
integer adc_phase;
initial begin
    adc_phase = 0;
    forever begin
        @(posedge adc_dco_p);
        if (reset_n) begin
            adc_d_p  = 8'h80 + ((adc_phase * 7) & 8'h3F) - 8'h20;
            adc_d_n  = ~adc_d_p;
            adc_phase = adc_phase + 1;
        end else begin
            adc_d_p = 8'h80;
            adc_d_n = 8'h7F;
        end
    end
end

// ----------------------------------------------------------------------------
// FT2232H send_cmd (carried over from tb_system_opcodes)
// Used here only to send opcode 0x04 (stream_control = range-only) at start
// so the USB write FSM doesn't deadlock waiting for unused doppler/detect data.
// ----------------------------------------------------------------------------
task send_cmd;
    input [7:0]  op;
    input [7:0]  addr;
    input [15:0] val;
    integer i;
    begin
        @(posedge ft601_clk_in); #1;
        ft_rxf_n = 1'b0;
        ft_data_drive = op;
        ft_data_drive_en = 1'b1;
        @(posedge ft601_clk_in); #1;
        @(posedge ft601_clk_in); #1;
        @(posedge ft601_clk_in); #1;
        ft_data_drive = addr;
        @(posedge ft601_clk_in); #1;
        ft_data_drive = val[15:8];
        @(posedge ft601_clk_in); #1;
        ft_data_drive = val[7:0];
        @(posedge ft601_clk_in); #1;
        ft_rxf_n = 1'b1;
        ft_data_drive_en = 1'b0;
        for (i = 0; i < 40; i = i + 1) @(posedge clk_100m);
    end
endtask

// ----------------------------------------------------------------------------
// USB write monitor (count writes during reset; only ft_wr_n in FT2232H mode)
// ----------------------------------------------------------------------------
integer usb_wr_count_total = 0;
always @(posedge ft601_clk_in) begin
    if (!reset_n)        usb_wr_count_total <= 0;
    else if (!ft_wr_n)   usb_wr_count_total <= usb_wr_count_total + 1;
end

// ----------------------------------------------------------------------------
// Observation counters
// ----------------------------------------------------------------------------
integer obs_dac_nonzero_count = 0;
reg     obs_seen_tx_mixer     = 1'b0;
reg     obs_seen_rx_mixer     = 1'b0;
reg     obs_seen_rf_switch    = 1'b0;
reg [5:0] obs_max_chirp       = 6'd0;
integer safety_simul_mixer_count = 0;
integer safety_mixer_deassert_fail_count = 0;
reg [3:0] mixer_disable_timer = 4'd0;

always @(posedge clk_100m) begin
    if (reset_n) begin
        if (tx_mixer_en)    obs_seen_tx_mixer  <= 1'b1;
        if (rx_mixer_en)    obs_seen_rx_mixer  <= 1'b1;
        if (fpga_rf_switch) obs_seen_rf_switch <= 1'b1;
        if (current_chirp > obs_max_chirp)
            obs_max_chirp <= current_chirp;
        // Safety: TX/RX mixer mutual exclusion
        if (tx_mixer_en && rx_mixer_en) begin
            safety_simul_mixer_count <= safety_simul_mixer_count + 1;
        end
        // Safety: mixer-disable propagation watchdog
        if (!stm32_mixers_enable) begin
            if (mixer_disable_timer < 4'd15)
                mixer_disable_timer <= mixer_disable_timer + 1'b1;
            if (mixer_disable_timer >= 4'd12 && (tx_mixer_en || rx_mixer_en))
                safety_mixer_deassert_fail_count <= safety_mixer_deassert_fail_count + 1;
        end else begin
            mixer_disable_timer <= 4'd0;
        end
    end
end

always @(posedge clk_120m_dac) begin
    if (reset_n && dac_data != 8'h80 && dac_data != 8'h00)
        obs_dac_nonzero_count = obs_dac_nonzero_count + 1;
end

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
integer i;

initial begin
    $display("============================================================");
    $display("  tb_system_mechanics — chirp/RF/safety/CDC mechanics");
    $display("============================================================");

    // Reset
    reset_n = 1'b0;
    repeat (20) @(posedge clk_100m);
    reset_n = 1'b1;
    repeat (50) @(posedge clk_100m);

    // Configure stream_control = range-only so the USB write FSM has a clean
    // exit from IDLE (otherwise it could wait on unused doppler/detect flags).
    send_cmd(8'h04, 8'h00, 16'h0001);

    // ====================================================================
    // GROUP 1: RESET & INITIALIZATION
    // ====================================================================
    $display("\n--- Group 1: Reset & Initialization ---");

    check(system_status == 4'b0000,
          "G1.1: system_status == 0 after reset");
    check(usb_wr_count_total == 0,
          "G1.2: No USB writes during/after reset");
    check(ft_wr_n == 1'b1,
          "G1.3: ft_wr_n == 1 after reset (FT2232H idle)");
    check(adc_pwdn == 1'b0,
          "G1.4: adc_pwdn == 0 (ADC enabled)");

    // ====================================================================
    // GROUP 2: TRANSMITTER CHAIN  (G2.2 covered by tb_e2e_dsp_to_host A6)
    // ====================================================================
    $display("\n--- Group 2: Transmitter Chain ---");

    stm32_mixers_enable = 1'b1;
    #100;

    // PR-AB.b expanded: scheduler auto-scans once mixers_enable=1 — no STM32
    // chirp trigger needed. Wait long enough to observe DAC, RF switch, and
    // both mixers exercised across TX (chirp) and RX (listen) phases.
    #40000;                 // 40 us — covers initial LONG_CHIRP -> LONG_LISTEN
    for (i = 0; i < 3; i = i + 1) begin
        #3000;
    end
    #5000;

    check(obs_dac_nonzero_count > 0,
          "G2.1: DAC output non-trivial (chirp generated)");
    check(obs_seen_rf_switch == 1'b1,
          "G2.3: fpga_rf_switch activated during chirp");
    check(obs_seen_tx_mixer == 1'b1,
          "G2.4: tx_mixer_en seen during chirp sequence");
    check(obs_seen_rx_mixer == 1'b1,
          "G2.5: rx_mixer_en seen during listen phase");

    // ====================================================================
    // GROUP 3: SAFETY ARCHITECTURE
    // ====================================================================
    $display("\n--- Group 3: Safety Architecture ---");

    check(safety_simul_mixer_count == 0,
          "G3.1: TX/RX mixers never simultaneously enabled");
    check(adc_pwdn == 1'b0,
          "G3.2: adc_pwdn remains 0 throughout operation");
    check(adar_tr_1 == adar_tr_2 && adar_tr_2 == adar_tr_3 && adar_tr_3 == adar_tr_4,
          "G3.3: All ADAR TR pins consistent");

    // G3.4: Disable mixers — verify they deassert within ~12 cycles
    stm32_mixers_enable = 1'b0;
    #500;
    check(tx_mixer_en == 1'b0 && rx_mixer_en == 1'b0,
          "G3.4: Mixers deassert when stm32_mixers_enable=0");
    check(safety_mixer_deassert_fail_count == 0,
          "G3.5: No mixer-still-on-after-12-cycles violations");

    // Re-enable for G7
    stm32_mixers_enable = 1'b1;
    #100;

    // ====================================================================
    // GROUP 7.3: TX chirp counter CDC  (G7.2/7.4 in tb_system_opcodes;
    // G7.1 retired in PR-AB.b expanded — STM32→FPGA chirp CDC path removed)
    // ====================================================================
    $display("\n--- Group 7: TX chirp counter CDC (120 MHz -> 100 MHz) ---");

    // G7.3: TX chirp counter CDC (120 MHz -> 100 MHz). Either the counter
    // advanced or DAC was active long enough to prove the path is alive.
    check(obs_max_chirp > 0 || obs_dac_nonzero_count > 100,
          "G7.3: TX chirp CDC path delivered data (counter or DAC active)");

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

// Watchdog
initial begin
    #1_500_000;   // 1.5 ms — comfortably above the ~80 us of stimulus
    $display("[WATCHDOG] tb_system_mechanics timeout");
    $display("  Tests: %0d, Pass: %0d, Fail: %0d",
             test_num, pass_count, fail_count);
    $finish;
end

endmodule
