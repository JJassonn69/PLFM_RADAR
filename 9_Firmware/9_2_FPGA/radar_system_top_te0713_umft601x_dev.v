`timescale 1ns / 1ps

module radar_system_top_te0713_umft601x_dev (
    input  wire        osc_50m,           // TE0713 on-board 50 MHz oscillator (U20)
    input  wire        ft601_clk_in,      // 100 MHz clock from FT601 chip
    inout  wire [31:0] ft601_data,
    output wire [3:0]  ft601_be,
    input  wire        ft601_txe,
    input  wire        ft601_rxf,
    output wire        ft601_wr_n,
    output wire        ft601_rd_n,
    output wire        ft601_oe_n,
    output wire        ft601_siwu_n,
    output wire        ft601_chip_reset_n,
    output wire        ft601_wakeup_n,
    output wire        ft601_gpio0,
    output wire        ft601_gpio1
);

// =========================================================================
// FT601 chip reset — keep HIGH (deasserted) at all times
// =========================================================================
// The FT601 is already running on USB before the FPGA is programmed.
// The XDC PULLUP on ft601_chip_reset_n (A14) keeps it HIGH during FPGA
// configuration. Once the RTL loads, we drive it HIGH immediately — no
// deliberate reset pulse. Resetting the FT601 in 245 Sync FIFO mode
// causes a USB disconnect that never recovers (confirmed empirically:
// 600 mode tolerates reset, 245 mode does not).
//
// The osc_50m clock is still used for internal POR (sys_reset_n below
// waits for ft601_clk_in), but ft601_chip_reset_n is unconditionally HIGH.
// =========================================================================
assign ft601_chip_reset_n = 1'b1;

// =========================================================================
// FT601-domain system POR — waits for ft601_clk_in to be alive
// =========================================================================
reg [15:0] por_counter = 16'd0;
reg [31:0] hb_counter = 32'd0;
reg [15:0] packet_div = 16'd0;
reg [2:0] stream_control_reg = 3'b000;  // default OFF — host must enable via opcode 0x04
reg        status_request_reg = 1'b0;
reg [31:0] range_profile_reg = 32'd0;
reg        range_valid_reg = 1'b0;
reg [15:0] doppler_real_reg = 16'd0;
reg [15:0] doppler_imag_reg = 16'd0;
reg        doppler_valid_reg = 1'b0;
reg        cfar_detection_reg = 1'b0;
reg        cfar_valid_reg = 1'b0;

wire        sys_reset_n;
wire [31:0] cmd_data;
wire        cmd_valid;
wire [7:0]  cmd_opcode;
wire [7:0]  cmd_addr;
wire [15:0] cmd_value;
wire        ft601_clk_out_unused;
wire        ft601_txe_n_unused;
wire        ft601_rxf_n_unused;

// Debug instrumentation (v7b)
wire [15:0] dbg_wr_strobes;
wire [15:0] dbg_txe_blocks;
wire [15:0] dbg_pkt_starts;
wire [15:0] dbg_pkt_completions;

// Self-test wiring
reg         self_test_trigger = 1'b0;
wire        self_test_busy;
wire        self_test_result_valid;
wire [4:0]  self_test_result_flags;
wire [7:0]  self_test_result_detail;
wire        self_test_capture_active;
wire [15:0] self_test_capture_data;
wire        self_test_capture_valid;
reg  [4:0]  self_test_flags_latched = 5'b00000;
reg  [7:0]  self_test_detail_latched = 8'd0;

assign sys_reset_n = por_counter[15];
assign ft601_wakeup_n = 1'b1;
assign ft601_gpio0 = hb_counter[24];
assign ft601_gpio1 = sys_reset_n;

always @(posedge ft601_clk_in) begin
    if (!sys_reset_n) begin
        por_counter <= por_counter + 1'b1;
        hb_counter <= 32'd0;
        packet_div <= 16'd0;
        stream_control_reg <= 3'b000;  // OFF until host enables
        status_request_reg <= 1'b0;
        range_profile_reg <= 32'd0;
        range_valid_reg <= 1'b0;
        doppler_real_reg <= 16'd0;
        doppler_imag_reg <= 16'd0;
        doppler_valid_reg <= 1'b0;
        cfar_detection_reg <= 1'b0;
        cfar_valid_reg <= 1'b0;
    end else begin
        hb_counter <= hb_counter + 1'b1;
        packet_div <= packet_div + 1'b1;

        status_request_reg <= 1'b0;
        range_valid_reg <= 1'b0;
        doppler_valid_reg <= 1'b0;
        cfar_valid_reg <= 1'b0;

        self_test_trigger <= 1'b0;

        if (self_test_result_valid) begin
            self_test_flags_latched  <= self_test_result_flags;
            self_test_detail_latched <= self_test_result_detail;
        end

        if (cmd_valid) begin
            case (cmd_opcode)
                8'h04: stream_control_reg <= cmd_value[2:0];
                8'h30: self_test_trigger  <= 1'b1;
                8'h31: status_request_reg <= 1'b1;
                8'hFF: status_request_reg <= 1'b1;
                default: ;
            endcase
        end

        if (packet_div == 16'hFFFF && stream_control_reg[0]) begin
            range_profile_reg <= {hb_counter[31:16], hb_counter[15:0] ^ 16'hA5A5};
            range_valid_reg <= 1'b1;

            if (stream_control_reg[1]) begin
                doppler_real_reg <= hb_counter[31:16];
                doppler_imag_reg <= hb_counter[15:0];
                doppler_valid_reg <= 1'b1;
            end

            if (stream_control_reg[2]) begin
                cfar_detection_reg <= hb_counter[10];
                cfar_valid_reg <= 1'b1;
            end
        end
    end
end

usb_data_interface usb_inst (
    .clk(ft601_clk_in),
    .reset_n(sys_reset_n),
    .ft601_reset_n(sys_reset_n),
    .range_profile(range_profile_reg),
    .range_valid(range_valid_reg),
    .doppler_real(doppler_real_reg),
    .doppler_imag(doppler_imag_reg),
    .doppler_valid(doppler_valid_reg),
    .cfar_detection(cfar_detection_reg),
    .cfar_valid(cfar_valid_reg),
    .ft601_data(ft601_data),
    .ft601_be(ft601_be),
    .ft601_txe_n(ft601_txe_n_unused),
    .ft601_rxf_n(ft601_rxf_n_unused),
    .ft601_txe(ft601_txe),
    .ft601_rxf(ft601_rxf),
    .ft601_wr_n(ft601_wr_n),
    .ft601_rd_n(ft601_rd_n),
    .ft601_oe_n(ft601_oe_n),
    .ft601_siwu_n(ft601_siwu_n),
    .ft601_srb(2'b00),
    .ft601_swb(2'b00),
    .ft601_clk_out(ft601_clk_out_unused),
    .ft601_clk_in(ft601_clk_in),
    .cmd_data(cmd_data),
    .cmd_valid(cmd_valid),
    .cmd_opcode(cmd_opcode),
    .cmd_addr(cmd_addr),
    .cmd_value(cmd_value),
    .stream_control(stream_control_reg),
    .status_request(status_request_reg),
    .status_cfar_threshold(16'h1234),
    .status_stream_ctrl(stream_control_reg),
    .status_radar_mode(2'b00),
    .status_long_chirp(16'd3000),
    .status_long_listen(16'd13700),
    .status_guard(16'd17540),
    .status_short_chirp(16'd50),
    .status_short_listen(16'd17450),
    .status_chirps_per_elev(6'd32),
    .status_range_mode(2'b01),
    .status_self_test_flags(self_test_flags_latched),
    .status_self_test_detail(self_test_detail_latched),
    .status_self_test_busy(self_test_busy),
    .dbg_wr_strobes(dbg_wr_strobes),
    .dbg_txe_blocks(dbg_txe_blocks),
    .dbg_pkt_starts(dbg_pkt_starts),
    .dbg_pkt_completions(dbg_pkt_completions),
    .write_idle()  // Not used in dev wrapper — no FIFO needed at slow data rate
);

// Board bring-up self-test controller
// ADC inputs tied to 0 (no AD9484 on dev board — Test 4 will timeout/fail)
fpga_self_test self_test_inst (
    .clk(ft601_clk_in),
    .reset_n(sys_reset_n),
    .trigger(self_test_trigger),
    .busy(self_test_busy),
    .result_valid(self_test_result_valid),
    .result_flags(self_test_result_flags),
    .result_detail(self_test_result_detail),
    .adc_data_in(16'd0),
    .adc_valid_in(1'b0),
    .capture_active(self_test_capture_active),
    .capture_data(self_test_capture_data),
    .capture_valid(self_test_capture_valid)
);

endmodule
