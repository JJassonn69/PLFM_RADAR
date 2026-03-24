`timescale 1ns / 1ps
//
// AERIS-10 TE0713+TE0701 Dev Build with UART + Self-Test
//
// Extends the heartbeat design with:
//   - UART TX/RX (115200 baud, 8N1) via FT2232HQ Channel B
//   - FPGA Self-Test controller (5 subsystems)
//   - Self-Test Reporter (sends results as 20-byte packet over UART)
//
// Uses TE0713 FIFO0CLK (50 MHz, Bank 14, LVCMOS15) at pin U20.
// UART pins on Bank 14: P16 (RXD from host), U18 (TXD to host), LVCMOS33.
// LEDs and status outputs on Bank 16 FMC LA pins (LVCMOS33).
//
// Operation:
//   - Heartbeat LEDs continue blinking (alive indicator).
//   - Self-test runs automatically on power-up after reset release (~500ms).
//   - Host can re-trigger self-test or request status via UART 'S' command.
//   - system_status[0] = UART TX activity (flashes when sending)
//   - system_status[1] = self-test busy
//   - system_status[2] = all tests passed (latched)
//   - system_status[3] = heartbeat (same as before)
//

module radar_system_top_te0713_dev (
    input  wire       clk_100m,        // TE0713 FIFO0CLK (actually 50 MHz)
    input  wire       uart_rxd,        // UART receive from host (Bank 14, P16)
    output wire       uart_txd,        // UART transmit to host (Bank 14, U18)
    output wire [3:0] user_led,
    output wire [3:0] system_status
);

// -------------------------------------------------------------------------
// Clock buffer
// -------------------------------------------------------------------------
wire clk_buf;

`ifdef SIMULATION
    assign clk_buf = clk_100m;
`else
    BUFG bufg_clk (
        .I(clk_100m),
        .O(clk_buf)
    );
`endif

// -------------------------------------------------------------------------
// Power-on reset generator (~500ms at 50 MHz = 25M cycles)
// Use 25-bit counter (33.5M counts) for margin.
// -------------------------------------------------------------------------
reg [24:0] por_cnt = 25'd0;
reg        reset_n = 1'b0;

always @(posedge clk_buf) begin
    if (!por_cnt[24]) begin
        por_cnt <= por_cnt + 25'd1;
    end else begin
        reset_n <= 1'b1;
    end
end

// -------------------------------------------------------------------------
// Heartbeat counter
// -------------------------------------------------------------------------
reg [31:0] hb_counter = 32'd0;

always @(posedge clk_buf) begin
    hb_counter <= hb_counter + 1'b1;
end

// -------------------------------------------------------------------------
// Auto-trigger self-test on rising edge of reset_n
// -------------------------------------------------------------------------
reg reset_n_d = 1'b0;
reg auto_trigger = 1'b0;

always @(posedge clk_buf) begin
    reset_n_d <= reset_n;
    auto_trigger <= (reset_n && !reset_n_d);  // Rising edge of reset_n
end

// -------------------------------------------------------------------------
// UART RX (receives commands from host)
// -------------------------------------------------------------------------
wire [7:0] rx_data;
wire       rx_data_valid;
wire       rx_frame_error;

uart_rx #(
    .CLK_FREQ(50_000_000),
    .BAUD_RATE(115200)
) uart_rx_inst (
    .clk(clk_buf),
    .reset_n(reset_n),
    .rxd(uart_rxd),
    .rx_data(rx_data),
    .rx_data_valid(rx_data_valid),
    .rx_frame_error(rx_frame_error)
);

// -------------------------------------------------------------------------
// Self-test trigger logic
// Host can send 'T' (0x54) to trigger self-test, or 'S' (0x53) for status.
// Also auto-triggers on power-up.
// -------------------------------------------------------------------------
wire host_trigger_test = rx_data_valid && (rx_data == 8'h54);  // 'T'
wire self_test_trigger = auto_trigger || host_trigger_test;

// -------------------------------------------------------------------------
// FPGA Self-Test Controller
// -------------------------------------------------------------------------
wire        self_test_busy;
wire        self_test_done;
wire [4:0]  result_flags;
wire [7:0]  result_detail;
wire        capture_active;
wire [15:0] capture_data;
wire        capture_valid;

fpga_self_test self_test_inst (
    .clk(clk_buf),
    .reset_n(reset_n),
    .trigger(self_test_trigger),
    .busy(self_test_busy),
    .result_valid(self_test_done),
    .result_flags(result_flags),
    .result_detail(result_detail),
    .adc_data_in(16'd0),          // No ADC connected in dev build
    .adc_valid_in(1'b0),
    .capture_active(capture_active),
    .capture_data(capture_data),
    .capture_valid(capture_valid)
);

// -------------------------------------------------------------------------
// Latch all-pass result for LED
// -------------------------------------------------------------------------
reg all_pass_latched = 1'b0;

always @(posedge clk_buf) begin
    if (!reset_n)
        all_pass_latched <= 1'b0;
    else if (self_test_done)
        all_pass_latched <= (result_flags == 5'b11111);
end

// -------------------------------------------------------------------------
// UART TX (driven by self-test reporter)
// -------------------------------------------------------------------------
wire [7:0] rpt_tx_data;
wire       rpt_tx_start;
wire       tx_busy;
wire       tx_done;

uart_tx #(
    .CLK_FREQ(50_000_000),
    .BAUD_RATE(115200)
) uart_tx_inst (
    .clk(clk_buf),
    .reset_n(reset_n),
    .tx_data(rpt_tx_data),
    .tx_start(rpt_tx_start),
    .txd(uart_txd),
    .tx_busy(tx_busy),
    .tx_done(tx_done)
);

// -------------------------------------------------------------------------
// Self-Test Reporter (formats results and sends via UART)
// -------------------------------------------------------------------------
wire report_busy;

uart_self_test_reporter #(
    .VERSION_MAJOR(8'd0),
    .VERSION_MINOR(8'd2)
) reporter_inst (
    .clk(clk_buf),
    .reset_n(reset_n),
    .self_test_done(self_test_done),
    .self_test_busy(self_test_busy),
    .result_flags(result_flags),
    .result_detail(result_detail),
    .rx_data(rx_data),
    .rx_data_valid(rx_data_valid),
    .heartbeat_cnt(hb_counter),
    .tx_data(rpt_tx_data),
    .tx_start(rpt_tx_start),
    .tx_busy(tx_busy),
    .report_busy(report_busy)
);

// -------------------------------------------------------------------------
// LED outputs (heartbeat, active indicators)
// -------------------------------------------------------------------------
assign user_led[0] = hb_counter[24];   // ~1.49 Hz blink
assign user_led[1] = hb_counter[25];   // ~0.75 Hz blink
assign user_led[2] = hb_counter[26];   // ~0.37 Hz blink
assign user_led[3] = hb_counter[27];   // ~0.19 Hz blink

// -------------------------------------------------------------------------
// Status outputs
// -------------------------------------------------------------------------
assign system_status[0] = tx_busy;           // UART TX activity
assign system_status[1] = self_test_busy;    // Self-test in progress
assign system_status[2] = all_pass_latched;  // All tests passed
assign system_status[3] = hb_counter[26];    // Heartbeat

endmodule
