`timescale 1ns / 1ps
//
// AERIS-10 TE0713+TE0701 VIO Build — Self-Test with JTAG Debug Readback
//
// Replaces UART path (blocked by TE0701 CPLD) with Vivado VIO debug core
// for reading self-test results over the existing JTAG connection.
//
// VIO Input Probes (FPGA → Vivado hardware manager):
//   probe_in0 [4:0]  — result_flags (per-subsystem pass/fail)
//   probe_in1 [7:0]  — result_detail (diagnostic byte)
//   probe_in2 [0:0]  — self_test_busy
//   probe_in3 [0:0]  — all_pass_latched
//   probe_in4 [31:0] — heartbeat_counter (confirms design is alive)
//   probe_in5 [7:0]  — version (major:minor packed)
//   probe_in6 [0:0]  — self_test_done pulse (latched for readback)
//
// VIO Output Probes (Vivado hardware manager → FPGA):
//   probe_out0 [0:0] — vio_trigger_test (press in Vivado to trigger self-test)
//   probe_out1 [0:0] — vio_request_status (press in Vivado to request status)
//
// Uses TE0713 FIFO0CLK (50 MHz, Bank 14, LVCMOS33) at pin U20.
// LEDs on Bank 16 FMC LA pins (LVCMOS33).
//

module radar_system_top_te0713_vio (
    input  wire       clk_100m,        // TE0713 FIFO0CLK (actually 50 MHz)
    output wire [3:0] user_led,
    output wire [3:0] system_status
);

// =========================================================================
// Parameters
// =========================================================================
localparam [7:0] VERSION_MAJOR = 8'd0;
localparam [7:0] VERSION_MINOR = 8'd3;   // v0.3 = VIO build

// =========================================================================
// Clock buffer
// =========================================================================
wire clk_buf;

`ifdef SIMULATION
    assign clk_buf = clk_100m;
`else
    BUFG bufg_clk (
        .I(clk_100m),
        .O(clk_buf)
    );
`endif

// =========================================================================
// Power-on reset generator (~500ms at 50 MHz = 25M cycles)
// Use 25-bit counter (33.5M counts) for margin.
// =========================================================================
reg [24:0] por_cnt = 25'd0;
reg        reset_n = 1'b0;

always @(posedge clk_buf) begin
    if (!por_cnt[24]) begin
        por_cnt <= por_cnt + 25'd1;
    end else begin
        reset_n <= 1'b1;
    end
end

// =========================================================================
// Heartbeat counter
// =========================================================================
reg [31:0] hb_counter = 32'd0;

always @(posedge clk_buf) begin
    hb_counter <= hb_counter + 1'b1;
end

// =========================================================================
// VIO probes — directly declared as wires / regs
// =========================================================================
// Output probes (from VIO → FPGA logic)
wire       vio_trigger_test;
wire       vio_request_status;

// =========================================================================
// Auto-trigger self-test on rising edge of reset_n
// =========================================================================
reg reset_n_d = 1'b0;
reg auto_trigger = 1'b0;

always @(posedge clk_buf) begin
    reset_n_d <= reset_n;
    auto_trigger <= (reset_n && !reset_n_d);  // Rising edge of reset_n
end

// =========================================================================
// VIO trigger edge detection
// The VIO output is a level signal. We need to detect rising edges to
// generate one-cycle trigger pulses.
// =========================================================================
reg vio_trigger_d = 1'b0;
reg vio_status_d  = 1'b0;
wire vio_trigger_pulse = vio_trigger_test && !vio_trigger_d;
wire vio_status_pulse  = vio_request_status && !vio_status_d;

always @(posedge clk_buf) begin
    vio_trigger_d <= vio_trigger_test;
    vio_status_d  <= vio_request_status;
end

// =========================================================================
// Self-test trigger: auto-trigger on power-up OR VIO trigger
// =========================================================================
wire self_test_trigger = auto_trigger || vio_trigger_pulse;

// =========================================================================
// FPGA Self-Test Controller
// =========================================================================
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

// =========================================================================
// Latch all-pass result for LED and VIO readback
// =========================================================================
reg all_pass_latched = 1'b0;

always @(posedge clk_buf) begin
    if (!reset_n)
        all_pass_latched <= 1'b0;
    else if (self_test_done)
        all_pass_latched <= (result_flags == 5'b11111);
end

// =========================================================================
// Latch self_test_done for VIO readback (stays high until next trigger)
// =========================================================================
reg test_done_latched = 1'b0;

always @(posedge clk_buf) begin
    if (!reset_n || self_test_trigger)
        test_done_latched <= 1'b0;
    else if (self_test_done)
        test_done_latched <= 1'b1;
end

// =========================================================================
// Latch result_flags for stable VIO readback (hold until next trigger)
// =========================================================================
reg [4:0]  result_flags_latched = 5'd0;
reg [7:0]  result_detail_latched = 8'd0;

always @(posedge clk_buf) begin
    if (!reset_n) begin
        result_flags_latched  <= 5'd0;
        result_detail_latched <= 8'd0;
    end else if (self_test_done) begin
        result_flags_latched  <= result_flags;
        result_detail_latched <= result_detail;
    end
end

// =========================================================================
// VIO Debug Core instantiation (synthesis only — stubbed for simulation)
// =========================================================================
// NOTE: For iverilog 13.0 compatibility, reg declarations MUST appear
// BEFORE the assign statements that reference them (no forward refs).
`ifdef SIMULATION
    // Simulation-only regs driven by testbench via hierarchical references
    reg sim_vio_trigger = 1'b0;
    reg sim_vio_status  = 1'b0;

    assign vio_trigger_test   = sim_vio_trigger;
    assign vio_request_status = sim_vio_status;
`else
    // Vivado VIO IP core — generated by build_te0713_vio.tcl
    // 7 input probes, 2 output probes
    vio_0 vio_debug_inst (
        .clk        (clk_buf),
        // Input probes: FPGA → Vivado
        .probe_in0  (result_flags_latched),   // [4:0]  pass/fail flags
        .probe_in1  (result_detail_latched),  // [7:0]  diagnostic detail
        .probe_in2  (self_test_busy),         // [0:0]  busy
        .probe_in3  (all_pass_latched),       // [0:0]  all-pass
        .probe_in4  (hb_counter),             // [31:0] heartbeat
        .probe_in5  ({VERSION_MAJOR[3:0], VERSION_MINOR[3:0]}), // [7:0] version
        .probe_in6  (test_done_latched),      // [0:0]  test done
        // Output probes: Vivado → FPGA
        .probe_out0 (vio_trigger_test),       // [0:0]  trigger self-test
        .probe_out1 (vio_request_status)      // [0:0]  request status
    );
`endif

// =========================================================================
// LED outputs (heartbeat, active indicators)
// =========================================================================
assign user_led[0] = hb_counter[24];   // ~1.49 Hz blink
assign user_led[1] = hb_counter[25];   // ~0.75 Hz blink
assign user_led[2] = hb_counter[26];   // ~0.37 Hz blink
assign user_led[3] = hb_counter[27];   // ~0.19 Hz blink

// =========================================================================
// Status outputs
// =========================================================================
assign system_status[0] = self_test_busy;    // Self-test in progress
assign system_status[1] = test_done_latched; // Test completed
assign system_status[2] = all_pass_latched;  // All tests passed
assign system_status[3] = hb_counter[26];    // Heartbeat

endmodule
