`timescale 1ns / 1ps
// ============================================================================
// AERIS-10 TE0713+TE0701 UART TX Blaster Top — Diagnostic Build
// ============================================================================
//
// Minimal top-level that continuously blasts UART data on BOTH candidate pins
// (P19 and U18) simultaneously. This bypasses all self-test/reporter complexity
// to determine if the TE0701 CPLD routes UART data to the FTDI at all.
//
// Both P19 and U18 are configured as OUTPUTS driving the same serial stream.
// The host should see data on /dev/ttyUSB0 if EITHER pin connects to FTDI RX.
//
// LEDs blink as heartbeat (alive indicator).
// system_status[0] = TX active, [3] = heartbeat.
//
// ============================================================================

module uart_tx_blaster_top (
    input  wire       clk_100m,        // TE0713 FIFO0CLK (actually 50 MHz)
    output wire       uart_pin_p19,    // Candidate UART pin (MIO14 path)
    output wire       uart_pin_u18,    // Candidate UART pin (MIO15 path)
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
// UART TX Blaster — sends pattern continuously
// -------------------------------------------------------------------------
wire txd_out;
wire tx_active;

uart_tx_blaster #(
    .CLK_FREQ(50_000_000),
    .BAUD_RATE(115200),
    .GAP_CYCLES(50_000)    // ~1ms between bytes
) u_blaster (
    .clk(clk_buf),
    .reset_n(reset_n),
    .txd(txd_out),
    .tx_active(tx_active)
);

// -------------------------------------------------------------------------
// Drive BOTH pins with the same serial output
// If one of these pins is the real TX path through the CPLD, we'll see data.
// -------------------------------------------------------------------------
assign uart_pin_p19 = txd_out;
assign uart_pin_u18 = txd_out;

// -------------------------------------------------------------------------
// LED outputs (heartbeat, active indicators)
// -------------------------------------------------------------------------
assign user_led[0] = hb_counter[24];   // ~1.49 Hz blink
assign user_led[1] = hb_counter[25];   // ~0.75 Hz blink
assign user_led[2] = tx_active;         // TX activity
assign user_led[3] = hb_counter[27];   // ~0.19 Hz blink

// -------------------------------------------------------------------------
// Status outputs
// -------------------------------------------------------------------------
assign system_status[0] = tx_active;         // TX activity
assign system_status[1] = 1'b0;             // Unused
assign system_status[2] = 1'b0;             // Unused
assign system_status[3] = hb_counter[26];   // Heartbeat

endmodule
