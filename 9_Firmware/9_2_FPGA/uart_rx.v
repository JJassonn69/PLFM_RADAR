`timescale 1ns / 1ps
// ============================================================================
// AERIS-10 UART Receiver — 8N1
// ============================================================================
//
// Parameterized UART receiver. Receives one byte at a time in 8N1 format
// (1 start bit, 8 data bits LSB-first, 1 stop bit, no parity).
//
// Interface:
//   - rxd is the serial input line (active-LOW start bit).
//   - rx_data_valid pulses HIGH for one cycle when a complete byte is received.
//   - rx_data holds the received byte (valid when rx_data_valid is HIGH).
//   - rx_frame_error is HIGH if the stop bit was not detected (framing error).
//
// Sampling strategy:
//   - Detect falling edge of rxd (start bit).
//   - Wait half a bit period to sample at center of each bit.
//   - Sample 8 data bits at bit-center, then check stop bit.
//
// Input synchronization:
//   - rxd is double-registered to avoid metastability.
//
// ============================================================================

module uart_rx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD_RATE = 115200
) (
    input  wire       clk,
    input  wire       reset_n,
    input  wire       rxd,           // Serial input (idle HIGH)
    output reg  [7:0] rx_data,       // Received byte
    output reg        rx_data_valid, // 1-cycle pulse: byte ready
    output reg        rx_frame_error // HIGH if stop bit missing
);

    // Baud divider — number of clk cycles per bit
    localparam integer BAUD_DIV = CLK_FREQ / BAUD_RATE;
    // Half-bit period for center sampling
    localparam integer HALF_DIV = BAUD_DIV / 2;

    // State encoding
    localparam [1:0] ST_IDLE  = 2'd0,
                     ST_START = 2'd1,
                     ST_DATA  = 2'd2,
                     ST_STOP  = 2'd3;

    // Input synchronizer (2-stage FF)
    reg rxd_r1;
    reg rxd_r2;

    reg [1:0]  state;
    reg [15:0] baud_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  rx_shift;

    // Synchronize rxd input
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rxd_r1 <= 1'b1;
            rxd_r2 <= 1'b1;
        end else begin
            rxd_r1 <= rxd;
            rxd_r2 <= rxd_r1;
        end
    end

    // Main receiver FSM
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state          <= ST_IDLE;
            rx_data        <= 8'd0;
            rx_data_valid  <= 1'b0;
            rx_frame_error <= 1'b0;
            baud_cnt       <= 16'd0;
            bit_idx        <= 3'd0;
            rx_shift       <= 8'd0;
        end else begin
            // Default: clear pulses
            rx_data_valid  <= 1'b0;
            rx_frame_error <= 1'b0;

            case (state)
                ST_IDLE: begin
                    // Wait for start bit (falling edge: rxd goes LOW)
                    if (rxd_r2 == 1'b0) begin
                        baud_cnt <= 16'd0;
                        state    <= ST_START;
                    end
                end

                ST_START: begin
                    // Wait to center of start bit (half bit period)
                    if (baud_cnt == HALF_DIV - 1) begin
                        // Verify start bit is still LOW
                        if (rxd_r2 == 1'b0) begin
                            baud_cnt <= 16'd0;
                            bit_idx  <= 3'd0;
                            rx_shift <= 8'd0;
                            state    <= ST_DATA;
                        end else begin
                            // False start — go back to idle
                            state <= ST_IDLE;
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 16'd1;
                    end
                end

                ST_DATA: begin
                    // Wait one full bit period, then sample
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 16'd0;
                        // Sample data bit (LSB first)
                        rx_shift <= {rxd_r2, rx_shift[7:1]};
                        if (bit_idx == 3'd7) begin
                            state <= ST_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 16'd1;
                    end
                end

                ST_STOP: begin
                    // Wait one full bit period, then check stop bit
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 16'd0;
                        rx_data  <= rx_shift;
                        if (rxd_r2 == 1'b1) begin
                            // Valid stop bit
                            rx_data_valid <= 1'b1;
                        end else begin
                            // Framing error — stop bit not HIGH
                            rx_frame_error <= 1'b1;
                        end
                        state <= ST_IDLE;
                    end else begin
                        baud_cnt <= baud_cnt + 16'd1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
