`timescale 1ns / 1ps
// ============================================================================
// AERIS-10 UART Transmitter — 8N1
// ============================================================================
//
// Parameterized UART transmitter. Sends one byte at a time in 8N1 format
// (1 start bit, 8 data bits LSB-first, 1 stop bit, no parity).
//
// Interface:
//   - Assert tx_start for one clock cycle with tx_data valid.
//   - tx_busy goes HIGH immediately and stays HIGH until the stop bit completes.
//   - tx_done pulses HIGH for one cycle when the byte finishes.
//   - txd is the serial output line (idle HIGH).
//
// Baud rate:
//   BAUD_DIV = CLK_FREQ / BAUD_RATE  (e.g. 50_000_000 / 115200 = 434)
//
// ============================================================================

module uart_tx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD_RATE = 115200
) (
    input  wire       clk,
    input  wire       reset_n,
    input  wire [7:0] tx_data,     // Byte to transmit
    input  wire       tx_start,    // 1-cycle strobe to begin transmission
    output reg        txd,         // Serial output (idle HIGH)
    output reg        tx_busy,     // HIGH while transmitting
    output reg        tx_done      // 1-cycle pulse when byte finishes
);

    // Baud divider — number of clk cycles per bit
    localparam integer BAUD_DIV = CLK_FREQ / BAUD_RATE;

    // State encoding
    localparam [1:0] ST_IDLE  = 2'd0,
                     ST_START = 2'd1,
                     ST_DATA  = 2'd2,
                     ST_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] baud_cnt;    // Baud rate counter (up to 65535)
    reg [2:0]  bit_idx;     // Data bit index (0..7)
    reg [7:0]  tx_shift;    // Shift register holding current byte

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state    <= ST_IDLE;
            txd      <= 1'b1;
            tx_busy  <= 1'b0;
            tx_done  <= 1'b0;
            baud_cnt <= 16'd0;
            bit_idx  <= 3'd0;
            tx_shift <= 8'd0;
        end else begin
            // Default: clear done pulse
            tx_done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    txd <= 1'b1;  // Idle line HIGH
                    if (tx_start) begin
                        tx_shift <= tx_data;
                        tx_busy  <= 1'b1;
                        baud_cnt <= 16'd0;
                        state    <= ST_START;
                    end
                end

                ST_START: begin
                    txd <= 1'b0;  // Start bit = LOW
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 16'd0;
                        bit_idx  <= 3'd0;
                        state    <= ST_DATA;
                    end else begin
                        baud_cnt <= baud_cnt + 16'd1;
                    end
                end

                ST_DATA: begin
                    txd <= tx_shift[0];  // LSB first
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 16'd0;
                        tx_shift <= {1'b0, tx_shift[7:1]};  // Shift right
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
                    txd <= 1'b1;  // Stop bit = HIGH
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 16'd0;
                        tx_busy  <= 1'b0;
                        tx_done  <= 1'b1;
                        state    <= ST_IDLE;
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
