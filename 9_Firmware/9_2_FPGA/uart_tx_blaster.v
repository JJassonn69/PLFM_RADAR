`timescale 1ns / 1ps
// ============================================================================
// AERIS-10 UART TX Blaster — Diagnostic Module
// ============================================================================
//
// Continuously transmits a rotating pattern of bytes at 115200 baud on a
// single serial output. The top-level mirrors this output to BOTH candidate
// UART pins (P19 and U18) so we can determine which pin the TE0701 CPLD
// actually routes to the FTDI Channel B RX input.
//
// Pattern: 0x55 ("U"), 0xAA, 0x0F, 0xF0, 0x55, ...
//   - 0x55 = alternating bits (good for baud rate detection)
//   - 0xAA = complementary alternating bits
//   - 0x0F / 0xF0 = nibble boundaries
//
// Also inserts a short inter-byte gap (~1 ms at 50 MHz) between transmissions
// for cleaner framing on the host side.
//
// ============================================================================

module uart_tx_blaster #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115200,
    parameter GAP_CYCLES = 50_000   // ~1 ms at 50 MHz between bytes
) (
    input  wire clk,
    input  wire reset_n,
    output wire txd,        // Serial output (idle HIGH)
    output wire tx_active   // HIGH when transmitting (for LED)
);

    // Pattern ROM — 4 bytes
    reg [7:0] pattern [0:3];
    initial begin
        pattern[0] = 8'h55;  // "U"
        pattern[1] = 8'hAA;
        pattern[2] = 8'h0F;
        pattern[3] = 8'hF0;
    end

    // State machine
    localparam ST_IDLE    = 2'd0,
               ST_SEND    = 2'd1,
               ST_WAIT_TX = 2'd2,
               ST_GAP     = 2'd3;

    reg [1:0]  state;
    reg [1:0]  pat_idx;     // Index into pattern ROM
    reg [15:0] gap_cnt;     // Inter-byte gap counter
    reg        tx_start_r;
    reg [7:0]  tx_data_r;

    wire tx_busy;
    wire tx_done;

    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_tx (
        .clk(clk),
        .reset_n(reset_n),
        .tx_data(tx_data_r),
        .tx_start(tx_start_r),
        .txd(txd),
        .tx_busy(tx_busy),
        .tx_done(tx_done)
    );

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state      <= ST_IDLE;
            pat_idx    <= 2'd0;
            gap_cnt    <= 16'd0;
            tx_start_r <= 1'b0;
            tx_data_r  <= 8'd0;
        end else begin
            tx_start_r <= 1'b0;  // Default: deassert

            case (state)
                ST_IDLE: begin
                    // Start sending immediately after reset
                    state <= ST_SEND;
                end

                ST_SEND: begin
                    tx_data_r  <= pattern[pat_idx];
                    tx_start_r <= 1'b1;
                    state      <= ST_WAIT_TX;
                end

                ST_WAIT_TX: begin
                    if (tx_done) begin
                        pat_idx <= pat_idx + 2'd1;  // Wraps at 4
                        gap_cnt <= 16'd0;
                        state   <= ST_GAP;
                    end
                end

                ST_GAP: begin
                    if (gap_cnt >= GAP_CYCLES[15:0] - 1) begin
                        state <= ST_SEND;
                    end else begin
                        gap_cnt <= gap_cnt + 16'd1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    assign tx_active = tx_busy;

endmodule
