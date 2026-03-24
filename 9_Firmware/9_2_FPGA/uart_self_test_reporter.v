`timescale 1ns / 1ps
// ============================================================================
// AERIS-10 UART Self-Test Reporter
// ============================================================================
//
// Bridges fpga_self_test results to UART TX output. When triggered (either
// by the self-test completion pulse or by a host status request via UART RX),
// this module serializes the self-test results as a human-readable ASCII
// status packet over UART.
//
// Packet format (fixed-length, 20 bytes):
//   Byte  0: 0xA5         (sync marker)
//   Byte  1: 0x5A         (sync marker)
//   Byte  2: 0x01         (packet type: self-test report)
//   Byte  3: result_flags [4:0] (bit per subsystem: 1=PASS, 0=FAIL)
//   Byte  4: result_detail[7:0] (diagnostic detail byte)
//   Byte  5: busy flag    (0x01 if test in progress, 0x00 if idle)
//   Byte  6: version major (firmware version)
//   Byte  7: version minor
//   Byte  8: heartbeat counter [31:24]
//   Byte  9: heartbeat counter [23:16]
//   Byte 10: heartbeat counter [15:8]
//   Byte 11: heartbeat counter [7:0]
//   Byte 12: ASCII 'P' or 'F' for BRAM test (human-readable summary)
//   Byte 13: ASCII 'P' or 'F' for CIC test
//   Byte 14: ASCII 'P' or 'F' for FFT test
//   Byte 15: ASCII 'P' or 'F' for ARITH test
//   Byte 16: ASCII 'P' or 'F' for ADC test
//   Byte 17: ASCII '\r'
//   Byte 18: ASCII '\n'
//   Byte 19: checksum (XOR of bytes 0..18)
//
// Trigger sources:
//   1. self_test_done pulse (from fpga_self_test result_valid)
//   2. Host request: UART RX receives byte 0x53 ('S' for Status)
//
// ============================================================================

module uart_self_test_reporter #(
    parameter VERSION_MAJOR = 8'd0,
    parameter VERSION_MINOR = 8'd2
) (
    input  wire        clk,
    input  wire        reset_n,

    // From fpga_self_test
    input  wire        self_test_done,     // 1-cycle pulse: results ready
    input  wire        self_test_busy,     // HIGH while test running
    input  wire [4:0]  result_flags,       // Per-test PASS/FAIL
    input  wire [7:0]  result_detail,      // Diagnostic detail

    // From UART RX (host command)
    input  wire [7:0]  rx_data,            // Received byte
    input  wire        rx_data_valid,      // 1-cycle pulse

    // Heartbeat counter (uptime indicator)
    input  wire [31:0] heartbeat_cnt,

    // To UART TX
    output reg  [7:0]  tx_data,
    output reg         tx_start,
    input  wire        tx_busy,

    // Status
    output reg         report_busy        // HIGH while sending report
);

    // Packet length
    localparam PKT_LEN = 20;

    // Status request command byte
    localparam CMD_STATUS = 8'h53;  // 'S'

    // State encoding
    localparam [1:0] ST_IDLE    = 2'd0,
                     ST_LOAD    = 2'd1,
                     ST_SEND    = 2'd2,
                     ST_WAIT    = 2'd3;

    reg [1:0]  state;
    reg [4:0]  byte_idx;
    reg [7:0]  pkt_buf [0:PKT_LEN-1];
    reg [7:0]  checksum;

    // Latched self-test results (capture on trigger)
    reg [4:0]  lat_flags;
    reg [7:0]  lat_detail;
    reg        lat_busy;
    reg [31:0] lat_hb;

    // Helper: pass/fail ASCII
    // Declared as regs before use (iverilog 13.0 forward-ref workaround)
    reg [7:0] pf_bram;
    reg [7:0] pf_cic;
    reg [7:0] pf_fft;
    reg [7:0] pf_arith;
    reg [7:0] pf_adc;

    // Trigger detection
    reg trigger_report;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            trigger_report <= 1'b0;
        end else begin
            if (state == ST_IDLE) begin
                if (self_test_done) begin
                    trigger_report <= 1'b1;
                end else if (rx_data_valid && rx_data == CMD_STATUS) begin
                    trigger_report <= 1'b1;
                end else begin
                    trigger_report <= 1'b0;
                end
            end else begin
                trigger_report <= 1'b0;
            end
        end
    end

    // Compute ASCII pass/fail characters (combinational)
    always @(*) begin
        pf_bram  = lat_flags[0] ? 8'h50 : 8'h46;  // 'P' or 'F'
        pf_cic   = lat_flags[1] ? 8'h50 : 8'h46;
        pf_fft   = lat_flags[2] ? 8'h50 : 8'h46;
        pf_arith = lat_flags[3] ? 8'h50 : 8'h46;
        pf_adc   = lat_flags[4] ? 8'h50 : 8'h46;
    end

    // Checksum computation (XOR of bytes 0..18)
    integer ci;
    reg [7:0] cksum_calc;
    always @(*) begin
        cksum_calc = 8'd0;
        for (ci = 0; ci < PKT_LEN - 1; ci = ci + 1) begin
            cksum_calc = cksum_calc ^ pkt_buf[ci];
        end
    end

    // Main FSM
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state       <= ST_IDLE;
            report_busy <= 1'b0;
            tx_start    <= 1'b0;
            tx_data     <= 8'd0;
            byte_idx    <= 5'd0;
            lat_flags   <= 5'd0;
            lat_detail  <= 8'd0;
            lat_busy    <= 1'b0;
            lat_hb      <= 32'd0;
        end else begin
            // Default
            tx_start <= 1'b0;

            case (state)
                ST_IDLE: begin
                    report_busy <= 1'b0;
                    if (trigger_report) begin
                        // Latch current values
                        lat_flags  <= result_flags;
                        lat_detail <= result_detail;
                        lat_busy   <= self_test_busy;
                        lat_hb     <= heartbeat_cnt;
                        report_busy <= 1'b1;
                        state      <= ST_LOAD;
                    end
                end

                ST_LOAD: begin
                    // Fill packet buffer
                    pkt_buf[0]  <= 8'hA5;
                    pkt_buf[1]  <= 8'h5A;
                    pkt_buf[2]  <= 8'h01;
                    pkt_buf[3]  <= {3'b000, lat_flags};
                    pkt_buf[4]  <= lat_detail;
                    pkt_buf[5]  <= {7'd0, lat_busy};
                    pkt_buf[6]  <= VERSION_MAJOR;
                    pkt_buf[7]  <= VERSION_MINOR;
                    pkt_buf[8]  <= lat_hb[31:24];
                    pkt_buf[9]  <= lat_hb[23:16];
                    pkt_buf[10] <= lat_hb[15:8];
                    pkt_buf[11] <= lat_hb[7:0];
                    pkt_buf[12] <= pf_bram;
                    pkt_buf[13] <= pf_cic;
                    pkt_buf[14] <= pf_fft;
                    pkt_buf[15] <= pf_arith;
                    pkt_buf[16] <= pf_adc;
                    pkt_buf[17] <= 8'h0D;  // '\r'
                    pkt_buf[18] <= 8'h0A;  // '\n'
                    // pkt_buf[19] filled next cycle from cksum_calc
                    byte_idx    <= 5'd0;
                    state       <= ST_SEND;
                end

                ST_SEND: begin
                    if (!tx_busy) begin
                        if (byte_idx < PKT_LEN - 1) begin
                            tx_data  <= pkt_buf[byte_idx];
                            tx_start <= 1'b1;
                            state    <= ST_WAIT;
                        end else if (byte_idx == PKT_LEN - 1) begin
                            // Send checksum byte
                            tx_data  <= cksum_calc;
                            tx_start <= 1'b1;
                            state    <= ST_WAIT;
                        end else begin
                            // All bytes sent
                            report_busy <= 1'b0;
                            state       <= ST_IDLE;
                        end
                    end
                end

                ST_WAIT: begin
                    // Wait for UART TX to accept byte and become busy
                    if (tx_busy) begin
                        byte_idx <= byte_idx + 5'd1;
                        state    <= ST_SEND;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
