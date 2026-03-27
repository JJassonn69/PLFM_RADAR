`timescale 1ns / 1ps
// ============================================================================
// Minimal blinky test — FT601 stability diagnostic
// ============================================================================
// Purpose: Determine if FT601 USB disconnect in 245 mode is caused by our
// RTL or by a hardware/configuration issue.
//
// This design does NOTHING except:
//   1. Blink an LED on ft601_gpio0 using the on-board 50 MHz oscillator
//   2. Hold ft601_chip_reset_n HIGH (FT601 stays running)
//   3. Hold all FT601 bus signals in their idle state
//   4. Leave ft601_clk_in connected but unused (just an IBUF)
//
// If the FT601 disconnects with THIS bitstream, the issue is NOT our RTL —
// it's hardware (wiring, power, signal integrity during config, etc.)
//
// If the FT601 stays connected, the issue is in our RTL's interaction with
// the FT601 bus (control signals, clock domain, etc.)
// ============================================================================

module blinky_ft601_test (
    input  wire        osc_50m,           // TE0713 on-board 50 MHz oscillator (U20)
    input  wire        ft601_clk_in,      // 100 MHz clock from FT601 (connected but unused)
    inout  wire [31:0] ft601_data,        // Tristated (not driven)
    output wire [3:0]  ft601_be,          // Driven to 4'b1111 (all bytes, idle)
    input  wire        ft601_txe,         // FT601 status (ignored)
    input  wire        ft601_rxf,         // FT601 status (ignored)
    output wire        ft601_wr_n,        // HIGH = idle (no write)
    output wire        ft601_rd_n,        // HIGH = idle (no read)
    output wire        ft601_oe_n,        // HIGH = idle (no output enable)
    output wire        ft601_siwu_n,      // HIGH = idle
    output wire        ft601_chip_reset_n,// HIGH = not in reset
    output wire        ft601_wakeup_n,    // HIGH = no wakeup
    output wire        ft601_gpio0,       // BLINK LED (~1 Hz)
    output wire        ft601_gpio1        // Steady ON = design is running
);

    // =====================================================================
    // Blink counter on osc_50m
    // =====================================================================
    reg [25:0] blink_counter = 26'd0;

    always @(posedge osc_50m)
        blink_counter <= blink_counter + 1'b1;

    // =====================================================================
    // Output assignments
    // =====================================================================

    // FT601 chip reset — ALWAYS HIGH, never reset the FT601
    assign ft601_chip_reset_n = 1'b1;

    // FT601 wakeup — tied HIGH (inactive)
    assign ft601_wakeup_n = 1'b1;

    // FT601 bus — all idle
    assign ft601_wr_n  = 1'b1;  // No write
    assign ft601_rd_n  = 1'b1;  // No read
    assign ft601_oe_n  = 1'b1;  // No output enable (data bus is input from FPGA side)
    assign ft601_siwu_n = 1'b1; // No send-immediate/wakeup
    assign ft601_be    = 4'b1111; // All byte enables active (doesn't matter when idle)

    // Data bus — NOT driven (tristate). The pulldowns in the XDC/on-board
    // will keep it at a known state.
    assign ft601_data = 32'bz;

    // GPIO LEDs
    assign ft601_gpio0 = blink_counter[24]; // ~1.5 Hz blink (50M / 2^25)
    assign ft601_gpio1 = 1'b1;              // Steady ON = FPGA is configured

endmodule
