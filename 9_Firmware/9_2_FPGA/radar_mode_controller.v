`timescale 1ns / 1ps

`include "radar_params.vh"

/**
 * radar_mode_controller.v
 *
 * Generates beam scanning and chirp mode control signals for the AERIS-10
 * receiver processing chain. This module drives:
 *   - use_long_chirp   : selects long (30us) or short (0.5us) chirp mode
 *   - mc_new_chirp     : toggle signal indicating new chirp start
 *   - mc_new_elevation : toggle signal indicating elevation step
 *   - mc_new_azimuth   : toggle signal indicating azimuth step
 *
 * These signals are consumed by matched_filter_multi_segment and
 * chirp_memory_loader_param in the receiver path.
 *
 * The controller mirrors the transmitter's chirp sequence defined in
 * plfm_chirp_controller_enhanced:
 *   - 32 chirps per elevation
 *   - 31 elevations per azimuth
 *   - 50 azimuths per full scan
 *
 * Chirp sequence depends on range_mode (host_range_mode, opcode 0x20):
 *   range_mode 2'b00 (3 km):  All short chirps only. Long chirp blind zone
 *     (4500 m) exceeds 3 km max range, so long chirps are useless.
 *   range_mode 2'b01 (long-range): Dual chirp — Long chirp → Listen → Guard
 *     → Short chirp → Listen. First half of chirps_per_elev are long, second
 *     half are short (blind-zone fill).
 *
 * Modes of operation (host_radar_mode, opcode 0x01):
 *   mode[1:0]:
 *     2'b00 = STM32-driven (pass through stm32 toggle signals)
 *     2'b01 = Free-running auto-scan (internal timing, short chirps only)
 *     2'b10 = Single-chirp (fire one chirp per trigger, for debug)
 *     2'b11 = Reserved
 *
 * Clock domain: clk (100 MHz)
 */

module radar_mode_controller #(
    parameter CHIRPS_PER_ELEVATION  = `RP_DEF_CHIRPS_PER_ELEV,
    parameter ELEVATIONS_PER_AZIMUTH = 31,
    parameter AZIMUTHS_PER_SCAN     = 50,

    // Timing in 100 MHz clock cycles
    // Long chirp: 30us = 3000 cycles at 100 MHz
    // Long listen: 137us = 13700 cycles
    // Guard: 175.4us = 17540 cycles
    // Short chirp: 0.5us = 50 cycles
    // Short listen: 174.5us = 17450 cycles
    parameter LONG_CHIRP_CYCLES   = `RP_DEF_LONG_CHIRP_CYCLES,
    parameter LONG_LISTEN_CYCLES  = `RP_DEF_LONG_LISTEN_CYCLES,
    parameter GUARD_CYCLES        = `RP_DEF_GUARD_CYCLES,
    parameter SHORT_CHIRP_CYCLES  = `RP_DEF_SHORT_CHIRP_CYCLES,
    parameter SHORT_LISTEN_CYCLES = `RP_DEF_SHORT_LISTEN_CYCLES
) (
    input wire clk,
    input wire reset_n,

    // Mode selection (host_radar_mode, opcode 0x01)
    input wire [1:0] mode,          // 00=STM32, 01=auto, 10=single, 11=rsvd

    // Range mode (host_range_mode, opcode 0x20)
    // Determines chirp type selection in pass-through and auto-scan modes.
    //   2'b00 = 3 km  (all short chirps — long blind zone > max range)
    //   2'b01 = Long-range (dual chirp: first half long, second half short)
    input wire [1:0] range_mode,

    // STM32 pass-through inputs (active in mode 00)
    input wire stm32_new_chirp,
    input wire stm32_new_elevation,
    input wire stm32_new_azimuth,

    // Single-chirp trigger (active in mode 10)
    input wire trigger,

    // Runtime-configurable timing inputs from host USB commands.
    // When connected, these override the compile-time parameters.
    input wire [15:0] cfg_long_chirp_cycles,
    input wire [15:0] cfg_long_listen_cycles,
    input wire [15:0] cfg_guard_cycles,
    input wire [15:0] cfg_short_chirp_cycles,
    input wire [15:0] cfg_short_listen_cycles,
    input wire [5:0]  cfg_chirps_per_elev,

    // Outputs to receiver processing chain
    output reg use_long_chirp,
    output reg mc_new_chirp,
    output reg mc_new_elevation,
    output reg mc_new_azimuth,

    // Beam position tracking
    output reg [5:0] chirp_count,
    output reg [5:0] elevation_count,
    output reg [5:0] azimuth_count,

    // Status
    output wire scanning,       // 1 = scan in progress
    output wire scan_complete   // pulse when full scan done

`ifdef FORMAL
    ,
    output wire [2:0]  fv_scan_state,
    output wire [17:0] fv_timer
`endif
);

// ============================================================================
// INTERNAL STATE
// ============================================================================

// Auto-scan state machine
reg [2:0] scan_state;
localparam S_IDLE        = 3'd0;
localparam S_LONG_CHIRP  = 3'd1;
localparam S_LONG_LISTEN = 3'd2;
localparam S_GUARD       = 3'd3;
localparam S_SHORT_CHIRP = 3'd4;
localparam S_SHORT_LISTEN = 3'd5;
localparam S_ADVANCE     = 3'd6;

// Timing counter
reg [17:0] timer;  // enough for up to 262143 cycles (~2.6ms at 100 MHz)

`ifdef FORMAL
assign fv_scan_state = scan_state;
assign fv_timer      = timer;
`endif

// Edge detection for STM32 pass-through
reg stm32_new_chirp_prev;
reg stm32_new_elevation_prev;
reg stm32_new_azimuth_prev;

// Trigger edge detection (for single-chirp mode)
reg trigger_prev;
wire trigger_pulse = trigger & ~trigger_prev;

// Scan completion
reg scan_done_pulse;

// ============================================================================
// EDGE DETECTION
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        stm32_new_chirp_prev     <= 1'b0;
        stm32_new_elevation_prev <= 1'b0;
        stm32_new_azimuth_prev   <= 1'b0;
        trigger_prev             <= 1'b0;
    end else begin
        stm32_new_chirp_prev     <= stm32_new_chirp;
        stm32_new_elevation_prev <= stm32_new_elevation;
        stm32_new_azimuth_prev   <= stm32_new_azimuth;
        trigger_prev             <= trigger;
    end
end

wire stm32_chirp_toggle     = stm32_new_chirp     ^ stm32_new_chirp_prev;
wire stm32_elevation_toggle = stm32_new_elevation  ^ stm32_new_elevation_prev;
wire stm32_azimuth_toggle   = stm32_new_azimuth    ^ stm32_new_azimuth_prev;

// ============================================================================
// MAIN STATE MACHINE
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        scan_state      <= S_IDLE;
        timer           <= 18'd0;
        use_long_chirp  <= 1'b0;  // Default short chirp (safe for 3 km mode)
        mc_new_chirp    <= 1'b0;
        mc_new_elevation <= 1'b0;
        mc_new_azimuth  <= 1'b0;
        chirp_count     <= 6'd0;
        elevation_count <= 6'd0;
        azimuth_count   <= 6'd0;
        scan_done_pulse <= 1'b0;
    end else begin
        // Clear one-shot signals
        scan_done_pulse <= 1'b0;

        case (mode)
        // ================================================================
        // MODE 00: STM32-driven pass-through
        // The STM32 firmware controls timing; we just detect toggle edges
        // and forward them to the receiver chain. Chirp type is determined
        // by range_mode:
        //   range_mode 00 (3 km):  ALL chirps are short (long blind zone
        //     4500 m exceeds 3072 m max range, so long chirps are useless).
        //   range_mode 01 (long-range): First half of chirps_per_elev are
        //     long, second half are short (blind-zone fill).
        // ================================================================
        2'b00: begin
            // Reset auto-scan state
            scan_state <= S_IDLE;
            timer      <= 18'd0;

            // Pass through toggle signals
            if (stm32_chirp_toggle) begin
                mc_new_chirp <= ~mc_new_chirp;  // Toggle output

                // Determine chirp type based on range_mode
                case (range_mode)
                `RP_RANGE_MODE_3KM: begin
                    // 3 km mode: all short chirps
                    use_long_chirp <= 1'b0;
                end
                `RP_RANGE_MODE_LONG: begin
                    // Long-range: first half long, second half short.
                    // chirps_per_elev is typically 32 (16 long + 16 short).
                    // Use cfg_chirps_per_elev[5:1] as the halfway point.
                    if (chirp_count < {1'b0, cfg_chirps_per_elev[5:1]})
                        use_long_chirp <= 1'b1;
                    else
                        use_long_chirp <= 1'b0;
                end
                default: begin
                    // Reserved modes: default to short chirp (safe)
                    use_long_chirp <= 1'b0;
                end
                endcase

                // Track chirp count
                if (chirp_count < cfg_chirps_per_elev - 1)
                    chirp_count <= chirp_count + 1;
                else
                    chirp_count <= 6'd0;
            end

            if (stm32_elevation_toggle) begin
                mc_new_elevation <= ~mc_new_elevation;
                chirp_count <= 6'd0;

                if (elevation_count < ELEVATIONS_PER_AZIMUTH - 1)
                    elevation_count <= elevation_count + 1;
                else
                    elevation_count <= 6'd0;
            end

            if (stm32_azimuth_toggle) begin
                mc_new_azimuth <= ~mc_new_azimuth;
                elevation_count <= 6'd0;

                if (azimuth_count < AZIMUTHS_PER_SCAN - 1)
                    azimuth_count <= azimuth_count + 1;
                else begin
                    azimuth_count <= 6'd0;
                    scan_done_pulse <= 1'b1;
                end
            end
        end

        // ================================================================
        // MODE 01: Free-running auto-scan
        // Internally generates chirp timing matching the transmitter.
        // For 3 km mode (range_mode 00): short chirps only. The long chirp
        // blind zone (4500 m) exceeds the 3072 m max range, making long
        // chirps useless. State machine skips S_LONG_CHIRP/LISTEN/GUARD.
        // For long-range mode (range_mode 01): full dual-chirp sequence.
        // NOTE: Auto-scan is primarily for bench testing without STM32.
        // ================================================================
        2'b01: begin
            case (scan_state)
            S_IDLE: begin
                // Start first chirp immediately
                timer           <= 18'd0;
                chirp_count     <= 6'd0;
                elevation_count <= 6'd0;
                azimuth_count   <= 6'd0;
                mc_new_chirp    <= ~mc_new_chirp;  // Toggle to start chirp

                // For 3 km mode, skip directly to short chirp
                if (range_mode == `RP_RANGE_MODE_3KM) begin
                    scan_state     <= S_SHORT_CHIRP;
                    use_long_chirp <= 1'b0;
                end else begin
                    scan_state     <= S_LONG_CHIRP;
                    use_long_chirp <= 1'b1;
                end

                `ifdef SIMULATION
                $display("[MODE_CTRL] Auto-scan starting, range_mode=%0d", range_mode);
                `endif
            end

            S_LONG_CHIRP: begin
                use_long_chirp <= 1'b1;
                if (timer < cfg_long_chirp_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_LONG_LISTEN;
                end
            end

            S_LONG_LISTEN: begin
                if (timer < cfg_long_listen_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_GUARD;
                end
            end

            S_GUARD: begin
                if (timer < cfg_guard_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_SHORT_CHIRP;
                    use_long_chirp <= 1'b0;
                end
            end

            S_SHORT_CHIRP: begin
                use_long_chirp <= 1'b0;
                if (timer < cfg_short_chirp_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_SHORT_LISTEN;
                end
            end

            S_SHORT_LISTEN: begin
                if (timer < cfg_short_listen_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_ADVANCE;
                end
            end

            S_ADVANCE: begin
                // Advance chirp/elevation/azimuth counters
                if (chirp_count < cfg_chirps_per_elev - 1) begin
                    // Next chirp in current elevation
                    chirp_count  <= chirp_count + 1;
                    mc_new_chirp <= ~mc_new_chirp;

                    // For 3 km mode: short chirps only, skip long phases
                    if (range_mode == `RP_RANGE_MODE_3KM) begin
                        scan_state     <= S_SHORT_CHIRP;
                        use_long_chirp <= 1'b0;
                    end else begin
                        scan_state     <= S_LONG_CHIRP;
                        use_long_chirp <= 1'b1;
                    end
                end else begin
                    chirp_count <= 6'd0;

                    if (elevation_count < ELEVATIONS_PER_AZIMUTH - 1) begin
                        // Next elevation
                        elevation_count  <= elevation_count + 1;
                        mc_new_chirp     <= ~mc_new_chirp;
                        mc_new_elevation <= ~mc_new_elevation;

                        if (range_mode == `RP_RANGE_MODE_3KM) begin
                            scan_state     <= S_SHORT_CHIRP;
                            use_long_chirp <= 1'b0;
                        end else begin
                            scan_state     <= S_LONG_CHIRP;
                            use_long_chirp <= 1'b1;
                        end
                    end else begin
                        elevation_count <= 6'd0;

                        if (azimuth_count < AZIMUTHS_PER_SCAN - 1) begin
                            // Next azimuth
                            azimuth_count    <= azimuth_count + 1;
                            mc_new_chirp     <= ~mc_new_chirp;
                            mc_new_elevation <= ~mc_new_elevation;
                            mc_new_azimuth   <= ~mc_new_azimuth;

                            if (range_mode == `RP_RANGE_MODE_3KM) begin
                                scan_state     <= S_SHORT_CHIRP;
                                use_long_chirp <= 1'b0;
                            end else begin
                                scan_state     <= S_LONG_CHIRP;
                                use_long_chirp <= 1'b1;
                            end
                        end else begin
                            // Full scan complete — restart
                            azimuth_count   <= 6'd0;
                            scan_done_pulse <= 1'b1;
                            mc_new_chirp    <= ~mc_new_chirp;
                            mc_new_elevation <= ~mc_new_elevation;
                            mc_new_azimuth  <= ~mc_new_azimuth;

                            if (range_mode == `RP_RANGE_MODE_3KM) begin
                                scan_state     <= S_SHORT_CHIRP;
                                use_long_chirp <= 1'b0;
                            end else begin
                                scan_state     <= S_LONG_CHIRP;
                                use_long_chirp <= 1'b1;
                            end

                            `ifdef SIMULATION
                            $display("[MODE_CTRL] Full scan complete, restarting");
                            `endif
                        end
                    end
                end
            end

            default: scan_state <= S_IDLE;
            endcase
        end

        // ================================================================
        // MODE 10: Single-chirp (debug mode)
        // Fire one chirp per trigger pulse, no scanning.
        // Chirp type depends on range_mode:
        //   3 km:  short chirp only
        //   Long-range: long chirp (for testing long-chirp path)
        // ================================================================
        2'b10: begin
            case (scan_state)
            S_IDLE: begin
                if (trigger_pulse) begin
                    timer        <= 18'd0;
                    mc_new_chirp <= ~mc_new_chirp;

                    if (range_mode == `RP_RANGE_MODE_3KM) begin
                        // 3 km: fire short chirp
                        scan_state     <= S_SHORT_CHIRP;
                        use_long_chirp <= 1'b0;
                    end else begin
                        // Long-range: fire long chirp
                        scan_state     <= S_LONG_CHIRP;
                        use_long_chirp <= 1'b1;
                    end
                end
            end

            S_LONG_CHIRP: begin
                if (timer < cfg_long_chirp_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_LONG_LISTEN;
                end
            end

            S_LONG_LISTEN: begin
                if (timer < cfg_long_listen_cycles - 1)
                    timer <= timer + 1;
                else begin
                    // Single long chirp done, return to idle
                    timer      <= 18'd0;
                    scan_state <= S_IDLE;
                end
            end

            S_SHORT_CHIRP: begin
                use_long_chirp <= 1'b0;
                if (timer < cfg_short_chirp_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_SHORT_LISTEN;
                end
            end

            S_SHORT_LISTEN: begin
                if (timer < cfg_short_listen_cycles - 1)
                    timer <= timer + 1;
                else begin
                    // Single short chirp done, return to idle
                    timer      <= 18'd0;
                    scan_state <= S_IDLE;
                end
            end

            default: scan_state <= S_IDLE;
            endcase
        end

        // ================================================================
        // MODE 11: Reserved — idle
        // ================================================================
        2'b11: begin
            scan_state <= S_IDLE;
            timer      <= 18'd0;
        end

        endcase
    end
end

// ============================================================================
// OUTPUT ASSIGNMENTS
// ============================================================================
assign scanning      = (scan_state != S_IDLE);
assign scan_complete = scan_done_pulse;

endmodule
