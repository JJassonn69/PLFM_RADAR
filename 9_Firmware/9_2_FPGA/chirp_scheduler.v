`timescale 1ns / 1ps

`include "radar_params.vh"

/**
 * chirp_scheduler.v
 *
 * Single source of truth for waveform identity and inter-chirp timing on the
 * RX side. Drives `wave_sel[1:0]` and `chirp_pulse` natively; downstream
 * modules (chirp_reference_rom, matched_filter_multi_segment, mti_canceller)
 * consume those without 1-bit shims.
 *
 * Operation: FPGA-paced auto-scan over the enabled sub-frames (SHORT, MEDIUM,
 * LONG). `host_subframe_enable[2:0]` masks individual waveforms out without
 * recompiling. Each sub-frame fires `host_chirps_per_subframe` chirps at the
 * per-waveform chirp/listen-cycle setpoints.
 *
 * The legacy multi-mode field (STM32 pass-through / single-chirp debug /
 * track dwell) was retired in PR-AB.b expanded (2026-05-11). All three
 * dead branches plus their host_* inputs (host_mode, host_trigger,
 * host_debug_wave_sel, host_track_*) and the track watchdog were stripped
 * — see project_aeris10_mode_strip_2026-05-11.md for rationale.
 *
 * Pulse outputs (chirp_pulse, subframe_pulse, frame_pulse) are 1-cycle
 * positive pulses, not toggles.
 *
 * PR-AB.b expanded commit 5 — beam-ready handshake: when
 * host_handshake_enable=1, the FSM enters S_BEAM_WAIT after frame_pulse
 * and only fires the next frame's first chirp once it observes an edge on
 * beam_ready_async (MCU PD8 toggle) or the ~80 ms watchdog expires.
 * Watchdog timeout sets the sticky output beam_handshake_watchdog_fired
 * (cleared only by reset_n) so the host can spot patterns falling behind
 * the chirp ladder. host_handshake_enable=0 preserves the legacy
 * always-on chirp cadence.
 *
 * Clock domain: clk (100 MHz), async-low reset.
 */

module chirp_scheduler (
    input  wire clk,
    input  wire reset_n,

    // 3-bit sub-frame enable mask (LONG|MEDIUM|SHORT)
    input  wire [2:0] host_subframe_enable,

    // 3-ladder timing (100 MHz cycles). host_*_listen sums with host_guard
    // to define the inter-chirp PRI. Each waveform has independent chirp/
    // listen so SHORT can run faster while LONG covers full eclipse.
    input  wire [15:0] host_short_chirp_cycles,
    input  wire [15:0] host_short_listen_cycles,
    input  wire [15:0] host_medium_chirp_cycles,
    input  wire [15:0] host_medium_listen_cycles,
    input  wire [15:0] host_long_chirp_cycles,
    input  wire [15:0] host_long_listen_cycles,
    input  wire [15:0] host_guard_cycles,

    // Frame structure (chirps per Doppler sub-frame, default 16)
    input  wire [5:0]  host_chirps_per_subframe,

    // Master enable (PR-E). When low, the scheduler holds in S_IDLE and
    // emits no chirp_pulse — the FSM resumes on the next clock edge after
    // mixers_enable returns high. Keeps the radar quiet between operator
    // commands and prevents stale chirp_pulses from being buffered by the
    // TX-side cdc_async_fifo before mixers come up.
    input  wire mixers_enable,

    // PR-AB.b expanded commit 5: beam-ready handshake. beam_ready_async is the
    // raw MCU PD8 GPIO toggle (CDC-synchronized inside this module on `clk`).
    // host_handshake_enable gates whether the FSM stalls in S_BEAM_WAIT after
    // each frame_pulse. Cold-reset default at the top level is 1'b0 (legacy
    // open-loop cadence) — host enables via opcode 0x1A once the MCU's PD8
    // toggle wiring is in place.
    input  wire beam_ready_async,
    input  wire host_handshake_enable,

    // ====== Outputs ======
    output reg  [1:0]  wave_sel,         // canonical waveform identity
    output reg         chirp_pulse,      // 1-cycle pulse: chirp begins this clk
    output reg         subframe_pulse,   // 1-cycle pulse: sub-frame complete
    output reg         frame_pulse,      // 1-cycle pulse: frame complete
    output reg  [5:0]  chirp_counter,    // chirp index inside current frame
    output reg  [1:0]  subframe_id,      // 0=SHORT, 1=MEDIUM, 2=LONG

    // Currently selected timing for the in-flight chirp (PR-E TX async FIFO)
    output wire [15:0] cfg_chirp_cycles,
    output wire [15:0] cfg_listen_cycles,
    output wire [15:0] cfg_guard_cycles,

    // PR-AB.b expanded commit 5: sticky handshake watchdog flag, cleared
    // only by reset_n. Plumbed into status_words[4][1] at the top level.
    output reg         beam_handshake_watchdog_fired
);

// ============================================================================
// Sub-frame helpers — pure functions of (subframe, mask)
// ============================================================================
function [1:0] first_enabled_subframe;
    input [2:0] mask;
    begin
        if      (mask[0]) first_enabled_subframe = 2'd0;  // SHORT
        else if (mask[1]) first_enabled_subframe = 2'd1;  // MEDIUM
        else if (mask[2]) first_enabled_subframe = 2'd2;  // LONG
        else              first_enabled_subframe = 2'd0;  // mask=000 fallback
    end
endfunction

function [1:0] next_enabled_subframe;
    input [1:0] cur;
    input [2:0] mask;
    reg   [1:0] try0, try1, try2;
    begin
        // Walk forward from cur+1, wrapping at 3, find first enabled bit.
        try0 = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
        try1 = (try0 == 2'd2) ? 2'd0 : (try0 + 2'd1);
        try2 = (try1 == 2'd2) ? 2'd0 : (try1 + 2'd1);
        if      (mask[try0]) next_enabled_subframe = try0;
        else if (mask[try1]) next_enabled_subframe = try1;
        else if (mask[try2]) next_enabled_subframe = try2;
        else                 next_enabled_subframe = cur;  // mask=000 fallback
    end
endfunction

function [1:0] subframe_to_wave;
    input [1:0] sf;
    begin
        case (sf)
            2'd0:    subframe_to_wave = `RP_WAVE_SHORT;
            2'd1:    subframe_to_wave = `RP_WAVE_MEDIUM;
            2'd2:    subframe_to_wave = `RP_WAVE_LONG;
            default: subframe_to_wave = `RP_WAVE_SHORT;
        endcase
    end
endfunction

// ============================================================================
// Output mux for selected timing — wave_sel drives chirp/listen window length.
// guard is shared across waveforms.
// ============================================================================
reg [15:0] sel_chirp_cycles;
reg [15:0] sel_listen_cycles;
always @(*) begin
    case (wave_sel)
        `RP_WAVE_SHORT:  begin
            sel_chirp_cycles  = host_short_chirp_cycles;
            sel_listen_cycles = host_short_listen_cycles;
        end
        `RP_WAVE_MEDIUM: begin
            sel_chirp_cycles  = host_medium_chirp_cycles;
            sel_listen_cycles = host_medium_listen_cycles;
        end
        `RP_WAVE_LONG:   begin
            sel_chirp_cycles  = host_long_chirp_cycles;
            sel_listen_cycles = host_long_listen_cycles;
        end
        default: begin
            sel_chirp_cycles  = host_short_chirp_cycles;
            sel_listen_cycles = host_short_listen_cycles;
        end
    endcase
end
assign cfg_chirp_cycles  = sel_chirp_cycles;
assign cfg_listen_cycles = sel_listen_cycles;
assign cfg_guard_cycles  = host_guard_cycles;

// ============================================================================
// Beam-ready CDC + edge detection (PR-AB.b expanded commit 5).
// beam_ready_async is a slow MCU GPIO toggle (PD8). Two ASYNC_REG flops bring
// it into clk, then a one-cycle delay lets us detect any transition (rising or
// falling) — the MCU drives via HAL_GPIO_TogglePin once per beam pattern, so
// successive frames see alternating polarities.
// ============================================================================
(* ASYNC_REG = "TRUE" *) reg [1:0] beam_ready_sync;
reg                                beam_ready_q_prev;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        beam_ready_sync   <= 2'b00;
        beam_ready_q_prev <= 1'b0;
    end else begin
        beam_ready_sync   <= {beam_ready_sync[0], beam_ready_async};
        beam_ready_q_prev <= beam_ready_sync[1];
    end
end
wire beam_ready_q    = beam_ready_sync[1];
wire beam_ready_edge = (beam_ready_q != beam_ready_q_prev);

// ============================================================================
// Main FSM — auto-scan over enabled sub-frames.
// ============================================================================
localparam S_IDLE      = 3'd0;
localparam S_CHIRP     = 3'd1;
localparam S_LISTEN    = 3'd2;
localparam S_GUARD     = 3'd3;
localparam S_ADVANCE   = 3'd4;
localparam S_BEAM_WAIT = 3'd5;

// Beam-ready watchdog: 23 bits at 100 MHz → ~83.9 ms = ~8 nominal frames
// (frame ≈ 8.05 ms full 3-PRI ladder, less when subframes are masked). Long
// enough to absorb MCU SPI bursts + scheduling jitter without auto-advancing,
// short enough to keep the radar moving when a pattern write actually drops.
localparam [22:0] BEAM_WATCHDOG_MAX = 23'd8_000_000;

reg [2:0]  state;
reg [16:0] timer;             // 17 bits cover LONG+listen+guard worst case
reg [22:0] beam_watchdog;     // counts clk cycles while in S_BEAM_WAIT

// Pre-computed wires used inside the FSM advance logic so non-blocking
// updates to subframe_id / wave_sel see the correct next value in the same
// clock edge as the bookkeeping update.
wire [1:0] first_sf = first_enabled_subframe(host_subframe_enable);
wire [1:0] next_sf  = next_enabled_subframe(subframe_id, host_subframe_enable);

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state          <= S_IDLE;
        timer          <= 17'd0;
        wave_sel       <= `RP_WAVE_SHORT;
        chirp_pulse    <= 1'b0;
        subframe_pulse <= 1'b0;
        frame_pulse    <= 1'b0;
        chirp_counter  <= 6'd0;
        subframe_id    <= 2'd0;
        beam_watchdog  <= 23'd0;
        beam_handshake_watchdog_fired <= 1'b0;
    end else if (!mixers_enable) begin
        // Master disable — quiesce the FSM so chirp_pulse never asserts and
        // the TX side stays at idle. beam_handshake_watchdog_fired is sticky
        // across mixers_enable cycles so the host can see late patterns even
        // after a soft restart.
        state          <= S_IDLE;
        timer          <= 17'd0;
        chirp_pulse    <= 1'b0;
        subframe_pulse <= 1'b0;
        frame_pulse    <= 1'b0;
        beam_watchdog  <= 23'd0;
    end else begin
        // Pulses default low — set high for one cycle on relevant transitions.
        chirp_pulse    <= 1'b0;
        subframe_pulse <= 1'b0;
        frame_pulse    <= 1'b0;

        case (state)
            S_IDLE: begin
                timer         <= 17'd0;
                chirp_counter <= 6'd0;
                subframe_id   <= first_sf;
                wave_sel      <= subframe_to_wave(first_sf);
                chirp_pulse   <= 1'b1;
                state         <= S_CHIRP;
            end
            S_CHIRP: begin
                if (timer + 17'd1 >= {1'b0, sel_chirp_cycles}) begin
                    timer <= 17'd0;
                    state <= S_LISTEN;
                end else timer <= timer + 17'd1;
            end
            S_LISTEN: begin
                if (timer + 17'd1 >= {1'b0, sel_listen_cycles}) begin
                    timer <= 17'd0;
                    state <= S_GUARD;
                end else timer <= timer + 17'd1;
            end
            S_GUARD: begin
                if (timer + 17'd1 >= {1'b0, host_guard_cycles}) begin
                    timer <= 17'd0;
                    state <= S_ADVANCE;
                end else timer <= timer + 17'd1;
            end
            S_ADVANCE: begin
                if (chirp_counter < host_chirps_per_subframe - 6'd1) begin
                    chirp_counter <= chirp_counter + 6'd1;
                    chirp_pulse   <= 1'b1;
                    state         <= S_CHIRP;
                end else begin
                    chirp_counter  <= 6'd0;
                    subframe_pulse <= 1'b1;
                    subframe_id    <= next_sf;
                    wave_sel       <= subframe_to_wave(next_sf);
                    if (next_sf == first_sf) begin
                        // Frame wrap — emit frame_pulse and (if enabled) stall
                        // in S_BEAM_WAIT until the MCU acknowledges via PD8.
                        frame_pulse <= 1'b1;
                        if (host_handshake_enable) begin
                            beam_watchdog <= 23'd0;
                            state         <= S_BEAM_WAIT;
                        end else begin
                            chirp_pulse <= 1'b1;
                            state       <= S_CHIRP;
                        end
                    end else begin
                        chirp_pulse <= 1'b1;
                        state       <= S_CHIRP;
                    end
                end
            end
            S_BEAM_WAIT: begin
                // Wait for an MCU PD8 toggle (any edge) OR the watchdog.
                // host_handshake_enable can drop mid-wait — release the FSM in
                // that case so disabling the handshake never strands the
                // radar between frames.
                if (beam_ready_edge || !host_handshake_enable) begin
                    beam_watchdog <= 23'd0;
                    chirp_pulse   <= 1'b1;
                    state         <= S_CHIRP;
                end else if (beam_watchdog >= BEAM_WATCHDOG_MAX) begin
                    beam_handshake_watchdog_fired <= 1'b1;
                    beam_watchdog <= 23'd0;
                    chirp_pulse   <= 1'b1;
                    state         <= S_CHIRP;
                end else begin
                    beam_watchdog <= beam_watchdog + 23'd1;
                end
            end
            default: state <= S_IDLE;
        endcase
    end
end

endmodule
