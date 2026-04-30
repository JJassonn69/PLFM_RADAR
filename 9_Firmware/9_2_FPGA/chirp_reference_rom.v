`timescale 1ns / 1ps

`include "radar_params.vh"

// ============================================================================
// chirp_reference_rom.v — 3-waveform matched-filter reference ROM (RX side)
// ============================================================================
// Replaces the chirp-v1 chirp_memory_loader_param.v (1-bit `use_long_chirp`,
// 6 .mem files, separate BRAMs for long/short).
//
// Carries one of {SHORT, MEDIUM, LONG} via wave_sel[1:0] — see RP_WAVE_*
// in radar_params.vh. The .mem files (PR-B) are uniformly 2048 entries each
// in Q15 I/Q hex; LONG occupies two 2048 segments; SHORT and MEDIUM each
// occupy a single 2048 segment with internal zero-pad past the chirp end.
//
// BRAM layout (single 8192x16 array per lane — Vivado infers 4 RAMB18/lane,
// 8 RAMB18 total. Same cost as chirp-v1 dual-array layout because LONG
// already needed 4 RAMB18; folding SHORT and MEDIUM into the same address
// space costs the 4 BRAMs we'd add for medium anyway):
//
//      addr[12:11]  region                 source files
//      ---------    ---------------------  --------------------------------
//      2'b00        SHORT  ([0..2047])     rx_short_{i,q}.mem
//      2'b01        MEDIUM ([0..2047])     rx_medium_{i,q}.mem
//      2'b10        LONG seg0 ([0..2047])  rx_long_seg0_{i,q}.mem
//      2'b11        LONG seg1 ([0..2047])  rx_long_seg1_{i,q}.mem
//
// Read addressing:
//      case (wave_sel)
//        RP_WAVE_SHORT:   full_addr = {2'b00, sample_addr}
//        RP_WAVE_MEDIUM:  full_addr = {2'b01, sample_addr}
//        RP_WAVE_LONG:    full_addr = {1'b1, segment_select[0], sample_addr}
//        default:         (RP_WAVE_RESERVED) zero-output, mem_ready still pulses
//      endcase
//
// Output semantics — drop-in compatible with chirp_memory_loader_param:
//   - Synchronous read: ref_i / ref_q valid 1 clk after mem_request.
//   - mem_ready pulses with ref data (1 clk after mem_request).
//   - SAME 1-cycle latency as the legacy module (preserves RX-B autocorrelation
//     peak alignment validated by tb_rxb_fullchain_latency).
//
// REQP-1839/1840 compliance (BRAM output registers cannot have async resets):
//   - The BRAM read block uses a SYNCHRONOUS reset, which Vivado maps to the
//     RAMB18 RSTREGB port. mem_ready (a non-BRAM control register) keeps the
//     async reset for clean post-reset behavior. Same split as the legacy
//     chirp_memory_loader_param.v.
// ============================================================================
module chirp_reference_rom (
    input  wire             clk,
    input  wire             reset_n,
    input  wire [1:0]       wave_sel,        // RP_WAVE_{SHORT,MEDIUM,LONG}
    input  wire [1:0]       segment_select,  // [0]=LONG seg index; ignored for SHORT/MEDIUM
    input  wire             mem_request,
    input  wire [10:0]      sample_addr,     // 0..2047 within the active waveform/segment
    output reg  [15:0]      ref_i,
    output reg  [15:0]      ref_q,
    output reg              mem_ready
);

    // -----------------------------------------------------------------------
    // BRAM arrays (one per Q15 lane). Vivado infers RAMB18 with sync read.
    // -----------------------------------------------------------------------
    (* ram_style = "block" *) reg [15:0] mem_i [0:8191];
    (* ram_style = "block" *) reg [15:0] mem_q [0:8191];

    // -----------------------------------------------------------------------
    // Initialization — load 4 distinct .mem files into 4 contiguous regions
    // of the unified BRAM. $readmemh range form lets us target each 2048-cell
    // segment independently. Vivado honors these for RAMB18 init contents.
    // -----------------------------------------------------------------------
    initial begin
        $readmemh("rx_short_i.mem",      mem_i, 0,    2047);
        $readmemh("rx_short_q.mem",      mem_q, 0,    2047);
        $readmemh("rx_medium_i.mem",     mem_i, 2048, 4095);
        $readmemh("rx_medium_q.mem",     mem_q, 2048, 4095);
        $readmemh("rx_long_seg0_i.mem",  mem_i, 4096, 6143);
        $readmemh("rx_long_seg0_q.mem",  mem_q, 4096, 6143);
        $readmemh("rx_long_seg1_i.mem",  mem_i, 6144, 8191);
        $readmemh("rx_long_seg1_q.mem",  mem_q, 6144, 8191);
    end

    // -----------------------------------------------------------------------
    // Address mux — combinational. Encodes the region select into addr[12:11]
    // and passes sample_addr through addr[10:0].
    // -----------------------------------------------------------------------
    reg [12:0] full_addr;
    always @(*) begin
        case (wave_sel)
            `RP_WAVE_SHORT:  full_addr = {2'b00, sample_addr};
            `RP_WAVE_MEDIUM: full_addr = {2'b01, sample_addr};
            `RP_WAVE_LONG:   full_addr = {1'b1, segment_select[0], sample_addr};
            default:         full_addr = 13'd0;  // RP_WAVE_RESERVED — read-zero region
        endcase
    end

    // -----------------------------------------------------------------------
    // BRAM read block — sync-only, sync reset (REQP-1839/1840). Single stage:
    // ref_i / ref_q valid 1 clk after mem_request, matching legacy timing.
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (!reset_n) begin
            ref_i <= 16'd0;
            ref_q <= 16'd0;
        end else if (mem_request) begin
            ref_i <= mem_i[full_addr];
            ref_q <= mem_q[full_addr];
        end
    end

    // -----------------------------------------------------------------------
    // Control register — async-resettable. mem_ready follows mem_request by
    // 1 clk to match the BRAM read latency.
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            mem_ready <= 1'b0;
        else
            mem_ready <= mem_request;
    end

endmodule
