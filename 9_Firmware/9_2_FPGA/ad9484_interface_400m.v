module ad9484_interface_400m (
    // ADC Physical Interface (LVDS)
    input wire [7:0] adc_d_p,        // ADC Data P
    input wire [7:0] adc_d_n,        // ADC Data N
    input wire adc_dco_p,            // Data Clock Output P (400MHz)
    input wire adc_dco_n,            // Data Clock Output N (400MHz)
    // Audit F-0.1: AD9484 OR (overrange) LVDS pair (SDR, like data — see
    // AUDIT-C4 note below; an earlier comment incorrectly described this as
    // DDR). Routed on the 50T main board to bank 14 pins M6/N6. Asserts for
    // any sample whose absolute value exceeds full-scale.
    input wire adc_or_p,
    input wire adc_or_n,

    // System Interface
    input wire sys_clk,              // 100MHz system clock (for control only)
    input wire reset_n,

    // Output at 400MHz domain
    output wire [7:0] adc_data_400m, // ADC data at 400MHz
    output wire adc_data_valid_400m, // Valid at 400MHz
    output wire adc_dco_bufg,        // Buffered 400MHz DCO clock for downstream use
    // Audit F-0.1: OR flag, clk_400m domain. High on any sample in the
    // current 400 MHz cycle where the ADC reports overrange.
    output wire adc_overrange_400m,
    // Audit F-7.4: MMCM lock indicator. Surfaces the internal mmcm_locked
    // wire so testbenches (and, in future, the host status pipeline) can
    // gate on a stable jitter-cleaned 400 MHz clock instead of guessing at
    // a fixed wait. Combinational MMCME2 output — synchronize before use
    // outside the clk_400m domain.
    output wire mmcm_locked
);

// LVDS to single-ended conversion
wire [7:0] adc_data;
wire adc_dco;

// IBUFDS for each data bit
// NOTE: IOSTANDARD and DIFF_TERM are set via XDC constraints, not RTL
// parameters, to support multiple FPGA targets with different bank voltages:
//   - XC7A200T (FBG484): Bank 14 VCCO = 2.5V → LVDS_25
//   - XC7A50T  (FTG256): Bank 14 VCCO = 3.3V → LVDS_33
genvar i;
generate
    for (i = 0; i < 8; i = i + 1) begin : data_buffers
        IBUFDS #(
            .DIFF_TERM("FALSE"),    // Overridden by XDC DIFF_TERM property
            .IOSTANDARD("DEFAULT")  // Overridden by XDC IOSTANDARD property
        ) ibufds_data (
            .O(adc_data[i]),
            .I(adc_d_p[i]),
            .IB(adc_d_n[i])
        );
    end
endgenerate

// IBUFDS for DCO
IBUFDS #(
    .DIFF_TERM("FALSE"),    // Overridden by XDC DIFF_TERM property
    .IOSTANDARD("DEFAULT")  // Overridden by XDC IOSTANDARD property
) ibufds_dco (
    .O(adc_dco),
    .I(adc_dco_p),
    .IB(adc_dco_n)
);

// ============================================================================
// Clock buffering strategy for source-synchronous ADC interface:
//
// BUFIO: Near-zero insertion delay, drives IOB primitives only (in this
//        module: the IOB-packed input FF on each data bit and the OR pair).
//        Eliminates the hold violation that a BUFG-clocked capture would
//        suffer from BUFG insertion delay.
//
// BUFG:  Global clock buffer for fabric logic (downstream processing and
//        the BUFIO→BUFG re-register stage). ~4 ns insertion delay, fine
//        for fabric-to-fabric paths.
// ============================================================================
wire adc_dco_bufio;   // Near-zero delay — drives IOB IFFs only
wire adc_dco_buffered; // BUFG output — drives fabric logic

BUFIO bufio_dco (
    .I(adc_dco),
    .O(adc_dco_bufio)
);

// MMCME2 jitter-cleaning wrapper replaces the direct BUFG.
// The PLL feedback loop attenuates input jitter from ~50 ps to ~20-30 ps,
// reducing clock uncertainty and improving WNS on the 400 MHz CIC path.
// Audit F-7.4: mmcm_locked is now exposed as a module output — see the
// port-list comment above.
wire mmcm_locked_int;

adc_clk_mmcm mmcm_inst (
    .clk_in       (adc_dco),          // 400 MHz from IBUFDS output
    .reset_n      (reset_n),
    .clk_400m_out (adc_dco_buffered), // Jitter-cleaned 400 MHz on BUFG
    .mmcm_locked  (mmcm_locked_int)
);
assign adc_dco_bufg = adc_dco_buffered;
assign mmcm_locked  = mmcm_locked_int;

// AUDIT-C4 (2026-05-01): AD9484 outputs SDR LVDS (datasheet p.5: "Output
// (LVDS—SDR)"; p.16: "data outputs are valid on the rising edge of DCO").
// One new sample per DCO period (DCO=fs=400 MHz); data is held stable for
// the full period. The chip has no DDR mode and no SPI access (CSB tied
// high on the production board, see Main_Board.sch:46719) so no register
// can change this.
//
// Capture on the FALLING DCO edge — 1.25 ns inside AD9484's stable data
// window. The rising DCO edge coincides with the AD9484 data transition
// (tSKEW = ±70 ps vs typical IFF setup ~150 ps), which would be
// functionally metastable; the falling edge has ~0.4 ns of setup margin
// against tPD = 0.85 ns. IOB=TRUE forces the FF into the input I/O block,
// giving near-zero clock-to-Q insertion delay matched to BUFIO.
//
// Previous (broken) behaviour: an IDDR captured both edges and a `dco_phase`
// FSM alternated Q1/Q2 in an attempt to demux a "DDR" stream. Because the
// chip is SDR, both edges represent the same sample, and the alternation
// produced approximately [s_{-1}, s_1, s_1, s_3, s_3, …] — odd-sample
// duplication with even-sample loss, equivalent to decimate-by-2 +
// ZOH upsample-by-2. The downstream 120 MHz IF then folded to ~80 MHz
// and corrupted the DDC. See git log for the audit trail.
(* IOB = "TRUE" *) reg [7:0] adc_data_iff;

always @(negedge adc_dco_bufio) begin
    adc_data_iff <= adc_data;
end

// ============================================================================
// Re-register the IFF output into the BUFG domain
// adc_data_iff is stable from one falling DCO edge to the next (full DCO
// period of validity), so the rising-edge BUFG capture has ample margin.
// BUFIO and BUFG are derived from the same source (adc_dco), so they are
// frequency-matched.
// ============================================================================
// Timing on the BUFIO→BUFG CDC edge is governed by a 3.000 ns
// set_max_delay in constraints/adc_clk_mmcm.xdc (1.2× the 2.500 ns period),
// which leaves the placer free and still fits inside the ADC data-valid
// window. The fabric BUFG-clocked re-register intentionally lives outside
// the IOB — packing it back into the IOB column was tried in earlier
// builds and rejected (the BUFG clock can't share the ILOGIC clock mux
// with a BUFIO-domain capture, and a pblock around the IDDR column
// pulled fanout into the I/O region and caused router congestion on 51
// unrelated paths).
reg [7:0] adc_data_iff_bufg;

always @(posedge adc_dco_buffered) begin
    adc_data_iff_bufg <= adc_data_iff;
end

// SDR output: one sample per BUFG cycle (400 MHz BUFG = 400 MSPS)
reg [7:0] adc_data_400m_reg;
reg adc_data_valid_400m_reg;

// ── Reset synchronizer ────────────────────────────────────────
// reset_n comes from the 100 MHz sys_clk domain.  Assertion (going low)
// is asynchronous and safe — the FFs enter reset instantly.  De-assertion
// (going high) must be synchronised to adc_dco_buffered to avoid
// metastability.  This is the classic "async assert, sync de-assert" pattern.
//
// mmcm_locked gates de-assertion: the 400 MHz domain stays in reset until
// the MMCM PLL has locked and the jitter-cleaned clock is stable.
// mmcm_locked is a combinational MMCME2 output and can glitch; sync it
// into the 400 MHz domain with a 2-FF chain before using it in the
// async-reset branch below so a LOCKED blip doesn't asynchronously
// re-reset the domain. The chain is itself async-reset by the raw
// reset_n so it forces reset_n_gated=0 at power-up (no valid adc_dco
// edges exist yet to clock the sync chain).
(* ASYNC_REG = "TRUE" *) reg [1:0] mmcm_locked_sync_400m;
always @(posedge adc_dco_buffered or negedge reset_n) begin
    if (!reset_n)
        mmcm_locked_sync_400m <= 2'b00;
    else
        mmcm_locked_sync_400m <= {mmcm_locked_sync_400m[0], mmcm_locked};
end
wire mmcm_locked_400m = mmcm_locked_sync_400m[1];

(* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync_400m;
wire reset_n_400m;
wire reset_n_gated = reset_n & mmcm_locked_400m;

always @(posedge adc_dco_buffered or negedge reset_n_gated) begin
    if (!reset_n_gated)
        reset_sync_400m <= 2'b00;           // async assert (or MMCM not locked)
    else
        reset_sync_400m <= {reset_sync_400m[0], 1'b1};  // sync de-assert
end
assign reset_n_400m = reset_sync_400m[1];

always @(posedge adc_dco_buffered or negedge reset_n_400m) begin
    if (!reset_n_400m) begin
        adc_data_400m_reg <= 8'b0;
        adc_data_valid_400m_reg <= 1'b0;
    end else begin
        adc_data_400m_reg <= adc_data_iff_bufg;
        adc_data_valid_400m_reg <= 1'b1;
    end
end

assign adc_data_400m = adc_data_400m_reg;
assign adc_data_valid_400m = adc_data_valid_400m_reg;

// ============================================================================
// Audit F-0.1 / AUDIT-C4: AD9484 OR (overrange) capture
// OR is an SDR LVDS pair (per AD9484 datasheet — the chip outputs SDR,
// not DDR; comment in earlier revision was wrong). One assertion per DCO
// period, valid on the rising DCO edge, held stable for the rest of the
// period. Captured at the falling DCO edge in the same IFF style as the
// data path. Downstream stickifies in its own domain.
// ============================================================================
wire adc_or_raw;
IBUFDS #(
    .DIFF_TERM("FALSE"),
    .IOSTANDARD("DEFAULT")
) ibufds_or (
    .O(adc_or_raw),
    .I(adc_or_p),
    .IB(adc_or_n)
);

(* IOB = "TRUE" *) reg adc_or_iff;
always @(negedge adc_dco_bufio) begin
    adc_or_iff <= adc_or_raw;
end

reg adc_or_iff_bufg;
always @(posedge adc_dco_buffered) begin
    adc_or_iff_bufg <= adc_or_iff;
end

reg adc_overrange_r;
always @(posedge adc_dco_buffered or negedge reset_n_400m) begin
    if (!reset_n_400m)
        adc_overrange_r <= 1'b0;
    else
        adc_overrange_r <= adc_or_iff_bufg;
end
assign adc_overrange_400m = adc_overrange_r;

endmodule