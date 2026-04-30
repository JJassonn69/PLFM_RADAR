`timescale 1ns / 1ps

// ============================================================================
// cdc_async_fifo — Cummings-style asynchronous FIFO (SNUG 2002, style #2)
// ============================================================================
// Replaces cdc_adc_to_processing for multi-bit data crossings where the data
// can change by arbitrary amounts between src_valid events (e.g. CIC samples
// at the 400→100 MHz boundary). The Gray-CDC anti-pattern that
// cdc_adc_to_processing exposes — independent data and toggle synchronizer
// chains that can skew under metastability and let the destination capture a
// half-resolved Gray word — does not apply here, because:
//
//   - Data does NOT cross domains; it sits in a dual-clock distRAM
//     (write port: src_clk, read port: dst_clk).
//   - Only the read/write Gray-coded POINTERS cross between domains. Pointer
//     counters genuinely change ±1 per increment, so Gray code's single-bit-
//     flip metastability protection holds by construction.
//
// Reference: Clifford Cummings, "Simulation and Synthesis Techniques for
// Asynchronous FIFO Design", SNUG 2002 (style #2, registered empty/full).
//
// Output semantics — drop-in compatible with cdc_adc_to_processing:
//   - dst_valid pulses HIGH for one dst_clk cycle per FIFO read.
//   - In matched-rate steady state (src_rate ≤ dst_rate) dst_valid is HIGH
//     every dst_clk cycle while data is flowing — same level shape that
//     cdc_adc_to_processing produced when the toggle changed every cycle.
//   - When the FIFO is empty, dst_valid stays LOW.
//
// Overrun semantics:
//   - overrun pulses HIGH for one src_clk cycle whenever src_valid arrives
//     while the FIFO is full. The write is dropped (no stomp on data already
//     in the FIFO). External logic latches/counts as needed (matches the
//     audit-F-1.2 sticky-overrun pattern in ddc_400m.v).
//
// XDC timing constraints:
//   The project XDC files (xc7a50t_ftg256.xdc, xc7a200t_fbg484.xdc) already
//   contain a blanket `set_false_path` between `clk_100m` and `adc_dco_p`
//   (the 100/400 MHz domains). This automatically covers both pointer
//   crossings here (wptr Gray src→dst and rptr Gray dst→src) — no XDC change
//   is needed. The 2-stage synchronizers carry ASYNC_REG="TRUE" so Vivado
//   places them in the same slice for MTBF; placement is unaffected by the
//   blanket false-path. This matches the project convention already used by
//   cdc_adc_to_processing and other CDC primitives in cdc_modules.v.
//
// Resource estimate: distributed-RAM FIFO, depth 16 × width 18 → 8 LUTRAMs
// per instance. Two instances on the CIC→FIR boundary = 16 LUTRAMs (~0.05%
// of XC7A50T LUT budget).
//
// Reset semantics: src and dst sides reset independently (async-reset on
// negedge of each domain's reset_n). The FIFO comes out of reset in the
// EMPTY state from both sides; writes are gated on `~full` so a write
// arriving before the dst side has come out of reset is safely held in the
// FIFO and drained once dst_reset_n deasserts.
// ============================================================================
module cdc_async_fifo #(
    parameter WIDTH = 18,
    parameter DEPTH = 16              // must be a power of 2
)(
    input  wire             src_clk,
    input  wire             dst_clk,
    input  wire             src_reset_n,
    input  wire             dst_reset_n,
    input  wire [WIDTH-1:0] src_data,
    input  wire             src_valid,
    output reg  [WIDTH-1:0] dst_data,
    output reg              dst_valid,
    output reg              overrun
);

    localparam ADDR_W = $clog2(DEPTH);

    // ---------- Storage (dual-clock distRAM; Vivado infers SLICEM LUTRAM) ----------
    // Note: no reset on `mem` — distRAM has no reset semantics, and forcing one
    // would block LUTRAM inference. Reads are gated on `~empty`, so a cell is
    // never read before it has been written; X-propagation is impossible in
    // sim by construction. The `initial` block zeroes cells purely for
    // simulator cleanliness; synthesis honors it as LUTRAM init values.
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < DEPTH; init_i = init_i + 1)
            mem[init_i] = {WIDTH{1'b0}};
    end

    // ---------- Source domain registers ----------
    reg  [ADDR_W:0]   wptr_bin;        // ADDR_W+1 bits: extra MSB enables full detect
    reg  [ADDR_W:0]   wptr_gray;
    reg               full;
    wire [ADDR_W-1:0] waddr           = wptr_bin[ADDR_W-1:0];

    // ---------- Destination domain registers ----------
    reg  [ADDR_W:0]   rptr_bin;
    reg  [ADDR_W:0]   rptr_gray;
    reg               empty;
    wire [ADDR_W-1:0] raddr           = rptr_bin[ADDR_W-1:0];

    // ---------- CDC: Gray pointer crossings (the only domain-crossing signals) ----------
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W:0] wptr_gray_dst [0:1];
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W:0] rptr_gray_src [0:1];

    // ---------- Pointer-next combinational ----------
    wire             do_write       = src_valid & ~full;
    wire             do_read        = ~empty;
    wire [ADDR_W:0]  wptr_bin_next  = wptr_bin + do_write;
    wire [ADDR_W:0]  wptr_gray_next = wptr_bin_next ^ (wptr_bin_next >> 1);
    wire [ADDR_W:0]  rptr_bin_next  = rptr_bin + do_read;
    wire [ADDR_W:0]  rptr_gray_next = rptr_bin_next ^ (rptr_bin_next >> 1);

    // ---------- Cummings full/empty conditions (style #2: registered) ----------
    // full: next-write-Gray equals synchronized-read-Gray with the two MSBs
    // inverted. This is the canonical "Gray pointer match with MSB twist"
    // detection that distinguishes "wrote one full lap and caught up" from
    // "wptr == rptr because both are at 0".
    wire wfull_val  = (wptr_gray_next ==
                       {~rptr_gray_src[1][ADDR_W:ADDR_W-1],
                         rptr_gray_src[1][ADDR_W-2:0]});
    wire rempty_val = (rptr_gray_next == wptr_gray_dst[1]);

    // ============================================================================
    // SOURCE DOMAIN
    // ============================================================================
    always @(posedge src_clk or negedge src_reset_n) begin
        if (!src_reset_n) begin
            wptr_bin  <= {(ADDR_W+1){1'b0}};
            wptr_gray <= {(ADDR_W+1){1'b0}};
            full      <= 1'b0;
            overrun   <= 1'b0;
        end else begin
            if (do_write) mem[waddr] <= src_data;
            wptr_bin  <= wptr_bin_next;
            wptr_gray <= wptr_gray_next;
            full      <= wfull_val;
            overrun   <= src_valid & full;     // 1-cycle pulse on dropped write
        end
    end

    // Synchronize destination read pointer (Gray) into source domain
    always @(posedge src_clk or negedge src_reset_n) begin
        if (!src_reset_n) begin
            rptr_gray_src[0] <= {(ADDR_W+1){1'b0}};
            rptr_gray_src[1] <= {(ADDR_W+1){1'b0}};
        end else begin
            rptr_gray_src[0] <= rptr_gray;
            rptr_gray_src[1] <= rptr_gray_src[0];
        end
    end

    // ============================================================================
    // DESTINATION DOMAIN
    // ============================================================================
    always @(posedge dst_clk or negedge dst_reset_n) begin
        if (!dst_reset_n) begin
            rptr_bin  <= {(ADDR_W+1){1'b0}};
            rptr_gray <= {(ADDR_W+1){1'b0}};
            empty     <= 1'b1;
            dst_data  <= {WIDTH{1'b0}};
            dst_valid <= 1'b0;
        end else begin
            if (do_read) begin
                dst_data <= mem[raddr];        // capture the read data
                rptr_bin  <= rptr_bin_next;
                rptr_gray <= rptr_gray_next;
            end
            empty     <= rempty_val;
            dst_valid <= do_read;              // 1-cycle pulse per read
        end
    end

    // Synchronize source write pointer (Gray) into destination domain
    always @(posedge dst_clk or negedge dst_reset_n) begin
        if (!dst_reset_n) begin
            wptr_gray_dst[0] <= {(ADDR_W+1){1'b0}};
            wptr_gray_dst[1] <= {(ADDR_W+1){1'b0}};
        end else begin
            wptr_gray_dst[0] <= wptr_gray;
            wptr_gray_dst[1] <= wptr_gray_dst[0];
        end
    end

endmodule
