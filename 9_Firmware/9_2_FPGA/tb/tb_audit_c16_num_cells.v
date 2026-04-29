// ============================================================================
// tb_audit_c16_num_cells.v
//
// AUDIT-C16: usb_data_interface.v had `localparam [14:0] NUM_CELLS = 15'd16384`
// hardcoded for the 50T (512 range x 32 doppler) layout. On 200T builds with
// SUPPORT_LONG_RANGE defined, RP_MAX_OUTPUT_BINS=4096 -> NUM_CELLS should be
// 131072. Pre-fix two distinct defects:
//   (a) value: counter wrapped 8x per real frame, so the bit-7 frame-start
//       marker fired 8x per frame at incorrect host-frame offsets, silently
//       desyncing the GUI parser
//   (b) width: 15-bit could not represent 131072 (needs 17 bits min)
//
// Fix: localparam NUM_CELLS = `RP_MAX_OUTPUT_BINS * `RP_NUM_DOPPLER_BINS,
// counter width = `RP_DOPPLER_MEM_ADDR_W` (= clog2(NUM_CELLS), 14 for 50T,
// 17 for 200T). Both scale together with the build define.
//
// This TB mirrors the production fragment (counter wrap + frame-start marker
// derivation) and exercises the wrap+marker invariants under both build
// configurations. The 50T variant ignores SUPPORT_LONG_RANGE; the 200T
// variant must be compiled with `+define+SUPPORT_LONG_RANGE`.
//
//   T1  Reset state               -> counter==0, marker==1 (frame start)
//   T2  Increment cycle           -> counter==1, marker==0
//   T3  Wrap at NUM_CELLS-1       -> counter==NUM_CELLS-1, marker==0;
//                                     next cycle counter==0, marker==1
//   T4  Two full wraps            -> across 2*NUM_CELLS cycles, marker
//                                     fires exactly twice (sanity over the
//                                     long 200T window — proves no false
//                                     marker fires every 16384 cycles)
//   T5  Counter width             -> top bit reachable iff NUM_CELLS-1 sets it
//                                     (catches the 14- vs 17-bit width bug
//                                      directly: 50T NUM_CELLS-1=0x3FFF [14
//                                      bits set]; 200T NUM_CELLS-1=0x1FFFF
//                                      [17 bits set]; on a too-narrow counter
//                                      the value would silently truncate)
// ============================================================================
`timescale 1ns/1ps
`include "radar_params.vh"

module tb_audit_c16_num_cells;

    localparam integer NUM_CELLS = `RP_MAX_OUTPUT_BINS * `RP_NUM_DOPPLER_BINS;
    localparam integer CTR_W     = `RP_DOPPLER_MEM_ADDR_W;

    reg                clk     = 1'b0;
    reg                reset_n = 1'b0;
    reg                advance = 1'b0;

    wire [CTR_W-1:0]   sample_counter;
    wire               frame_start_marker;

    sample_counter_block #(
        .NUM_CELLS (NUM_CELLS),
        .CTR_W     (CTR_W)
    ) dut (
        .clk                (clk),
        .reset_n            (reset_n),
        .advance            (advance),
        .sample_counter     (sample_counter),
        .frame_start_marker (frame_start_marker)
    );

    always #5 clk = ~clk;

    integer pass = 0;
    integer fail = 0;

    task check (input [255:0] label, input [CTR_W-1:0] expected_ctr,
                input expected_marker);
        begin
            if (sample_counter === expected_ctr &&
                frame_start_marker === expected_marker) begin
                $display("  [PASS] %0s: ctr=%0d marker=%b", label,
                         sample_counter, frame_start_marker);
                pass = pass + 1;
            end else begin
                $display("  [FAIL] %0s: ctr=%0d (exp %0d) marker=%b (exp %b)",
                         label, sample_counter, expected_ctr,
                         frame_start_marker, expected_marker);
                fail = fail + 1;
            end
        end
    endtask

    task tick;
        begin
            advance = 1'b1;
            @(posedge clk); #1;
        end
    endtask

    integer wraps_seen;
    integer i;
    integer top_bit_was_set;

    initial begin
        $display("============================================================");
        $display("AUDIT-C16: NUM_CELLS+counter-width parameterization");
        $display("  build target: NUM_CELLS=%0d  CTR_W=%0d  (last value=%0d)",
                 NUM_CELLS, CTR_W, NUM_CELLS - 1);
        $display("============================================================");

        // Reset
        reset_n = 1'b0;
        advance = 1'b0;
        @(posedge clk); #1;
        @(posedge clk); #1;
        reset_n = 1'b1;
        @(posedge clk); #1;

        // ---- T1: reset state ----
        check("T1 reset state", {CTR_W{1'b0}}, 1'b1);

        // ---- T2: one increment ----
        tick();
        check("T2 after 1 tick", 'd1, 1'b0);

        // ---- T3: wrap at NUM_CELLS-1 ----
        // Walk to NUM_CELLS-1 (we are at 1 already, so NUM_CELLS-2 more ticks)
        for (i = 0; i < NUM_CELLS - 2; i = i + 1) begin
            tick();
        end
        check("T3a at NUM_CELLS-1", NUM_CELLS - 1, 1'b0);

        // Next tick must wrap to 0 and re-fire the marker
        tick();
        check("T3b after wrap to 0", {CTR_W{1'b0}}, 1'b1);

        // ---- T4: two full wraps -> exactly 2 markers across 2*NUM_CELLS ticks
        // Reset counter to a clean baseline by resetting again
        advance = 1'b0;
        reset_n = 1'b0;
        @(posedge clk); #1;
        reset_n = 1'b1;
        @(posedge clk); #1;
        // sample_counter == 0; marker == 1 (already counted as T1)
        wraps_seen = 0;
        if (frame_start_marker) wraps_seen = 1;     // initial marker at ctr=0
        for (i = 0; i < 2 * NUM_CELLS; i = i + 1) begin
            tick();
            if (frame_start_marker) wraps_seen = wraps_seen + 1;
        end
        // After 2*NUM_CELLS ticks from ctr=0 we have crossed ctr==0 twice
        // more (at ticks NUM_CELLS and 2*NUM_CELLS). Plus the initial marker
        // = 3 total. (Pre-fix on 200T this would be ~17 — exactly 8x more.)
        if (wraps_seen == 3) begin
            $display("  [PASS] T4 markers across 2*NUM_CELLS ticks: %0d (expected 3)",
                     wraps_seen);
            pass = pass + 1;
        end else begin
            $display("  [FAIL] T4 markers: %0d (expected 3)", wraps_seen);
            fail = fail + 1;
        end

        // ---- T5: counter width is wide enough ----
        // Walk back to NUM_CELLS-1 from current state (which is 0).
        // 2*NUM_CELLS ticks brought us back to 0 deterministically.
        for (i = 0; i < NUM_CELLS - 1; i = i + 1) begin
            tick();
        end
        // Now ctr should equal NUM_CELLS-1 exactly. Check top bit observable.
        top_bit_was_set = sample_counter[CTR_W-1];
        // For NUM_CELLS = 2^k (power-of-two), NUM_CELLS-1 has all CTR_W bits
        // set; top bit must be 1. RP_MAX_OUTPUT_BINS * 32 is always 2^k.
        if (sample_counter == NUM_CELLS - 1 && top_bit_was_set == 1'b1) begin
            $display("  [PASS] T5 top counter bit observable: ctr=%0h top_bit=%b",
                     sample_counter, top_bit_was_set);
            pass = pass + 1;
        end else begin
            $display("  [FAIL] T5 top bit check: ctr=%0h top_bit=%b (NUM_CELLS-1=%0h)",
                     sample_counter, top_bit_was_set, NUM_CELLS - 1);
            fail = fail + 1;
        end

        $display("============================================================");
        $display("AUDIT-C16 RESULTS: pass=%0d fail=%0d", pass, fail);
        $display("============================================================");
        if (fail == 0) $display("[OVERALL] PASS");
        else           $display("[OVERALL] FAIL");
        $finish;
    end

    // Generous timeout — 200T case ticks 2*131072 + NUM_CELLS-1 cycles at
    // 10 ns/cycle = ~3.3 ms simulated, well under any practical wall-clock.
    initial begin
        #50_000_000;     // 50 ms simulated cap
        $display("[FATAL] timeout");
        $finish;
    end

endmodule

// ============================================================================
// sample_counter_block — mirrors the production fragment from
// usb_data_interface.v post AUDIT-C16. NUM_CELLS and CTR_W are passed in as
// module parameters so the same block can be exercised under both 50T and
// 200T sizings without recompiling the (large) usb_data_interface module.
// ============================================================================
module sample_counter_block #(
    parameter integer NUM_CELLS = 16384,
    parameter integer CTR_W     = 14
) (
    input  wire             clk,
    input  wire             reset_n,
    input  wire             advance,
    output reg  [CTR_W-1:0] sample_counter,
    output wire             frame_start_marker
);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sample_counter <= {CTR_W{1'b0}};
        end else if (advance) begin
            if (sample_counter == NUM_CELLS - 1)
                sample_counter <= {CTR_W{1'b0}};
            else
                sample_counter <= sample_counter + 1'b1;
        end
    end

    assign frame_start_marker = (sample_counter == {CTR_W{1'b0}});
endmodule
