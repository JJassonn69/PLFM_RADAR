`timescale 1ns / 1ps

// ============================================================================
// tb_doppler_frame_start_gate.v — AUDIT-S3 regression
// ============================================================================
// Verifies the post-fix gating in doppler_processor_optimized's S_IDLE state.
//
// AUDIT-S3 (pre-fix bug): S_IDLE had two independent if-branches — one for
// frame_start_pulse (resets pointers) and one for data_valid (transitions to
// S_ACCUMULATE). A data_valid arriving BEFORE frame_start_pulse would
// transition the FSM with whatever pointers happened to be set, and the BRAM
// write block would write into addr (write_chirp_index*RANGE_BINS+0).
//
// Post-fix: a `frame_armed` register is set on frame_start_pulse and cleared
// on transition to S_ACCUMULATE. Both the FSM transition and the BRAM write
// are gated on `(frame_start_pulse || frame_armed)`. data_valid arriving
// before frame_start_pulse is silently dropped at the input.
//
// Tests:
//   T1  data_valid alone (no frame_start) → state stays S_IDLE, mem_we=0
//   T2  frame_start_pulse alone (no data_valid) → state stays S_IDLE,
//       frame_armed=1, mem_we=0
//   T3  frame_start, then data_valid 5 cycles later (normal MF latency case)
//       → state→S_ACCUMULATE on the data_valid cycle, mem_we=1, mem_waddr_r=0
//   T4  frame_start and data_valid on the SAME cycle
//       → state→S_ACCUMULATE this cycle, mem_we=1, mem_waddr_r=0
//   T5  long burst of data_valid in S_IDLE before any frame_start_pulse
//       → state stays S_IDLE for the entire burst, mem_we never asserts
// ============================================================================

`include "radar_params.vh"

module tb_doppler_frame_start_gate;

localparam CLK_PERIOD = 10.0;  // 100 MHz

// Tiny config for fast simulation. We only exercise the S_IDLE gating; the
// rest of the FSM (S_ACCUMULATE, FFT pipeline) is not driven to completion.
localparam DOPPLER_FFT_SIZE   = 16;   // unchanged
localparam RANGE_BINS         = 8;
localparam CHIRPS_PER_FRAME   = 4;
localparam CHIRPS_PER_SUBFRAME = 2;
localparam DATA_WIDTH          = 16;

reg clk;
reg reset_n;
reg [31:0] range_data;
reg data_valid;
reg new_chirp_frame;

wire [31:0] doppler_output;
wire        doppler_valid;
wire [`RP_DOPPLER_BIN_WIDTH-1:0]   doppler_bin;
wire [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin;
wire [`RP_SUBFRAME_ID_WIDTH-1:0]   sub_frame;
wire        processing_active;
wire        frame_complete;
wire [3:0]  dut_status;

doppler_processor_optimized #(
    .DOPPLER_FFT_SIZE(DOPPLER_FFT_SIZE),
    .RANGE_BINS(RANGE_BINS),
    .CHIRPS_PER_FRAME(CHIRPS_PER_FRAME),
    .CHIRPS_PER_SUBFRAME(CHIRPS_PER_SUBFRAME),
    .DATA_WIDTH(DATA_WIDTH)
) dut (
    .clk(clk),
    .reset_n(reset_n),
    .range_data(range_data),
    .data_valid(data_valid),
    .new_chirp_frame(new_chirp_frame),
    .doppler_output(doppler_output),
    .doppler_valid(doppler_valid),
    .doppler_bin(doppler_bin),
    .range_bin(range_bin),
    .sub_frame(sub_frame),
    .processing_active(processing_active),
    .frame_complete(frame_complete),
    .status(dut_status)
);

// ----------------------------------------------------------------------------
// Hierarchical refs to internal state
// ----------------------------------------------------------------------------
wire [2:0] dut_state    = dut.state;
wire       dut_armed    = dut.frame_armed;
wire       dut_mem_we   = dut.mem_we;
wire [`RP_DOPPLER_MEM_ADDR_W-1:0] dut_mem_waddr = dut.mem_waddr_r;

localparam [2:0] S_IDLE       = 3'b000;
localparam [2:0] S_ACCUMULATE = 3'b001;

// ----------------------------------------------------------------------------
// Clock
// ----------------------------------------------------------------------------
initial clk = 0;
always #(CLK_PERIOD / 2) clk = ~clk;

// ----------------------------------------------------------------------------
// Test bookkeeping
// ----------------------------------------------------------------------------
integer pass_count = 0;
integer fail_count = 0;
integer test_num   = 0;

task check;
    input cond;
    input [255:0] label;
    begin
        test_num = test_num + 1;
        if (cond) begin
            $display("[PASS %0d] %0s", test_num, label);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL %0d] %0s  (state=%0d armed=%0b mem_we=%0b waddr=%0d)",
                     test_num, label, dut_state, dut_armed, dut_mem_we, dut_mem_waddr);
            fail_count = fail_count + 1;
        end
    end
endtask

task do_reset;
    begin
        reset_n         = 0;
        data_valid      = 0;
        new_chirp_frame = 0;
        range_data      = 32'h0;
        repeat (4) @(posedge clk);
        #1 reset_n = 1;
        @(posedge clk); #1;
    end
endtask

// ----------------------------------------------------------------------------
// Stimulus
// ----------------------------------------------------------------------------
initial begin
    $dumpfile("tb_doppler_frame_start_gate.vcd");
    $dumpvars(0, tb_doppler_frame_start_gate);

    $display("\n=== AUDIT-S3 regression: doppler S_IDLE frame_start gating ===\n");

    // ============================================================
    // T1: data_valid alone (no frame_start) → no transition, no write
    // ============================================================
    do_reset;
    range_data = 32'hAAAA_BBBB;
    data_valid = 1;
    @(posedge clk); #1;
    check(dut_state == S_IDLE,        "T1.a: state stays S_IDLE under data_valid w/o frame_start");
    check(dut_armed == 1'b0,          "T1.b: frame_armed remains 0");
    check(dut_mem_we == 1'b0,         "T1.c: mem_we does NOT fire");

    // Hold for 5 more cycles to confirm no late transition / no write
    repeat (5) @(posedge clk);
    #1;
    check(dut_state == S_IDLE,        "T1.d: state still S_IDLE after 5 more cycles");
    check(dut_mem_we == 1'b0,         "T1.e: mem_we still 0 after 5 more cycles");

    data_valid = 0;
    @(posedge clk); #1;

    // ============================================================
    // T2: frame_start_pulse alone, no data_valid → frame_armed=1, no write
    // ============================================================
    do_reset;
    new_chirp_frame = 1;       // edge-detector inside DUT will fire one-shot
    @(posedge clk); #1;
    new_chirp_frame = 0;
    @(posedge clk); #1;        // frame_start_pulse fires this cycle
    check(dut_armed == 1'b1,          "T2.a: frame_armed set after frame_start_pulse");
    check(dut_state == S_IDLE,        "T2.b: state stays S_IDLE without data_valid");
    check(dut_mem_we == 1'b0,         "T2.c: mem_we does NOT fire (no data_valid)");

    // Wait some cycles — armed should persist
    repeat (10) @(posedge clk);
    #1;
    check(dut_armed == 1'b1,          "T2.d: frame_armed persists across idle cycles");
    check(dut_state == S_IDLE,        "T2.e: state still S_IDLE");

    // ============================================================
    // T3: armed (from T2), then data_valid → transition + write to addr 0
    // ============================================================
    range_data = 32'h1234_5678;
    data_valid = 1;
    @(posedge clk); #1;
    check(dut_state == S_ACCUMULATE,  "T3.a: state→S_ACCUMULATE on first data_valid after frame_start");
    check(dut_armed == 1'b0,          "T3.b: frame_armed cleared on transition");
    check(dut_mem_we == 1'b1,         "T3.c: mem_we asserted on transition");
    check(dut_mem_waddr == 0,         "T3.d: first sample writes to addr 0");

    data_valid = 0;
    @(posedge clk); #1;

    // ============================================================
    // T4: same-cycle frame_start_pulse + data_valid → transition + write
    // ============================================================
    do_reset;
    new_chirp_frame = 1;
    @(posedge clk); #1;        // 1st cycle of new_chirp_frame=1
    new_chirp_frame = 0;
    range_data      = 32'hDEAD_BEEF;
    data_valid      = 1;
    // On THIS cycle: new_chirp_frame_d1=1 (latched last cycle), new_chirp_frame=0
    //   → frame_start_pulse = 0 (XOR is 1 only on rising edge)
    // We need to time the data_valid to coincide with the rising-edge cycle.
    // So actually: drive new_chirp_frame=1 simultaneously with data_valid=1
    // for ONE cycle, starting from a state where new_chirp_frame_d1=0.
    @(posedge clk); #1;         // give the FSM a cycle to settle (T4 prep)
    do_reset;                   // reclean
    range_data      = 32'hDEAD_BEEF;
    data_valid      = 1;
    new_chirp_frame = 1;
    @(posedge clk); #1;          // first cycle: new_chirp_frame_d1 still 0
                                 // → frame_start_pulse fires THIS cycle.
                                 // Same cycle as data_valid → transition+write.
    check(dut_state == S_ACCUMULATE,  "T4.a: same-cycle pulse+data → state→S_ACCUMULATE");
    check(dut_mem_we == 1'b1,         "T4.b: same-cycle → mem_we fires");
    check(dut_mem_waddr == 0,         "T4.c: same-cycle → write to addr 0");
    check(dut_armed == 1'b0,          "T4.d: same-cycle → frame_armed disarmed");

    new_chirp_frame = 0;
    data_valid      = 0;
    @(posedge clk); #1;

    // ============================================================
    // T5: long data_valid burst with no frame_start → no transition, no write
    // (regression for the audit's specific concern)
    // ============================================================
    do_reset;
    range_data = 32'hCAFE_F00D;
    data_valid = 1;
    begin : t5_burst
        integer i;
        integer mem_we_count;
        mem_we_count = 0;
        for (i = 0; i < 100; i = i + 1) begin
            @(posedge clk); #1;
            if (dut_mem_we) mem_we_count = mem_we_count + 1;
            if (dut_state != S_IDLE) begin
                $display("[FAIL] T5: state left S_IDLE at cycle %0d (state=%0d)", i, dut_state);
                fail_count = fail_count + 1;
                disable t5_burst;
            end
        end
        check(mem_we_count == 0,      "T5.a: 100 cycles of data_valid w/o frame_start → 0 BRAM writes");
        check(dut_state == S_IDLE,    "T5.b: state still S_IDLE after 100-cycle burst");
        check(dut_armed == 1'b0,      "T5.c: frame_armed never set");
    end
    data_valid = 0;
    @(posedge clk); #1;

    // ============================================================
    // Summary
    // ============================================================
    $display("\n=== AUDIT-S3 frame-start gate regression ===");
    $display("  PASSED: %0d / %0d", pass_count, test_num);
    $display("  FAILED: %0d / %0d", fail_count, test_num);
    if (fail_count == 0)
        $display("  ** ALL TESTS PASSED **");
    else
        $display("  ** SOME TESTS FAILED **");
    $display("");

    #20 $finish;
end

// Timeout safety
initial begin
    #500_000;
    $display("[FAIL] Watchdog timeout — TB hung");
    $finish;
end

endmodule
