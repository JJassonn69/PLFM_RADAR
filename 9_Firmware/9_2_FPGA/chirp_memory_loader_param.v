`timescale 1ns / 1ps
module chirp_memory_loader_param #(
    parameter LONG_I_FILE_SEG0 = "long_chirp_seg0_i.mem",
    parameter LONG_Q_FILE_SEG0 = "long_chirp_seg0_q.mem",
    parameter LONG_I_FILE_SEG1 = "long_chirp_seg1_i.mem",
    parameter LONG_Q_FILE_SEG1 = "long_chirp_seg1_q.mem",
    parameter SHORT_I_FILE = "short_chirp_i.mem",
    parameter SHORT_Q_FILE = "short_chirp_q.mem",
    parameter DEBUG = 1
)(
    input wire clk,
    input wire reset_n,
    input wire [1:0] segment_select,
    input wire mem_request,
    input wire use_long_chirp,
    input wire [10:0] sample_addr,
    output reg [15:0] ref_i,
    output reg [15:0] ref_q,
    output reg mem_ready
);

// Memory declarations — 2 long segments × 2048 = 4096 samples
(* ram_style = "block" *) reg [15:0] long_chirp_i [0:4095];
(* ram_style = "block" *) reg [15:0] long_chirp_q [0:4095];
(* ram_style = "block" *) reg [15:0] short_chirp_i [0:2047];
(* ram_style = "block" *) reg [15:0] short_chirp_q [0:2047];

// Initialize memory
integer i;

initial begin
    `ifdef SIMULATION
    if (DEBUG) begin
        $display("[MEM] Starting memory initialization for 2 long chirp segments");
    end
    `endif

    // === LOAD LONG CHIRP — 2 SEGMENTS ===
    // Segment 0 (addresses 0-2047)
    $readmemh(LONG_I_FILE_SEG0, long_chirp_i, 0, 2047);
    $readmemh(LONG_Q_FILE_SEG0, long_chirp_q, 0, 2047);
    `ifdef SIMULATION
    if (DEBUG) $display("[MEM] Loaded long chirp segment 0 (0-2047)");
    `endif

    // Segment 1 (addresses 2048-4095)
    $readmemh(LONG_I_FILE_SEG1, long_chirp_i, 2048, 4095);
    $readmemh(LONG_Q_FILE_SEG1, long_chirp_q, 2048, 4095);
    `ifdef SIMULATION
    if (DEBUG) $display("[MEM] Loaded long chirp segment 1 (2048-4095)");
    `endif

    // === LOAD SHORT CHIRP ===
    // Load first 50 samples (0-49)
    $readmemh(SHORT_I_FILE, short_chirp_i, 0, 49);
    $readmemh(SHORT_Q_FILE, short_chirp_q, 0, 49);
    `ifdef SIMULATION
    if (DEBUG) $display("[MEM] Loaded short chirp (0-49)");
    `endif

    // Zero pad remaining samples (50-2047)
    for (i = 50; i < 2048; i = i + 1) begin
        short_chirp_i[i] = 16'h0000;
        short_chirp_q[i] = 16'h0000;
    end
    `ifdef SIMULATION
    if (DEBUG) $display("[MEM] Zero-padded short chirp from 50-2047");

    // === VERIFICATION ===
    if (DEBUG) begin
        $display("[MEM] Memory loading complete. Verification samples:");
        $display("  Long[0]:     I=%h Q=%h", long_chirp_i[0], long_chirp_q[0]);
        $display("  Long[2047]:  I=%h Q=%h", long_chirp_i[2047], long_chirp_q[2047]);
        $display("  Long[2048]:  I=%h Q=%h", long_chirp_i[2048], long_chirp_q[2048]);
        $display("  Long[4095]:  I=%h Q=%h", long_chirp_i[4095], long_chirp_q[4095]);
        $display("  Short[0]:    I=%h Q=%h", short_chirp_i[0], short_chirp_q[0]);
        $display("  Short[49]:   I=%h Q=%h", short_chirp_i[49], short_chirp_q[49]);
        $display("  Short[50]:   I=%h Q=%h (zero-padded)", short_chirp_i[50], short_chirp_q[50]);
    end
    `endif
end

// Memory access logic
// long_addr: segment_select[0] selects segment (0 or 1), sample_addr[10:0] selects within
wire [11:0] long_addr = {segment_select[0], sample_addr};

// ---- BRAM read block (sync-only, sync reset) ----
// REQP-1839/1840 fix: BRAM output registers cannot have async resets.
// We use a synchronous reset instead, which Vivado maps to the BRAM
// RSTREGB port (supported by 7-series BRAM primitives).
always @(posedge clk) begin
    if (!reset_n) begin
        ref_i <= 16'd0;
        ref_q <= 16'd0;
    end else if (mem_request) begin
        if (use_long_chirp) begin
            ref_i <= long_chirp_i[long_addr];
            ref_q <= long_chirp_q[long_addr];

            `ifdef SIMULATION
            if (DEBUG && $time < 100) begin
                $display("[MEM @%0t] Long chirp: seg=%b, addr=%d, I=%h, Q=%h",
                        $time, segment_select, long_addr,
                        long_chirp_i[long_addr], long_chirp_q[long_addr]);
            end
            `endif
        end else begin
            // Short chirp (0-2047)
            ref_i <= short_chirp_i[sample_addr];
            ref_q <= short_chirp_q[sample_addr];

            `ifdef SIMULATION
            if (DEBUG && $time < 100) begin
                $display("[MEM @%0t] Short chirp: addr=%d, I=%h, Q=%h",
                        $time, sample_addr, short_chirp_i[sample_addr], short_chirp_q[sample_addr]);
            end
            `endif
        end
    end
end

// ---- Control block (async reset for mem_ready only) ----
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        mem_ready <= 1'b0;
    end else begin
        mem_ready <= mem_request;
    end
end

endmodule
