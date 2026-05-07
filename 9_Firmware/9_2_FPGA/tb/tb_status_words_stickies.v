// ============================================================================
// tb_status_words_stickies.v
//
// AUDIT-S10 + PR-AB.b: status_words[5][6:5] CDC packing for the two
// control-fault flags (range-decimator watchdog F-6.4, CIC->FIR CDC overrun
// F-1.2) AND the gpio_dig7 fault-OR semantic. PR-AB.b Step 1 made the F-6.4
// half sticky in the source clock domain so a slow host poll cannot miss
// the event; F-1.2 is already sticky inside ddc_400m.v. PR-AB.b drives
// gpio_dig7 = (watchdog | cic_fir_overrun) so the MCU stuck-high sampler
// (main.cpp:880-1000) can fire attemptErrorRecovery(ERROR_FPGA_DSP_STALL).
// status_words[5][7] is reserved (must stay 0).
//
// Replaces the GROUP B tests in the retired tb_audit_s10_gpio_split.v and
// also restores the GROUP A dig7 fault-OR coverage (the dig5 saturation
// portion of GROUP A lives in the radar_receiver_final TBs). gpio_dig6
// stretcher (chirp_scheduler frame_pulse) coverage is in
// tb_dig6_frame_pulse.v.
//
//   T1  Reset state           -> sync regs 0, status[6:5]=0, dig7=0
//   T2  Watchdog asserted     -> after 2 ft_clk edges, status[5]=1, dig7=1
//   T3  CIC overrun asserted  -> after 2 ft_clk edges, status[6]=1, dig7=1
//   T4a Both asserted         -> status[6:5]=11, dig7=1
//   T4b Both cleared          -> status[6:5]=00, dig7=0
//   T5  status_words[5][7] stays 0 (reserved bit not stomped by sync); dig7=1
//   T6  status_words[5][4:0] (self_test_flags) pass through; dig7=0
// ============================================================================
`timescale 1ns/1ps

module tb_status_words_stickies;

    reg        clk_src   = 1'b0;   // 100 MHz radar domain
    reg        ft_clk    = 1'b0;   // 60 MHz USB domain
    reg        reset_n   = 1'b0;
    reg        src_watchdog;
    reg        src_overrun;
    reg [4:0]  src_self_test_flags;
    reg        status_req_pulse;

    wire [31:0] status_word_5;
    wire        gpio_dig7;          // PR-AB.b: production OR of fault flags

    status_packing_block status_dut (
        .clk                     (clk_src),
        .ft_clk                  (ft_clk),
        .reset_n                 (reset_n),
        .status_range_decim_watchdog (src_watchdog),
        .status_ddc_cic_fir_overrun  (src_overrun),
        .status_self_test_flags  (src_self_test_flags),
        .status_req_pulse_ft     (status_req_pulse),
        .status_word_5           (status_word_5)
    );

    // Mirrors production combinational OR in radar_system_top.v:
    //   assign gpio_dig7 = rx_range_decim_watchdog | rx_ddc_cic_fir_overrun;
    gpio_dig7_or_block dig7_dut (
        .watchdog (src_watchdog),
        .overrun  (src_overrun),
        .gpio_dig7(gpio_dig7)
    );

    always #5  clk_src = ~clk_src;
    always #8  ft_clk  = ~ft_clk;

    integer pass = 0;
    integer fail = 0;

    task check_status (input [127:0] label, input [31:0] mask, input [31:0] expected);
        begin
            if ((status_word_5 & mask) === (expected & mask)) begin
                $display("  [PASS] %0s: word5=%h (masked %h)",
                         label, status_word_5, status_word_5 & mask);
                pass = pass + 1;
            end else begin
                $display("  [FAIL] %0s: word5=%h masked %h (exp %h)",
                         label, status_word_5, status_word_5 & mask, expected & mask);
                fail = fail + 1;
            end
        end
    endtask

    task check_dig7 (input [127:0] label, input expected);
        begin
            if (gpio_dig7 === expected) begin
                $display("  [PASS] %0s: dig7=%b", label, gpio_dig7);
                pass = pass + 1;
            end else begin
                $display("  [FAIL] %0s: dig7=%b (exp %b)", label, gpio_dig7, expected);
                fail = fail + 1;
            end
        end
    endtask

    task pulse_status_req;
        begin
            @(posedge ft_clk); #1;
            status_req_pulse = 1'b1;
            @(posedge ft_clk); #1;
            status_req_pulse = 1'b0;
            @(posedge ft_clk); #1;
        end
    endtask

    initial begin
        $display("============================================================");
        $display("status_words[5][6:5] CDC + gpio_dig7 fault-OR (AUDIT-S10 + PR-AB.b)");
        $display("============================================================");

        src_watchdog        = 1'b0;
        src_overrun         = 1'b0;
        src_self_test_flags = 5'b00000;
        status_req_pulse    = 1'b0;

        reset_n = 1'b0;
        repeat (5) @(posedge ft_clk);
        reset_n = 1'b1;
        repeat (3) @(posedge ft_clk);

        // T1 reset state
        pulse_status_req();
        check_status("T1 reset state",
                     32'h000000E0,    // [7:5]
                     32'h00000000);
        check_dig7("T1 dig7 idle low", 1'b0);

        // T2 watchdog asserted only
        @(posedge clk_src); #1;
        src_watchdog = 1'b1;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T2 watchdog asserted",
                     32'h00000060,    // [6:5]
                     32'h00000020);   // [5]=1
        check_dig7("T2 dig7 watchdog -> high", 1'b1);

        // T3 cic_fir_overrun asserted only (clear watchdog first)
        @(posedge clk_src); #1;
        src_watchdog = 1'b0;
        src_overrun  = 1'b1;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T3 cic_fir_overrun asserted",
                     32'h00000060,
                     32'h00000040);   // [6]=1
        check_dig7("T3 dig7 overrun -> high", 1'b1);

        // T4 both, then both cleared
        @(posedge clk_src); #1;
        src_watchdog = 1'b1;
        src_overrun  = 1'b1;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T4a both asserted",
                     32'h00000060,
                     32'h00000060);   // [6:5]=11
        check_dig7("T4a dig7 both -> high", 1'b1);

        @(posedge clk_src); #1;
        src_watchdog = 1'b0;
        src_overrun  = 1'b0;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T4b both cleared",
                     32'h00000060,
                     32'h00000000);
        check_dig7("T4b dig7 both cleared -> low", 1'b0);

        // T5 reserved bit [7] stays 0 even when neighbours are 1
        @(posedge clk_src); #1;
        src_watchdog = 1'b1;
        src_overrun  = 1'b1;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T5 [7] reserved stays 0",
                     32'h00000080,
                     32'h00000000);
        check_dig7("T5 dig7 both asserted -> high", 1'b1);

        // T6 self_test_flags pass through unchanged
        @(posedge clk_src); #1;
        src_watchdog        = 1'b0;
        src_overrun         = 1'b0;
        src_self_test_flags = 5'b10110;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T6 self_test_flags untouched",
                     32'h0000001F,
                     32'h00000016);
        check_dig7("T6 dig7 cleared -> low", 1'b0);

        $display("============================================================");
        $display("STATUS STICKIES + DIG7 OR RESULTS: pass=%0d fail=%0d", pass, fail);
        $display("============================================================");
        if (fail == 0) $display("[OVERALL] PASS");
        else           $display("[OVERALL] FAIL");
        $finish;
    end

    initial begin
        #1_000_000;
        $display("[FATAL] timeout");
        $finish;
    end

endmodule

// ============================================================================
// status_packing_block — mirrors the production CDC fragment from
// usb_data_interface.v (and usb_data_interface_ft2232h.v) for the AUDIT-S10
// telemetry path. Source-domain inputs cross to ft_clk via 2-FF level sync,
// then pack into status_words[5][6:5]. Self-test flags pass through into
// status_words[5][4:0] for a sanity check that the packing keeps the
// neighbouring fields untouched. Bit [7] is intentionally reserved.
// ============================================================================
module status_packing_block (
    input  wire        clk,        // 100 MHz radar domain (unused but mirrors prod port list)
    input  wire        ft_clk,
    input  wire        reset_n,
    input  wire        status_range_decim_watchdog,
    input  wire        status_ddc_cic_fir_overrun,
    input  wire [4:0]  status_self_test_flags,
    input  wire        status_req_pulse_ft,
    output reg  [31:0] status_word_5
);
    (* ASYNC_REG = "TRUE" *) reg range_decim_watchdog_sync_0;
    reg                          range_decim_watchdog_sync_1;
    (* ASYNC_REG = "TRUE" *) reg ddc_cic_fir_overrun_sync_0;
    reg                          ddc_cic_fir_overrun_sync_1;

    always @(posedge ft_clk or negedge reset_n) begin
        if (!reset_n) begin
            range_decim_watchdog_sync_0 <= 1'b0;
            range_decim_watchdog_sync_1 <= 1'b0;
            ddc_cic_fir_overrun_sync_0  <= 1'b0;
            ddc_cic_fir_overrun_sync_1  <= 1'b0;
            status_word_5               <= 32'd0;
        end else begin
            range_decim_watchdog_sync_0 <= status_range_decim_watchdog;
            range_decim_watchdog_sync_1 <= range_decim_watchdog_sync_0;
            ddc_cic_fir_overrun_sync_0  <= status_ddc_cic_fir_overrun;
            ddc_cic_fir_overrun_sync_1  <= ddc_cic_fir_overrun_sync_0;

            if (status_req_pulse_ft) begin
                status_word_5 <= {7'd0, 1'b0,                  // [31:24] busy slot tied 0 in TB
                                  8'd0,                        // [23:16] reserved
                                  8'd0,                        // [15:8]  detail tied 0 in TB
                                  1'd0,                        // [7]     reserved
                                  ddc_cic_fir_overrun_sync_1,  // [6]
                                  range_decim_watchdog_sync_1, // [5]
                                  status_self_test_flags};     // [4:0]
            end
        end
    end
endmodule

// ============================================================================
// gpio_dig7_or_block — mirrors the production combinational OR in
// radar_system_top.v:
//
//   assign gpio_dig7 = rx_range_decim_watchdog | rx_ddc_cic_fir_overrun;
//
// MCU's PD15 stuck-high sampler at main.cpp:880-1000 fires
// attemptErrorRecovery(ERROR_FPGA_DSP_STALL) → bitstream reload on either
// fault class.
// ============================================================================
module gpio_dig7_or_block (
    input  wire watchdog,
    input  wire overrun,
    output wire gpio_dig7
);
    assign gpio_dig7 = watchdog | overrun;
endmodule
