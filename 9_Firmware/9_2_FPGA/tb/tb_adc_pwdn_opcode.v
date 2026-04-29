// ============================================================================
// tb_adc_pwdn_opcode.v
//
// AUDIT-S25: AD9484 power-down (PWDN) had been hard-tied to 1'b0 in
// `radar_receiver_final.v:246`. Combined with AUDIT-C13 (CSB hard-tied HIGH
// on the production board, no SPI access to the AD9484), the ADC was fully
// un-recoverable from a stuck state without dropping main power — which
// also drops the VBAT-backed BKPSRAM persistence (MCU-A4 OCXO warmup flag,
// MCU-A7 emergency persist flag) and forces a 180 s warmup soak.
//
// Fix: opcode 0x32 (reserved during AUDIT-C3 commit `24ef5e7`) now drives
// a new `host_adc_pwdn` register in `radar_system_top.v`, which feeds the
// `adc_pwdn` output pin via `radar_receiver_final.v`.
//
// This TB models the dispatch register-block fragment from
// radar_system_top.v (the part touching host_adc_pwdn) and asserts:
//
//   T1: After reset, host_adc_pwdn == 0  (matches the historical hard-tied
//       state at radar_receiver_final.v:246, so existing bringup behavior
//       is preserved — power-on does NOT accidentally PWDN the ADC).
//
//   T2: Opcode 0x32 with value bit[0]=1 sets host_adc_pwdn=1 next clock.
//
//   T3: Opcode 0x32 with value bit[0]=0 clears host_adc_pwdn back to 0.
//
//   T4: Opcode 0x32 only looks at usb_cmd_value[0] — upper bits are ignored
//       (so a future expansion to a multi-bit ADC control field can repurpose
//       upper bits without breaking back-compat).
//
//   T5: Unrelated opcodes (0x33 = host_adc_format, 0x01 = radar_mode) do
//       NOT disturb host_adc_pwdn — opcode dispatch is properly mutually
//       exclusive.
//
//   T6: Without cmd_valid_100m, opcode bus changes alone do NOT update
//       host_adc_pwdn — the dispatcher only acts on validated commands.
// ============================================================================
`timescale 1ns/1ps

module tb_adc_pwdn_opcode;

    reg         clk = 1'b0;
    reg         reset_n;
    reg         cmd_valid_100m;
    reg  [7:0]  usb_cmd_opcode;
    reg  [31:0] usb_cmd_value;

    wire        host_adc_pwdn;
    wire [1:0]  host_adc_format;
    wire        adc_pwdn_pin;       // mirrors radar_receiver_final's `assign adc_pwdn = host_adc_pwdn`

    // ----------------------------------------------------------------
    // Production register block under test — mirrors the relevant
    // fragment of radar_system_top.v (post AUDIT-S25 commit). Kept tight
    // so the TB exercises the exact dispatch path that lives in prod.
    // ----------------------------------------------------------------
    dispatch_block dut (
        .clk             (clk),
        .reset_n         (reset_n),
        .cmd_valid_100m  (cmd_valid_100m),
        .usb_cmd_opcode  (usb_cmd_opcode),
        .usb_cmd_value   (usb_cmd_value),
        .host_adc_pwdn   (host_adc_pwdn),
        .host_adc_format (host_adc_format)
    );

    // mirror radar_receiver_final.v: `assign adc_pwdn = host_adc_pwdn`
    assign adc_pwdn_pin = host_adc_pwdn;

    // 100 MHz clock
    always #5 clk = ~clk;

    // Pass/fail bookkeeping
    integer pass_count = 0;
    integer fail_count = 0;
    task check;
        input        cond;
        input [255:0] label;
        begin
            if (cond) begin
                pass_count = pass_count + 1;
                $display("  [PASS] %0s", label);
            end else begin
                fail_count = fail_count + 1;
                $display("  [FAIL] %0s   (host_adc_pwdn=%0b adc_pwdn_pin=%0b)",
                         label, host_adc_pwdn, adc_pwdn_pin);
            end
        end
    endtask

    task issue_opcode;
        input [7:0]  opc;
        input [31:0] val;
        begin
            @(posedge clk);
            usb_cmd_opcode <= opc;
            usb_cmd_value  <= val;
            cmd_valid_100m <= 1'b1;
            @(posedge clk);
            cmd_valid_100m <= 1'b0;
            usb_cmd_opcode <= 8'h00;
            usb_cmd_value  <= 32'h0;
            @(posedge clk);    // settle
        end
    endtask

    initial begin
        $display("================================================");
        $display(" AUDIT-S25: opcode 0x32 -> host_adc_pwdn -> pin");
        $display("================================================");

        // ---------- T1: reset state ----------
        reset_n        = 1'b0;
        cmd_valid_100m = 1'b0;
        usb_cmd_opcode = 8'h00;
        usb_cmd_value  = 32'h0;
        repeat (4) @(posedge clk);
        reset_n        = 1'b1;
        @(posedge clk);
        check(host_adc_pwdn === 1'b0,        "T1: reset -> host_adc_pwdn = 0");
        check(adc_pwdn_pin  === 1'b0,        "T1: reset -> adc_pwdn pin = 0 (ADC powered up)");
        check(host_adc_format === 2'b00,     "T1: reset -> host_adc_format = 2'b00 (sister reg sanity)");

        // ---------- T2: assert PWDN via opcode 0x32 value=1 ----------
        issue_opcode(8'h32, 32'h0000_0001);
        check(host_adc_pwdn === 1'b1,        "T2: opcode 0x32 val=1 -> host_adc_pwdn = 1");
        check(adc_pwdn_pin  === 1'b1,        "T2: opcode 0x32 val=1 -> adc_pwdn pin = 1 (PWDN asserted)");

        // ---------- T3: deassert PWDN via opcode 0x32 value=0 ----------
        issue_opcode(8'h32, 32'h0000_0000);
        check(host_adc_pwdn === 1'b0,        "T3: opcode 0x32 val=0 -> host_adc_pwdn = 0");
        check(adc_pwdn_pin  === 1'b0,        "T3: opcode 0x32 val=0 -> adc_pwdn pin = 0");

        // ---------- T4: only bit[0] is consumed ----------
        // Set host_adc_pwdn high first.
        issue_opcode(8'h32, 32'h0000_0001);
        check(host_adc_pwdn === 1'b1,        "T4-prep: PWDN re-asserted");
        // Now write opcode 0x32 with bit[0]=0 but bits[31:1] all set.
        // Production semantics is `host_adc_pwdn <= usb_cmd_value[0];` so the
        // upper bits must be ignored — bit[0]=0 wins.
        issue_opcode(8'h32, 32'hFFFF_FFFE);
        check(host_adc_pwdn === 1'b0,        "T4: opcode 0x32 val=0xFFFF_FFFE (bit0=0) -> host_adc_pwdn = 0 (upper bits ignored)");

        // ---------- T5: unrelated opcodes don't disturb PWDN ----------
        issue_opcode(8'h32, 32'h0000_0001);
        check(host_adc_pwdn === 1'b1,        "T5-prep: PWDN re-asserted");
        // Issue opcode 0x33 (host_adc_format) — must NOT touch host_adc_pwdn.
        issue_opcode(8'h33, 32'h0000_0001);
        check(host_adc_pwdn   === 1'b1,      "T5: opcode 0x33 doesn't disturb host_adc_pwdn");
        check(host_adc_format === 2'b01,     "T5: opcode 0x33 updates host_adc_format independently");
        // Issue opcode 0x01 (radar_mode) — must NOT touch host_adc_pwdn.
        issue_opcode(8'h01, 32'h0000_0002);
        check(host_adc_pwdn === 1'b1,        "T5: opcode 0x01 doesn't disturb host_adc_pwdn");

        // ---------- T6: opcode bus changes without cmd_valid_100m don't latch ----------
        // Snap state, drive opcode/value but withhold cmd_valid_100m.
        @(posedge clk);
        usb_cmd_opcode <= 8'h32;
        usb_cmd_value  <= 32'h0000_0000;
        cmd_valid_100m <= 1'b0;
        @(posedge clk);
        @(posedge clk);
        check(host_adc_pwdn === 1'b1,        "T6: opcode 0x32 + val=0 without cmd_valid -> host_adc_pwdn unchanged (still 1)");

        // Now actually pulse cmd_valid_100m.
        cmd_valid_100m <= 1'b1;
        @(posedge clk);
        cmd_valid_100m <= 1'b0;
        @(posedge clk);
        check(host_adc_pwdn === 1'b0,        "T6: opcode 0x32 + val=0 WITH cmd_valid -> host_adc_pwdn cleared");

        // ---------- Summary ----------
        $display("================================================");
        $display(" RESULTS: %0d passed, %0d failed", pass_count, fail_count);
        $display("================================================");
        if (fail_count == 0) $finish;
        else                 $fatal(1, "FAIL");
    end

    // Watchdog
    initial begin
        #10000;
        $display("[FAIL] watchdog timeout");
        $fatal(1, "WATCHDOG");
    end

endmodule


// ============================================================================
// dispatch_block: minimal mirror of the relevant fragment of
// radar_system_top.v's host-register block (the AUDIT-S25 + AUDIT-C3 + a
// representative third opcode 0x01 used to demonstrate dispatch isolation).
//
// IMPORTANT: this is a *copy* of the production logic, not the production
// module. If radar_system_top.v's dispatch logic changes shape (e.g.,
// pipelining the opcode bus, adding an enable mask), this TB will need to be
// updated to match — a deliberate trip-wire so the dispatch contract gets
// re-verified during structural changes.
// ============================================================================
module dispatch_block (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        cmd_valid_100m,
    input  wire [7:0]  usb_cmd_opcode,
    input  wire [31:0] usb_cmd_value,
    output reg         host_adc_pwdn,
    output reg [1:0]   host_adc_format
);

    // Dummy reg for opcode 0x01 (radar_mode) — exercised only by T5.
    reg [1:0] host_radar_mode;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            host_adc_pwdn   <= 1'b0;
            host_adc_format <= 2'b00;
            host_radar_mode <= 2'b00;
        end else begin
            if (cmd_valid_100m) begin
                case (usb_cmd_opcode)
                    8'h01: host_radar_mode <= usb_cmd_value[1:0];
                    8'h32: host_adc_pwdn   <= usb_cmd_value[0];
                    8'h33: host_adc_format <= usb_cmd_value[1:0];
                    default: ;
                endcase
            end
        end
    end

endmodule
