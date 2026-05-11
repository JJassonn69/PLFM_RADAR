`timescale 1ns / 1ps
`include "radar_params.vh"

// ============================================================================
// tb_usb_protocol_v2.v
//
// PR-G focused round-trip verification for usb_data_interface_ft2232h.v:
//   1. Opcode 0x2D (host_cfar_alpha_soft) write path — verify cmd_value
//      reaches the cmd_* outputs of the read FSM with the right byte order.
//   2. Bulk frame header v2 — verify byte0=0xAA, byte1=0x02 (version),
//      byte2=stream flags, bytes3-8=frame_num/range/doppler counts.
//   3. Status packet length — verify 34 bytes (M-5; was 30 / PR-G; 26 / v1).
//      status_words[6] carries detect_count_cand/detect_threshold_soft (PR-G).
//      status_words[7] carries medium_chirp/medium_listen (M-5).
//   4. PR-G FSM trim — full-frame header/body length consistency. With all
//      streams enabled, total emitted bytes must equal 9 (hdr) + range×2 +
//      range×doppler×2 (doppler) + range×doppler×2/8 (detect) + 1 (footer).
//      Catches future header-vs-body drift and confirms padding is skipped.
//   5. PR-G G2 — MEDIUM ladder timing opcodes (0x17, 0x18) round-trip via
//      cmd_opcode/cmd_value (the host_medium_*_cycles registers live in
//      radar_system_top, exercised at integration level by tb_system_opcodes).
// ============================================================================

module tb_usb_protocol_v2;
    localparam CLK_PER    = 10.0;     // 100 MHz
    localparam FT_CLK_PER = 16.667;   // 60 MHz

    reg clk         = 1'b0;
    reg ft_clk      = 1'b0;
    reg reset_n     = 1'b0;
    reg ft_reset_n  = 1'b0;

    // Radar inputs (clk domain)
    reg [31:0] range_profile = 32'd0;
    reg        range_valid   = 1'b0;
    reg [15:0] doppler_real  = 16'd0;
    reg [15:0] doppler_imag  = 16'd0;
    reg        doppler_valid = 1'b0;
    reg [`RP_DETECT_CLASS_WIDTH-1:0] cfar_detect_class = `RP_DETECT_NONE;
    reg        cfar_valid    = 1'b0;

    reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin_in   = 0;
    reg [`RP_DOPPLER_BIN_WIDTH-1:0]   doppler_bin_in = 0;
    reg                               frame_complete = 1'b0;

    // FT2232H interface
    wire [7:0] ft_data;
    reg        ft_rxf_n = 1'b1;
    reg        ft_txe_n = 1'b0;
    wire       ft_rd_n;
    wire       ft_wr_n;
    wire       ft_oe_n;
    wire       ft_siwu;

    // Bidirectional data: tristate driver from TB for read path
    reg [7:0]  ft_data_drive   = 8'd0;
    reg        ft_data_drive_en = 1'b0;
    assign ft_data = ft_data_drive_en ? ft_data_drive : 8'hzz;
    pulldown pd[7:0] (ft_data);

    wire [31:0] cmd_data;
    wire        cmd_valid;
    wire [7:0]  cmd_opcode;
    wire [7:0]  cmd_addr;
    wire [15:0] cmd_value;

    // PR-G v2: enable all 3 streams (range|doppler|cfar). Bits [5:3] reserved=0.
    reg [5:0] stream_control     = 6'b000_111;
    reg [5:0] status_stream_ctrl = 6'b000_111;
    // PR-U / M-8: production 3-PRI ladder (SHORT|MEDIUM|LONG).
    reg [2:0] subframe_enable    = 3'b111;

    // Status inputs (mostly tied off; PR-G additions below)
    reg        status_request = 1'b0;
    reg [15:0] status_cfar_threshold = 16'h1234;
    reg [15:0] status_long_chirp = 16'd0;
    reg [15:0] status_long_listen = 16'd0;
    reg [15:0] status_guard = 16'd0;
    reg [15:0] status_short_chirp = 16'd0;
    reg [15:0] status_short_listen = 16'd0;
    // M-5: status_words[7] medium PRI readback (test default = production
    // RP_DEF_MEDIUM_*_CYCLES so the round-trip canary is the real boot value)
    reg [15:0] status_medium_chirp  = 16'd`RP_DEF_MEDIUM_CHIRP_CYCLES;
    reg [15:0] status_medium_listen = 16'd`RP_DEF_MEDIUM_LISTEN_CYCLES;
    reg [5:0]  status_chirps_per_elev = 6'd0;
    reg        status_chirps_mismatch = 1'b0;
    reg [4:0]  status_self_test_flags = 5'd0;
    reg [7:0]  status_self_test_detail = 8'd0;
    reg        status_self_test_busy = 1'b0;
    reg [3:0]  status_agc_current_gain = 4'd0;
    reg [7:0]  status_agc_peak_magnitude = 8'd0;
    reg [7:0]  status_agc_saturation_count = 8'd0;
    reg        status_agc_enable = 1'b0;
    reg        status_range_decim_watchdog = 1'b0;
    reg        status_ddc_cic_fir_overrun  = 1'b0;
    // PR-G new
    reg [7:0]  status_cfar_alpha_soft       = `RP_DEF_CFAR_ALPHA_SOFT;  // 0x18
    reg [16:0] status_detect_threshold_soft = 17'h00ABC;
    reg [15:0] status_detect_count_cand     = 16'd42;

    integer pass = 0;
    integer fail = 0;

    always #(CLK_PER/2)    clk = ~clk;
    always #(FT_CLK_PER/2) ft_clk = ~ft_clk;

    usb_data_interface_ft2232h u_dut (
        .clk(clk),
        .reset_n(reset_n),
        .ft_reset_n(ft_reset_n),
        .range_profile(range_profile),
        .range_valid(range_valid),
        .doppler_real(doppler_real),
        .doppler_imag(doppler_imag),
        .doppler_valid(doppler_valid),
        .cfar_detect_class(cfar_detect_class),
        .cfar_valid(cfar_valid),
        .range_bin_in(range_bin_in),
        .doppler_bin_in(doppler_bin_in),
        .frame_complete(frame_complete),
        .ft_data(ft_data),
        .ft_rxf_n(ft_rxf_n),
        .ft_txe_n(ft_txe_n),
        .ft_rd_n(ft_rd_n),
        .ft_wr_n(ft_wr_n),
        .ft_oe_n(ft_oe_n),
        .ft_siwu(ft_siwu),
        .ft_clk(ft_clk),
        .cmd_data(cmd_data),
        .cmd_valid(cmd_valid),
        .cmd_opcode(cmd_opcode),
        .cmd_addr(cmd_addr),
        .cmd_value(cmd_value),
        .stream_control(stream_control),
        // PR-U / M-8: per-frame snapshot of host_subframe_enable echoed in
        // v2 frame byte 2 bits[5:3].
        .subframe_enable(subframe_enable),
        .status_request(status_request),
        .status_cfar_threshold(status_cfar_threshold),
        .status_stream_ctrl(status_stream_ctrl),
        .status_long_chirp(status_long_chirp),
        .status_long_listen(status_long_listen),
        .status_guard(status_guard),
        .status_short_chirp(status_short_chirp),
        .status_short_listen(status_short_listen),
        // M-5: medium PRI readback feeding status_words[7]
        .status_medium_chirp(status_medium_chirp),
        .status_medium_listen(status_medium_listen),
        .status_chirps_per_elev(status_chirps_per_elev),
        .status_chirps_mismatch(status_chirps_mismatch),
        .status_self_test_flags(status_self_test_flags),
        .status_self_test_detail(status_self_test_detail),
        .status_self_test_busy(status_self_test_busy),
        .status_agc_current_gain(status_agc_current_gain),
        .status_agc_peak_magnitude(status_agc_peak_magnitude),
        .status_agc_saturation_count(status_agc_saturation_count),
        .status_agc_enable(status_agc_enable),
        .status_range_decim_watchdog(status_range_decim_watchdog),
        .status_ddc_cic_fir_overrun(status_ddc_cic_fir_overrun),
        // PR-AB.b expanded commit 5: beam-handshake watchdog tied off here;
        // exercised by tb_chirp_scheduler_handshake.v and word-4 layout test
        // refreshed below.
        .status_beam_handshake_watchdog(1'b0),
        .status_cfar_alpha_soft(status_cfar_alpha_soft),
        .status_detect_threshold_soft(status_detect_threshold_soft),
        .status_detect_count_cand(status_detect_count_cand)
    );

    // Capture egress bytes. egress_count counts ALL emitted bytes (used by
    // TEST 4 to verify total frame length). egress_bytes only buffers the
    // first 36 (header + a few status bytes — enough for TESTS 2, 3, 4 to
    // index byte-level checks).
    reg [7:0]  egress_bytes [0:35];
    integer    egress_count = 0;
    always @(posedge ft_clk) begin
        if (!ft_wr_n && !ft_txe_n) begin
            if (egress_count < 36)
                egress_bytes[egress_count] <= ft_data;
            egress_count <= egress_count + 1;
        end
    end

    task check_b;
        input [127:0] tag;
        input         cond;
        begin
            if (cond) begin
                $display("[PASS] %0s", tag);
                pass = pass + 1;
            end else begin
                $display("[FAIL] %0s", tag);
                fail = fail + 1;
            end
        end
    endtask

    task wait_clk;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) @(posedge clk);
        end
    endtask

    // 4-byte command bus driver (host → FPGA, ft_clk domain).
    // Read FSM: RD_IDLE (edge 1: see rxf_n, schedule transition) → RD_OE_ASSERT
    // (edge 2: schedule RD_READING) → RD_READING (edges 3,4,5,6: each samples
    // ft_data via NBA). Byte N must be on the bus at edge (N+2) of the sequence.
    task send_cmd;
        input [7:0]  op;
        input [7:0]  addr;
        input [15:0] val;
        begin
            @(posedge ft_clk); #1;          // Edge 0
            ft_rxf_n         = 1'b0;
            ft_data_drive    = op;
            ft_data_drive_en = 1'b1;
            @(posedge ft_clk); #1;          // Edge 1: RD_IDLE → RD_OE_ASSERT (NBA)
            @(posedge ft_clk); #1;          // Edge 2: RD_OE_ASSERT → RD_READING (NBA)
            @(posedge ft_clk); #1;          // Edge 3: RD_READING samples op (1st)
            ft_data_drive    = addr;
            @(posedge ft_clk); #1;          // Edge 4: samples addr (2nd)
            ft_data_drive    = val[15:8];
            @(posedge ft_clk); #1;          // Edge 5: samples val_hi (3rd)
            ft_data_drive    = val[7:0];
            @(posedge ft_clk); #1;          // Edge 6: samples val_lo (4th, transitions out)
            ft_rxf_n         = 1'b1;
            ft_data_drive_en = 1'b0;
            wait_clk(20);                   // CDC propagation to clk domain
        end
    endtask

    initial begin
        $display("\n========== tb_usb_protocol_v2 ==========");
        // Reset
        reset_n    = 1'b0;
        ft_reset_n = 1'b0;
        wait_clk(10);
        reset_n    = 1'b1;
        ft_reset_n = 1'b1;
        wait_clk(20);

        // -------------------------------------------------------------
        // TEST 1: Opcode 0x2D (host_cfar_alpha_soft) round trip
        // -------------------------------------------------------------
        $display("\n[TEST 1] Opcode 0x2D (cfar_alpha_soft) round trip");
        send_cmd(`RP_OP_CFAR_ALPHA_SOFT, 8'h00, 16'h0024);  // 0x24 in Q4.4 = 2.25
        check_b("T1.1: cmd_opcode=0x2D",  cmd_opcode == 8'h2D);
        check_b("T1.2: cmd_value lower 8b=0x24", cmd_value[7:0] == 8'h24);

        // -------------------------------------------------------------
        // TEST 2: Frame header v2 — 9 bytes, byte1=0x02
        // -------------------------------------------------------------
        $display("\n[TEST 2] Frame header v2 emission");
        // Disable all stream sections (HDR -> FOOTER fast drain)
        stream_control = 6'b000_000;
        wait_clk(50);  // Let CDC propagate
        egress_count = 0;
        @(posedge clk);
        frame_complete = 1'b1;
        @(posedge clk);
        frame_complete = 1'b0;
        // Wait for full frame drain (10 bytes = 10 ft_clk + slack)
        wait_clk(150);
        check_b("T2.1: byte0 = 0xAA",         egress_bytes[0] == 8'hAA);
        check_b("T2.2: byte1 = 0x02 (ver)",   egress_bytes[1] == `RP_USB_PROTOCOL_VERSION);
        // PR-U / M-8: byte 2 = {2'b00, subframe_enable[2:0], stream[2:0]}.
        // subframe_enable defaults to 3'b111 → byte 2 = (0b111 << 3) | 0 = 0x38.
        check_b("T2.3: byte2 = {00, sf=111, stream=0} = 0x38",
                egress_bytes[2] == 8'h38);
        // Byte 3-4 = frame_number snapshot. snapshot latches OLD frame_number
        // at frame_complete (NBA), so first frame emitted carries fn=0.
        check_b("T2.4: byte3 = fn[15:8]=0",   egress_bytes[3] == 8'h00);
        check_b("T2.5: byte4 = fn[7:0]=0",    egress_bytes[4] == 8'h00);
        check_b("T2.6: byte5/6 = range_bins=512",
                {egress_bytes[5], egress_bytes[6]} == 16'd512);
        check_b("T2.7: byte7/8 = doppler_bins=48",
                {egress_bytes[7], egress_bytes[8]} == 16'd48);
        check_b("T2.8: byte9 = footer 0x55",  egress_bytes[9] == 8'h55);

        // -------------------------------------------------------------
        // TEST 3: Status packet length = 34 bytes (M-5);
        //   word[6] carries 2-tier CFAR telemetry (PR-G);
        //   word[7] carries medium_chirp/medium_listen (M-5).
        // -------------------------------------------------------------
        $display("\n[TEST 3] Status packet length 34B + word[6]/word[7] fields");
        egress_count = 0;
        @(posedge clk);
        status_request = 1'b1;
        @(posedge clk);
        status_request = 1'b0;
        wait_clk(300);  // Wait for status drain
        check_b("T3.1: byte0 = 0xBB (status header)", egress_bytes[0] == 8'hBB);
        // M-5: footer moved 29→33 with the 4-byte word[7] insertion.
        check_b("T3.2: byte33 = 0x55 (footer)",       egress_bytes[33] == 8'h55);
        check_b("T3.3: status_words[6] count_cand[15:8]=0", egress_bytes[25] == 8'h00);
        check_b("T3.4: status_words[6] count_cand[7:0]=42", egress_bytes[26] == 8'd42);
        check_b("T3.5: status_words[6] thr_soft[15:8]=0x0A", egress_bytes[27] == 8'h0A);
        check_b("T3.6: status_words[6] thr_soft[7:0]=0xBC",  egress_bytes[28] == 8'hBC);
        // alpha_soft (0x18) packed into word[4][9:2] → byte at index 19,20.
        // PR-AB.b expanded: word[4][1:0] formerly range_mode, now reserved 2'd0.
        // word[4] = {gain[3:0], peak[7:0], sat[7:0], en, mismatch, alpha_soft[7:0], 2'd0}
        // When alpha_soft=0x18: word[4][7:0] = {8'h18, 2'b00} = 8'h60.
        check_b("T3.7: status_words[4][7:0] = alpha_soft<<2 = 0x60 (alpha=0x18)",
                egress_bytes[20] == 8'h60);
        // M-5: status_words[7] = {medium_chirp, medium_listen}.
        // Defaults from RP_DEF_*: 500 (0x01F4) chirp / 15600 (0x3CF0) listen.
        check_b("T3.8: status_words[7] medium_chirp[15:8]=0x01",  egress_bytes[29] == 8'h01);
        check_b("T3.9: status_words[7] medium_chirp[7:0]=0xF4",   egress_bytes[30] == 8'hF4);
        check_b("T3.10: status_words[7] medium_listen[15:8]=0x3C", egress_bytes[31] == 8'h3C);
        check_b("T3.11: status_words[7] medium_listen[7:0]=0xF0",  egress_bytes[32] == 8'hF0);

        // -------------------------------------------------------------
        // TEST 4: full-frame header/body length consistency (PR-G trim)
        // -------------------------------------------------------------
        $display("\n[TEST 4] Full-frame header/body length consistency (PR-G trim)");
        // Re-enable all 3 streams so HDR + range + doppler + detect + footer
        // are all emitted. We don't fill BRAMs — only the byte count matters.
        stream_control = 6'b000_111;
        wait_clk(50);  // CDC propagate
        egress_count = 0;
        @(posedge clk);
        frame_complete = 1'b1;
        @(posedge clk);
        frame_complete = 1'b0;
        // Worst-case drain: 9 + 1024 + 49152 + 6144 + 1 = 56330 bytes.
        // Each doppler byte takes ~1 ft_clk (MSB then LSB, both at 60 MHz).
        // Detect = 1 byte/ft_clk. Plus FSM transitions, so allow ~70k ft_clk.
        wait_clk(120_000);  // ~1.2 ms in clk-domain (covers 60 MHz drain)
        check_b("T4.1: egress_count == expected total",
                egress_count == (`RP_FRAME_HDR_BYTES
                                 + `RP_NUM_RANGE_BINS * 2
                                 + `RP_NUM_RANGE_BINS * `RP_NUM_DOPPLER_BINS * 2
                                 + (`RP_NUM_RANGE_BINS * `RP_NUM_DOPPLER_BINS * 2) / 8
                                 + 1));
        check_b("T4.2: header byte0 = 0xAA (frame still framed correctly)",
                egress_bytes[0] == 8'hAA);
        check_b("T4.3: header byte1 = protocol version 0x02",
                egress_bytes[1] == `RP_USB_PROTOCOL_VERSION);
        check_b("T4.4: header byte5/6 = range_bins=512",
                {egress_bytes[5], egress_bytes[6]} == 16'd512);
        check_b("T4.5: header byte7/8 = doppler_bins=48",
                {egress_bytes[7], egress_bytes[8]} == 16'd48);
        // Sanity: doppler section must NOT be the old 65536-byte padded size.
        // Old (pre-trim) total was 9 + 1024 + 65536 + 8192 + 1 = 74762.
        // New (post-trim) total = 56330. Catch if FSM regresses to padded.
        check_b("T4.6: emitted bytes < pre-trim padded total (74762)",
                egress_count < 74762);
        $display("    egress_count = %0d (expected 56330)", egress_count);

        // -------------------------------------------------------------
        // TEST 5: MEDIUM ladder timing opcodes (PR-G G2) — round-trip via cmd bus
        // -------------------------------------------------------------
        $display("\n[TEST 5] MEDIUM ladder timing opcodes (0x17, 0x18)");
        send_cmd(`RP_OP_MEDIUM_CHIRP_CYCLES, 8'h00, 16'd750);
        check_b("T5.1: cmd_opcode=0x17 (MEDIUM_CHIRP_CYCLES)", cmd_opcode == 8'h17);
        check_b("T5.2: cmd_value=750",                          cmd_value == 16'd750);

        send_cmd(`RP_OP_MEDIUM_LISTEN_CYCLES, 8'h00, 16'd16500);
        check_b("T5.3: cmd_opcode=0x18 (MEDIUM_LISTEN_CYCLES)", cmd_opcode == 8'h18);
        check_b("T5.4: cmd_value=16500",                         cmd_value == 16'd16500);

        // -------------------------------------------------------------
        // Done
        // -------------------------------------------------------------
        $display("\n-----------------------------------------------------------");
        $display("RESULTS: %0d PASS, %0d FAIL", pass, fail);
        $display("-----------------------------------------------------------");
        if (fail == 0) $display("[OVERALL PASS]"); else $display("[OVERALL FAIL]");
        $finish;
    end

    // Watchdog
    initial begin
        #20_000_000;
        $display("[TIMEOUT] tb_usb_protocol_v2 watchdog");
        $finish;
    end

endmodule
