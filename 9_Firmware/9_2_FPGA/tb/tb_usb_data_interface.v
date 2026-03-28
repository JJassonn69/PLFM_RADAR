`timescale 1ns / 1ps

// ============================================================================
// tb_usb_data_interface.v — v9 Rewrite
//
// Isolation testbench for usb_data_interface.v (v9d architecture).
// Tests the USB FSM module directly without the full radar system wrapper.
//
// v9 architecture: Independent packet types (range/doppler/cfar), each with
// its own header->data->footer path. 4-bit write FSM states. Priority arbiter:
// Status > Range > Doppler > CFAR.
//
// Compile: iverilog -DSIMULATION -o tb_usb -Wall tb/tb_usb_data_interface.v usb_data_interface.v
// Run:     vvp tb_usb
// ============================================================================
module tb_usb_data_interface;

    // ── Parameters ─────────────────────────────────────────────
    localparam CLK_PERIOD     = 10.0;  // 100 MHz main clock
    localparam FT_CLK_PERIOD  = 10.0;  // 100 MHz FT601 clock

    // v9 Write FSM state definitions (4-bit, mirror the DUT)
    localparam [3:0] S_IDLE             = 4'd0,
                     S_SEND_RANGE_HDR   = 4'd1,
                     S_SEND_RANGE_DATA  = 4'd2,
                     S_SEND_DOPPLER_HDR = 4'd3,
                     S_SEND_DOPPLER_DATA= 4'd4,
                     S_SEND_CFAR_HDR    = 4'd5,
                     S_SEND_CFAR_DATA   = 4'd6,
                     S_SEND_FOOTER      = 4'd7,
                     S_WAIT_ACK         = 4'd8,
                     S_SEND_STATUS      = 4'd9;

    // ── Signals ────────────────────────────────────────────────
    reg         clk;
    reg         reset_n;

    // Radar data inputs — Range
    reg  [31:0] range_profile;
    reg         range_valid;

    // Radar data inputs — Doppler (v9: with metadata)
    reg  [15:0] doppler_real;
    reg  [15:0] doppler_imag;
    reg         doppler_valid;
    reg  [5:0]  doppler_range_bin;
    reg  [4:0]  doppler_doppler_bin;
    reg         doppler_sub_frame;

    // Radar data inputs — CFAR (v9: full detection report)
    reg         cfar_detection;
    reg         cfar_valid;
    reg  [5:0]  cfar_detect_range;
    reg  [4:0]  cfar_detect_doppler;
    reg  [16:0] cfar_detect_mag;
    reg  [16:0] cfar_detect_thr;

    // FT601 interface
    wire [31:0] ft601_data;
    wire [3:0]  ft601_be;
    wire        ft601_txe_n;
    wire        ft601_rxf_n;
    reg         ft601_txe;
    reg         ft601_rxf;
    wire        ft601_wr_n;
    wire        ft601_rd_n;
    wire        ft601_oe_n;
    wire        ft601_siwu_n;
    reg  [1:0]  ft601_srb;
    reg  [1:0]  ft601_swb;
    wire        ft601_clk_out;
    reg         ft601_clk_in;

    // Pulldown: when nobody drives, data reads as 0 (not X)
    pulldown pd[31:0] (ft601_data);

    // Host-to-FPGA data bus driver (for read path testing)
    reg [31:0] host_data_drive;
    reg        host_data_drive_en;
    assign ft601_data = host_data_drive_en ? host_data_drive : 32'hzzzz_zzzz;

    // DUT command outputs (read path)
    wire [31:0] cmd_data;
    wire        cmd_valid;
    wire [7:0]  cmd_opcode;
    wire [7:0]  cmd_addr;
    wire [15:0] cmd_value;

    // Stream control + status readback inputs
    reg  [2:0]  stream_control;
    reg         status_request;
    reg  [15:0] status_cfar_threshold;
    reg  [2:0]  status_stream_ctrl;
    reg  [1:0]  status_radar_mode;
    reg  [15:0] status_long_chirp;
    reg  [15:0] status_long_listen;
    reg  [15:0] status_guard;
    reg  [15:0] status_short_chirp;
    reg  [15:0] status_short_listen;
    reg  [5:0]  status_chirps_per_elev;
    reg  [1:0]  status_range_mode;

    // Self-test status readback inputs
    reg  [4:0]  status_self_test_flags;
    reg  [7:0]  status_self_test_detail;
    reg         status_self_test_busy;

    // CFAR debug counters (v9c)
    reg  [15:0] cfar_dbg_cells_processed;
    reg  [7:0]  cfar_dbg_cols_completed;
    reg  [15:0] cfar_dbg_valid_count;
    reg  [15:0] cfar_detect_count;

    // Debug outputs
    wire [15:0] dbg_wr_strobes;
    wire [15:0] dbg_txe_blocks;
    wire [15:0] dbg_pkt_starts;
    wire [15:0] dbg_pkt_completions;

    // v9d pending flag exports
    wire        range_pending_out;
    wire        doppler_pending_out;
    wire        cfar_pending_out;
    wire        write_idle;

    // ── Clock generators ───────────────────────────────────────
    always #(CLK_PERIOD / 2) clk = ~clk;
    always #(FT_CLK_PERIOD / 2) ft601_clk_in = ~ft601_clk_in;

    // ── DUT ────────────────────────────────────────────────────
    usb_data_interface uut (
        .clk              (clk),
        .reset_n          (reset_n),
        .ft601_reset_n    (reset_n),

        // Range
        .range_profile    (range_profile),
        .range_valid      (range_valid),

        // Doppler (v9: with metadata)
        .doppler_real     (doppler_real),
        .doppler_imag     (doppler_imag),
        .doppler_valid    (doppler_valid),
        .doppler_range_bin     (doppler_range_bin),
        .doppler_doppler_bin   (doppler_doppler_bin),
        .doppler_sub_frame     (doppler_sub_frame),

        // CFAR (v9: full detection report)
        .cfar_detection   (cfar_detection),
        .cfar_valid       (cfar_valid),
        .cfar_detect_range   (cfar_detect_range),
        .cfar_detect_doppler (cfar_detect_doppler),
        .cfar_detect_mag     (cfar_detect_mag),
        .cfar_detect_thr     (cfar_detect_thr),

        // FT601 interface
        .ft601_data       (ft601_data),
        .ft601_be         (ft601_be),
        .ft601_txe_n      (ft601_txe_n),
        .ft601_rxf_n      (ft601_rxf_n),
        .ft601_txe        (ft601_txe),
        .ft601_rxf        (ft601_rxf),
        .ft601_wr_n       (ft601_wr_n),
        .ft601_rd_n       (ft601_rd_n),
        .ft601_oe_n       (ft601_oe_n),
        .ft601_siwu_n     (ft601_siwu_n),
        .ft601_srb        (ft601_srb),
        .ft601_swb        (ft601_swb),
        .ft601_clk_out    (ft601_clk_out),
        .ft601_clk_in     (ft601_clk_in),

        // Host command outputs
        .cmd_data         (cmd_data),
        .cmd_valid        (cmd_valid),
        .cmd_opcode       (cmd_opcode),
        .cmd_addr         (cmd_addr),
        .cmd_value        (cmd_value),

        // Stream control + status readback
        .stream_control        (stream_control),
        .status_request        (status_request),
        .status_cfar_threshold (status_cfar_threshold),
        .status_stream_ctrl    (status_stream_ctrl),
        .status_radar_mode     (status_radar_mode),
        .status_long_chirp     (status_long_chirp),
        .status_long_listen    (status_long_listen),
        .status_guard          (status_guard),
        .status_short_chirp    (status_short_chirp),
        .status_short_listen   (status_short_listen),
        .status_chirps_per_elev(status_chirps_per_elev),
        .status_range_mode     (status_range_mode),

        // Self-test status readback
        .status_self_test_flags (status_self_test_flags),
        .status_self_test_detail(status_self_test_detail),
        .status_self_test_busy  (status_self_test_busy),

        // CFAR debug counters (v9c)
        .cfar_dbg_cells_processed(cfar_dbg_cells_processed),
        .cfar_dbg_cols_completed (cfar_dbg_cols_completed),
        .cfar_dbg_valid_count    (cfar_dbg_valid_count),
        .cfar_detect_count       (cfar_detect_count),

        // Debug outputs
        .dbg_wr_strobes     (dbg_wr_strobes),
        .dbg_txe_blocks     (dbg_txe_blocks),
        .dbg_pkt_starts     (dbg_pkt_starts),
        .dbg_pkt_completions(dbg_pkt_completions),

        // v9d exports
        .write_idle         (write_idle),
        .range_pending_out  (range_pending_out),
        .doppler_pending_out(doppler_pending_out),
        .cfar_pending_out   (cfar_pending_out)
    );

    // ── Test bookkeeping ───────────────────────────────────────
    integer pass_count;
    integer fail_count;
    integer test_num;
    integer csv_file;

    // ══════════════════════════════════════════════════════════
    // CONTINUOUS WRITE MONITOR
    //
    // Records every word written to the FT601 bus (wr_n=0 &&
    // data_oe=1) into a circular buffer. The test stimulus uses
    // mon_reset to clear and mon_count to read back.
    // ══════════════════════════════════════════════════════════
    reg [31:0] mon_buf [0:63];   // up to 64 words per test
    integer    mon_count;
    reg        mon_active;       // enable/disable monitoring

    always @(posedge ft601_clk_in) begin
        if (mon_active && ft601_wr_n === 1'b0 && uut.ft601_data_oe === 1'b1) begin
            if (mon_count < 64) begin
                mon_buf[mon_count] = ft601_data;
                mon_count = mon_count + 1;
            end
        end
    end

    task mon_reset;
        integer i;
        begin
            mon_count = 0;
            for (i = 0; i < 64; i = i + 1)
                mon_buf[i] = 32'hDEAD_DEAD;
            mon_active = 1;
        end
    endtask

    task mon_stop;
        begin
            mon_active = 0;
        end
    endtask

    // ── Check task (512-bit label) ─────────────────────────────
    task check;
        input cond;
        input [511:0] label;
        begin
            test_num = test_num + 1;
            if (cond) begin
                $display("[PASS] Test %0d: %0s", test_num, label);
                pass_count = pass_count + 1;
                $fwrite(csv_file, "%0d,PASS,%0s\n", test_num, label);
            end else begin
                $display("[FAIL] Test %0d: %0s", test_num, label);
                fail_count = fail_count + 1;
                $fwrite(csv_file, "%0d,FAIL,%0s\n", test_num, label);
            end
        end
    endtask

    // ── Check with value display (for debugging failures) ──────
    task check_val;
        input cond;
        input [511:0] label;
        input [31:0] actual;
        input [31:0] expected;
        begin
            test_num = test_num + 1;
            if (cond) begin
                $display("[PASS] Test %0d: %0s", test_num, label);
                pass_count = pass_count + 1;
                $fwrite(csv_file, "%0d,PASS,%0s\n", test_num, label);
            end else begin
                $display("[FAIL] Test %0d: %0s (got=0x%08h, exp=0x%08h)", test_num, label, actual, expected);
                fail_count = fail_count + 1;
                $fwrite(csv_file, "%0d,FAIL,%0s\n", test_num, label);
            end
        end
    endtask

    // ── Helper: apply reset ────────────────────────────────────
    task apply_reset;
        begin
            mon_active           = 0;
            reset_n              = 0;
            range_profile        = 32'h0;
            range_valid          = 0;
            doppler_real         = 16'h0;
            doppler_imag         = 16'h0;
            doppler_valid        = 0;
            doppler_range_bin    = 6'd0;
            doppler_doppler_bin  = 5'd0;
            doppler_sub_frame    = 1'b0;
            cfar_detection       = 0;
            cfar_valid           = 0;
            cfar_detect_range    = 6'd0;
            cfar_detect_doppler  = 5'd0;
            cfar_detect_mag      = 17'd0;
            cfar_detect_thr      = 17'd0;
            ft601_txe            = 0;
            ft601_rxf            = 1;
            ft601_srb            = 2'b00;
            ft601_swb            = 2'b00;
            host_data_drive      = 32'h0;
            host_data_drive_en   = 0;
            stream_control        = 3'b111;
            status_request        = 0;
            status_cfar_threshold = 16'd10000;
            status_stream_ctrl    = 3'b111;
            status_radar_mode     = 2'b00;
            status_long_chirp     = 16'd3000;
            status_long_listen    = 16'd13700;
            status_guard          = 16'd17540;
            status_short_chirp    = 16'd50;
            status_short_listen   = 16'd17450;
            status_chirps_per_elev = 6'd32;
            status_range_mode     = 2'b00;
            status_self_test_flags  = 5'b00000;
            status_self_test_detail = 8'd0;
            status_self_test_busy   = 1'b0;
            cfar_dbg_cells_processed = 16'd0;
            cfar_dbg_cols_completed  = 8'd0;
            cfar_dbg_valid_count     = 16'd0;
            cfar_detect_count        = 16'd0;
            repeat (6) @(posedge ft601_clk_in);
            reset_n = 1;
            // Wait for stream_control CDC (2-stage sync) + startup lockout (255 cycles)
            repeat (260) @(posedge ft601_clk_in);
        end
    endtask

    // ── Helper: wait for DUT to reach a specific write FSM state ─
    task wait_for_state;
        input [3:0] target;
        input integer max_cyc;
        integer cnt;
        begin
            cnt = 0;
            while (uut.current_state !== target && cnt < max_cyc) begin
                @(posedge ft601_clk_in);
                cnt = cnt + 1;
            end
        end
    endtask

    // ── Helper: wait for FSM to return to IDLE ─────────────────
    task wait_idle;
        input integer max_cyc;
        begin
            wait_for_state(S_IDLE, max_cyc);
        end
    endtask

    // ── Helper: wait for a packet to fully complete ────────────
    // First waits for the FSM to LEAVE IDLE (proving it started),
    // then waits for it to RETURN to IDLE (proving it finished).
    // This avoids the NBA race where wait_idle returns immediately
    // because the FSM hasn't left IDLE yet.
    task wait_packet_done;
        input integer max_cyc;
        integer cnt;
        begin
            // Phase 1: Wait for FSM to leave IDLE
            cnt = 0;
            while (uut.current_state === S_IDLE && cnt < max_cyc) begin
                @(posedge ft601_clk_in);
                cnt = cnt + 1;
            end
            // Phase 2: Wait for FSM to return to IDLE
            wait_for_state(S_IDLE, max_cyc);
        end
    endtask

    // ── Helper: pulse range_valid for one ft601_clk cycle ──────
    task pulse_range_valid;
        input [31:0] data;
        begin
            @(posedge ft601_clk_in);
            range_profile = data;
            range_valid   = 1;
            @(posedge ft601_clk_in);
            range_valid = 0;
        end
    endtask

    // ── Helper: pulse doppler_valid for one ft601_clk cycle ────
    task pulse_doppler_valid;
        input [15:0] dr;
        input [15:0] di;
        input [5:0]  rbin;
        input [4:0]  dbin;
        input        sub;
        begin
            @(posedge ft601_clk_in);
            doppler_real        = dr;
            doppler_imag        = di;
            doppler_range_bin   = rbin;
            doppler_doppler_bin = dbin;
            doppler_sub_frame   = sub;
            doppler_valid       = 1;
            @(posedge ft601_clk_in);
            doppler_valid = 0;
        end
    endtask

    // ── Helper: pulse cfar_valid for one ft601_clk cycle ───────
    task pulse_cfar_valid;
        input        det;
        input [5:0]  rbin;
        input [4:0]  dbin;
        input [16:0] mag;
        input [16:0] thr;
        begin
            @(posedge ft601_clk_in);
            cfar_detection      = det;
            cfar_detect_range   = rbin;
            cfar_detect_doppler = dbin;
            cfar_detect_mag     = mag;
            cfar_detect_thr     = thr;
            cfar_valid          = 1;
            @(posedge ft601_clk_in);
            cfar_valid = 0;
        end
    endtask

    // ── Helper: wait for read FSM state ────────────────────────
    task wait_for_read_state;
        input [2:0] target;
        input integer max_cyc;
        integer cnt;
        begin
            cnt = 0;
            while (uut.read_state !== target && cnt < max_cyc) begin
                @(posedge ft601_clk_in);
                cnt = cnt + 1;
            end
        end
    endtask

    // ── Helper: send a host command via the read path ──────────
    // Word layout matching RTL decode (usb_data_interface.v:425-429):
    //   cmd_opcode = rx_data_captured[7:0]
    //   cmd_addr   = rx_data_captured[15:8]
    //   cmd_value  = {rx_data_captured[23:16], rx_data_captured[31:24]}
    task send_host_command;
        input [7:0]  opcode;
        input [7:0]  addr;
        input [15:0] value;
        reg [31:0] cmd_word;
        begin
            cmd_word = {value[7:0], value[15:8], addr, opcode};
            ft601_rxf = 0;
            wait_for_read_state(3'd1, 20); // RD_OE_ASSERT
            @(posedge ft601_clk_in); #1;
            host_data_drive = cmd_word;
            host_data_drive_en = 1;
            wait_for_read_state(3'd2, 20); // RD_READING
            @(posedge ft601_clk_in); #1;
            wait_for_read_state(3'd4, 20); // RD_PROCESS
            host_data_drive_en = 0;
            host_data_drive = 32'h0;
            ft601_rxf = 1;
            @(posedge ft601_clk_in); #1;
            wait_for_read_state(3'd0, 20); // RD_IDLE
            @(posedge ft601_clk_in); #1;
        end
    endtask

    // ── Stimulus ───────────────────────────────────────────────
    initial begin
        $dumpfile("tb_usb_data_interface.vcd");
        $dumpvars(0, tb_usb_data_interface);

        clk          = 0;
        ft601_clk_in = 0;
        pass_count   = 0;
        fail_count   = 0;
        test_num     = 0;
        mon_active   = 0;
        mon_count    = 0;
        host_data_drive    = 32'h0;
        host_data_drive_en = 0;

        csv_file = $fopen("tb_usb_data_interface.csv", "w");
        $fwrite(csv_file, "test_num,pass_fail,label\n");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 1: Reset Behaviour
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 1: Reset Behaviour ---");
        apply_reset;
        reset_n = 0;
        repeat (4) @(posedge ft601_clk_in); #1;

        check(uut.current_state === S_IDLE,
              "State is IDLE after reset");
        check(ft601_wr_n === 1'b1,
              "ft601_wr_n=1 after reset");
        check(uut.ft601_data_oe === 1'b0,
              "ft601_data_oe=0 after reset");
        check(ft601_rd_n === 1'b1,
              "ft601_rd_n=1 after reset");
        check(ft601_oe_n === 1'b1,
              "ft601_oe_n=1 after reset");
        check(ft601_siwu_n === 1'b1,
              "ft601_siwu_n=1 after reset");
        check(uut.range_data_pending === 1'b0,
              "range_data_pending=0 after reset");
        check(uut.doppler_data_pending === 1'b0,
              "doppler_data_pending=0 after reset");
        check(uut.cfar_data_pending === 1'b0,
              "cfar_data_pending=0 after reset");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 2: Range Packet (0xAA)
        //
        // v9: Range triggers its own independent packet:
        //   [0xAA hdr] [data] [data<<8] [data<<16] [data<<24] [0x55 footer]
        //   = 6 words total
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 2: Range Packet ---");
        apply_reset;
        ft601_txe = 0;
        mon_reset;

        pulse_range_valid(32'hDEAD_BEEF);
        wait_packet_done(200);
        repeat (2) @(posedge ft601_clk_in);
        mon_stop;

        check_val(mon_count == 6, "Range packet: 6 words captured",
                  mon_count, 6);
        check_val(mon_buf[0] === {24'h000000, 8'hAA},
                  "Range packet: word 0 = header 0xAA",
                  mon_buf[0], {24'h000000, 8'hAA});
        check_val(mon_buf[1] === 32'hDEAD_BEEF,
                  "Range packet: word 1 = range_profile_cap",
                  mon_buf[1], 32'hDEAD_BEEF);
        check_val(mon_buf[2] === {24'hADBEEF, 8'h00},
                  "Range packet: word 2 = data<<8",
                  mon_buf[2], {24'hADBEEF, 8'h00});
        check_val(mon_buf[3] === {16'hBEEF, 16'h0000},
                  "Range packet: word 3 = data<<16",
                  mon_buf[3], {16'hBEEF, 16'h0000});
        check_val(mon_buf[4] === {8'hEF, 24'h000000},
                  "Range packet: word 4 = data<<24",
                  mon_buf[4], {8'hEF, 24'h000000});
        check_val(mon_buf[5] === {24'h000000, 8'h55},
                  "Range packet: word 5 = footer 0x55",
                  mon_buf[5], {24'h000000, 8'h55});

        check(uut.current_state === S_IDLE,
              "Range packet: FSM back in IDLE");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 3: Doppler Packet (0xCC)
        //
        // v9: Independent Doppler packet:
        //   [0xCC hdr] [metadata+I] [Q+I] [0x55 footer] = 4 words
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 3: Doppler Packet ---");
        apply_reset;
        ft601_txe = 0;
        mon_reset;

        pulse_doppler_valid(16'h1234, 16'h5678, 6'd42, 5'd15, 1'b1);
        wait_packet_done(200);
        repeat (2) @(posedge ft601_clk_in);
        mon_stop;

        check_val(mon_count == 4, "Doppler packet: 4 words captured",
                  mon_count, 4);
        check_val(mon_buf[0] === {24'h000000, 8'hCC},
                  "Doppler packet: word 0 = header 0xCC",
                  mon_buf[0], {24'h000000, 8'hCC});

        // Word 1: {range_bin[5:0], doppler_bin[4:0], sub_frame, 4'b0000, I[15:0]}
        // range_bin=42=6'b101010, doppler_bin=15=5'b01111, sub_frame=1
        begin : doppler_word1_check
            reg [31:0] exp_w1;
            exp_w1 = {6'd42, 5'd15, 1'b1, 4'b0000, 16'h1234};
            check_val(mon_buf[1] === exp_w1,
                      "Doppler packet: word 1 = metadata+I",
                      mon_buf[1], exp_w1);
        end

        // Word 2: {Q[15:0], I[15:0]}
        check_val(mon_buf[2] === {16'h5678, 16'h1234},
                  "Doppler packet: word 2 = {Q, I}",
                  mon_buf[2], {16'h5678, 16'h1234});
        check_val(mon_buf[3] === {24'h000000, 8'h55},
                  "Doppler packet: word 3 = footer 0x55",
                  mon_buf[3], {24'h000000, 8'h55});

        check(uut.current_state === S_IDLE,
              "Doppler packet: FSM back in IDLE");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 4: CFAR Detection Packet (0xDD)
        //
        // v9: Independent CFAR packet:
        //   [0xDD hdr] [flag+range+doppler+mag] [threshold] [0x55] = 4 words
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 4: CFAR Detection Packet ---");
        apply_reset;
        ft601_txe = 0;
        mon_reset;

        pulse_cfar_valid(1'b1, 6'd33, 5'd10, 17'd98765, 17'd54321);
        wait_packet_done(200);
        repeat (2) @(posedge ft601_clk_in);
        mon_stop;

        check_val(mon_count == 4, "CFAR packet: 4 words captured",
                  mon_count, 4);
        check_val(mon_buf[0] === {24'h000000, 8'hDD},
                  "CFAR packet: word 0 = header 0xDD",
                  mon_buf[0], {24'h000000, 8'hDD});

        // Word 1: {flag, range[5:0], doppler[4:0], 3'b000, mag[16:0]}
        begin : cfar_word1_check
            reg [31:0] exp_w1;
            exp_w1 = {1'b1, 6'd33, 5'd10, 3'b000, 17'd98765};
            check_val(mon_buf[1] === exp_w1,
                      "CFAR packet: word 1 = {flag,range,doppler,mag}",
                      mon_buf[1], exp_w1);
        end

        // Word 2: {15'b0, threshold[16:0]}
        check_val(mon_buf[2] === {15'b0, 17'd54321},
                  "CFAR packet: word 2 = threshold",
                  mon_buf[2], {15'b0, 17'd54321});
        check_val(mon_buf[3] === {24'h000000, 8'h55},
                  "CFAR packet: word 3 = footer 0x55",
                  mon_buf[3], {24'h000000, 8'h55});

        check(uut.current_state === S_IDLE,
              "CFAR packet: FSM back in IDLE");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 5: Priority Arbiter
        //
        // When multiple data types are pending simultaneously,
        // the FSM should service them in priority order:
        // Range > Doppler > CFAR.
        //
        // Strategy: assert all three valid simultaneously, then
        // monitor which headers appear in order.
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 5: Priority Arbiter ---");
        apply_reset;
        ft601_txe = 0;
        mon_reset;

        // Assert all three valid simultaneously
        @(posedge ft601_clk_in);
        range_profile       = 32'hAAAA_1111;
        range_valid         = 1;
        doppler_real        = 16'hBBBB;
        doppler_imag        = 16'hCCCC;
        doppler_range_bin   = 6'd1;
        doppler_doppler_bin = 5'd2;
        doppler_sub_frame   = 1'b0;
        doppler_valid       = 1;
        cfar_detection      = 1'b1;
        cfar_detect_range   = 6'd3;
        cfar_detect_doppler = 5'd4;
        cfar_detect_mag     = 17'd1000;
        cfar_detect_thr     = 17'd500;
        cfar_valid          = 1;
        @(posedge ft601_clk_in);
        range_valid   = 0;
        doppler_valid = 0;
        cfar_valid    = 0;

        // Wait for all three packets to complete
        // Range(6) + Doppler(4) + CFAR(4) = 14 words + WAIT_ACK gaps
        // Use wait_packet_done for first, then wait for subsequent IDLE returns
        wait_packet_done(100);  // After range packet
        repeat (2) @(posedge ft601_clk_in);
        wait_packet_done(100);  // After doppler packet
        repeat (2) @(posedge ft601_clk_in);
        wait_packet_done(100);  // After cfar packet
        repeat (2) @(posedge ft601_clk_in);
        mon_stop;

        // Should have captured 14 words total: 6(range) + 4(doppler) + 4(cfar)
        check_val(mon_count == 14, "Priority: 14 total words (6+4+4)",
                  mon_count, 14);

        // First packet header should be range (0xAA)
        check_val(mon_buf[0] === {24'h0, 8'hAA},
                  "Priority: first header = 0xAA (range)",
                  mon_buf[0], {24'h0, 8'hAA});
        // Second packet header at index 6 should be doppler (0xCC)
        check_val(mon_buf[6] === {24'h0, 8'hCC},
                  "Priority: second header = 0xCC (doppler)",
                  mon_buf[6], {24'h0, 8'hCC});
        // Third packet header at index 10 should be cfar (0xDD)
        check_val(mon_buf[10] === {24'h0, 8'hDD},
                  "Priority: third header = 0xDD (cfar)",
                  mon_buf[10], {24'h0, 8'hDD});

        check(uut.current_state === S_IDLE,
              "Priority: FSM back in IDLE");
        check(uut.range_data_pending === 1'b0 &&
              uut.doppler_data_pending === 1'b0 &&
              uut.cfar_data_pending === 1'b0,
              "Priority: all pending flags cleared");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 6: Disabled-Stream Discard (v9d)
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 6: Disabled-Stream Discard ---");

        // 6a: Disable doppler stream, pulse doppler_valid
        apply_reset;
        ft601_txe = 0;
        stream_control = 3'b101;  // range + cfar only
        repeat (6) @(posedge ft601_clk_in);

        pulse_doppler_valid(16'hAAAA, 16'h5555, 6'd0, 5'd0, 1'b0);
        repeat (5) @(posedge ft601_clk_in); #1;
        check(uut.doppler_data_pending === 1'b0,
              "Discard: doppler pending cleared when stream disabled");

        // 6b: Disable range stream, pulse range_valid
        apply_reset;
        ft601_txe = 0;
        stream_control = 3'b110;
        repeat (6) @(posedge ft601_clk_in);

        pulse_range_valid(32'h1234_5678);
        repeat (5) @(posedge ft601_clk_in); #1;
        check(uut.range_data_pending === 1'b0,
              "Discard: range pending cleared when stream disabled");

        // 6c: Disable cfar stream, pulse cfar_valid
        apply_reset;
        ft601_txe = 0;
        stream_control = 3'b011;
        repeat (6) @(posedge ft601_clk_in);

        pulse_cfar_valid(1'b1, 6'd5, 5'd3, 17'd100, 17'd50);
        repeat (5) @(posedge ft601_clk_in); #1;
        check(uut.cfar_data_pending === 1'b0,
              "Discard: cfar pending cleared when stream disabled");

        // 6d: Disable all streams — no packets should be sent
        apply_reset;
        ft601_txe = 0;
        stream_control = 3'b000;
        repeat (6) @(posedge ft601_clk_in);

        pulse_range_valid(32'hDEAD_DEAD);
        repeat (10) @(posedge ft601_clk_in); #1;
        check(uut.current_state === S_IDLE,
              "Discard: FSM stays IDLE when all streams disabled");
        check(uut.range_data_pending === 1'b0,
              "Discard: range pending discarded when all disabled");

        // 6e: Disable doppler, enable range+cfar: pulse all three.
        // Should get range + cfar packets, no doppler.
        apply_reset;
        ft601_txe = 0;
        stream_control = 3'b101;
        repeat (6) @(posedge ft601_clk_in);
        mon_reset;

        @(posedge ft601_clk_in);
        range_profile       = 32'hFACE_FEED;
        range_valid         = 1;
        doppler_real        = 16'h1111;
        doppler_imag        = 16'h2222;
        doppler_valid       = 1;
        cfar_detection      = 1'b1;
        cfar_detect_range   = 6'd10;
        cfar_detect_doppler = 5'd5;
        cfar_detect_mag     = 17'd2000;
        cfar_detect_thr     = 17'd1000;
        cfar_valid          = 1;
        @(posedge ft601_clk_in);
        range_valid   = 0;
        doppler_valid = 0;
        cfar_valid    = 0;

        // Wait for packets to complete
        wait_packet_done(200);
        repeat (2) @(posedge ft601_clk_in);
        wait_packet_done(200);
        repeat (2) @(posedge ft601_clk_in);
        mon_stop;

        // Should get 10 words: range(6) + cfar(4), no doppler
        check_val(mon_count == 10, "Discard mix: 10 words (6+4, no doppler)",
                  mon_count, 10);
        check_val(mon_buf[0] === {24'h0, 8'hAA},
                  "Discard mix: first header = 0xAA (range)",
                  mon_buf[0], {24'h0, 8'hAA});
        check_val(mon_buf[6] === {24'h0, 8'hDD},
                  "Discard mix: second header = 0xDD (cfar, doppler skipped)",
                  mon_buf[6], {24'h0, 8'hDD});
        check(uut.doppler_data_pending === 1'b0,
              "Discard mix: doppler pending was discarded");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 7: Pending Flag Exports (v9d)
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 7: Pending Flag Exports ---");
        apply_reset;
        ft601_txe = 1;  // Backpressure to stall at HDR state

        // Pulse range_valid — pending goes high, then IDLE consumes
        // it and transitions to SEND_RANGE_HDR, but stalls there
        // because ft601_txe_r=1. The pending flag is cleared in IDLE
        // transition, so we must check BEFORE the IDLE arbiter fires.
        //
        // Actually, the pending flag is cleared INSIDE the IDLE case
        // (RTL line 477: range_data_pending <= 1'b0 when going to
        // SEND_RANGE_HDR). So by the time we check, it's already 0.
        //
        // Better test: check that the EXPORT wire matches internal reg.
        // We can verify by hierarchical access.
        @(posedge ft601_clk_in);
        range_profile = 32'hABCD_EF01;
        range_valid   = 1;
        @(posedge ft601_clk_in);
        range_valid = 0;
        // At this point, range_data_pending was just set. On the NEXT
        // posedge, IDLE will consume it. Check immediately.
        #1;
        check(range_pending_out === uut.range_data_pending,
              "Pending export: range_pending_out matches internal flag");

        // Verify the export goes to 0 after consumption
        wait_for_state(S_SEND_RANGE_HDR, 10);
        repeat (2) @(posedge ft601_clk_in); #1;
        check(range_pending_out === 1'b0,
              "Pending export: range_pending_out=0 after IDLE consumes");

        // Release backpressure, let packet complete
        ft601_txe = 0;
        wait_idle(200);

        // Same test for doppler
        ft601_txe = 1;
        @(posedge ft601_clk_in);
        doppler_valid = 1;
        @(posedge ft601_clk_in);
        doppler_valid = 0;
        #1;
        check(doppler_pending_out === uut.doppler_data_pending,
              "Pending export: doppler_pending_out matches internal flag");

        ft601_txe = 0;
        wait_idle(200);

        // Same test for cfar
        ft601_txe = 1;
        @(posedge ft601_clk_in);
        cfar_valid = 1;
        cfar_detection = 1'b1;
        @(posedge ft601_clk_in);
        cfar_valid = 0;
        #1;
        check(cfar_pending_out === uut.cfar_data_pending,
              "Pending export: cfar_pending_out matches internal flag");

        ft601_txe = 0;
        wait_idle(200);

        // ════════════════════════════════════════════════════════
        // TEST GROUP 8: Backpressure (ft601_txe stall)
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 8: Backpressure ---");
        apply_reset;
        ft601_txe = 1;  // FIFO full

        pulse_range_valid(32'hBBBB_CCCC);

        wait_for_state(S_SEND_RANGE_HDR, 50);
        repeat (10) @(posedge ft601_clk_in); #1;

        check(uut.current_state === S_SEND_RANGE_HDR,
              "Backpressure: stalled in SEND_RANGE_HDR when ft601_txe=1");
        check(ft601_wr_n === 1'b1,
              "Backpressure: ft601_wr_n=1 during stall");

        ft601_txe = 0;
        repeat (3) @(posedge ft601_clk_in); #1;

        check(uut.current_state !== S_SEND_RANGE_HDR,
              "Backpressure: resumed from SEND_RANGE_HDR after release");

        wait_idle(200);
        #1;
        check(uut.current_state === S_IDLE,
              "Backpressure: packet completed after resume");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 9: Clock Forwarding
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 9: Clock Forwarding ---");
        apply_reset;
        repeat (2) @(posedge ft601_clk_in);

        begin : clk_fwd_block
            integer match_count;
            match_count = 0;
            repeat (20) begin
                @(posedge ft601_clk_in); #1;
                if (ft601_clk_out === 1'b1)
                    match_count = match_count + 1;
            end
            check(match_count === 20,
                  "ft601_clk_out follows ft601_clk_in (forwarded clock)");
        end

        // ════════════════════════════════════════════════════════
        // TEST GROUP 10: Bus Release (IDLE and WAIT_ACK)
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 10: Bus Release ---");
        apply_reset;
        #1;

        check(uut.ft601_data_oe === 1'b0,
              "Bus release: ft601_data_oe=0 in IDLE");
        check(ft601_data === 32'h0000_0000,
              "Bus release: ft601_data=0 in IDLE (pulldown)");

        ft601_txe = 0;
        pulse_range_valid(32'h1111_2222);
        wait_for_state(S_WAIT_ACK, 200);
        #1;
        check(uut.ft601_data_oe === 1'b0,
              "Bus release: ft601_data_oe=0 in WAIT_ACK");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 11: Multiple Consecutive Packets
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 11: Multiple Consecutive Packets ---");
        apply_reset;
        ft601_txe = 0;

        pulse_range_valid(32'hAAAA_BBBB);
        wait_packet_done(200);
        #1;
        check(uut.current_state === S_IDLE,
              "Multi-packet: range 1 complete");

        repeat (4) @(posedge ft601_clk_in);

        pulse_range_valid(32'hCCCC_DDDD);
        wait_packet_done(200);
        #1;
        check(uut.current_state === S_IDLE,
              "Multi-packet: range 2 complete");
        check(uut.range_profile_cap === 32'hCCCC_DDDD,
              "Multi-packet: range 2 data captured correctly");

        pulse_doppler_valid(16'h1111, 16'h2222, 6'd5, 5'd10, 1'b0);
        wait_packet_done(200);
        #1;
        check(uut.current_state === S_IDLE,
              "Multi-packet: doppler complete");

        pulse_cfar_valid(1'b1, 6'd7, 5'd3, 17'd500, 17'd250);
        wait_packet_done(200);
        #1;
        check(uut.current_state === S_IDLE,
              "Multi-packet: cfar complete");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 12: Read Path - Single Command
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 12: Read Path - Single Command ---");
        apply_reset;

        send_host_command(8'h01, 8'h00, 16'h0002);

        check(cmd_opcode === 8'h01,
              "Read path: cmd_opcode=0x01 (set mode)");
        check(cmd_addr === 8'h00,
              "Read path: cmd_addr=0x00");
        check(cmd_value === 16'h0002,
              "Read path: cmd_value=0x0002 (single-chirp mode)");
        check(uut.read_state === 3'd0,
              "Read path: FSM returned to RD_IDLE");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 13: Read Path - Multiple Commands
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 13: Read Path - Multiple Commands ---");
        apply_reset;

        send_host_command(8'h01, 8'h00, 16'h0001);
        check(cmd_opcode === 8'h01,
              "Multi-cmd 1: opcode=0x01 (set mode)");
        check(cmd_value === 16'h0001,
              "Multi-cmd 1: value=0x0001 (auto-scan)");

        send_host_command(8'h02, 8'h00, 16'h0000);
        check(cmd_opcode === 8'h02,
              "Multi-cmd 2: opcode=0x02 (trigger)");

        send_host_command(8'h03, 8'h00, 16'h1234);
        check(cmd_opcode === 8'h03,
              "Multi-cmd 3: opcode=0x03 (CFAR threshold)");
        check(cmd_value === 16'h1234,
              "Multi-cmd 3: value=0x1234");

        send_host_command(8'h04, 8'h00, 16'h0005);
        check(cmd_opcode === 8'h04,
              "Multi-cmd 4: opcode=0x04 (stream control)");
        check(cmd_value === 16'h0005,
              "Multi-cmd 4: value=0x0005 (range+cfar)");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 14: Read/Write Interleave
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 14: Read/Write Interleave ---");
        apply_reset;
        ft601_txe = 1;  // Backpressure stall

        pulse_range_valid(32'hFACE_FEED);
        wait_for_state(S_SEND_RANGE_HDR, 50);
        repeat (2) @(posedge ft601_clk_in); #1;

        ft601_rxf = 0;
        repeat (3) @(posedge ft601_clk_in); #1;

        check(uut.read_state === 3'd0,
              "Interleave: Read FSM stays in RD_IDLE while write active");

        ft601_rxf = 1;
        ft601_txe = 0;
        wait_idle(200);
        @(posedge ft601_clk_in); #1;

        check(uut.current_state === S_IDLE,
              "Interleave: write packet completed");

        send_host_command(8'h01, 8'h00, 16'h0002);
        check(cmd_opcode === 8'h01,
              "Interleave: read after write cmd_opcode=0x01");
        check(cmd_value === 16'h0002,
              "Interleave: read after write cmd_value=0x0002");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 15: Status Readback (v9c: 10-word format)
        //
        // [0xBB] [word0..word7] [0x55] = 10 words
        //
        // NOTE: status_request uses EDGE detection, not level.
        // The RTL does: status_req_edge = status_request && !status_req_prev
        // Since both clk and ft601_clk_in are the same in this TB,
        // we pulse status_request for 1 ft601_clk cycle.
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 15: Status Readback ---");
        apply_reset;
        ft601_txe = 0;
        mon_reset;

        // Set known status input values
        status_cfar_threshold  = 16'hABCD;
        status_stream_ctrl     = 3'b101;
        status_radar_mode      = 2'b01;
        status_long_chirp      = 16'd3000;
        status_long_listen     = 16'd13700;
        status_guard           = 16'd17540;
        status_short_chirp     = 16'd50;
        status_short_listen    = 16'd17450;
        status_chirps_per_elev = 6'd32;
        status_range_mode      = 2'b10;
        status_self_test_flags  = 5'b11111;
        status_self_test_detail = 8'hA5;
        status_self_test_busy   = 1'b0;
        cfar_dbg_cells_processed = 16'd1024;
        cfar_dbg_cols_completed  = 8'd32;
        cfar_dbg_valid_count     = 16'd2048;
        cfar_detect_count        = 16'd112;

        // Let inputs settle for 1 cycle before pulse
        @(posedge ft601_clk_in);

        // Pulse status_request: set BETWEEN edges so the DUT samples
        // req=0 on one posedge (captured to prev) then req=1 on the
        // next posedge, producing the rising edge the RTL expects.
        #2; // mid-cycle — after posedge has sampled req=0
        status_request = 1;
        @(posedge ft601_clk_in); // DUT sees req=1, prev=0 → edge!
        @(posedge ft601_clk_in); // DUT latches prev=1
        #2;
        status_request = 0;

        // Wait for status packet to complete
        wait_for_state(S_SEND_STATUS, 30);
        wait_idle(200);
        repeat (2) @(posedge ft601_clk_in);
        mon_stop;

        check_val(mon_count == 10, "Status readback: 10 words captured",
                  mon_count, 10);
        check_val(mon_buf[0] === {24'h000000, 8'hBB},
                  "Status readback: word 0 = header 0xBB",
                  mon_buf[0], {24'h000000, 8'hBB});

        // status_words[0] = {0xFF, 3'b000, mode[1:0], 5'b00000, stream_ctrl[2:0], threshold[15:0]}
        begin : status_w0_check
            reg [31:0] exp_w0;
            exp_w0 = {8'hFF, 3'b000, 2'b01, 5'b00000, 3'b101, 16'hABCD};
            check_val(mon_buf[1] === exp_w0,
                      "Status readback: word 1 = status_words[0]",
                      mon_buf[1], exp_w0);
        end

        check_val(mon_buf[2] === {16'd3000, 16'd13700},
                  "Status readback: word 2 = {long_chirp, long_listen}",
                  mon_buf[2], {16'd3000, 16'd13700});
        check_val(mon_buf[3] === {16'd17540, 16'd50},
                  "Status readback: word 3 = {guard, short_chirp}",
                  mon_buf[3], {16'd17540, 16'd50});
        check_val(mon_buf[4] === {16'd17450, 10'd0, 6'd32},
                  "Status readback: word 4 = {short_listen, 0, chirps_per_elev}",
                  mon_buf[4], {16'd17450, 10'd0, 6'd32});
        check_val(mon_buf[5] === {30'd0, 2'b10},
                  "Status readback: word 5 = range_mode=2'b10",
                  mon_buf[5], {30'd0, 2'b10});

        begin : status_w5_check
            reg [31:0] exp_w5;
            exp_w5 = {7'd0, 1'b0, 8'd0, 8'hA5, 3'd0, 5'b11111};
            check_val(mon_buf[6] === exp_w5,
                      "Status readback: word 6 = self-test {busy=0,detail=A5,flags=1F}",
                      mon_buf[6], exp_w5);
        end

        // v9c words 6-7
        begin : status_w6_check
            reg [31:0] exp_w6;
            exp_w6 = {16'd1024, 8'd32, 8'd0};
            check_val(mon_buf[7] === exp_w6,
                      "Status readback: word 7 = {cfar_cells, cfar_cols, 0}",
                      mon_buf[7], exp_w6);
        end

        begin : status_w7_check
            reg [31:0] exp_w7;
            exp_w7 = {16'd112, 16'd2048};
            check_val(mon_buf[8] === exp_w7,
                      "Status readback: word 8 = {detect_count, valid_count}",
                      mon_buf[8], exp_w7);
        end

        check_val(mon_buf[9] === {24'h000000, 8'h55},
                  "Status readback: word 9 = footer 0x55",
                  mon_buf[9], {24'h000000, 8'h55});

        check(uut.current_state === S_IDLE,
              "Status readback: returned to IDLE");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 16: Status Priority Over Data
        //
        // Status request should be serviced before pending data.
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 16: Status Priority Over Data ---");
        apply_reset;
        ft601_txe = 0;
        mon_reset;

        // Pulse range_valid AND status_request simultaneously.
        // Set between edges so the DUT sees a clean 0→1 transition.
        // range_valid must be held for a full clock cycle so the DUT
        // captures it into range_data_pending.
        @(posedge ft601_clk_in);
        #2;
        range_profile  = 32'h1234_5678;
        range_valid    = 1;
        status_request = 1;
        @(posedge ft601_clk_in);  // DUT samples both: req edge + range_valid
        #2;
        range_valid    = 0;       // deassert mid-cycle after DUT has sampled
        @(posedge ft601_clk_in);  // DUT latches status_req_prev=1
        #2;
        status_request = 0;

        // Wait for both packets to complete
        // Status(10) + Range(6) = 16 words
        wait_packet_done(300);
        repeat (2) @(posedge ft601_clk_in);
        wait_packet_done(200);
        repeat (2) @(posedge ft601_clk_in);
        mon_stop;

        // First header should be status (0xBB) — higher priority
        check_val(mon_buf[0] === {24'h0, 8'hBB},
                  "Status priority: first header = 0xBB (status)",
                  mon_buf[0], {24'h0, 8'hBB});
        // After 10-word status, range header at index 10
        check_val(mon_buf[10] === {24'h0, 8'hAA},
                  "Status priority: second header = 0xAA (range after status)",
                  mon_buf[10], {24'h0, 8'hAA});
        check_val(mon_count == 16, "Status priority: 16 total words (10+6)",
                  mon_count, 16);

        // ════════════════════════════════════════════════════════
        // TEST GROUP 17: Chirp Timing Opcodes (read path)
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 17: Chirp Timing Opcodes ---");
        apply_reset;

        send_host_command(8'h10, 8'h00, 16'd2500);
        check(cmd_opcode === 8'h10,
              "Chirp opcode: 0x10 (long chirp cycles)");
        check(cmd_value === 16'd2500,
              "Chirp opcode: value=2500");

        send_host_command(8'h11, 8'h00, 16'd12000);
        check(cmd_opcode === 8'h11,
              "Chirp opcode: 0x11 (long listen cycles)");
        check(cmd_value === 16'd12000,
              "Chirp opcode: value=12000");

        send_host_command(8'h12, 8'h00, 16'd15000);
        check(cmd_opcode === 8'h12,
              "Chirp opcode: 0x12 (guard cycles)");
        check(cmd_value === 16'd15000,
              "Chirp opcode: value=15000");

        send_host_command(8'h13, 8'h00, 16'd40);
        check(cmd_opcode === 8'h13,
              "Chirp opcode: 0x13 (short chirp cycles)");
        check(cmd_value === 16'd40,
              "Chirp opcode: value=40");

        send_host_command(8'h14, 8'h00, 16'd16000);
        check(cmd_opcode === 8'h14,
              "Chirp opcode: 0x14 (short listen cycles)");
        check(cmd_value === 16'd16000,
              "Chirp opcode: value=16000");

        send_host_command(8'h15, 8'h00, 16'd16);
        check(cmd_opcode === 8'h15,
              "Chirp opcode: 0x15 (chirps per elevation)");
        check(cmd_value === 16'd16,
              "Chirp opcode: value=16");

        send_host_command(8'hFF, 8'h00, 16'h0000);
        check(cmd_opcode === 8'hFF,
              "Chirp opcode: 0xFF (status request)");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 18: Self-Test Readback Variants
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 18: Self-Test Readback Variants ---");
        apply_reset;
        ft601_txe = 0;

        status_self_test_flags  = 5'b10110;
        status_self_test_detail = 8'h42;
        status_self_test_busy   = 1'b1;

        // Set between edges for clean 0→1 edge detection
        @(posedge ft601_clk_in);
        #2;
        status_request = 1;
        @(posedge ft601_clk_in);
        @(posedge ft601_clk_in);
        #2;
        status_request = 0;

        wait_for_state(S_SEND_STATUS, 30);
        #1;
        check(uut.current_state === S_SEND_STATUS,
              "Self-test readback: FSM entered SEND_STATUS");

        wait_idle(200);
        #1;
        check(uut.current_state === S_IDLE,
              "Self-test readback: returned to IDLE");

        begin : selftest_check
            reg [31:0] exp;
            exp = {7'd0, 1'b1, 8'd0, 8'h42, 3'd0, 5'b10110};
            check_val(uut.status_words[5] === exp,
                      "Self-test readback: word 5 = {busy=1,detail=42,flags=16}",
                      uut.status_words[5], exp);
        end

        // ════════════════════════════════════════════════════════
        // TEST GROUP 19: Write-Idle Indicator
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 19: Write-Idle Indicator ---");
        apply_reset;
        #1;

        check(write_idle === 1'b1,
              "Write-idle: high when FSM idle after startup");

        ft601_txe = 0;
        pulse_range_valid(32'h9999_8888);
        wait_for_state(S_SEND_RANGE_HDR, 20);
        #1;
        check(write_idle === 1'b0,
              "Write-idle: low during packet transmission");

        wait_idle(200);
        repeat (2) @(posedge ft601_clk_in); #1;
        check(write_idle === 1'b1,
              "Write-idle: high again after packet complete");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 20: Debug Counter Verification
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 20: Debug Counter Verification ---");
        apply_reset;
        ft601_txe = 0;

        begin : dbg_counter_block
            reg [15:0] starts_before, completions_before;
            starts_before      = dbg_pkt_starts;
            completions_before = dbg_pkt_completions;

            pulse_range_valid(32'h1234_5678);
            wait_packet_done(200);
            repeat (2) @(posedge ft601_clk_in); #1;

            check_val(dbg_pkt_starts === starts_before + 16'd1,
                      "Debug counter: pkt_starts incremented by 1",
                      dbg_pkt_starts, starts_before + 16'd1);
            check_val(dbg_pkt_completions === completions_before + 16'd1,
                      "Debug counter: pkt_completions incremented by 1",
                      dbg_pkt_completions, completions_before + 16'd1);
        end

        // ════════════════════════════════════════════════════════
        // TEST GROUP 21: Doppler Packet with Different Metadata
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 21: Doppler Metadata Variants ---");
        apply_reset;
        ft601_txe = 0;

        // Test with max bin values: range_bin=63, doppler_bin=31, sub_frame=0
        mon_reset;
        pulse_doppler_valid(16'hFFFF, 16'h8000, 6'd63, 5'd31, 1'b0);
        wait_packet_done(200);
        repeat (2) @(posedge ft601_clk_in);
        mon_stop;

        begin : doppler_max_check
            reg [31:0] exp_w1;
            exp_w1 = {6'd63, 5'd31, 1'b0, 4'b0000, 16'hFFFF};
            check_val(mon_buf[1] === exp_w1,
                      "Doppler max bins: word 1 metadata correct",
                      mon_buf[1], exp_w1);
            check_val(mon_buf[2] === {16'h8000, 16'hFFFF},
                      "Doppler max bins: word 2 = {Q, I}",
                      mon_buf[2], {16'h8000, 16'hFFFF});
        end

        // Test with zero bins
        mon_reset;
        pulse_doppler_valid(16'h0000, 16'h0000, 6'd0, 5'd0, 1'b0);
        wait_packet_done(200);
        repeat (2) @(posedge ft601_clk_in);
        mon_stop;

        begin : doppler_zero_check
            reg [31:0] exp_w1;
            exp_w1 = {6'd0, 5'd0, 1'b0, 4'b0000, 16'h0000};
            check_val(mon_buf[1] === exp_w1,
                      "Doppler zero bins: word 1 metadata correct",
                      mon_buf[1], exp_w1);
        end

        // ════════════════════════════════════════════════════════
        // TEST GROUP 22: CFAR Packet with Edge Values
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 22: CFAR Edge Values ---");
        apply_reset;
        ft601_txe = 0;

        mon_reset;
        pulse_cfar_valid(1'b1, 6'd63, 5'd31, 17'd131071, 17'd131071);
        wait_packet_done(200);
        repeat (2) @(posedge ft601_clk_in);
        mon_stop;

        begin : cfar_max_check
            reg [31:0] exp_w1, exp_w2;
            exp_w1 = {1'b1, 6'd63, 5'd31, 3'b000, 17'd131071};
            exp_w2 = {15'b0, 17'd131071};
            check_val(mon_buf[1] === exp_w1,
                      "CFAR max values: word 1 correct",
                      mon_buf[1], exp_w1);
            check_val(mon_buf[2] === exp_w2,
                      "CFAR max values: word 2 = max threshold",
                      mon_buf[2], exp_w2);
        end

        // ════════════════════════════════════════════════════════
        // TEST GROUP 23: Startup Lockout
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 23: Startup Lockout ---");

        // Custom reset: don't wait for lockout to expire
        mon_active = 0;
        reset_n = 0;
        range_profile = 32'h0;
        range_valid   = 0;
        stream_control = 3'b111;
        ft601_txe = 0;
        ft601_rxf = 1;
        repeat (6) @(posedge ft601_clk_in);
        reset_n = 1;
        repeat (6) @(posedge ft601_clk_in);  // stream_control CDC

        // Pulse range_valid during lockout
        @(posedge ft601_clk_in);
        range_profile = 32'hA0CE_0001;
        range_valid   = 1;
        @(posedge ft601_clk_in);
        range_valid = 0;
        repeat (10) @(posedge ft601_clk_in); #1;

        check(uut.current_state === S_IDLE,
              "Startup lockout: FSM stays IDLE during lockout");

        // Wait for lockout to expire
        repeat (250) @(posedge ft601_clk_in);

        // The pending range data should now be consumed
        wait_for_state(S_SEND_RANGE_HDR, 20);
        wait_idle(200);
        #1;
        check(uut.current_state === S_IDLE,
              "Startup lockout: pending data consumed after lockout expires");

        // ════════════════════════════════════════════════════════
        // Summary
        // ════════════════════════════════════════════════════════
        $display("");
        $display("========================================");
        $display("  USB DATA INTERFACE TESTBENCH RESULTS");
        $display("  PASSED: %0d / %0d", pass_count, test_num);
        $display("  FAILED: %0d / %0d", fail_count, test_num);
        if (fail_count == 0)
            $display("  ** ALL TESTS PASSED **");
        else
            $display("  ** SOME TESTS FAILED **");
        $display("========================================");
        $display("");

        $fclose(csv_file);
        #100;
        $finish;
    end

endmodule
