`timescale 1ns / 1ps
`include "radar_params.vh"

// ============================================================================
// tb_e2e_dsp_to_host.v
//
// PR-Z A6 — End-to-end DSP-to-host integration test.
//
// Drives the back-half of the radar pipeline (range_decim_in -> Doppler ->
// DC-notch -> CFAR -> USB pack -> FT2232H egress) with a deterministic
// single-target stimulus and asserts every stage transition against
// Python-computed expected values produced by:
//
//   tb/cosim/gen_e2e_stimulus.py     (range_decim_packed.hex)
//   tb/cosim/gen_e2e_expected.py     (expected_*.hex + expected_frame.bin)
//
// Replaces the 0-assertion radar_system_tb + USB_MODE=1 smoke (~5 min wall)
// with one TB that catches:
//   * Doppler FFT axis flips, sub-frame indexing (E1-E3)
//   * S-1 DC-notch off-by-one (E4)
//   * 2-tier CFAR class encoding (E5-E6)
//   * USB header layout drift (E7)
//   * M-8 byte 2 subframe_enable hard-tie / mask flip (E8)
//   * Magnitude row endianness/byte-ordering (E9)
//   * Detect-map 2-bit packing (E10)
//   * Footer placement (E11)
//   * Python parse round-trip (E12 — runs separately as
//     tb_e2e_dsp_to_host_parse.py)
//
// Compile:
//   iverilog -g2012 -DSIMULATION -o tb/tb_e2e_dsp_to_host_reg.vvp \
//     tb/tb_e2e_dsp_to_host.v doppler_processor.v xfft_16.v fft_engine.v \
//     mti_canceller.v cfar_ca.v usb_data_interface_ft2232h.v \
//     edge_detector.v cdc_modules.v cdc_async_fifo.v
//
// Run:  vvp tb/tb_e2e_dsp_to_host_reg.vvp
// ============================================================================

module tb_e2e_dsp_to_host;

    // ====================================================================
    // PARAMETERS — must align with gen_e2e_stimulus.py / gen_e2e_expected.py
    // ====================================================================
    localparam CLK_PERIOD       = 10.0;           // 100 MHz
    localparam FT_CLK_PERIOD    = 16.667;         // 60 MHz
    localparam CHIRPS           = 48;
    localparam RANGE_BINS       = 512;
    localparam DOPPLER_TOTAL    = 48;
    localparam STIM_LEN         = CHIRPS * RANGE_BINS;          // 24576
    localparam DOPPLER_OUT_LEN  = RANGE_BINS * DOPPLER_TOTAL;   // 24576
    localparam EXPECTED_FRAME_BYTES = 55306;                    // gen_e2e_expected.py

    // Test config (mirrors gen_e2e_expected.py).
    localparam [2:0] TEST_HOST_DC_NOTCH_WIDTH = 3'd1;
    localparam [5:0] TEST_STREAM_CONTROL      = 6'b000_110;     // doppler+cfar
    localparam [2:0] TEST_SUBFRAME_ENABLE     = 3'b101;         // LONG|SHORT (drop MEDIUM)
    localparam [7:0] TEST_FLAGS_BYTE          = 8'h2E;          // (sf<<3)|stream

    // CFAR config (matches gen_e2e_expected.py + RP_DEF_CFAR_*).
    localparam [3:0] TEST_CFAR_GUARD = 4'd2;
    localparam [4:0] TEST_CFAR_TRAIN = 5'd8;
    localparam [7:0] TEST_CFAR_ALPHA      = 8'h30;
    localparam [7:0] TEST_CFAR_ALPHA_SOFT = 8'h18;
    localparam [1:0] TEST_CFAR_MODE       = 2'b00;              // CA
    localparam       TEST_CFAR_ENABLE     = 1'b1;
    localparam [15:0] TEST_CFAR_SIMPLE_THR = 16'd0;             // unused when CFAR enabled

    // ====================================================================
    // CLOCKS + RESET
    // ====================================================================
    reg clk_100m   = 1'b0;
    reg ft_clk     = 1'b0;
    reg reset_n    = 1'b0;
    reg ft_reset_n = 1'b0;

    always #(CLK_PERIOD / 2.0)    clk_100m = ~clk_100m;
    always #(FT_CLK_PERIOD / 2.0) ft_clk   = ~ft_clk;

    // ====================================================================
    // STIMULUS / GOLDEN MEMS (loaded by $readmemh)
    // ====================================================================
    reg [31:0] stim_mem [0:STIM_LEN-1];
    reg signed [15:0] expected_doppler_raw_i [0:DOPPLER_OUT_LEN-1];
    reg signed [15:0] expected_doppler_raw_q [0:DOPPLER_OUT_LEN-1];
    reg signed [15:0] expected_doppler_notched_i [0:DOPPLER_OUT_LEN-1];
    reg signed [15:0] expected_doppler_notched_q [0:DOPPLER_OUT_LEN-1];
    reg [1:0]  expected_cfar_class [0:DOPPLER_OUT_LEN-1];

    initial begin
        $readmemh("tb/cosim/e2e_data/range_decim_packed.hex", stim_mem);
        $readmemh("tb/cosim/e2e_data/expected_doppler_raw_i.hex", expected_doppler_raw_i);
        $readmemh("tb/cosim/e2e_data/expected_doppler_raw_q.hex", expected_doppler_raw_q);
        $readmemh("tb/cosim/e2e_data/expected_doppler_notched_i.hex", expected_doppler_notched_i);
        $readmemh("tb/cosim/e2e_data/expected_doppler_notched_q.hex", expected_doppler_notched_q);
        $readmemh("tb/cosim/e2e_data/expected_cfar_class.hex", expected_cfar_class);
    end

    // ====================================================================
    // FAITHFUL PRODUCTION WIRING
    //
    // range_decim -> mti_canceller -> doppler_processor -> [DC notch] ->
    //                                                       \-> cfar_ca
    //                                                        \-> edge_detect
    //                                                                 (frame_complete level -> 1-cyc pulse)
    //                                                                 -> usb_data_interface_ft2232h
    //
    // Mirrors radar_receiver_final.v lines 583-665. The edge detector is the
    // [RX-E FIX] inline pattern (init prev=1 to suppress reset-glitch pulse).
    // ====================================================================

    // ---- Stimulus driver (range_decim_in level) ----
    reg signed [15:0] mti_in_i;
    reg signed [15:0] mti_in_q;
    reg               mti_in_valid;
    reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] mti_in_range_bin;
    reg [1:0]         mti_wave_sel;     // 0=SHORT, 1=MEDIUM, 2=LONG
    reg               new_chirp_frame;

    // ---- MTI canceller (production default mti_enable=1) ----
    wire signed [15:0]                       mti_out_i;
    wire signed [15:0]                       mti_out_q;
    wire                                     mti_out_valid;
    wire [`RP_RANGE_BIN_WIDTH_MAX-1:0]       mti_out_range_bin;
    wire                                     mti_first_chirp;
    wire [7:0]                               mti_saturation_count;

    // mti_enable=0 matches production cold-reset default (radar_system_top.v:1096
    // `host_mti_enable <= 1'b0`). With MTI disabled the module is a transparent
    // pass-through (1-cycle pipeline delay only) — exercises the integration
    // wiring without changing the data. To test mti_enable=1 behaviour, drive
    // host_mti_enable via opcode 0x26 in a future test variant.
    mti_canceller #(
        .NUM_RANGE_BINS(`RP_MAX_OUTPUT_BINS),
        .DATA_WIDTH    (`RP_DATA_WIDTH)
    ) u_mti (
        .clk           (clk_100m),
        .reset_n       (reset_n),
        .range_i_in    (mti_in_i),
        .range_q_in    (mti_in_q),
        .range_valid_in(mti_in_valid),
        .range_bin_in  (mti_in_range_bin),
        .range_i_out   (mti_out_i),
        .range_q_out   (mti_out_q),
        .range_valid_out(mti_out_valid),
        .range_bin_out (mti_out_range_bin),
        .mti_enable    (1'b0),                  // production cold-reset default
        .wave_sel      (mti_wave_sel),
        .mti_first_chirp(mti_first_chirp),
        .mti_saturation_count(mti_saturation_count)
    );

    // Repack {Q, I} for doppler_processor (matches radar_receiver_final.v:635)
    wire [31:0] doppler_input_data = {mti_out_q, mti_out_i};

    // ---- DOPPLER PROCESSOR ----
    wire [31:0]                              doppler_output;
    wire                                     doppler_valid;
    wire [`RP_DOPPLER_BIN_WIDTH-1:0]         doppler_bin;
    wire [`RP_RANGE_BIN_WIDTH_MAX-1:0]       range_bin_out;
    wire                                     doppler_frame_complete_level;
    wire [3:0]                               doppler_status;

    doppler_processor_optimized #(
        .CHIRPS_PER_FRAME(48),
        .CHIRPS_PER_SUBFRAME(16),
        .RANGE_BINS(RANGE_BINS)
    ) u_doppler (
        .clk            (clk_100m),
        .reset_n        (reset_n),
        .range_data     (doppler_input_data),
        .data_valid     (mti_out_valid),
        .new_chirp_frame(new_chirp_frame),
        .doppler_output (doppler_output),
        .doppler_valid  (doppler_valid),
        .doppler_bin    (doppler_bin),
        .range_bin      (range_bin_out),
        .sub_frame      (),
        .processing_active(),
        .frame_complete (doppler_frame_complete_level),
        .status         (doppler_status)
    );

    // ---- Inline edge detector (radar_receiver_final.v:191-208 [RX-E FIX]) ----
    // doppler_frame_complete_level is HIGH at reset (state=S_IDLE, empty buffer);
    // initialize prev=1 so the rising-edge AND ~prev never asserts on the
    // first valid clk after reset.
    reg  doppler_frame_done_prev;
    wire doppler_frame_complete;
    always @(posedge clk_100m or negedge reset_n) begin
        if (!reset_n)
            doppler_frame_done_prev <= 1'b1;
        else
            doppler_frame_done_prev <= doppler_frame_complete_level;
    end
    assign doppler_frame_complete = doppler_frame_complete_level &
                                    ~doppler_frame_done_prev;

    // ====================================================================
    // DC NOTCH (combinational, post-S-1 inclusive comparators)
    // Production wiring: notched -> cfar_ca; RAW -> usb_data_interface
    // ====================================================================
    wire [3:0]  bin_within_sf = doppler_bin[3:0];
    wire [4:0]  notch_lo = {2'b00, TEST_HOST_DC_NOTCH_WIDTH};       // 0..7
    wire [4:0]  notch_hi = 5'd16 - notch_lo;                        // 9..16
    wire        dc_notch_active = (TEST_HOST_DC_NOTCH_WIDTH != 3'd0) &&
                                  ({1'b0, bin_within_sf} <= notch_lo ||
                                   {1'b0, bin_within_sf} >= notch_hi);
    wire [31:0] notched_doppler = dc_notch_active ? 32'd0 : doppler_output;

    // ====================================================================
    // CFAR (sees notched data — same as production)
    // ====================================================================
    wire                                  cfar_detect_flag;
    wire [`RP_DETECT_CLASS_WIDTH-1:0]     cfar_detect_class;
    wire                                  cfar_detect_valid;
    wire [`RP_RANGE_BIN_WIDTH_MAX-1:0]    cfar_detect_range;
    wire [`RP_DOPPLER_BIN_WIDTH-1:0]      cfar_detect_doppler;
    wire [16:0]                           cfar_detect_magnitude;
    wire [16:0]                           cfar_detect_threshold;
    wire [16:0]                           cfar_detect_threshold_soft;
    wire [15:0]                           cfar_detect_count;
    wire [15:0]                           cfar_detect_count_cand;
    wire                                  cfar_busy;
    wire [7:0]                            cfar_status;

    cfar_ca u_cfar (
        .clk                  (clk_100m),
        .reset_n              (reset_n),
        .doppler_data         (notched_doppler),
        .doppler_valid        (doppler_valid),
        .doppler_bin_in       (doppler_bin),
        .range_bin_in         (range_bin_out),
        .frame_complete       (doppler_frame_complete),
        .cfg_guard_cells      (TEST_CFAR_GUARD),
        .cfg_train_cells      (TEST_CFAR_TRAIN),
        .cfg_alpha            (TEST_CFAR_ALPHA),
        .cfg_alpha_soft       (TEST_CFAR_ALPHA_SOFT),
        .cfg_cfar_mode        (TEST_CFAR_MODE),
        .cfg_cfar_enable      (TEST_CFAR_ENABLE),
        .cfg_simple_threshold (TEST_CFAR_SIMPLE_THR),
        .detect_flag          (cfar_detect_flag),
        .detect_class         (cfar_detect_class),
        .detect_valid         (cfar_detect_valid),
        .detect_range         (cfar_detect_range),
        .detect_doppler       (cfar_detect_doppler),
        .detect_magnitude     (cfar_detect_magnitude),
        .detect_threshold     (cfar_detect_threshold),
        .detect_threshold_soft(cfar_detect_threshold_soft),
        .detect_count         (cfar_detect_count),
        .detect_count_cand    (cfar_detect_count_cand),
        .cfar_busy            (cfar_busy),
        .cfar_status          (cfar_status)
    );

    // ---- 1-cycle sync register CFAR -> USB ----
    // Mirrors radar_system_top.v:763-772 (rx_detect_*). Without this register
    // the cfar_valid pulse and the doppler_valid pulse for the same cell
    // arrive at usb_data_interface_ft2232h at different cycles, causing the
    // cfar BRAM RMW state machine to miss writes (E12.17 fail symptom).
    reg [`RP_DETECT_CLASS_WIDTH-1:0] cfar_class_reg;
    reg                              cfar_valid_reg;
    reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] cfar_range_reg;
    reg [`RP_DOPPLER_BIN_WIDTH-1:0]   cfar_doppler_reg;
    always @(posedge clk_100m or negedge reset_n) begin
        if (!reset_n) begin
            cfar_class_reg   <= `RP_DETECT_NONE;
            cfar_valid_reg   <= 1'b0;
            cfar_range_reg   <= {`RP_RANGE_BIN_WIDTH_MAX{1'b0}};
            cfar_doppler_reg <= {`RP_DOPPLER_BIN_WIDTH{1'b0}};
        end else begin
            cfar_class_reg   <= cfar_detect_class;
            cfar_valid_reg   <= cfar_detect_valid;
            cfar_range_reg   <= cfar_detect_range;
            cfar_doppler_reg <= cfar_detect_doppler;
        end
    end

    // ====================================================================
    // USB FT2232H EGRESS (sees RAW doppler + CFAR class)
    // ====================================================================
    wire [7:0] ft_data;
    reg        ft_rxf_n = 1'b1;
    reg        ft_txe_n = 1'b0;
    wire       ft_rd_n;
    wire       ft_wr_n;
    wire       ft_oe_n;
    wire       ft_siwu;

    // Bidirectional ft_data — TB drives nothing (no host commands in A6).
    pulldown pd[7:0] (ft_data);

    wire [31:0] cmd_data;
    wire        cmd_valid;
    wire [7:0]  cmd_opcode;
    wire [7:0]  cmd_addr;
    wire [15:0] cmd_value;

    usb_data_interface_ft2232h u_usb (
        .clk              (clk_100m),
        .reset_n          (reset_n),
        .ft_reset_n       (ft_reset_n),

        // Radar data inputs — mirrors radar_system_top.v production wiring at
        // line 818-824 + 920-955. After the Bug A + Bug B fix, the registered
        // cfar bins are muxed in when rx_detect_valid is high so the USB RMW
        // address tracks cfar's per-cell counters (not doppler's stale (511,47)
        // tail). See project_aeris10_usb_cfar_stale_bin_2026-05-05.md.
        .range_profile    (32'd0),
        .range_valid      (1'b0),
        .doppler_real     (doppler_output[15:0]),
        .doppler_imag     (doppler_output[31:16]),
        .doppler_valid    (doppler_valid),
        .cfar_detect_class(cfar_class_reg),
        .cfar_valid       (cfar_valid_reg),
        .range_bin_in     (cfar_valid_reg ? cfar_range_reg   : range_bin_out),
        .doppler_bin_in   (cfar_valid_reg ? cfar_doppler_reg : doppler_bin),
        .frame_complete   (doppler_frame_complete),

        // FT2232H bus
        .ft_data          (ft_data),
        .ft_rxf_n         (ft_rxf_n),
        .ft_txe_n         (ft_txe_n),
        .ft_rd_n          (ft_rd_n),
        .ft_wr_n          (ft_wr_n),
        .ft_oe_n          (ft_oe_n),
        .ft_siwu          (ft_siwu),
        .ft_clk           (ft_clk),

        // Host command bus (no commands in A6)
        .cmd_data         (cmd_data),
        .cmd_valid        (cmd_valid),
        .cmd_opcode       (cmd_opcode),
        .cmd_addr         (cmd_addr),
        .cmd_value        (cmd_value),

        // Stream + subframe_enable
        .stream_control   (TEST_STREAM_CONTROL),
        .subframe_enable  (TEST_SUBFRAME_ENABLE),

        // Status (tied off — A6 does not exercise opcode 0xFF)
        .status_request             (1'b0),
        .status_cfar_threshold      (16'd0),
        .status_stream_ctrl         (TEST_STREAM_CONTROL),
        .status_long_chirp          (16'd0),
        .status_long_listen         (16'd0),
        .status_guard               (16'd0),
        .status_short_chirp         (16'd0),
        .status_short_listen        (16'd0),
        // M-5: medium PRI readback ports (A6 doesn't exercise status reads;
        // tied off to keep DUT-port arity in sync).
        .status_medium_chirp        (16'd0),
        .status_medium_listen       (16'd0),
        .status_chirps_per_elev     (6'd0),
        .status_chirps_mismatch     (1'b0),
        .status_self_test_flags     (5'd0),
        .status_self_test_detail    (8'd0),
        .status_self_test_busy      (1'b0),
        .status_agc_current_gain    (4'd0),
        .status_agc_peak_magnitude  (8'd0),
        .status_agc_saturation_count(8'd0),
        .status_agc_enable          (1'b0),
        .status_range_decim_watchdog(1'b0),
        .status_ddc_cic_fir_overrun (1'b0),
        .status_beam_handshake_watchdog(1'b0),  // commit 5 — tied off, e2e path doesn't model handshake
        .status_cfar_alpha_soft     (TEST_CFAR_ALPHA_SOFT),
        .status_detect_threshold_soft(17'd0),
        .status_detect_count_cand   (16'd0)
    );

    // ====================================================================
    // DIAGNOSTIC COUNTERS (frame_filling, doppler/cfar pulses)
    // ====================================================================
    integer dop_valid_to_usb = 0;
    integer cfar_valid_to_usb = 0;
    integer frame_complete_pulses = 0;
    reg     prev_frame_complete = 1'b0;
    always @(posedge clk_100m) begin
        if (!reset_n) begin
            dop_valid_to_usb     <= 0;
            cfar_valid_to_usb    <= 0;
            frame_complete_pulses<= 0;
            prev_frame_complete  <= 1'b0;
        end else begin
            if (doppler_valid)         dop_valid_to_usb     <= dop_valid_to_usb + 1;
            if (cfar_detect_valid)     cfar_valid_to_usb    <= cfar_valid_to_usb + 1;
            // count frame_complete edges
            if (doppler_frame_complete && !prev_frame_complete)
                frame_complete_pulses <= frame_complete_pulses + 1;
            prev_frame_complete <= doppler_frame_complete;
        end
    end

    // ====================================================================
    // EGRESS CAPTURE (ft_clk domain)
    // ====================================================================
    // Buffer the full expected frame so we can compare byte-for-byte.
    reg [7:0] egress_bytes [0:EXPECTED_FRAME_BYTES + 16];
    integer    egress_count = 0;

    always @(posedge ft_clk) begin
        if (!ft_wr_n && !ft_txe_n) begin
            if (egress_count < EXPECTED_FRAME_BYTES + 16)
                egress_bytes[egress_count] <= ft_data;
            egress_count <= egress_count + 1;
        end
    end


    // ====================================================================
    // PASS / FAIL TRACKING
    // ====================================================================
    integer pass_count = 0;
    integer fail_count = 0;
    integer test_count = 0;

    task check_b;
        input [255:0] tag;
        input         cond;
        begin
            test_count = test_count + 1;
            if (cond) begin
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] %0s", tag);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // ====================================================================
    // MAIN TEST SEQUENCE
    // ====================================================================
    integer i;
    integer doppler_out_idx;
    integer cfar_capture_idx;
    reg signed [15:0] cap_doppler_i [0:DOPPLER_OUT_LEN-1];
    reg signed [15:0] cap_doppler_q [0:DOPPLER_OUT_LEN-1];
    reg [1:0]         cap_cfar_class [0:DOPPLER_OUT_LEN-1];

    initial begin
        // ---- Init ----
        mti_in_i         = 16'sd0;
        mti_in_q         = 16'sd0;
        mti_in_valid     = 1'b0;
        mti_in_range_bin = {`RP_RANGE_BIN_WIDTH_MAX{1'b0}};
        mti_wave_sel     = 2'd0;
        new_chirp_frame  = 1'b0;
        for (i = 0; i < DOPPLER_OUT_LEN; i = i + 1) begin
            cap_doppler_i[i]  = 16'sd0;
            cap_doppler_q[i]  = 16'sd0;
            cap_cfar_class[i] = 2'd0;
        end

        $display("============================================================");
        $display("  PR-Z A6 — End-to-End DSP-to-Host Integration Test");
        $display("  stimulus:  %0d cells (%0d chirps x %0d range)",
                 STIM_LEN, CHIRPS, RANGE_BINS);
        $display("  expected:  %0d-byte bulk frame (flags=0x%02h)",
                 EXPECTED_FRAME_BYTES, TEST_FLAGS_BYTE);
        $display("============================================================");

        // ---- Reset ----
        reset_n    = 1'b0;
        ft_reset_n = 1'b0;
        #(CLK_PERIOD * 20);
        reset_n    = 1'b1;
        ft_reset_n = 1'b1;
        #(CLK_PERIOD * 10);

        // ---- Pulse new_chirp_frame ----
        @(posedge clk_100m);
        new_chirp_frame <= 1'b1;
        @(posedge clk_100m);
        @(posedge clk_100m);
        new_chirp_frame <= 1'b0;
        @(posedge clk_100m);

        // ---- Drive stimulus stream into MTI ----
        // chirp-major: stim_mem[chirp*512 + range_bin] = {Q[31:16], I[15:0]}
        // wave_sel transitions at sub-frame boundaries (chirps 0/16/32) so
        // mti_canceller fires mti_first_chirp at each sub-frame start —
        // matching radar_receiver_final.v's chirp_scheduler-driven wave_sel.
        $display("\n--- Feeding %0d stimulus cells (chirp-major, MTI input) ---",
                 STIM_LEN);
        for (i = 0; i < STIM_LEN; i = i + 1) begin : drive_stim
            integer chirp_idx;
            integer range_idx;
            chirp_idx = i / RANGE_BINS;
            range_idx = i % RANGE_BINS;
            @(posedge clk_100m);
            // wave_sel: 0=SHORT (chirps 0..15), 1=MEDIUM (16..31), 2=LONG (32..47)
            mti_wave_sel     <= chirp_idx[5:4];
            mti_in_i         <= $signed(stim_mem[i][15:0]);
            mti_in_q         <= $signed(stim_mem[i][31:16]);
            mti_in_range_bin <= range_idx[`RP_RANGE_BIN_WIDTH_MAX-1:0];
            mti_in_valid     <= 1'b1;
        end
        @(posedge clk_100m);
        mti_in_valid     <= 1'b0;
        mti_in_i         <= 16'sd0;
        mti_in_q         <= 16'sd0;
        mti_in_range_bin <= {`RP_RANGE_BIN_WIDTH_MAX{1'b0}};

        // ---- Capture doppler + CFAR outputs concurrently ----
        // Doppler emits 24576 cells in range-major (rb 0 dbins 0..47, etc.).
        // CFAR starts emitting AFTER frame_complete; takes another ~24576
        // cycles to drain (one detect_valid pulse per (range,doppler) cell).
        // USB egress runs concurrently in ft_clk domain — captured below.
        // fork ... join (not join_any!) waits for BOTH capture phases to
        // complete; the top-level $60_000_000 watchdog handles overall timeout.
        doppler_out_idx  = 0;
        cfar_capture_idx = 0;
        fork
            begin : capture_doppler
                while (doppler_out_idx < DOPPLER_OUT_LEN) begin
                    @(posedge clk_100m);
                    if (doppler_valid) begin
                        cap_doppler_i[doppler_out_idx] = doppler_output[15:0];
                        cap_doppler_q[doppler_out_idx] = doppler_output[31:16];
                        doppler_out_idx                = doppler_out_idx + 1;
                    end
                end
            end
            begin : capture_cfar
                while (cfar_capture_idx < DOPPLER_OUT_LEN) begin
                    @(posedge clk_100m);
                    if (cfar_detect_valid) begin
                        // CFAR emits in (col=doppler, row=range) order. Index
                        // by flat range-major to align with expected_*.hex.
                        cap_cfar_class[cfar_detect_range * DOPPLER_TOTAL +
                                       cfar_detect_doppler] = cfar_detect_class;
                        cfar_capture_idx = cfar_capture_idx + 1;
                    end
                end
            end
        join

        $display("  doppler_out=%0d cfar=%0d (waiting for USB egress)",
                 doppler_out_idx, cfar_capture_idx);
        $display("  DIAG: dop_valid_to_usb=%0d  cfar_valid_to_usb=%0d  frame_complete_pulses=%0d",
                 dop_valid_to_usb, cfar_valid_to_usb, frame_complete_pulses);
        $display("        usb.frame_filling=%b  usb.frame_number=%0d",
                 u_usb.frame_filling, u_usb.frame_number);
        $display("  diag: cfar class at target cells:");
        $display("        (67,  2) = %0d", cap_cfar_class[67 * DOPPLER_TOTAL +  2]);
        $display("        (67, 18) = %0d", cap_cfar_class[67 * DOPPLER_TOTAL + 18]);
        $display("        (67, 34) = %0d", cap_cfar_class[67 * DOPPLER_TOTAL + 34]);
        $display("        (67,  0) = %0d (notched)", cap_cfar_class[67 * DOPPLER_TOTAL +  0]);
        $display("        (67, 16) = %0d (notched)", cap_cfar_class[67 * DOPPLER_TOTAL + 16]);

        // ---- Wait for USB egress to complete ----
        $display("\n--- Capturing %0d-byte egress frame ---", EXPECTED_FRAME_BYTES);
        wait (egress_count >= EXPECTED_FRAME_BYTES);
        $display("  egress_count = %0d", egress_count);

        // ====================================================================
        // ASSERTIONS — backed by Python-computed expected values
        // ====================================================================

        // ---- E1-E3: Doppler peaks at expected (range=67, doppler=2/18/34) ----
        //
        // Compare full doppler bus against expected_doppler_raw_*.hex
        // (gen_e2e_expected.py wrote raw, NOT notched, since the USB stream
        // sees raw — see comment near pack_bulk_frame). Allow ±1 LSB to
        // tolerate any benign rounding asymmetry vs. fpga_model.py.
        begin : doppler_compare
            integer mismatch_count;
            integer di, dq;
            integer ei, eq;
            mismatch_count = 0;
            for (i = 0; i < DOPPLER_OUT_LEN; i = i + 1) begin
                di = $signed(cap_doppler_i[i]);
                dq = $signed(cap_doppler_q[i]);
                ei = $signed(expected_doppler_raw_i[i]);
                eq = $signed(expected_doppler_raw_q[i]);
                if ((di > ei + 1) || (di < ei - 1) ||
                    (dq > eq + 1) || (dq < eq - 1)) begin
                    mismatch_count = mismatch_count + 1;
                    if (mismatch_count <= 5)
                        $display("    [doppler mismatch] idx=%0d RTL=(%0d,%0d) REF=(%0d,%0d)",
                                 i, di, dq, ei, eq);
                end
            end
            check_b("E1-E3: doppler bus matches expected (within +/-1 LSB)",
                    mismatch_count == 0);
            $display("    doppler_mismatches=%0d / %0d", mismatch_count,
                     DOPPLER_OUT_LEN);
        end

        // ---- E4: DC notch in CFAR-side data path ----
        // Bin 0 of every range/sub-frame should be NONE (notched -> 0 magnitude
        // -> never crosses CFAR threshold). Target lives at bin 2, well clear
        // of the W=1 notch, so its CFAR class should still be CONFIRMED.
        check_b("E4.a: CFAR class at (67, 2) is CONFIRMED (target outside notch)",
                cap_cfar_class[67 * DOPPLER_TOTAL + 2] == `RP_DETECT_CONFIRMED);
        check_b("E4.b: CFAR class at (67, 0) is NONE (DC bin notched)",
                cap_cfar_class[67 * DOPPLER_TOTAL + 0] == `RP_DETECT_NONE);
        check_b("E4.c: CFAR class at (67, 16) is NONE (sub-frame 1 DC bin notched)",
                cap_cfar_class[67 * DOPPLER_TOTAL + 16] == `RP_DETECT_NONE);

        // ---- E5: Three target cells must all be CONFIRMED ----
        check_b("E5.a: CFAR class at (67, 2)  = CONFIRMED",
                cap_cfar_class[67 * DOPPLER_TOTAL +  2] == `RP_DETECT_CONFIRMED);
        check_b("E5.b: CFAR class at (67, 18) = CONFIRMED",
                cap_cfar_class[67 * DOPPLER_TOTAL + 18] == `RP_DETECT_CONFIRMED);
        check_b("E5.c: CFAR class at (67, 34) = CONFIRMED",
                cap_cfar_class[67 * DOPPLER_TOTAL + 34] == `RP_DETECT_CONFIRMED);

        // ---- E6: Known-NONE neighbor cells (clean noise floor) ----
        check_b("E6.a: CFAR class at (60,  2) = NONE",
                cap_cfar_class[60 * DOPPLER_TOTAL +  2] == `RP_DETECT_NONE);
        check_b("E6.b: CFAR class at (75,  5) = NONE",
                cap_cfar_class[75 * DOPPLER_TOTAL +  5] == `RP_DETECT_NONE);
        check_b("E6.c: CFAR class at (200,10) = NONE",
                cap_cfar_class[200 * DOPPLER_TOTAL + 10] == `RP_DETECT_NONE);

        // ---- E7: USB header layout ----
        check_b("E7.1: byte0 = 0xAA (magic)",
                egress_bytes[0] == 8'hAA);
        check_b("E7.2: byte1 = 0x02 (version)",
                egress_bytes[1] == `RP_USB_PROTOCOL_VERSION);

        // ---- E8: byte 2 carries subframe_enable[5:3] | stream[2:0] ----
        // Catches the M-8/M-11 hard-tie class of bug — if subframe_enable is
        // mis-routed or hard-tied, this assertion trips.
        check_b("E8: byte2 = 0x2E (sf_en=0b101 + stream=0x06)",
                egress_bytes[2] == TEST_FLAGS_BYTE);

        // ---- E7 (cont): n_range, n_doppler ----
        check_b("E7.3: byte5/6 = n_range = 512 (BE)",
                {egress_bytes[5], egress_bytes[6]} == 16'd512);
        check_b("E7.4: byte7/8 = n_doppler = 48 (BE)",
                {egress_bytes[7], egress_bytes[8]} == 16'd48);

        // ---- E11: footer ----
        check_b("E11: last byte = 0x55 (footer)",
                egress_bytes[EXPECTED_FRAME_BYTES - 1] == 8'h55);

        // ---- E9, E10, E12: byte-for-byte and Python parse round-trip ----
        // Heavy lifting deferred to tb_e2e_dsp_to_host_parse.py (PR-Z.3).
        // Dump captured frame for the parser.
        $writememh("tb/cosim/e2e_data/captured_frame.hex", egress_bytes,
                   0, EXPECTED_FRAME_BYTES - 1);
        $display("\n  wrote captured_frame.hex (%0d bytes) for E9/E10/E12",
                 EXPECTED_FRAME_BYTES);

        // ====================================================================
        // SUMMARY
        // ====================================================================
        $display("\n============================================================");
        $display("  RESULTS");
        $display("    pass = %0d  fail = %0d  total = %0d",
                 pass_count, fail_count, test_count);
        $display("    egress = %0d / %0d expected",
                 egress_count, EXPECTED_FRAME_BYTES);
        $display("============================================================");
        if (fail_count == 0)
            $display("[OVERALL PASS] %0d/%0d", pass_count, test_count);
        else
            $display("[OVERALL FAIL] %0d/%0d (failures=%0d)",
                     pass_count, test_count, fail_count);

        #(CLK_PERIOD * 100);
        $finish;
    end

    // ====================================================================
    // Top-level watchdog — 60 s wall budget per the A6 scope memo.
    // 60 s wall ≈ ~6_000_000 cycles at iverilog's interpreted speed; bound
    // sim time at 60 ms simulated for a comfortable margin.
    // ====================================================================
    initial begin
        #60_000_000;
        $display("[TIMEOUT] tb_e2e_dsp_to_host exceeded 60 ms simulated");
        $display("  egress_count = %0d / %0d", egress_count, EXPECTED_FRAME_BYTES);
        $display("[OVERALL FAIL] watchdog");
        $finish;
    end

endmodule
