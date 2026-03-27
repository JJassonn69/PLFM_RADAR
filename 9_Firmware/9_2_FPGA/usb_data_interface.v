module usb_data_interface (
    input wire clk,              // Main clock (100MHz recommended)
    input wire reset_n,
    input wire ft601_reset_n,    // FT601-domain synchronized reset
    
    // Radar data inputs
    input wire [31:0] range_profile,
    input wire range_valid,
    input wire [15:0] doppler_real,
    input wire [15:0] doppler_imag,
    input wire doppler_valid,
    input wire cfar_detection,
    input wire cfar_valid,
    
    // FT601 Interface (Slave FIFO mode)
    // Data bus
    inout wire [31:0] ft601_data,    // 32-bit bidirectional data bus
    output reg [3:0] ft601_be,       // Byte enable (4 lanes for 32-bit mode)
    
    // Control signals
    output reg ft601_txe_n,          // Transmit enable (active low)
    output reg ft601_rxf_n,          // Receive enable (active low)
    input wire ft601_txe,             // Transmit FIFO empty
    input wire ft601_rxf,             // Receive FIFO full
    output reg ft601_wr_n,            // Write strobe (active low)
    output reg ft601_rd_n,            // Read strobe (active low)
    output reg ft601_oe_n,            // Output enable (active low)
    output reg ft601_siwu_n,          // Send immediate / Wakeup
    
    // FIFO flags
    input wire [1:0] ft601_srb,       // Selected read buffer
    input wire [1:0] ft601_swb,       // Selected write buffer
    
    // Clock
    output wire ft601_clk_out,        // Output clock to FT601 (forwarded via ODDR)
    input wire ft601_clk_in,          // Clock from FT601 (60/100MHz)
    
    // ========== HOST COMMAND OUTPUTS (Gap 4: USB Read Path) ==========
    // These outputs are registered in the ft601_clk domain and must be
    // CDC-synchronized by the consumer (radar_system_top.v) before use
    // in the clk_100m domain.
    //
    // Command word format: {opcode[7:0], addr[7:0], value[15:0]}
    //   0x01 = Set radar mode (value[1:0] -> mode_controller mode)
    //   0x02 = Single chirp trigger (value ignored, pulse cmd_valid)
    //   0x03 = Set CFAR threshold (value[15:0] -> threshold)
    //   0x04 = Set stream control (value[2:0] -> enable range/doppler/cfar)
    //   0x10-0x15 = Chirp timing configuration (Gap 2)
    //   0xFF = Status request (triggers status response packet)
    output reg [31:0] cmd_data,      // Last received command word
    output reg cmd_valid,            // Pulse: new command received (ft601_clk domain)
    output reg [7:0] cmd_opcode,     // Decoded opcode for convenience
    output reg [7:0] cmd_addr,       // Decoded register address
    output reg [15:0] cmd_value,     // Decoded value

    // Gap 2: Stream control input (clk_100m domain, CDC'd internally)
    // Bit 0 = range stream enable
    // Bit 1 = doppler stream enable
    // Bit 2 = cfar/detection stream enable
    input wire [2:0] stream_control,

    // Gap 2: Status readback inputs (clk_100m domain, CDC'd internally)
    // When status_request pulses, the write FSM sends a status response
    // packet containing the current register values.
    input wire status_request,                 // 1-cycle pulse in clk_100m when 0xFF received
    input wire [15:0] status_cfar_threshold,   // Current CFAR threshold
    input wire [2:0]  status_stream_ctrl,      // Current stream control
    input wire [1:0]  status_radar_mode,       // Current radar mode
    input wire [15:0] status_long_chirp,       // Current long chirp cycles
    input wire [15:0] status_long_listen,      // Current long listen cycles
    input wire [15:0] status_guard,            // Current guard cycles
    input wire [15:0] status_short_chirp,      // Current short chirp cycles
    input wire [15:0] status_short_listen,     // Current short listen cycles
    input wire [5:0]  status_chirps_per_elev,  // Current chirps per elevation
    input wire [1:0]  status_range_mode,       // Fix 7: Current range mode (0x20)

    // Self-test status readback (opcode 0x31 / included in 0xFF status packet)
    input wire [4:0]  status_self_test_flags,  // Per-test PASS(1)/FAIL(0) latched
    input wire [7:0]  status_self_test_detail, // Diagnostic detail byte latched
    input wire        status_self_test_busy,   // Self-test FSM still running

    // ========== DEBUG INSTRUMENTATION (v7b) ==========
    // Free-running counters visible via status packet for diagnosing the
    // 5-byte-per-packet write FSM stall. All counters are in ft601_clk domain.
    output wire [15:0] dbg_wr_strobes,     // Total WR_N=0 assertions (write cycles)
    output wire [15:0] dbg_txe_blocks,     // Cycles where FSM wanted to write but TXE was high
    output wire [15:0] dbg_pkt_starts,     // SEND_HEADER entries (packet starts)
    output wire [15:0] dbg_pkt_completions // WAIT_ACK entries (packet completions)
);

// USB packet structure (same as before)
localparam HEADER = 8'hAA;
localparam FOOTER = 8'h55;

// FT601 configuration
localparam FT601_DATA_WIDTH = 32;
localparam FT601_BURST_SIZE = 512;    // Max burst size in bytes

// ============================================================================
// WRITE FSM State definitions (Verilog-2001 compatible)
// ============================================================================
localparam [2:0] IDLE                = 3'd0,
                 SEND_HEADER         = 3'd1,
                 SEND_RANGE_DATA     = 3'd2,
                 SEND_DOPPLER_DATA   = 3'd3,
                 SEND_DETECTION_DATA = 3'd4,
                 SEND_FOOTER         = 3'd5,
                 WAIT_ACK            = 3'd6,
                 SEND_STATUS         = 3'd7;  // Gap 2: status readback

reg [2:0] current_state;
reg [7:0] byte_counter;
reg [31:0] data_buffer;
reg [31:0] ft601_data_out;
reg ft601_data_oe;  // Output enable for bidirectional data bus

// ========== DEBUG COUNTERS (v7b) ==========
reg [15:0] dbg_wr_strobes_r;      // Incremented each cycle WR_N is asserted LOW
reg [15:0] dbg_txe_blocks_r;      // Incremented when FSM is in a write state but TXE is HIGH
reg [15:0] dbg_pkt_starts_r;      // Incremented on each SEND_HEADER entry
reg [15:0] dbg_pkt_completions_r; // Incremented on each WAIT_ACK entry

assign dbg_wr_strobes     = dbg_wr_strobes_r;
assign dbg_txe_blocks     = dbg_txe_blocks_r;
assign dbg_pkt_starts     = dbg_pkt_starts_r;
assign dbg_pkt_completions = dbg_pkt_completions_r;

// ============================================================================
// READ FSM State definitions (Gap 4: USB Read Path)
// ============================================================================
// FT601 245 synchronous FIFO read protocol:
//   1. Host has data available: RXF_N = 0 (active low)
//   2. FPGA asserts OE_N = 0 (bus turnaround: FT601 starts driving data bus)
//   3. Wait 1 cycle for bus turnaround settling
//   4. FPGA asserts RD_N = 0 (start reading: data valid on each posedge)
//   5. Sample ft601_data[31:0] while RD_N = 0 and RXF_N = 0
//   6. Deassert RD_N = 1, then OE_N = 1
//
// The read FSM only activates when the write FSM is IDLE and RXF indicates
// data is available. This prevents bus contention between TX and RX.
localparam [2:0] RD_IDLE      = 3'd0,  // Waiting for RXF
                 RD_OE_ASSERT = 3'd1,  // Assert OE_N=0, wait turnaround
                 RD_READING   = 3'd2,  // Assert RD_N=0, sample data
                 RD_DEASSERT  = 3'd3,  // Deassert RD_N, then OE_N
                 RD_PROCESS   = 3'd4;  // Process received command word

reg [2:0] read_state;
reg [31:0] rx_data_captured;  // Data word read from host

// ========== POST-POR STARTUP LOCKOUT ==========
// After reset release, the CDC synchronizers need several cycles to settle.
// During this window, stale edges on status_req_edge (or other CDC artifacts)
// could cause the write FSM to enter SEND_STATUS spuriously. A lockout counter
// prevents the write FSM from leaving IDLE for 256 ft601_clk cycles (~2.56us
// at 100MHz). Any status_req_edge pulses during lockout are harmlessly consumed
// because the IDLE state won't act on them.
reg [7:0] startup_lockout_ctr;
wire startup_lockout_active = (startup_lockout_ctr != 8'hFF);

// ========== SEND_STATUS WATCHDOG ==========
// If the write FSM enters SEND_STATUS and TXE_N stays HIGH (FT601 IN FIFO
// full / backpressure), the FSM would stall forever, blocking the read FSM
// from accepting new host commands. A watchdog counter aborts the status
// send if it can't complete within 65536 cycles (~655us).
reg [15:0] status_watchdog_ctr;

// ========== CDC INPUT SYNCHRONIZERS (clk domain -> ft601_clk_in domain) ==========
// The valid signals arrive from clk_100m but the state machine runs on ft601_clk_in.
// Even though both are 100 MHz, they are asynchronous clocks and need synchronization.

// 2-stage synchronizers for valid signals
(* ASYNC_REG = "TRUE" *) reg [1:0] range_valid_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] doppler_valid_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] cfar_valid_sync;

// Delayed versions of sync[1] for proper edge detection
reg range_valid_sync_d;
reg doppler_valid_sync_d;
reg cfar_valid_sync_d;

// Holding registers: data captured in SOURCE domain (clk_100m) when valid
// asserts, then read by ft601 domain after synchronized valid edge.
// This is safe because the data is stable for the entire time the valid
// pulse is being synchronized (2+ ft601_clk cycles).
reg [31:0] range_profile_hold;
reg [15:0] doppler_real_hold;
reg [15:0] doppler_imag_hold;
reg cfar_detection_hold;

// Gap 2: Status request toggle register (clk_100m domain).
// Declared here (before the always block) to satisfy iverilog forward-ref rules.
reg status_req_toggle_100m;

// Source-domain holding registers (clk domain)
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        range_profile_hold <= 32'd0;
        doppler_real_hold  <= 16'd0;
        doppler_imag_hold  <= 16'd0;
        cfar_detection_hold <= 1'b0;
        status_req_toggle_100m <= 1'b0;
    end else begin
        if (range_valid)
            range_profile_hold <= range_profile;
        if (doppler_valid) begin
            doppler_real_hold <= doppler_real;
            doppler_imag_hold <= doppler_imag;
        end
        if (cfar_valid)
            cfar_detection_hold <= cfar_detection;
        // Gap 2: Toggle on status request pulse (CDC to ft601_clk)
        if (status_request)
            status_req_toggle_100m <= ~status_req_toggle_100m;
    end
end

// FT601-domain captured data (sampled from holding regs on sync'd edge)
reg [31:0] range_profile_cap;
reg [15:0] doppler_real_cap;
reg [15:0] doppler_imag_cap;
reg cfar_detection_cap;

// Data-pending flags (ft601_clk domain).
// Set when a valid edge is detected, cleared when the write FSM consumes
// or skips the data. Prevents the write FSM from blocking forever when
// a stream's valid hasn't fired yet (e.g., Doppler needs 32 chirps).
reg doppler_data_pending;
reg cfar_data_pending;

// Gap 2: CDC for stream_control (clk_100m -> ft601_clk_in)
// stream_control changes infrequently (only on host USB command), so
// per-bit 2-stage synchronizers are sufficient. No Gray coding needed
// because the bits are independent enables.
// Fix #5: Default to range-only (3'b001) on reset to prevent write FSM
// deadlock before host configures streams. With all streams enabled on
// reset, the first range_valid triggers the write FSM which then blocks
// forever on SEND_DOPPLER_DATA (Doppler hasn't produced data yet).
(* ASYNC_REG = "TRUE" *) reg [2:0] stream_ctrl_sync_0;
(* ASYNC_REG = "TRUE" *) reg [2:0] stream_ctrl_sync_1;
wire stream_range_en   = stream_ctrl_sync_1[0];
wire stream_doppler_en = stream_ctrl_sync_1[1];
wire stream_cfar_en    = stream_ctrl_sync_1[2];

// Gap 2: Status request CDC (toggle CDC, same pattern as cmd_valid)
// status_request is a 1-cycle pulse in clk_100m. Toggle→sync→edge-detect.
// NOTE: status_req_toggle_100m declared above (before source-domain always block)
(* ASYNC_REG = "TRUE" *) reg [1:0] status_req_sync;
reg status_req_sync_prev;
wire status_req_edge = status_req_sync[1] ^ status_req_sync_prev;

// v7c: Status request pending flag — set by CDC edge, cleared when consumed.
// This prevents the combinational XOR wire from re-triggering on subsequent
// cycles if the sync chain produces multi-cycle glitches or if both clocks
// are the same (dev wrapper) and timing is marginal.
reg status_req_pending;

// v7c: Status busy flag — HIGH while SEND_STATUS is active. Prevents
// re-entry from IDLE into SEND_STATUS if status_req_pending somehow
// re-asserts before the previous status send completes. Belt-and-suspenders.
reg status_busy;

// Status snapshot: captured in ft601_clk domain when status request arrives.
// The clk_100m-domain status inputs are stable for many cycles after the
// command decode, so sampling them a few ft601_clk cycles later is safe.
// v7b: expanded to 8 words (added 2 debug counter words)
reg [31:0] status_words [0:7];  // 8 status words (6 config + 2 debug)
reg [3:0] status_word_idx;      // v7b: expanded from 3 to 4 bits for 10 transfers

wire range_valid_ft;
wire doppler_valid_ft;
wire cfar_valid_ft;

always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n) begin
        range_valid_sync   <= 2'b00;
        doppler_valid_sync <= 2'b00;
        cfar_valid_sync    <= 2'b00;
        range_valid_sync_d   <= 1'b0;
        doppler_valid_sync_d <= 1'b0;
        cfar_valid_sync_d    <= 1'b0;
        range_profile_cap  <= 32'd0;
        doppler_real_cap   <= 16'd0;
        doppler_imag_cap   <= 16'd0;
        cfar_detection_cap <= 1'b0;
        // Default to stream OFF on reset — host must explicitly enable
        stream_ctrl_sync_0 <= 3'b000;
        stream_ctrl_sync_1 <= 3'b000;
        // Gap 2: status request CDC reset
        status_req_sync <= 2'b00;
        status_req_sync_prev <= 1'b0;
    end else begin
        // Synchronize valid strobes (2-stage sync chain)
        range_valid_sync   <= {range_valid_sync[0],   range_valid};
        doppler_valid_sync <= {doppler_valid_sync[0], doppler_valid};
        cfar_valid_sync    <= {cfar_valid_sync[0],    cfar_valid};

        // Gap 2: stream control CDC (2-stage)
        stream_ctrl_sync_0 <= stream_control;
        stream_ctrl_sync_1 <= stream_ctrl_sync_0;

        // Gap 2: status request CDC (toggle sync + edge detect)
        status_req_sync <= {status_req_sync[0], status_req_toggle_100m};
        status_req_sync_prev <= status_req_sync[1];

        // v7c: status_req_pending and status_busy are driven in the main
        // FSM always block below (same block) to avoid multi-driver DRC.

        // Gap 2: Capture status snapshot when request arrives in ft601 domain
        // v7c: Capture on the CDC edge itself (before pending could be cleared)
        if (status_req_edge) begin
            // Pack register values into 5x 32-bit status words
            // Word 0: {0xFF, mode[1:0], stream_ctrl[2:0], cfar_threshold[15:0]}
            status_words[0] <= {8'hFF, 3'b000, status_radar_mode,
                                5'b00000, status_stream_ctrl,
                                status_cfar_threshold};
            // Word 1: {long_chirp_cycles[15:0], long_listen_cycles[15:0]}
            status_words[1] <= {status_long_chirp, status_long_listen};
            // Word 2: {guard_cycles[15:0], short_chirp_cycles[15:0]}
            status_words[2] <= {status_guard, status_short_chirp};
            // Word 3: {short_listen_cycles[15:0], chirps_per_elev[5:0], 10'b0}
            status_words[3] <= {status_short_listen, 10'd0, status_chirps_per_elev};
            // Word 4: Fix 7 — range_mode in bits [1:0], rest reserved
            status_words[4] <= {30'd0, status_range_mode};
            // Word 5: Self-test results {reserved[6:0], busy, reserved[7:0], detail[7:0], reserved[2:0], flags[4:0]}
            status_words[5] <= {7'd0, status_self_test_busy,
                                8'd0, status_self_test_detail,
                                3'd0, status_self_test_flags};
            // Word 6: Debug counters {dbg_wr_strobes[15:0], dbg_txe_blocks[15:0]}
            status_words[6] <= {dbg_wr_strobes_r, dbg_txe_blocks_r};
            // Word 7: Debug counters {dbg_pkt_starts[15:0], dbg_pkt_completions[15:0]}
            status_words[7] <= {dbg_pkt_starts_r, dbg_pkt_completions_r};
        end

        // Delayed version of sync[1] for edge detection
        range_valid_sync_d   <= range_valid_sync[1];
        doppler_valid_sync_d <= doppler_valid_sync[1];
        cfar_valid_sync_d    <= cfar_valid_sync[1];

        // Capture data on rising edge of FULLY SYNCHRONIZED valid (sync[1])
        // Data in holding regs is stable by the time sync[1] rises (2+ cycles)
        if (range_valid_sync[1] && !range_valid_sync_d)
            range_profile_cap <= range_profile_hold;
        if (doppler_valid_sync[1] && !doppler_valid_sync_d) begin
            doppler_real_cap <= doppler_real_hold;
            doppler_imag_cap <= doppler_imag_hold;
        end
        if (cfar_valid_sync[1] && !cfar_valid_sync_d) begin
            cfar_detection_cap <= cfar_detection_hold;
        end
    end
end

// Rising-edge detect on FULLY SYNCHRONIZED valid (sync[1], not sync[0])
// This provides full 2-stage metastability protection before use.
assign range_valid_ft   = range_valid_sync[1]   && !range_valid_sync_d;
assign doppler_valid_ft = doppler_valid_sync[1]  && !doppler_valid_sync_d;
assign cfar_valid_ft    = cfar_valid_sync[1]     && !cfar_valid_sync_d;

// FT601 data bus direction control
assign ft601_data = ft601_data_oe ? ft601_data_out : 32'hzzzz_zzzz;

// ========== INPUT PIPELINE REGISTERS (timing fix) ==========
// ft601_txe and ft601_rxf are sampled directly from IOB pads. Without
// registration they feed into FSM combinational logic that drives IOB
// output register CE pins, creating a pad-to-pad path with only ~0.06 ns
// slack.  A single pipeline stage gives a full clock period of margin.
// The 1-cycle latency is transparent: FT601 FIFO flags stay asserted for
// many cycles in normal operation.
reg ft601_txe_r;
reg ft601_rxf_r;

always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n) begin
        current_state <= IDLE;
        read_state <= RD_IDLE;
        byte_counter <= 0;
        ft601_data_out <= 0;
        ft601_data_oe <= 0;
        ft601_be <= 4'b1111;  // All bytes enabled for 32-bit mode
        ft601_txe_n <= 1;
        ft601_rxf_n <= 1;
        ft601_wr_n <= 1;
        ft601_rd_n <= 1;
        ft601_oe_n <= 1;
        ft601_siwu_n <= 1;
        rx_data_captured <= 32'd0;
        cmd_data <= 32'd0;
        cmd_valid <= 1'b0;
        cmd_opcode <= 8'd0;
        cmd_addr <= 8'd0;
        cmd_value <= 16'd0;
        doppler_data_pending <= 1'b0;
        cfar_data_pending <= 1'b0;
        ft601_txe_r <= 1'b1;  // Default: FIFO not ready
        ft601_rxf_r <= 1'b1;  // Default: no host data
        startup_lockout_ctr <= 8'd0;  // Start lockout on reset
        status_watchdog_ctr <= 16'd0;
        dbg_wr_strobes_r <= 16'd0;
        dbg_txe_blocks_r <= 16'd0;
        dbg_pkt_starts_r <= 16'd0;
        dbg_pkt_completions_r <= 16'd0;
        status_word_idx <= 4'd0;       // v7c: moved from CDC block to avoid multi-driver
        status_req_pending <= 1'b0;    // v7c: status request pending flag
        status_busy <= 1'b0;           // v7c: status send in progress flag
        // NOTE: ft601_clk_out is driven by the clk-domain always block below.
        // Do NOT assign it here (ft601_clk_in domain) — causes multi-driven net.
    end else begin
        // Default: clear one-shot signals
        cmd_valid <= 1'b0;

        // Input pipeline registers (timing fix: breaks pad-to-fabric critical path)
        ft601_txe_r <= ft601_txe;
        ft601_rxf_r <= ft601_rxf;

        // Post-POR startup lockout: count up to 255, then stay at 255.
        // During lockout, write FSM stays in IDLE and ignores all triggers.
        if (startup_lockout_active) begin
            startup_lockout_ctr <= startup_lockout_ctr + 8'd1;
            // v7c: Drain any spurious status_req_pending during lockout
            status_req_pending <= 1'b0;
        end else begin
            // v7c: Set pending flag on CDC edge if not already busy.
            // This is the ONLY place status_req_pending is set (outside reset).
            // It is cleared by: IDLE→SEND_STATUS transition, or startup lockout drain.
            if (status_req_edge && !status_busy)
                status_req_pending <= 1'b1;
        end

        // Data-pending flag management: set on valid edge, cleared when
        // consumed or skipped by write FSM. Must be in this always block
        // (not the CDC sync block) to avoid Vivado multiple-driver DRC error.
        if (doppler_valid_ft)
            doppler_data_pending <= 1'b1;
        if (cfar_valid_ft)
            cfar_data_pending <= 1'b1;

        // ================================================================
        // READ FSM — host-to-FPGA command path (Gap 4)
        //
        // The read FSM takes priority over write when both could activate.
        // It only starts when the write FSM is IDLE and ft601_rxf
        // indicates data from host is available.
        // ================================================================
        case (read_state)
            RD_IDLE: begin
                // Only start reading if write FSM is idle and host has data.
                // ft601_rxf active-low: 0 means data available from host.
                if (current_state == IDLE && !ft601_rxf_r) begin
                    ft601_oe_n <= 1'b0;     // Assert OE: tell FT601 to drive bus
                    ft601_data_oe <= 1'b0;  // FPGA releases bus (FT601 drives)
                    read_state <= RD_OE_ASSERT;
                end
            end

            RD_OE_ASSERT: begin
                // 1-cycle turnaround: OE_N asserted, bus settling.
                // FT601 spec requires 1 clock of OE_N before RD_N assertion.
                if (!ft601_rxf_r) begin
                    ft601_rd_n <= 1'b0;     // Assert RD: start reading
                    read_state <= RD_READING;
                end else begin
                    // Host withdrew data — abort
                    ft601_oe_n <= 1'b1;
                    read_state <= RD_IDLE;
                end
            end

            RD_READING: begin
                // Data is valid on ft601_data. Sample it.
                // For now we read a single 32-bit command word per transaction.
                rx_data_captured <= ft601_data;
                ft601_rd_n <= 1'b1;         // Deassert RD
                read_state <= RD_DEASSERT;
            end

            RD_DEASSERT: begin
                // Deassert OE_N (1 cycle after RD_N deasserted)
                ft601_oe_n <= 1'b1;
                read_state <= RD_PROCESS;
            end

            RD_PROCESS: begin
                // Decode the received command word and pulse cmd_valid.
                // FT601 245 mode byte lane mapping (little-endian):
                //   DATA[7:0]   = first USB byte  = opcode
                //   DATA[15:8]  = second USB byte = addr
                //   DATA[31:16] = bytes 3-4       = value (big-endian within)
                // Host sends: struct.pack(">I", (opcode<<24)|(addr<<16)|value)
                //   => wire bytes: [opcode, addr, value_hi, value_lo]
                //   => FT601 presents: DATA = {value_lo, value_hi, addr, opcode}
                cmd_data   <= rx_data_captured;
                cmd_opcode <= rx_data_captured[7:0];
                cmd_addr   <= rx_data_captured[15:8];
                cmd_value  <= {rx_data_captured[23:16], rx_data_captured[31:24]};
                cmd_valid  <= 1'b1;
                read_state <= RD_IDLE;
            end

            default: read_state <= RD_IDLE;
        endcase

        // ================================================================
        // WRITE FSM — FPGA-to-host data streaming (existing)
        //
        // Only operates when read FSM is idle (no bus contention).
        // ================================================================
        if (read_state == RD_IDLE) begin
            // Per-cycle safe defaults: deassert write signals every cycle.
            // Individual states re-assert them only when TXE allows a write.
            // This prevents WR_N from staying LOW when TXE_N goes HIGH mid-packet.
            ft601_wr_n <= 1'b1;
            ft601_data_oe <= 1'b0;

            // ============ DEBUG COUNTER LOGIC (v7b) ============
            // Count TXE blocks: FSM is in a write-capable state but TXE is HIGH.
            // This tells us whether the FSM is stalling on FT601 backpressure.
            if (current_state != IDLE && current_state != WAIT_ACK && ft601_txe_r)
                dbg_txe_blocks_r <= dbg_txe_blocks_r + 16'd1;

            case (current_state)
                IDLE: begin
                    // During startup lockout, ignore all triggers.
                    // Any status_req_pending flags set during lockout are
                    // harmlessly consumed when the lockout expires.
                    if (!startup_lockout_active) begin
                        // Gap 2: Status readback takes priority
                        // v7c: Use registered pending flag + !busy guard
                        if (status_req_pending && !status_busy && ft601_rxf_r) begin
                            current_state <= SEND_STATUS;
                            status_word_idx <= 4'd0;
                            status_watchdog_ctr <= 16'd0;
                            status_busy <= 1'b1;       // v7c: Mark status send in progress
                            status_req_pending <= 1'b0; // v7c: Consume the request
                        end
                        // Trigger write FSM on range_valid edge (primary data source).
                        // Doppler/cfar data_pending flags are checked inside
                        // SEND_DOPPLER_DATA and SEND_DETECTION_DATA to skip or send.
                        // Do NOT trigger on pending flags alone — they're sticky and
                        // would cause repeated packet starts without new range data.
                        else if (range_valid_ft && stream_range_en) begin
                            // Don't start write if a read is about to begin
                            if (ft601_rxf_r) begin  // rxf=1 means no host data pending
                                current_state <= SEND_HEADER;
                                byte_counter <= 0;
                            end
                        end
                    end
                end
                
                SEND_HEADER: begin
                    if (!ft601_txe_r) begin  // FT601 TX FIFO not empty
                        ft601_data_oe <= 1;
                        ft601_data_out <= {24'h000000, HEADER};  // v7b: Header in full 32-bit word
                        ft601_be <= 4'b1111;  // v7b: Always use full 32-bit BE
                        ft601_wr_n <= 0;     // Assert write strobe
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        dbg_pkt_starts_r <= dbg_pkt_starts_r + 16'd1;
                        // Gap 2: skip to first enabled stream
                        if (stream_range_en)
                            current_state <= SEND_RANGE_DATA;
                        else if (stream_doppler_en)
                            current_state <= SEND_DOPPLER_DATA;
                        else if (stream_cfar_en)
                            current_state <= SEND_DETECTION_DATA;
                        else
                            current_state <= SEND_FOOTER;  // No streams — send footer only
                    end
                end
                
                SEND_RANGE_DATA: begin
                    if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;  // All bytes valid for 32-bit word
                        
                        case (byte_counter)
                            0: ft601_data_out <= range_profile_cap;
                            1: ft601_data_out <= {range_profile_cap[23:0], 8'h00};
                            2: ft601_data_out <= {range_profile_cap[15:0], 16'h0000};
                            3: ft601_data_out <= {range_profile_cap[7:0], 24'h000000};
                        endcase
                        
                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        
                        if (byte_counter == 3) begin
                            byte_counter <= 0;
                            // Gap 2: skip disabled streams
                            if (stream_doppler_en)
                                current_state <= SEND_DOPPLER_DATA;
                            else if (stream_cfar_en)
                                current_state <= SEND_DETECTION_DATA;
                            else
                                current_state <= SEND_FOOTER;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end
                end
                
                SEND_DOPPLER_DATA: begin
                    if (!ft601_txe_r && doppler_data_pending) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;
                        
                        case (byte_counter)
                            0: ft601_data_out <= {doppler_real_cap, doppler_imag_cap};
                            1: ft601_data_out <= {doppler_imag_cap, doppler_real_cap[15:8], 8'h00};
                            2: ft601_data_out <= {doppler_real_cap[7:0], doppler_imag_cap[15:8], 16'h0000};
                            3: ft601_data_out <= {doppler_imag_cap[7:0], 24'h000000};
                        endcase
                        
                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        
                        if (byte_counter == 3) begin
                            byte_counter <= 0;
                            doppler_data_pending <= 1'b0;
                            if (stream_cfar_en)
                                current_state <= SEND_DETECTION_DATA;
                            else
                                current_state <= SEND_FOOTER;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end else if (!doppler_data_pending) begin
                        // No doppler data available yet — skip to next stream
                        byte_counter <= 0;
                        if (stream_cfar_en)
                            current_state <= SEND_DETECTION_DATA;
                        else
                            current_state <= SEND_FOOTER;
                    end
                end
                
                SEND_DETECTION_DATA: begin
                    if (!ft601_txe_r && cfar_data_pending) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;  // v7b: full 32-bit word
                        ft601_data_out <= {24'b0, 7'b0, cfar_detection_cap};
                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        cfar_data_pending <= 1'b0;
                        current_state <= SEND_FOOTER;
                    end else if (!cfar_data_pending) begin
                        // No CFAR data available yet — skip to footer
                        current_state <= SEND_FOOTER;
                    end
                end
                
                SEND_FOOTER: begin
                    if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;  // v7b: full 32-bit word
                        ft601_data_out <= {24'h000000, FOOTER};
                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        current_state <= WAIT_ACK;
                    end
                end

                // Gap 2: Status readback — send 8 x 32-bit status words (v7b: +2 debug)
                // Format: BB_HEADER, status_words[0..7], FOOTER = 10 transfers
                // Watchdog: abort if can't complete within 65536 cycles
                SEND_STATUS: begin
                    status_watchdog_ctr <= status_watchdog_ctr + 16'd1;
                    if (status_watchdog_ctr == 16'hFFFF) begin
                        // Watchdog timeout — abort status send, return to IDLE
                        status_word_idx <= 4'd0;
                        status_busy <= 1'b0;  // v7c: Clear busy on watchdog abort
                        current_state <= IDLE;
                    end else if (!ft601_txe_r) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;  // v7b: always full 32-bit
                        case (status_word_idx)
                            4'd0: begin
                                // Send status header marker (0xBB = status response)
                                ft601_data_out <= {24'h000000, 8'hBB};
                            end
                            4'd1: ft601_data_out <= status_words[0];
                            4'd2: ft601_data_out <= status_words[1];
                            4'd3: ft601_data_out <= status_words[2];
                            4'd4: ft601_data_out <= status_words[3];
                            4'd5: ft601_data_out <= status_words[4];
                            4'd6: ft601_data_out <= status_words[5];
                            4'd7: ft601_data_out <= status_words[6];  // v7b: debug word 1
                            4'd8: ft601_data_out <= status_words[7];  // v7b: debug word 2
                            4'd9: begin
                                // Send status footer
                                ft601_data_out <= {24'h000000, FOOTER};
                            end
                            default: ;
                        endcase
                        ft601_wr_n <= 0;
                        dbg_wr_strobes_r <= dbg_wr_strobes_r + 16'd1;
                        if (status_word_idx == 4'd9) begin
                            status_word_idx <= 4'd0;
                            current_state <= WAIT_ACK;
                        end else begin
                            status_word_idx <= status_word_idx + 1;
                        end
                    end
                end
                
                WAIT_ACK: begin
                    // wr_n and data_oe already deasserted by per-cycle defaults
                    dbg_pkt_completions_r <= dbg_pkt_completions_r + 16'd1;
                    status_busy <= 1'b0;  // v7c: Clear busy flag (safe for data packets too)
                    current_state <= IDLE;
                end
            endcase
        end
    end
end

// ============================================================================
// FT601 clock output forwarding
// ============================================================================
// Forward ft601_clk_in back out via ODDR so that the forwarded clock at the
// pin has the same insertion delay as the data outputs (both go through the
// same BUFG). This makes the output delay analysis relative to the generated
// clock at the pin, where insertion delays cancel.

`ifndef SIMULATION
ODDR #(
    .DDR_CLK_EDGE("OPPOSITE_EDGE"),
    .INIT(1'b0),
    .SRTYPE("SYNC")
) oddr_ft601_clk (
    .Q(ft601_clk_out),
    .C(ft601_clk_in),
    .CE(1'b1),
    .D1(1'b1),
    .D2(1'b0),
    .R(1'b0),
    .S(1'b0)
);
`else
// Simulation: behavioral clock forwarding
reg ft601_clk_out_sim;
always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n)
        ft601_clk_out_sim <= 1'b0;
    else
        ft601_clk_out_sim <= 1'b1;
end
// In simulation, just pass the clock through
assign ft601_clk_out = ft601_clk_in;
`endif

endmodule