`timescale 1ns / 1ps
// ============================================================================
// fft_engine_axi_bridge — drop-in fft_engine replacement backed by xfft_2048
// ============================================================================
// Port list mirrors fft_engine.v exactly, so call sites in
// matched_filter_processing_chain.v swap with a single module-name change
// (`fft_engine` → `fft_engine_axi_bridge`). Internally instantiates the
// xfft_2048 AXI-Stream wrapper, which routes to either the LogiCORE FFT v9.1
// (synth/XSim, when FFT_USE_XILINX_IP is defined) or the in-house batched
// fft_engine (iverilog fallback). Either way the legacy interface is the same.
//
// Behavior contract preserved from fft_engine:
//   - `start` pulse begins a frame; `inverse` selects FWD/INV
//   - feed N samples on din_re/im with din_valid (any spacing OK)
//   - dout_re/im pulse out with dout_valid for N samples
//   - `done` pulses on the last output sample (tlast)
//   - `busy` is high from start through done
//
// Latency: replaces fft_engine's ~150-180K-cycle iterative compute with the
// LogiCORE Pipelined Streaming ~N + ~150-cycle pipeline. Functional behavior
// is identical from the chain's view.
// ============================================================================

module fft_engine_axi_bridge #(
    parameter N            = 2048,
    parameter LOG2N        = 11,
    parameter DATA_W       = 16,
    parameter INTERNAL_W   = 32,
    parameter TWIDDLE_W    = 16,
    parameter TWIDDLE_FILE = "fft_twiddle_2048.mem"
) (
    input  wire                          clk,
    input  wire                          reset_n,
    input  wire                          start,
    input  wire                          inverse,

    input  wire signed [DATA_W-1:0]      din_re,
    input  wire signed [DATA_W-1:0]      din_im,
    input  wire                          din_valid,

    output wire signed [DATA_W-1:0]      dout_re,
    output wire signed [DATA_W-1:0]      dout_im,
    output wire                          dout_valid,

    output reg                           busy,
    output reg                           done
);

// ============================================================================
// AXI-Stream signals to/from xfft_2048
// ============================================================================
reg  [7:0]  cfg_tdata;
reg         cfg_tvalid;
wire        cfg_tready;

reg  [31:0] axi_din_tdata;
reg         axi_din_tvalid;
reg         axi_din_tlast;
wire        axi_din_tready;

wire [31:0] axi_dout_tdata;
wire [7:0]  axi_dout_tuser;
wire        axi_dout_tvalid;
wire        axi_dout_tlast;

// 1-deep skid buffer absorbs LogiCORE FFT v9.1 nonrealtime backpressure
// (PG109: tready may dip briefly during pipeline / BFP normalization events).
// Upstream matched_filter_processing_chain has no flow-control input, so the
// bridge cannot push back — must buffer. Sustained 2+ cycle backpressure sets
// overflow_sticky for debug visibility.
reg  [31:0]      skid_data;
reg              skid_valid;
reg              skid_last;
reg  [LOG2N:0]   accept_count;     // beats actually accepted by IP (tvalid&&tready)
reg              overflow_sticky;  // sticky: skid+active both full when upstream pushed

// xfft_2048 wrapper. AXI master always-accept (no backpressure modeling here).
xfft_2048 u_xfft (
    .aclk                 (clk),
    .aresetn              (reset_n),
    .s_axis_config_tdata  (cfg_tdata),
    .s_axis_config_tvalid (cfg_tvalid),
    .s_axis_config_tready (cfg_tready),
    .s_axis_data_tdata    (axi_din_tdata),
    .s_axis_data_tvalid   (axi_din_tvalid),
    .s_axis_data_tlast    (axi_din_tlast),
    .s_axis_data_tready   (axi_din_tready),
    .m_axis_data_tdata    (axi_dout_tdata),
    .m_axis_data_tuser    (axi_dout_tuser),
    .m_axis_data_tvalid   (axi_dout_tvalid),
    .m_axis_data_tlast    (axi_dout_tlast),
    .m_axis_data_tready   (1'b1)
);

// Output mapping: AXI {Q,I} 32-bit → fft_engine-style separate re/im
assign dout_re    = $signed(axi_dout_tdata[15:0]);
assign dout_im    = $signed(axi_dout_tdata[31:16]);
assign dout_valid = axi_dout_tvalid;

// ============================================================================
// Bridge FSM
// ============================================================================
// On `start`: latch inverse, send config (one-cycle pulse with FWD bit), then
// open the data path. Track sample count so we can assert tlast on the Nth
// input. `busy` raised on start, dropped after done. `done` pulsed on last
// output (tlast).
// ============================================================================
localparam [1:0] S_IDLE   = 2'd0,
                 S_CFG    = 2'd1,
                 S_FEED   = 2'd2,
                 S_DRAIN  = 2'd3;

reg [1:0]                state;
reg                      inverse_latched;
reg [LOG2N:0]            in_count;       // counts inputs accepted into the IP

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state           <= S_IDLE;
        cfg_tdata       <= 8'd0;
        cfg_tvalid      <= 1'b0;
        axi_din_tdata   <= 32'd0;
        axi_din_tvalid  <= 1'b0;
        axi_din_tlast   <= 1'b0;
        in_count        <= 0;
        inverse_latched <= 1'b0;
        busy            <= 1'b0;
        done            <= 1'b0;
        skid_data       <= 32'd0;
        skid_valid      <= 1'b0;
        skid_last       <= 1'b0;
        accept_count    <= 0;
        overflow_sticky <= 1'b0;
    end else begin
        // Defaults — pulses
        done <= 1'b0;

        case (state)
        S_IDLE: begin
            axi_din_tvalid <= 1'b0;
            axi_din_tlast  <= 1'b0;
            cfg_tvalid     <= 1'b0;
            skid_valid     <= 1'b0;
            if (start) begin
                inverse_latched <= inverse;
                cfg_tdata       <= {7'd0, ~inverse};   // tdata[0]=1 → FWD
                cfg_tvalid      <= 1'b1;
                in_count        <= 0;
                accept_count    <= 0;
                busy            <= 1'b1;
                state           <= S_CFG;
            end
        end

        S_CFG: begin
            // Hold cfg_tvalid until IP accepts (tready). Then open data path.
            if (cfg_tready) begin
                cfg_tvalid <= 1'b0;
                state      <= S_FEED;
            end
        end

        S_FEED: begin
            // Phase 1: handshake — IP accepted current beat. Drain skid into
            // active (or clear active). Advance accept_count.
            if (axi_din_tvalid && axi_din_tready) begin
                accept_count <= accept_count + 1'b1;
                if (skid_valid) begin
                    axi_din_tdata  <= skid_data;
                    axi_din_tlast  <= skid_last;
                    axi_din_tvalid <= 1'b1;
                end else begin
                    axi_din_tvalid <= 1'b0;
                    axi_din_tlast  <= 1'b0;
                end
                skid_valid <= 1'b0;
            end

            // Phase 2: load incoming sample. NBA "last assignment wins" lets
            // these overrides supersede Phase 1 when both fire same cycle.
            if (din_valid && (in_count < N)) begin
                if (axi_din_tvalid && axi_din_tready) begin
                    // Active was just drained / shifted into this cycle
                    if (skid_valid) begin
                        // Skid → active; new sample → skid (skid stays full)
                        skid_data  <= {din_im, din_re};
                        skid_last  <= (in_count == N - 1);
                        skid_valid <= 1'b1;
                    end else begin
                        // Active became empty; new sample → active
                        axi_din_tdata  <= {din_im, din_re};
                        axi_din_tlast  <= (in_count == N - 1);
                        axi_din_tvalid <= 1'b1;
                    end
                    in_count <= in_count + 1'b1;
                end else begin
                    // No handshake this cycle
                    if (!axi_din_tvalid) begin
                        axi_din_tdata  <= {din_im, din_re};
                        axi_din_tlast  <= (in_count == N - 1);
                        axi_din_tvalid <= 1'b1;
                        in_count       <= in_count + 1'b1;
                    end else if (!skid_valid) begin
                        skid_data  <= {din_im, din_re};
                        skid_last  <= (in_count == N - 1);
                        skid_valid <= 1'b1;
                        in_count   <= in_count + 1'b1;
                    end else begin
                        // Both slots full — sample lost. Sticky flag for debug.
                        overflow_sticky <= 1'b1;
                    end
                end
            end

            // Transition to drain on the cycle the Nth beat is accepted.
            // Override Phase 1+2 loads — no more samples to deliver.
            if (axi_din_tvalid && axi_din_tready && (accept_count + 1'b1 == N)) begin
                axi_din_tvalid <= 1'b0;
                axi_din_tlast  <= 1'b0;
                state          <= S_DRAIN;
            end
        end

        S_DRAIN: begin
            // Wait for tlast on output, then return to idle.
            if (axi_dout_tvalid && axi_dout_tlast) begin
                done  <= 1'b1;
                busy  <= 1'b0;
                state <= S_IDLE;
            end
        end

        default: state <= S_IDLE;
        endcase
    end
end

endmodule
