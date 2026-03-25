`timescale 1ns / 1ps

module tb_range_compression_physics;

localparam integer N = 1024;
// Keep amplitude low enough to avoid 16-bit saturation in behavioral FFT
// (1024-pt FFT of chirp with amp=A produces max ~A*50 per bin)
localparam integer CHIRP_SCALE = 250;
localparam integer CLK_PERIOD_NS = 10;

reg clk;
reg reset_n;

reg [15:0] adc_data_i_reg;
reg [15:0] adc_data_q_reg;
reg adc_valid_reg;
reg [5:0] chirp_counter_reg;
reg [15:0] long_chirp_real_reg;
reg [15:0] long_chirp_imag_reg;
reg [15:0] short_chirp_real_reg;
reg [15:0] short_chirp_imag_reg;

wire signed [15:0] range_profile_i;
wire signed [15:0] range_profile_q;
wire range_profile_valid;
wire [3:0] chain_state;

integer pass_count;
integer fail_count;
integer test_num;

reg signed [15:0] chirp_ref_i [0:N-1];
reg signed [15:0] chirp_ref_q [0:N-1];

reg signed [15:0] out_i [0:N-1];
reg signed [15:0] out_q [0:N-1];
reg [31:0] out_mag [0:N-1];

integer output_count;

real phase;
real chirp_rate;
integer n;

integer lfsr;

matched_filter_processing_chain dut (
    .clk(clk),
    .reset_n(reset_n),
    .adc_data_i(adc_data_i_reg),
    .adc_data_q(adc_data_q_reg),
    .adc_valid(adc_valid_reg),
    .chirp_counter(chirp_counter_reg),
    .long_chirp_real(long_chirp_real_reg),
    .long_chirp_imag(long_chirp_imag_reg),
    .short_chirp_real(short_chirp_real_reg),
    .short_chirp_imag(short_chirp_imag_reg),
    .range_profile_i(range_profile_i),
    .range_profile_q(range_profile_q),
    .range_profile_valid(range_profile_valid),
    .chain_state(chain_state)
);

initial clk = 1'b0;
always #5 clk = ~clk;

task check;
    input condition;
    input [255:0] msg;
    begin
        if (condition) begin
            $display("[PASS] R%0d: %0s", test_num, msg);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] R%0d: %0s", test_num, msg);
            fail_count = fail_count + 1;
        end
    end
endtask

function integer abs16;
    input signed [15:0] v;
    begin
        if (v < 0)
            abs16 = -v;
        else
            abs16 = v;
    end
endfunction

function integer clamp16;
    input integer v;
    begin
        if (v > 32767)
            clamp16 = 32767;
        else if (v < -32768)
            clamp16 = -32768;
        else
            clamp16 = v;
    end
endfunction

task apply_reset;
    begin
        reset_n <= 1'b0;
        adc_data_i_reg <= 16'd0;
        adc_data_q_reg <= 16'd0;
        adc_valid_reg <= 1'b0;
        chirp_counter_reg <= 6'd0;
        long_chirp_real_reg <= 16'd0;
        long_chirp_imag_reg <= 16'd0;
        short_chirp_real_reg <= 16'd0;
        short_chirp_imag_reg <= 16'd0;
        repeat (5) @(posedge clk);
        reset_n <= 1'b1;
        repeat (5) @(posedge clk);
    end
endtask

function signed [7:0] lfsr_noise;
    input integer amp;
    reg fb;
    integer raw;
    integer s;
    begin
        fb = lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0];
        lfsr = {lfsr[30:0], fb};
        raw = lfsr[7:0];
        s = raw - 128;
        s = (s * amp) / 128;
        if (s > 127) s = 127;
        if (s < -128) s = -128;
        lfsr_noise = s[7:0];
    end
endfunction

task run_frame_and_capture;
    input integer delay1;
    input integer amp1;
    input integer delay2;
    input integer amp2;
    input integer noise_amp;
    integer k;
    integer si;
    integer sq;
    integer ni;
    integer nq;
    integer wait_cycles;
    begin
        output_count = 0;

        for (k = 0; k < N; k = k + 1) begin
            si = 0;
            sq = 0;

            if (delay1 >= 0 && k >= delay1) begin
                si = si + ((amp1 * chirp_ref_i[k - delay1]) / CHIRP_SCALE);
                sq = sq + ((amp1 * chirp_ref_q[k - delay1]) / CHIRP_SCALE);
            end

            if (delay2 >= 0 && k >= delay2) begin
                si = si + ((amp2 * chirp_ref_i[k - delay2]) / CHIRP_SCALE);
                sq = sq + ((amp2 * chirp_ref_q[k - delay2]) / CHIRP_SCALE);
            end

            if (noise_amp > 0) begin
                ni = lfsr_noise(noise_amp);
                nq = lfsr_noise(noise_amp);
                si = si + ni;
                sq = sq + nq;
            end

            @(posedge clk);
            adc_valid_reg <= 1'b1;
            chirp_counter_reg <= 6'd0;
            long_chirp_real_reg <= chirp_ref_i[k];
            long_chirp_imag_reg <= chirp_ref_q[k];
            short_chirp_real_reg <= chirp_ref_i[k];
            short_chirp_imag_reg <= chirp_ref_q[k];
            adc_data_i_reg <= clamp16(si);
            adc_data_q_reg <= clamp16(sq);
        end

        @(posedge clk);
        adc_valid_reg <= 1'b0;
        adc_data_i_reg <= 16'd0;
        adc_data_q_reg <= 16'd0;
        long_chirp_real_reg <= 16'd0;
        long_chirp_imag_reg <= 16'd0;
        short_chirp_real_reg <= 16'd0;
        short_chirp_imag_reg <= 16'd0;

        wait_cycles = 0;
        while (output_count < N && wait_cycles < 250000) begin
            @(posedge clk);
            wait_cycles = wait_cycles + 1;
            if (range_profile_valid) begin
                out_i[output_count] = range_profile_i;
                out_q[output_count] = range_profile_q;
                out_mag[output_count] = abs16(range_profile_i) + abs16(range_profile_q);
                output_count = output_count + 1;
            end
        end

        while (chain_state != 4'd0 && wait_cycles < 260000) begin
            @(posedge clk);
            wait_cycles = wait_cycles + 1;
        end
    end
endtask

task find_peak;
    output integer peak_bin;
    output integer peak_mag;
    integer i;
    begin
        peak_bin = 0;
        peak_mag = 0;
        for (i = 0; i < output_count; i = i + 1) begin
            if ($signed({1'b0, out_mag[i]}) > peak_mag) begin
                peak_mag = out_mag[i];
                peak_bin = i;
            end
        end
    end
endtask

task find_top_two_peaks;
    output integer p1_bin;
    output integer p1_mag;
    output integer p2_bin;
    output integer p2_mag;
    integer i;
    integer best2_bin;
    integer sep;
    begin
        find_peak(p1_bin, p1_mag);
        p2_bin = 0;
        p2_mag = 0;
        best2_bin = 0;

        for (i = 0; i < output_count; i = i + 1) begin
            sep = i - p1_bin;
            if (sep < 0) sep = -sep;
            if (sep > (N - sep)) sep = N - sep;
            if (sep > 2) begin
                if ($signed({1'b0, out_mag[i]}) > p2_mag) begin
                    p2_mag = out_mag[i];
                    best2_bin = i;
                end
            end
        end

        p2_bin = best2_bin;
    end
endtask

task compute_noise_floor;
    input integer peak_bin;
    output integer avg_noise;
    output integer samples_used;
    integer i;
    integer sep;
    integer sum;
    begin
        sum = 0;
        samples_used = 0;
        for (i = 0; i < output_count; i = i + 1) begin
            sep = i - peak_bin;
            if (sep < 0) sep = -sep;
            if (sep > (N - sep)) sep = N - sep;
            if (sep > 2) begin
                sum = sum + out_mag[i];
                samples_used = samples_used + 1;
            end
        end
        if (samples_used > 0)
            avg_noise = sum / samples_used;
        else
            avg_noise = 0;
    end
endtask

task find_max_output_mag;
    output integer max_mag;
    integer i;
    begin
        max_mag = 0;
        for (i = 0; i < output_count; i = i + 1) begin
            if (out_mag[i] > max_mag)
                max_mag = out_mag[i];
        end
    end
endtask

initial begin
    $dumpfile("tb_range_compression_physics.vcd");
    $dumpvars(0, tb_range_compression_physics);

    pass_count = 0;
    fail_count = 0;
    test_num = 0;
    lfsr = 32'h1ACE_B00C;

    chirp_rate = 3.14159265 * 50.0e6 / (1024.0 * 100.0e6);
    for (n = 0; n < N; n = n + 1) begin
        phase = chirp_rate * n * n;
        chirp_ref_i[n] = $rtoi(8000.0 * $cos(phase));
        chirp_ref_q[n] = $rtoi(8000.0 * $sin(phase));
    end

    $display("");
    $display("============================================================");
    $display("  AERIS-10 Range Compression Physics Testbench");
    $display("============================================================");

    // R1: Point target at known delay
    begin
        integer peak_bin;
        integer peak_mag;
        test_num = 1;
        apply_reset;
        run_frame_and_capture(100, CHIRP_SCALE, -1, 0, 0);
        find_peak(peak_bin, peak_mag);
        $display("  R1: peak_bin=%0d, peak_mag=%0d, output_count=%0d", peak_bin, peak_mag, output_count);
        // Debug: dump magnitudes around expected peak (bin 100) and actual peak
        $display("  R1 debug: out_mag[0]=%0d [50]=%0d [98]=%0d [99]=%0d [100]=%0d [101]=%0d [102]=%0d [145]=%0d [200]=%0d",
                 out_mag[0], out_mag[50], out_mag[98], out_mag[99], out_mag[100], out_mag[101], out_mag[102], out_mag[145], out_mag[200]);
        // Dump first 10 outputs
        $display("  R1 first10: [0]=%0d [1]=%0d [2]=%0d [3]=%0d [4]=%0d [5]=%0d [6]=%0d [7]=%0d [8]=%0d [9]=%0d",
                 out_mag[0], out_mag[1], out_mag[2], out_mag[3], out_mag[4], out_mag[5], out_mag[6], out_mag[7], out_mag[8], out_mag[9]);
        check(output_count == N, "Captured 1024 output samples");
        check((peak_bin >= 98) && (peak_bin <= 102), "Point-target peak near delay bin 100 (+/-2)");
    end

    // R2: Two close targets, well-resolved
    begin
        integer p1_bin;
        integer p1_mag;
        integer p2_bin;
        integer p2_mag;
        integer ok_pair;
        test_num = 2;
        apply_reset;
        run_frame_and_capture(400, CHIRP_SCALE, 416, CHIRP_SCALE, 0);
        find_top_two_peaks(p1_bin, p1_mag, p2_bin, p2_mag);

        $display("  R2: p1_bin=%0d, p1_mag=%0d, p2_bin=%0d, p2_mag=%0d", p1_bin, p1_mag, p2_bin, p2_mag);

        ok_pair = 0;
        if (((p1_bin >= 398) && (p1_bin <= 402) && (p2_bin >= 414) && (p2_bin <= 418)) ||
            ((p2_bin >= 398) && (p2_bin <= 402) && (p1_bin >= 414) && (p1_bin <= 418)))
            ok_pair = 1;

        check(output_count == N, "Captured 1024 output samples");
        check(ok_pair == 1, "Two distinct peaks near bins 400 and 416");
    end

    // R3: Processing gain / SNR
    begin
        integer peak_bin;
        integer peak_mag;
        integer avg_noise;
        integer n_noise;
        integer ratio;
        test_num = 3;
        apply_reset;
        run_frame_and_capture(100, CHIRP_SCALE, -1, 0, 10);
        find_peak(peak_bin, peak_mag);
        compute_noise_floor(peak_bin, avg_noise, n_noise);
        if (avg_noise > 0)
            ratio = peak_mag / avg_noise;
        else
            ratio = 0;

        $display("  R3: peak_bin=%0d, peak_mag=%0d, avg_noise=%0d, ratio=%0d", peak_bin, peak_mag, avg_noise, ratio);

        check(output_count == N, "Captured 1024 output samples");
        check(ratio > 20, "Peak-to-noise ratio exceeds 20 (~26 dB, limited by 16-bit behavioral FFT)");
    end

    // R4: Zero input
    begin
        integer max_mag;
        test_num = 4;
        apply_reset;
        run_frame_and_capture(-1, 0, -1, 0, 0);
        find_max_output_mag(max_mag);
        check(output_count == N, "Captured 1024 output samples");
        check(max_mag < 10, "Zero-input output magnitude remains near zero (<10)");
    end

    // R5: Identity (signal equals reference)
    begin
        integer peak_bin;
        integer peak_mag;
        test_num = 5;
        apply_reset;
        run_frame_and_capture(0, CHIRP_SCALE, -1, 0, 0);
        find_peak(peak_bin, peak_mag);
        $display("  R5: peak_bin=%0d, peak_mag=%0d", peak_bin, peak_mag);
        check(output_count == N, "Captured 1024 output samples");
        check(peak_bin == 0, "Autocorrelation peak at bin 0 for zero delay");
    end

    // R6: Sidelobe level
    begin
        integer p1_bin;
        integer p1_mag;
        integer p2_bin;
        integer p2_mag;
        integer ratio;
        test_num = 6;
        apply_reset;
        run_frame_and_capture(100, CHIRP_SCALE, -1, 0, 0);
        find_top_two_peaks(p1_bin, p1_mag, p2_bin, p2_mag);
        $display("  R6: p1_bin=%0d, p1_mag=%0d, p2_bin=%0d, p2_mag=%0d", p1_bin, p1_mag, p2_bin, p2_mag);
        if (p2_mag > 0)
            ratio = p1_mag / p2_mag;
        else
            ratio = 999999;
        check(output_count == N, "Captured 1024 output samples");
        check(ratio > 3, "Peak-to-highest sidelobe ratio exceeds 3 (~10 dB)");
    end

    $display("");
    $display("============================================================");
    $display("  RANGE COMPRESSION PHYSICS RESULTS");
    $display("  Passed: %0d  Failed: %0d  Total: %0d", pass_count, fail_count, pass_count + fail_count);
    $display("============================================================");
    $display("");

    $finish;
end

initial begin
    #200_000_000;
    $display("[FAIL] R0: Timeout watchdog hit at 200ms");
    $finish;
end

endmodule
