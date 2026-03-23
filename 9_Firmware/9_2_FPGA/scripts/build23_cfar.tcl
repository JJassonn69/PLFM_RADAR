################################################################################
# build23_cfar.tcl
#
# AERIS-10 Build 23: CA-CFAR Integration + Production Fixes 1-7
# Target: XC7A200T-2FBG484I
# Design: radar_system_top
# Base:   Build 21 (v0.1.4-build21, WNS +0.156 ns, WHS +0.064 ns)
#
# Changes vs Build 21:
#   - cfar_ca.v: new CA/GO/SO-CFAR detector module with BRAM magnitude buffer
#     (2048x17), sliding-window noise estimation, host-configurable parameters.
#     Replaces old fixed-threshold magnitude comparator in radar_system_top.v.
#   - rx_gain_control.v: new digital gain control (power-of-2 shift) [Fix 3].
#   - radar_system_top.v: CFAR instantiation, 5 new host config registers
#     (0x21-0x25), registered bridge to detection outputs, frame_complete wiring.
#     Production fixes 1-7 (detection bugs, naming, gain, Doppler clamp,
#     decimator watchdog, bypass removal, range-mode register).
#   - radar_receiver_final.v: doppler_frame_done_out output port for CFAR.
#   - range_bin_decimator.v: watchdog timer [Fix 5].
#   - ddc_400m.v: minor fix.
#   - usb_data_interface.v: stream control + data_pending fixes.
#
# Expected impact vs Build 21:
#   - LUTs: 6192 -> ~6500 (+~300 from CFAR logic + gain control)
#   - FFs:  9064 -> ~9300 (+~240 from CFAR pipeline + gain regs)
#   - BRAM: 16 -> 17 (+1 from CFAR 2048x17 magnitude buffer)
#   - DSP:  139 -> 140 (+1 from CFAR alpha multiplier)
#   - Timing: should remain positive. CFAR is single-clock-domain (clk_100m),
#     no new CDC paths introduced.
#
# Generates ALL reports required for the 15-point Vivado TCL Build Report.
#
# Usage:
#   vivado -mode batch -source build23_cfar.tcl
#
# Author: auto-generated for Jason Stone
# Date:   2026-03-20
################################################################################

# ==============================================================================
# 0. Configuration
# ==============================================================================

set project_name    "aeris10_radar"
set project_dir     "/home/jason-stone/PLFM_RADAR_work/vivado_project"
set rtl_dir         "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set top_module      "radar_system_top"
set fpga_part       "xc7a200tfbg484-2"
set report_dir      "${project_dir}/reports_build23"
set sim_dir         "${project_dir}/sim"
set bitstream_dir   "${project_dir}/bitstream"
set build_tag       "build23"

file mkdir $report_dir
file mkdir $sim_dir
file mkdir $bitstream_dir

# Record start time
set build_start [clock seconds]
set build_timestamp [clock format $build_start -format {%Y-%m-%d %H:%M:%S}]

puts "================================================================"
puts "  AERIS-10 Build 23: CA-CFAR Integration + Production Fixes"
puts "  Target:    $fpga_part"
puts "  Top:       $top_module"
puts "  Reports:   $report_dir"
puts "  Started:   $build_timestamp"
puts "================================================================"

# ==============================================================================
# 1. Project Creation + Source Files
# ==============================================================================

create_project $project_name $project_dir -part $fpga_part -force
set_property target_language Verilog [current_project]

# --- Add RTL sources ---
set rtl_files [list \
    "${rtl_dir}/adc_clk_mmcm.v" \
    "${rtl_dir}/ad9484_interface_400m.v" \
    "${rtl_dir}/cdc_modules.v" \
    "${rtl_dir}/cfar_ca.v" \
    "${rtl_dir}/chirp_lut_init.v" \
    "${rtl_dir}/chirp_memory_loader_param.v" \
    "${rtl_dir}/cic_decimator_4x_enhanced.v" \
    "${rtl_dir}/dac_interface_single.v" \
    "${rtl_dir}/ddc_400m.v" \
    "${rtl_dir}/ddc_input_interface.v" \
    "${rtl_dir}/doppler_processor.v" \
    "${rtl_dir}/edge_detector.v" \
    "${rtl_dir}/fft_1024_forward.v" \
    "${rtl_dir}/fft_1024_inverse.v" \
    "${rtl_dir}/fir_lowpass.v" \
    "${rtl_dir}/frequency_matched_filter.v" \
    "${rtl_dir}/latency_buffer.v" \
    "${rtl_dir}/level_shifter_interface.v" \
    "${rtl_dir}/lvds_to_cmos_400m.v" \
    "${rtl_dir}/matched_filter_multi_segment.v" \
    "${rtl_dir}/matched_filter_processing_chain.v" \
    "${rtl_dir}/nco_400m_enhanced.v" \
    "${rtl_dir}/plfm_chirp_controller.v" \
    "${rtl_dir}/radar_mode_controller.v" \
    "${rtl_dir}/radar_receiver_final.v" \
    "${rtl_dir}/radar_system_top.v" \
    "${rtl_dir}/radar_transmitter.v" \
    "${rtl_dir}/range_bin_decimator.v" \
    "${rtl_dir}/rx_gain_control.v" \
    "${rtl_dir}/usb_data_interface.v" \
    "${rtl_dir}/usb_packet_analyzer.v" \
    "${rtl_dir}/xfft_32.v" \
    "${rtl_dir}/fft_engine.v" \
]

set file_count 0
foreach f $rtl_files {
    if {[file exists $f]} {
        add_files -norecurse $f
        incr file_count
    } else {
        puts "  WARNING: RTL file not found: $f"
    }
}
puts "  Added $file_count RTL files"

# Add .mem files for BRAM initialization
set mem_files [glob -nocomplain "${rtl_dir}/*.mem"]
foreach f $mem_files {
    add_files -norecurse $f
    puts "  Added MEM: [file tail $f]"
}

# Add constraints
add_files -fileset constrs_1 -norecurse "${project_dir}/synth_only.xdc"
add_files -fileset constrs_1 -norecurse "${rtl_dir}/constraints/adc_clk_mmcm.xdc"

set_property top $top_module [current_fileset]
set_property verilog_define {FFT_XPM_BRAM} [current_fileset]

# ==============================================================================
# 2. Synthesis
# ==============================================================================

puts ""
puts "================================================================"
puts "  Phase 1/5: Synthesis"
puts "================================================================"

set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.KEEP_EQUIVALENT_REGISTERS true [get_runs synth_1]

set synth_start [clock seconds]
launch_runs synth_1 -jobs 8
wait_on_run synth_1
set synth_elapsed [expr {[clock seconds] - $synth_start}]

set synth_status [get_property STATUS [get_runs synth_1]]
puts "  Synthesis status: $synth_status"
puts "  Synthesis time:   ${synth_elapsed}s ([expr {$synth_elapsed/60}]m [expr {$synth_elapsed%60}]s)"

if {[string match "*ERROR*" $synth_status] || [string match "*FAILED*" $synth_status]} {
    puts "CRITICAL: SYNTHESIS FAILED - aborting build"
    close_project
    exit 1
}

# Post-synth timing (for comparison)
open_run synth_1 -name synth_1
report_timing_summary -delay_type min_max -max_paths 10 -file "${report_dir}/01_timing_post_synth.rpt"
report_utilization -file "${report_dir}/01_utilization_post_synth.rpt"
close_design

# ==============================================================================
# 3. Implementation (opt -> place -> phys_opt -> route -> post_route_phys_opt)
# ==============================================================================

puts ""
puts "================================================================"
puts "  Phase 2/5: Implementation"
puts "================================================================"

# Aggressive directives for best timing
set_property STEPS.OPT_DESIGN.ARGS.DIRECTIVE Explore [get_runs impl_1]
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraTimingOpt [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

set impl_start [clock seconds]
launch_runs impl_1 -jobs 8
wait_on_run impl_1
set impl_elapsed [expr {[clock seconds] - $impl_start}]

set impl_status [get_property STATUS [get_runs impl_1]]
puts "  Implementation status: $impl_status"
puts "  Implementation time:   ${impl_elapsed}s ([expr {$impl_elapsed/60}]m [expr {$impl_elapsed%60}]s)"

if {![string match "*Complete*" $impl_status]} {
    puts "CRITICAL: IMPLEMENTATION FAILED: $impl_status"
    close_project
    exit 1
}

# ==============================================================================
# 4. Bitstream Generation
# ==============================================================================

puts ""
puts "================================================================"
puts "  Phase 3/5: Bitstream Generation"
puts "================================================================"

set bit_start [clock seconds]
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
set bit_elapsed [expr {[clock seconds] - $bit_start}]
puts "  Bitstream time: ${bit_elapsed}s"

# Copy bitstream to known location
set bit_src "${project_dir}/aeris10_radar.runs/impl_1/${top_module}.bit"
if {[file exists $bit_src]} {
    file copy -force $bit_src "${bitstream_dir}/${top_module}_${build_tag}.bit"
    puts "  Bitstream: ${bitstream_dir}/${top_module}_${build_tag}.bit"
    puts "  Size: [file size $bit_src] bytes"
} else {
    puts "  WARNING: Bitstream file not found at $bit_src"
}

# ==============================================================================
# 5. Comprehensive Report Generation
# ==============================================================================

puts ""
puts "================================================================"
puts "  Phase 4/5: Report Generation (15-point checklist)"
puts "================================================================"

# Open the routed design for reporting
open_run impl_1 -name impl_1

# --- Checklist Item 2: Timing Summary ---
puts "  [2/15] Timing Summary..."
report_timing_summary -delay_type min_max -max_paths 100 \
    -report_unconstrained \
    -file "${report_dir}/02_timing_summary.rpt"

# --- Checklist Item 3: Clock Analysis ---
puts "  [3/15] Clock Analysis..."
report_clocks -file "${report_dir}/03_clocks.rpt"
report_clock_interaction -delay_type min_max \
    -file "${report_dir}/03_clock_interaction.rpt"
report_clock_networks -file "${report_dir}/03_clock_networks.rpt"

# --- Checklist Item 4: Utilization ---
puts "  [4/15] Utilization..."
report_utilization -file "${report_dir}/04_utilization.rpt"
report_utilization -hierarchical -file "${report_dir}/04_utilization_hierarchical.rpt"

# --- Checklist Item 5: Power ---
puts "  [5/15] Power Report..."
report_power -file "${report_dir}/05_power.rpt"

# --- Checklist Item 6: DRC ---
puts "  [6/15] DRC..."
report_drc -file "${report_dir}/06_drc.rpt"

# --- Checklist Item 7: IO and Constraints ---
puts "  [7/15] IO Report..."
report_io -file "${report_dir}/07_io.rpt"
report_timing -from [all_inputs] -to [all_outputs] -max_paths 20 \
    -file "${report_dir}/07_io_timing.rpt"

# --- Checklist Item 8: Congestion Analysis ---
puts "  [8/15] Congestion Analysis..."
report_design_analysis -congestion -file "${report_dir}/08_congestion.rpt"

# --- Checklist Item 9: Route Status ---
puts "  [9/15] Route Status..."
report_route_status -file "${report_dir}/09_route_status.rpt"

# --- Checklist Item 10: Critical Paths ---
puts "  [10/15] Critical Paths..."
report_timing -max_paths 20 -sort_by slack -nworst 5 \
    -file "${report_dir}/10_critical_paths_setup.rpt"
report_timing -delay_type min -max_paths 20 -sort_by slack -nworst 5 \
    -file "${report_dir}/10_critical_paths_hold.rpt"
report_high_fanout_nets -timing -load_type -max_nets 20 \
    -file "${report_dir}/10_high_fanout_nets.rpt"

# --- Checklist Item 11: QoR Summary ---
puts "  [11/15] QoR Summary..."
report_design_analysis -timing -file "${report_dir}/11_design_analysis_timing.rpt"
report_design_analysis -logic_level_distribution -file "${report_dir}/11_logic_level_dist.rpt"
report_methodology -file "${report_dir}/11_methodology.rpt"

# --- Checklist Item 12: CDC Analysis ---
puts "  [12/15] CDC Analysis..."
report_cdc -details -file "${report_dir}/12_cdc.rpt"

# --- Checklist Item 13: Log Scan (captured separately in build log) ---
puts "  [13/15] Log scan - see build23.log"

# --- Additional reports ---
puts "  [extra] Generating additional diagnostic reports..."

if {[catch {report_exceptions -file "${report_dir}/13_exceptions.rpt"} err]} {
    puts "  WARNING: report_exceptions failed: $err"
    puts "  (Known Vivado 2025.2 issue - non-critical)"
}
if {[catch {check_timing -verbose -file "${report_dir}/13_check_timing.rpt"} err]} {
    puts "  WARNING: check_timing failed: $err"
    puts "  (Known Vivado 2025.2 issue - non-critical)"
}

# Compile configuration summary into a single text file
set summary_fh [open "${report_dir}/00_build23_summary.txt" w]
puts $summary_fh "================================================================"
puts $summary_fh "  AERIS-10 Build 23 - CA-CFAR Integration + Production Fixes"
puts $summary_fh "================================================================"
puts $summary_fh ""
puts $summary_fh "Build Tag:       $build_tag"
puts $summary_fh "Build Timestamp: $build_timestamp"
puts $summary_fh "FPGA Part:       $fpga_part"
puts $summary_fh "Top Module:      $top_module"
puts $summary_fh "RTL Files:       $file_count"
puts $summary_fh "Synth Status:    $synth_status"
puts $summary_fh "Synth Time:      ${synth_elapsed}s"
puts $summary_fh "Impl Status:     $impl_status"
puts $summary_fh "Impl Time:       ${impl_elapsed}s"
puts $summary_fh "Bitstream Time:  ${bit_elapsed}s"
puts $summary_fh ""

# Extract key timing numbers
puts $summary_fh "--- Timing ---"
set wns [get_property STATS.WNS [current_design]]
set tns [get_property STATS.TNS [current_design]]
set whs [get_property STATS.WHS [current_design]]
set ths [get_property STATS.THS [current_design]]
set fail_ep [get_property STATS.TPWS [current_design]]
puts $summary_fh "  WNS:  $wns ns"
puts $summary_fh "  TNS:  $tns ns"
puts $summary_fh "  WHS:  $whs ns"
puts $summary_fh "  THS:  $ths ns"
puts $summary_fh ""
puts $summary_fh "  Build 21 Baseline: WNS = +0.156 ns, WHS = +0.064 ns"
puts $summary_fh "  Build 20 Baseline: WNS = +0.426 ns, WHS = +0.058 ns"
puts $summary_fh "  Delta WNS vs B21: [expr {$wns - 0.156}] ns"
puts $summary_fh "  Delta WHS vs B21: [expr {$whs - 0.064}] ns"
puts $summary_fh ""

# Extract utilization
puts $summary_fh "--- Utilization ---"
set lut_used   [llength [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ CLB.LUT.*}]]
set ff_used    [llength [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ CLB.FF.*}]]
set bram_used  [llength [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ BMEM.*}]]
set dsp_used   [llength [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ MULT.DSP.*}]]
puts $summary_fh "  LUTs:  $lut_used / 134600"
puts $summary_fh "  FFs:   $ff_used / 269200"
puts $summary_fh "  BRAM:  $bram_used cells"
puts $summary_fh "  DSP:   $dsp_used cells"
puts $summary_fh ""
puts $summary_fh "  Build 21 Baseline: LUTs=6192, FFs=9064, BRAM=16, DSP=139"
puts $summary_fh "  Expected Build 23: LUTs~6500, FFs~9300, BRAM=17, DSP=140"
puts $summary_fh "  (CFAR adds ~300 LUTs, ~240 FFs, 1 BRAM18K, 1 DSP48)"
puts $summary_fh ""

# Route status
set unrouted [llength [get_nets -hierarchical -filter {ROUTE_STATUS == UNROUTED}]]
puts $summary_fh "--- Route ---"
puts $summary_fh "  Unrouted nets: $unrouted"
puts $summary_fh ""

# MMCM usage
puts $summary_fh "--- MMCM Usage ---"
set mmcm_count [llength [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ CLOCK.MMCM.*}]]
puts $summary_fh "  MMCME2 used: $mmcm_count / 10"
puts $summary_fh "  Expected: 1 (adc_clk_mmcm jitter cleaner)"
puts $summary_fh ""

# Bitstream
if {[file exists $bit_src]} {
    puts $summary_fh "--- Bitstream ---"
    puts $summary_fh "  File: ${top_module}_${build_tag}.bit"
    puts $summary_fh "  Size: [file size $bit_src] bytes"
} else {
    puts $summary_fh "--- Bitstream ---"
    puts $summary_fh "  WARNING: NOT GENERATED"
}
puts $summary_fh ""

# Signoff
puts $summary_fh "--- Final Signoff ---"
set signoff_pass 1
if {$wns < 0} {
    puts $summary_fh "  FAIL: WNS = $wns (negative slack)"
    set signoff_pass 0
} else {
    puts $summary_fh "  PASS: WNS = $wns ns (no setup violations)"
}
if {$whs < 0} {
    puts $summary_fh "  FAIL: WHS = $whs (hold violation)"
    set signoff_pass 0
} else {
    puts $summary_fh "  PASS: WHS = $whs ns (no hold violations)"
}
if {$tns != 0} {
    puts $summary_fh "  FAIL: TNS = $tns (total negative slack)"
    set signoff_pass 0
} else {
    puts $summary_fh "  PASS: TNS = 0 ns"
}
if {$unrouted > 0} {
    puts $summary_fh "  FAIL: $unrouted unrouted nets"
    set signoff_pass 0
} else {
    puts $summary_fh "  PASS: All nets routed"
}
if {[file exists $bit_src]} {
    puts $summary_fh "  PASS: Bitstream generated"
} else {
    puts $summary_fh "  FAIL: No bitstream"
    set signoff_pass 0
}
puts $summary_fh ""

# Timing regression check vs Build 21
if {$wns < 0.0} {
    puts $summary_fh "  *** CRITICAL: WNS is NEGATIVE - timing violation! ***"
    puts $summary_fh "  *** CFAR integration may need pipelining or register retiming ***"
}
if {$whs < 0.0} {
    puts $summary_fh "  *** CRITICAL: WHS is NEGATIVE - hold violation! ***"
}

if {$signoff_pass} {
    puts $summary_fh "  *** SIGNOFF: PASS ***"
} else {
    puts $summary_fh "  *** SIGNOFF: FAIL ***"
}

close $summary_fh
puts "  Summary written to: ${report_dir}/00_build23_summary.txt"

# ==============================================================================
# 6. SDF + Timing Netlist (for post-route simulation)
# ==============================================================================

puts ""
puts "================================================================"
puts "  Phase 5/5: SDF + Timing Netlist"
puts "================================================================"

write_verilog -force -mode timesim "${sim_dir}/post_impl_timesim.v"
write_sdf -force "${sim_dir}/post_impl_timesim.sdf"

close_design
open_run synth_1 -name synth_1
write_verilog -force -mode funcsim "${sim_dir}/post_synth_funcsim.v"

# ==============================================================================
# Done
# ==============================================================================

set build_total [expr {[clock seconds] - $build_start}]
set build_end [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]

puts ""
puts "================================================================"
puts "  BUILD 23 COMPLETE"
puts "================================================================"
puts "  Started:    $build_timestamp"
puts "  Finished:   $build_end"
puts "  Total time: ${build_total}s ([expr {$build_total/60}]m [expr {$build_total%60}]s)"
puts "  Synth:      ${synth_elapsed}s"
puts "  Impl:       ${impl_elapsed}s"
puts "  Bitstream:  ${bit_elapsed}s"
puts "  Reports:    $report_dir"
puts "  Bitstream:  ${bitstream_dir}/${top_module}_${build_tag}.bit"
puts "  WNS: $wns ns | WHS: $whs ns | TNS: $tns ns"
puts "  Build 21 baseline: WNS +0.156 | WHS +0.064"
if {$signoff_pass} {
    puts "  SIGNOFF: PASS"
} else {
    puts "  SIGNOFF: FAIL"
}
puts "================================================================"

close_project
puts "Done."
