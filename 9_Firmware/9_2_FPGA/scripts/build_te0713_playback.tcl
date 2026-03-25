# build_te0713_playback.tcl
#
# Creates a new Vivado project for the BRAM playback build, instantiates
# a VIO IP with the correct probe configuration, synthesizes, implements,
# and generates a bitstream.
#
# VIO "vio_playback": 16 input probes, 5 output probes
# Debug hub: USER scan chain 3, clock divider enabled (for STARTUPE2 ~65 MHz)
#
# Usage:
#   vivado -mode batch -source scripts/build_te0713_playback.tcl

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_playback"]
set reports_dir [file join $build_dir "reports"]
set ip_dir [file join $build_dir "ip"]

file mkdir $build_dir
file mkdir $reports_dir
file mkdir $ip_dir

# ==========================================================================
# Create project
# ==========================================================================
puts "INFO: Creating project..."
create_project aeris10_te0713_playback $build_dir \
    -part xc7a200tfbg484-2 -force

set_property target_language Verilog [current_project]

# ==========================================================================
# Add source files
# ==========================================================================
puts "INFO: Adding source files..."
set rtl_files [list \
    [file join $project_root "radar_system_top_te0713_playback.v"] \
    [file join $project_root "bram_playback.v"] \
    [file join $project_root "range_bin_decimator.v"] \
    [file join $project_root "mti_canceller.v"] \
    [file join $project_root "doppler_processor.v"] \
    [file join $project_root "xfft_16.v"] \
    [file join $project_root "xfft_32.v"] \
    [file join $project_root "fft_engine.v"] \
    [file join $project_root "cfar_ca.v"] \
    [file join $project_root "fpga_self_test.v"] \
]

foreach f $rtl_files {
    if {![file exists $f]} {
        error "Source file not found: $f"
    }
    add_files $f
}

set_property top radar_system_top_te0713_playback [current_fileset]

# Define FFT_XPM_BRAM so fft_engine uses XPM TDP BRAM instead of behavioral model
set_property verilog_define {FFT_XPM_BRAM} [current_fileset]

# Add hex data file for BRAM initialization
set hex_file [file join $project_root "tb" "cosim" "real_data" "hex" "fullchain_range_input.hex"]
if {[file exists $hex_file]} {
    # Copy hex to build dir so synthesis can find it
    file copy -force $hex_file [file join $build_dir "fullchain_range_input.hex"]
    # Also copy to project root for good measure
    file copy -force $hex_file [file join $project_root "fullchain_range_input.hex"]
    puts "INFO: Copied fullchain_range_input.hex to build directory"
} else {
    error "Hex file not found: $hex_file"
}

# ==========================================================================
# Add constraints
# ==========================================================================
puts "INFO: Adding constraints..."
add_files -fileset constrs_1 [file join $project_root "constraints" "te0713_playback.xdc"]

# ==========================================================================
# Create VIO IP: vio_playback
# ==========================================================================
puts "INFO: Creating VIO IP..."
create_ip -name vio -vendor xilinx.com -library ip -version 3.0 \
    -module_name vio_playback -dir $ip_dir

set_property -dict [list \
    CONFIG.C_PROBE_IN0_WIDTH  {16} \
    CONFIG.C_PROBE_IN1_WIDTH  {6}  \
    CONFIG.C_PROBE_IN2_WIDTH  {5}  \
    CONFIG.C_PROBE_IN3_WIDTH  {17} \
    CONFIG.C_PROBE_IN4_WIDTH  {1}  \
    CONFIG.C_PROBE_IN5_WIDTH  {1}  \
    CONFIG.C_PROBE_IN6_WIDTH  {1}  \
    CONFIG.C_PROBE_IN7_WIDTH  {1}  \
    CONFIG.C_PROBE_IN8_WIDTH  {1}  \
    CONFIG.C_PROBE_IN9_WIDTH  {6}  \
    CONFIG.C_PROBE_IN10_WIDTH {32} \
    CONFIG.C_PROBE_IN11_WIDTH {5}  \
    CONFIG.C_PROBE_IN12_WIDTH {8}  \
    CONFIG.C_PROBE_IN13_WIDTH {1}  \
    CONFIG.C_PROBE_IN14_WIDTH {17} \
    CONFIG.C_PROBE_IN15_WIDTH {8}  \
    CONFIG.C_PROBE_OUT0_WIDTH {1}  \
    CONFIG.C_PROBE_OUT1_WIDTH {1}  \
    CONFIG.C_PROBE_OUT2_WIDTH {1}  \
    CONFIG.C_PROBE_OUT3_WIDTH {3}  \
    CONFIG.C_PROBE_OUT4_WIDTH {1}  \
    CONFIG.C_NUM_PROBE_IN     {16} \
    CONFIG.C_NUM_PROBE_OUT    {5}  \
    CONFIG.C_PROBE_OUT0_INIT_VAL {0x0} \
    CONFIG.C_PROBE_OUT1_INIT_VAL {0x0} \
    CONFIG.C_PROBE_OUT2_INIT_VAL {0x0} \
    CONFIG.C_PROBE_OUT3_INIT_VAL {0x0} \
    CONFIG.C_PROBE_OUT4_INIT_VAL {0x0} \
    CONFIG.C_EN_PROBE_IN_ACTIVITY {0}  \
] [get_ips vio_playback]

generate_target all [get_ips vio_playback]

puts "INFO: Synthesizing VIO IP..."
synth_ip [get_ips vio_playback]

# ==========================================================================
# Run Synthesis
# ==========================================================================
puts "INFO: Launching synthesis..."
launch_runs synth_1 -jobs 8
wait_on_run synth_1

set synth_status [get_property STATUS [get_runs synth_1]]
puts "INFO: synth_1 status: $synth_status"
if {![string match "*Complete*" $synth_status]} {
    error "Synthesis failed: $synth_status"
}

# ==========================================================================
# Configure debug hub for USER scan chain 3 + clock divider
# ==========================================================================
puts "INFO: Configuring debug hub..."
open_run synth_1

# Verify STARTUPE2
set startupe2_cells [get_cells -hier -filter {REF_NAME == STARTUPE2}]
puts "INFO: STARTUPE2 cells: $startupe2_cells"
if {[llength $startupe2_cells] == 0} {
    error "STARTUPE2 NOT found!"
}

# Configure debug hub — chain 3, clock divider enabled
set_property C_USER_SCAN_CHAIN 3 [get_debug_cores dbg_hub]
set_property C_CLK_INPUT_FREQ_HZ 65000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER true [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_buf]

puts "  C_USER_SCAN_CHAIN    = [get_property C_USER_SCAN_CHAIN [get_debug_cores dbg_hub]]"
puts "  C_CLK_INPUT_FREQ_HZ  = [get_property C_CLK_INPUT_FREQ_HZ [get_debug_cores dbg_hub]]"
puts "  C_ENABLE_CLK_DIVIDER = [get_property C_ENABLE_CLK_DIVIDER [get_debug_cores dbg_hub]]"

save_constraints
implement_debug_core
close_design

# ==========================================================================
# Run Implementation + Bitstream
# ==========================================================================
puts "INFO: Launching implementation through write_bitstream..."
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set impl_status [get_property STATUS [get_runs impl_1]]
puts "INFO: impl_1 status: $impl_status"

if {![string match "*Complete*" $impl_status]} {
    set impl_log [file join $build_dir "aeris10_te0713_playback.runs" "impl_1" "runme.log"]
    if {[file exists $impl_log]} {
        puts "INFO: Last 50 lines of impl log:"
        set f [open $impl_log r]
        set lines [split [read $f] "\n"]
        close $f
        set start [expr {max(0, [llength $lines] - 50)}]
        for {set i $start} {$i < [llength $lines]} {incr i} {
            puts "  [lindex $lines $i]"
        }
    }
    error "Implementation failed: $impl_status"
}

# ==========================================================================
# Reports
# ==========================================================================
puts "INFO: Generating reports..."
open_run impl_1

report_clocks -file [file join $reports_dir "clocks.rpt"]
report_timing_summary -report_unconstrained -max_paths 100 \
    -file [file join $reports_dir "timing_summary.rpt"]
report_drc -file [file join $reports_dir "drc.rpt"]
report_utilization -file [file join $reports_dir "utilization.rpt"]

# Verify STARTUPE2 in final design
set startupe2_final [get_cells -hier -filter {REF_NAME == STARTUPE2}]
puts "INFO: STARTUPE2 in final design: $startupe2_final"

# Report BRAM utilization specifically
puts ""
puts "BRAM Utilization:"
set bram_report [report_utilization -return_string]
foreach line [split $bram_report "\n"] {
    if {[string match "*BRAM*" $line] || [string match "*Block RAM*" $line]} {
        puts "  $line"
    }
}

# Bitstream location
set bit_files [glob -nocomplain [file join $build_dir "aeris10_te0713_playback.runs" "impl_1" "*.bit"]]
set ltx_files [glob -nocomplain [file join $build_dir "aeris10_te0713_playback.runs" "impl_1" "*.ltx"]]

puts ""
puts "============================================================"
puts "  PLAYBACK BUILD COMPLETE"
puts "============================================================"
puts "  STARTUPE2: [llength $startupe2_final] cell(s)"
puts "  Bitstream: $bit_files"
puts "  Probes:    $ltx_files"
puts "  Reports:   $reports_dir"
puts ""
puts "  Debug hub: USER_SCAN_CHAIN=3, CLK=65MHz, CLK_DIVIDER=enabled"
puts "  VIO: 16 input probes, 5 output probes"
puts "  Pipeline: BRAM -> Decim -> MTI -> Doppler -> DC Notch -> CFAR"
puts "  BRAM data: fullchain_range_input.hex (32768 x 32-bit)"
puts ""
puts "  To read results:"
puts "    vivado -mode batch -source scripts/read_playback_results.tcl"
puts "============================================================"
