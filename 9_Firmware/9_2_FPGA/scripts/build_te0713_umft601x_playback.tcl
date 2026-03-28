# build_te0713_umft601x_playback.tcl
#
# Vivado batch build for the UNIFIED playback + USB streaming design.
# Combines BRAM playback DSP pipeline with FT601 USB interface.
#
# Target: TE0713 (XC7A200T-2FBG484) + TE0701 carrier + UMFT601X-B
# Clock: ft601_clk_in (100 MHz) — single clock domain, no STARTUPE2
# No VIO — all control via USB host commands
#
# Usage:
#   vivado -mode batch -source scripts/build_te0713_umft601x_playback.tcl

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_umft601x_playback"]
set reports_dir [file join $build_dir "reports"]

file mkdir $build_dir
file mkdir $reports_dir

# ==========================================================================
# Create project
# ==========================================================================
puts "INFO: Creating project..."
create_project -force aeris10_te0713_umft601x_playback $build_dir \
    -part xc7a200tfbg484-2

set_property target_language Verilog [current_project]

# ==========================================================================
# Add source files
# ==========================================================================
puts "INFO: Adding source files..."
set rtl_files [list \
    [file join $project_root "radar_system_top_te0713_umft601x_playback.v"] \
    [file join $project_root "usb_data_interface.v"] \
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

set_property top radar_system_top_te0713_umft601x_playback [current_fileset]

# Define FFT_XPM_BRAM so fft_engine uses XPM TDP BRAM instead of behavioral model
set_property verilog_define {FFT_XPM_BRAM} [current_fileset]

# ==========================================================================
# Add .mem files for FFT twiddle BRAM initialization
# ==========================================================================
set mem_files [glob -nocomplain [file join $project_root "*.mem"]]
foreach f $mem_files {
    add_files $f
    puts "INFO: Added mem file: $f"
}

# ==========================================================================
# Copy hex data file for BRAM initialization
# ==========================================================================
set hex_file [file join $project_root "tb" "cosim" "real_data" "hex" "fullchain_range_input.hex"]
if {[file exists $hex_file]} {
    file copy -force $hex_file [file join $build_dir "fullchain_range_input.hex"]
    file copy -force $hex_file [file join $project_root "fullchain_range_input.hex"]
    puts "INFO: Copied fullchain_range_input.hex to build directory"
} else {
    error "Hex file not found: $hex_file"
}

# ==========================================================================
# Add constraints — reuse the FMC FT601 XDC (same pinout as dev wrapper)
# ==========================================================================
puts "INFO: Adding constraints..."
set xdc_file [file join $project_root "constraints" "te0713_te0701_umft601x.xdc"]
if {![file exists $xdc_file]} {
    error "XDC not found: $xdc_file"
}
add_files -fileset constrs_1 -norecurse $xdc_file

update_compile_order -fileset sources_1

# ==========================================================================
# Use Performance_ExplorePostRoutePhysOpt strategy for timing closure
# ==========================================================================
set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]

# ==========================================================================
# Run Synthesis + Implementation + Bitstream
# ==========================================================================
puts "INFO: Launching implementation to bitstream (Performance_ExplorePostRoutePhysOpt)..."
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set impl_status [get_property STATUS [get_runs impl_1]]
puts "INFO: impl_1 status: $impl_status"

if {![string match "*Complete*" $impl_status]} {
    set impl_log [file join $build_dir "aeris10_te0713_umft601x_playback.runs" "impl_1" "runme.log"]
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
    error "Implementation did not complete successfully. Status: $impl_status"
}

# ==========================================================================
# Reports
# ==========================================================================
puts "INFO: Generating reports..."
open_run impl_1

report_clocks -file [file join $reports_dir "clocks.rpt"]
report_clock_interaction -file [file join $reports_dir "clock_interaction.rpt"]
report_timing_summary -report_unconstrained -max_paths 100 \
    -file [file join $reports_dir "timing_summary.rpt"]
report_cdc -details -file [file join $reports_dir "cdc.rpt"]
report_exceptions -file [file join $reports_dir "exceptions.rpt"]
report_drc -file [file join $reports_dir "drc.rpt"]
report_utilization -file [file join $reports_dir "utilization.rpt"]

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
set bit_files [glob -nocomplain [file join $build_dir "aeris10_te0713_umft601x_playback.runs" "impl_1" "*.bit"]]

puts ""
puts "============================================================"
puts "  UNIFIED PLAYBACK + USB BUILD COMPLETE (v8)"
puts "============================================================"
puts "  Bitstream: $bit_files"
puts "  Reports:   $reports_dir"
puts ""
puts "  Clock:     ft601_clk_in (100 MHz, single domain)"
puts "  Pipeline:  BRAM -> Decim -> MTI -> Doppler -> DC Notch -> CFAR -> USB"
puts "  BRAM data: fullchain_range_input.hex (32768 x 32-bit)"
puts "  Control:   USB host commands (no VIO)"
puts "============================================================"
