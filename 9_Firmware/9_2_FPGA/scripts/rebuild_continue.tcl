# rebuild_continue.tcl
#
# Continuation script: synthesis already completed with STARTUPE2 RTL.
# This script opens the synthesized design, configures debug hub,
# and runs implementation through bitstream.
#
# Usage:
#   vivado -mode batch -source scripts/rebuild_continue.tcl

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_vio"]
set reports_dir [file join $build_dir "reports"]

file mkdir $reports_dir

# ==========================================================================
# Open existing project
# ==========================================================================
puts "INFO: Opening existing project..."
open_project [file join $build_dir "aeris10_te0713_vio.xpr"]

# ==========================================================================
# Open synthesized design and configure debug hub
# ==========================================================================
puts "INFO: Opening synthesized design..."
open_run synth_1

# Verify STARTUPE2 is present
set startupe2_cells [get_cells -hier -filter {REF_NAME == STARTUPE2}]
puts "INFO: STARTUPE2 cells: $startupe2_cells"
if {[llength $startupe2_cells] == 0} {
    error "STARTUPE2 NOT found — RTL not updated!"
}

# Verify BUFG clock path
set bufg_cells [get_cells -hier -filter {REF_NAME == BUFG}]
foreach bg $bufg_cells {
    set i_net [get_nets -of_objects [get_pins $bg/I]]
    set o_net [get_nets -of_objects [get_pins $bg/O]]
    puts "  BUFG $bg: I=$i_net, O=$o_net"
}

# Configure debug hub
puts "INFO: Configuring debug hub: USER chain 2, 65 MHz..."
set_property C_USER_SCAN_CHAIN 2 [get_debug_cores dbg_hub]
set_property C_CLK_INPUT_FREQ_HZ 65000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_buf]

puts "  C_USER_SCAN_CHAIN  = [get_property C_USER_SCAN_CHAIN [get_debug_cores dbg_hub]]"
puts "  C_CLK_INPUT_FREQ_HZ = [get_property C_CLK_INPUT_FREQ_HZ [get_debug_cores dbg_hub]]"

# Save constraints
puts "INFO: Saving constraints..."
save_constraints

implement_debug_core
close_design

# ==========================================================================
# Reset and run implementation through write_bitstream
# ==========================================================================
puts "INFO: Resetting impl_1..."
reset_run impl_1

puts "INFO: Launching implementation through write_bitstream..."
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set impl_status [get_property STATUS [get_runs impl_1]]
puts "INFO: impl_1 status: $impl_status"

if {![string match "*Complete*" $impl_status]} {
    set impl_log [file join $build_dir "aeris10_te0713_vio.runs" "impl_1" "runme.log"]
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
# Reports & verification
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

# Report bitstream location
set bit_files [glob -nocomplain [file join $build_dir "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
set ltx_files [glob -nocomplain [file join $build_dir "aeris10_te0713_vio.runs" "impl_1" "*.ltx"]]

puts ""
puts "============================================================"
puts "  BUILD COMPLETE"
puts "============================================================"
puts "  STARTUPE2: [llength $startupe2_final] cell(s) in design"
puts "  Bitstream: $bit_files"
puts "  Probes:    $ltx_files"
puts "  Reports:   $reports_dir"
puts ""
puts "  Debug hub: USER_SCAN_CHAIN=2, CLK=65MHz (STARTUPE2 internal)"
puts ""
puts "  Next step:"
puts "    vivado -mode batch -source scripts/read_vio_selftest.tcl"
puts "============================================================"
