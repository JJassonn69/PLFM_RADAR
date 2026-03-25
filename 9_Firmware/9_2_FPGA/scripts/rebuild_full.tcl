# rebuild_full.tcl
#
# Full rebuild of the VIO bitstream after RTL changes.
# Opens the existing Vivado project, resets synthesis to force re-read of
# updated source files (STARTUPE2 internal clock), configures the debug hub
# on BSCAN USER chain 2 with 65 MHz clock, and rebuilds through bitstream.
#
# Usage:
#   vivado -mode batch -source scripts/rebuild_full.tcl

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_vio"]
set reports_dir [file join $build_dir "reports"]

file mkdir $reports_dir

# ==========================================================================
# Open existing project
# ==========================================================================
puts "INFO: Opening existing project..."
open_project [file join $build_dir "aeris10_te0713_vio.xpr"]

# Verify the source files are the updated versions
puts "INFO: Checking source files..."
set src_files [get_files -of_objects [get_filesets sources_1]]
foreach f $src_files {
    puts "  SRC: $f"
}
set xdc_files [get_files -of_objects [get_filesets constrs_1]]
foreach f $xdc_files {
    puts "  XDC: $f"
}

# ==========================================================================
# Reset and re-run synthesis (forces re-read of updated RTL)
# ==========================================================================
puts "INFO: Resetting synth_1 to force re-synthesis from updated RTL..."
reset_run synth_1

puts "INFO: Launching synthesis..."
launch_runs synth_1 -jobs 8
wait_on_run synth_1

set synth_status [get_property STATUS [get_runs synth_1]]
puts "INFO: synth_1 status: $synth_status"
if {![string match "*Complete*" $synth_status]} {
    # Dump synthesis log for debugging
    set synth_log [file join $build_dir "aeris10_te0713_vio.runs" "synth_1" "runme.log"]
    if {[file exists $synth_log]} {
        puts "INFO: Last 50 lines of synth log:"
        set f [open $synth_log r]
        set lines [split [read $f] "\n"]
        close $f
        set start [expr {max(0, [llength $lines] - 50)}]
        for {set i $start} {$i < [llength $lines]} {incr i} {
            puts "  [lindex $lines $i]"
        }
    }
    error "Synthesis did not complete successfully. Status: $synth_status"
}

# ==========================================================================
# Configure debug hub: USER chain 2, 65 MHz clock
# ==========================================================================
puts "INFO: Opening synthesized design to configure debug hub..."
open_run synth_1

# Verify STARTUPE2 is in the design
set startupe2_cells [get_cells -hier -filter {REF_NAME == STARTUPE2}]
puts "INFO: STARTUPE2 cells found: $startupe2_cells"
if {[llength $startupe2_cells] == 0} {
    error "STARTUPE2 NOT found in synthesized design! RTL may not have been updated."
}

# Check the buffered clock net
set bufg_cells [get_cells -hier -filter {REF_NAME == BUFG}]
puts "INFO: BUFG cells: $bufg_cells"
foreach bg $bufg_cells {
    set i_net [get_nets -of_objects [get_pins $bg/I]]
    set o_net [get_nets -of_objects [get_pins $bg/O]]
    puts "  BUFG $bg: I=$i_net, O=$o_net"
}

# Configure debug hub
puts "INFO: Configuring debug hub..."
set_property C_USER_SCAN_CHAIN 2 [get_debug_cores dbg_hub]
set_property C_CLK_INPUT_FREQ_HZ 65000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]

# Explicitly connect debug hub clock to our buffered clock net
connect_debug_port dbg_hub/clk [get_nets clk_buf]

# Print debug core properties for verification
puts "INFO: Debug hub properties after configuration:"
puts "  C_USER_SCAN_CHAIN  = [get_property C_USER_SCAN_CHAIN [get_debug_cores dbg_hub]]"
puts "  C_CLK_INPUT_FREQ_HZ = [get_property C_CLK_INPUT_FREQ_HZ [get_debug_cores dbg_hub]]"
# Note: get_pins doesn't accept debug_core objects directly; use get_cells pattern
set dbg_clk_pins [get_pins -quiet -hier -filter {NAME =~ dbg_hub/*clk*}]
if {[llength $dbg_clk_pins] > 0} {
    foreach p $dbg_clk_pins {
        set n [get_nets -quiet -of_objects $p]
        puts "  Pin $p -> net $n"
    }
} else {
    puts "  (Could not query dbg_hub clock pins — non-fatal)"
}

# Save constraints (save_design is deprecated in Vivado 2025.2)
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
    # Dump implementation log for debugging
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
    error "Implementation did not complete successfully. Status: $impl_status"
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

# Verify dbg_hub clock in final design
set dbg_clk_pins_final [get_pins -quiet -hier -filter {NAME =~ dbg_hub/*clk*}]
if {[llength $dbg_clk_pins_final] > 0} {
    foreach p $dbg_clk_pins_final {
        set n [get_nets -quiet -of_objects $p]
        puts "INFO: Final dbg_hub pin $p -> net $n"
    }
} else {
    puts "INFO: (Could not query dbg_hub clock pins in impl design — non-fatal)"
}

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
puts "  Debug hub: USER_SCAN_CHAIN=2, CLK=65MHz"
puts ""
puts "  Next step:"
puts "    vivado -mode batch -source scripts/read_vio_selftest.tcl"
puts "============================================================"
