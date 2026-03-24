# build_te0713_vio.tcl
#
# Vivado batch build for TE0713/TE0701 VIO debug target.
# Generates the VIO IP core in-project, then builds to bitstream.
#
# Usage:
#   vivado -mode batch -source scripts/build_te0713_vio.tcl

set script_dir [file dirname [file normalize [info script]]]
set project_root [file normalize [file join $script_dir ".."]]

set project_name "aeris10_te0713_vio"
set build_dir [file join $project_root "vivado_te0713_vio"]
set reports_dir [file join $build_dir "reports"]

set top_file [file join $project_root "radar_system_top_te0713_vio.v"]
set self_test_file [file join $project_root "fpga_self_test.v"]
set xdc_file [file join $project_root "constraints" "te0713_vio.xdc"]

file mkdir $build_dir
file mkdir $reports_dir
file mkdir [file join $build_dir "ip"]

# Create project
create_project -force $project_name $build_dir -part xc7a200tfbg484-2
set_property target_language Verilog [current_project]

# Add RTL sources
add_files -norecurse $top_file
add_files -norecurse $self_test_file
add_files -fileset constrs_1 -norecurse $xdc_file

# ==========================================================================
# Generate VIO IP core
# ==========================================================================
# vio_0: 7 input probes, 2 output probes
# Input probes read FPGA state; output probes drive FPGA from Vivado GUI
# ==========================================================================
puts "INFO: Creating VIO IP core..."

create_ip -name vio -vendor xilinx.com -library ip -version 3.0 \
    -module_name vio_0 -dir [file join $build_dir "ip"]

set_property -dict [list \
    CONFIG.C_PROBE_IN0_WIDTH  {5}  \
    CONFIG.C_PROBE_IN1_WIDTH  {8}  \
    CONFIG.C_PROBE_IN2_WIDTH  {1}  \
    CONFIG.C_PROBE_IN3_WIDTH  {1}  \
    CONFIG.C_PROBE_IN4_WIDTH  {32} \
    CONFIG.C_PROBE_IN5_WIDTH  {8}  \
    CONFIG.C_PROBE_IN6_WIDTH  {1}  \
    CONFIG.C_PROBE_OUT0_WIDTH {1}  \
    CONFIG.C_PROBE_OUT1_WIDTH {1}  \
    CONFIG.C_NUM_PROBE_IN     {7}  \
    CONFIG.C_NUM_PROBE_OUT    {2}  \
    CONFIG.C_PROBE_OUT0_INIT_VAL {0x0} \
    CONFIG.C_PROBE_OUT1_INIT_VAL {0x0} \
    CONFIG.C_EN_PROBE_IN_ACTIVITY {0}  \
] [get_ips vio_0]

generate_target all [get_ips vio_0]

puts "INFO: VIO IP core created successfully."

# Set top module
set_property top radar_system_top_te0713_vio [current_fileset]
update_compile_order -fileset sources_1

# ==========================================================================
# Build: synthesis → implementation → bitstream
# ==========================================================================
puts "INFO: Launching implementation to bitstream..."
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set impl_status [get_property STATUS [get_runs impl_1]]
puts "INFO: impl_1 status: $impl_status"

if {![string match "*Complete*" $impl_status]} {
    error "Implementation did not complete successfully. Status: $impl_status"
}

# ==========================================================================
# Reports
# ==========================================================================
open_run impl_1

report_clocks -file [file join $reports_dir "clocks.rpt"]
report_clock_interaction -file [file join $reports_dir "clock_interaction.rpt"]
report_timing_summary -report_unconstrained -max_paths 100 \
    -file [file join $reports_dir "timing_summary.rpt"]
report_cdc -details -file [file join $reports_dir "cdc.rpt"]
report_exceptions -file [file join $reports_dir "exceptions.rpt"]
report_drc -file [file join $reports_dir "drc.rpt"]
report_utilization -file [file join $reports_dir "utilization.rpt"]

set bit_file [get_property BITSTREAM.FILE [current_design]]

puts "INFO: Build complete."
puts "INFO: Bitstream: $bit_file"
puts "INFO: Reports:   $reports_dir"
puts ""
puts "INFO: To program and read self-test results, use:"
puts "INFO:   vivado -mode batch -source scripts/read_vio_selftest.tcl"
