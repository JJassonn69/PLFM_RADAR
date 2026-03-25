# rebuild_bscan2.tcl
#
# Rebuilds the VIO bitstream with C_USER_SCAN_CHAIN=2
# to avoid collision with the TE0713 onboard CPLD which uses USER1.
#
# Usage:
#   vivado -mode batch -source scripts/rebuild_bscan2.tcl

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_vio"]

open_project [file join $build_dir "aeris10_te0713_vio.xpr"]

# Open synthesized design
open_run synth_1

puts "INFO: Current C_USER_SCAN_CHAIN = [get_property C_USER_SCAN_CHAIN [get_debug_cores dbg_hub]]"
puts "INFO: Changing to USER_SCAN_CHAIN=2..."

set_property C_USER_SCAN_CHAIN 2 [get_debug_cores dbg_hub]
set_property C_CLK_INPUT_FREQ_HZ 65000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_buf]

# Save the design first (required before implement_debug_core)
save_design

implement_debug_core

puts "INFO: New C_USER_SCAN_CHAIN = [get_property C_USER_SCAN_CHAIN [get_debug_cores dbg_hub]]"
close_design

puts "INFO: Resetting and relaunching implementation..."
reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set impl_status [get_property STATUS [get_runs impl_1]]
puts "INFO: impl_1 status: $impl_status"

if {![string match "*Complete*" $impl_status]} {
    error "Implementation failed: $impl_status"
}

puts "INFO: Bitstream built with USER_SCAN_CHAIN=2"
