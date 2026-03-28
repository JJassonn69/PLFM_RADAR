# program_v8_playback.tcl — Program FPGA with v8 unified playback+USB bitstream
open_hw_manager
connect_hw_server -url localhost:3121
open_hw_target localhost:3121/xilinx_tcf/Xilinx/TE0000993115A
set device [lindex [get_hw_devices] 0]
current_hw_device $device
set bit_file "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA/vivado_te0713_umft601x_playback/aeris10_te0713_umft601x_playback.runs/impl_1/radar_system_top_te0713_umft601x_playback.bit"
if {![file exists $bit_file]} {
    error "Bitstream not found: $bit_file"
}
set_property PROGRAM.FILE $bit_file $device
program_hw_devices $device
puts "INFO: v8 playback+USB bitstream programmed successfully"
close_hw_target
disconnect_hw_server
close_hw_manager
