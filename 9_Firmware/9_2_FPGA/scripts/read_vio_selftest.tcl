# read_vio_selftest.tcl
#
# Programs the VIO bitstream onto the TE0713, then reads back self-test
# results via the VIO debug core over JTAG.
#
# Usage:
#   vivado -mode batch -source scripts/read_vio_selftest.tcl
#
# Prerequisites:
#   - hw_server running (or will be started by open_hw_manager)
#   - FTDI Channel A (JTAG) unbound from ftdi_sio kernel driver
#   - Bitstream built by build_te0713_vio.tcl

set script_dir [file dirname [file normalize [info script]]]
set project_root [file normalize [file join $script_dir ".."]]
set build_dir [file join $project_root "vivado_te0713_vio"]

# Find bitstream
set bit_file [file join $build_dir \
    "${::env(USER)}_te0713_vio" "aeris10_te0713_vio.runs" "impl_1" \
    "radar_system_top_te0713_vio.bit"]

# Fallback: search for it
if {![file exists $bit_file]} {
    set bit_file [glob -nocomplain [file join $build_dir \
        "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
    if {[llength $bit_file] == 0} {
        # Try home directory copy
        set bit_file "$::env(HOME)/te0713-vio.bit"
        if {![file exists $bit_file]} {
            error "Cannot find VIO bitstream. Run build_te0713_vio.tcl first."
        }
    } else {
        set bit_file [lindex $bit_file 0]
    }
}

puts "INFO: Using bitstream: $bit_file"

# ==========================================================================
# Connect to hardware
# ==========================================================================
open_hw_manager
connect_hw_server -allow_non_jtag

# Find the target
set targets [get_hw_targets]
puts "INFO: Available targets: $targets"

if {[llength $targets] == 0} {
    error "No JTAG targets found. Check USB connection and ftdi_sio unbind."
}

# Open first target (should be Trenz TE0701)
open_hw_target [lindex $targets 0]

set devices [get_hw_devices]
puts "INFO: Available devices: $devices"

set fpga [lindex $devices 0]
current_hw_device $fpga

# ==========================================================================
# Program bitstream
# ==========================================================================
puts "INFO: Programming $fpga with VIO bitstream..."
set_property PROGRAM.FILE $bit_file $fpga
set_property PROBES.FILE {} $fpga
set_property FULL_PROBES.FILE {} $fpga
program_hw_devices $fpga
refresh_hw_device $fpga

puts "INFO: Programming complete. Waiting for self-test to run..."
after 2000   ;# Wait 2 seconds for POR + self-test to complete

# ==========================================================================
# Read VIO probes
# ==========================================================================
puts "INFO: Reading VIO probes..."

# Find the VIO core
set vio_cores [get_hw_vios -of_objects $fpga]
puts "INFO: VIO cores found: $vio_cores"

if {[llength $vio_cores] == 0} {
    puts "ERROR: No VIO cores detected. Check bitstream has VIO IP."
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    error "No VIO cores found"
}

set vio [lindex $vio_cores 0]

# Refresh to get current probe values
refresh_hw_vio $vio

# Read all input probes
set result_flags  [get_property INPUT_VALUE [get_hw_probes probe_in0 -of_objects $vio]]
set result_detail [get_property INPUT_VALUE [get_hw_probes probe_in1 -of_objects $vio]]
set busy          [get_property INPUT_VALUE [get_hw_probes probe_in2 -of_objects $vio]]
set all_pass      [get_property INPUT_VALUE [get_hw_probes probe_in3 -of_objects $vio]]
set heartbeat     [get_property INPUT_VALUE [get_hw_probes probe_in4 -of_objects $vio]]
set version       [get_property INPUT_VALUE [get_hw_probes probe_in5 -of_objects $vio]]
set test_done     [get_property INPUT_VALUE [get_hw_probes probe_in6 -of_objects $vio]]

# ==========================================================================
# Decode and display results
# ==========================================================================
puts ""
puts "============================================================"
puts "  AERIS-10 Self-Test Results (via JTAG VIO)"
puts "============================================================"
puts "  Heartbeat:     $heartbeat (design is alive if changing)"
puts "  Version:       $version"
puts "  Test Done:     $test_done"
puts "  Busy:          $busy"
puts "  All Pass:      $all_pass"
puts "  Result Flags:  $result_flags"
puts "  Result Detail: $result_detail"
puts ""

# Decode result_flags bit by bit
# Flags are 5 bits: [4]=ADC, [3]=ARITH, [2]=FFT, [1]=CIC, [0]=BRAM
set flag_names [list "BRAM" "CIC" "FFT" "ARITH" "ADC"]
puts "  Per-subsystem results:"
for {set i 0} {$i < 5} {incr i} {
    set name [lindex $flag_names $i]
    # result_flags comes back as binary string from VIO
    puts "    Test $i ($name): (check bit $i of result_flags above)"
}

puts ""
puts "============================================================"
puts ""

# ==========================================================================
# Read again after a short delay to confirm heartbeat is changing
# ==========================================================================
after 1000
refresh_hw_vio $vio
set heartbeat2 [get_property INPUT_VALUE [get_hw_probes probe_in4 -of_objects $vio]]
puts "  Heartbeat (1s later): $heartbeat2"

if {$heartbeat eq $heartbeat2} {
    puts "  WARNING: Heartbeat not changing — design may not be running!"
} else {
    puts "  OK: Heartbeat is incrementing — design is alive."
}

# ==========================================================================
# Optionally trigger a new self-test via VIO
# ==========================================================================
puts ""
puts "INFO: Triggering self-test via VIO probe_out0..."

# Set trigger high
set_property OUTPUT_VALUE 1 [get_hw_probes probe_out0 -of_objects $vio]
commit_hw_vio $vio
after 100

# Set trigger low (edge detection in FPGA will generate pulse)
set_property OUTPUT_VALUE 0 [get_hw_probes probe_out0 -of_objects $vio]
commit_hw_vio $vio

# Wait for test to complete
after 2000
refresh_hw_vio $vio

set result_flags2  [get_property INPUT_VALUE [get_hw_probes probe_in0 -of_objects $vio]]
set result_detail2 [get_property INPUT_VALUE [get_hw_probes probe_in1 -of_objects $vio]]
set busy2          [get_property INPUT_VALUE [get_hw_probes probe_in2 -of_objects $vio]]
set all_pass2      [get_property INPUT_VALUE [get_hw_probes probe_in3 -of_objects $vio]]
set test_done2     [get_property INPUT_VALUE [get_hw_probes probe_in6 -of_objects $vio]]

puts ""
puts "============================================================"
puts "  Re-triggered Self-Test Results"
puts "============================================================"
puts "  Test Done:     $test_done2"
puts "  Busy:          $busy2"
puts "  All Pass:      $all_pass2"
puts "  Result Flags:  $result_flags2"
puts "  Result Detail: $result_detail2"
puts "============================================================"

# Cleanup
close_hw_target
disconnect_hw_server
close_hw_manager

puts "INFO: Done. JTAG connection closed."
