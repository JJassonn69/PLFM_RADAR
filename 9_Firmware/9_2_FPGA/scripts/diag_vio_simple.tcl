# diag_vio_simple.tcl
#
# Minimal diagnostic: program VIO self-test, wait longer, try detection
# with extra diagnostics about the hw_server/device state.

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_vio"]

set bit_file [glob -nocomplain [file join $build_dir "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
set bit_file [lindex $bit_file 0]
set ltx_file [file join [file dirname $bit_file] "radar_system_top_te0713_vio.ltx"]

puts "INFO: bit=$bit_file"
puts "INFO: ltx=$ltx_file"

open_hw_manager
connect_hw_server -allow_non_jtag

# List all targets and devices
set targets [get_hw_targets]
puts "INFO: Available targets: $targets"
puts "INFO: Number of targets: [llength $targets]"
foreach t $targets {
    puts "  Target: $t"
}

open_hw_target [lindex $targets 0]

set devices [get_hw_devices]
puts "INFO: Available devices: $devices"
foreach d $devices {
    puts "  Device: $d, PART=[get_property PART $d], IR_LENGTH=[get_property IR_LENGTH $d]"
}

set fpga [lindex $devices 0]
current_hw_device $fpga

# Check what properties are set
puts ""
puts "INFO: Device properties BEFORE programming:"
puts "  PART = [get_property PART $fpga]"
catch {puts "  REGISTER.IR.BIT_COUNT = [get_property REGISTER.IR.BIT_COUNT $fpga]"}
catch {puts "  JTAG_CABLE_SERIAL = [get_property JTAG_CABLE_SERIAL $fpga]"}
catch {puts "  BSCAN_SWITCH_USER_MASK = [get_property BSCAN_SWITCH_USER_MASK $fpga]"}

# Program
set_property PROGRAM.FILE $bit_file $fpga
set_property PROBES.FILE $ltx_file $fpga
set_property FULL_PROBES.FILE $ltx_file $fpga

puts ""
puts "INFO: Programming..."
program_hw_devices $fpga
puts "INFO: Programmed OK"

# Wait LONGER - 10 seconds
puts "INFO: Waiting 10 seconds for STARTUPE2 clock + POR + self-test..."
after 10000

# Check device properties after programming
puts ""
puts "INFO: Device properties AFTER programming:"
catch {puts "  BSCAN_SWITCH_USER_MASK = [get_property BSCAN_SWITCH_USER_MASK $fpga]"}

# Try 1: refresh WITHOUT update_hw_probes first
puts ""
puts "=== ATTEMPT 1: refresh_hw_device (no update_hw_probes), mask=0100 ==="
set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
set rc [catch {refresh_hw_device $fpga} msg]
puts "  rc=$rc"
if {$rc != 0} { puts "  msg=$msg" }
set vios [get_hw_vios -quiet -of_objects $fpga]
puts "  VIOs=[llength $vios]"

# Try 2: refresh WITH update_hw_probes
puts ""
puts "=== ATTEMPT 2: refresh_hw_device -update_hw_probes true, mask=0100 ==="
set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]
puts "  rc=$rc"
if {$rc != 0} { puts "  msg=$msg" }
set vios [get_hw_vios -quiet -of_objects $fpga]
puts "  VIOs=[llength $vios]"

# Try 3: close and reopen target, then try
puts ""
puts "=== ATTEMPT 3: Close/reopen target, then refresh ==="
close_hw_target
after 2000
open_hw_target [lindex [get_hw_targets] 0]
set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga

# Don't reprogram - bitstream should still be loaded
set_property PROBES.FILE $ltx_file $fpga
set_property FULL_PROBES.FILE $ltx_file $fpga
set_property BSCAN_SWITCH_USER_MASK 0100 $fpga

set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]
puts "  rc=$rc"
if {$rc != 0} { puts "  msg=$msg" }
set vios [get_hw_vios -quiet -of_objects $fpga]
puts "  VIOs=[llength $vios]"

# Try 4: reprogram AGAIN and try immediately
puts ""
puts "=== ATTEMPT 4: Reprogram + immediate refresh ==="
set_property PROGRAM.FILE $bit_file $fpga
set_property PROBES.FILE $ltx_file $fpga
set_property FULL_PROBES.FILE $ltx_file $fpga
program_hw_devices $fpga
puts "  Programmed again. Waiting 5s..."
after 5000

set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]
puts "  rc=$rc"
if {$rc != 0} { puts "  msg=$msg" }
set vios [get_hw_vios -quiet -of_objects $fpga]
puts "  VIOs=[llength $vios]"

if {[llength $vios] > 0} {
    puts "  >>> VIO DETECTED! <<<"
    set vio [lindex $vios 0]
    set all_probes [get_hw_probes -quiet -of_objects $vio]
    puts "  Probes: [llength $all_probes]"
    foreach p $all_probes {
        set pname [get_property NAME $p]
        set val "(none)"
        catch {set val [get_property INPUT_VALUE $p]}
        catch {set val [get_property OUTPUT_VALUE $p]}
        puts "    $pname = $val"
    }
}

# Try 5: mask=1111 (all chains)
puts ""
puts "=== ATTEMPT 5: mask=1111 (ALL chains) ==="
set_property BSCAN_SWITCH_USER_MASK 1111 $fpga
set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]
puts "  rc=$rc"
set vios [get_hw_vios -quiet -of_objects $fpga]
puts "  VIOs=[llength $vios]"

puts ""
puts "=== DIAGNOSTIC COMPLETE ==="

close_hw_target
disconnect_hw_server
close_hw_manager
