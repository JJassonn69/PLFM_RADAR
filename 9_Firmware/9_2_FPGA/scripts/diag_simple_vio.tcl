# diag_simple_vio.tcl
#
# Minimal VIO detection diagnostic with extended stabilization.
# Programs the known-working VIO self-test bitstream and tries
# debug hub detection with increasing delays.

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_vio"]
set bit_file [glob -nocomplain [file join $build_dir "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
set bit_file [lindex $bit_file 0]
set ltx_file [file join [file dirname $bit_file] "radar_system_top_te0713_vio.ltx"]

puts "INFO: bit = $bit_file"
puts "INFO: ltx = $ltx_file"

open_hw_manager
connect_hw_server -allow_non_jtag

puts ""
puts "=== HW Targets ==="
set targets [get_hw_targets]
puts "  Targets: $targets"
foreach t $targets {
    puts "  Target: $t"
}

open_hw_target [lindex $targets 0]
set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga

puts ""
puts "=== Device Properties ==="
puts "  Device: [get_property NAME $fpga]"
puts "  Part:   [get_property PART $fpga]"

# Load probes BEFORE programming
set_property PROGRAM.FILE $bit_file $fpga
set_property PROBES.FILE $ltx_file $fpga
set_property FULL_PROBES.FILE $ltx_file $fpga

puts ""
puts "=== PROGRAMMING ==="
program_hw_devices $fpga
puts "  Programmed OK"

# Try debug hub detection with increasing delays
foreach delay {1000 3000 5000 8000 10000} {
    puts ""
    puts "=== ATTEMPT: wait ${delay}ms then mask=0100 ==="
    after $delay
    
    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]
    
    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "  rc=$rc VIOs=[llength $vios]"
    
    if {[llength $vios] > 0} {
        puts "  >>> VIO DETECTED! <<<"
        set vio [lindex $vios 0]
        set probes [get_hw_probes -quiet -of_objects $vio]
        puts "  Probes: [llength $probes]"
        
        foreach p $probes {
            set pname [get_property NAME $p]
            set ptype [get_property TYPE $p]
            set val "(none)"
            if {[catch {set val [get_property INPUT_VALUE $p]}]} {
                catch {set val [get_property OUTPUT_VALUE $p]}
            }
            puts "  $ptype $pname = '$val'"
        }
        
        puts ""
        puts ">>> SUCCESS at delay=${delay}ms <<<"
        close_hw_target
        disconnect_hw_server
        close_hw_manager
        exit 0
    }
}

# If we get here, try reprogramming and then immediate detection
puts ""
puts "=== ATTEMPT: Reprogram + immediate mask=0100 ==="
program_hw_devices $fpga
after 500
set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]
set vios [get_hw_vios -quiet -of_objects $fpga]
puts "  rc=$rc VIOs=[llength $vios]"

if {[llength $vios] > 0} {
    puts "  >>> VIO DETECTED on reprogram! <<<"
} else {
    puts ""
    puts "=== ATTEMPT: Set mask BEFORE programming ==="
    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    program_hw_devices $fpga
    after 3000
    set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]
    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "  rc=$rc VIOs=[llength $vios]"
    
    if {[llength $vios] > 0} {
        puts "  >>> VIO DETECTED with pre-set mask! <<<"
    } else {
        puts ""
        puts ">>> ALL ATTEMPTS FAILED <<<"
        puts ">>> Debug hub is not responding. Possible causes:"
        puts ">>>   1. STARTUPE2 CFGMCLK not running (power issue?)"
        puts ">>>   2. Board needs physical power cycle (12V off/on)"
        puts ">>>   3. USB disconnect/reconnect needed"
        puts ">>>   4. Corrupt hw_server state"
    }
}

close_hw_target
disconnect_hw_server
close_hw_manager
