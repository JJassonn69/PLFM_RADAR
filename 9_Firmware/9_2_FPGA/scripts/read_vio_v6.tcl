# read_vio_v6.tcl
#
# VIO reader - uses proven sequence: mask=0010 then mask=0100
# Reads probes IMMEDIATELY after detection (no 3s delay).

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_vio"]

set bit_file [glob -nocomplain [file join $build_dir "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
set bit_file [lindex $bit_file 0]
set ltx_file [file join [file dirname $bit_file] "radar_system_top_te0713_vio.ltx"]

# Connect & program
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target [lindex [get_hw_targets] 0]
set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga

set_property PROGRAM.FILE $bit_file $fpga
if {[file exists $ltx_file]} {
    set_property PROBES.FILE $ltx_file $fpga
    set_property FULL_PROBES.FILE $ltx_file $fpga
}
program_hw_devices $fpga
puts "INFO: Programmed OK"

# Wait for STARTUPE2 to stabilize + self-test to run
after 2000

# Prime with mask=0010
puts "INFO: Priming with mask=0010..."
set_property BSCAN_SWITCH_USER_MASK 0010 $fpga
catch {refresh_hw_device -update_hw_probes true $fpga}

# Now use mask=0100 which consistently finds the VIO
puts "INFO: Switching to mask=0100..."
set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
catch {refresh_hw_device -update_hw_probes true $fpga}

set vios [get_hw_vios -quiet -of_objects $fpga]
puts "INFO: VIOs: [llength $vios] ($vios)"

if {[llength $vios] == 0} {
    puts "ERROR: VIO not detected"
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    error "No VIO"
}

set vio [lindex $vios 0]
puts "INFO: VIO: $vio"

# Read probes IMMEDIATELY - no delay
puts "INFO: Reading probes..."

# First, try refresh_hw_vio with error handling
set refresh_ok [catch {refresh_hw_vio $vio} refresh_err]
if {$refresh_ok != 0} {
    puts "WARNING: refresh_hw_vio failed: $refresh_err"
    puts "INFO: Trying to reconnect..."
    # Reconnect
    disconnect_hw_server
    after 500
    connect_hw_server -allow_non_jtag
    open_hw_target [lindex [get_hw_targets] 0]
    set fpga [lindex [get_hw_devices] 0]
    current_hw_device $fpga
    if {[file exists $ltx_file]} {
        set_property PROBES.FILE $ltx_file $fpga
        set_property FULL_PROBES.FILE $ltx_file $fpga
    }
    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    refresh_hw_device -update_hw_probes true $fpga
    set vios [get_hw_vios -quiet -of_objects $fpga]
    if {[llength $vios] > 0} {
        set vio [lindex $vios 0]
        set refresh_ok [catch {refresh_hw_vio $vio} refresh_err2]
        puts "INFO: Retry refresh: $refresh_ok ($refresh_err2)"
    }
}

# Read all probes
set all_probes [get_hw_probes -quiet -of_objects $vio]
puts ""
puts "============================================================"
puts "  ALL PROBES"
puts "============================================================"
puts "  Count: [llength $all_probes]"
puts "  Names: $all_probes"

foreach p $all_probes {
    set pname [get_property NAME $p]
    set ptype [get_property TYPE $p]
    set pw [get_property WIDTH $p]
    set val "?"
    if {[catch {set val [get_property INPUT_VALUE $p]}]} {
        if {[catch {set val [get_property OUTPUT_VALUE $p]}]} {
            set val "(no value)"
        }
    }
    puts "  $ptype $pname \[${pw}b\]: $val"
}

puts ""
puts "============================================================"
puts "  SELF-TEST RESULTS"
puts "============================================================"
foreach {pname label} {
    result_flags_latched   "Result Flags"
    result_detail_latched  "Result Detail"
    self_test_busy         "Busy"
    all_pass_latched       "All Pass"
    hb_counter             "Heartbeat"
    test_done_latched      "Test Done"
} {
    set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pname}*"]
    if {[llength $p] > 0} {
        set val [get_property INPUT_VALUE [lindex $p 0]]
        set w [get_property WIDTH [lindex $p 0]]
        puts "  $label \[${w}b\]: $val"
    } else {
        puts "  $label: NOT FOUND"
    }
}

# Heartbeat check
puts ""
set hb_p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *hb_counter*"]
if {[llength $hb_p] > 0} {
    set hb1 [get_property INPUT_VALUE [lindex $hb_p 0]]
    after 1000
    catch {refresh_hw_vio $vio}
    set hb2 [get_property INPUT_VALUE [lindex $hb_p 0]]
    puts "  Heartbeat 1: $hb1"
    puts "  Heartbeat 2: $hb2"
    if {$hb1 eq $hb2} {
        puts "  WARNING: Not changing!"
    } else {
        puts "  OK: Design alive!"
    }
}

puts ""
puts "============================================================"
puts "  COMPLETE"
puts "============================================================"
close_hw_target
disconnect_hw_server
close_hw_manager
