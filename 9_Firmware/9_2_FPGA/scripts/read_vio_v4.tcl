# read_vio_v4.tcl
#
# VIO reader - uses the working approach: mask=0100 to prime,
# then close/reopen with mask=1111 preset.
# Reads probes by RTL signal names (not probe_inN).

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_vio"]

set bit_file [glob -nocomplain [file join $build_dir "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
set bit_file [lindex $bit_file 0]
set ltx_file [file join [file dirname $bit_file] "radar_system_top_te0713_vio.ltx"]
puts "INFO: Bitstream: $bit_file"

# Connect
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target [lindex [get_hw_targets] 0]
set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga

# Program
set_property PROGRAM.FILE $bit_file $fpga
if {[file exists $ltx_file]} {
    set_property PROBES.FILE $ltx_file $fpga
    set_property FULL_PROBES.FILE $ltx_file $fpga
}
program_hw_devices $fpga
puts "INFO: Programmed (End of startup status: HIGH)"

# Prime with mask=0100 (this somehow initializes the BSCAN state)
set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
catch {refresh_hw_device -update_hw_probes true $fpga}

# Close and reopen with mask=1111
close_hw_target
after 1000
open_hw_target [lindex [get_hw_targets] 0]
set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga
if {[file exists $ltx_file]} {
    set_property PROBES.FILE $ltx_file $fpga
    set_property FULL_PROBES.FILE $ltx_file $fpga
}
set_property BSCAN_SWITCH_USER_MASK 1111 $fpga
refresh_hw_device -update_hw_probes true $fpga

set vio [lindex [get_hw_vios -of_objects $fpga] 0]
puts "INFO: VIO core: $vio"

# Wait for power-on self-test to complete
puts "INFO: Waiting 3s for self-test..."
after 3000

# Read probes
refresh_hw_vio $vio

puts ""
puts "============================================================"
puts "  AERIS-10 Self-Test Results (via JTAG VIO)"
puts "============================================================"

# Read by RTL signal name — these are the actual probe names
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
        puts "  $label ($pname) \[${w}b\]: $val"
    } else {
        puts "  $label ($pname): NOT FOUND"
    }
}

# Also try to read version (probe_in5) if it exists under a different name
set all_probes [get_hw_probes -quiet -of_objects $vio]
puts ""
puts "  All probe names: $all_probes"

# Heartbeat check (read twice)
puts ""
puts "INFO: Heartbeat check (two reads 1s apart)..."
set hb_p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *hb_counter*"]
if {[llength $hb_p] > 0} {
    set hb [lindex $hb_p 0]
    set hb1 [get_property INPUT_VALUE $hb]
    after 1000
    refresh_hw_vio $vio
    set hb2 [get_property INPUT_VALUE $hb]
    puts "  Heartbeat read 1: $hb1"
    puts "  Heartbeat read 2: $hb2"
    if {$hb1 eq $hb2} {
        puts "  WARNING: Heartbeat NOT changing — clock may not be running!"
    } else {
        puts "  OK: Heartbeat is incrementing — FPGA design is alive!"
    }
}

# Trigger a new self-test via VIO
puts ""
puts "INFO: Triggering self-test via vio_trigger_test..."
set trig_p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *vio_trigger_test*"]
if {[llength $trig_p] > 0} {
    set trig [lindex $trig_p 0]
    set_property OUTPUT_VALUE 1 $trig
    commit_hw_vio $vio
    after 100
    set_property OUTPUT_VALUE 0 $trig
    commit_hw_vio $vio
    after 2000

    refresh_hw_vio $vio

    puts ""
    puts "============================================================"
    puts "  Re-triggered Self-Test Results"
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
            puts "  $label: $val"
        }
    }
}

puts ""
puts "============================================================"
puts "  DONE"
puts "============================================================"

close_hw_target
disconnect_hw_server
close_hw_manager
puts "INFO: Connection closed."
