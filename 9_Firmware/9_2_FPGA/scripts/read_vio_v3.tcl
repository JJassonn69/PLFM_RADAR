# read_vio_v3.tcl
#
# VIO reader with retry logic and state-priming approach.

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_vio"]

set bit_file [glob -nocomplain [file join $build_dir "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
set bit_file [lindex $bit_file 0]
set ltx_file [file join [file dirname $bit_file] "radar_system_top_te0713_vio.ltx"]
puts "INFO: Bitstream: $bit_file"
puts "INFO: Probes: $ltx_file"

# Connect to hardware
open_hw_manager
connect_hw_server -allow_non_jtag

open_hw_target [lindex [get_hw_targets] 0]
set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga
puts "INFO: Device: $fpga"

# Program
set_property PROGRAM.FILE $bit_file $fpga
if {[file exists $ltx_file]} {
    set_property PROBES.FILE $ltx_file $fpga
    set_property FULL_PROBES.FILE $ltx_file $fpga
}
program_hw_devices $fpga
puts "INFO: Programmed OK"

# =========================================================================
# Strategy: Try multiple approaches to detect the debug hub
# The previous sweep showed VIO detected on mask=1111 but only AFTER
# attempting mask=0100 (which caused an error that may have primed state)
# =========================================================================

puts ""
puts "INFO: === Approach 1: Try mask=0010 (USER2 direct) ==="
set_property BSCAN_SWITCH_USER_MASK 0010 $fpga
set result [catch {refresh_hw_device -update_hw_probes true $fpga} errmsg]
puts "INFO: Result: $result ($errmsg)"
set vios [get_hw_vios -quiet -of_objects $fpga]
puts "INFO: VIOs: $vios"

if {[llength $vios] == 0} {
    puts ""
    puts "INFO: === Approach 2: Try mask=0100 to prime state ==="
    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga} errmsg2
    puts "INFO: 0100 result: $errmsg2"

    puts ""
    puts "INFO: === Approach 3: Now try mask=1111 ==="
    set_property BSCAN_SWITCH_USER_MASK 1111 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga} errmsg3
    puts "INFO: 1111 result: $errmsg3"
    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "INFO: VIOs: $vios"
}

if {[llength $vios] == 0} {
    puts ""
    puts "INFO: === Approach 4: Close and reopen target with mask preset ==="
    close_hw_target
    after 1000

    # Reopen
    open_hw_target [lindex [get_hw_targets] 0]
    set fpga [lindex [get_hw_devices] 0]
    current_hw_device $fpga

    # Don't reprogram — bitstream is still loaded
    if {[file exists $ltx_file]} {
        set_property PROBES.FILE $ltx_file $fpga
        set_property FULL_PROBES.FILE $ltx_file $fpga
    }

    # Set mask BEFORE refresh
    set_property BSCAN_SWITCH_USER_MASK 1111 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga} errmsg4
    puts "INFO: Reopen result: $errmsg4"
    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "INFO: VIOs: $vios"
}

if {[llength $vios] == 0} {
    puts ""
    puts "INFO: === Approach 5: Full sweep (reproducing what worked before) ==="
    foreach mask {0001 0010 0100 1000 0011 0101 0110 1111} {
        puts "  Trying mask=$mask..."
        set_property BSCAN_SWITCH_USER_MASK $mask $fpga
        catch {refresh_hw_device -update_hw_probes true $fpga} sweep_err
        set vios [get_hw_vios -quiet -of_objects $fpga]
        if {[llength $vios] > 0} {
            puts "  >>> FOUND VIO with mask=$mask <<<"
            break
        }
    }
}

# Final check
set vios [get_hw_vios -quiet -of_objects $fpga]
puts ""
puts "INFO: Final VIO cores: $vios"

if {[llength $vios] == 0} {
    puts "ERROR: VIO detection failed with all approaches"
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    error "No VIO cores detected"
}

set vio [lindex $vios 0]
puts "INFO: Using VIO: $vio"

# Wait for self-test
after 3000

# List all probes
puts ""
puts "============================================================"
puts "  ALL PROBES"
puts "============================================================"
set all_probes [get_hw_probes -quiet -of_objects $vio]
puts "  Probe list: $all_probes"
puts ""

foreach probe $all_probes {
    set pname [get_property NAME $probe]
    set ptype [get_property TYPE $probe]
    set pwidth [get_property WIDTH $probe]
    if {$ptype eq "INPUT" || $ptype eq "in"} {
        set val [get_property INPUT_VALUE $probe]
        puts "  IN  $pname \[width=$pwidth\]: $val"
    } else {
        set val [get_property OUTPUT_VALUE $probe]
        puts "  OUT $pname \[width=$pwidth\]: $val"
    }
}

# Heartbeat check
puts ""
puts "INFO: Heartbeat check..."
set hb_probes [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *probe_in4*"]
if {[llength $hb_probes] > 0} {
    set hb [lindex $hb_probes 0]
    refresh_hw_vio $vio
    set hb1 [get_property INPUT_VALUE $hb]
    after 1000
    refresh_hw_vio $vio
    set hb2 [get_property INPUT_VALUE $hb]
    puts "  Heartbeat 1: $hb1"
    puts "  Heartbeat 2: $hb2"
    if {$hb1 eq $hb2} {
        puts "  WARNING: Not changing!"
    } else {
        puts "  OK: Design is alive!"
    }
}

# Trigger self-test
puts ""
puts "INFO: Triggering self-test..."
set trig [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *probe_out0*"]
if {[llength $trig] > 0} {
    set t [lindex $trig 0]
    set_property OUTPUT_VALUE 1 $t
    commit_hw_vio $vio
    after 100
    set_property OUTPUT_VALUE 0 $t
    commit_hw_vio $vio
    after 2000
    refresh_hw_vio $vio

    puts ""
    puts "============================================================"
    puts "  SELF-TEST RESULTS"
    puts "============================================================"
    foreach probe $all_probes {
        set pname [get_property NAME $probe]
        set ptype [get_property TYPE $probe]
        if {$ptype eq "INPUT" || $ptype eq "in"} {
            set val [get_property INPUT_VALUE $probe]
            puts "  $pname = $val"
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
