# read_vio_v2.tcl
#
# Simplified VIO reader: programs FPGA, detects VIO with mask=1111,
# lists all probes, and reads their values.

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_vio"]

# Find bitstream
set bit_file [glob -nocomplain [file join $build_dir "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
if {[llength $bit_file] == 0} {
    error "No bitstream found"
}
set bit_file [lindex $bit_file 0]
puts "INFO: Bitstream: $bit_file"

# Find probes file
set ltx_file [file join [file dirname $bit_file] "radar_system_top_te0713_vio.ltx"]
if {![file exists $ltx_file]} {
    set ltx_file [file join [file dirname $bit_file] "debug_nets.ltx"]
}
puts "INFO: Probes: $ltx_file"

# Kill stale servers
catch {exec pkill -9 -f hw_server}
catch {exec pkill -9 -f cs_server}
after 1000

# Connect
open_hw_manager
connect_hw_server -allow_non_jtag

set targets [get_hw_targets]
puts "INFO: Targets: $targets"
open_hw_target [lindex $targets 0]

set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga
puts "INFO: Device: $fpga"

# Program
set_property PROGRAM.FILE $bit_file $fpga
if {[file exists $ltx_file]} {
    set_property PROBES.FILE $ltx_file $fpga
    set_property FULL_PROBES.FILE $ltx_file $fpga
    puts "INFO: Loaded probes file"
} else {
    set_property PROBES.FILE {} $fpga
    set_property FULL_PROBES.FILE {} $fpga
    puts "WARNING: No probes file"
}

program_hw_devices $fpga
puts "INFO: Programming complete (End of startup status: HIGH)"

# Set mask to 1111 (scan all USER chains) — this is what worked
puts "INFO: Setting BSCAN_SWITCH_USER_MASK = 1111"
set_property BSCAN_SWITCH_USER_MASK 1111 $fpga

# Refresh to detect debug hub
puts "INFO: Refreshing device..."
refresh_hw_device -update_hw_probes true $fpga

# Check for VIO
set vio_cores [get_hw_vios -of_objects $fpga]
puts "INFO: VIO cores: $vio_cores"

if {[llength $vio_cores] == 0} {
    puts "ERROR: No VIO cores found even with mask=1111"
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    error "No VIO cores"
}

set vio [lindex $vio_cores 0]
puts "INFO: Using VIO: $vio"

# Wait for self-test
puts "INFO: Waiting 3s for self-test to complete..."
after 3000

# List ALL probes
puts ""
puts "============================================================"
puts "  LISTING ALL PROBES"
puts "============================================================"

set all_probes [get_hw_probes -of_objects $vio]
puts "INFO: All probes: $all_probes"
puts ""

# Read each probe
foreach probe $all_probes {
    set dir [get_property TYPE $probe]
    set width [get_property WIDTH $probe]
    if {$dir eq "INPUT" || $dir eq "in"} {
        set val [get_property INPUT_VALUE $probe]
        puts "  IN  $probe (width=$width): $val"
    } elseif {$dir eq "OUTPUT" || $dir eq "out"} {
        set val [get_property OUTPUT_VALUE $probe]
        puts "  OUT $probe (width=$width): $val"
    } else {
        # Try both
        catch {
            set val [get_property INPUT_VALUE $probe]
            puts "  ???($dir) $probe (width=$width): INPUT=$val"
        }
        catch {
            set val [get_property OUTPUT_VALUE $probe]
            puts "  ???($dir) $probe (width=$width): OUTPUT=$val"
        }
    }
}

puts ""
puts "============================================================"
puts "  PROBE VALUES (first read)"
puts "============================================================"

# Try to read probes by various naming patterns
foreach pattern {probe_in0 probe_in1 probe_in2 probe_in3 probe_in4 probe_in5 probe_in6 probe_out0 probe_out1} {
    set matches [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *$pattern*"]
    if {[llength $matches] > 0} {
        set p [lindex $matches 0]
        set dir [get_property TYPE $p]
        if {$dir eq "INPUT" || $dir eq "in"} {
            puts "  $pattern = [get_property INPUT_VALUE $p]"
        } else {
            puts "  $pattern = [get_property OUTPUT_VALUE $p]"
        }
    } else {
        puts "  $pattern = (not found)"
    }
}

# Read heartbeat twice to verify design is running
puts ""
puts "INFO: Reading heartbeat twice (1s apart)..."
set hb_probes [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *probe_in4*"]
if {[llength $hb_probes] > 0} {
    set hb [lindex $hb_probes 0]
    refresh_hw_vio $vio
    set hb1 [get_property INPUT_VALUE $hb]
    after 1000
    refresh_hw_vio $vio
    set hb2 [get_property INPUT_VALUE $hb]
    puts "  Heartbeat read 1: $hb1"
    puts "  Heartbeat read 2: $hb2"
    if {$hb1 eq $hb2} {
        puts "  WARNING: Heartbeat NOT changing — design may not be running!"
    } else {
        puts "  OK: Heartbeat is incrementing — design is alive!"
    }
} else {
    puts "  (heartbeat probe not found by name)"
}

# Trigger self-test via probe_out0
puts ""
puts "INFO: Triggering self-test..."
set trig_probes [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *probe_out0*"]
if {[llength $trig_probes] > 0} {
    set trig [lindex $trig_probes 0]
    set_property OUTPUT_VALUE 1 $trig
    commit_hw_vio $vio
    after 100
    set_property OUTPUT_VALUE 0 $trig
    commit_hw_vio $vio
    after 2000
    
    refresh_hw_vio $vio
    puts ""
    puts "============================================================"
    puts "  SELF-TEST RESULTS (after trigger)"
    puts "============================================================"
    foreach pattern {probe_in0 probe_in1 probe_in2 probe_in3 probe_in4 probe_in5 probe_in6} {
        set matches [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *$pattern*"]
        if {[llength $matches] > 0} {
            set p [lindex $matches 0]
            puts "  $pattern = [get_property INPUT_VALUE $p]"
        }
    }
} else {
    puts "  (trigger probe not found by name)"
}

puts ""
puts "============================================================"
puts "  DONE"
puts "============================================================"

# Cleanup
close_hw_target
disconnect_hw_server
close_hw_manager
puts "INFO: Connection closed."
