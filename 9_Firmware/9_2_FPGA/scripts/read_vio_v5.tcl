# read_vio_v5.tcl
#
# VIO reader - reproduces exact sequence from v3 that detected VIO:
# 1. Program
# 2. mask=0010, refresh (fails — but primes USER2)
# 3. mask=0100, refresh (detected VIO in v3!)
# 4. If still no VIO, close/reopen with mask=0100
# Then reads probes.

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
puts "INFO: Programmed OK"

# Wait after programming for STARTUPE2 to stabilize
after 3000
puts "INFO: Waited 3s for clock stabilization"

# Step 1: mask=0010 (USER2 — our chain)
puts "INFO: Step 1: mask=0010"
set_property BSCAN_SWITCH_USER_MASK 0010 $fpga
catch {refresh_hw_device -update_hw_probes true $fpga} e1
set v1 [get_hw_vios -quiet -of_objects $fpga]
puts "INFO: VIOs after 0010: [llength $v1] ($v1)"

# Step 2: mask=0100 (USER3 — should prime something)
if {[llength $v1] == 0} {
    puts "INFO: Step 2: mask=0100"
    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga} e2
    set v2 [get_hw_vios -quiet -of_objects $fpga]
    puts "INFO: VIOs after 0100: [llength $v2] ($v2)"
}

set vios [get_hw_vios -quiet -of_objects $fpga]

# Step 3: close/reopen with mask=0100 (this is what worked in v3)
if {[llength $vios] == 0} {
    puts "INFO: Step 3: Close/reopen with mask=0100"
    close_hw_target
    after 2000
    open_hw_target [lindex [get_hw_targets] 0]
    set fpga [lindex [get_hw_devices] 0]
    current_hw_device $fpga
    if {[file exists $ltx_file]} {
        set_property PROBES.FILE $ltx_file $fpga
        set_property FULL_PROBES.FILE $ltx_file $fpga
    }
    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga} e3
    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "INFO: VIOs after reopen 0100: [llength $vios] ($vios)"
}

# Step 4: close/reopen with mask=1111
if {[llength $vios] == 0} {
    puts "INFO: Step 4: Close/reopen with mask=1111"
    close_hw_target
    after 2000
    open_hw_target [lindex [get_hw_targets] 0]
    set fpga [lindex [get_hw_devices] 0]
    current_hw_device $fpga
    if {[file exists $ltx_file]} {
        set_property PROBES.FILE $ltx_file $fpga
        set_property FULL_PROBES.FILE $ltx_file $fpga
    }
    set_property BSCAN_SWITCH_USER_MASK 1111 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga} e4
    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "INFO: VIOs after reopen 1111: [llength $vios] ($vios)"
}

# Step 5: Full sweep with close/reopen for each
if {[llength $vios] == 0} {
    puts "INFO: Step 5: Full sweep with close/reopen"
    foreach mask {0001 0010 0011 0100 0101 0110 0111 1000 1001 1010 1011 1100 1101 1110 1111} {
        close_hw_target
        after 500
        open_hw_target [lindex [get_hw_targets] 0]
        set fpga [lindex [get_hw_devices] 0]
        current_hw_device $fpga
        if {[file exists $ltx_file]} {
            set_property PROBES.FILE $ltx_file $fpga
            set_property FULL_PROBES.FILE $ltx_file $fpga
        }
        set_property BSCAN_SWITCH_USER_MASK $mask $fpga
        catch {refresh_hw_device -update_hw_probes true $fpga} sweep_err
        set vios [get_hw_vios -quiet -of_objects $fpga]
        puts "  mask=$mask: VIOs=[llength $vios]"
        if {[llength $vios] > 0} {
            puts "  >>> FOUND with mask=$mask <<<"
            break
        }
    }
}

# Final check
set vios [get_hw_vios -quiet -of_objects $fpga]
puts ""
puts "INFO: Final VIO count: [llength $vios]"

if {[llength $vios] == 0} {
    puts "ERROR: VIO not detected with any approach"
    puts ""
    puts "DIAGNOSTIC: Trying without probes file..."
    close_hw_target
    after 1000
    open_hw_target [lindex [get_hw_targets] 0]
    set fpga [lindex [get_hw_devices] 0]
    current_hw_device $fpga
    # Clear probes
    set_property PROBES.FILE {} $fpga
    set_property FULL_PROBES.FILE {} $fpga
    set_property BSCAN_SWITCH_USER_MASK 1111 $fpga
    catch {refresh_hw_device $fpga} e_noprobe
    set vios_noprobe [get_hw_vios -quiet -of_objects $fpga]
    puts "  VIOs without probes file: [llength $vios_noprobe] ($vios_noprobe)"
    
    # Also read raw JTAG data
    puts ""
    puts "DIAGNOSTIC: JTAG device info:"
    puts "  REGISTER.USERCODE = [get_property REGISTER.USERCODE $fpga]"
    puts "  REGISTER.IDCODE = [get_property REGISTER.IDCODE $fpga]"

    close_hw_target
    disconnect_hw_server
    close_hw_manager
    error "VIO detection failed"
}

set vio [lindex $vios 0]
puts "INFO: VIO: $vio"

# Wait for self-test
after 3000
refresh_hw_vio $vio

# List all probes first
set all_probes [get_hw_probes -quiet -of_objects $vio]
puts ""
puts "============================================================"
puts "  ALL PROBES on $vio"
puts "============================================================"
puts "  Count: [llength $all_probes]"
foreach p $all_probes {
    set pname [get_property NAME $p]
    set ptype [get_property TYPE $p]
    set pw [get_property WIDTH $p]
    # Use catch for reading values since const probes may not have INPUT/OUTPUT_VALUE
    set val "?"
    if {[catch {set val [get_property INPUT_VALUE $p]}]} {
        catch {set val [get_property OUTPUT_VALUE $p]}
    }
    puts "  $ptype $pname \[${pw}b\]: $val"
}

# Read named probes
puts ""
puts "============================================================"
puts "  AERIS-10 Self-Test Results"
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
        puts "  $label ($pname) \[${w}b\]: $val"
    } else {
        puts "  $label ($pname): NOT FOUND"
    }
}

# Heartbeat check
puts ""
set hb_p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *hb_counter*"]
if {[llength $hb_p] > 0} {
    set hb1 [get_property INPUT_VALUE [lindex $hb_p 0]]
    after 1000
    refresh_hw_vio $vio
    set hb2 [get_property INPUT_VALUE [lindex $hb_p 0]]
    puts "  Heartbeat read 1: $hb1"
    puts "  Heartbeat read 2: $hb2"
    if {$hb1 eq $hb2} {
        puts "  WARNING: Not changing!"
    } else {
        puts "  OK: Design is alive!"
    }
}

# Trigger self-test
puts ""
puts "INFO: Triggering self-test..."
set trig [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *vio_trigger_test*"]
if {[llength $trig] > 0} {
    set_property OUTPUT_VALUE 1 [lindex $trig 0]
    commit_hw_vio $vio
    after 100
    set_property OUTPUT_VALUE 0 [lindex $trig 0]
    commit_hw_vio $vio
    after 2000
    refresh_hw_vio $vio

    puts ""
    puts "============================================================"
    puts "  Re-triggered Results"
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
            puts "  $label: [get_property INPUT_VALUE [lindex $p 0]]"
        }
    }
}

puts ""
puts "============================================================"
puts "  COMPLETE"
puts "============================================================"
close_hw_target
disconnect_hw_server
close_hw_manager
