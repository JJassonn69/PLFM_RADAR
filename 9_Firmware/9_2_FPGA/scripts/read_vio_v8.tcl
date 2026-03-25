# read_vio_v8.tcl
#
# VIO reader for chain 3 rebuild.
# Strategy:
# 1. Program bitstream
# 2. Try each mask individually AND with prime sequences
# 3. When VIO detected, skip refresh_hw_vio (known to fail with Xicom error)
# 4. Read INPUT_VALUE directly from cached probe data
# 5. If blank, try refresh_hw_vio as last resort

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_vio"]

set bit_file [glob -nocomplain [file join $build_dir "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
set bit_file [lindex $bit_file 0]
set ltx_file [file join [file dirname $bit_file] "radar_system_top_te0713_vio.ltx"]

puts "INFO: bit_file = $bit_file"
puts "INFO: ltx_file = $ltx_file"

# Connect
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target [lindex [get_hw_targets] 0]

set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga

set_property PROGRAM.FILE $bit_file $fpga
if {[file exists $ltx_file]} {
    set_property PROBES.FILE $ltx_file $fpga
    set_property FULL_PROBES.FILE $ltx_file $fpga
    puts "INFO: LTX probes file loaded"
}

# Program
puts "INFO: Programming..."
program_hw_devices $fpga
puts "INFO: Programmed OK"

# Wait for STARTUPE2 + self-test
puts "INFO: Waiting 3s for stabilization..."
after 3000

# ============================================================
# Helper: try a mask and report results
# ============================================================
proc try_mask {fpga mask label} {
    puts ""
    puts "--- $label: mask=$mask ---"
    set_property BSCAN_SWITCH_USER_MASK $mask $fpga
    set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]
    
    set vios [get_hw_vios -quiet -of_objects $fpga]
    set nvio [llength $vios]
    puts "  Result: rc=$rc, VIOs=$nvio"
    
    if {$nvio > 0} {
        set vio [lindex $vios 0]
        set probes [get_hw_probes -quiet -of_objects $vio]
        puts "  Probes: [llength $probes]"
        
        # Try reading without refresh_hw_vio first
        set has_data 0
        foreach pname {result_flags_latched hb_counter test_done_latched} {
            set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pname}*"]
            if {[llength $p] > 0} {
                set val ""
                catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
                puts "  $pname = '$val'"
                if {$val ne ""} {
                    set has_data 1
                }
            }
        }
        
        if {!$has_data} {
            # Try refresh_hw_vio as last resort
            puts "  Cached values blank, trying refresh_hw_vio..."
            set rrc [catch {refresh_hw_vio $vio} rmsg]
            puts "  refresh_hw_vio: rc=$rrc"
            if {$rrc == 0} {
                foreach pname {result_flags_latched hb_counter test_done_latched} {
                    set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pname}*"]
                    if {[llength $p] > 0} {
                        set val ""
                        catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
                        puts "  $pname = '$val' (after refresh)"
                        if {$val ne ""} {
                            set has_data 1
                        }
                    }
                }
            }
        }
        
        return $has_data
    }
    return 0
}

# ============================================================
# Phase 1: Try each mask individually
# ============================================================
puts ""
puts "============================================================"
puts "  PHASE 1: Individual masks (design built with chain 3)"
puts "============================================================"

set found 0
foreach {mask label} {
    0100 "USER3 (chain 3 direct)"
    0010 "USER2"
    1000 "USER4"
    0001 "USER1"
    1111 "ALL"
} {
    set result [try_mask $fpga $mask $label]
    if {$result} {
        puts "  >>> DATA READ SUCCESSFULLY with mask=$mask <<<"
        set found 1
        break
    }
}

# ============================================================
# Phase 2: Prime sequences (if no data yet)
# ============================================================
if {!$found} {
    puts ""
    puts "============================================================"
    puts "  PHASE 2: Prime sequences"
    puts "============================================================"
    
    # All 2-step prime combinations
    foreach {prime_mask read_mask label} {
        0010 0100 "Prime USER2 -> Read USER3"
        0100 0010 "Prime USER3 -> Read USER2"
        0001 0100 "Prime USER1 -> Read USER3"
        0100 1000 "Prime USER3 -> Read USER4"
        1000 0100 "Prime USER4 -> Read USER3"
        0010 1000 "Prime USER2 -> Read USER4"
        0001 1000 "Prime USER1 -> Read USER4"
        0001 0010 "Prime USER1 -> Read USER2"
    } {
        puts ""
        puts "--- $label ---"
        puts "  Priming with mask=$prime_mask..."
        set_property BSCAN_SWITCH_USER_MASK $prime_mask $fpga
        catch {refresh_hw_device -update_hw_probes true $fpga}
        
        set result [try_mask $fpga $read_mask "  Then read mask=$read_mask"]
        if {$result} {
            puts "  >>> DATA READ with prime=$prime_mask read=$read_mask <<<"
            set found 1
            break
        }
    }
}

# ============================================================
# Phase 3: Full dump of whatever we have
# ============================================================
puts ""
puts "============================================================"
puts "  PHASE 3: Final state dump"
puts "============================================================"

set vios [get_hw_vios -quiet -of_objects $fpga]
puts "Final VIO count: [llength $vios]"

if {[llength $vios] > 0} {
    set vio [lindex $vios 0]
    set all_probes [get_hw_probes -quiet -of_objects $vio]
    puts "Final probe count: [llength $all_probes]"
    
    foreach p $all_probes {
        set pname [get_property NAME $p]
        set ptype [get_property TYPE $p]
        set pw [get_property WIDTH $p]
        set val "(none)"
        if {[catch {set val [get_property INPUT_VALUE $p]}]} {
            if {[catch {set val [get_property OUTPUT_VALUE $p]}]} {
                set val "(error)"
            }
        }
        puts "  $ptype  $pname  ${pw}b  = '$val'"
    }
    
    # Self-test summary
    puts ""
    puts "------------------------------------------------------------"
    puts "  SELF-TEST SUMMARY"
    puts "------------------------------------------------------------"
    foreach {pname label} {
        result_flags_latched   "Result Flags (expect 0F)"
        result_detail_latched  "Result Detail (expect AD)"
        self_test_busy         "Busy (expect 0)"
        all_pass_latched       "All Pass (expect 0)"
        hb_counter             "Heartbeat (expect incrementing)"
        test_done_latched      "Test Done (expect 1)"
    } {
        set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pname}*"]
        if {[llength $p] > 0} {
            set val "(read failed)"
            catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
            set w [get_property WIDTH [lindex $p 0]]
            puts "  $label  ${w}b = '$val'"
        } else {
            puts "  $label  PROBE NOT FOUND"
        }
    }
}

puts ""
puts "============================================================"
puts "  SCRIPT COMPLETE (found_data=$found)"
puts "============================================================"
close_hw_target
disconnect_hw_server
close_hw_manager
