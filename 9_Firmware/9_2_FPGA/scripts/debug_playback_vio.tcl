# debug_playback_vio.tcl
#
# Diagnostic script for VIO detection failure on playback bitstream.
# Strategy:
#   Phase A: Program KNOWN WORKING VIO self-test bitstream, verify chain 3 works
#   Phase B: Program playback bitstream, try all masks + prime sequences
#   Phase C: Try alternate LTX file (debug_nets.ltx)
#
# This isolates whether the problem is JTAG/hardware vs the playback build.

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"

# Known-working VIO self-test build
set vio_build_dir [file join $project_root "vivado_te0713_vio"]
set vio_bit [glob -nocomplain [file join $vio_build_dir "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
set vio_bit [lindex $vio_bit 0]
set vio_ltx [file join [file dirname $vio_bit] "radar_system_top_te0713_vio.ltx"]

# Playback build
set pb_build_dir [file join $project_root "vivado_te0713_playback"]
set pb_bit [glob -nocomplain [file join $pb_build_dir "aeris10_te0713_playback.runs" "impl_1" "*.bit"]]
set pb_bit [lindex $pb_bit 0]
set pb_ltx [file join [file dirname $pb_bit] "radar_system_top_te0713_playback.ltx"]
set pb_ltx_debug [file join [file dirname $pb_bit] "debug_nets.ltx"]

puts "============================================================"
puts "  FILE PATHS"
puts "============================================================"
puts "  VIO self-test bit: $vio_bit"
puts "  VIO self-test ltx: $vio_ltx"
puts "  Playback bit:      $pb_bit"
puts "  Playback ltx:      $pb_ltx"
puts "  Playback dbg ltx:  $pb_ltx_debug"
puts ""
puts "  VIO bit exists: [file exists $vio_bit]"
puts "  VIO ltx exists: [file exists $vio_ltx]"
puts "  PB bit exists:  [file exists $pb_bit]"
puts "  PB ltx exists:  [file exists $pb_ltx]"
puts "  PB dbg exists:  [file exists $pb_ltx_debug]"
puts ""

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
    if {$rc != 0} {
        puts "  Error msg: $msg"
    }
    
    if {$nvio > 0} {
        set vio [lindex $vios 0]
        set probes [get_hw_probes -quiet -of_objects $vio]
        puts "  Probes: [llength $probes]"
        
        # Try reading cached values
        set has_data 0
        foreach p $probes {
            set pname [get_property NAME $p]
            set ptype [get_property TYPE $p]
            if {$ptype eq "INPUT"} {
                set val ""
                catch {set val [get_property INPUT_VALUE $p]}
                if {$val ne ""} {
                    set has_data 1
                    puts "  $pname = '$val'"
                }
            }
        }
        
        if {!$has_data} {
            puts "  Cached values blank, trying refresh_hw_vio..."
            set rrc [catch {refresh_hw_vio $vio} rmsg]
            puts "  refresh_hw_vio: rc=$rrc, msg=$rmsg"
            if {$rrc == 0} {
                foreach p $probes {
                    set pname [get_property NAME $p]
                    set ptype [get_property TYPE $p]
                    if {$ptype eq "INPUT"} {
                        set val ""
                        catch {set val [get_property INPUT_VALUE $p]}
                        if {$val ne ""} {
                            set has_data 1
                            puts "  $pname = '$val' (after refresh)"
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
# Connect to JTAG
# ============================================================
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target [lindex [get_hw_targets] 0]

set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga

puts "INFO: Connected to FPGA: $fpga"

# ============================================================
# PHASE A: Program known-working VIO self-test bitstream
# ============================================================
puts ""
puts "============================================================"
puts "  PHASE A: PROGRAM KNOWN-WORKING VIO SELF-TEST BITSTREAM"
puts "============================================================"

set_property PROGRAM.FILE $vio_bit $fpga
set_property PROBES.FILE $vio_ltx $fpga
set_property FULL_PROBES.FILE $vio_ltx $fpga

puts "INFO: Programming VIO self-test bitstream..."
set rc [catch {program_hw_devices $fpga} msg]
puts "INFO: Program rc=$rc"
if {$rc != 0} {
    puts "ERROR: Programming failed: $msg"
    puts "ERROR: JTAG chain may be broken! Aborting."
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    return
}
puts "INFO: VIO self-test bitstream programmed OK"

# Wait for stabilization
puts "INFO: Waiting 3s for stabilization..."
after 3000

# Try chain 3 (the known-working mask)
puts "INFO: Testing chain 3 mask=0100..."
set vio_result [try_mask $fpga "0100" "VIO self-test chain 3"]

if {$vio_result} {
    puts ""
    puts "*** PHASE A RESULT: VIO SELF-TEST WORKS - JTAG chain is healthy ***"
} else {
    puts ""
    puts "*** PHASE A RESULT: VIO SELF-TEST ALSO FAILS - JTAG chain issue! ***"
    puts "*** Trying all masks as fallback... ***"
    
    set found 0
    foreach {mask label} {
        0010 "USER2"
        1000 "USER4"
        0001 "USER1"
        1111 "ALL"
    } {
        set result [try_mask $fpga $mask $label]
        if {$result} {
            puts "  >>> VIO self-test works with mask=$mask <<<"
            set found 1
            break
        }
    }
    
    if {!$found} {
        puts ""
        puts "*** CRITICAL: VIO self-test doesn't work with ANY mask ***"
        puts "*** JTAG chain may be broken. Stopping. ***"
        close_hw_target
        disconnect_hw_server
        close_hw_manager
        return
    }
}

# ============================================================
# PHASE B: Program playback bitstream with primary LTX
# ============================================================
puts ""
puts "============================================================"
puts "  PHASE B: PROGRAM PLAYBACK BITSTREAM (primary LTX)"
puts "============================================================"

set_property PROGRAM.FILE $pb_bit $fpga
set_property PROBES.FILE $pb_ltx $fpga
set_property FULL_PROBES.FILE $pb_ltx $fpga

puts "INFO: Programming playback bitstream..."
set rc [catch {program_hw_devices $fpga} msg]
puts "INFO: Program rc=$rc"
if {$rc != 0} {
    puts "ERROR: Playback programming failed: $msg"
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    return
}
puts "INFO: Playback bitstream programmed OK"

puts "INFO: Waiting 3s for stabilization..."
after 3000

# Try all masks
set pb_found 0
foreach {mask label} {
    0100 "Playback chain 3 (primary)"
    0010 "Playback USER2"
    1000 "Playback USER4"
    0001 "Playback USER1"
    1111 "Playback ALL"
} {
    set result [try_mask $fpga $mask $label]
    if {$result} {
        puts "  >>> PLAYBACK VIO works with mask=$mask <<<"
        set pb_found 1
        break
    }
}

# Try prime sequences if no luck
if {!$pb_found} {
    puts ""
    puts "--- Prime sequences for playback ---"
    foreach {prime_mask read_mask label} {
        0010 0100 "Prime USER2 -> Read USER3"
        0100 0010 "Prime USER3 -> Read USER2"
        0001 0100 "Prime USER1 -> Read USER3"
        1000 0100 "Prime USER4 -> Read USER3"
    } {
        puts ""
        puts "--- $label ---"
        set_property BSCAN_SWITCH_USER_MASK $prime_mask $fpga
        catch {refresh_hw_device -update_hw_probes true $fpga}
        
        set result [try_mask $fpga $read_mask "Then $label"]
        if {$result} {
            puts "  >>> PLAYBACK VIO with prime=$prime_mask read=$read_mask <<<"
            set pb_found 1
            break
        }
    }
}

# ============================================================
# PHASE C: Try with debug_nets.ltx if primary LTX failed
# ============================================================
if {!$pb_found && [file exists $pb_ltx_debug]} {
    puts ""
    puts "============================================================"
    puts "  PHASE C: TRY debug_nets.ltx"
    puts "============================================================"
    
    # Re-program (LTX is loaded at program time)
    set_property PROBES.FILE $pb_ltx_debug $fpga
    set_property FULL_PROBES.FILE $pb_ltx_debug $fpga
    
    # Don't need to reprogram the bit file, just refresh with new LTX
    puts "INFO: Trying with debug_nets.ltx..."
    
    foreach {mask label} {
        0100 "debug_nets chain 3"
        0010 "debug_nets USER2"
        1000 "debug_nets USER4"
        0001 "debug_nets USER1"
        1111 "debug_nets ALL"
    } {
        set result [try_mask $fpga $mask $label]
        if {$result} {
            puts "  >>> debug_nets.ltx works with mask=$mask <<<"
            set pb_found 1
            break
        }
    }
}

# ============================================================
# PHASE D: Try with NO LTX (just detect debug hub)
# ============================================================
if {!$pb_found} {
    puts ""
    puts "============================================================"
    puts "  PHASE D: TRY WITHOUT LTX (raw debug hub detection)"
    puts "============================================================"
    
    set_property PROBES.FILE "" $fpga
    set_property FULL_PROBES.FILE "" $fpga
    
    foreach {mask label} {
        0100 "No-LTX chain 3"
        0010 "No-LTX USER2"
        1000 "No-LTX USER4"
        0001 "No-LTX USER1"
        1111 "No-LTX ALL"
    } {
        puts ""
        puts "--- $label: mask=$mask ---"
        set_property BSCAN_SWITCH_USER_MASK $mask $fpga
        set rc [catch {refresh_hw_device $fpga} msg]
        puts "  rc=$rc"
        if {$rc != 0} {
            puts "  msg: $msg"
        }
        
        # Check if any debug cores detected even without probes
        set vios [get_hw_vios -quiet -of_objects $fpga]
        puts "  VIOs detected (no probes): [llength $vios]"
        if {[llength $vios] > 0} {
            puts "  >>> Debug hub found without LTX at mask=$mask <<<"
            set pb_found 1
            break
        }
    }
}

# ============================================================
# FINAL SUMMARY
# ============================================================
puts ""
puts "============================================================"
puts "  DIAGNOSTIC SUMMARY"
puts "============================================================"
puts "  VIO self-test bitstream: [expr {$vio_result ? "WORKS" : "FAILED"}]"
puts "  Playback bitstream VIO:  [expr {$pb_found ? "WORKS" : "FAILED"}]"
puts ""
if {!$pb_found} {
    puts "  DIAGNOSIS: The debug hub in the playback bitstream is not responding."
    puts "  Possible causes:"
    puts "    1. Debug hub clock (clk_buf) not reaching the MMCM"
    puts "    2. Debug hub was optimized away or incorrectly connected"
    puts "    3. BSCANE2 primitive not properly connected to debug hub"
    puts "    4. Need to open the implemented design and check debug hub connectivity"
    puts ""
    puts "  Recommended next step: Open the playback design in Vivado GUI mode"
    puts "  and run: report_debug_core -name DbgReport"
}
puts "============================================================"

close_hw_target
disconnect_hw_server
close_hw_manager
