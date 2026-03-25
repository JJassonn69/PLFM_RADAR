# diag_vio_playback.tcl
#
# Diagnostic script to debug VIO detection on playback bitstream.
# Strategy:
#   Phase A: Program the KNOWN WORKING VIO self-test bitstream and verify
#            chain 3 detection works (proves JTAG is healthy).
#   Phase B: Reprogram with the playback bitstream and use robust multi-mask
#            retry logic (adapted from read_vio_v8.tcl).
#   Phase C: If VIO detected on playback, read self-test + trigger playback
#            and read CFAR results.

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"

# Known-working VIO self-test build
set vio_build_dir [file join $project_root "vivado_te0713_vio"]
set vio_bit_file [glob -nocomplain [file join $vio_build_dir "aeris10_te0713_vio.runs" "impl_1" "*.bit"]]
set vio_bit_file [lindex $vio_bit_file 0]
set vio_ltx_file [file join [file dirname $vio_bit_file] "radar_system_top_te0713_vio.ltx"]

# Playback build
set pb_build_dir [file join $project_root "vivado_te0713_playback"]
set pb_bit_file [glob -nocomplain [file join $pb_build_dir "aeris10_te0713_playback.runs" "impl_1" "*.bit"]]
set pb_bit_file [lindex $pb_bit_file 0]
set pb_ltx_file [file join [file dirname $pb_bit_file] "radar_system_top_te0713_playback.ltx"]
# Also check for debug_nets.ltx as alternative
set pb_dbg_ltx [file join [file dirname $pb_bit_file] "debug_nets.ltx"]

puts "============================================================"
puts "  DIAGNOSTIC: VIO Playback Debug"
puts "============================================================"
puts "  VIO self-test bit: $vio_bit_file"
puts "  VIO self-test ltx: $vio_ltx_file"
puts "  Playback bit:      $pb_bit_file"
puts "  Playback ltx:      $pb_ltx_file"
puts "  Playback dbg ltx:  $pb_dbg_ltx"
puts "  VIO bit exists:    [file exists $vio_bit_file]"
puts "  VIO ltx exists:    [file exists $vio_ltx_file]"
puts "  PB bit exists:     [file exists $pb_bit_file]"
puts "  PB ltx exists:     [file exists $pb_ltx_file]"
puts "  PB dbg ltx exists: [file exists $pb_dbg_ltx]"
puts ""

# ============================================================
# Helper: try a mask, report results (from read_vio_v8.tcl)
# ============================================================
proc try_mask {fpga mask label {probe_patterns {}}} {
    puts ""
    puts "--- $label: mask=$mask ---"
    set_property BSCAN_SWITCH_USER_MASK $mask $fpga
    set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]

    set vios [get_hw_vios -quiet -of_objects $fpga]
    set nvio [llength $vios]
    puts "  Result: rc=$rc, VIOs=$nvio"
    if {$rc != 0} {
        puts "  Message: $msg"
    }

    if {$nvio > 0} {
        set vio [lindex $vios 0]
        set probes [get_hw_probes -quiet -of_objects $vio]
        puts "  Probes: [llength $probes]"

        # Use provided probe patterns, or default to common self-test probes
        if {[llength $probe_patterns] == 0} {
            set probe_patterns {hb_counter}
        }

        set has_data 0
        foreach pname $probe_patterns {
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
            puts "  Cached values blank, trying refresh_hw_vio..."
            set rrc [catch {refresh_hw_vio $vio} rmsg]
            puts "  refresh_hw_vio: rc=$rrc msg=$rmsg"
            if {$rrc == 0} {
                foreach pname $probe_patterns {
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
# Helper: full mask sweep with optional priming
# ============================================================
proc mask_sweep {fpga phase_label {probe_patterns {}}} {
    puts ""
    puts "============================================================"
    puts "  $phase_label: Individual masks"
    puts "============================================================"

    set found 0
    foreach {mask label} {
        0100 "USER3 (chain 3 direct)"
        0010 "USER2"
        1000 "USER4"
        0001 "USER1"
        1111 "ALL"
    } {
        set result [try_mask $fpga $mask $label $probe_patterns]
        if {$result} {
            puts "  >>> DATA READ SUCCESSFULLY with mask=$mask <<<"
            set found 1
            break
        }
    }

    if {!$found} {
        puts ""
        puts "  $phase_label: Prime sequences..."
        foreach {prime_mask read_mask label} {
            0010 0100 "Prime USER2 -> Read USER3"
            0100 0010 "Prime USER3 -> Read USER2"
            0001 0100 "Prime USER1 -> Read USER3"
            1000 0100 "Prime USER4 -> Read USER3"
        } {
            puts ""
            puts "--- $label ---"
            puts "  Priming with mask=$prime_mask..."
            set_property BSCAN_SWITCH_USER_MASK $prime_mask $fpga
            catch {refresh_hw_device -update_hw_probes true $fpga}

            set result [try_mask $fpga $read_mask "Read after prime" $probe_patterns]
            if {$result} {
                puts "  >>> DATA READ with prime=$prime_mask read=$read_mask <<<"
                set found 1
                break
            }
        }
    }

    return $found
}

# ============================================================
# Connect to JTAG
# ============================================================
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target [lindex [get_hw_targets] 0]
set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga

# ############################################################
# PHASE A: Test with KNOWN WORKING VIO self-test bitstream
# ############################################################
puts ""
puts "############################################################"
puts "  PHASE A: KNOWN WORKING VIO SELF-TEST BITSTREAM"
puts "############################################################"

set_property PROGRAM.FILE $vio_bit_file $fpga
if {[file exists $vio_ltx_file]} {
    set_property PROBES.FILE $vio_ltx_file $fpga
    set_property FULL_PROBES.FILE $vio_ltx_file $fpga
}

puts "INFO: Programming VIO self-test bitstream..."
program_hw_devices $fpga
puts "INFO: Programmed OK. Waiting 3s..."
after 3000

set vio_ok [mask_sweep $fpga "PHASE A (VIO self-test)" {result_flags_latched hb_counter test_done_latched}]

if {$vio_ok} {
    puts ""
    puts ">>> PHASE A PASSED: JTAG chain healthy, VIO self-test detected <<<"
} else {
    puts ""
    puts ">>> PHASE A FAILED: Even the known-working VIO build doesn't detect! <<<"
    puts ">>> This means JTAG/hardware issue, not a playback build problem. <<<"
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    exit 1
}

# ############################################################
# PHASE B: Reprogram with PLAYBACK bitstream
# ############################################################
puts ""
puts "############################################################"
puts "  PHASE B: PLAYBACK BITSTREAM"
puts "############################################################"

set_property PROGRAM.FILE $pb_bit_file $fpga
if {[file exists $pb_ltx_file]} {
    set_property PROBES.FILE $pb_ltx_file $fpga
    set_property FULL_PROBES.FILE $pb_ltx_file $fpga
    puts "INFO: Using primary LTX: $pb_ltx_file"
}

puts "INFO: Programming playback bitstream..."
program_hw_devices $fpga
puts "INFO: Programmed OK. Waiting 3s..."
after 3000

set pb_ok [mask_sweep $fpga "PHASE B (Playback, primary LTX)" {hb_counter playback_done detect_count}]

# If primary LTX didn't work, try debug_nets.ltx
if {!$pb_ok && [file exists $pb_dbg_ltx]} {
    puts ""
    puts "  Primary LTX failed. Trying debug_nets.ltx..."
    set_property PROBES.FILE $pb_dbg_ltx $fpga
    set_property FULL_PROBES.FILE $pb_dbg_ltx $fpga
    
    # Reprogram to reset debug hub state
    puts "INFO: Reprogramming with debug_nets.ltx..."
    program_hw_devices $fpga
    puts "INFO: Reprogrammed OK. Waiting 3s..."
    after 3000
    
    set pb_ok [mask_sweep $fpga "PHASE B (Playback, debug_nets.ltx)" {hb_counter playback_done detect_count}]
}

# If still not working, try with NO LTX (just detect debug hub)
if {!$pb_ok} {
    puts ""
    puts "  Both LTX files failed. Trying with no LTX (bare debug hub scan)..."
    set_property PROBES.FILE "" $fpga
    set_property FULL_PROBES.FILE "" $fpga
    
    puts "INFO: Reprogramming with no LTX..."
    program_hw_devices $fpga
    puts "INFO: Reprogrammed OK. Waiting 3s..."
    after 3000
    
    # Just check if debug hub responds at all
    foreach {mask label} {0100 "USER3" 0010 "USER2" 0001 "USER1" 1000 "USER4" 1111 "ALL"} {
        puts ""
        puts "--- Bare scan: $label mask=$mask ---"
        set_property BSCAN_SWITCH_USER_MASK $mask $fpga
        set rc [catch {refresh_hw_device $fpga} msg]
        puts "  rc=$rc"
        if {$rc != 0} {
            puts "  msg=$msg"
        }
        set vios [get_hw_vios -quiet -of_objects $fpga]
        puts "  VIOs=[llength $vios]"
        if {[llength $vios] > 0} {
            puts "  >>> Debug hub responded on mask=$mask without LTX! <<<"
            set pb_ok 2 ;# special value = hub found but no probes
            break
        }
    }
}

if {$pb_ok == 0} {
    puts ""
    puts "############################################################"
    puts "  DIAGNOSIS: Debug hub NOT responding on playback bitstream"
    puts "  but IS working on VIO self-test bitstream."
    puts ""
    puts "  This means the playback build has a debug hub clock or"
    puts "  connectivity issue. Likely causes:"
    puts "  1. clk_buf net not actually connected to debug hub"
    puts "  2. Debug hub clock MMCM not locking"
    puts "  3. Debug hub got optimized away or placed incorrectly"
    puts ""
    puts "  RECOMMENDED: Re-open the implemented design and check"
    puts "  debug hub clock connectivity with:"
    puts "    report_debug_core"
    puts "    report_clocks"
    puts "    get_nets clk_buf"
    puts "############################################################"
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    exit 1
}

# ############################################################
# PHASE C: Read results (only if VIO detected)
# ############################################################
if {$pb_ok >= 1} {
    puts ""
    puts "############################################################"
    puts "  PHASE C: READ PLAYBACK RESULTS"
    puts "############################################################"

    set vios [get_hw_vios -quiet -of_objects $fpga]
    if {[llength $vios] == 0} {
        puts "  ERROR: No VIO cores available for reading."
        close_hw_target
        disconnect_hw_server
        close_hw_manager
        exit 1
    }

    set vio [lindex $vios 0]
    set all_probes [get_hw_probes -quiet -of_objects $vio]
    puts "  VIO probes: [llength $all_probes]"

    # Dump ALL probe values
    puts ""
    puts "  --- ALL PROBE VALUES ---"
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
    puts "  --- SELF-TEST ---"
    foreach {pname label} {
        result_flags_latched   "Result Flags"
        result_detail_latched  "Result Detail"
        test_done_latched      "Self-Test Done"
        hb_counter        "Heartbeat"
    } {
        set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pname}*"]
        if {[llength $p] > 0} {
            set val "(fail)"
            catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
            puts "  $label = $val"
        }
    }

    # Read pre-playback state
    puts ""
    puts "  --- PRE-PLAYBACK STATE ---"
    foreach {pname label} {
        playback_done    "Playback done"
        playback_active  "Playback active"
        detect_count     "Detection count"
        chirp_count      "Chirp count"
    } {
        set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pname}*"]
        if {[llength $p] > 0} {
            set val "(fail)"
            catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
            puts "  $label = $val"
        }
    }

    # --- Trigger playback ---
    puts ""
    puts "  --- TRIGGERING PLAYBACK ---"

    # Set CFAR/MTI/DC controls first
    # NOTE: Use Verilog net names (from LTX), NOT probe_outN indices.
    foreach {probe_pat val desc} {
        cfar_enable      0   "cfar_enable=0 (simple threshold)"
        mti_enable       0   "mti_enable=0"
        dc_notch_width   0   "dc_notch_width=0"
    } {
        set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${probe_pat}*"]
        if {[llength $p] > 0} {
            set_property OUTPUT_VALUE $val [lindex $p 0]
            commit_hw_vio [lindex $p 0]
            puts "  Set $desc"
        } else {
            puts "  WARNING: probe for $probe_pat NOT FOUND"
        }
    }

    # Trigger playback (rising edge on playback_trigger)
    set p_trig [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *playback_trigger*"]
    if {[llength $p_trig] > 0} {
        set_property OUTPUT_VALUE 1 [lindex $p_trig 0]
        commit_hw_vio [lindex $p_trig 0]
        puts "  playback_trigger = 1"
        after 100
        set_property OUTPUT_VALUE 0 [lindex $p_trig 0]
        commit_hw_vio [lindex $p_trig 0]
        puts "  playback_trigger = 0"
    }

    # Wait for pipeline
    puts "  Waiting 5s for pipeline completion..."
    after 5000

    # Refresh and read results
    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga}

    puts ""
    puts "  --- PLAYBACK RESULTS ---"
    foreach {pname label} {
        playback_done        "Playback done (expect 1)"
        playback_active      "Playback active (expect 0)"
        chirp_count          "Chirp count (expect 100000)"
        doppler_frame_done   "Doppler frame done (expect 1)"
        cfar_busy            "CFAR busy (expect 0)"
        cfar_status          "CFAR status"
        detect_count         "Detection count"
        detect_flag          "Detection flag"
        detect_range         "Last detect range"
        detect_doppler       "Last detect Doppler"
        detect_mag           "Last detect magnitude"
        detect_thr           "Last detect threshold"
        hb_counter           "Heartbeat"
    } {
        set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pname}*"]
        if {[llength $p] > 0} {
            set val "(fail)"
            catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
            puts "  $label = $val"
        } else {
            puts "  $label = (PROBE NOT FOUND)"
        }
    }

    # --- Phase 2: CA-CFAR mode ---
    puts ""
    puts "  --- RE-TRIGGERING WITH CA-CFAR ---"
    set p_cfar [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *cfar_enable*"]
    if {[llength $p_cfar] > 0} {
        set_property OUTPUT_VALUE 1 [lindex $p_cfar 0]
        commit_hw_vio [lindex $p_cfar 0]
        puts "  cfar_enable = 1 (CA-CFAR)"
    } else {
        puts "  WARNING: cfar_enable probe NOT FOUND"
    }

    if {[llength $p_trig] > 0} {
        set_property OUTPUT_VALUE 1 [lindex $p_trig 0]
        commit_hw_vio [lindex $p_trig 0]
        after 100
        set_property OUTPUT_VALUE 0 [lindex $p_trig 0]
        commit_hw_vio [lindex $p_trig 0]
        puts "  Playback triggered"
    }

    puts "  Waiting 5s..."
    after 5000

    catch {refresh_hw_device -update_hw_probes true $fpga}

    puts ""
    puts "  --- CA-CFAR RESULTS ---"
    foreach {pname label} {
        playback_done        "Playback done"
        detect_count         "Detection count"
        detect_flag          "Detection flag"
        detect_range         "Last detect range"
        detect_doppler       "Last detect Doppler"
        detect_mag           "Last detect magnitude"
        detect_thr           "Last detect threshold"
        cfar_status          "CFAR status"
    } {
        set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pname}*"]
        if {[llength $p] > 0} {
            set val "(fail)"
            catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
            puts "  $label = $val"
        }
    }
}

puts ""
puts "############################################################"
puts "  DIAGNOSTIC COMPLETE"
puts "############################################################"
close_hw_target
disconnect_hw_server
close_hw_manager
