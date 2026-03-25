# diag_playback_only.tcl
#
# Program PLAYBACK bitstream and try VIO detection with same simple approach
# that worked for the VIO self-test bitstream.

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_playback"]

set bit_file [glob -nocomplain [file join $build_dir "aeris10_te0713_playback.runs" "impl_1" "*.bit"]]
set bit_file [lindex $bit_file 0]
set ltx_file [file join [file dirname $bit_file] "radar_system_top_te0713_playback.ltx"]

puts "INFO: bit = $bit_file"
puts "INFO: ltx = $ltx_file"
puts "INFO: bit exists = [file exists $bit_file]"
puts "INFO: ltx exists = [file exists $ltx_file]"

open_hw_manager
connect_hw_server -allow_non_jtag

set targets [get_hw_targets]
puts "INFO: Targets: $targets"

open_hw_target [lindex $targets 0]
set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga

# Load probes BEFORE programming (same as working diag_simple_vio.tcl)
set_property PROGRAM.FILE $bit_file $fpga
set_property PROBES.FILE $ltx_file $fpga
set_property FULL_PROBES.FILE $ltx_file $fpga

puts ""
puts "=== PROGRAMMING PLAYBACK BITSTREAM ==="
program_hw_devices $fpga
puts "  Programmed OK"

# Try detection with increasing delays
foreach delay {1000 3000 5000 8000 10000} {
    puts ""
    puts "=== ATTEMPT: wait ${delay}ms then mask=0100 ==="
    after $delay

    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]

    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "  rc=$rc VIOs=[llength $vios]"

    if {$rc != 0} {
        puts "  msg=$msg"
    }

    if {[llength $vios] > 0} {
        puts "  >>> VIO DETECTED! <<<"
        set vio [lindex $vios 0]
        set probes [get_hw_probes -quiet -of_objects $vio]
        puts "  Total probes: [llength $probes]"

        # Dump all probe values
        puts ""
        puts "  --- ALL PROBE VALUES ---"
        foreach p $probes {
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

        # Self-test results
        puts ""
        puts "  --- SELF-TEST ---"
        foreach {pattern label} {
            result_flags_latched   "Flags"
            result_detail_latched  "Detail"
            test_done_latched      "Done"
            hb_counter        "Heartbeat"
        } {
            set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pattern}*"]
            if {[llength $p] > 0} {
                set val "(fail)"
                catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
                puts "    $label = $val"
            } else {
                puts "    $label = NOT FOUND"
            }
        }

        # Configure: CFAR off (simple threshold), MTI off, DC notch off
        # NOTE: Use Verilog net names (from LTX), NOT probe_outN indices.
        puts ""
        puts "  --- CONFIGURE PIPELINE ---"
        foreach {pattern value desc} {
            cfar_enable      0   "cfar_enable=0 (simple threshold)"
            mti_enable       0   "mti_enable=0"
            dc_notch_width   0   "dc_notch_width=0"
        } {
            set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pattern}*"]
            if {[llength $p] > 0} {
                set_property OUTPUT_VALUE $value [lindex $p 0]
                commit_hw_vio [lindex $p 0]
                puts "    Set $desc"
            } else {
                puts "    WARNING: probe for $pattern NOT FOUND"
            }
        }

        # Trigger playback
        puts ""
        puts "  --- TRIGGER PLAYBACK ---"
        set p_trig [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *playback_trigger*"]
        if {[llength $p_trig] > 0} {
            set_property OUTPUT_VALUE 1 [lindex $p_trig 0]
            commit_hw_vio [lindex $p_trig 0]
            puts "    playback_trigger = 1"
            after 100
            set_property OUTPUT_VALUE 0 [lindex $p_trig 0]
            commit_hw_vio [lindex $p_trig 0]
            puts "    playback_trigger = 0"
        }

        # Wait for pipeline
        puts "    Waiting 5s for pipeline..."
        after 5000

        # Refresh and read results
        set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
        catch {refresh_hw_device -update_hw_probes true $fpga}

        puts ""
        puts "  --- PLAYBACK RESULTS (Simple Threshold) ---"
        foreach {pattern label} {
            playback_done        "Playback done (expect 1)"
            playback_active      "Playback active (expect 0)"
            chirp_count          "Chirp count (expect 100000)"
            doppler_frame_done   "Doppler done (expect 1)"
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
            set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pattern}*"]
            if {[llength $p] > 0} {
                set val "(fail)"
                catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
                puts "    $label = $val"
            } else {
                puts "    $label = PROBE NOT FOUND"
            }
        }

        # CA-CFAR mode
        puts ""
        puts "  --- RE-RUN WITH CA-CFAR ---"
        set p_cfar [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *cfar_enable*"]
        if {[llength $p_cfar] > 0} {
            set_property OUTPUT_VALUE 1 [lindex $p_cfar 0]
            commit_hw_vio [lindex $p_cfar 0]
            puts "    cfar_enable = 1"
        } else {
            puts "    WARNING: cfar_enable probe NOT FOUND"
        }

        if {[llength $p_trig] > 0} {
            set_property OUTPUT_VALUE 1 [lindex $p_trig 0]
            commit_hw_vio [lindex $p_trig 0]
            after 100
            set_property OUTPUT_VALUE 0 [lindex $p_trig 0]
            commit_hw_vio [lindex $p_trig 0]
            puts "    Triggered"
        }

        puts "    Waiting 5s..."
        after 5000

        catch {refresh_hw_device -update_hw_probes true $fpga}

        puts ""
        puts "  --- CA-CFAR RESULTS ---"
        foreach {pattern label} {
            playback_done      "Playback done"
            detect_count       "Detection count"
            detect_flag        "Detection flag"
            detect_range       "Last detect range"
            detect_doppler     "Last detect Doppler"
            detect_mag         "Last detect magnitude"
            detect_thr         "Last detect threshold"
            cfar_status        "CFAR status"
        } {
            set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pattern}*"]
            if {[llength $p] > 0} {
                set val "(fail)"
                catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
                puts "    $label = $val"
            }
        }

        puts ""
        puts "=== PLAYBACK DIAGNOSTIC COMPLETE ==="
        close_hw_target
        disconnect_hw_server
        close_hw_manager
        exit 0
    }
}

# If all delays fail, try ALL masks
puts ""
puts "=== ALL DELAYS FAILED. Trying other masks... ==="
foreach {mask label} {
    0010 "USER2"
    1000 "USER4"
    0001 "USER1"
    1111 "ALL"
} {
    puts ""
    puts "--- $label: mask=$mask ---"
    set_property BSCAN_SWITCH_USER_MASK $mask $fpga
    set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]
    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "  rc=$rc VIOs=[llength $vios]"
    if {[llength $vios] > 0} {
        puts "  >>> VIO DETECTED with $label <<<"
        break
    }
}

puts ""
puts "=== PLAYBACK VIO DETECTION FAILED ==="
puts "Debug hub not responding on any mask."

close_hw_target
disconnect_hw_server
close_hw_manager
