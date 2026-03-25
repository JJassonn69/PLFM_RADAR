# diag_playback_detect.tcl
#
# 4-configuration playback diagnostic for the AERIS-10 radar FPGA.
# Programs the playback bitstream, triggers playback 4 times with different
# pipeline configurations, and reads detection results after each run.
#
# Requires RTL fixes:
#   - bram_playback.v: ST_DONE → ST_FRAME_PULSE on playback_start (re-trigger)
#   - cfar_ca.v: detect_count resets at each frame start
#
# Configurations tested:
#   A: Simple threshold (cfar=0, mti=0, dc_notch=0) — baseline
#   B: Simple threshold + MTI + DC notch (cfar=0, mti=1, dc_notch=1)
#   C: CA-CFAR only (cfar=1, mti=0, dc_notch=0)
#   D: CA-CFAR + MTI + DC notch (cfar=1, mti=1, dc_notch=1) — full chain

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_playback"]

set bit_file [glob -nocomplain [file join $build_dir "aeris10_te0713_playback.runs" "impl_1" "*.bit"]]
set bit_file [lindex $bit_file 0]
set ltx_file [file join [file dirname $bit_file] "radar_system_top_te0713_playback.ltx"]

puts "INFO: bit = $bit_file"
puts "INFO: ltx = $ltx_file"
puts "INFO: bit exists: [file exists $bit_file]"
puts "INFO: ltx exists: [file exists $ltx_file]"

# ====================================================================
# Connect and program
# ====================================================================
open_hw_manager
connect_hw_server -allow_non_jtag

set targets [get_hw_targets]
puts "INFO: Targets: $targets"
open_hw_target [lindex $targets 0]

set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga
puts "INFO: Device: $fpga, Part: [get_property PART $fpga]"

set_property PROGRAM.FILE $bit_file $fpga
set_property PROBES.FILE $ltx_file $fpga
set_property FULL_PROBES.FILE $ltx_file $fpga

puts ""
puts "=== PROGRAMMING PLAYBACK BITSTREAM ==="
program_hw_devices $fpga
puts "  Programmed OK"

# Wait for debug hub
puts "  Waiting 1500ms for debug hub..."
after 1500

set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
set rc [catch {refresh_hw_device -update_hw_probes true $fpga} msg]
set vios [get_hw_vios -quiet -of_objects $fpga]
puts "  VIO detection: rc=$rc count=[llength $vios]"

if {[llength $vios] == 0} {
    puts "ERROR: VIO not detected. Aborting."
    close_hw_target
    disconnect_hw_server
    close_hw_manager
    exit 1
}

set vio [lindex $vios 0]
puts "  >>> VIO DETECTED <<<"

# ====================================================================
# Self-test results (auto-ran on POR)
# ====================================================================
puts ""
puts "=== SELF-TEST RESULTS ==="
foreach {pat label} {
    result_flags_latched    "Result Flags"
    result_detail_latched   "Result Detail"
    test_done_latched       "Self-Test Done"
    hb_counter              "Heartbeat"
} {
    set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pat}*"]
    if {[llength $p] > 0} {
        set val "(fail)"
        catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
        puts "  $label = $val"
    } else {
        puts "  $label = PROBE NOT FOUND"
    }
}

# ====================================================================
# Helper: Read all detection results
# ====================================================================
proc read_results {vio label} {
    puts ""
    puts "  === $label ==="
    foreach {pat desc} {
        playback_done        "pb_done"
        playback_active      "pb_active"
        chirp_count          "chirp_count"
        doppler_frame_done   "doppler_done"
        cfar_busy            "cfar_busy"
        cfar_status          "cfar_status"
        detect_count         "detect_count"
        detect_flag          "detect_flag"
        detect_range         "last_range"
        detect_doppler       "last_doppler"
        detect_mag           "last_magnitude"
        detect_thr           "last_threshold"
        hb_counter           "heartbeat"
    } {
        set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pat}*"]
        if {[llength $p] > 0} {
            set val "(fail)"
            catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
            puts "    $desc = $val"
        } else {
            puts "    $desc = NOT FOUND"
        }
    }
}

# ====================================================================
# Helper: Set VIO config and trigger playback
# ====================================================================
proc run_config {vio fpga cfar_en mti_en dc_notch config_name} {
    puts ""
    puts "============================================================"
    puts "  CONFIG $config_name: cfar=$cfar_en mti=$mti_en dc_notch=$dc_notch"
    puts "============================================================"

    # Set configuration
    foreach {pat val} [list \
        cfar_enable     $cfar_en \
        mti_enable      $mti_en \
        dc_notch_width  $dc_notch \
    ] {
        set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pat}*"]
        if {[llength $p] > 0} {
            set_property OUTPUT_VALUE $val [lindex $p 0]
            commit_hw_vio [lindex $p 0]
        } else {
            puts "  WARNING: probe *${pat}* not found"
        }
    }
    puts "  Configuration set"

    # Trigger playback (rising edge)
    set p_trig [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *playback_trigger*"]
    if {[llength $p_trig] > 0} {
        set_property OUTPUT_VALUE 1 [lindex $p_trig 0]
        commit_hw_vio [lindex $p_trig 0]
        after 100
        set_property OUTPUT_VALUE 0 [lindex $p_trig 0]
        commit_hw_vio [lindex $p_trig 0]
        puts "  Triggered"
    } else {
        puts "  ERROR: playback_trigger probe not found!"
        return
    }

    # Wait for pipeline (playback + decimation + doppler + CFAR)
    # 32 chirps x 1024 samples + 31 gaps of 200 cycles + Doppler + CFAR ≈ 50000 cycles @ ~65 MHz ≈ 0.8ms
    # Plus CFAR phase 2 ≈ 8500 cycles. Being generous with 5 seconds.
    puts "  Waiting 5 seconds for pipeline..."
    after 5000

    # Refresh probes
    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga}

    # Read results
    read_results $vio "RESULTS: Config $config_name"
}

# ====================================================================
# Run all 4 configurations
# ====================================================================

# Config A: Simple threshold — baseline (already proven to work)
run_config $vio $fpga 0 0 0 "A (Simple Threshold)"

# Config B: Simple threshold + MTI + DC notch
run_config $vio $fpga 0 1 1 "B (Simple Thr + MTI + DC Notch)"

# Config C: CA-CFAR only
run_config $vio $fpga 1 0 0 "C (CA-CFAR only)"

# Config D: CA-CFAR + MTI + DC notch — full chain
run_config $vio $fpga 1 1 1 "D (CA-CFAR + MTI + DC Notch = FULL CHAIN)"

# ====================================================================
# Summary
# ====================================================================
puts ""
puts "============================================================"
puts "  ALL 4 CONFIGURATIONS COMPLETE"
puts "============================================================"
puts "  Compare detect_count values across configs to verify:"
puts "    A (baseline) should have the most detections (low threshold)"
puts "    B (MTI+DC notch) should have fewer (clutter removal)"
puts "    C (CA-CFAR) should have adaptive threshold detections"
puts "    D (full chain) should show refined detections"
puts "============================================================"

close_hw_target
disconnect_hw_server
close_hw_manager
exit 0
