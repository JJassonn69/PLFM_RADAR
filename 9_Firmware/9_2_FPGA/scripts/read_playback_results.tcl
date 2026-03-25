# read_playback_results.tcl
#
# Programs the BRAM playback bitstream, triggers playback via VIO,
# and reads CFAR detection results over JTAG.
#
# VIO "vio_playback" probes:
#   Input (FPGA -> Vivado):
#     probe_in0  [15:0]  detect_count
#     probe_in1  [5:0]   detect_range
#     probe_in2  [4:0]   detect_doppler
#     probe_in3  [16:0]  detect_magnitude
#     probe_in4  [0:0]   detect_flag
#     probe_in5  [0:0]   cfar_busy
#     probe_in6  [0:0]   doppler_frame_done
#     probe_in7  [0:0]   playback_done
#     probe_in8  [0:0]   playback_active
#     probe_in9  [5:0]   chirp_count
#     probe_in10 [31:0]  heartbeat
#     probe_in11 [4:0]   self_test_flags
#     probe_in12 [7:0]   self_test_detail
#     probe_in13 [0:0]   self_test_done
#     probe_in14 [16:0]  detect_threshold
#     probe_in15 [7:0]   cfar_status
#
#   Output (Vivado -> FPGA):
#     probe_out0 [0:0]   playback_trigger
#     probe_out1 [0:0]   cfar_enable
#     probe_out2 [0:0]   mti_enable
#     probe_out3 [2:0]   dc_notch_width
#     probe_out4 [0:0]   trigger_self_test
#
# Usage:
#   vivado -mode batch -source scripts/read_playback_results.tcl

set project_root "/home/jason-stone/PLFM_RADAR_work/PLFM_RADAR/9_Firmware/9_2_FPGA"
set build_dir [file join $project_root "vivado_te0713_playback"]

set bit_file [glob -nocomplain [file join $build_dir "aeris10_te0713_playback.runs" "impl_1" "*.bit"]]
set bit_file [lindex $bit_file 0]
set ltx_file [file join [file dirname $bit_file] "radar_system_top_te0713_playback.ltx"]

puts "INFO: bit_file = $bit_file"
puts "INFO: ltx_file = $ltx_file"

# ==========================================================================
# Connect and program
# ==========================================================================
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

puts "INFO: Programming..."
program_hw_devices $fpga
puts "INFO: Programmed OK"

# Wait for POR + auto self-test
puts "INFO: Waiting 3s for stabilization + self-test..."
after 3000

# ==========================================================================
# Detect VIO (chain 3, mask=0100)
# ==========================================================================
puts "INFO: Setting BSCAN mask for chain 3..."
set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
refresh_hw_device -update_hw_probes true $fpga

set vios [get_hw_vios -quiet -of_objects $fpga]
puts "INFO: VIO cores found: [llength $vios]"
if {[llength $vios] == 0} {
    error "No VIO cores detected! Check chain 3 build."
}

set vio [lindex $vios 0]
set probes [get_hw_probes -quiet -of_objects $vio]
puts "INFO: Total probes: [llength $probes]"

# ==========================================================================
# Helper proc to read a probe value
# ==========================================================================
proc read_probe {vio pattern} {
    set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pattern}*"]
    if {[llength $p] > 0} {
        set val ""
        catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
        return $val
    }
    return "(not found)"
}

proc write_probe {vio pattern value} {
    set p [get_hw_probes -quiet -of_objects $vio -filter "NAME =~ *${pattern}*"]
    if {[llength $p] > 0} {
        set_property OUTPUT_VALUE $value [lindex $p 0]
        commit_hw_vio [lindex $p 0]
        return 1
    }
    return 0
}

# ==========================================================================
# Phase 1: Read self-test results (auto-triggered on POR)
# ==========================================================================
puts ""
puts "============================================================"
puts "  PHASE 1: SELF-TEST RESULTS"
puts "============================================================"

set st_flags  [read_probe $vio "result_flags_latched"]
set st_detail [read_probe $vio "result_detail_latched"]
set st_done   [read_probe $vio "test_done_latched"]
set hb        [read_probe $vio "hb_counter"]

puts "  Self-test flags:  $st_flags (expect 0F = BRAM/CIC/FFT/ARITH pass)"
puts "  Self-test detail: $st_detail (expect AD = ADC timeout)"
puts "  Self-test done:   $st_done (expect 1)"
puts "  Heartbeat:        $hb (should be nonzero/incrementing)"

# ==========================================================================
# Phase 2: Configure CFAR and trigger playback
# ==========================================================================
puts ""
puts "============================================================"
puts "  PHASE 2: CONFIGURE AND TRIGGER PLAYBACK"
puts "============================================================"

# First read pre-playback state
set pb_done   [read_probe $vio "playback_done"]
set pb_active [read_probe $vio "playback_active"]
puts "  Pre-trigger: playback_done=$pb_done, playback_active=$pb_active"

# Enable CFAR (simple threshold mode — cfar_enable=0 means simple threshold)
# NOTE: Use Verilog net names (from LTX), NOT probe_outN indices.
puts "  Setting CFAR enable=0 (simple threshold), MTI=0 (off), DC notch=0 (off)..."
write_probe $vio "cfar_enable"    0   ;# cfar_enable = 0 (simple threshold)
write_probe $vio "mti_enable"     0   ;# mti_enable = 0
write_probe $vio "dc_notch_width" 0   ;# dc_notch_width = 0

# Trigger playback (rising edge)
puts "  Triggering playback..."
write_probe $vio "playback_trigger" 1   ;# playback_trigger = 1
after 100
write_probe $vio "playback_trigger" 0   ;# playback_trigger = 0

# ==========================================================================
# Phase 3: Wait for playback to complete
# ==========================================================================
puts ""
puts "============================================================"
puts "  PHASE 3: WAITING FOR PLAYBACK + PROCESSING"
puts "============================================================"

# Playback takes ~32 * (1024 + 200) = ~39,168 clocks at ~65 MHz ≈ ~0.6ms
# Doppler processing takes additional ~200k clocks ≈ ~3ms
# CFAR takes additional ~50k clocks ≈ ~0.8ms
# Total: ~5ms. Wait 5 seconds for safety (VIO read latency dominates).

puts "  Waiting 5 seconds for pipeline to complete..."
after 5000

# Refresh VIO to get latest values
refresh_hw_device -update_hw_probes true $fpga

# ==========================================================================
# Phase 4: Read results
# ==========================================================================
puts ""
puts "============================================================"
puts "  PHASE 4: PLAYBACK RESULTS"
puts "============================================================"

set pb_done     [read_probe $vio "playback_done"]
set pb_active   [read_probe $vio "playback_active"]
set chirp_cnt   [read_probe $vio "chirp_count"]
set dop_done    [read_probe $vio "doppler_frame_done"]
set cfar_busy   [read_probe $vio "cfar_busy"]
set cfar_status [read_probe $vio "cfar_status"]
set det_count   [read_probe $vio "detect_count"]
set det_flag    [read_probe $vio "detect_flag"]
set det_range   [read_probe $vio "detect_range"]
set det_doppler [read_probe $vio "detect_doppler"]
set det_mag     [read_probe $vio "detect_mag"]
set det_thr     [read_probe $vio "detect_thr"]
set hb2         [read_probe $vio "hb_counter"]

puts "  Playback done:       $pb_done (expect 1)"
puts "  Playback active:     $pb_active (expect 0)"
puts "  Chirp count:         $chirp_cnt (expect 32 = 100000b)"
puts "  Doppler frame done:  $dop_done (expect 1)"
puts "  CFAR busy:           $cfar_busy (expect 0)"
puts "  CFAR status:         $cfar_status"
puts ""
puts "  DETECTION RESULTS:"
puts "  ------------------"
puts "  Detection count:     $det_count"
puts "  Detection flag:      $det_flag"
puts "  Last detect range:   $det_range"
puts "  Last detect Doppler: $det_doppler"
puts "  Last detect mag:     $det_mag"
puts "  Last detect thresh:  $det_thr"
puts ""
puts "  Heartbeat:           $hb2"

# ==========================================================================
# Phase 5: Try with CFAR enabled (CA-CFAR mode)
# ==========================================================================
puts ""
puts "============================================================"
puts "  PHASE 5: RE-RUN WITH CA-CFAR ENABLED"
puts "============================================================"

write_probe $vio "cfar_enable" 1   ;# cfar_enable = 1 (CA-CFAR)

# Re-trigger playback
puts "  Triggering playback with CA-CFAR..."
write_probe $vio "playback_trigger" 1
after 100
write_probe $vio "playback_trigger" 0

puts "  Waiting 5 seconds..."
after 5000

refresh_hw_device -update_hw_probes true $fpga

set det_count2   [read_probe $vio "detect_count"]
set det_flag2    [read_probe $vio "detect_flag"]
set det_range2   [read_probe $vio "detect_range"]
set det_doppler2 [read_probe $vio "detect_doppler"]
set det_mag2     [read_probe $vio "detect_mag"]
set det_thr2     [read_probe $vio "detect_thr"]
set cfar_stat2   [read_probe $vio "cfar_status"]
set pb_done2     [read_probe $vio "playback_done"]

puts "  Playback done:       $pb_done2"
puts ""
puts "  CA-CFAR DETECTION RESULTS:"
puts "  --------------------------"
puts "  Detection count:     $det_count2"
puts "  Detection flag:      $det_flag2"
puts "  Last detect range:   $det_range2"
puts "  Last detect Doppler: $det_doppler2"
puts "  Last detect mag:     $det_mag2"
puts "  Last detect thresh:  $det_thr2"
puts "  CFAR status:         $cfar_stat2"

# ==========================================================================
# Summary
# ==========================================================================
puts ""
puts "============================================================"
puts "  SUMMARY"
puts "============================================================"
puts "  Pipeline: BRAM -> Decim(peak,1024->64) -> MTI(off) -> Doppler(2x16pt) -> CFAR"
puts "  Data: ADI CN0566 10.525 GHz FMCW, 32 chirps x 1024 range bins"
puts "  Clock: STARTUPE2 CFGMCLK (~65 MHz)"
puts ""
puts "  Simple Threshold Mode:"
puts "    Detections: $det_count"
puts ""
puts "  CA-CFAR Mode:"
puts "    Detections: $det_count2"
puts "============================================================"

close_hw_target
disconnect_hw_server
close_hw_manager
