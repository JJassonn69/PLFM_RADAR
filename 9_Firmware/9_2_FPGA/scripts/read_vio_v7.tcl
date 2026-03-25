# read_vio_v7.tcl
#
# Strategy: 
# 1. Reduce JTAG TCK speed (STARTUPE2 CFGMCLK is uncalibrated 50-100 MHz)
# 2. Use proven 0010->0100 mask sequence
# 3. Do NOT call refresh_hw_vio (it fails with Xicom error)
#    Instead, rely on refresh_hw_device which already uploads probe values
#    ("Uploading output probe values for VIO core" message)
# 4. Read cached values directly from probe objects

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

# Reduce JTAG TCK speed BEFORE programming
# Default is ~15 MHz; try 1 MHz to help with STARTUPE2 clock uncertainty
set tgt [get_hw_targets *]
puts "INFO: Setting JTAG TCK to 1 MHz..."
catch {set_property PARAM.FREQUENCY 1000000 $tgt} tck_err
if {$tck_err ne ""} {
    puts "WARNING: Could not set TCK frequency: $tck_err"
    # Try alternate property name
    catch {set_property JTAG.FREQUENCY 1000000 $tgt} tck_err2
    if {$tck_err2 ne ""} {
        puts "WARNING: Could not set JTAG frequency either: $tck_err2"
    }
}

set fpga [lindex [get_hw_devices] 0]
current_hw_device $fpga

set_property PROGRAM.FILE $bit_file $fpga
if {[file exists $ltx_file]} {
    set_property PROBES.FILE $ltx_file $fpga
    set_property FULL_PROBES.FILE $ltx_file $fpga
    puts "INFO: LTX probes file loaded"
} else {
    puts "WARNING: LTX file not found: $ltx_file"
}

# Program
puts "INFO: Programming..."
program_hw_devices $fpga
puts "INFO: Programmed OK"

# Wait for STARTUPE2 oscillator to stabilize + self-test to complete
puts "INFO: Waiting 3s for STARTUPE2 stabilization + self-test..."
after 3000

# ============================================================
# APPROACH A: Try reading with reduced TCK and no refresh_hw_vio
# ============================================================

proc try_detect_and_read {fpga ltx_file attempt_label} {
    puts ""
    puts "============================================================"
    puts "  ATTEMPT: $attempt_label"
    puts "============================================================"
    
    # Prime with mask=0010 (always fails but initializes BSCAN state)
    puts "INFO: Priming with mask=0010..."
    set_property BSCAN_SWITCH_USER_MASK 0010 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga}
    
    # Switch to mask=0100 (consistently finds VIO after 0010 prime)
    puts "INFO: Switching to mask=0100..."
    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    set detect_err [catch {refresh_hw_device -update_hw_probes true $fpga} detect_msg]
    puts "INFO: refresh_hw_device result: rc=$detect_err msg=$detect_msg"
    
    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "INFO: VIOs found: [llength $vios]"
    
    if {[llength $vios] == 0} {
        puts "ERROR: No VIO detected in $attempt_label"
        return 0
    }
    
    set vio [lindex $vios 0]
    puts "INFO: VIO core: $vio"
    
    # DO NOT call refresh_hw_vio - this is what fails!
    # Instead, try reading the cached values from refresh_hw_device directly
    
    # List all probes
    set all_probes [get_hw_probes -quiet -of_objects $vio]
    puts "INFO: Total probes: [llength $all_probes]"
    
    if {[llength $all_probes] == 0} {
        puts "ERROR: No probes found"
        return 0
    }
    
    puts ""
    puts "------------------------------------------------------------"
    puts "  ALL PROBES (cached values, no refresh_hw_vio)"
    puts "------------------------------------------------------------"
    
    set read_count 0
    foreach p $all_probes {
        set pname [get_property NAME $p]
        set ptype [get_property TYPE $p]
        set pw [get_property WIDTH $p]
        
        # Try INPUT_VALUE first (for input probes), then OUTPUT_VALUE
        set val "(unread)"
        set rc1 [catch {set val [get_property INPUT_VALUE $p]} err1]
        if {$rc1 != 0} {
            set rc2 [catch {set val [get_property OUTPUT_VALUE $p]} err2]
            if {$rc2 != 0} {
                set val "(no value: $err1)"
            }
        }
        puts "  $ptype  $pname  \[${pw}b\]: $val"
        if {$val ne "(unread)" && $val ne "(no value: $err1)"} {
            incr read_count
        }
    }
    puts "INFO: Successfully read $read_count / [llength $all_probes] probes"
    
    # Self-test specific readout
    puts ""
    puts "------------------------------------------------------------"
    puts "  SELF-TEST RESULTS"
    puts "------------------------------------------------------------"
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
            set val "(read failed)"
            catch {set val [get_property INPUT_VALUE [lindex $p 0]]}
            set w [get_property WIDTH [lindex $p 0]]
            puts "  $label \[${w}b\]: $val"
        } else {
            puts "  $label: PROBE NOT FOUND"
        }
    }
    
    return $read_count
}

# ============================================================
# ATTEMPT 1: With 1 MHz TCK
# ============================================================
set result [try_detect_and_read $fpga $ltx_file "1 MHz TCK"]

if {$result == 0} {
    # ============================================================
    # ATTEMPT 2: Try even slower TCK (250 kHz)
    # ============================================================
    puts ""
    puts "INFO: Trying 250 kHz TCK..."
    set tgt [get_hw_targets *]
    catch {set_property PARAM.FREQUENCY 250000 $tgt}
    catch {set_property JTAG.FREQUENCY 250000 $tgt}
    
    set result [try_detect_and_read $fpga $ltx_file "250 kHz TCK"]
}

if {$result == 0} {
    # ============================================================
    # ATTEMPT 3: Disconnect, reconnect, full prime at 250 kHz
    # ============================================================
    puts ""
    puts "INFO: Full reconnect attempt..."
    close_hw_target
    disconnect_hw_server
    after 1000
    connect_hw_server -allow_non_jtag
    open_hw_target [lindex [get_hw_targets] 0]
    
    set tgt [get_hw_targets *]
    catch {set_property PARAM.FREQUENCY 250000 $tgt}
    catch {set_property JTAG.FREQUENCY 250000 $tgt}
    
    set fpga [lindex [get_hw_devices] 0]
    current_hw_device $fpga
    if {[file exists $ltx_file]} {
        set_property PROBES.FILE $ltx_file $fpga
        set_property FULL_PROBES.FILE $ltx_file $fpga
    }
    
    set result [try_detect_and_read $fpga $ltx_file "Reconnect + 250 kHz TCK"]
}

if {$result == 0} {
    # ============================================================
    # ATTEMPT 4: Try mask=1000 (USER4) in case scan chain mapping differs
    # ============================================================
    puts ""
    puts "INFO: Trying alternative mask sequences..."
    
    # Try 0010 -> 1000
    puts "INFO: Prime 0010, then 1000..."
    set_property BSCAN_SWITCH_USER_MASK 0010 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga}
    set_property BSCAN_SWITCH_USER_MASK 1000 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga}
    
    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "INFO: VIOs with mask=1000: [llength $vios]"
    
    # Try 0100 -> 0010
    puts "INFO: Prime 0100, then 0010..."
    set_property BSCAN_SWITCH_USER_MASK 0100 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga}
    set_property BSCAN_SWITCH_USER_MASK 0010 $fpga
    catch {refresh_hw_device -update_hw_probes true $fpga}
    
    set vios [get_hw_vios -quiet -of_objects $fpga]
    puts "INFO: VIOs with reversed prime: [llength $vios]"
}

# ============================================================
# ATTEMPT 5: Direct XSDB-style register read via run_hw_jtag
# If all VIO approaches fail, try raw JTAG scan chain access
# ============================================================
if {$result == 0} {
    puts ""
    puts "============================================================"
    puts "  ATTEMPT: Raw JTAG IDCODE read (sanity check)"
    puts "============================================================"
    # At minimum, try to read the device IDCODE to prove JTAG works
    set idcode [get_property REGISTER.IR.BIT_LENGTH $fpga]
    puts "INFO: IR length: $idcode"
    set usercode [catch {get_property REGISTER.USERCODE $fpga} uc_val]
    puts "INFO: USERCODE: rc=$usercode val=$uc_val"
}

puts ""
puts "============================================================"
puts "  SCRIPT COMPLETE"
puts "============================================================"
close_hw_target
disconnect_hw_server
close_hw_manager
