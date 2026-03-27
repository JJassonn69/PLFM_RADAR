# ============================================================================
# Vivado batch build script — Blinky FT601 stability test
# ============================================================================
# Minimal design: just blinks LED and holds FT601 bus idle.
# Uses same XDC as the full design (all pins constrained).
# ============================================================================

set proj_name "blinky_ft601_test"
set proj_dir  "[file normalize [file join [pwd] vivado_${proj_name}]]"
set src_dir   [pwd]
set xdc_file  "[file join $src_dir constraints te0713_te0701_umft601x.xdc]"
set top       "blinky_ft601_test"
set part      "xc7a200tfbg484-2"

# Reports directory
set reports_dir [file join $proj_dir reports]
file mkdir $reports_dir

# --- Create project ---
create_project $proj_name $proj_dir -part $part -force

# --- Add source ---
add_files -norecurse [file join $src_dir blinky_ft601_test.v]
set_property top $top [current_fileset]

# --- Add constraints ---
add_files -fileset constrs_1 -norecurse $xdc_file

# --- Synthesis ---
set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
launch_runs synth_1 -jobs 8
wait_on_run synth_1
open_run synth_1

# --- Report clocks ---
report_clocks -file [file join $reports_dir "clocks.rpt"]

# --- Implementation ---
set_property strategy Performance_Explore [get_runs impl_1]
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
open_run impl_1

# --- Reports ---
report_timing_summary -max_paths 10 -file [file join $reports_dir "timing_summary.rpt"]
report_drc -file [file join $reports_dir "drc.rpt"]
report_utilization -file [file join $reports_dir "utilization.rpt"]

puts "INFO: Blinky build complete."
puts "INFO: Reports: $reports_dir"
