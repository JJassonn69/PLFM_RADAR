# ============================================================================
# AERIS-10 TE0713/TE0701 PLAYBACK BUILD CONSTRAINTS (INTERNAL CLOCK)
# ============================================================================
# Target part: XC7A200T-2FBG484C (TE0713-03-82C46-A)
# Board: TE0701-06 carrier
# Same pin assignments as te0713_vio.xdc — same physical board.
# ============================================================================

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design]

# Status/output IO standards (Bank 16, VCCO = 3.3V)
set_property IOSTANDARD LVCMOS33 [get_ports {user_led[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports {system_status[*]}]

# Clock constraint for internal STARTUPE2 CFGMCLK (~65 MHz)
create_clock -name clk_internal -period 15.000 [get_pins startup_inst/CFGMCLK]

# --------------------------------------------------------------------------
# Pin assignments (identical to VIO build)
# --------------------------------------------------------------------------

# user_led[0..3] -> FMC_LA14_N/P, FMC_LA13_N/P
set_property PACKAGE_PIN A19 [get_ports {user_led[0]}]
set_property PACKAGE_PIN A18 [get_ports {user_led[1]}]
set_property PACKAGE_PIN F20 [get_ports {user_led[2]}]
set_property PACKAGE_PIN F19 [get_ports {user_led[3]}]

# system_status[0..3] -> FMC_LA5_N/P, FMC_LA6_N/P
set_property PACKAGE_PIN F18 [get_ports {system_status[0]}]
set_property PACKAGE_PIN E18 [get_ports {system_status[1]}]
set_property PACKAGE_PIN C22 [get_ports {system_status[2]}]
set_property PACKAGE_PIN B22 [get_ports {system_status[3]}]
