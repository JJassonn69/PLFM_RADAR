# ============================================================================
# AERIS-10 TE0713/TE0701 VIO BUILD CONSTRAINTS
# ============================================================================
# Target part: XC7A200T-2FBG484C (TE0713-03-82C46-A)
# Board: TE0701-06 carrier
#
# Minimal constraint set for VIO debug build. No UART pins — self-test
# results are read via Vivado VIO over JTAG.
# ============================================================================

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design]

# Clock IOSTANDARD (Bank 14, VCCIO = 3.3V)
set_property IOSTANDARD LVCMOS33 [get_ports {clk_100m}]

# Status/output IO standards (Bank 16, VCCO = 3.3V)
set_property IOSTANDARD LVCMOS33 [get_ports {user_led[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports {system_status[*]}]

# Clock constraint (TE0713 FIFO0CLK = 50 MHz)
create_clock -name clk_100m -period 20.000 [get_ports {clk_100m}]
set_input_jitter [get_clocks clk_100m] 0.100

# --------------------------------------------------------------------------
# Pin assignments
# --------------------------------------------------------------------------
set_property PACKAGE_PIN U20 [get_ports {clk_100m}]

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

# --------------------------------------------------------------------------
# No UART pins in VIO build — self-test readback is via JTAG VIO core
# --------------------------------------------------------------------------
