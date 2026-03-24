# ============================================================================
# AERIS-10 TE0713/TE0701 UART TX BLASTER — DIAGNOSTIC CONSTRAINTS
# ============================================================================
# This XDC is for the uart_tx_blaster_top diagnostic build.
# BOTH P19 and U18 are configured as OUTPUTS driving the same serial stream.
# This lets us determine which pin the TE0701 CPLD routes to FTDI Channel B.
# ============================================================================

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design]

# Clock (TE0713 FIFO0CLK at U20, Bank 14, 3.3V)
set_property IOSTANDARD LVCMOS33 [get_ports {clk_100m}]
create_clock -name clk_100m -period 20.000 [get_ports {clk_100m}]
set_input_jitter [get_clocks clk_100m] 0.100
set_property PACKAGE_PIN U20 [get_ports {clk_100m}]

# LEDs (Bank 16 FMC LA pins)
set_property IOSTANDARD LVCMOS33 [get_ports {user_led[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports {system_status[*]}]

set_property PACKAGE_PIN A19 [get_ports {user_led[0]}]
set_property PACKAGE_PIN A18 [get_ports {user_led[1]}]
set_property PACKAGE_PIN F20 [get_ports {user_led[2]}]
set_property PACKAGE_PIN F19 [get_ports {user_led[3]}]

set_property PACKAGE_PIN F18 [get_ports {system_status[0]}]
set_property PACKAGE_PIN E18 [get_ports {system_status[1]}]
set_property PACKAGE_PIN C22 [get_ports {system_status[2]}]
set_property PACKAGE_PIN B22 [get_ports {system_status[3]}]

# --------------------------------------------------------------------------
# UART TX Blaster pins — BOTH configured as OUTPUTS
# Both are in Bank 14 (VCCIO = 3.3V on TE0713).
#
# P19 = B14_L24_P -> JM1-92 -> JB1-91 (MIO14) -> CPLD -> FTDI BDBUS0
# U18 = B14_L18_N -> JM1-85 -> JB1-86 (MIO15) -> CPLD -> FTDI BDBUS1
#
# In normal operation, one of these would be FPGA TX (output) and the other
# FPGA RX (input). For this diagnostic, BOTH are outputs to see which one
# the CPLD actually routes to the FTDI's RX input.
# --------------------------------------------------------------------------
set_property PACKAGE_PIN P19 [get_ports {uart_pin_p19}]
set_property IOSTANDARD LVCMOS33 [get_ports {uart_pin_p19}]

set_property PACKAGE_PIN U18 [get_ports {uart_pin_u18}]
set_property IOSTANDARD LVCMOS33 [get_ports {uart_pin_u18}]

# Both are asynchronous outputs — false path
set_false_path -to [get_ports {uart_pin_p19}]
set_false_path -to [get_ports {uart_pin_u18}]
