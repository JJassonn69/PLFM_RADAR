#!/usr/bin/env python3
"""
Generate KiCad 10 schematic sub-sheets for the AERIS-10 MCU Subsystem.
Uses global labels to interconnect between sheets.
All pin assignments match the firmware main.h and existing schematic analysis.
"""
import uuid
import os

OUT_DIR = "/Users/ganeshpanth/PLFM_RADAR/4_Schematics and Boards Layout/4_6_Schematics/MCU_Subsystem"

def uid():
    return str(uuid.uuid4())

def power_symbol(name, ref, x, y, orientation=0, lib="power"):
    """Generate a power symbol (GND, +3V3, etc.)"""
    # Map common power names to their lib_id
    power_map = {
        "GND": ("GND", "GND"),
        "+3V3": ("+3V3", "+3V3"),
        "+5V": ("+5V", "+5V"),
        "+1V8": ("+1V8", "+1V8"),
        "+1V0": ("+1V0", "+1V0"),
        "VDDA": ("VDDA", "VDDA"),
        "VBUS": ("VBUS", "VBUS"),
    }
    
    sym_name, val = power_map.get(name, (name, name))
    u = uid()
    
    # Determine pin direction based on power type
    if name == "GND":
        pin_dir = "U"
        pin_y = 0  # pin at bottom for GND symbols
    else:
        pin_dir = "D"
        pin_y = 0
    
    return f"""
	(symbol
		(lib_id "{lib}:{sym_name}")
		(at {x} {y} {orientation})
		(unit 1)
		(exclude_from_sim no)
		(in_bom no)
		(on_board no)
		(dnp no)
		(uuid "{u}")
		(property "Reference" "#{ref}"
			(at {x} {y-2.54} 0)
			(effects
				(font (size 1.27 1.27))
				(hide yes)
			)
		)
		(property "Value" "{val}"
			(at {x} {y+2.54} 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(pin "{pin_dir}" (uuid "{uid()}"))
	)"""

def global_label(name, x, y, direction="input", shape="passive"):
    """Generate a global label for inter-sheet connections."""
    u = uid()
    return f"""
	(global_label "{name}"
		(shape {shape})
		(at {x} {y} {"0" if direction in ("input", "output", "passive") else "180"})
		(effects
			(font (size 1.27 1.27))
			(justify left)
		)
		(uuid "{u}")
		(property "Intersheets" ""
			(at 0 0 0)
			(effects
				(font (size 1.27 1.27))
				(hide yes)
			)
		)
	)"""

def net_label(name, x, y, orientation=0):
    u = uid()
    return f"""
	(label "{name}"
		(at {x} {y} {orientation})
		(effects
			(font (size 1.27 1.27))
			(justify left)
		)
		(uuid "{u}")
	)"""

def wire(x1, y1, x2, y2):
    u = uid()
    return f"""
	(wire
		(pts
			(xy {x1} {y1}) (xy {x2} {y2})
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "{u}")
	)"""

def no_connect(x, y):
    u = uid()
    return f"""
	(no_connect
		(at {x} {y})
		(uuid "{u}")
	)"""

def capacitor(ref, value, x, y, u_id=None, fp="Capacitor_SMD:C_0201_0603Metric"):
    """Place a decoupling capacitor."""
    u = u_id or uid()
    return f"""
	(symbol
		(lib_id "Device:C")
		(at {x} {y} 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "{u}")
		(property "Reference" "{ref}"
			(at {x+2.54} {y} 0)
			(effects
				(font (size 1.27 1.27))
				(justify left)
			)
		)
		(property "Value" "{value}"
			(at {x+2.54} {y+2.54} 0)
			(effects
				(font (size 1.27 1.27))
				(justify left)
			)
		)
		(property "Footprint" "{fp}"
			(at {x} {y} 0)
			(effects
				(font (size 1.27 1.27))
				(hide yes)
			)
		)
		(pin "1" (uuid "{uid()}"))
		(pin "2" (uuid "{uid()}"))
	)"""

def resistor(ref, value, x, y, orientation=0, fp="Resistor_SMD:R_0201_0603Metric"):
    u = uid()
    return f"""
	(symbol
		(lib_id "Device:R")
		(at {x} {y} {orientation})
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "{u}")
		(property "Reference" "{ref}"
			(at {x+2.54} {y} 0)
			(effects
				(font (size 1.27 1.27))
				(justify left)
			)
		)
		(property "Value" "{value}"
			(at {x+2.54} {y+2.54} 0)
			(effects
				(font (size 1.27 1.27))
				(justify left)
			)
		)
		(property "Footprint" "{fp}"
			(at {x} {y} 0)
			(effects
				(font (size 1.27 1.27))
				(hide yes)
			)
		)
		(pin "1" (uuid "{uid()}"))
		(pin "2" (uuid "{uid()}"))
	)"""

def led(ref, color, x, y):
    u = uid()
    return f"""
	(symbol
		(lib_id "Device:LED")
		(at {x} {y} 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "{u}")
		(property "Reference" "{ref}"
			(at {x+2.54} {y} 0)
			(effects
				(font (size 1.27 1.27))
				(justify left)
			)
		)
		(property "Value" "{color}"
			(at {x+2.54} {y+2.54} 0)
			(effects
				(font (size 1.27 1.27))
				(justify left)
			)
		)
		(pin "K" (uuid "{uid()}"))
		(pin "A" (uuid "{uid()}"))
	)"""

def text_note(txt, x, y, size=2.54):
    u = uid()
    return f"""
	(text "{txt}"
		(exclude_from_sim no)
		(at {x} {y} 0)
		(effects
			(font (size {size} {size}))
			(justify left)
		)
		(uuid "{u}")
	)"""

def sheet_header(title, date="2026-03-22", rev="0.1", comment1="", paper="A3"):
    return f"""(kicad_sch
	(version 20260306)
	(generator "eeschema")
	(generator_version "10.0")
	(uuid "{uid()}")
	(paper "{paper}")
	(title_block
		(title "{title}")
		(date "{date}")
		(rev "{rev}")
		(company "PLFM_RADAR Project")
		(comment 1 "{comment1}")
	)
	(lib_symbols)
"""

def sheet_footer(page_num):
    return f"""
	(sheet_instances
		(path "/"
			(page "{page_num}")
		)
	)
	(embedded_fonts no)
)
"""

def connector_header(ref, pins, x, y, name="Conn"):
    """Generate a generic pin header connector."""
    u = uid()
    pin_entries = ""
    for i in range(1, pins+1):
        pin_entries += f'\n\t\t(pin "{i}" (uuid "{uid()}"))'
    
    return f"""
	(symbol
		(lib_id "Connector_Generic:Conn_01x{pins:02d}")
		(at {x} {y} 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "{u}")
		(property "Reference" "{ref}"
			(at {x+5.08} {y} 0)
			(effects
				(font (size 1.27 1.27))
				(justify left)
			)
		)
		(property "Value" "{name}"
			(at {x+5.08} {y+2.54} 0)
			(effects
				(font (size 1.27 1.27))
				(justify left)
			)
		){pin_entries}
	)"""


# ============================================================================
# SHEET 1: MCU Core - STM32F746ZGT7 with power, clocks, reset, debug, decoupling
# ============================================================================
def generate_mcu_core():
    s = sheet_header(
        "AERIS-10 MCU Core",
        comment1="STM32F746ZGT7 - Power, Clocks, Reset, Debug, Decoupling"
    )
    
    # --- Title / notes ---
    s += text_note("STM32F746ZGT7 Core Circuit\\n"
                   "LQFP-144 package\\n"
                   "All VDD/VSS pins must have local 100nF decoupling\\n"
                   "VDDA requires additional 1uF + 100nF + ferrite bead",
                   25.4, 25.4, 2.0)
    
    # --- STM32F746ZGT7 symbol ---
    mcu_x, mcu_y = 152.4, 139.7
    mcu_uuid = uid()
    s += f"""
	(symbol
		(lib_id "MCU_ST_STM32F7:STM32F746ZGTx")
		(at {mcu_x} {mcu_y} 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "{mcu_uuid}")
		(property "Reference" "U1"
			(at {mcu_x} {mcu_y - 68} 0)
			(effects
				(font (size 1.524 1.524))
			)
		)
		(property "Value" "STM32F746ZGTx"
			(at {mcu_x} {mcu_y + 68} 0)
			(effects
				(font (size 1.524 1.524))
			)
		)
		(property "Footprint" "Package_QFP:LQFP-144_20x20mm_P0.5mm"
			(at {mcu_x} {mcu_y} 0)
			(effects
				(font (size 1.27 1.27))
				(hide yes)
			)
		)
		(property "Datasheet" "https://www.st.com/resource/en/datasheet/stm32f746zg.pdf"
			(at {mcu_x} {mcu_y} 0)
			(effects
				(font (size 1.27 1.27))
				(hide yes)
			)
		)
		(pin "1" (uuid "{uid()}"))
	)"""
    
    # --- VDD decoupling capacitors (100nF each, per ST AN4661) ---
    # STM32F746ZGT in LQFP-144 has multiple VDD pins: need 100nF per VDD pin
    cap_x_start = 45
    cap_y = 40
    vdd_caps = [
        ("C1", "100nF", "VDD pin 11"),
        ("C2", "100nF", "VDD pin 19"),
        ("C3", "100nF", "VDD pin 28"),
        ("C4", "100nF", "VDD pin 50"),
        ("C5", "100nF", "VDD pin 75"),
        ("C6", "100nF", "VDD pin 100"),
        ("C7", "100nF", "VDD pin 131"),
        ("C8", "4.7uF", "VDD bulk"),
    ]
    for i, (ref, val, note) in enumerate(vdd_caps):
        cx = cap_x_start + i * 12.7
        s += capacitor(ref, val, cx, cap_y)
        s += power_symbol("+3V3", f"pwr_vdd_{i}", cx, cap_y - 5.08)
        s += power_symbol("GND", f"pwr_gnd_vdd_{i}", cx, cap_y + 5.08)
        s += wire(cx, cap_y - 5.08 + 1.27, cx, cap_y - 2.54)  # +3V3 to cap pin 1
        s += wire(cx, cap_y + 2.54, cx, cap_y + 5.08 - 1.27)  # cap pin 2 to GND
    
    # --- VDDA decoupling (ferrite bead + 1uF + 100nF) ---
    s += text_note("VDDA Filtering (AN4661)", 45, 65, 1.5)
    
    # Ferrite bead L1: +3V3 -> VDDA_FILTERED
    s += f"""
	(symbol
		(lib_id "Device:L_Ferrite")
		(at 55 72 90)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "{uid()}")
		(property "Reference" "FB1"
			(at 55 69 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(property "Value" "600R@100MHz"
			(at 55 75 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(property "Footprint" "Inductor_SMD:L_0402_1005Metric"
			(at 55 72 0)
			(effects
				(font (size 1.27 1.27))
				(hide yes)
			)
		)
		(pin "1" (uuid "{uid()}"))
		(pin "2" (uuid "{uid()}"))
	)"""
    s += power_symbol("+3V3", "pwr_vdda_in", 45, 72)
    s += wire(45, 72 + 1.27, 52.46, 72)  # +3V3 to ferrite
    
    # VDDA caps
    s += capacitor("C9", "1uF", 65, 78, fp="Capacitor_SMD:C_0402_1005Metric")
    s += capacitor("C10", "100nF", 75, 78)
    s += net_label("VDDA", 60, 72)
    s += wire(57.54, 72, 75, 72)  # ferrite out to caps
    s += wire(65, 72, 65, 78 - 2.54)  # to C9 pin 1
    s += wire(75, 72, 75, 78 - 2.54)  # to C10 pin 1
    s += power_symbol("GND", "pwr_gnd_vdda1", 65, 78 + 5.08)
    s += power_symbol("GND", "pwr_gnd_vdda2", 75, 78 + 5.08)
    s += wire(65, 78 + 2.54, 65, 78 + 5.08 - 1.27)
    s += wire(75, 78 + 2.54, 75, 78 + 5.08 - 1.27)
    
    # --- VCAP decoupling (2x 2.2uF for internal regulator) ---
    s += text_note("VCAP (Internal 1.2V Regulator)", 100, 65, 1.5)
    s += capacitor("C11", "2.2uF", 110, 78, fp="Capacitor_SMD:C_0402_1005Metric")
    s += capacitor("C12", "2.2uF", 120, 78, fp="Capacitor_SMD:C_0402_1005Metric")
    s += net_label("VCAP1", 110, 72)
    s += net_label("VCAP2", 120, 72)
    s += wire(110, 72, 110, 78 - 2.54)
    s += wire(120, 72, 120, 78 - 2.54)
    s += power_symbol("GND", "pwr_gnd_vcap1", 110, 78 + 5.08)
    s += power_symbol("GND", "pwr_gnd_vcap2", 120, 78 + 5.08)
    s += wire(110, 78 + 2.54, 110, 78 + 5.08 - 1.27)
    s += wire(120, 78 + 2.54, 120, 78 + 5.08 - 1.27)
    
    # --- HSE Crystal (8 MHz) ---
    s += text_note("HSE 8MHz Crystal", 200, 25.4, 1.5)
    s += f"""
	(symbol
		(lib_id "Device:Crystal")
		(at 215 40 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "{uid()}")
		(property "Reference" "Y1"
			(at 215 33 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(property "Value" "8MHz"
			(at 215 36 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(property "Footprint" "Crystal:Crystal_SMD_3215-4Pin_3.2x1.5mm"
			(at 215 40 0)
			(effects
				(font (size 1.27 1.27))
				(hide yes)
			)
		)
		(pin "1" (uuid "{uid()}"))
		(pin "2" (uuid "{uid()}"))
	)"""
    # HSE load caps (per datasheet ~20pF, actual depends on crystal)
    s += capacitor("C13", "20pF", 207, 50, fp="Capacitor_SMD:C_0201_0603Metric")
    s += capacitor("C14", "20pF", 223, 50, fp="Capacitor_SMD:C_0201_0603Metric")
    s += global_label("STM32_OSC_P", 205, 40, "bidirectional", "passive")
    s += global_label("STM32_OSC_N", 225, 40, "bidirectional", "passive")
    s += wire(205, 40, 212.46, 40)  # OSC_P to crystal pin 1
    s += wire(217.54, 40, 225, 40)  # crystal pin 2 to OSC_N
    s += wire(207, 40, 207, 50 - 2.54)  # to C13
    s += wire(223, 40, 223, 50 - 2.54)  # to C14
    s += power_symbol("GND", "pwr_gnd_hse1", 207, 50 + 5.08)
    s += power_symbol("GND", "pwr_gnd_hse2", 223, 50 + 5.08)
    s += wire(207, 50 + 2.54, 207, 50 + 5.08 - 1.27)
    s += wire(223, 50 + 2.54, 223, 50 + 5.08 - 1.27)
    
    # --- LSE Crystal (32.768 kHz) ---
    s += text_note("LSE 32.768kHz Crystal", 200, 65, 1.5)
    s += f"""
	(symbol
		(lib_id "Device:Crystal")
		(at 215 78 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "{uid()}")
		(property "Reference" "Y2"
			(at 215 71 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(property "Value" "32.768kHz"
			(at 215 74 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(property "Footprint" "Crystal:Crystal_SMD_2012-2Pin_2.0x1.2mm"
			(at 215 78 0)
			(effects
				(font (size 1.27 1.27))
				(hide yes)
			)
		)
		(pin "1" (uuid "{uid()}"))
		(pin "2" (uuid "{uid()}"))
	)"""
    s += capacitor("C15", "6.8pF", 207, 88, fp="Capacitor_SMD:C_0201_0603Metric")
    s += capacitor("C16", "6.8pF", 223, 88, fp="Capacitor_SMD:C_0201_0603Metric")
    s += global_label("STM32_OSC_32_P", 205, 78, "bidirectional", "passive")
    s += global_label("STM32_OSC_32_N", 225, 78, "bidirectional", "passive")
    s += wire(205, 78, 212.46, 78)
    s += wire(217.54, 78, 225, 78)
    s += wire(207, 78, 207, 88 - 2.54)
    s += wire(223, 78, 223, 88 - 2.54)
    s += power_symbol("GND", "pwr_gnd_lse1", 207, 88 + 5.08)
    s += power_symbol("GND", "pwr_gnd_lse2", 223, 88 + 5.08)
    s += wire(207, 88 + 2.54, 207, 88 + 5.08 - 1.27)
    s += wire(223, 88 + 2.54, 223, 88 + 5.08 - 1.27)
    
    # --- NRST circuit (100nF cap + 10K pullup + pushbutton) ---
    s += text_note("Reset Circuit", 250, 25.4, 1.5)
    s += resistor("R1", "10k", 260, 35)
    s += capacitor("C17", "100nF", 260, 55)
    s += power_symbol("+3V3", "pwr_rst_pu", 260, 28)
    s += wire(260, 28 + 1.27, 260, 35 - 2.54)  # +3V3 to R1
    s += global_label("STM32_NRST", 250, 45, "bidirectional", "passive")
    s += wire(250, 45, 260, 45)
    s += wire(260, 35 + 2.54, 260, 45)  # R1 to NRST node
    s += wire(260, 45, 260, 55 - 2.54)  # NRST node to C17
    s += power_symbol("GND", "pwr_gnd_rst", 260, 55 + 5.08)
    s += wire(260, 55 + 2.54, 260, 55 + 5.08 - 1.27)
    
    # Reset pushbutton S1
    s += f"""
	(symbol
		(lib_id "Switch:SW_Push")
		(at 275 45 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "{uid()}")
		(property "Reference" "S1"
			(at 275 40 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(property "Value" "RESET"
			(at 275 48 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(pin "1" (uuid "{uid()}"))
		(pin "2" (uuid "{uid()}"))
	)"""
    s += wire(260, 45, 272.46, 45)  # NRST node to switch
    s += power_symbol("GND", "pwr_gnd_sw", 280, 45)
    s += wire(277.54, 45, 280, 45)
    
    # --- SWD Debug Header (JP2) ---
    s += text_note("SWD Debug Header", 250, 65, 1.5)
    s += connector_header("JP2", 5, 265, 80, "SWD_Header")
    # Pin 1: SWCLK, Pin 2: SWDIO, Pin 3: SWO, Pin 4: NRST, Pin 5: GND
    swd_labels = [
        ("STM32_SWCLK", 75),
        ("STM32_SWDIO", 77.54),
        ("STM32_SWO", 80.08),
        ("STM32_NRST", 82.62),
    ]
    for name, ly in swd_labels:
        s += global_label(name, 255, ly, "bidirectional", "passive")
        s += wire(255, ly, 265, ly)
    s += power_symbol("GND", "pwr_gnd_swd", 255, 85.16)
    s += wire(255, 85.16, 265, 85.16)
    
    # BOOT0 pin - tied to GND via 10K for normal boot
    s += text_note("BOOT0 = GND (boot from flash)", 250, 95, 1.5)
    s += resistor("R2", "10k", 265, 105)
    s += global_label("BOOT0", 255, 100, "output", "passive")
    s += wire(255, 100, 265, 100)
    s += wire(265, 100, 265, 105 - 2.54)
    s += power_symbol("GND", "pwr_gnd_boot", 265, 105 + 5.08)
    s += wire(265, 105 + 2.54, 265, 105 + 5.08 - 1.27)
    
    # --- Global labels for MCU power pins ---
    # These connect to the MCU symbol's power pins
    power_labels = [
        "+3V3", "GND", "VDDA", "VCAP1", "VCAP2",
        "STM32_OSC_P", "STM32_OSC_N", "STM32_OSC_32_P", "STM32_OSC_32_N",
        "STM32_NRST", "STM32_SWCLK", "STM32_SWDIO", "STM32_SWO", "BOOT0",
    ]
    
    # Global labels for all MCU functional pins (connecting to other sheets)
    func_y_start = 120
    func_x = 25.4
    
    s += text_note("MCU Pin Global Labels\\n(Connect to other sheets via global labels)", 25.4, func_y_start - 5, 1.5)
    
    # SPI1 pins (FPGA SPI bridge) - PA5/PA6/PA7
    spi1_labels = ["STM32_SCLK1", "STM32_MOSI1", "STM32_MISO1"]
    for i, name in enumerate(spi1_labels):
        s += global_label(name, func_x, func_y_start + i * 5.08, "bidirectional", "bidirectional")
    
    # SPI4 pins (AD9523/ADF4382) - PE2/PE5/PE6
    func_y = func_y_start + 20
    spi4_labels = ["STM32_SCLK4", "STM32_MOSI4", "STM32_MISO4"]
    for i, name in enumerate(spi4_labels):
        s += global_label(name, func_x, func_y + i * 5.08, "bidirectional", "bidirectional")
    
    # I2C pins
    func_y = func_y_start + 40
    i2c_labels = ["STM32_SCL1", "STM32_SDA1", "STM32_SCL2", "STM32_SDA2", "STM32_SCL3", "STM32_SDA3"]
    for i, name in enumerate(i2c_labels):
        s += global_label(name, func_x, func_y + i * 5.08, "bidirectional", "bidirectional")
    
    # UART pins
    func_y = func_y_start + 75
    uart_labels = ["STM32_TX3", "STM32_RX3", "STM32_TX5", "STM32_RX5"]
    for i, name in enumerate(uart_labels):
        s += global_label(name, func_x, func_y + i * 5.08, "bidirectional", "bidirectional")
    
    # USB FS pins
    func_y = func_y_start + 100
    usb_labels = ["STM32_USB_FS_D_P", "STM32_USB_FS_D_N", "STM32_USB_FS_ID"]
    for i, name in enumerate(usb_labels):
        s += global_label(name, func_x, func_y + i * 5.08, "bidirectional", "bidirectional")
    
    s += sheet_footer("2")
    return s

# ============================================================================
# SHEET 2: Communication Interfaces - SPI, I2C, UART, USB-FS
# ============================================================================
def generate_comm_interfaces():
    s = sheet_header(
        "AERIS-10 Communication Interfaces",
        comment1="SPI, I2C, UART, USB-FS Connectors and Protection"
    )
    
    # ---- SPI1 Section (to FPGA for ADAR1000 bridge) ----
    s += text_note("SPI1 - FPGA/ADAR1000 Bridge\\n"
                   "MCU SPI1 (PA5/PA6/PA7) -> FPGA -> ADAR1000\\n"
                   "3.3V domain on MCU side, FPGA translates to 1.8V", 25.4, 25.4, 1.5)
    
    spi1_y = 40
    spi1_labels = [
        ("STM32_SCLK1", "SPI1_SCK (PA5)"),
        ("STM32_MOSI1", "SPI1_MOSI (PA7)"),
        ("STM32_MISO1", "SPI1_MISO (PA6)"),
        ("ADAR_1_CS_3V3", "ADAR1 CS (PA0)"),
        ("ADAR_2_CS_3V3", "ADAR2 CS (PA1)"),
        ("ADAR_3_CS_3V3", "ADAR3 CS (PA2)"),
        ("ADAR_4_CS_3V3", "ADAR4 CS (PA3)"),
    ]
    for i, (name, note) in enumerate(spi1_labels):
        y = spi1_y + i * 5.08
        s += global_label(name, 25.4, y, "bidirectional", "bidirectional")
        s += text_note(note, 60, y, 1.0)
    
    # ---- SPI4 Section (AD9523 + ADF4382 clock/LO synth) ----
    s += text_note("SPI4 - Clock/LO Synthesizer\\n"
                   "MCU SPI4 (PE2/PE5/PE6) -> AD9523 + ADF4382\\n"
                   "Direct connection, 3.3V domain", 25.4, 85, 1.5)
    
    spi4_y = 100
    spi4_labels = [
        ("STM32_SCLK4", "SPI4_SCK (PE2)"),
        ("STM32_MOSI4", "SPI4_MOSI (PE6)"),
        ("STM32_MISO4", "SPI4_MISO (PE5)"),
        ("AD9523_CS", "AD9523 CS (PF7)"),
        ("ADF4382_RX_CS", "ADF4382 RX CS (PG10)"),
        ("ADF4382_TX_CS", "ADF4382 TX CS (PG14)"),
    ]
    for i, (name, note) in enumerate(spi4_labels):
        y = spi4_y + i * 5.08
        s += global_label(name, 25.4, y, "bidirectional", "bidirectional")
        s += text_note(note, 60, y, 1.0)
    
    # AD9523 control pins
    s += text_note("AD9523 Control GPIO", 25.4, 140, 1.5)
    ad9523_y = 150
    ad9523_labels = [
        ("AD9523_PD", "Power Down (PF3)"),
        ("AD9523_REF_SEL", "Ref Select (PF4)"),
        ("AD9523_SYNC", "Sync (PF5)"),
        ("AD9523_RESET", "Reset (PF6)"),
        ("AD9523_STATUS0", "Status 0 (PF8)"),
        ("AD9523_STATUS1", "Status 1 (PF9)"),
        ("AD9523_EEPROM_SEL", "EEPROM Select (PF10)"),
    ]
    for i, (name, note) in enumerate(ad9523_labels):
        y = ad9523_y + i * 5.08
        s += global_label(name, 25.4, y, "bidirectional", "bidirectional")
        s += text_note(note, 60, y, 1.0)
    
    # ADF4382 control pins
    s += text_note("ADF4382 Control GPIO (RX + TX LO)", 150, 25.4, 1.5)
    adf_y = 40
    adf_labels = [
        ("ADF4382_RX_LKDET", "RX Lock Detect (PG6)"),
        ("ADF4382_RX_DELADJ", "RX Delay Adjust PWM (PG7)"),
        ("ADF4382_RX_DELSTR", "RX Delay Strobe (PG8)"),
        ("ADF4382_RX_CE", "RX Chip Enable (PG9)"),
        ("ADF4382_TX_LKDET", "TX Lock Detect (PG11)"),
        ("ADF4382_TX_DELSTR", "TX Delay Strobe (PG12)"),
        ("ADF4382_TX_DELADJ", "TX Delay Adjust PWM (PG13)"),
        ("ADF4382_TX_CE", "TX Chip Enable (PG15)"),
    ]
    for i, (name, note) in enumerate(adf_labels):
        y = adf_y + i * 5.08
        s += global_label(name, 150, y, "bidirectional", "bidirectional")
        s += text_note(note, 190, y, 1.0)
    
    # ---- I2C1 (PA/DAC bias) ----
    s += text_note("I2C1 - PA Bias DAC/ADC\\n"
                   "DAC5578 (gate voltage) + ADS7830 (current sense)\\n"
                   "PB6=SCL1, PB7=SDA1, 3.3V domain, 4.7K pullups", 150, 90, 1.5)
    
    i2c1_y = 110
    s += global_label("STM32_SCL1", 150, i2c1_y, "bidirectional", "bidirectional")
    s += global_label("STM32_SDA1", 150, i2c1_y + 5.08, "bidirectional", "bidirectional")
    # I2C pullups
    s += resistor("R3", "4.7k", 185, i2c1_y - 5)
    s += resistor("R4", "4.7k", 195, i2c1_y - 5)
    s += power_symbol("+3V3", "pwr_i2c1_pu", 190, i2c1_y - 15)
    s += wire(185, i2c1_y - 5 - 2.54, 185, i2c1_y - 12)
    s += wire(195, i2c1_y - 5 - 2.54, 195, i2c1_y - 12)
    s += wire(185, i2c1_y - 12, 190, i2c1_y - 12)
    s += wire(190, i2c1_y - 12, 195, i2c1_y - 12)
    s += wire(190, i2c1_y - 15 + 1.27, 190, i2c1_y - 12)
    s += wire(185, i2c1_y - 5 + 2.54, 185, i2c1_y)
    s += wire(185, i2c1_y, 175, i2c1_y)  # connect pullup to SCL
    s += wire(195, i2c1_y - 5 + 2.54, 195, i2c1_y + 5.08)
    s += wire(195, i2c1_y + 5.08, 175, i2c1_y + 5.08)  # connect pullup to SDA
    
    # DAC control pins
    dac_y = i2c1_y + 15
    dac_labels = [
        ("DAC_1_VG_CLR", "DAC1 Clear (PB4)"),
        ("DAC_1_VG_LDAC", "DAC1 LDAC (PB5)"),
        ("DAC_2_VG_CLR", "DAC2 Clear (PB8)"),
        ("DAC_2_VG_LDAC", "DAC2 LDAC (PB9)"),
    ]
    for i, (name, note) in enumerate(dac_labels):
        y = dac_y + i * 5.08
        s += global_label(name, 150, y, "output", "output")
        s += text_note(note, 190, y, 1.0)
    
    # ---- I2C2 (secondary) ----
    s += text_note("I2C2 - Secondary I2C Bus\\n"
                   "PF0=SDA2, PF1=SCL2, 4.7K pullups", 150, 155, 1.5)
    i2c2_y = 170
    s += global_label("STM32_SCL2", 150, i2c2_y, "bidirectional", "bidirectional")
    s += global_label("STM32_SDA2", 150, i2c2_y + 5.08, "bidirectional", "bidirectional")
    s += resistor("R5", "4.7k", 185, i2c2_y - 5)
    s += resistor("R6", "4.7k", 195, i2c2_y - 5)
    s += power_symbol("+3V3", "pwr_i2c2_pu", 190, i2c2_y - 15)
    s += wire(185, i2c2_y - 5 - 2.54, 185, i2c2_y - 12)
    s += wire(195, i2c2_y - 5 - 2.54, 195, i2c2_y - 12)
    s += wire(185, i2c2_y - 12, 195, i2c2_y - 12)
    s += wire(190, i2c2_y - 15 + 1.27, 190, i2c2_y - 12)
    
    # ---- I2C3 (Sensors: IMU + Barometer) ----
    s += text_note("I2C3 - Sensor Bus\\n"
                   "GY-85 IMU + BMP180 Barometer\\n"
                   "PA8=SCL3, PC9=SDA3, 4.7K pullups", 150, 195, 1.5)
    i2c3_y = 210
    s += global_label("STM32_SCL3", 150, i2c3_y, "bidirectional", "bidirectional")
    s += global_label("STM32_SDA3", 150, i2c3_y + 5.08, "bidirectional", "bidirectional")
    s += resistor("R7", "4.7k", 185, i2c3_y - 5)
    s += resistor("R8", "4.7k", 195, i2c3_y - 5)
    s += power_symbol("+3V3", "pwr_i2c3_pu", 190, i2c3_y - 15)
    s += wire(185, i2c3_y - 5 - 2.54, 185, i2c3_y - 12)
    s += wire(195, i2c3_y - 5 - 2.54, 195, i2c3_y - 12)
    s += wire(185, i2c3_y - 12, 195, i2c3_y - 12)
    s += wire(190, i2c3_y - 15 + 1.27, 190, i2c3_y - 12)
    
    # Sensor interrupt pins
    sens_y = i2c3_y + 15
    sens_labels = [
        ("MAG_DRDY", "Magnetometer Data Ready (PC6)"),
        ("ACC_INT", "Accelerometer Interrupt (PC7)"),
        ("GYR_INT", "Gyroscope Interrupt (PC8)"),
    ]
    for i, (name, note) in enumerate(sens_labels):
        y = sens_y + i * 5.08
        s += global_label(name, 150, y, "input", "input")
        s += text_note(note, 190, y, 1.0)
    
    # ---- USART3 (Debug/Diagnostics at 115200) ----
    s += text_note("USART3 - Debug Console (115200 baud)\\n"
                   "PB10=TX3, PB11=RX3", 25.4, 200, 1.5)
    uart3_y = 215
    s += global_label("STM32_TX3", 25.4, uart3_y, "output", "output")
    s += global_label("STM32_RX3", 25.4, uart3_y + 5.08, "input", "input")
    s += connector_header("JP17", 3, 70, uart3_y, "UART3_Debug")
    s += wire(50, uart3_y, 70, uart3_y)
    s += wire(50, uart3_y + 5.08, 70, uart3_y + 2.54)
    s += power_symbol("GND", "pwr_gnd_uart3", 70, uart3_y + 5.08)
    s += wire(70, uart3_y + 5.08, 70, uart3_y + 5.08)
    
    # ---- UART5 (GPS at 9600) ----
    s += text_note("UART5 - GPS Module (9600 baud)\\n"
                   "PC12=TX5, PD2=RX5", 25.4, 235, 1.5)
    uart5_y = 250
    s += global_label("STM32_TX5", 25.4, uart5_y, "output", "output")
    s += global_label("STM32_RX5", 25.4, uart5_y + 5.08, "input", "input")
    s += connector_header("JP18", 4, 70, uart5_y, "GPS_Header")
    
    # ---- USB FS (MCU CDC for host control) ----
    s += text_note("USB Full-Speed (CDC)\\n"
                   "PA11=USB_DM, PA12=USB_DP, PA10=USB_ID\\n"
                   "Mini-USB connector X53", 25.4, 270, 1.5)
    usb_y = 290
    s += global_label("STM32_USB_FS_D_N", 25.4, usb_y, "bidirectional", "bidirectional")
    s += global_label("STM32_USB_FS_D_P", 25.4, usb_y + 5.08, "bidirectional", "bidirectional")
    s += global_label("STM32_USB_FS_ID", 25.4, usb_y + 10.16, "input", "input")
    
    # USB connector symbol (generic)
    s += f"""
	(symbol
		(lib_id "Connector:USB_B_Mini")
		(at 80 {usb_y + 5} 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "{uid()}")
		(property "Reference" "X53"
			(at 80 {usb_y - 5} 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(property "Value" "USB_B_Mini"
			(at 80 {usb_y + 15} 0)
			(effects
				(font (size 1.27 1.27))
			)
		)
		(pin "1" (uuid "{uid()}"))
		(pin "2" (uuid "{uid()}"))
		(pin "3" (uuid "{uid()}"))
		(pin "4" (uuid "{uid()}"))
		(pin "5" (uuid "{uid()}"))
	)"""
    
    # ESD protection on USB lines
    s += text_note("Add USB ESD protection (USBLC6-2SC6)\\non D+/D- lines", 100, usb_y, 1.0)
    
    s += sheet_footer("3")
    return s

# ============================================================================
# SHEET 3: FPGA Interface - SPI bridge, handshake, level shifting
# ============================================================================
def generate_fpga_interface():
    s = sheet_header(
        "AERIS-10 FPGA-MCU Interface",
        comment1="SPI Bridge, Handshake Signals, Level Shifting 3.3V <-> 1.8V"
    )
    
    s += text_note("FPGA-MCU Interface\\n\\n"
                   "The STM32 operates at 3.3V I/O.\\n"
                   "The Artix-7 FPGA has mixed I/O banks:\\n"
                   "  - Bank 34: 1.8V (ADAR1000 SPI, ADC/DAC data)\\n"
                   "  - Bank 14/16: 3.3V (MCU GPIO, FT601)\\n\\n"
                   "SPI1 from MCU enters FPGA on 3.3V bank pins,\\n"
                   "FPGA internally routes to 1.8V bank for ADAR1000.\\n"
                   "No external level shifter needed for SPI bridge -\\n"
                   "FPGA handles voltage translation between banks.", 25.4, 25.4, 1.5)
    
    # ---- SPI Bridge (MCU -> FPGA -> ADAR1000) ----
    s += text_note("SPI1 Bridge Path: MCU (3.3V) -> FPGA -> ADAR1000 (1.8V)", 25.4, 85, 2.0)
    
    bridge_y = 100
    mcu_spi_labels = [
        ("STM32_SCLK1", "MCU SPI1 CLK (PA5)"),
        ("STM32_MOSI1", "MCU SPI1 MOSI (PA7)"),
        ("STM32_MISO1", "MCU SPI1 MISO (PA6)"),
        ("ADAR_1_CS_3V3", "ADAR1 CS via FPGA (PA0)"),
        ("ADAR_2_CS_3V3", "ADAR2 CS via FPGA (PA1)"),
        ("ADAR_3_CS_3V3", "ADAR3 CS via FPGA (PA2)"),
        ("ADAR_4_CS_3V3", "ADAR4 CS via FPGA (PA3)"),
    ]
    
    for i, (name, note) in enumerate(mcu_spi_labels):
        y = bridge_y + i * 7.62
        s += global_label(name, 25.4, y, "bidirectional", "bidirectional")
        s += text_note(f"-> FPGA pin (3.3V bank) -> ", 60, y, 1.0)
    
    # FPGA-side 1.8V labels (what comes out the other side)
    fpga_spi_labels = [
        ("STM32_SCLK_1V8", "FPGA -> ADAR SPI CLK"),
        ("STM32_MOSI_1V8", "FPGA -> ADAR SPI MOSI"),
        ("STM32_MISO_1V8", "FPGA <- ADAR SPI MISO"),
        ("ADAR_1_CS_1V8", "FPGA -> ADAR1 CS"),
        ("ADAR_2_CS_1V8", "FPGA -> ADAR2 CS"),
        ("ADAR_3_CS_1V8", "FPGA -> ADAR3 CS"),
        ("ADAR_4_CS_1V8", "FPGA -> ADAR4 CS"),
    ]
    for i, (name, note) in enumerate(fpga_spi_labels):
        y = bridge_y + i * 7.62
        s += global_label(name, 160, y, "bidirectional", "bidirectional")
        s += text_note(note, 200, y, 1.0)
    
    # ---- FPGA Handshake / Control Signals (MCU -> FPGA, 3.3V direct) ----
    s += text_note("MCU -> FPGA Handshake Signals\\n"
                   "Direct 3.3V GPIO connections (FPGA 3.3V bank)\\n"
                   "Active-high pulses synchronized inside FPGA via CDC", 25.4, 165, 1.5)
    
    hs_y = 185
    hs_labels = [
        ("DIG_0", "New Chirp Trigger (PD8)"),
        ("DIG_1", "New Elevation (PD9)"),
        ("DIG_2", "New Azimuth (PD10)"),
        ("DIG_3", "Mixers Enable (PD11)"),
        ("DIG_4", "FPGA Reset (PD12)"),
        ("DIG_5", "Reserved (PD13)"),
        ("DIG_6", "Reserved (PD14)"),
        ("DIG_7", "Reserved (PD15)"),
    ]
    for i, (name, note) in enumerate(hs_labels):
        y = hs_y + i * 5.08
        s += global_label(name, 25.4, y, "output", "output")
        s += text_note(note, 60, y, 1.0)
    
    # Series resistors on handshake lines (33 ohm for signal integrity)
    s += text_note("Note: 33R series resistors recommended\\n"
                   "on all MCU->FPGA digital lines\\n"
                   "for signal integrity and ESD protection", 25.4, hs_y + 45, 1.0)
    
    # ---- FPGA JTAG / Configuration ----
    s += text_note("FPGA JTAG (directly from debug header or\\n"
                   "dedicated JTAG connector, not through MCU)", 150, 165, 1.5)
    
    jtag_y = 185
    jtag_labels = [
        ("FPGA_TCK", "JTAG Clock"),
        ("FPGA_TMS", "JTAG Mode Select"),
        ("FPGA_TDI", "JTAG Data In"),
        ("FPGA_TDO", "JTAG Data Out"),
    ]
    for i, (name, note) in enumerate(jtag_labels):
        y = jtag_y + i * 5.08
        s += global_label(name, 150, y, "bidirectional", "bidirectional")
        s += text_note(note, 190, y, 1.0)
    
    # JTAG header
    s += connector_header("JP_JTAG", 6, 220, jtag_y, "FPGA_JTAG")
    
    # ---- FPGA Clock Inputs ----
    s += text_note("FPGA Clock Inputs\\n"
                   "(from AD9523 clock distribution)", 150, 215, 1.5)
    
    clk_y = 230
    clk_labels = [
        ("FPGA_SYS_CLOCK", "System Clock (from AD9523)"),
        ("FPGA_DAC_CLOCK", "DAC Clock (from AD9523)"),
        ("FPGA_ADC_CLOCK_P", "ADC Clock P (LVDS from AD9523)"),
        ("FPGA_ADC_CLOCK_N", "ADC Clock N (LVDS from AD9523)"),
    ]
    for i, (name, note) in enumerate(clk_labels):
        y = clk_y + i * 5.08
        s += global_label(name, 150, y, "input", "input")
        s += text_note(note, 200, y, 1.0)
    
    # SMA connectors for clock test points
    s += text_note("SMA test points for clock verification", 150, 255, 1.0)
    
    s += sheet_footer("4")
    return s

# ============================================================================
# SHEET 4: Sensors & Actuators
# ============================================================================
def generate_sensors_actuators():
    s = sheet_header(
        "AERIS-10 Sensors & Actuators",
        comment1="IMU, Barometer, GPS, Stepper Motor, PA Bias DAC/ADC"
    )
    
    # ---- GY-85 IMU (I2C3) ----
    s += text_note("GY-85 9-DOF IMU Module\\n"
                   "ADXL345 Accelerometer + ITG3200 Gyroscope + HMC5883L Magnetometer\\n"
                   "Connected via I2C3 (PA8=SCL, PC9=SDA)\\n"
                   "3.3V supply, interrupt lines to MCU", 25.4, 25.4, 1.5)
    
    imu_y = 50
    # IMU connector (8-pin: VCC, GND, SCL, SDA, DRDY, INT_ACC, INT_GYR, NC)
    s += connector_header("J_IMU", 8, 50, imu_y, "GY-85_IMU")
    
    imu_pin_labels = [
        ("+3V3", imu_y),
        ("GND", imu_y + 2.54),
        ("STM32_SCL3", imu_y + 5.08),
        ("STM32_SDA3", imu_y + 7.62),
        ("MAG_DRDY", imu_y + 10.16),
        ("ACC_INT", imu_y + 12.7),
        ("GYR_INT", imu_y + 15.24),
    ]
    for name, y in imu_pin_labels:
        if name in ("+3V3", "GND"):
            s += power_symbol(name if name != "+3V3" else "+3V3", f"pwr_imu_{name}", 40, y)
            s += wire(40, y, 50, y)
        else:
            s += global_label(name, 30, y, "bidirectional", "bidirectional")
            s += wire(30, y, 50, y)
    
    # ---- BMP180 Barometer (I2C3) ----
    s += text_note("BMP180 Barometric Pressure Sensor\\n"
                   "Connected via I2C3 (shared with IMU)\\n"
                   "3.3V supply", 25.4, 95, 1.5)
    
    baro_y = 115
    s += connector_header("J_BARO", 4, 50, baro_y, "BMP180")
    baro_labels = [
        ("+3V3", baro_y),
        ("GND", baro_y + 2.54),
        ("STM32_SCL3", baro_y + 5.08),
        ("STM32_SDA3", baro_y + 7.62),
    ]
    for name, y in baro_labels:
        if name in ("+3V3", "GND"):
            s += power_symbol(name if name != "+3V3" else "+3V3", f"pwr_baro_{name}", 40, y)
            s += wire(40, y, 50, y)
        else:
            s += global_label(name, 30, y, "bidirectional", "bidirectional")
            s += wire(30, y, 50, y)
    
    # ---- GPS Module (UART5) ----
    s += text_note("GPS Module (NMEA via UART5)\\n"
                   "PC12=TX5, PD2=RX5\\n"
                   "9600 baud, 3.3V logic", 25.4, 145, 1.5)
    
    gps_y = 165
    s += connector_header("J_GPS", 4, 50, gps_y, "GPS_Module")
    gps_labels = [
        ("+3V3", gps_y),
        ("GND", gps_y + 2.54),
        ("STM32_TX5", gps_y + 5.08),
        ("STM32_RX5", gps_y + 7.62),
    ]
    for name, y in gps_labels:
        if name in ("+3V3", "GND"):
            s += power_symbol(name if name != "+3V3" else "+3V3", f"pwr_gps_{name}", 40, y)
            s += wire(40, y, 50, y)
        else:
            s += global_label(name, 30, y, "bidirectional", "bidirectional")
            s += wire(30, y, 50, y)
    
    # ---- Stepper Motor Driver (Azimuth/Elevation) ----
    s += text_note("Stepper Motor Control\\n"
                   "PD4=CW/CCW direction, PD5=Step Clock\\n"
                   "External stepper driver module", 150, 25.4, 1.5)
    
    step_y = 50
    s += global_label("STEPPER_CW_P", 150, step_y, "output", "output")
    s += global_label("STEPPER_CLK_P", 150, step_y + 5.08, "output", "output")
    s += text_note("Direction (PD4)", 190, step_y, 1.0)
    s += text_note("Step Clock (PD5)", 190, step_y + 5.08, 1.0)
    s += connector_header("J_STEP", 4, 220, step_y, "Stepper_Driver")
    
    # ---- PA Bias DAC/ADC (I2C1) ----
    s += text_note("PA Gate Bias Control\\n"
                   "2x DAC5578 (8-ch 8-bit I2C DAC) for gate voltages\\n"
                   "2x ADS7830 (8-ch 8-bit I2C ADC) for current sense\\n"
                   "Connected via I2C1 (PB6=SCL, PB7=SDA)", 150, 80, 1.5)
    
    dac_y = 110
    # DAC5578 #1
    s += text_note("DAC5578 #1 (U_DAC1)\\nAddr: 0x48\\nGate voltage PA1-PA4", 150, dac_y, 1.2)
    s += global_label("STM32_SCL1", 150, dac_y + 15, "bidirectional", "bidirectional")
    s += global_label("STM32_SDA1", 150, dac_y + 20, "bidirectional", "bidirectional")
    s += global_label("DAC_1_VG_CLR", 150, dac_y + 25, "output", "output")
    s += global_label("DAC_1_VG_LDAC", 150, dac_y + 30, "output", "output")
    
    # DAC5578 #2
    s += text_note("DAC5578 #2 (U_DAC2)\\nAddr: 0x4A\\nGate voltage PA5-PA8", 150, dac_y + 40, 1.2)
    s += global_label("DAC_2_VG_CLR", 150, dac_y + 55, "output", "output")
    s += global_label("DAC_2_VG_LDAC", 150, dac_y + 60, "output", "output")
    
    # ADS7830 ADCs
    s += text_note("ADS7830 #1 (U_ADC1)\\nAddr: 0x48\\nPA current sense 1-4", 250, dac_y, 1.2)
    s += text_note("ADS7830 #2 (U_ADC2)\\nAddr: 0x4A\\nPA current sense 5-8", 250, dac_y + 40, 1.2)
    
    # ---- RF PA Enable / Control ----
    s += text_note("RF PA Control\\n"
                   "PD6=EN/DIS_RFPA_VDD (PA supply enable)\\n"
                   "PD7=EN/DIS_COOLING (cooling fan)", 150, 195, 1.5)
    
    rfpa_y = 215
    s += global_label("EN_DIS_RFPA_VDD", 150, rfpa_y, "output", "output")
    s += global_label("EN_DIS_COOLING", 150, rfpa_y + 5.08, "output", "output")
    s += text_note("PA VDD Enable (PD6) - active high, drives FET gate", 190, rfpa_y, 1.0)
    s += text_note("Cooling Fan Enable (PD7) - active high", 190, rfpa_y + 5.08, 1.0)
    
    s += sheet_footer("5")
    return s

# ============================================================================
# SHEET 5: Power Sequencing & LED Indicators
# ============================================================================
def generate_power_sequencing():
    s = sheet_header(
        "AERIS-10 Power Sequencing & Indicators",
        comment1="MCU-controlled power enables, LED status indicators, watchdog"
    )
    
    s += text_note("Power Enable Sequencing Network\\n\\n"
                   "The STM32 controls all subsystem power rails via GPIO.\\n"
                   "Power-on sequence (from main.cpp):\\n"
                   "  1. +1V0_FPGA (PE7) - FPGA core\\n"
                   "  2. +1V8_FPGA (PE8) - FPGA aux/bank 34\\n"
                   "  3. +3V3_FPGA (PE9) - FPGA I/O banks\\n"
                   "  4. +1V8_CLOCK (PG4) - AD9523 clock synth\\n"
                   "  5. +3V3_CLOCK (PG5) - ADF4382 LO synth\\n"
                   "  6. +5V0_ADAR (PE10) - ADAR1000 beamformers\\n"
                   "  7. +3V3_ADAR12/34 (PE11/PE12) - ADAR regulators\\n"
                   "  8. +3V3_ADTR (PE13) - ADC/DAC transmitter\\n"
                   "  9. +3V3_SW (PE14) - RF switches\\n"
                   "  10. +5V0_PA1/2/3 (PG0/1/2) - PA stages\\n"
                   "  11. +5V5_PA (PG3) - PA final stage\\n\\n"
                   "Each enable drives the EN pin of a regulator/LDO.\\n"
                   "100K pulldown on each EN pin ensures safe default off.", 25.4, 25.4, 1.5)
    
    # ---- FPGA Power Enables ----
    s += text_note("FPGA Power Enables", 25.4, 120, 2.0)
    fpga_pwr_y = 135
    fpga_pwr = [
        ("EN_P_1V0_FPGA", "+1V0 FPGA Core (PE7)", "R10"),
        ("EN_P_1V8_FPGA", "+1V8 FPGA Aux (PE8)", "R11"),
        ("EN_P_3V3_FPGA", "+3V3 FPGA I/O (PE9)", "R12"),
    ]
    for i, (name, note, rref) in enumerate(fpga_pwr):
        y = fpga_pwr_y + i * 10.16
        s += global_label(name, 25.4, y, "output", "output")
        s += text_note(note, 60, y, 1.0)
        # Pulldown resistor
        s += resistor(rref, "100k", 45, y + 5)
        s += wire(45, y, 45, y + 5 - 2.54)
        s += power_symbol("GND", f"pwr_gnd_{name}", 45, y + 5 + 5.08)
        s += wire(45, y + 5 + 2.54, 45, y + 5 + 5.08 - 1.27)
    
    # ---- Clock Power Enables ----
    s += text_note("Clock/LO Power Enables", 25.4, 170, 2.0)
    clk_pwr_y = 185
    clk_pwr = [
        ("EN_P_1V8_CLOCK", "+1V8 Clock Synth (PG4)", "R13"),
        ("EN_P_3V3_CLOCK", "+3V3 LO Synth (PG5)", "R14"),
    ]
    for i, (name, note, rref) in enumerate(clk_pwr):
        y = clk_pwr_y + i * 10.16
        s += global_label(name, 25.4, y, "output", "output")
        s += text_note(note, 60, y, 1.0)
        s += resistor(rref, "100k", 45, y + 5)
        s += wire(45, y, 45, y + 5 - 2.54)
        s += power_symbol("GND", f"pwr_gnd_{name}", 45, y + 5 + 5.08)
        s += wire(45, y + 5 + 2.54, 45, y + 5 + 5.08 - 1.27)
    
    # ---- Beamformer / RF Power Enables ----
    s += text_note("Beamformer & RF Power Enables", 150, 120, 2.0)
    bf_pwr_y = 135
    bf_pwr = [
        ("EN_P_5V0_ADAR", "+5V0 ADAR1000 (PE10)", "R15"),
        ("EN_P_3V3_ADAR12", "+3V3 ADAR 1&2 (PE11)", "R16"),
        ("EN_P_3V3_ADAR34", "+3V3 ADAR 3&4 (PE12)", "R17"),
        ("EN_P_3V3_ADTR", "+3V3 ADC/DAC (PE13)", "R18"),
        ("EN_P_3V3_SW", "+3V3 RF Switches (PE14)", "R19"),
        ("EN_P_3V3_VDD_SW", "+3V3 VDD Switches (PE15)", "R20"),
    ]
    for i, (name, note, rref) in enumerate(bf_pwr):
        y = bf_pwr_y + i * 10.16
        s += global_label(name, 150, y, "output", "output")
        s += text_note(note, 190, y, 1.0)
        s += resistor(rref, "100k", 170, y + 5)
        s += wire(170, y, 170, y + 5 - 2.54)
        s += power_symbol("GND", f"pwr_gnd_{name}", 170, y + 5 + 5.08)
        s += wire(170, y + 5 + 2.54, 170, y + 5 + 5.08 - 1.27)
    
    # ---- PA Power Enables ----
    s += text_note("Power Amplifier Enables", 150, 210, 2.0)
    pa_pwr_y = 225
    pa_pwr = [
        ("EN_P_5V0_PA1", "+5V0 PA Stage 1 (PG0)", "R21"),
        ("EN_P_5V0_PA2", "+5V0 PA Stage 2 (PG1)", "R22"),
        ("EN_P_5V0_PA3", "+5V0 PA Stage 3 (PG2)", "R23"),
        ("EN_P_5V5_PA", "+5V5 PA Final (PG3)", "R24"),
    ]
    for i, (name, note, rref) in enumerate(pa_pwr):
        y = pa_pwr_y + i * 10.16
        s += global_label(name, 150, y, "output", "output")
        s += text_note(note, 190, y, 1.0)
        s += resistor(rref, "100k", 170, y + 5)
        s += wire(170, y, 170, y + 5 - 2.54)
        s += power_symbol("GND", f"pwr_gnd_{name}", 170, y + 5 + 5.08)
        s += wire(170, y + 5 + 2.54, 170, y + 5 + 5.08 - 1.27)
    
    # ---- Status LEDs ----
    s += text_note("Status LEDs\\n"
                   "PF12=LED1 (power), PF13=LED2 (heartbeat),\\n"
                   "PF14=LED3 (activity), PF15=LED4 (error)\\n"
                   "Active-high, 1K series resistors", 25.4, 220, 1.5)
    
    led_y = 250
    leds = [
        ("LED_1", "D1", "R25", "1k", "Green", "Power OK"),
        ("LED_2", "D2", "R26", "1k", "Blue", "Heartbeat"),
        ("LED_3", "D3", "R27", "1k", "Yellow", "Activity"),
        ("LED_4", "D4", "R28", "1k", "Red", "Error"),
    ]
    for i, (gpio_name, led_ref, res_ref, res_val, color, purpose) in enumerate(leds):
        x = 25.4 + i * 25.4
        y = led_y
        s += global_label(f"{gpio_name}_Pin", x, y, "output", "output")
        s += resistor(res_ref, res_val, x + 10, y)
        s += wire(x + 5, y, x + 10, y - 2.54)
        s += led(led_ref, color, x + 10, y + 7)
        s += wire(x + 10, y + 2.54, x + 10, y + 7 - 2.54)
        s += power_symbol("GND", f"pwr_gnd_led_{i}", x + 10, y + 7 + 5.08)
        s += wire(x + 10, y + 7 + 2.54, x + 10, y + 7 + 5.08 - 1.27)
        s += text_note(f"{color}\\n{purpose}", x + 5, y + 15, 1.0)
    
    # ---- Watchdog ----
    s += text_note("Internal Watchdog (IWDG)\\n"
                   "Configured in firmware - no external components\\n"
                   "Timeout protects against MCU lockup during PA operation", 25.4, 290, 1.5)
    
    s += sheet_footer("6")
    return s


# ============================================================================
# MAIN - Generate all sheets
# ============================================================================
if __name__ == "__main__":
    sheets = {
        "MCU_Core.kicad_sch": generate_mcu_core(),
        "Comm_Interfaces.kicad_sch": generate_comm_interfaces(),
        "FPGA_Interface.kicad_sch": generate_fpga_interface(),
        "Sensors_Actuators.kicad_sch": generate_sensors_actuators(),
        "Power_Sequencing.kicad_sch": generate_power_sequencing(),
    }
    
    for filename, content in sheets.items():
        filepath = os.path.join(OUT_DIR, filename)
        with open(filepath, "w") as f:
            f.write(content)
        print(f"Generated: {filepath}")
    
    print("\nAll 5 schematic sheets generated successfully.")
    print(f"Project directory: {OUT_DIR}")
