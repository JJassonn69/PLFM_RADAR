# U42 FPGA Connection Map — XC7A200T-2FBG484I

> **MainBoard FPGA swap reference**
> Schematic: `RADAR_Main_Board_2.kicad_sch` (Digital sheet)
> PCB: `RADAR_Main_Board.kicad_pcb`

---

## STATUS SUMMARY

| Category | Count | Status |
|---|---|---|
| Signal nets (I/O) | 77 | All connected in schematic via wire+label |
| Power pins (VCCINT/VCCBRAM/VCCAUX/VCCO/GND) | 143 | All connected in schematic via global_label |
| Unexpected extra nets found | 6 | Need review (see Section 6) |
| PCB traces routed | 0 of 77 | **All old traces deleted — needs full re-route** |

---

## 1. BANK 14 I/O — Unit 1 (27 signals)

Unit 1 is placed at schematic position **(200, 450)**. All pins exit to the **right** (pin angle 180°, meaning pin stub points left, wire goes right from symbol edge).

| # | Net Name | New Ball | Pin Function | Connects To | LVDS? |
|---|---|---|---|---|---|
| 1 | `ADC_D0_N` | V22 | IO_L3N_T0_DQS_EMCCLK_14 | U1 (AD9484) pin | N pair |
| 2 | `ADC_D0_P` | T21 | IO_L4P_T0_D04_14 | U1 (AD9484) pin | P pair |
| 3 | `ADC_D1_N` | R21 | IO_L2N_T0_D03_14 | U1 (AD9484) pin | N pair |
| 4 | `ADC_D1_P` | U22 | IO_L3P_T0_DQS_PUDC_B_14 | U1 (AD9484) pin | P pair |
| 5 | `ADC_D2_N` | R22 | IO_L1N_T0_D01_DIN_14 | U1 (AD9484) pin | N pair |
| 6 | `ADC_D2_P` | P21 | IO_L2P_T0_D02_14 | U1 (AD9484) pin | P pair |
| 7 | `ADC_D3_N` | T18 | IO_L20N_T3_A07_D23_14 | U1 (AD9484) pin | N pair |
| 8 | `ADC_D3_P` | N17 | IO_L21P_T3_DQS_14 | U1 (AD9484) pin | P pair |
| 9 | `ADC_D4_N` | R14 | IO_L19N_T3_A09_D25_VREF_14 | U1 (AD9484) pin | N pair |
| 10 | `ADC_D4_P` | R18 | IO_L20P_T3_A08_D24_14 | U1 (AD9484) pin | P pair |
| 11 | `ADC_D5_N` | V19 | IO_L14N_T2_SRCC_14 | U1 (AD9484) pin | N pair |
| 12 | `ADC_D5_P` | AA19 | IO_L15P_T2_DQS_RDWR_B_14 | U1 (AD9484) pin | P pair |
| 13 | `ADC_D6_N` | AB20 | IO_L15N_T2_DQS_DOUT_CSO_B_14 | U1 (AD9484) pin | N pair |
| 14 | `ADC_D6_P` | V17 | IO_L16P_T2_CSI_B_14 | U1 (AD9484) pin | P pair |
| 15 | `ADC_D7_P` | Y18 | IO_L13P_T2_MRCC_14 | U1 (AD9484) pin | single |
| 16 | `ADC_OR_N` | AB18 | IO_L17N_T2_A13_D29_14 | U1 (AD9484) overrange | N pair |
| 17 | `ADC_OR_P` | U17 | IO_L18P_T2_A12_D28_14 | U1 (AD9484) overrange | P pair |
| 18 | `ADC_PWRD` | Y19 | IO_L13N_T2_MRCC_14 | U1 (AD9484) power down | single |
| 19 | `FPGA_ADC_CLOCK_N` | N14 | IO_L23N_T3_A02_D18_14 | ADC data clock | N pair |
| 20 | `FPGA_ADC_CLOCK_P` | P16 | IO_L24P_T3_A01_D17_14 | ADC data clock | P pair |
| 21 | `FPGA_FLASH_DQ0` | U20 | IO_L11P_T1_SRCC_14 | U9 (MT25QL) DQ0 | single |
| 22 | `FPGA_FLASH_DQ1` | AB22 | IO_L10N_T1_D15_14 | U9 (MT25QL) DQ1 | single |
| 23 | `FPGA_FLASH_DQ2` | AB21 | IO_L10P_T1_D14_14 | U9 (MT25QL) DQ2 | single |
| 24 | `FPGA_FLASH_DQ3` | Y22 | IO_L9N_T1_DQS_D13_14 | U9 (MT25QL) DQ3 | single |
| 25 | `FPGA_FLASH_NCS` | T19 | IO_L6P_T0_FCS_B_14 | U9 (MT25QL) chip select | single |
| 26 | `FPGA_FLASH_NRST` | T20 | IO_L6N_T0_D08_VREF_14 | U9 (MT25QL) reset | single |
| 27 | `FPGA_PUDC_B` | Y21 | IO_L9P_T1_DQS_14 | Internal pull-up control | single |

### Bank 14 Power: `+3V3_FPGA` (VCCO_14)
| Ball | Function |
|---|---|
| M14 | VCCO_14 |
| P18 | VCCO_14 |
| R15 | VCCO_14 |
| T22 | VCCO_14 |
| U19 | VCCO_14 |
| Y20 | VCCO_14 |

### Bank 13 Power: `+3V3_FPGA` (VCCO_13) — also in Unit 1
| Ball | Function |
|---|---|
| AA17 | VCCO_13 |
| AB14 | VCCO_13 |
| V16 | VCCO_13 |
| W13 | VCCO_13 |
| Y10 | VCCO_13 |

---

## 2. BANK 15/16 I/O — Unit 2 (28 signals)

Unit 2 is placed at schematic position **(380, 450)**. All pins exit to the **left** (pin angle 0°, pin stub points right, wire goes left from symbol edge).

| # | Net Name | New Ball | Pin Function | Connects To | Type |
|---|---|---|---|---|---|
| 1 | `ADAR_1_CS_3V3` | M22 | IO_L15N_T2_DQS_ADV_B_15 | ADAR1_0 chip select (3V3 domain) | single |
| 2 | `ADAR_2_CS_3V3` | N22 | IO_L15P_T2_DQS_15 | ADAR2_0 chip select (3V3 domain) | single |
| 3 | `ADAR_3_CS_3V3` | L20 | IO_L14N_T2_SRCC_15 | ADAR3_0 chip select (3V3 domain) | single |
| 4 | `ADAR_4_CS_3V3` | L19 | IO_L14P_T2_SRCC_15 | ADAR4_0 chip select (3V3 domain) | single |
| 5 | `DAC_0` | G18 | IO_L4N_T0_15 | U3 (AD9708) D0 | single |
| 6 | `DAC_1` | J15 | IO_L5P_T0_AD9P_15 | U3 (AD9708) D1 | single |
| 7 | `DAC_2` | H18 | IO_L6N_T0_VREF_15 | U3 (AD9708) D2 | single |
| 8 | `DAC_3` | H22 | IO_L7N_T1_AD2N_15 | U3 (AD9708) D3 | single |
| 9 | `DAC_4` | H20 | IO_L8P_T1_AD10P_15 | U3 (AD9708) D4 | single |
| 10 | `DAC_5` | G20 | IO_L8N_T1_AD10N_15 | U3 (AD9708) D5 | single |
| 11 | `DAC_6` | K22 | IO_L9N_T1_DQS_AD3N_15 | U3 (AD9708) D6 | single |
| 12 | `DAC_7` | M21 | IO_L10P_T1_AD11P_15 | U3 (AD9708) D7 | single |
| 13 | `DAC_SLEEP` | G16 | IO_L2N_T0_AD8N_15 | U3 (AD9708) sleep | single |
| 14 | `DIG_0` | L13 | IO_L20N_T3_A19_15 | General purpose digital I/O | single |
| 15 | `DIG_1` | M13 | IO_L20P_T3_A20_15 | General purpose digital I/O | single |
| 16 | `DIG_2` | K14 | IO_L19N_T3_A21_VREF_15 | General purpose digital I/O | single |
| 17 | `DIG_3` | K13 | IO_L19P_T3_A22_15 | General purpose digital I/O | single |
| 18 | `DIG_4` | M20 | IO_L18N_T2_A23_15 | General purpose digital I/O | single |
| 19 | `DIG_5` | N20 | IO_L18P_T2_A24_15 | General purpose digital I/O | single |
| 20 | `DIG_6` | N19 | IO_L17N_T2_A25_15 | General purpose digital I/O | single |
| 21 | `DIG_7` | N18 | IO_L17P_T2_A26_15 | General purpose digital I/O | single |
| 22 | `FPGA_CLOCK_TEST` | K18 | IO_L13P_T2_MRCC_15 | Clock test point | single |
| 23 | `FPGA_SYS_CLOCK` | M15 | IO_L24P_T3_RS1_15 | System clock input | single |
| 24 | `MIX_RX_EN` | L15 | IO_L22N_T3_A16_15 | Mixer RX enable | single |
| 25 | `MIX_TX_EN` | H13 | IO_L1P_T0_AD0P_15 | Mixer TX enable | single |
| 26 | `STM32_MISO1` | M18 | IO_L16P_T2_A28_15 | U2 (STM32) SPI MISO (3V3 domain) | single |
| 27 | `STM32_MOSI1` | L18 | IO_L16N_T2_A27_15 | U2 (STM32) SPI MOSI (3V3 domain) | single |
| 28 | `STM32_SCLK1` | K19 | IO_L13N_T2_MRCC_15 | U2 (STM32) SPI clock (3V3 domain) | single |

### Bank 15 Power: `+3V3_FPGA` (VCCO_15)
| Ball | Function |
|---|---|
| G19 | VCCO_15 |
| H16 | VCCO_15 |
| J13 | VCCO_15 |
| K20 | VCCO_15 |
| L17 | VCCO_15 |
| N21 | VCCO_15 |

### Bank 16 Power: `+3V3_FPGA` (VCCO_16)
| Ball | Function |
|---|---|
| A17 | VCCO_16 |
| B14 | VCCO_16 |
| C21 | VCCO_16 |
| D18 | VCCO_16 |
| E15 | VCCO_16 |
| F22 | VCCO_16 |

---

## 3. BANK 34/35 I/O — Unit 3 (22 signals)

Unit 3 is placed at schematic position **(560, 450)**. All pins exit to the **left** (pin angle 0°).

| # | Net Name | New Ball | Pin Function | Connects To | Type |
|---|---|---|---|---|---|
| 1 | `ADAR_1_CS_1V8` | Y2 | IO_L4N_T0_34 | ADAR1_0 chip select (1V8 domain) | single |
| 2 | `ADAR_2_CS_1V8` | W2 | IO_L4P_T0_34 | ADAR2_0 chip select (1V8 domain) | single |
| 3 | `ADAR_3_CS_1V8` | R2 | IO_L3N_T0_DQS_34 | ADAR3_0 chip select (1V8 domain) | single |
| 4 | `ADAR_4_CS_1V8` | R3 | IO_L3P_T0_DQS_34 | ADAR4_0 chip select (1V8 domain) | single |
| 5 | `ADAR_RX_LOAD_1` | AA5 | IO_L10P_T1_34 | ADAR1_0 RX load | single |
| 6 | `ADAR_RX_LOAD_2` | AB1 | IO_L7N_T1_34 | ADAR2_0 RX load | single |
| 7 | `ADAR_RX_LOAD_3` | AB2 | IO_L8N_T1_34 | ADAR3_0 RX load | single |
| 8 | `ADAR_RX_LOAD_4` | AA3 | IO_L9N_T1_DQS_34 | ADAR4_0 RX load | single |
| 9 | `ADAR_TR_1` | U1 | IO_L1N_T0_34 | ADAR1_0 T/R switch | single |
| 10 | `ADAR_TR_2` | T1 | IO_L1P_T0_34 | ADAR2_0 T/R switch | single |
| 11 | `ADAR_TR_3` | T3 | IO_0_34 | ADAR3_0 T/R switch | single |
| 12 | `ADAR_TX_LOAD_2` | AA1 | IO_L7P_T1_34 | ADAR2_0 TX load | single |
| 13 | `ADAR_TX_LOAD_3` | AB3 | IO_L8P_T1_34 | ADAR3_0 TX load | single |
| 14 | `ADAR_TX_LOAD_4` | Y3 | IO_L9P_T1_DQS_34 | ADAR4_0 TX load | single |
| 15 | `STM32_MISO_1V8` | U2 | IO_L2P_T0_34 | U2 (STM32) SPI MISO (1V8 domain) | single |
| 16 | `STM32_MOSI_1V8` | V2 | IO_L2N_T0_34 | U2 (STM32) SPI MOSI (1V8 domain) | single |
| 17 | `STM32_SCLK_1V8` | U3 | IO_L6P_T0_34 | U2 (STM32) SPI clock (1V8 domain) | single |
| 18 | `FPGA_FLASH_CLK` | *see Unit 6* | *shared net — label only here* | U9 (MT25QL) SPI clock | single |
| 19 | `FPGA_FLASH_DQ0` | *see Unit 1* | *shared net — label only here* | U9 (MT25QL) DQ0 | single |
| 20 | `FPGA_FLASH_DQ1` | *see Unit 1* | *shared net — label only here* | U9 (MT25QL) DQ1 | single |
| 21 | `FPGA_FLASH_NCS` | *see Unit 1* | *shared net — label only here* | U9 (MT25QL) chip select | single |
| 22 | `FPGA_FLASH_NRST` | *see Unit 1* | *shared net — label only here* | U9 (MT25QL) reset | single |

> **Note:** Items 18-22 are labels present near Unit 3 that share net names with Unit 1/Unit 6 pins. The actual FPGA ball for these signals is assigned via Unit 1 (for DQ0-3, NCS, NRST) and Unit 6 (for CLK). These labels likely connect to the flash IC nearby.

### Bank 34 Power: `+1V8_FPGA` (VCCO_34)
| Ball | Function |
|---|---|
| AA7 | VCCO_34 |
| AB4 | VCCO_34 |
| R5 | VCCO_34 |
| T2 | VCCO_34 |
| V6 | VCCO_34 |
| W3 | VCCO_34 |

### Bank 35 Power: `+1V8_FPGA` (VCCO_35)
| Ball | Function |
|---|---|
| C1 | VCCO_35 |
| F2 | VCCO_35 |
| H6 | VCCO_35 |
| J3 | VCCO_35 |
| M4 | VCCO_35 |
| N1 | VCCO_35 |

---

## 4. CONFIGURATION & JTAG — Unit 6 (5 signals)

Unit 6 is placed at schematic position **(90, 450)**. Pins exit to the **right** (pin angle 180°).

| # | Net Name | New Ball | Pin Function | Connects To | Type |
|---|---|---|---|---|---|
| 1 | `FPGA_FLASH_CLK` | L12 | CCLK_0 | U9 (MT25QL) SPI clock | dedicated |
| 2 | `FPGA_TCK` | V12 | TCK_0 | JTAG test clock | dedicated |
| 3 | `FPGA_TDI` | R13 | TDI_0 | JTAG test data in | dedicated |
| 4 | `FPGA_TDO` | U13 | TDO_0 | JTAG test data out | dedicated |
| 5 | `FPGA_TMS` | T13 | TMS_0 | JTAG test mode select | dedicated |

> **CRITICAL**: Do NOT use FT_PROG on the FT2232HQ EEPROM — will permanently break JTAG.

### Bank 0 Power: `+1V8_FPGA` (VCCO_0) — in Unit 6
| Ball | Function |
|---|---|
| F12 | VCCO_0 |
| T12 | VCCO_0 |

### Other Unit 6 dedicated pins (no signal net, but need power/config wiring)

> **NOTE**: These balls were corrected on 2026-03-26. The original table had OLD XC7A50T
> ball numbers paired with correct function names. The table below uses the correct
> XC7A200T-FBG484 ball assignments (verified against KiCad symbol and Xilinx pinout file).

| Ball | Function | Old XC7A50T Ball | Required Connection | How to Wire in KiCad |
|---|---|---|---|---|
| **U8** | **CFGBVS_0** | E7 | Tie to VCCO_0 (+1V8_FPGA) — tells FPGA that Bank 0/14/15 config I/O use 1.8V levels | Wire directly to +1V8_FPGA power rail (no resistor needed). Place a global_label `+1V8_FPGA` on the pin. |
| **G11** | **DONE_0** | H10 | Config done indicator. Directly connect to VCCO_0 through a 330R pull-up. Active-high when config is complete. Optional: route to an LED or STM32 GPIO for status monitoring. | Add a 330Ω resistor from this pin to +1V8_FPGA. In KiCad: wire pin → net label `FPGA_DONE`, then place a resistor symbol (R, 330Ω) between that net and a +1V8_FPGA power symbol. |
| **U12** | **INIT_B_0** | K10 | Active-low config error flag. Pull up to VCCO_0 through 4.7kΩ. Goes low if config CRC fails. Optional: route to STM32 GPIO. | Add a 4.7kΩ resistor from this pin to +1V8_FPGA. Same pattern as DONE_0 above. |
| **N12** | **PROGRAM_B_0** | L9 | Active-low config reset. Pull up to VCCO_0 through 4.7kΩ. Directly connect or route to a push-button that shorts to GND for manual re-programming. | Add a 4.7kΩ resistor from this pin to +1V8_FPGA. Optional: add a tactile switch to GND for manual reset. |
| **U11** | **M0_0** | M9 | Mode select bit 0. For Master SPI mode (to boot from flash U9): tie to **GND** | Wire to GND power symbol. |
| **U10** | **M1_0** | M10 | Mode select bit 1. For Master SPI mode: tie to **GND** | Wire to GND power symbol. |
| **U9** | **M2_0** | M11 | Mode select bit 2. For Master SPI mode: tie to **VCCO_0 (+1V8_FPGA)** via 4.7kΩ pull-up | Add a 4.7kΩ resistor from this pin to +1V8_FPGA. |
| **L9** | **VREFN_0** | H7 | XADC negative reference. If XADC not used: tie to **GND** through a 0Ω/short or leave connected to GND plane. If XADC used: connect to external 1.25V reference negative. | Wire to GND power symbol (XADC not used in this design). |
| **L10** | **VP_0** | H8 | XADC positive analog input. If XADC not used: leave unconnected (has internal pull-down) or tie to GND through 100Ω. | Leave unconnected or add 100Ω to GND. Mark with no-connect flag (X) in KiCad if leaving open. |
| **M9** | **VN_0** | J7 | XADC negative analog input. If XADC not used: leave unconnected or tie to GND through 100Ω. | Same as VP_0 — leave unconnected or 100Ω to GND. |
| **M10** | **VREFP_0** | J8 | XADC positive reference. If XADC not used: tie to **VCCADC** (+1V8_FPGA) through 100Ω, or leave unconnected. | Wire 100Ω to +1V8_FPGA, or mark no-connect. |
| **N9** | **DXN_0** | K7 | Thermal diode negative. If not monitoring die temperature externally: leave unconnected. | Place no-connect flag (X) on pin in KiCad. |
| **N10** | **DXP_0** | K8 | Thermal diode positive. If not monitoring die temperature externally: leave unconnected. | Place no-connect flag (X) on pin in KiCad. |

#### Mode Select Truth Table (M[2:1:0])

| M2 | M1 | M0 | Mode | Use Case |
|---|---|---|---|---|
| 0 | 0 | 0 | Master Serial | — |
| 0 | 0 | 1 | **Master SPI** | **Boot from flash U9 (MT25QL) — USE THIS** |
| 0 | 1 | 0 | Master BPI (x8) | — |
| 0 | 1 | 1 | Master BPI (x16) | — |
| 1 | 0 | 0 | Master Serial | — |
| 1 | 0 | 1 | JTAG only | Debug/development |
| 1 | 1 | 0 | Slave SelectMAP | — |
| 1 | 1 | 1 | Slave Serial | — |

> **For Master SPI boot from U9**: M2=0, M1=0, M0=1 → **M2=GND, M1=GND, M0=VCCO** (pull-up to +1V8_FPGA)
>
> Alternatively, during development you may want JTAG-only mode (M2=1, M1=0, M0=1 → M2=VCCO, M1=GND, M0=VCCO).
> Consider using 4.7kΩ pull-up/pull-down resistors instead of direct ties so mode pins can be overridden with jumpers.

#### How to Wire Pull-Up/Pull-Down Resistors in KiCad Schematic

**General pattern for a pull-up resistor:**
1. From the FPGA dedicated pin, draw a short wire stub (e.g., 2.54mm / 1 grid unit)
2. Place a resistor symbol (`Device:R`) on that wire stub, oriented vertically
3. Set the resistor value in properties (e.g., `4.7k`, `330`, `100`)
4. Set a reference designator (KiCad will auto-assign, e.g., R_NEW1)
5. From the other end of the resistor, draw a wire stub and attach a **power symbol**:
   - For pull-up to 1.8V: use `+1V8_FPGA` global power label (or the power symbol from your library)
   - For pull-down to GND: use `GND` power symbol
6. The resistor does NOT need a footprint assigned yet — you can assign during annotation/footprint assignment. Use 0402 or 0603 SMD footprint.

**For direct ties (no resistor):**
1. Wire directly from the FPGA pin to the power symbol (e.g., CFGBVS_0 → +1V8_FPGA)

**For no-connect pins:**
1. Select the "Place No Connect Flag" tool (shortcut: `Q` in KiCad 10)
2. Click on the pin endpoint to place an X mark
3. This tells ERC that the pin is intentionally unconnected

#### Resistor Value Summary for Config Pins

| Pin | Resistor | To | Value | Purpose |
|---|---|---|---|---|
| CFGBVS_0 (U8) | none (direct tie) | +1V8_FPGA | — | Sets config voltage level |
| DONE_0 (G11) | R_pull-up | +1V8_FPGA | 330Ω | Config done indicator |
| INIT_B_0 (U12) | R_pull-up | +1V8_FPGA | 4.7kΩ | Config error flag |
| PROGRAM_B_0 (N12) | R_pull-up | +1V8_FPGA | 4.7kΩ | Config reset (active-low) |
| M0_0 (U11) | R_pull-up | +1V8_FPGA | 4.7kΩ | Mode = 1 (Master SPI) |
| M1_0 (U10) | R_pull-down | GND | 4.7kΩ | Mode = 0 |
| M2_0 (U9) | R_pull-down | GND | 4.7kΩ | Mode = 0 |
| VREFN_0 (L9) | none (direct tie) | GND | — | XADC not used |
| VREFP_0 (M10) | 100Ω | +1V8_FPGA | 100Ω | XADC ref (or no-connect) |
| VP_0 (L10) | no-connect or 100Ω | GND | 100Ω | XADC not used |
| VN_0 (M9) | no-connect or 100Ω | GND | 100Ω | XADC not used |
| DXN_0 (N9) | no-connect | — | — | Thermal diode not used |
| DXP_0 (N10) | no-connect | — | — | Thermal diode not used |

---

## 5. POWER & GROUND — Unit 7 (112 pins)

Unit 7 is placed at schematic position **(90, 300)**. Power pins exit **left** (offset -25.4) and **right** (offset +25.4).

### +1V0_FPGA — VCCINT (14 balls)
| Ball | Side |
|---|---|
| H8 | right |
| H10 | right |
| J7 | right |
| J9 | right |
| K8 | right |
| L7 | right |
| M8 | right |
| N7 | right |
| P8 | right |
| P10 | right |
| R7 | right |
| R9 | right |
| T8 | right |
| T10 | right |

### +1V0_FPGA — VCCBRAM (3 balls)
| Ball | Side |
|---|---|
| J11 | left |
| L11 | left |
| N11 | left |

### +1V8_FPGA — VCCAUX (5 balls)
| Ball | Side |
|---|---|
| H12 | left |
| K12 | left |
| M12 | left |
| P12 | left |
| R11 | left |

### +1V8_FPGA — VCCADC (1 ball)
| Ball |
|---|
| K10 |

### +1V8_FPGA — VCCBATT (1 ball)
| Ball |
|---|
| E12 |

### GND (88 balls)

**Left-side GND pins (48 balls):**
A2, A3, A5, A7, A9, A11, A12, A22, B3, B12, B19, C3, C6, C10, C12, C16, D3, D4, D8, D12, D13, E4, E5, E7, E9, E11, E20, F5, F11, F17, G5, G6, G7, G8, G9, G10, G12, G14, H1, H7, H9, H11, H21, J8, K9 (GNDADC)

**Right-side GND pins (40 balls):**
J10, J12, J18, K5, K7, K11, K15, L2, L8, L22, M7, M11, M19, N6, N8, N16, P3, P7, P9, P11, P13, R8, R10, R12, R20, T7, T9, T11, T17, U4, U14, V1, V11, V21, W8, W18, Y5, Y15, AA2, AA12, AA22, AB9, AB19

---

## 6. UNEXPECTED NETS (need review)

The audit found 6 nets connected near U42 that are **not in the original 77-signal migration map**:

| Net Name | Label Type | Near Unit | Action Needed |
|---|---|---|---|
| `ADAR_TR_4` | global_label | Unit 3 | **Missing from migration map** — was this on the old FPGA? |
| `ADAR_TX_LOAD_1` | global_label | Unit 3 | **Missing from migration map** — was this on the old FPGA? |
| `ADC_DCO_N` | global_label | Unit 1 & 6 | ADC data clock output (negative) — may be alternate name for ADC clock |
| `ADC_DCO_P` | global_label | Unit 1 & 6 | ADC data clock output (positive) — may be alternate name for ADC clock |
| `FPGA_DAC_CLOCK` | label | Unit 2 | DAC clock output — likely pre-existing from old design |
| `M3S_VCTRL` | global_label | Unit 2 | VCO control voltage — likely pre-existing from old design |

> These may be pre-existing connections from the old XC7A50T design that weren't in the migration map, or they could be artifacts. Check if these nets need FPGA ball assignments or should be removed.

---

## 7. PCB RE-ROUTING CHECKLIST

All 77 signal traces were deleted from the PCB. They need to be re-routed from the new BGA484 pads to their destination components. The new footprint (`Xilinx_FBG484`) is placed at the same center position as the old BGA256.

### Priority 1 — High-speed LVDS (route as differential pairs, length-matched)
| Pair | Ball P | Ball N | Target |
|---|---|---|---|
| ADC_D0 | T21 | V22 | U1 (AD9484) |
| ADC_D1 | U22 | R21 | U1 |
| ADC_D2 | P21 | R22 | U1 |
| ADC_D3 | N17 | T18 | U1 |
| ADC_D4 | R18 | R14 | U1 |
| ADC_D5 | AA19 | V19 | U1 |
| ADC_D6 | V17 | AB20 | U1 |
| ADC_OR | U17 | AB18 | U1 |
| ADC_CLOCK | P16 | N14 | U1 |

### Priority 2 — SPI buses (matched length within bus)
| Bus | Signals | Balls | Target |
|---|---|---|---|
| STM32 SPI (3V3) | MISO1, MOSI1, SCLK1 | M18, L18, K19 | U2 (STM32) |
| STM32 SPI (1V8) | MISO, MOSI, SCLK | U2, V2, U3 | U2 (STM32) via level shifter |
| ADAR SPI CS (3V3) | CS_1-4 | M22, N22, L20, L19 | ADAR1000 x4 |
| ADAR SPI CS (1V8) | CS_1-4 | Y2, W2, R2, R3 | ADAR1000 x4 via level shifter |

### Priority 3 — DAC parallel bus
| Signal | Ball | Target |
|---|---|---|
| DAC_0 | G18 | U3 (AD9708) |
| DAC_1 | J15 | U3 |
| DAC_2 | H18 | U3 |
| DAC_3 | H22 | U3 |
| DAC_4 | H20 | U3 |
| DAC_5 | G20 | U3 |
| DAC_6 | K22 | U3 |
| DAC_7 | M21 | U3 |
| DAC_SLEEP | G16 | U3 |

### Priority 4 — Flash SPI
| Signal | Ball | Target |
|---|---|---|
| FLASH_CLK | L12 (CCLK) | U9 (MT25QL) |
| FLASH_DQ0 | U20 | U9 |
| FLASH_DQ1 | AB22 | U9 |
| FLASH_DQ2 | AB21 | U9 |
| FLASH_DQ3 | Y22 | U9 |
| FLASH_NCS | T19 | U9 |
| FLASH_NRST | T20 | U9 |

### Priority 5 — JTAG (keep short, avoid stubs)
| Signal | Ball | Target |
|---|---|---|
| TCK | V12 | FT2232HQ JTAG header |
| TDI | R13 | FT2232HQ |
| TDO | U13 | FT2232HQ |
| TMS | T13 | FT2232HQ |

### Priority 6 — Control signals
| Signal | Ball | Target |
|---|---|---|
| MIX_RX_EN | L15 | Mixer control |
| MIX_TX_EN | H13 | Mixer control |
| ADAR_TR_1 | U1 | ADAR T/R switch |
| ADAR_TR_2 | T1 | ADAR T/R switch |
| ADAR_TR_3 | T3 | ADAR T/R switch |
| ADAR_RX_LOAD_1-4 | AA5, AB1, AB2, AA3 | ADAR RX load |
| ADAR_TX_LOAD_2-4 | AA1, AB3, Y3 | ADAR TX load |
| ADC_PWRD | Y19 | ADC power down |
| FPGA_PUDC_B | Y21 | Internal pull-up during config |

### Priority 7 — Clock and digital I/O
| Signal | Ball | Target |
|---|---|---|
| FPGA_SYS_CLOCK | M15 | System clock oscillator |
| FPGA_CLOCK_TEST | K18 | Test point |
| DIG_0-7 | L13,M13,K14,K13,M20,N20,N19,N18 | GPIO header/test |

### Priority 8 — Power fanout vias (BGA escape routing)
Route power planes/vias from BGA pads to internal power layers:
- 14x VCCINT (1V0) + 3x VCCBRAM (1V0) balls to `+1V0_FPGA` plane
- 5x VCCAUX (1V8) + 1x VCCADC + 1x VCCBATT + 12x VCCO_34/35 + 2x VCCO_0 balls to `+1V8_FPGA` plane
- 11x VCCO_13/14 + 12x VCCO_15/16 balls to `+3V3_FPGA` plane
- 88x GND balls to GND plane

---

## 8. OLD-TO-NEW BALL CROSS-REFERENCE

For tracing where each signal moved from on the old XC7A50T (256-BGA) to the new XC7A200T (484-BGA):

| Net | Old Ball (XC7A50T) | New Ball (XC7A200T) | Same Function? |
|---|---|---|---|
| ADC_D0_N | M15 | V22 | Yes |
| ADC_D0_P | L14 | T21 | Yes |
| ADC_D1_N | K16 | R21 | Yes |
| ADC_D1_P | L15 | U22 | Yes |
| ADC_D2_N | J14 | R22 | Yes |
| ADC_D2_P | K15 | P21 | Yes |
| ADC_D3_N | R8 | T18 | Yes |
| ADC_D3_P | T7 | N17 | Yes |
| ADC_D4_N | N6 | R14 | Yes |
| ADC_D4_P | P8 | R18 | Yes |
| ADC_D5_N | P11 | V19 | Yes |
| ADC_D5_P | R12 | AA19 | Yes |
| ADC_D6_N | T12 | AB20 | Yes |
| ADC_D6_P | R13 | V17 | Yes |
| ADC_D7_P | N11 | Y18 | Yes |
| ADC_OR_N | R11 | AB18 | Yes |
| ADC_OR_P | N9 | U17 | Yes |
| ADC_PWRD | N12 | Y19 | Yes |
| FPGA_ADC_CLOCK_N | T5 | N14 | Yes |
| FPGA_ADC_CLOCK_P | R6 | P16 | Yes |
| FPGA_FLASH_DQ0 | N13 | U20 | Yes |
| FPGA_FLASH_DQ1 | T15 | AB22 | Yes |
| FPGA_FLASH_DQ2 | T14 | AB21 | Yes |
| FPGA_FLASH_DQ3 | R16 | Y22 | Yes |
| FPGA_FLASH_NCS | L12 | T19 | Yes |
| FPGA_FLASH_NRST | M12 | T20 | Yes |
| FPGA_PUDC_B | R15 | Y21 | Yes |
| ADAR_1_CS_3V3 | D15 | M22 | Yes |
| ADAR_2_CS_3V3 | D14 | N22 | Yes |
| ADAR_3_CS_3V3 | D11 | L20 | Yes |
| ADAR_4_CS_3V3 | D16 | L19 | Yes |
| DAC_0 | B11 | G18 | Yes |
| DAC_1 | B12 | J15 | Yes |
| DAC_2 | D9 | H18 | Yes |
| DAC_3 | A14 | H22 | Yes |
| DAC_4 | C14 | H20 | Yes |
| DAC_5 | B14 | G20 | Yes |
| DAC_6 | A15 | K22 | Yes |
| DAC_7 | C16 | M21 | Yes |
| DAC_SLEEP | A9 | G16 | Yes |
| DIG_0 | H13 | L13 | Yes |
| DIG_1 | H12 | M13 | Yes |
| DIG_2 | G12 | K14 | Yes |
| DIG_3 | H11 | K13 | Yes |
| DIG_4 | E15 | M20 | Yes |
| DIG_5 | F15 | N20 | Yes |
| DIG_6 | D16 | N19 | Yes |
| DIG_7 | E16 | N18 | Yes |
| FPGA_CLOCK_TEST | E12 | K18 | Yes |
| FPGA_SYS_CLOCK | H14 | M15 | Yes |
| MIX_RX_EN | G16 | L15 | Yes |
| MIX_TX_EN | C8 | H13 | Yes |
| STM32_MISO1 | F12 | M18 | Yes |
| STM32_MOSI1 | F13 | L18 | Yes |
| STM32_SCLK1 | E13 | K19 | Yes |
| ADAR_1_CS_1V8 | P1 | Y2 | Yes |
| ADAR_2_CS_1V8 | N1 | W2 | Yes |
| ADAR_3_CS_1V8 | N2 | R2 | Yes |
| ADAR_4_CS_1V8 | N3 | R3 | Yes |
| ADAR_RX_LOAD_1 | P5 | AA5 | Yes |
| ADAR_RX_LOAD_2 | R1 | AB1 | Yes |
| ADAR_RX_LOAD_3 | T2 | AB2 | Yes |
| ADAR_RX_LOAD_4 | T3 | AA3 | Yes |
| ADAR_TR_1 | M4 | U1 | Yes |
| ADAR_TR_2 | L4 | T1 | Yes |
| ADAR_TR_3 | L5 | T3 | Yes |
| ADAR_TX_LOAD_2 | R2 | AA1 | Yes |
| ADAR_TX_LOAD_3 | R3 | AB3 | Yes |
| ADAR_TX_LOAD_4 | T4 | Y3 | Yes |
| STM32_MISO_1V8 | M2 | U2 | Yes |
| STM32_MOSI_1V8 | M1 | V2 | Yes |
| STM32_SCLK_1V8 | M5 | U3 | Yes |
| FPGA_FLASH_CLK | E8 | L12 | Yes |
| FPGA_TCK | L7 | V12 | Yes |
| FPGA_TDI | N7 | R13 | Yes |
| FPGA_TDO | N8 | U13 | Yes |
| FPGA_TMS | M7 | T13 | Yes |
