# U42 New Connections Guide — XC7A200T-2FBG484I

> **Purpose**: This guide covers ALL connections that need to be added beyond the
> original 77-signal migration. It includes: missed legacy signals, the complete
> FT601 USB 3.0 interface, STM32 control signals, and passive component requirements.
>
> **Schematic**: `RADAR_Main_Board_2.kicad_sch` (Digital sheet)
> **All connections go to U42 (FPGA)**

---

## OVERVIEW

| Category | Signal Count | Status |
|---|---|---|
| Original migration (in U42_CONNECTION_MAP.md) | 77 signals + 13 config + 143 power | Done |
| **GROUP A: Missed legacy signals** | **7 signals** | **NEW — wire these first** |
| **GROUP B: FT601 USB 3.0 interface** | **44 signals** | **NEW — entire new interface** |
| **GROUP C: STM32 control GPIO** | **4 signals** | **NEW — RTL expects these** |
| **GROUP D: Decoupling & termination passives** | ~25 components | **NEW — add to schematic** |
| Total new I/O connections | **55 signals** | |
| Grand total after this guide | **132 signals** | |

### What About the Remaining 153 Unused I/O Pins?

The XC7A200T-FBG484 has 285 total I/O pins. After wiring 132 signals, 153 I/O pins
remain unused. These should be left unconnected in the schematic and marked with
no-connect flags (X) in KiCad to suppress ERC warnings. Unused pins have internal
weak pull-downs and are safe to leave floating. You can assign no-connect flags in
bulk after all connections are complete.

---

## GROUP A: MISSED LEGACY SIGNALS (7 signals)

These were connected on the old XC7A50T but were accidentally omitted from the
77-signal migration map. They MUST be wired for the board to function correctly.

### A1. ADC_D7_N (CRITICAL — completes ADC data bus)

| Parameter | Value |
|---|---|
| **Net name** | `ADC_D7_N` |
| **FPGA ball** | **P17** |
| **FPGA pin function** | `IO_L21N_T3_DQS_A06_D22_14` |
| **Bank** | 14 (VCCO = 3.3V) |
| **KiCad unit** | Unit 1 |
| **Connects to** | U1 (AD9484) pin 19 via R10 (series resistor) |
| **Old XC7A50T ball** | R7 |
| **Signal type** | LVDS negative (differential pair with ADC_D7_P on Y18) |
| **Why missing** | The migration only had ADC_D7_P — the N leg was dropped |

**Wiring instructions:**
1. On Unit 1 of U42, find the pin `IO_L21N_T3_DQS_A06_D22_14` (ball P17)
2. Draw a wire from this pin and place a label `ADC_D7_N`
3. This label already exists in the schematic connecting to U1 via R10
4. No new passive components needed — R10 already exists

**PCB note:** Route as differential pair with ADC_D7_P (Y18). Match length to other
ADC pairs. Keep spacing consistent with other LVDS pairs.

---

### A2. ADC_DCO_P and ADC_DCO_N (CRITICAL — ADC data capture clock)

| Parameter | ADC_DCO_P | ADC_DCO_N |
|---|---|---|
| **Net name** | `ADC_DCO_P` | `ADC_DCO_N` |
| **FPGA ball** | **W19** | **W20** |
| **FPGA pin function** | `IO_L12P_T1_MRCC_14` | `IO_L12N_T1_MRCC_14` |
| **Bank** | 14 | 14 |
| **KiCad unit** | Unit 1 | Unit 1 |
| **Connects to** | U1 (AD9484) pin 50 via R12 | U1 (AD9484) pin 49 via R12 |
| **Old XC7A50T ball** | N14 | P14 |
| **Signal type** | LVDS clock positive | LVDS clock negative |

> **IMPORTANT**: These were previously listed as "artifacts" in Section 6 of the
> original connection map. That was INCORRECT. They were genuinely connected to the
> old FPGA. The ADC_DCO (Data Clock Output) is the AD9484's output clock that
> accompanies the data — the FPGA uses it to sample the LVDS data bus. Without this,
> the FPGA cannot capture ADC data.

> **Why MRCC pins?** The DCO is a clock signal. MRCC (Multi-Region Clock Capable)
> pins can drive clock networks across multiple clock regions, which is ideal for a
> 500 MSPS ADC data capture clock.

**Wiring instructions:**
1. On Unit 1 of U42, find pins `IO_L12P_T1_MRCC_14` (W19) and `IO_L12N_T1_MRCC_14` (W20)
2. Draw wires and place labels `ADC_DCO_P` and `ADC_DCO_N`
3. These labels connect to U1 (AD9484) through R12 (100 ohm differential termination resistor, already exists on board)
4. No new passive components needed

**Passive components (already on board — verify):**
- R12: 100 ohm differential termination resistor between DCO_P and DCO_N at the FPGA input. This is standard for LVDS clock reception. R12 should already be placed near U1.

**PCB note:** Route as tightly coupled differential pair. This is the highest-priority
clock signal — keep trace length short, avoid vias if possible. Place close to the
ADC data pairs for minimum skew.

---

### A3. ADAR_TX_LOAD_1 (Beamformer control — completes TX load set)

| Parameter | Value |
|---|---|
| **Net name** | `ADAR_TX_LOAD_1` |
| **FPGA ball** | **AB5** |
| **FPGA pin function** | `IO_L10N_T1_34` |
| **Bank** | 34 (VCCO = 1.8V) |
| **KiCad unit** | Unit 3 |
| **Connects to** | ADAR1_0 (ADAR1000ACCZN) pin N3 (TX_LOAD) |
| **Old XC7A50T ball** | P3 |
| **Signal type** | Digital output (1.8V LVCMOS) |

> Previously flagged as "artifact" — confirmed it WAS connected on old FPGA.
> You already have TX_LOAD_2, TX_LOAD_3, TX_LOAD_4. This completes the set.

**Complete signal path (verified from PCB backup):**
```
U42 ball AB5 (new) -----> net ADAR_TX_LOAD_1 -----> ADAR1_0 pin N3 (TX_LOAD)
                                    |
                                    +--- R46 (4.7k, 0201) ---> GND  (pull-down)
```

**All four ADAR TX_LOAD signals follow the same pattern:**

| Signal | FPGA Ball (new) | Old Ball | Destination | Pull-down Resistor | Value | Package |
|---|---|---|---|---|---|---|
| `ADAR_TX_LOAD_1` | AB5 | P3 | ADAR1_0 pin N3 (TX_LOAD) | R46 | 4.7k ohm | 0201 |
| `ADAR_TX_LOAD_2` | AA1 | T4 | ADAR2_0 pin N3 (TX_LOAD) | R70 | 4.7k ohm | 0201 |
| `ADAR_TX_LOAD_3` | AB3 | R3 | ADAR3_0 pin N3 (TX_LOAD) | R73 | 4.7k ohm | 0201 |
| `ADAR_TX_LOAD_4` | Y3 | R2 | ADAR4_0 pin N3 (TX_LOAD) | R76 | 4.7k ohm | 0201 |

**All four ADAR RX_LOAD signals (already wired, for reference):**

| Signal | FPGA Ball (new) | Old Ball | Destination | Pull-down Resistor | Value | Package |
|---|---|---|---|---|---|---|
| `ADAR_RX_LOAD_1` | AA5 | M5 | ADAR1_0 pin N2 (RX_LOAD) | R47 | 4.7k ohm | 0201 |
| `ADAR_RX_LOAD_2` | AB1 | T2 | ADAR2_0 pin N2 (RX_LOAD) | R71 | 4.7k ohm | 0201 |
| `ADAR_RX_LOAD_3` | AB2 | R1 | ADAR3_0 pin N2 (RX_LOAD) | R74 | 4.7k ohm | 0201 |
| `ADAR_RX_LOAD_4` | AA3 | N4 | ADAR4_0 pin N2 (RX_LOAD) | R77 | 4.7k ohm | 0201 |

> **R46/R47/R70/R71/R73/R74/R76/R77 are NOT series resistors** — they are 4.7k
> pull-down resistors to GND. Purpose: ensures LOAD pins default LOW when the FPGA
> is unconfigured during power-up, preventing accidental SPI register loads to the
> ADAR1000 beamformers.

**Wiring instructions:**
1. On Unit 3 of U42, find pin `IO_L10N_T1_34` (AB5)
2. Draw a wire and place a global label `ADAR_TX_LOAD_1`
3. The label connects through the hierarchy to ADAR1_0 pin N3 on the RF sheet
4. R46 (4.7k pull-down to GND) already exists on the board — no new passive components needed

---

### A4. ADAR_TR_4 (Beamformer T/R control — completes TR set)

| Parameter | Value |
|---|---|
| **Net name** | `ADAR_TR_4` |
| **FPGA ball** | **Y1** |
| **FPGA pin function** | `IO_L5N_T0_34` |
| **Bank** | 34 (VCCO = 1.8V) |
| **KiCad unit** | Unit 3 |
| **Connects to** | ADAR4_0 (ADAR1000ACCZN) pin N6 (TR) |
| **Old XC7A50T ball** | P4 |
| **Signal type** | Digital output (1.8V LVCMOS) |

> Previously flagged as "artifact" — confirmed it WAS connected on old FPGA.
> You already have TR_1, TR_2, TR_3. This completes the set.

**Complete signal path (verified from PCB backup):**
```
U42 ball Y1 (new) -----> net ADAR_TR_4 -----> ADAR4_0 pin N6 (TR)
                                  |
                                  +--- R161 (22.1k, 0201) ---> GND  (pull-down)
```

**All four ADAR TR signals follow the same pattern:**

| Signal | FPGA Ball (new) | Old Ball | Destination | Pull-down Resistor | Value | Package |
|---|---|---|---|---|---|---|
| `ADAR_TR_1` | U1 | N2 | ADAR1_0 pin N6 (TR) | R158 | 22.1k ohm | 0201 |
| `ADAR_TR_2` | T1 | N1 | ADAR2_0 pin N6 (TR) | R159 | 22.1k ohm | 0201 |
| `ADAR_TR_3` | T3 | P1 | ADAR3_0 pin N6 (TR) | R160 | 22.1k ohm | 0201 |
| `ADAR_TR_4` | Y1 | P4 | ADAR4_0 pin N6 (TR) | R161 | 22.1k ohm | 0201 |

> **R158-R161 are NOT series resistors** — they are 22.1k pull-down resistors to GND.
> Purpose: ensures the TR pin defaults LOW (receive mode) when the FPGA is
> unconfigured or tri-stated during power-up. The ADAR1000 TR pin selects between
> transmit (HIGH) and receive (LOW) mode.

**Wiring instructions:**
1. On Unit 3, find pin `IO_L5N_T0_34` (Y1)
2. Draw a wire and place a global label `ADAR_TR_4`
3. The label connects through the hierarchy to ADAR4_0 pin N6 on the RF sheet
4. R161 (22.1k pull-down to GND) already exists on the board — no new passive components needed

---

### A5. M3S_VCTRL / fpga_rf_switch (RF switch control)

| Parameter | Value |
|---|---|
| **Net name** | `M3S_VCTRL` |
| **RTL port name** | `fpga_rf_switch` |
| **FPGA ball** | **G15** |
| **FPGA pin function** | `IO_L2P_T0_AD8P_15` |
| **Bank** | 15 (VCCO = 3.3V) |
| **KiCad unit** | Unit 2 |
| **Connects to** | RF switch network (RF_SW_1..RF_SW_16 pin 6, M3SWA2-34DR+ U$1 pin 6) |
| **Old XC7A50T ball** | G15 |
| **Signal type** | Digital output (3.3V LVCMOS) |

> This controls the T/R RF switch path for all 16 ADTR1107 modules. Essential for
> radar operation — determines whether the antenna array is transmitting or receiving.

**Wiring instructions:**
1. On Unit 2, find pin `IO_L2P_T0_AD8P_15` (G15)
2. Draw a wire and place a global label `M3S_VCTRL`
3. The label should connect through the hierarchy to the RF sheet where the switches are
4. No new passive components needed at the FPGA side

---

### A6. FPGA_DAC_CLOCK (DAC clock test output)

| Parameter | Value |
|---|---|
| **Net name** | `FPGA_DAC_CLOCK` |
| **FPGA ball** | **G21** |
| **FPGA pin function** | `IO_L24P_T3_16` |
| **Bank** | 16 (VCCO = 3.3V) |
| **KiCad unit** | Unit 2 |
| **Connects to** | J18 pin 1 (SMA test connector) |
| **Old XC7A50T ball** | C13 |
| **Signal type** | Clock output (3.3V LVCMOS) |

> This is a test/debug output that brings the FPGA-generated DAC clock out to an
> SMA connector (J18). Not strictly required for operation but useful for debugging.
> The actual DAC clock input on U3 (AD9708) comes from a separate source (J20/DAC_CLOCK
> net), not directly from the FPGA.

**Wiring instructions:**
1. On Unit 2, find pin `IO_L24P_T3_16` (G21)
2. Draw a wire and place a label `FPGA_DAC_CLOCK`
3. This connects to J18 SMA connector
4. No new passive components needed

**PCB note:** Route with 50-ohm controlled impedance trace to SMA connector.

---

### GROUP A SUMMARY

| # | Net Name | Ball | Bank | Unit | Destination | Resistor on Net | Priority |
|---|---|---|---|---|---|---|---|
| A1 | `ADC_D7_N` | P17 | 14 | 1 | U1 (AD9484) pin 19 via R10 (100R diff term) | R10 already exists | CRITICAL |
| A2 | `ADC_DCO_P` | W19 | 14 | 1 | U1 (AD9484) pin 50 via R12 (100R diff term) | R12 already exists | CRITICAL |
| A3 | `ADC_DCO_N` | W20 | 14 | 1 | U1 (AD9484) pin 49 via R12 (100R diff term) | R12 already exists | CRITICAL |
| A4 | `ADAR_TX_LOAD_1` | AB5 | 34 | 3 | ADAR1_0 pin N3 (TX_LOAD) | R46 (4.7k pull-down to GND, 0201) | HIGH |
| A5 | `ADAR_TR_4` | Y1 | 34 | 3 | ADAR4_0 pin N6 (TR) | R161 (22.1k pull-down to GND, 0201) | HIGH |
| A6 | `M3S_VCTRL` | G15 | 15 | 2 | RF switches (RF_SW_1..16) | none at FPGA side | HIGH |
| A7 | `FPGA_DAC_CLOCK` | G21 | 16 | 2 | J18 SMA connector | none | MEDIUM |

**New passive components needed for Group A: NONE** (all resistors already exist on the board)

---

## GROUP B: FT601 USB 3.0 INTERFACE (44 signals)

The FT601Q-B-T (U6) is the USB 3.0 to parallel FIFO bridge. On the old MainBoard,
the FT601 was placed but **never connected to the FPGA** — all its data/control pins
have no net assignment in the PCB. This is a **completely new interface** that needs
full schematic wiring + new passive components.

### Architecture

```
PC (USB 3.0) <---> FT601Q (U6) <---> FPGA (U42 Bank 16)
                    |                    |
                    | 32-bit parallel    |
                    | FIFO bus @ 100MHz  |
                    | = 3.2 Gbps max     |
                    |                    |
                    +--------------------+
```

The FT601 provides a synchronous 245 FIFO interface:
- 32-bit data bus (bidirectional)
- 4-bit byte enable
- 100 MHz clock (output from FT601 to FPGA)
- Handshake signals: TXE#, RXF#, WR#, RD#, OE#

### Bank Assignment

All FT601 signals go to **Bank 16** (VCCO = +3V3_FPGA).

Bank 16 has 50 available I/O pins, we need 44, leaving 6 spare pins in the bank.

> **IMPORTANT**: The FT601 VCCIO on this board uses VCC33 (3.3V), matching Bank 16
> VCCO. This is required for correct voltage levels on the parallel bus.

### B1. FT601 Clock Input (CRITICAL — must be on MRCC pin)

| Parameter | Value |
|---|---|
| **Net name** | `FT601_CLK` |
| **RTL port** | `ft601_clk_in` |
| **FPGA ball** | **C18** |
| **FPGA pin function** | `IO_L13P_T2_MRCC_16` |
| **Bank** | 16 (VCCO = 3.3V) |
| **KiCad unit** | Unit 2 |
| **Connects to** | U6 (FT601Q) pin 58 (CLK) |
| **Signal type** | Clock input, 100 MHz |
| **IOSTANDARD** | LVCMOS33 |

> The 100 MHz FIFO clock MUST be on an MRCC pin so it can drive global/regional
> clock buffers inside the FPGA. C18 is `IO_L13P_T2_MRCC_16`.

**Required passive components:**
- **C_FT601_CLK**: 22pF decoupling capacitor from `FT601_CLK` net to GND, placed
  close to the FPGA ball. Reduces clock ringing. (0402 or 0603)
- **R_FT601_CLK**: Optional 33 ohm series resistor at the FT601 output (U6 pin 58)
  for source termination. Recommended for 100 MHz clock.

**Wiring in KiCad:**
1. On U6 pin 58 (CLK), draw a wire to a series resistor R_FT601_CLK (33 ohm)
2. From resistor output, place label `FT601_CLK`
3. On Unit 2 of U42, find pin `IO_L13P_T2_MRCC_16` (C18)
4. Draw a wire and place the same label `FT601_CLK`
5. Place a 22pF cap (C_FT601_CLK) from this net to GND near the FPGA pin

---

### B2. FT601 32-Bit Data Bus

| Signal | FPGA Ball | Pin Function | FT601 Pin | Direction |
|---|---|---|---|---|
| `FT601_D0` | A13 | IO_L10P_T1_16 | 40 | bidir |
| `FT601_D1` | A14 | IO_L10N_T1_16 | 41 | bidir |
| `FT601_D2` | A15 | IO_L9P_T1_DQS_16 | 42 | bidir |
| `FT601_D3` | A16 | IO_L9N_T1_DQS_16 | 43 | bidir |
| `FT601_D4` | A18 | IO_L17P_T2_16 | 44 | bidir |
| `FT601_D5` | A19 | IO_L17N_T2_16 | 45 | bidir |
| `FT601_D6` | A20 | IO_L16N_T2_16 | 46 | bidir |
| `FT601_D7` | A21 | IO_L21N_T3_DQS_16 | 47 | bidir |
| `FT601_D8` | B13 | IO_L8N_T1_16 | 50 | bidir |
| `FT601_D9` | B15 | IO_L7P_T1_16 | 51 | bidir |
| `FT601_D10` | B16 | IO_L7N_T1_16 | 52 | bidir |
| `FT601_D11` | B17 | IO_L11P_T1_SRCC_16 | 53 | bidir |
| `FT601_D12` | B18 | IO_L11N_T1_SRCC_16 | 54 | bidir |
| `FT601_D13` | B20 | IO_L16P_T2_16 | 55 | bidir |
| `FT601_D14` | B21 | IO_L21P_T3_DQS_16 | 56 | bidir |
| `FT601_D15` | B22 | IO_L20N_T3_16 | 57 | bidir |
| `FT601_D16` | D14 | IO_L6P_T0_16 | 60 | bidir |
| `FT601_D17` | D15 | IO_L6N_T0_VREF_16 | 61 | bidir |
| `FT601_D18` | D16 | IO_L5N_T0_16 | 62 | bidir |
| `FT601_D19` | D17 | IO_L12P_T1_MRCC_16 | 63 | bidir |
| `FT601_D20` | D19 | IO_L14N_T2_SRCC_16 | 64 | bidir |
| `FT601_D21` | D20 | IO_L19P_T3_16 | 65 | bidir |
| `FT601_D22` | D21 | IO_L23N_T3_16 | 66 | bidir |
| `FT601_D23` | D22 | IO_L22N_T3_16 | 67 | bidir |
| `FT601_D24` | E13 | IO_L4P_T0_16 | 69 | bidir |
| `FT601_D25` | E14 | IO_L4N_T0_16 | 70 | bidir |
| `FT601_D26` | E16 | IO_L5P_T0_16 | 71 | bidir |
| `FT601_D27` | E17 | IO_L2N_T0_16 | 72 | bidir |
| `FT601_D28` | E18 | IO_L15N_T2_DQS_16 | 73 | bidir |
| `FT601_D29` | E19 | IO_L14P_T2_SRCC_16 | 74 | bidir |
| `FT601_D30` | E21 | IO_L23P_T3_16 | 75 | bidir |
| `FT601_D31` | E22 | IO_L22P_T3_16 | 76 | bidir |

**Required passive components for data bus:**
- **33 ohm series resistors (x32)**: Place one 33-ohm series resistor on each data
  line at the FT601 side (between U6 data pin and the net label). These provide
  source termination for the 100 MHz parallel bus and reduce reflections/EMI.
  - Ref des: R_FTD0 through R_FTD31
  - Value: 33 ohm
  - Package: 0402 (recommended for density) or 0201
  - Place as close to U6 as possible

> **Note on resistor arrays**: For 32 data lines, consider using 8x 4-resistor
> arrays (e.g., CRA06S series, 4x33R in 0612 package) instead of 32 discrete
> resistors to save board space. Each array has 4 independent 33-ohm resistors.

**Wiring pattern for each data line:**
```
U6 pin XX (DATA_n) --[wire]--> R_FTDn (33R) --[wire]--> label "FT601_Dn"

U42 pin (Bank 16)  --[wire]--> label "FT601_Dn"
```

---

### B3. FT601 Byte Enables

| Signal | FPGA Ball | Pin Function | FT601 Pin | Direction |
|---|---|---|---|---|
| `FT601_BE0` | C13 | IO_L8P_T1_16 | 4 | output (FPGA->FT601) |
| `FT601_BE1` | C14 | IO_L3P_T0_DQS_16 | 5 | output |
| `FT601_BE2` | C15 | IO_L3N_T0_DQS_16 | 6 | output |
| `FT601_BE3` | C17 | IO_L12N_T1_MRCC_16 | 7 | output |

**Required passive components:**
- **33 ohm series resistors (x4)**: R_FTBE0 through R_FTBE3, same as data bus.

---

### B4. FT601 Control Signals

| Signal | FPGA Ball | Pin Function | FT601 Pin | Direction | Description |
|---|---|---|---|---|---|
| `FT601_TXE_N` | C19 | IO_L13N_T2_MRCC_16 | 8 | input (FT601->FPGA) | TX FIFO not full (active low = can write) |
| `FT601_RXF_N` | C20 | IO_L19N_T3_VREF_16 | 9 | input (FT601->FPGA) | RX FIFO has data (active low = can read) |
| `FT601_WR_N` | C22 | IO_L20P_T3_16 | 11 | output (FPGA->FT601) | Write strobe (active low) |
| `FT601_RD_N` | F13 | IO_L1P_T0_16 | 12 | output (FPGA->FT601) | Read strobe (active low) |
| `FT601_OE_N` | F14 | IO_L1N_T0_16 | 13 | output (FPGA->FT601) | Output enable (active low, assert before RD_N) |
| `FT601_SIWU_N` | F15 | IO_0_16 | 10 | output (FPGA->FT601) | Send immediate / wake up |
| `FT601_RESET_N` | F16 | IO_L2P_T0_16 | 15 | output (FPGA->FT601) | Chip reset (active low) |

**Required passive components for control signals:**
- **33 ohm series resistors (x7)**: R_FT_TXE through R_FT_RST on each control line
- **10k pull-up on RESET_N**: R_FT_RST_PU (10k ohm) from FT601_RESET_N to +3V3_FPGA.
  This ensures the FT601 is not held in reset if the FPGA pins are tri-stated
  during configuration.
- **10k pull-up on SIWU_N**: R_FT_SIWU_PU (10k ohm) from FT601_SIWU_N to +3V3_FPGA.
  Default inactive (high) when FPGA is configuring.

**Wiring for control signals with pull-ups:**
```
FT601_RESET_N:
  U6 pin 15 --[33R]--> net "FT601_RESET_N" --[wire]--> U42 (F16)
                                             |
                                             +--[10k]---> +3V3_FPGA

FT601_SIWU_N:
  U6 pin 10 --[33R]--> net "FT601_SIWU_N" --[wire]--> U42 (F15)
                                            |
                                            +--[10k]---> +3V3_FPGA
```

---

### B5. FT601 Power and Decoupling (U6 side — NOT on FPGA)

The FT601 already has power connections in the PCB (AVDD, VCC33, VCCIO, VBUS, VDDA).
However, verify these decoupling capacitors exist near U6:

| Power Pin | Net | Required Decoupling |
|---|---|---|
| Pin 2 (AVDD) | AVDD | 100nF + 4.7uF to GND |
| Pin 14 (VCCIO) | VCCIO_2 | 100nF to GND |
| Pin 20 (VCC33) | VCC33_2 | 100nF to GND |
| Pin 24 (VCC33) | VCC33_3 | 100nF to GND |
| Pin 28 (VDDA) | VDDA | 100nF + 4.7uF to GND (analog supply, critical) |
| Pin 38 (VCC33) | VCC33 | 100nF to GND |
| Pin 49 (VCCIO) | VCCIO_3 | 100nF to GND |
| Pin 59 (VCCIO) | VCCIO_4 | 100nF to GND |
| Pin 68 (VCCIO) | VCCIO | 100nF to GND |

> These should already be in the Eagle-imported design. Verify they exist before
> adding duplicates.

### B6. FT601 USB Connector Connections

The FT601 USB side connections (USB3.0 SuperSpeed differential pairs, VBUS, etc.)
are separate from the FPGA interface and should already be routed to a USB connector.
This guide only covers the FPGA-side parallel FIFO interface.

---

### GROUP B PASSIVE COMPONENT SUMMARY

| Component | Qty | Value | Package | Purpose |
|---|---|---|---|---|
| Series resistors (data bus) | 32 | 33 ohm | 0402 | Source termination for FT601_D[0:31] |
| Series resistors (byte enable) | 4 | 33 ohm | 0402 | Source termination for FT601_BE[0:3] |
| Series resistors (control) | 7 | 33 ohm | 0402 | Source termination for control signals |
| Series resistor (clock) | 1 | 33 ohm | 0402 | Source termination for FT601_CLK |
| Pull-up (RESET_N) | 1 | 10k ohm | 0402 | Hold reset high during FPGA config |
| Pull-up (SIWU_N) | 1 | 10k ohm | 0402 | Hold SIWU inactive during FPGA config |
| Decoupling cap (clock) | 1 | 22 pF | 0402 | Clock input filtering |
| **TOTAL NEW PASSIVES** | **47** | | | |

> **Alternative to 44 discrete series resistors**: Use 11x resistor arrays
> (4x33R each = 44 resistors). Example part: Bourns CAY16-33R0F4 (4x33R, 0612).
> This reduces component count from 44 to 11 and saves significant board space.

---

## GROUP C: STM32 CONTROL GPIO (4 signals)

The RTL (`radar_system_top.v`) expects these control inputs from the STM32. They
were NOT on the old MainBoard PCB — they are currently only in the dev board XDC.
However, the RTL uses them for radar scan control, so they should be added for the
production board.

### Where to Connect on STM32

The STM32F746ZGT7 (U2) has the DIG_[0:7] bus already connected (PD8-PD15).
These 4 new control signals should use other available STM32 GPIO pins. Check which
STM32 pins are available (not already assigned in schematic).

Suggested STM32 pins (from LQFP-144 package, verify availability):
- PE0 (pin 141) for stm32_new_chirp
- PE1 (pin 142) for stm32_new_elevation
- PE2 (pin 1) for stm32_new_azimuth
- PE3 (pin 2) for stm32_mixers_enable

> **IMPORTANT**: Verify these STM32 pins are actually unused in the current
> schematic before assigning. The pin suggestions above are preliminary.

### Signal Assignments

These signals are 3.3V digital, so they go to a 3.3V FPGA bank. Bank 15 has
available pins.

| Signal | FPGA Ball | Pin Function | Bank | Unit | Direction | Description |
|---|---|---|---|---|---|---|
| `STM32_NEW_CHIRP` | J16 | IO_0_15 | 15 | 2 | input | STM32 triggers new chirp |
| `STM32_NEW_ELEVATION` | M17 | IO_25_15 | 15 | 2 | input | STM32 signals new elevation step |
| `STM32_NEW_AZIMUTH` | L14 | IO_L22P_T3_A17_15 | 15 | 2 | input | STM32 signals new azimuth step |
| `STM32_MIXERS_ENABLE` | J17 | IO_L21N_T3_DQS_A18_15 | 15 | 2 | input | STM32 enables mixer circuits |

**Required passive components:**
- **10k pull-down resistors (x4)**: One on each signal, to GND. Ensures defined
  logic level (low/inactive) when STM32 or FPGA are in reset/unconfigured state.
  - R_NC (10k) — STM32_NEW_CHIRP to GND
  - R_NE (10k) — STM32_NEW_ELEVATION to GND
  - R_NA (10k) — STM32_NEW_AZIMUTH to GND
  - R_ME (10k) — STM32_MIXERS_ENABLE to GND

**Wiring pattern:**
```
STM32 (U2) pin XX --[wire]--> label "STM32_NEW_CHIRP"
                                        |
                                        +--[10k]---> GND

U42 pin (Bank 15)  --[wire]--> label "STM32_NEW_CHIRP"
```

**PCB note:** These are slow control signals (<1 MHz). No impedance control needed.
Route with standard traces, keep reasonably short (<50mm).

---

### GROUP C PASSIVE COMPONENT SUMMARY

| Component | Qty | Value | Package | Purpose |
|---|---|---|---|---|
| Pull-down resistors | 4 | 10k ohm | 0402 | Default inactive state during reset |
| **TOTAL** | **4** | | | |

---

## GROUP D: ADDITIONAL PASSIVE COMPONENTS (FPGA support)

These are passive components needed for the FPGA itself (beyond signal connections),
some of which may already be on the board from the old design.

### D1. FPGA Decoupling Capacitors

The XC7A200T requires more decoupling than the XC7A50T due to higher power
consumption. Xilinx UG483 recommends per-bank and per-rail decoupling.

**VCCINT (1.0V) decoupling — verify/add near FPGA:**
| Component | Value | Qty | Notes |
|---|---|---|---|
| Bulk caps | 47uF, 6.3V | 2 | Tantalum or ceramic, near FPGA |
| Decoupling | 4.7uF, 6.3V | 4 | Ceramic, distributed around BGA |
| Decoupling | 100nF | 14 | One per VCCINT ball (may share nearby pairs) |

**VCCAUX (1.8V) decoupling:**
| Component | Value | Qty | Notes |
|---|---|---|---|
| Bulk cap | 47uF, 6.3V | 1 | Near FPGA |
| Decoupling | 4.7uF | 2 | Ceramic |
| Decoupling | 100nF | 5 | One per VCCAUX ball |

**VCCO per bank decoupling (already partially on board from old design):**
| Bank | VCCO | 100nF caps needed | 4.7uF caps needed |
|---|---|---|---|
| Bank 13 | +3V3_FPGA | 5 (one per VCCO pin) | 1 |
| Bank 14 | +3V3_FPGA | 6 | 1 |
| Bank 15 | +3V3_FPGA | 6 | 1 |
| Bank 16 | +3V3_FPGA | 6 | 1 |
| Bank 34 | +1V8_FPGA | 6 | 1 |
| Bank 35 | +1V8_FPGA | 6 | 1 |

> **CHECK FIRST**: The old XC7A50T design likely had decoupling capacitors already
> placed. Many of these may still be on the PCB. Only add what's missing. The main
> concern is that the XC7A200T has more VCCO pins (especially Bank 16 which was
> unused before) so additional decoupling may be needed.

### D2. VREF Pin Handling

Bank 14 has a VREF pin (`IO_L19N_T3_A09_D25_VREF_14` on R14) which is used for
`ADC_D4_N`. This is fine — the VREF function is only active when the bank uses
an I/O standard that requires a reference voltage (like SSTL or HSTL). Since we're
using LVCMOS33/LVDS_25, the VREF function is not active and the pin works as a
normal I/O.

Bank 16 has `IO_L6N_T0_VREF_16` (D15) assigned to `FT601_D17`. Same situation —
LVCMOS33 doesn't need VREF, so it's fine as a normal I/O.

### D3. Bank 16 VCCO Connections (NEW — was unused)

Bank 16 was completely unused on the old design. Verify that ALL 6 VCCO_16 pins
are connected to +3V3_FPGA in the schematic:

| Ball | Function | Must connect to |
|---|---|---|
| A17 | VCCO_16 | +3V3_FPGA |
| B14 | VCCO_16 | +3V3_FPGA |
| C21 | VCCO_16 | +3V3_FPGA |
| D18 | VCCO_16 | +3V3_FPGA |
| E15 | VCCO_16 | +3V3_FPGA |
| F22 | VCCO_16 | +3V3_FPGA |

> These should already be connected from the power pin wiring (fpga_power.py script
> connected all VCCO pins). Verify in schematic that these have +3V3_FPGA global labels.

---

## WIRING ORDER (RECOMMENDED)

Follow this order for minimum rework:

### Phase 1: Critical Missing Signals (do first)
1. `ADC_D7_N` (P17) — label only, no new components
2. `ADC_DCO_P` (W19) — label only
3. `ADC_DCO_N` (W20) — label only
4. `ADAR_TX_LOAD_1` (AB5) — label only
5. `ADAR_TR_4` (Y1) — label only
6. `M3S_VCTRL` (G15) — global label
7. `FPGA_DAC_CLOCK` (G21) — label only

### Phase 2: STM32 Control GPIO (simple, 4 signals)
8-11. Wire STM32_NEW_CHIRP, _NEW_ELEVATION, _NEW_AZIMUTH, _MIXERS_ENABLE
- Add 4x 10k pull-down resistors in schematic

### Phase 3: FT601 Interface (largest — 44 signals + 47 passives)
12. Place FT601 clock connection with series R and decoupling cap
13. Wire FT601 data bus (32 lines) — add series resistors
14. Wire FT601 byte enables (4 lines) — add series resistors
15. Wire FT601 control signals (7 lines) — add series resistors + pull-ups
16. Verify FT601 power connections and decoupling

### Phase 4: Verify & Clean Up
17. Verify Bank 16 VCCO connections
18. Check decoupling capacitors (add missing ones)
19. Place no-connect flags on all unused FPGA I/O pins
20. Run ERC

---

## COMPLETE NEW PASSIVE COMPONENT BOM

| Ref | Value | Qty | Package | Purpose |
|---|---|---|---|---|
| R_FTD0..R_FTD31 | 33 ohm | 32 | 0402 | FT601 data bus series termination |
| R_FTBE0..R_FTBE3 | 33 ohm | 4 | 0402 | FT601 byte enable series termination |
| R_FT_TXE, etc. | 33 ohm | 7 | 0402 | FT601 control series termination |
| R_FT_CLK | 33 ohm | 1 | 0402 | FT601 clock series termination |
| R_FT_RST_PU | 10k ohm | 1 | 0402 | FT601 RESET_N pull-up to 3.3V |
| R_FT_SIWU_PU | 10k ohm | 1 | 0402 | FT601 SIWU_N pull-up to 3.3V |
| R_NC, R_NE, R_NA, R_ME | 10k ohm | 4 | 0402 | STM32 GPIO pull-downs |
| C_FT_CLK | 22 pF | 1 | 0402 | FT601 clock decoupling |
| **TOTAL** | | **51** | | |

> **Resistor array alternative**: Replace 44 discrete 33R resistors with 11x
> CAY16-33R0F4 (4x33R array, 0612 package). Total component count drops to 18.

---

## RTL PORT-TO-SCHEMATIC NET NAME MAPPING

For updating the XDC constraints file after wiring:

| RTL Port | Schematic Net Name | FPGA Ball | IOSTANDARD |
|---|---|---|---|
| `adc_d_n[7]` | `ADC_D7_N` | P17 | LVDS_25 |
| `adc_dco_p` | `ADC_DCO_P` | W19 | LVDS_25 |
| `adc_dco_n` | `ADC_DCO_N` | W20 | LVDS_25 |
| `adar_tx_load_1` | `ADAR_TX_LOAD_1` | AB5 | LVCMOS18 |
| `adar_tr_4` | `ADAR_TR_4` | Y1 | LVCMOS18 |
| `fpga_rf_switch` | `M3S_VCTRL` | G15 | LVCMOS33 |
| `dac_clk` (if routed to J18) | `FPGA_DAC_CLOCK` | G21 | LVCMOS33 |
| `stm32_new_chirp` | `STM32_NEW_CHIRP` | J16 | LVCMOS33 |
| `stm32_new_elevation` | `STM32_NEW_ELEVATION` | M17 | LVCMOS33 |
| `stm32_new_azimuth` | `STM32_NEW_AZIMUTH` | L14 | LVCMOS33 |
| `stm32_mixers_enable` | `STM32_MIXERS_ENABLE` | J17 | LVCMOS33 |
| `ft601_clk_in` | `FT601_CLK` | C18 | LVCMOS33 |
| `ft601_data[0]` | `FT601_D0` | A13 | LVCMOS33 |
| `ft601_data[1]` | `FT601_D1` | A14 | LVCMOS33 |
| `ft601_data[2]` | `FT601_D2` | A15 | LVCMOS33 |
| `ft601_data[3]` | `FT601_D3` | A16 | LVCMOS33 |
| `ft601_data[4]` | `FT601_D4` | A18 | LVCMOS33 |
| `ft601_data[5]` | `FT601_D5` | A19 | LVCMOS33 |
| `ft601_data[6]` | `FT601_D6` | A20 | LVCMOS33 |
| `ft601_data[7]` | `FT601_D7` | A21 | LVCMOS33 |
| `ft601_data[8]` | `FT601_D8` | B13 | LVCMOS33 |
| `ft601_data[9]` | `FT601_D9` | B15 | LVCMOS33 |
| `ft601_data[10]` | `FT601_D10` | B16 | LVCMOS33 |
| `ft601_data[11]` | `FT601_D11` | B17 | LVCMOS33 |
| `ft601_data[12]` | `FT601_D12` | B18 | LVCMOS33 |
| `ft601_data[13]` | `FT601_D13` | B20 | LVCMOS33 |
| `ft601_data[14]` | `FT601_D14` | B21 | LVCMOS33 |
| `ft601_data[15]` | `FT601_D15` | B22 | LVCMOS33 |
| `ft601_data[16]` | `FT601_D16` | D14 | LVCMOS33 |
| `ft601_data[17]` | `FT601_D17` | D15 | LVCMOS33 |
| `ft601_data[18]` | `FT601_D18` | D16 | LVCMOS33 |
| `ft601_data[19]` | `FT601_D19` | D17 | LVCMOS33 |
| `ft601_data[20]` | `FT601_D20` | D19 | LVCMOS33 |
| `ft601_data[21]` | `FT601_D21` | D20 | LVCMOS33 |
| `ft601_data[22]` | `FT601_D22` | D21 | LVCMOS33 |
| `ft601_data[23]` | `FT601_D23` | D22 | LVCMOS33 |
| `ft601_data[24]` | `FT601_D24` | E13 | LVCMOS33 |
| `ft601_data[25]` | `FT601_D25` | E14 | LVCMOS33 |
| `ft601_data[26]` | `FT601_D26` | E16 | LVCMOS33 |
| `ft601_data[27]` | `FT601_D27` | E17 | LVCMOS33 |
| `ft601_data[28]` | `FT601_D28` | E18 | LVCMOS33 |
| `ft601_data[29]` | `FT601_D29` | E19 | LVCMOS33 |
| `ft601_data[30]` | `FT601_D30` | E21 | LVCMOS33 |
| `ft601_data[31]` | `FT601_D31` | E22 | LVCMOS33 |
| `ft601_be[0]` | `FT601_BE0` | C13 | LVCMOS33 |
| `ft601_be[1]` | `FT601_BE1` | C14 | LVCMOS33 |
| `ft601_be[2]` | `FT601_BE2` | C15 | LVCMOS33 |
| `ft601_be[3]` | `FT601_BE3` | C17 | LVCMOS33 |
| `ft601_txe` (active low) | `FT601_TXE_N` | C19 | LVCMOS33 |
| `ft601_rxf` (active low) | `FT601_RXF_N` | C20 | LVCMOS33 |
| `ft601_wr_n` | `FT601_WR_N` | C22 | LVCMOS33 |
| `ft601_rd_n` | `FT601_RD_N` | F13 | LVCMOS33 |
| `ft601_oe_n` | `FT601_OE_N` | F14 | LVCMOS33 |
| `ft601_siwu_n` | `FT601_SIWU_N` | F15 | LVCMOS33 |
| (reset control) | `FT601_RESET_N` | F16 | LVCMOS33 |

---

## SIGNALS NOT WIRED (INTENTIONAL)

These RTL ports exist but do NOT need MainBoard PCB connections:

| RTL Port | Reason |
|---|---|
| `reset_n` | Not on old board. FPGA has internal power-on-reset. PROGRAM_B handles config reset. |
| `clk_120m_dac` | Dev-board only. MainBoard DAC clock comes from external oscillator via J20, not FPGA. |
| `ft601_clk_out` | Internal RTL use only (optional clock output for debug) |
| `ft601_srb[1:0]`, `ft601_swb[1:0]` | FT601 session status bits — optional, not needed for basic operation |
| `ft601_txe_n`, `ft601_rxf_n` (duplicate active-high/low) | RTL has both polarities — only wire the active-low versions |
| `current_elevation[5:0]` | Debug output — route to test header or STM32 GPIO if desired, not required |
| `current_azimuth[5:0]` | Debug output — same |
| `current_chirp[5:0]` | Debug output — same |
| `new_chirp_frame` | Debug output — same |
| `dbg_doppler_data[31:0]` | Debug only — 32 bits, impractical to route out |
| `dbg_doppler_valid` | Debug only |
| `dbg_doppler_bin[4:0]` | Debug only |
| `dbg_range_bin[5:0]` | Debug only |
| `system_status[3:0]` | Debug LEDs on dev board — route to LEDs or test header if desired |

> If you want `system_status[3:0]` visible, you could add 4 LEDs with 1k series
> resistors to spare Bank 13 pins. This is optional.
