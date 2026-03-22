# AERIS-10 Board-Day Smoke Test Guide

**Target:** TE0713-03 (XC7A200T-2FBG484) + TE0701-06 carrier + UMFT601X-B FMC LPC  
**Tag:** `v0.1.9-timing-fix` (commit `f8dd6ba`)  
**Date:** 2026-03-22

---

## 1. Bitstream Inventory

| # | Bitstream File | Tag | Purpose | WNS | Use When |
|---|---------------|-----|---------|-----|----------|
| 1 | `docs/artifacts/te0713-te0701-heartbeat-2026-03-21.bit` | v0.1.7 | First-power heartbeat only | +17.863 ns | **First ever power-on** — verifies JTAG, clock, config |
| 2 | `docs/artifacts/te0713-te0701-umft601x-dev-2026-03-21.bit` | v0.1.8 | FT601 USB data path (pre-timing-fix) | +0.059 ns | Fallback if v0.1.9 build not available |
| 3 | `docs/artifacts/te0713-te0701-umft601x-dev-2026-03-22.bit` | v0.1.9 | FT601 USB + timing fix + self-test | +0.349 ns | **Primary smoke test image** |

### Building Bitstream #3 (v0.1.9)

The remote build server (`livepeerservice.ddns.net`) runs Vivado 2025.2. When online:

```bash
# SSH to build server
ssh -i ~/.ssh/id_ed25519 jason-stone@livepeerservice.ddns.net

# Pull latest code
cd /path/to/PLFM_RADAR && git pull

# Run Vivado batch build
/mnt/bcache/Xilinx/Vivado/2025.2/Vivado/bin/vivado -mode batch \
  -source 9_Firmware/9_2_FPGA/scripts/build_te0713_umft601x_dev.tcl

# Bitstream output:
#   9_Firmware/9_2_FPGA/vivado_te0713_umft601x_dev/aeris10_te0713_umft601x_dev.runs/impl_1/*.bit
# Timing report:
#   9_Firmware/9_2_FPGA/vivado_te0713_umft601x_dev/reports/timing_summary.rpt
```

Or locally if Vivado 2025.2 is installed:

```bash
cd 9_Firmware/9_2_FPGA
vivado -mode batch -source scripts/build_te0713_umft601x_dev.tcl
```

Copy the `.bit` file to `docs/artifacts/te0713-te0701-umft601x-dev-2026-03-22.bit`.

---

## 2. Hardware Setup

### 2A. Board Assembly

1. Mount **TE0713-03** module onto **TE0701-06** carrier (B2B connectors)
2. Plug **UMFT601X-B** into **TE0701 J10** (FMC LPC connector)
3. Connect **USB 3.0 Micro-B cable** from UMFT601X-B to host PC
4. Connect **Trenz USB cable** for JTAG programming

### 2B. Jumper Configuration

**TE0701-06 Carrier:**
- VIOTB = **3.3V** (bank voltage for FMC LPC I/O)

**UMFT601X-B Module:**

| Jumper | Setting | Purpose |
|--------|---------|---------|
| JP1 | Open | Default |
| JP2 | Pins 2-3 | Default |
| JP3 | Open | Default |
| JP4 | Pins 2-3 | 245 Sync FIFO mode |
| JP5 | Pins 2-3 | 245 Sync FIFO mode |
| JP6 | Short | 3.3V VCCIO |

### 2C. Power

- TE0701 powered via USB or external 5V supply
- Verify 3.3V rail stable before programming

---

## 3. Smoke Test Procedure

### Stage 1: First Power — Heartbeat Image (LOWEST RISK)

**Purpose:** Verify FPGA configures, clock runs, basic I/O works.  
**Bitstream:** `docs/artifacts/te0713-te0701-heartbeat-2026-03-21.bit`

```
Step 1.1  Open Vivado Hardware Manager
          Vivado > Open Hardware Manager > Open Target > Auto Connect

Step 1.2  Verify JTAG enumeration
          Should see: xc7a200t (TE0713)
          If NOT: check B2B seating, power, JTAG cable

Step 1.3  Program heartbeat bitstream
          Right-click device > Program Device
          Select: docs/artifacts/te0713-te0701-heartbeat-2026-03-21.bit
          Click "Program"

Step 1.4  Verify heartbeat
          - LED/scope on heartbeat output should toggle at ~3 Hz
          - gpio1 should go HIGH after ~328 µs (POR complete)
          ✅ PASS: FPGA configured, clock running, reset released
          ❌ FAIL: Check clock source, power rails, B2B connector
```

### Stage 2: FT601 USB Integration — Dev Image

**Purpose:** Verify FT601 USB 3.0 data path, host commands, synthetic data streaming.  
**Bitstream:** `docs/artifacts/te0713-te0701-umft601x-dev-2026-03-22.bit` (v0.1.9 — timing fix + real self-test)  
**Fallback:** `docs/artifacts/te0713-te0701-umft601x-dev-2026-03-21.bit` (v0.1.8 — hardwired self-test)

```
Step 2.1  Program FT601 dev bitstream
          Same procedure as Step 1.3 with the dev bitstream

Step 2.2  Verify heartbeat still works
          - ft601_gpio0 should toggle at ~6 Hz (bit 24 of counter at 100 MHz)
          - ft601_gpio1 = HIGH (reset complete)
          ✅ If no toggle: FPGA not running. Reprogram.

Step 2.3  Check FT601 USB enumeration on host PC
          Linux:   lsusb | grep 0403:601f
          macOS:   system_profiler SPUSBDataType | grep -A5 "FT601"
          Windows: Device Manager > USB > "FTDI FT601 USB 3.0 Bridge"
          ✅ PASS: FT601 enumerated as USB 3.0 device
          ❌ FAIL: Check USB cable (must be 3.0), JP4/JP5 (Sync FIFO mode),
                   VIOTB jumper (3.3V), FMC connector seating

Step 2.4  Install host-side dependencies
          cd 9_Firmware/9_3_GUI
          pip install -r requirements_dashboard.txt
          # For live hardware: also install ftd3xx
          # Linux: pip install ftd3xx (+ FTDI D3XX driver)
          # macOS: download from ftdichip.com
          # Windows: pip install ftd3xx

Step 2.5  Run automated smoke test
          cd 9_Firmware/9_3_GUI
          python smoke_test.py --live

          Expected output:
            [INFO] Connecting to FT601...
            [INFO] Sending self-test trigger (opcode 0x30)...
            [INFO] Reading self-test results...
            [INFO] ┌──────────────────────────────────┐
            [INFO] │ FPGA Self-Test Results            │
            [INFO] ├──────────────────────────────────┤
            [INFO] │ BRAM Write/Read Pattern    PASS  │
            [INFO] │ CIC Integrator Arithmetic  PASS  │
            [INFO] │ FFT Butterfly Arithmetic   PASS  │
            [INFO] │ Saturating Add (MTI)       PASS  │
            [INFO] │ ADC Raw Data Capture       PASS  │  ← may FAIL if ADC not connected
            [INFO] └──────────────────────────────────┘

          Exit code 0 = all passed, 1 = failure, 2 = comms error

          NOTE: Test 4 (ADC capture) will likely FAIL or timeout on
          the dev wrapper because ADC inputs are not connected.
          This is expected. The other 4 tests should all PASS.

Step 2.6  (Optional) Capture ADC raw data
          python smoke_test.py --live --adc-dump adc_raw.npy
          # Saves 256 raw ADC samples to adc_raw.npy for offline analysis

Step 2.7  Verify data streaming with dashboard
          python radar_dashboard.py --live
          # Should show real-time range profile with synthetic test pattern
          # Packet header: 0xAE10xxxx (xxxx = packet type)
          # Range data: counter XOR 0xA5A5 pattern

Step 2.8  Verify host commands
          In the dashboard command panel:
            - Send "Status Request" (0xFF) → should get status response packet
            - Send "Stream Control" (0x04) with value 0x07 → enables all 3 streams
            - Send "Stream Control" (0x04) with value 0x00 → stops all streams
          ✅ PASS: Commands acknowledged, streams start/stop on demand
```

### Stage 3: Manual Verification (if no ftd3xx driver)

If the D3XX Python driver is not available, use the FTDI D3XX utility:

```
Step 3.1  Download FT600/FT601 Data Streamer utility from ftdichip.com

Step 3.2  Open utility, select FT601 device

Step 3.3  Read channel 0 — should see continuous data with pattern:
          Bytes: AE 10 01 xx ... (range profile packets)

Step 3.4  Write channel 0 — send host command:
          FF 00 00 00  (status request opcode 0xFF)
          Should trigger a status response packet in the read stream
```

---

## 4. What Each Signal Tells You

| Observable | Expected | Diagnosis if wrong |
|-----------|----------|-------------------|
| JTAG enumerates xc7a200t | Yes | Check power, B2B connector, JTAG cable |
| ft601_gpio0 toggles ~6 Hz | Yes | FPGA not running or clock dead |
| ft601_gpio1 = HIGH | Yes (after 328 µs) | POR not completing — clock issue |
| FT601 USB enumeration | VID 0x0403, PID 0x601F | USB cable, JP4/JP5, VIOTB |
| Self-test bits 0-3 PASS | Yes | Logic error in FPGA — check timing report |
| Self-test bit 4 (ADC) | FAIL expected on dev board | Normal — ADC not connected in dev wrapper |
| Read data = 0xAE10xxxx packets | Yes | USB data path working |
| Status response after 0xFF | Yes | Host command path working |

---

## 5. Relevant Source Files

| File | Purpose |
|------|---------|
| `9_Firmware/9_2_FPGA/radar_system_top_te0713_umft601x_dev.v` | Dev wrapper top module |
| `9_Firmware/9_2_FPGA/usb_data_interface.v` | FT601 USB data interface |
| `9_Firmware/9_2_FPGA/fpga_self_test.v` | 5-subsystem self-test controller |
| `9_Firmware/9_2_FPGA/constraints/te0713_te0701_umft601x.xdc` | Pin/timing constraints |
| `9_Firmware/9_2_FPGA/scripts/build_te0713_umft601x_dev.tcl` | Vivado batch build script |
| `9_Firmware/9_3_GUI/smoke_test.py` | Host-side self-test runner |
| `9_Firmware/9_3_GUI/radar_dashboard.py` | Real-time data visualization |
| `9_Firmware/9_3_GUI/radar_protocol.py` | Protocol layer (parsing, commands) |

---

## 6. Known Limitations (Dev Wrapper)

1. **No real radar signal chain** — dev wrapper generates synthetic counter/XOR test data
2. **ADC self-test will fail** — ADC inputs tied to 0 in dev wrapper (no AD9484 connected)
3. **Self-test ADC limitation** — `fpga_self_test.v` is fully wired in v0.1.9. Tests 0-3 (BRAM, CIC, FFT, MTI arithmetic) run on real hardware. Test 4 (ADC capture) will timeout/fail because ADC inputs are tied to `16'd0` in the dev wrapper (no AD9484 connected). This is expected — all 4 logic tests should PASS.
4. **Single clock domain** — dev wrapper runs everything on `ft601_clk_in` (100 MHz). Production design has separate 400 MHz ADC clock.

---

## 7. Troubleshooting

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| JTAG doesn't enumerate | Power, B2B connector | Reseat module, check power LED |
| Configures but no heartbeat | Clock not reaching FPGA | Check TE0701 clock source, oscillator |
| FT601 not enumerating | Wrong FIFO mode | Verify JP4=2-3, JP5=2-3 (245 Sync FIFO) |
| FT601 enumerates as USB 2.0 | Bad cable or port | Use USB 3.0 cable + USB 3.0 port |
| Data reads return zeros | ft601_txe stuck high | Check VIOTB=3.3V, FMC connector pins |
| Intermittent data errors | Timing marginal | Use v0.1.9 bitstream (timing fix), not v0.1.8 |
| smoke_test.py timeout | D3XX driver issue | Verify `import ftd3xx` works in Python |
