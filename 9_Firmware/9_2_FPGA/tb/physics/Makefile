# =============================================================================
# Makefile — AERIS-10 Physics-Aware Test Suite
#
# Builds and runs physics testbenches using Icarus Verilog (iverilog/vvp).
# All builds use -DSIMULATION to enable behavioral models for Xilinx primitives.
#
# Usage:
#   make all        — build and run all physics testbenches
#   make ddc        — build and run DDC physics testbench
#   make mti        — build and run MTI physics testbench
#   make doppler    — build and run Doppler physics testbench
#   make cfar       — build and run CFAR physics testbench
#   make range      — build and run range compression physics testbench
#   make e2e        — build and run end-to-end physics testbench
#   make compile    — compile only (no run)
#   make clean      — remove build artifacts
# =============================================================================

# Icarus Verilog
IVERILOG := iverilog
VVP      := vvp
IVFLAGS  := -DSIMULATION -g2012

# Directories (relative to this Makefile)
TB_DIR   := .
RTL_DIR  := ../..

# ---- Source file sets per testbench ----

DDC_SRCS := \
	$(TB_DIR)/tb_ddc_physics.v \
	$(RTL_DIR)/ddc_400m.v \
	$(RTL_DIR)/nco_400m_enhanced.v \
	$(RTL_DIR)/cic_decimator_4x_enhanced.v \
	$(RTL_DIR)/fir_lowpass.v \
	$(RTL_DIR)/cdc_modules.v

MTI_SRCS := \
	$(TB_DIR)/tb_mti_physics.v \
	$(RTL_DIR)/mti_canceller.v

DOPPLER_SRCS := \
	$(TB_DIR)/tb_doppler_physics.v \
	$(TB_DIR)/radar_target_model.v \
	$(RTL_DIR)/doppler_processor.v \
	$(RTL_DIR)/xfft_16.v \
	$(RTL_DIR)/xfft_32.v \
	$(RTL_DIR)/fft_engine.v

CFAR_SRCS := \
	$(TB_DIR)/tb_cfar_physics.v \
	$(RTL_DIR)/cfar_ca.v

RANGE_SRCS := \
	$(TB_DIR)/tb_range_compression_physics.v \
	$(RTL_DIR)/matched_filter_processing_chain.v

E2E_SRCS := \
	$(TB_DIR)/tb_end_to_end_physics.v \
	$(RTL_DIR)/matched_filter_processing_chain.v \
	$(RTL_DIR)/range_bin_decimator.v \
	$(RTL_DIR)/mti_canceller.v \
	$(RTL_DIR)/doppler_processor.v \
	$(RTL_DIR)/xfft_16.v \
	$(RTL_DIR)/xfft_32.v \
	$(RTL_DIR)/fft_engine.v \
	$(RTL_DIR)/cfar_ca.v

# ---- Output VVP files ----
DDC_VVP     := tb_ddc_physics.vvp
MTI_VVP     := tb_mti_physics.vvp
DOPPLER_VVP := tb_doppler_physics.vvp
CFAR_VVP    := tb_cfar_physics.vvp
RANGE_VVP   := tb_range_compression_physics.vvp
E2E_VVP     := tb_end_to_end_physics.vvp

ALL_VVP := $(DDC_VVP) $(MTI_VVP) $(DOPPLER_VVP) $(CFAR_VVP) $(RANGE_VVP) $(E2E_VVP)

# ---- Phony targets ----
.PHONY: all compile clean ddc mti doppler cfar range e2e help

help:
	@echo "AERIS-10 Physics Test Suite"
	@echo ""
	@echo "Targets:"
	@echo "  all      — build and run ALL physics testbenches"
	@echo "  ddc      — DDC (D1-D7): tone response, I/Q orthogonality, image rejection"
	@echo "  mti      — MTI (M1-M7): clutter suppression, blind speeds, saturation"
	@echo "  doppler  — Doppler (P1-P10): staggered-PRF bug, velocity mapping"
	@echo "  cfar     — CFAR (C1-C7): detection, false alarm, GO/SO modes"
	@echo "  range    — Range compression (R1-R6): peak location, resolution, SNR"
	@echo "  e2e      — End-to-end (E1-E4): full chain range+Doppler+detection"
	@echo "  compile  — compile all (no run)"
	@echo "  clean    — remove .vvp and .vcd files"

all: ddc mti doppler cfar range e2e
	@echo ""
	@echo "========================================"
	@echo "  All physics testbenches complete."
	@echo "========================================"

compile: $(ALL_VVP)
	@echo "All testbenches compiled successfully."

# ---- Individual targets: compile + run ----

ddc: $(DDC_VVP)
	@echo "--- Running DDC physics testbench ---"
	$(VVP) $(DDC_VVP)

mti: $(MTI_VVP)
	@echo "--- Running MTI physics testbench ---"
	$(VVP) $(MTI_VVP)

doppler: $(DOPPLER_VVP)
	@echo "--- Running Doppler physics testbench ---"
	$(VVP) $(DOPPLER_VVP)

cfar: $(CFAR_VVP)
	@echo "--- Running CFAR physics testbench ---"
	$(VVP) $(CFAR_VVP)

range: $(RANGE_VVP)
	@echo "--- Running range compression physics testbench ---"
	$(VVP) $(RANGE_VVP)

e2e: $(E2E_VVP)
	@echo "--- Running end-to-end physics testbench ---"
	$(VVP) $(E2E_VVP)

# ---- Compile rules ----

$(DDC_VVP): $(DDC_SRCS)
	$(IVERILOG) $(IVFLAGS) -o $@ $(DDC_SRCS)

$(MTI_VVP): $(MTI_SRCS)
	$(IVERILOG) $(IVFLAGS) -o $@ $(MTI_SRCS)

$(DOPPLER_VVP): $(DOPPLER_SRCS)
	$(IVERILOG) $(IVFLAGS) -o $@ $(DOPPLER_SRCS)

$(CFAR_VVP): $(CFAR_SRCS)
	$(IVERILOG) $(IVFLAGS) -o $@ $(CFAR_SRCS)

$(RANGE_VVP): $(RANGE_SRCS)
	$(IVERILOG) $(IVFLAGS) -o $@ $(RANGE_SRCS)

$(E2E_VVP): $(E2E_SRCS)
	$(IVERILOG) $(IVFLAGS) -o $@ $(E2E_SRCS)

# ---- Cleanup ----
clean:
	rm -f $(ALL_VVP) *.vcd
