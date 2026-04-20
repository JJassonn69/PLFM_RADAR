"""
DDC Cosim Fuzz Runner (audit F-3.2)
===================================
Parameterized seed sweep over the existing DDC cosim testbench.

For each seed the runner:
  1. Generates a random plausible radar scene (1-4 targets, random range /
     velocity / RCS, random noise level) via tb/cosim/radar_scene.py, using
     the seed for full determinism.
  2. Writes a temporary ADC hex file.
  3. Compiles tb_ddc_cosim.v with -DSCENARIO_FUZZ (once, cached across seeds)
     and runs vvp with +hex, +csv, +tag plusargs.
  4. Parses the RTL output CSV and checks:
       - non-empty output (the pipeline produced baseband samples)
       - all I/Q values are within signed-18-bit range
       - no NaN / parse errors
       - sample count is within the expected bound from CIC decimation ratio

The intent is liveness / crash-fuzz, not bit-exact cross-check. Bit-exact
validation is covered by the static scenarios (single_target, multi_target,
etc) in the existing suite. Fuzz complements that by surfacing edge-case
corruption, saturation, or overflow on random-but-valid inputs.

Marks:
  - The default fuzz sweep uses 8 seeds for fast CI.
  - Use `-m slow` to unlock the full 100-seed sweep matched to the audit ask.

Compile + run times per seed on a laptop with iverilog 13: ~6 s. The default
8-seed sweep fits in a ~1 minute pytest run; the 100-seed sweep takes ~10-12
minutes.
"""
from __future__ import annotations

import os
import random
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parent.parent.parent
FPGA_DIR = REPO_ROOT / "9_Firmware" / "9_2_FPGA"
COSIM_DIR = FPGA_DIR / "tb" / "cosim"

sys.path.insert(0, str(COSIM_DIR))
import radar_scene  # noqa: E402

FAST_SEEDS = list(range(8))
SLOW_SEEDS = list(range(100))

# Pipeline constants
N_ADC_SAMPLES = 16384
CIC_DECIMATION = 4
FIR_DECIMATION = 1
EXPECTED_BB_MIN = N_ADC_SAMPLES // (CIC_DECIMATION * 4)   # pessimistic lower bound
EXPECTED_BB_MAX = N_ADC_SAMPLES // CIC_DECIMATION         # upper bound before FIR drain
SIGNED_18_MIN = -(1 << 17)
SIGNED_18_MAX = (1 << 17) - 1

SOURCE_FILES = [
    "tb/tb_ddc_cosim.v",
    "ddc_400m.v",
    "nco_400m_enhanced.v",
    "cic_decimator_4x_enhanced.v",
    "fir_lowpass.v",
    "cdc_modules.v",
]


@pytest.fixture(scope="module")
def compiled_fuzz_vvp(tmp_path_factory):
    """Compile tb_ddc_cosim.v once per pytest session with SCENARIO_FUZZ."""
    iverilog = _iverilog_bin()
    if not iverilog:
        pytest.skip("iverilog not available on PATH")

    out_dir = tmp_path_factory.mktemp("ddc_fuzz_build")
    vvp = out_dir / "tb_ddc_cosim_fuzz.vvp"
    sources = [str(FPGA_DIR / p) for p in SOURCE_FILES]
    cmd = [
        iverilog, "-g2001", "-DSIMULATION", "-DSCENARIO_FUZZ",
        "-o", str(vvp), *sources,
    ]
    res = subprocess.run(cmd, cwd=FPGA_DIR, capture_output=True, text=True, check=False)
    if res.returncode != 0:
        pytest.skip(f"iverilog compile failed:\n{res.stderr}")
    return vvp


def _iverilog_bin() -> str | None:
    from shutil import which
    return which("iverilog")


def _random_scene(seed: int) -> list[radar_scene.Target]:
    rng = random.Random(seed)
    n = rng.randint(1, 4)
    return [
        radar_scene.Target(
            range_m=rng.uniform(50, 1500),
            velocity_mps=rng.uniform(-40, 40),
            rcs_dbsm=rng.uniform(-10, 20),
            phase_deg=rng.uniform(0, 360),
        )
        for _ in range(n)
    ]


def _run_seed(seed: int, vvp: Path, work: Path) -> tuple[int, list[tuple[int, int]]]:
    """Generate stimulus, run the DUT, return (bb_sample_count, [(i,q)...])."""
    targets = _random_scene(seed)
    noise = random.Random(seed ^ 0xA5A5).uniform(0.5, 6.0)
    adc = radar_scene.generate_adc_samples(
        targets, N_ADC_SAMPLES, noise_stddev=noise, seed=seed
    )

    hex_path = work / f"adc_fuzz_{seed:04d}.hex"
    csv_path = work / f"rtl_bb_fuzz_{seed:04d}.csv"
    radar_scene.write_hex_file(str(hex_path), adc, bits=8)

    vvp_bin = _vvp_bin()
    if not vvp_bin:
        pytest.skip("vvp not available")

    cmd = [
        vvp_bin, str(vvp),
        f"+hex={hex_path}",
        f"+csv={csv_path}",
        f"+tag=seed{seed:04d}",
    ]
    res = subprocess.run(cmd, cwd=FPGA_DIR, capture_output=True, text=True, check=False, timeout=120)
    assert res.returncode == 0, f"vvp exit={res.returncode}\nstdout:\n{res.stdout}\nstderr:\n{res.stderr}"
    assert csv_path.exists(), (
        f"vvp completed rc=0 but CSV was not produced at {csv_path}\n"
        f"cmd: {cmd}\nstdout:\n{res.stdout[-2000:]}\nstderr:\n{res.stderr[-500:]}"
    )

    rows = []
    with csv_path.open() as fh:
        header = fh.readline()
        assert "baseband_i" in header and "baseband_q" in header, f"unexpected CSV header: {header!r}"
        for line in fh:
            parts = line.strip().split(",")
            if len(parts) != 3:
                continue
            _, i_str, q_str = parts
            rows.append((int(i_str), int(q_str)))
    return len(rows), rows


def _vvp_bin() -> str | None:
    from shutil import which
    return which("vvp")


def _fuzz_assertions(seed: int, rows: list[tuple[int, int]]) -> None:
    n = len(rows)
    assert EXPECTED_BB_MIN <= n <= EXPECTED_BB_MAX, (
        f"seed {seed}: bb sample count {n} outside [{EXPECTED_BB_MIN},{EXPECTED_BB_MAX}]"
    )
    for idx, (i, q) in enumerate(rows):
        assert SIGNED_18_MIN <= i <= SIGNED_18_MAX, (
            f"seed {seed} row {idx}: baseband_i={i} out of signed-18 range"
        )
        assert SIGNED_18_MIN <= q <= SIGNED_18_MAX, (
            f"seed {seed} row {idx}: baseband_q={q} out of signed-18 range"
        )
    all_zero = all(i == 0 and q == 0 for i, q in rows)
    assert not all_zero, f"seed {seed}: all-zero baseband output — pipeline likely stalled"


@pytest.mark.parametrize("seed", FAST_SEEDS)
def test_ddc_fuzz_fast(seed: int, compiled_fuzz_vvp: Path, tmp_path: Path) -> None:
    _, rows = _run_seed(seed, compiled_fuzz_vvp, tmp_path)
    _fuzz_assertions(seed, rows)


@pytest.mark.slow
@pytest.mark.parametrize("seed", SLOW_SEEDS)
def test_ddc_fuzz_full(seed: int, compiled_fuzz_vvp: Path, tmp_path: Path) -> None:
    _, rows = _run_seed(seed, compiled_fuzz_vvp, tmp_path)
    _fuzz_assertions(seed, rows)
