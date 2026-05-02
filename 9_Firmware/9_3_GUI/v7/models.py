"""
v7.models — Data classes, enums, and theme constants for the PLFM Radar GUI V7.

This module defines the core data structures used throughout the application:
  - RadarTarget, RadarSettings, GPSData (dataclasses)
  - TileServer (enum for map tile providers)
  - Dark theme color constants
  - Optional dependency availability flags
"""

import logging
from dataclasses import dataclass, asdict
from enum import Enum


# ---------------------------------------------------------------------------
# Optional dependency flags (graceful degradation)
# ---------------------------------------------------------------------------
try:
    import usb.core
    import usb.util  # noqa: F401 — availability check
    USB_AVAILABLE = True
except ImportError:
    USB_AVAILABLE = False
    logging.warning("pyusb not available. USB functionality will be disabled.")

try:
    from pyftdi.ftdi import Ftdi  # noqa: F401 — availability check
    from pyftdi.usbtools import UsbTools  # noqa: F401 — availability check
    from pyftdi.ftdi import FtdiError  # noqa: F401 — availability check
    FTDI_AVAILABLE = True
except ImportError:
    FTDI_AVAILABLE = False
    logging.warning("pyftdi not available. FTDI functionality will be disabled.")

try:
    from scipy import signal as _scipy_signal  # noqa: F401 — availability check
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    logging.warning("scipy not available. Some DSP features will be disabled.")

try:
    from sklearn.cluster import DBSCAN as _DBSCAN  # noqa: F401 — availability check
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False
    logging.warning("sklearn not available. Clustering will be disabled.")

try:
    from filterpy.kalman import KalmanFilter as _KalmanFilter  # noqa: F401 — availability check
    FILTERPY_AVAILABLE = True
except ImportError:
    FILTERPY_AVAILABLE = False
    logging.warning("filterpy not available. Kalman tracking will be disabled.")

# ---------------------------------------------------------------------------
# Dark theme color constants (shared by all modules)
# ---------------------------------------------------------------------------
DARK_BG = "#2b2b2b"
DARK_FG = "#e0e0e0"
DARK_ACCENT = "#3c3f41"
DARK_HIGHLIGHT = "#4e5254"
DARK_BORDER = "#555555"
DARK_TEXT = "#cccccc"
DARK_BUTTON = "#3c3f41"
DARK_BUTTON_HOVER = "#4e5254"
DARK_TREEVIEW = "#3c3f41"
DARK_TREEVIEW_ALT = "#404040"
DARK_SUCCESS = "#4CAF50"
DARK_WARNING = "#FFC107"
DARK_ERROR = "#F44336"
DARK_INFO = "#2196F3"

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class RadarTarget:
    """Represents a detected radar target."""
    id: int
    range: float           # Range in meters
    velocity: float        # Velocity in m/s (positive = approaching)
    azimuth: float         # Azimuth angle in degrees
    elevation: float       # Elevation angle in degrees
    latitude: float = 0.0
    longitude: float = 0.0
    snr: float = 0.0       # Signal-to-noise ratio in dB
    timestamp: float = 0.0
    track_id: int = -1
    classification: str = "unknown"

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)


@dataclass
class RadarSettings:
    """Radar system display/map configuration.

    FPGA register parameters (chirp timing, CFAR, MTI, gain, etc.) are
    controlled directly via 4-byte opcode commands — see the FPGA Control
    tab and Opcode enum in radar_protocol.py.  This dataclass holds only
    host-side display/map settings and physical-unit conversion factors.

    range_resolution and velocity_resolution below are placeholders. Live
    operation derives the actual values from WaveformConfig in
    workers.py:RadarDataWorker (see GUI-C3 fix); these literals are only
    consulted by code paths that have not yet been migrated, and should
    not be relied on for physics-accurate display.
    """
    system_frequency: float = 10.5e9    # Hz (carrier, used for velocity calc)
    range_resolution: float = 6.0        # Meters per range bin (c/(2*Fs)*decim = 1.5*4)
    velocity_resolution: float = 1.0     # m/s per Doppler bin (calibrate to waveform)
    max_distance: float = 3072           # Max detection range (m), 3 km mode
    map_size: float = 4000               # Map display size (m)
    coverage_radius: float = 3072        # Map coverage radius (m), 3 km mode


@dataclass
class GPSData:
    """GPS position and orientation data."""
    latitude: float
    longitude: float
    altitude: float
    pitch: float            # Pitch angle in degrees
    heading: float = 0.0    # Heading in degrees (0 = North)
    timestamp: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


# ---------------------------------------------------------------------------
# Tile server enum
# ---------------------------------------------------------------------------

@dataclass
class ProcessingConfig:
    """Host-side signal processing pipeline configuration.

    These control host-side DSP that runs AFTER the FPGA processing
    pipeline.  FPGA-side MTI, CFAR, and DC notch are controlled via
    register opcodes from the FPGA Control tab.

    Controls: DBSCAN clustering, Kalman tracking, and optional
    host-side reprocessing (MTI, CFAR, windowing, DC notch).
    """

    # MTI (Moving Target Indication)
    mti_enabled: bool = False
    mti_order: int = 2                     # 1, 2, or 3

    # CFAR (Constant False Alarm Rate)
    cfar_enabled: bool = False
    cfar_type: str = "CA-CFAR"             # CA-CFAR, OS-CFAR, GO-CFAR, SO-CFAR
    cfar_guard_cells: int = 2
    cfar_training_cells: int = 8
    cfar_threshold_factor: float = 5.0     # PFA-related scalar

    # DC Notch / DC Removal
    dc_notch_enabled: bool = False

    # Windowing (applied before FFT)
    window_type: str = "Hann"              # None, Hann, Hamming, Blackman, Kaiser, Chebyshev

    # Detection threshold (dB above noise floor)
    detection_threshold_db: float = 12.0

    # DBSCAN Clustering
    clustering_enabled: bool = True
    clustering_eps: float = 100.0
    clustering_min_samples: int = 2

    # Kalman Tracking
    tracking_enabled: bool = True


# ---------------------------------------------------------------------------
# Tile server enum
# ---------------------------------------------------------------------------

class TileServer(Enum):
    """Available map tile servers."""
    OPENSTREETMAP = "osm"
    GOOGLE_MAPS = "google"
    GOOGLE_SATELLITE = "google_sat"
    GOOGLE_HYBRID = "google_hybrid"
    ESRI_SATELLITE = "esri_sat"


# ---------------------------------------------------------------------------
# Waveform configuration (physical parameters for bin→unit conversion)
# ---------------------------------------------------------------------------

@dataclass
class WaveformConfig:
    """Physical waveform parameters for converting bins to SI units.

    PR-Q (3-PRI staggered ladder, audit C-5 Doppler unfolding):
      - SHORT  sub-frame:  1 us chirp /  175 us PRI
      - MEDIUM sub-frame:  5 us chirp /  161 us PRI
      - LONG   sub-frame: 30 us chirp /  167 us PRI

    Each sub-frame produces ``chirps_per_subframe`` Doppler bins
    (16 → 48 total).  Per-subframe v_unamb is ~+/-42 m/s; the host runs
    3-PRI Chinese-Remainder unfolding (see PR-Q.5
    processing.unfold_velocity_crt) to recover targets out to
    ``extended_max_velocity_mps_crt``.
    """

    sample_rate_hz: float = 100e6        # DDC output I/Q rate (matched filter input)
    bandwidth_hz: float = 20e6           # Chirp bandwidth (time-bandwidth product / display)
    chirp_duration_s: float = 30e-6      # LONG chirp ramp time (longest of the three)

    # Per-subframe PRIs (PR-Q stagger; mirrors radar_params.vh
    # RP_DEF_{SHORT,MEDIUM,LONG}_LISTEN_CYCLES + chirp cycles).
    pri_short_s:  float = 175e-6         # SHORT  PRI (1 us chirp + 174 us listen)
    pri_medium_s: float = 161e-6         # MEDIUM PRI (5 us chirp + 156 us listen)
    pri_long_s:   float = 167e-6         # LONG   PRI (30 us chirp + 137 us listen)

    center_freq_hz:      float = 10.5e9  # X-band carrier (radar_scene.py F_CARRIER)
    n_range_bins:        int = 512       # After decimation (3 km mode; 4096 in 20 km)
    n_doppler_bins:      int = 48        # 3 sub-frames * 16 chirps (matches RP_NUM_DOPPLER_BINS)
    chirps_per_subframe: int = 16        # Chirps in one Doppler sub-frame
    num_subframes:       int = 3         # SHORT, MEDIUM, LONG
    fft_size:            int = 2048      # Pre-decimation matched-filter FFT length
    decimation_factor:   int = 4         # 2048 -> 512

    # ------------------------------------------------------------------
    # Range
    # ------------------------------------------------------------------
    @property
    def range_resolution_m(self) -> float:
        """Meters per decimated range bin (matched-filter pulse compression).

        Each IFFT output bin spans c / (2 * Fs); after decimation the bin
        spacing grows by ``decimation_factor``.
        """
        c = 299_792_458.0
        raw_bin = c / (2.0 * self.sample_rate_hz)
        return raw_bin * self.decimation_factor

    @property
    def max_range_m(self) -> float:
        """Maximum unambiguous range in meters."""
        return self.range_resolution_m * self.n_range_bins

    # ------------------------------------------------------------------
    # Velocity (per sub-frame)
    # ------------------------------------------------------------------
    def _v_res(self, pri_s: float) -> float:
        c = 299_792_458.0
        wavelength = c / self.center_freq_hz
        return wavelength / (2.0 * self.chirps_per_subframe * pri_s)

    @property
    def velocity_resolution_short_mps(self) -> float:
        """m/s per Doppler bin in the SHORT sub-frame."""
        return self._v_res(self.pri_short_s)

    @property
    def velocity_resolution_medium_mps(self) -> float:
        """m/s per Doppler bin in the MEDIUM sub-frame."""
        return self._v_res(self.pri_medium_s)

    @property
    def velocity_resolution_long_mps(self) -> float:
        """m/s per Doppler bin in the LONG sub-frame."""
        return self._v_res(self.pri_long_s)

    @property
    def max_velocity_short_mps(self) -> float:
        """Per-subframe SHORT v_unamb (+/-)."""
        return self.velocity_resolution_short_mps * self.chirps_per_subframe / 2.0

    @property
    def max_velocity_medium_mps(self) -> float:
        """Per-subframe MEDIUM v_unamb (+/-)."""
        return self.velocity_resolution_medium_mps * self.chirps_per_subframe / 2.0

    @property
    def max_velocity_long_mps(self) -> float:
        """Per-subframe LONG v_unamb (+/-)."""
        return self.velocity_resolution_long_mps * self.chirps_per_subframe / 2.0

    def extended_max_velocity_mps_crt(self, max_alias_k: int = 6) -> float:
        """CRT-extended unambiguous velocity ceiling (PR-Q C-5).

        Three coprime PRIs let the host resolve aliases up to
        ``max_alias_k`` folds before the alias set itself becomes
        ambiguous.  Returns the velocity beyond which detections must
        be flagged AMBIGUOUS even after CRT unfolding.

        Ceiling is set by the largest per-subframe v_unamb (smallest
        PRI) times the alias search depth.  For PR-Q stagger
        (175/161/167 us) with K=6 the practical ceiling is ~266 m/s,
        well above typical UAS speeds (50-80 m/s).
        """
        v_unamb = max(
            self.max_velocity_short_mps,
            self.max_velocity_medium_mps,
            self.max_velocity_long_mps,
        )
        return v_unamb * max_alias_k
