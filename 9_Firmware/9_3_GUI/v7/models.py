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
class WaveformConfig:
    """FMCW waveform parameters for bin-to-physical-unit conversion.

    Defaults are for the ADI CN0566 phaser (10 GHz, 500 MHz BW, 300 µs chirp,
    4 MSPS ADC).  For the PLFM FPGA waveform the values differ — but the FPGA
    pipeline hardcodes its own bin widths, so this config is only used for
    Raw IQ Replay (host-side FFT processing).
    """
    sample_rate_hz: float = 4e6         # ADC sample rate (Hz)
    bandwidth_hz: float = 500e6         # Chirp bandwidth (Hz)
    chirp_duration_s: float = 300e-6    # Chirp sweep time (s)
    center_freq_hz: float = 10e9        # Carrier frequency (Hz)

    # --- derived quantities (need n_samples, n_chirps from file) ---

    def range_resolution(self, n_samples: int) -> float:
        """Metres per range bin for an N-point range FFT.

        range_per_bin = c · fs / (2 · N · slope)
        where slope = BW / T_chirp.
        """
        c = 299_792_458.0
        slope = self.bandwidth_hz / self.chirp_duration_s
        return c * self.sample_rate_hz / (2.0 * n_samples * slope)

    def velocity_resolution(self, n_samples: int, n_chirps: int) -> float:
        """m/s per Doppler bin for an M-chirp Doppler FFT.

        vel_per_bin = λ · fs / (2 · N · M)
        where λ = c / fc, N = n_samples (PRI = N/fs), M = n_chirps.
        """
        c = 299_792_458.0
        wavelength = c / self.center_freq_hz
        return wavelength * self.sample_rate_hz / (2.0 * n_samples * n_chirps)

    def max_range(self, n_range_bins: int, n_samples: int) -> float:
        """Maximum unambiguous range for the given bin count."""
        return self.range_resolution(n_samples) * n_range_bins


@dataclass
class RadarSettings:
    """Radar system display/map configuration.

    FPGA register parameters (chirp timing, CFAR, MTI, gain, etc.) are
    controlled directly via 4-byte opcode commands — see the FPGA Control
    tab and Opcode enum in radar_protocol.py.  This dataclass holds only
    host-side display/map settings and physical-unit conversion factors.

    range_resolution and velocity_resolution should be calibrated to
    the actual waveform parameters.
    """
    system_frequency: float = 10e9      # Hz (carrier, used for velocity calc)
    range_resolution: float = 781.25    # Meters per range bin (default: 50km/64)
    velocity_resolution: float = 1.0    # m/s per Doppler bin (calibrate to waveform)
    max_distance: float = 50000         # Max detection range (m)
    map_size: float = 50000             # Map display size (m)
    coverage_radius: float = 50000      # Map coverage radius (m)


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
