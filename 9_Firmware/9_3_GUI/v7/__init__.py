"""
v7 — PLFM Radar GUI V7 (PyQt6 edition).

Re-exports all public classes and functions from sub-modules for convenient
top-level imports:

    from v7 import RadarDashboard, RadarTarget, RadarSettings, ...
"""

# Models / constants
from .models import (
    RadarTarget,
    RadarSettings,
    WaveformConfig,
    GPSData,
    ProcessingConfig,
    TileServer,
    DARK_BG, DARK_FG, DARK_ACCENT, DARK_HIGHLIGHT, DARK_BORDER,
    DARK_TEXT, DARK_BUTTON, DARK_BUTTON_HOVER,
    DARK_TREEVIEW, DARK_TREEVIEW_ALT,
    DARK_SUCCESS, DARK_WARNING, DARK_ERROR, DARK_INFO,
    USB_AVAILABLE, FTDI_AVAILABLE, SCIPY_AVAILABLE,
    SKLEARN_AVAILABLE, FILTERPY_AVAILABLE,
)

# Hardware interfaces — production protocol via radar_protocol.py
from .hardware import (
    FT2232HConnection,
    ReplayConnection,
    RadarProtocol,
    Opcode,
    RadarAcquisition,
    RadarFrame,
    StatusResponse,
    DataRecorder,
    STM32USBInterface,
)

# Processing pipeline
from .processing import (
    RadarProcessor,
    USBPacketParser,
    apply_pitch_correction,
)

# Workers, map widget, and dashboard require PyQt6 — import lazily so that
# tests/CI environments without PyQt6 can still access models/hardware/processing.
try:
    from .workers import (
        RadarDataWorker,
        GPSDataWorker,
        TargetSimulator,
        polar_to_geographic,
    )

    from .map_widget import (
        MapBridge,
        RadarMapWidget,
    )

    from .dashboard import (
        RadarDashboard,
        RangeDopplerCanvas,
    )
except ImportError:  # PyQt6 not installed (e.g. CI headless runner)
    pass

__all__ = [  # noqa: RUF022
    # models
    "RadarTarget", "RadarSettings", "WaveformConfig", "GPSData", "ProcessingConfig", "TileServer",
    "DARK_BG", "DARK_FG", "DARK_ACCENT", "DARK_HIGHLIGHT", "DARK_BORDER",
    "DARK_TEXT", "DARK_BUTTON", "DARK_BUTTON_HOVER",
    "DARK_TREEVIEW", "DARK_TREEVIEW_ALT",
    "DARK_SUCCESS", "DARK_WARNING", "DARK_ERROR", "DARK_INFO",
    "USB_AVAILABLE", "FTDI_AVAILABLE", "SCIPY_AVAILABLE",
    "SKLEARN_AVAILABLE", "FILTERPY_AVAILABLE",
    # hardware — production FPGA protocol
    "FT2232HConnection", "ReplayConnection", "RadarProtocol", "Opcode",
    "RadarAcquisition", "RadarFrame", "StatusResponse", "DataRecorder",
    "STM32USBInterface",
    # processing
    "RadarProcessor", "USBPacketParser",
    "apply_pitch_correction",
    # workers
    "RadarDataWorker", "GPSDataWorker", "TargetSimulator",
    "polar_to_geographic",
    # map
    "MapBridge", "RadarMapWidget",
    # dashboard
    "RadarDashboard", "RangeDopplerCanvas",
]
