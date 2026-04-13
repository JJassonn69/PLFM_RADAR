"""
v7.dashboard — Main application window for the PLFM Radar GUI V7.

RadarDashboard is a QMainWindow with six tabs:
  1. Main View   — Range-Doppler matplotlib canvas (64x32), device combos,
                   Start/Stop, targets table
  2. Map View    — Embedded Leaflet map + sidebar
  3. FPGA Control — Full FPGA register control panel (all 27 opcodes incl. AGC,
                    bit-width validation, grouped layout matching production)
  4. AGC Monitor — Real-time AGC strip charts (gain, peak magnitude, saturation)
  5. Diagnostics — Connection indicators, packet stats, dependency status,
                   self-test results, log viewer
  6. Settings    — Host-side DSP parameters + About section

Uses production radar_protocol.py for all FPGA communication:
  - FT2232HConnection for real hardware
  - ReplayConnection for offline .npy replay
  - Mock mode (FT2232HConnection(mock=True)) for development

The old STM32 magic-packet start flow has been removed. FPGA registers
are controlled directly via 4-byte {opcode, addr, value_hi, value_lo}
commands sent over FT2232H.
"""

import time
import re
import logging
from collections import deque
from pathlib import Path

import numpy as np

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QTabWidget, QSplitter, QGroupBox, QFrame, QScrollArea,
    QLabel, QPushButton, QComboBox, QCheckBox,
    QDoubleSpinBox, QSpinBox, QLineEdit,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QPlainTextEdit, QStatusBar, QMessageBox,
)
from PyQt6.QtCore import Qt, QLocale, QTimer, pyqtSignal, pyqtSlot, QObject

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from .models import (
    RadarTarget, RadarSettings, WaveformConfig, GPSData, ProcessingConfig,
    DARK_BG, DARK_FG, DARK_ACCENT, DARK_HIGHLIGHT, DARK_BORDER,
    DARK_TEXT, DARK_BUTTON, DARK_BUTTON_HOVER,
    DARK_TREEVIEW, DARK_TREEVIEW_ALT,
    DARK_SUCCESS, DARK_WARNING, DARK_ERROR, DARK_INFO,
    USB_AVAILABLE, FTDI_AVAILABLE, SCIPY_AVAILABLE,
    SKLEARN_AVAILABLE, FILTERPY_AVAILABLE,
)
from .hardware import (
    FT2232HConnection,
    ReplayConnection,
    RadarProtocol,
    RadarFrame,
    StatusResponse,
    DataRecorder,
    STM32USBInterface,
)
from .processing import RadarProcessor, USBPacketParser, RawIQFrameProcessor
from .workers import RadarDataWorker, RawIQReplayWorker, GPSDataWorker, TargetSimulator
from .raw_iq_replay import RawIQReplayController, PlaybackState
from .agc_sim import AGCConfig
from .map_widget import RadarMapWidget

logger = logging.getLogger(__name__)

# Frame dimensions from FPGA
NUM_RANGE_BINS = 64
NUM_DOPPLER_BINS = 32

# Force C locale (period as decimal separator) for all QDoubleSpinBox instances.
_C_LOCALE = QLocale(QLocale.Language.C)
_C_LOCALE.setNumberOptions(QLocale.NumberOption.RejectGroupSeparator)


def _make_dspin() -> QDoubleSpinBox:
    """Create a QDoubleSpinBox with C locale (no comma decimals)."""
    sb = QDoubleSpinBox()
    sb.setLocale(_C_LOCALE)
    return sb


def _parse_waveform_from_filename(name: str) -> WaveformConfig | None:
    """Try to extract waveform params from ADI phaser filename convention.

    Expected pattern fragments (order-independent):
      ``<N>MSPS`` or ``<N>MSps`` → sample rate in MHz
      ``<N>M``  (followed by _ or end) → bandwidth in MHz
      ``<N>u``  (followed by _ or end) → chirp duration in µs

    Returns a WaveformConfig with parsed values (defaults for un-parsed),
    or None if nothing recognisable was found.
    """
    cfg = WaveformConfig()  # ADI phaser defaults
    found = False

    # Sample rate: "4MSPS" or "4MSps"
    m = re.search(r"(\d+)M[Ss][Pp][Ss]", name)
    if m:
        cfg.sample_rate_hz = float(m.group(1)) * 1e6
        found = True

    # Bandwidth: "500M" (must NOT be followed by S for MSPS)
    m = re.search(r"(\d+)M(?![Ss])", name)
    if m:
        cfg.bandwidth_hz = float(m.group(1)) * 1e6
        found = True

    # Chirp duration: "300u"
    m = re.search(r"(\d+)u", name)
    if m:
        cfg.chirp_duration_s = float(m.group(1)) * 1e-6
        found = True

    return cfg if found else None


# =============================================================================
# Range-Doppler Canvas (matplotlib)
# =============================================================================

class RangeDopplerCanvas(FigureCanvasQTAgg):
    """Matplotlib canvas showing a Range-Doppler map with dark theme.

    Adapts dynamically to incoming frame dimensions (e.g. 64x32 from FPGA,
    or different sizes from Raw IQ Replay).
    """

    def __init__(self, _parent=None):
        fig = Figure(figsize=(10, 6), facecolor=DARK_BG)
        self.ax = fig.add_subplot(111, facecolor=DARK_ACCENT)

        # Initial backing data — will resize on first update_map call
        self._n_range = NUM_RANGE_BINS
        self._n_doppler = NUM_DOPPLER_BINS
        self._data = np.zeros((self._n_range, self._n_doppler))
        self.im = self.ax.imshow(
            self._data, aspect="auto", cmap="hot",
            extent=[0, self._n_doppler, 0, self._n_range], origin="lower",
        )

        self.ax.set_title(
            f"Range-Doppler Map ({self._n_range}x{self._n_doppler})",
            color=DARK_FG,
        )
        self.ax.set_xlabel("Doppler Bin", color=DARK_FG)
        self.ax.set_ylabel("Range Bin", color=DARK_FG)
        self.ax.tick_params(colors=DARK_FG)
        for spine in self.ax.spines.values():
            spine.set_color(DARK_BORDER)

        fig.tight_layout()
        super().__init__(fig)

    def update_map(self, magnitude: np.ndarray, _detections: np.ndarray = None):
        """Update the heatmap with new magnitude data.

        Automatically resizes the canvas if the incoming shape differs from
        the current backing array.
        """
        nr, nd = magnitude.shape
        if nr != self._n_range or nd != self._n_doppler:
            self._n_range = nr
            self._n_doppler = nd
            self._data = np.zeros((nr, nd))
            self.im.set_extent([0, nd, 0, nr])
            self.ax.set_title(
                f"Range-Doppler Map ({nr}x{nd})", color=DARK_FG)

        display = np.log10(magnitude + 1)
        self.im.set_data(display)
        self.im.set_clim(vmin=display.min(), vmax=max(display.max(), 0.1))
        self.draw_idle()


# =============================================================================
# RadarDashboard — main window
# =============================================================================

class RadarDashboard(QMainWindow):
    """Main application window with 5 tabs."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("AERIS-10 Radar System V7 — PyQt6")
        self.setGeometry(100, 60, 1500, 950)

        # ---- Core objects --------------------------------------------------
        self._settings = RadarSettings()
        self._radar_position = GPSData(
            latitude=41.9028, longitude=12.4964,
            altitude=0.0, pitch=0.0, heading=0.0, timestamp=0.0,
        )

        # Hardware interfaces — production protocol
        self._connection: FT2232HConnection | None = None
        self._stm32 = STM32USBInterface()
        self._recorder = DataRecorder()

        # Processing
        self._processor = RadarProcessor()
        self._usb_parser = USBPacketParser()
        self._processing_config = ProcessingConfig()

        # Device lists
        self._stm32_devices: list = []

        # Workers (created on demand)
        self._radar_worker: RadarDataWorker | None = None
        self._gps_worker: GPSDataWorker | None = None
        self._simulator: TargetSimulator | None = None

        # Raw IQ Replay
        self._replay_controller: RawIQReplayController | None = None
        self._replay_worker: RawIQReplayWorker | None = None
        self._iq_processor: RawIQFrameProcessor | None = None

        # State
        self._running = False
        self._demo_mode = False
        self._replay_status_override: str | None = None  # "playing"/"paused"
        self._start_time = time.time()
        self._current_frame: RadarFrame | None = None
        self._last_status: StatusResponse | None = None
        self._frame_count = 0
        self._gps_packet_count = 0
        self._last_stats: dict = {}
        self._current_targets: list[RadarTarget] = []

        # FPGA control parameter widgets
        self._param_spins: dict = {}  # opcode_hex -> QSpinBox

        # AGC visualization history (ring buffers)
        self._agc_history_len = 256
        self._agc_gain_history: deque[int] = deque(maxlen=self._agc_history_len)
        self._agc_peak_history: deque[int] = deque(maxlen=self._agc_history_len)
        self._agc_sat_history: deque[int] = deque(maxlen=self._agc_history_len)
        self._agc_last_redraw: float = 0.0  # throttle chart redraws
        self._AGC_REDRAW_INTERVAL: float = 0.5  # seconds between redraws

        # ---- Build UI ------------------------------------------------------
        self._apply_dark_theme()
        self._setup_ui()
        self._setup_statusbar()

        # GUI refresh timer (100 ms)
        self._gui_timer = QTimer(self)
        self._gui_timer.timeout.connect(self._refresh_gui)
        self._gui_timer.start(100)

        # Log handler for diagnostics (thread-safe via Qt signal)
        self._log_bridge = _LogSignalBridge(self)
        self._log_bridge.log_message.connect(self._log_append)
        self._log_handler = _QtLogHandler(self._log_bridge)
        self._log_handler.setLevel(logging.INFO)
        logging.getLogger().addHandler(self._log_handler)

        logger.info("RadarDashboard initialised (production protocol)")

    # =====================================================================
    # Dark theme
    # =====================================================================

    def _apply_dark_theme(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background-color: {DARK_BG};
                color: {DARK_FG};
            }}
            QTabWidget::pane {{
                border: 1px solid {DARK_BORDER};
                background-color: {DARK_BG};
            }}
            QTabBar::tab {{
                background-color: {DARK_ACCENT};
                color: {DARK_FG};
                padding: 8px 18px;
                border: 1px solid {DARK_BORDER};
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }}
            QTabBar::tab:selected {{
                background-color: {DARK_HIGHLIGHT};
            }}
            QTabBar::tab:hover {{
                background-color: {DARK_BUTTON_HOVER};
            }}
            QGroupBox {{
                border: 1px solid {DARK_BORDER};
                border-radius: 4px;
                margin-top: 12px;
                padding-top: 12px;
                font-weight: bold;
                color: {DARK_FG};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
            }}
            QPushButton {{
                background-color: {DARK_BUTTON};
                color: {DARK_FG};
                border: 1px solid {DARK_BORDER};
                padding: 6px 16px;
                border-radius: 4px;
            }}
            QPushButton:hover {{
                background-color: {DARK_BUTTON_HOVER};
            }}
            QPushButton:pressed {{
                background-color: {DARK_HIGHLIGHT};
            }}
            QPushButton:disabled {{
                color: {DARK_BORDER};
            }}
            QComboBox {{
                background-color: {DARK_ACCENT};
                color: {DARK_FG};
                border: 1px solid {DARK_BORDER};
                padding: 4px 8px;
                border-radius: 4px;
            }}
            QLineEdit, QSpinBox, QDoubleSpinBox {{
                background-color: {DARK_ACCENT};
                color: {DARK_FG};
                border: 1px solid {DARK_BORDER};
                padding: 4px 8px;
                border-radius: 4px;
            }}
            QCheckBox {{
                color: {DARK_FG};
                spacing: 6px;
            }}
            QLabel {{
                color: {DARK_FG};
            }}
            QTableWidget {{
                background-color: {DARK_TREEVIEW};
                alternate-background-color: {DARK_TREEVIEW_ALT};
                color: {DARK_FG};
                gridline-color: {DARK_BORDER};
                border: 1px solid {DARK_BORDER};
            }}
            QTableWidget::item:selected {{
                background-color: {DARK_INFO};
            }}
            QHeaderView::section {{
                background-color: {DARK_HIGHLIGHT};
                color: {DARK_FG};
                padding: 6px;
                border: none;
                border-right: 1px solid {DARK_BORDER};
                border-bottom: 1px solid {DARK_BORDER};
            }}
            QPlainTextEdit {{
                background-color: {DARK_ACCENT};
                color: {DARK_FG};
                border: 1px solid {DARK_BORDER};
                font-family: 'Courier New', monospace;
                font-size: 11px;
            }}
            QScrollBar:vertical {{
                background-color: {DARK_ACCENT};
                width: 12px;
            }}
            QScrollBar::handle:vertical {{
                background-color: {DARK_HIGHLIGHT};
                border-radius: 6px;
                min-height: 20px;
            }}
            QStatusBar {{
                background-color: {DARK_ACCENT};
                color: {DARK_FG};
            }}
        """)

    # =====================================================================
    # UI construction
    # =====================================================================

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        self._tabs = QTabWidget()
        main_layout.addWidget(self._tabs)

        self._create_main_tab()
        self._create_map_tab()
        self._create_fpga_control_tab()
        self._create_agc_monitor_tab()
        self._create_diagnostics_tab()
        self._create_settings_tab()

    # -----------------------------------------------------------------
    # TAB 1: Main View
    # -----------------------------------------------------------------

    def _create_main_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)

        # ---- Control bar ---------------------------------------------------
        ctrl = QFrame()
        ctrl.setStyleSheet(f"background-color: {DARK_ACCENT}; border-radius: 4px;")
        ctrl_layout = QGridLayout(ctrl)
        ctrl_layout.setContentsMargins(8, 6, 8, 6)

        # Row 0: connection mode + device combos + buttons
        ctrl_layout.addWidget(QLabel("Mode:"), 0, 0)
        self._mode_combo = QComboBox()
        self._mode_combo.addItems([
            "Mock", "Live FT2232H", "Replay (.npy)", "Raw IQ Replay (.npy)"])
        self._mode_combo.setCurrentIndex(0)
        ctrl_layout.addWidget(self._mode_combo, 0, 1)

        ctrl_layout.addWidget(QLabel("STM32 GPS:"), 0, 2)
        self._stm32_combo = QComboBox()
        self._stm32_combo.setMinimumWidth(200)
        ctrl_layout.addWidget(self._stm32_combo, 0, 3)

        refresh_btn = QPushButton("Refresh Devices")
        refresh_btn.clicked.connect(self._refresh_devices)
        ctrl_layout.addWidget(refresh_btn, 0, 4)

        self._start_btn = QPushButton("Start Radar")
        self._start_btn.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_SUCCESS}; color: white; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: #66BB6A; }}"
        )
        self._start_btn.clicked.connect(self._start_radar)
        ctrl_layout.addWidget(self._start_btn, 0, 7)

        self._stop_btn = QPushButton("Stop Radar")
        self._stop_btn.setEnabled(False)
        self._stop_btn.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_ERROR}; color: white; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: #EF5350; }}"
        )
        self._stop_btn.clicked.connect(self._stop_radar)
        ctrl_layout.addWidget(self._stop_btn, 0, 8)

        self._demo_btn_main = QPushButton("Start Demo")
        self._demo_btn_main.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_INFO}; color: white; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: #42A5F5; }}"
        )
        self._demo_btn_main.clicked.connect(self._toggle_demo_main)
        ctrl_layout.addWidget(self._demo_btn_main, 0, 9)

        # Row 1: status labels
        self._gps_label = QLabel("GPS: Waiting for data...")
        ctrl_layout.addWidget(self._gps_label, 1, 0, 1, 3)

        self._pitch_label = QLabel("Pitch: --.--\u00b0")
        ctrl_layout.addWidget(self._pitch_label, 1, 3, 1, 2)

        self._status_label_main = QLabel("Status: Ready")
        self._status_label_main.setAlignment(Qt.AlignmentFlag.AlignRight)
        ctrl_layout.addWidget(self._status_label_main, 1, 5, 1, 5)

        # Row 2: Raw IQ playback controls (hidden until Raw IQ mode active)
        self._playback_frame = QFrame()
        self._playback_frame.setStyleSheet(
            f"background-color: {DARK_HIGHLIGHT}; border-radius: 4px;")
        pb_layout = QHBoxLayout(self._playback_frame)
        pb_layout.setContentsMargins(8, 4, 8, 4)

        self._pb_play_btn = QPushButton("Play")
        self._pb_play_btn.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_SUCCESS}; color: white; }}")
        self._pb_play_btn.clicked.connect(self._pb_play_pause)
        pb_layout.addWidget(self._pb_play_btn)

        self._pb_step_btn = QPushButton("Step")
        self._pb_step_btn.clicked.connect(self._pb_step)
        pb_layout.addWidget(self._pb_step_btn)

        self._pb_stop_btn = QPushButton("Stop")
        self._pb_stop_btn.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_ERROR}; color: white; }}")
        self._pb_stop_btn.clicked.connect(self._stop_radar)
        pb_layout.addWidget(self._pb_stop_btn)

        pb_layout.addWidget(QLabel("FPS:"))
        self._pb_fps_spin = _make_dspin()
        self._pb_fps_spin.setRange(0.1, 60.0)
        self._pb_fps_spin.setValue(10.0)
        self._pb_fps_spin.setSingleStep(1.0)
        self._pb_fps_spin.valueChanged.connect(self._pb_fps_changed)
        pb_layout.addWidget(self._pb_fps_spin)

        self._pb_loop_check = QCheckBox("Loop")
        self._pb_loop_check.setChecked(True)
        self._pb_loop_check.toggled.connect(self._pb_loop_changed)
        pb_layout.addWidget(self._pb_loop_check)

        self._pb_frame_label = QLabel("Frame: 0 / 0")
        self._pb_frame_label.setStyleSheet(
            f"color: {DARK_INFO}; font-weight: bold;")
        pb_layout.addWidget(self._pb_frame_label)

        self._pb_file_label = QLabel("")
        self._pb_file_label.setStyleSheet(f"color: {DARK_TEXT}; font-size: 10px;")
        pb_layout.addWidget(self._pb_file_label)

        pb_layout.addStretch()
        self._playback_frame.setVisible(False)
        ctrl_layout.addWidget(self._playback_frame, 2, 0, 1, 10)

        # -- Waveform config row (Raw IQ replay only) -----------------------
        self._waveform_frame = QFrame()
        self._waveform_frame.setStyleSheet(
            f"background-color: {DARK_ACCENT}; border-radius: 4px;")
        wf_layout = QHBoxLayout(self._waveform_frame)
        wf_layout.setContentsMargins(8, 4, 8, 4)

        wf_layout.addWidget(QLabel("Waveform:"))

        wf_layout.addWidget(QLabel("fs (MHz):"))
        self._wf_fs_spin = _make_dspin()
        self._wf_fs_spin.setRange(0.1, 100.0)
        self._wf_fs_spin.setValue(4.0)
        self._wf_fs_spin.setDecimals(2)
        self._wf_fs_spin.setToolTip("ADC sample rate in MHz")
        wf_layout.addWidget(self._wf_fs_spin)

        wf_layout.addWidget(QLabel("BW (MHz):"))
        self._wf_bw_spin = _make_dspin()
        self._wf_bw_spin.setRange(1.0, 5000.0)
        self._wf_bw_spin.setValue(500.0)
        self._wf_bw_spin.setDecimals(1)
        self._wf_bw_spin.setToolTip("Chirp bandwidth in MHz")
        wf_layout.addWidget(self._wf_bw_spin)

        wf_layout.addWidget(QLabel("T (us):"))
        self._wf_chirp_spin = _make_dspin()
        self._wf_chirp_spin.setRange(1.0, 10000.0)
        self._wf_chirp_spin.setValue(300.0)
        self._wf_chirp_spin.setDecimals(1)
        self._wf_chirp_spin.setToolTip("Chirp duration in microseconds")
        wf_layout.addWidget(self._wf_chirp_spin)

        wf_layout.addWidget(QLabel("fc (GHz):"))
        self._wf_fc_spin = _make_dspin()
        self._wf_fc_spin.setRange(0.1, 100.0)
        self._wf_fc_spin.setValue(10.0)
        self._wf_fc_spin.setDecimals(2)
        self._wf_fc_spin.setToolTip("Carrier frequency in GHz")
        wf_layout.addWidget(self._wf_fc_spin)

        self._wf_res_label = QLabel("")
        self._wf_res_label.setStyleSheet(f"color: {DARK_INFO}; font-size: 10px;")
        wf_layout.addWidget(self._wf_res_label)

        wf_layout.addStretch()
        self._waveform_frame.setVisible(False)
        ctrl_layout.addWidget(self._waveform_frame, 3, 0, 1, 10)

        layout.addWidget(ctrl)

        # ---- Display area (range-doppler + targets table) ------------------
        display_splitter = QSplitter(Qt.Orientation.Horizontal)

        # Range-Doppler canvas
        self._rdm_canvas = RangeDopplerCanvas()
        display_splitter.addWidget(self._rdm_canvas)

        # Targets table
        targets_group = QGroupBox("Detected Targets")
        tg_layout = QVBoxLayout(targets_group)

        self._targets_table_main = QTableWidget()
        self._targets_table_main.setColumnCount(5)
        self._targets_table_main.setHorizontalHeaderLabels([
            "Range (m)", "Velocity (m/s)", "Magnitude", "SNR (dB)", "Track ID",
        ])
        self._targets_table_main.setAlternatingRowColors(True)
        self._targets_table_main.setSelectionBehavior(
            QTableWidget.SelectionBehavior.SelectRows
        )
        header = self._targets_table_main.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        tg_layout.addWidget(self._targets_table_main)

        display_splitter.addWidget(targets_group)
        display_splitter.setSizes([800, 400])

        layout.addWidget(display_splitter, stretch=1)
        self._tabs.addTab(tab, "Main View")

    # -----------------------------------------------------------------
    # TAB 2: Map View
    # -----------------------------------------------------------------

    def _create_map_tab(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(4, 4, 4, 4)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Map widget
        self._map_widget = RadarMapWidget(
            radar_lat=self._radar_position.latitude,
            radar_lon=self._radar_position.longitude,
        )
        self._map_widget.targetSelected.connect(self._on_target_selected)
        splitter.addWidget(self._map_widget)

        # Sidebar
        sidebar = QWidget()
        sidebar.setMaximumWidth(320)
        sidebar.setMinimumWidth(280)
        sb_layout = QVBoxLayout(sidebar)
        sb_layout.setContentsMargins(8, 8, 8, 8)

        # Radar position group
        pos_group = QGroupBox("Radar Position")
        pos_layout = QGridLayout(pos_group)

        self._lat_spin = _make_dspin()
        self._lat_spin.setRange(-90, 90)
        self._lat_spin.setDecimals(6)
        self._lat_spin.setValue(self._radar_position.latitude)
        self._lat_spin.valueChanged.connect(self._on_position_changed)

        self._lon_spin = _make_dspin()
        self._lon_spin.setRange(-180, 180)
        self._lon_spin.setDecimals(6)
        self._lon_spin.setValue(self._radar_position.longitude)
        self._lon_spin.valueChanged.connect(self._on_position_changed)

        self._alt_spin = _make_dspin()
        self._alt_spin.setRange(0, 50000)
        self._alt_spin.setDecimals(1)
        self._alt_spin.setValue(0.0)
        self._alt_spin.setSuffix(" m")

        self._heading_spin = _make_dspin()
        self._heading_spin.setRange(0, 360)
        self._heading_spin.setDecimals(1)
        self._heading_spin.setValue(0.0)
        self._heading_spin.setSuffix("\u00b0")
        self._heading_spin.setWrapping(True)
        self._heading_spin.valueChanged.connect(self._on_position_changed)

        pos_layout.addWidget(QLabel("Latitude:"), 0, 0)
        pos_layout.addWidget(self._lat_spin, 0, 1)
        pos_layout.addWidget(QLabel("Longitude:"), 1, 0)
        pos_layout.addWidget(self._lon_spin, 1, 1)
        pos_layout.addWidget(QLabel("Altitude:"), 2, 0)
        pos_layout.addWidget(self._alt_spin, 2, 1)
        pos_layout.addWidget(QLabel("Heading:"), 3, 0)
        pos_layout.addWidget(self._heading_spin, 3, 1)

        sb_layout.addWidget(pos_group)

        # Coverage group
        cov_group = QGroupBox("Coverage")
        cov_layout = QGridLayout(cov_group)

        self._coverage_spin = _make_dspin()
        self._coverage_spin.setRange(1, 200)
        self._coverage_spin.setDecimals(1)
        self._coverage_spin.setValue(self._settings.coverage_radius / 1000)
        self._coverage_spin.setSuffix(" km")
        self._coverage_spin.valueChanged.connect(self._on_coverage_changed)

        cov_layout.addWidget(QLabel("Radius:"), 0, 0)
        cov_layout.addWidget(self._coverage_spin, 0, 1)

        sb_layout.addWidget(cov_group)

        # Demo controls group
        demo_group = QGroupBox("Demo Mode")
        demo_layout = QVBoxLayout(demo_group)

        self._demo_btn_map = QPushButton("Start Demo")
        self._demo_btn_map.setCheckable(True)
        self._demo_btn_map.clicked.connect(self._toggle_demo_map)
        demo_layout.addWidget(self._demo_btn_map)

        add_btn = QPushButton("Add Random Target")
        add_btn.clicked.connect(self._add_demo_target)
        demo_layout.addWidget(add_btn)

        sb_layout.addWidget(demo_group)

        # Selected target info
        info_group = QGroupBox("Selected Target")
        info_layout = QVBoxLayout(info_group)

        self._target_info_label = QLabel("No target selected")
        self._target_info_label.setWordWrap(True)
        self._target_info_label.setStyleSheet(f"color: {DARK_TEXT}; padding: 8px;")
        info_layout.addWidget(self._target_info_label)

        sb_layout.addWidget(info_group)
        sb_layout.addStretch()

        splitter.addWidget(sidebar)
        splitter.setSizes([900, 300])

        layout.addWidget(splitter)
        self._tabs.addTab(tab, "Map View")

    # -----------------------------------------------------------------
    # TAB 3: FPGA Control (production register map)
    # -----------------------------------------------------------------

    def _create_fpga_control_tab(self):
        """FPGA register control panel — all 22 opcodes with validation.

        Layout: 3-column scrollable:
          Left:   Radar Operation + Signal Processing + Diagnostics
          Center: Waveform Timing
          Right:  Detection (CFAR) + Custom Command
        """
        tab = QWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        inner = QWidget()
        outer_layout = QHBoxLayout(inner)
        outer_layout.setContentsMargins(8, 8, 8, 8)
        outer_layout.setSpacing(12)

        # ── Left column ──────────────────────────────────────────────
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)

        # -- Radar Operation --
        grp_op = QGroupBox("Radar Operation")
        op_layout = QVBoxLayout(grp_op)

        btn_mode_on = QPushButton("Radar Mode On")
        btn_mode_on.clicked.connect(lambda: self._send_fpga_cmd(0x01, 1))
        op_layout.addWidget(btn_mode_on)

        btn_mode_off = QPushButton("Radar Mode Off")
        btn_mode_off.clicked.connect(lambda: self._send_fpga_cmd(0x01, 0))
        op_layout.addWidget(btn_mode_off)

        btn_trigger = QPushButton("Trigger Chirp")
        btn_trigger.clicked.connect(lambda: self._send_fpga_cmd(0x02, 1))
        op_layout.addWidget(btn_trigger)

        # Stream Control (3-bit mask)
        self._add_fpga_param_row(op_layout, "Stream Control", 0x04, 7, 3,
                                 "0-7, 3-bit mask, rst=7")

        btn_status = QPushButton("Request Status")
        btn_status.clicked.connect(lambda: self._send_fpga_cmd(0xFF, 0))
        op_layout.addWidget(btn_status)

        left_layout.addWidget(grp_op)

        # -- Signal Processing --
        grp_sp = QGroupBox("Signal Processing")
        sp_layout = QVBoxLayout(grp_sp)

        sp_params = [
            ("Detect Threshold",  0x03, 10000, 16, "0-65535, rst=10000"),
            ("Gain Shift",        0x16, 0,     4,  "0-15, dir+shift"),
            ("MTI Enable",        0x26, 0,     1,  "0=off, 1=on"),
            ("DC Notch Width",    0x27, 0,     3,  "0-7 bins"),
        ]
        for label, opcode, default, bits, hint in sp_params:
            self._add_fpga_param_row(sp_layout, label, opcode, default, bits, hint)

        # MTI quick toggles
        mti_row = QHBoxLayout()
        btn_mti_on = QPushButton("Enable MTI")
        btn_mti_on.clicked.connect(lambda: self._send_fpga_cmd(0x26, 1))
        mti_row.addWidget(btn_mti_on)
        btn_mti_off = QPushButton("Disable MTI")
        btn_mti_off.clicked.connect(lambda: self._send_fpga_cmd(0x26, 0))
        mti_row.addWidget(btn_mti_off)
        sp_layout.addLayout(mti_row)

        left_layout.addWidget(grp_sp)

        # -- Diagnostics --
        grp_diag = QGroupBox("Diagnostics")
        diag_layout = QVBoxLayout(grp_diag)

        btn_selftest = QPushButton("Run Self-Test")
        btn_selftest.clicked.connect(lambda: self._send_fpga_cmd(0x30, 1))
        diag_layout.addWidget(btn_selftest)

        btn_selftest_read = QPushButton("Read Self-Test Result")
        btn_selftest_read.clicked.connect(lambda: self._send_fpga_cmd(0x31, 0))
        diag_layout.addWidget(btn_selftest_read)

        # Self-test result labels
        st_group = QGroupBox("Self-Test Results")
        st_layout = QVBoxLayout(st_group)
        self._st_labels = {}
        for name, default_text in [
            ("busy", "Busy: --"),
            ("flags", "Flags: -----"),
            ("detail", "Detail: 0x--"),
            ("t0", "T0 BRAM: --"),
            ("t1", "T1 CIC:  --"),
            ("t2", "T2 FFT:  --"),
            ("t3", "T3 Arith: --"),
            ("t4", "T4 ADC:  --"),
        ]:
            lbl = QLabel(default_text)
            lbl.setStyleSheet("font-family: 'Courier New', monospace; font-size: 11px;")
            st_layout.addWidget(lbl)
            self._st_labels[name] = lbl
        diag_layout.addWidget(st_group)

        left_layout.addWidget(grp_diag)
        left_layout.addStretch()
        outer_layout.addWidget(left, stretch=1)

        # ── Center column: Waveform Timing ────────────────────────────
        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(0, 0, 0, 0)

        grp_wf = QGroupBox("Waveform Timing")
        wf_layout = QVBoxLayout(grp_wf)

        wf_params = [
            ("Long Chirp Cycles",   0x10, 3000,  16, "0-65535, rst=3000"),
            ("Long Listen Cycles",  0x11, 13700, 16, "0-65535, rst=13700"),
            ("Guard Cycles",        0x12, 17540, 16, "0-65535, rst=17540"),
            ("Short Chirp Cycles",  0x13, 50,    16, "0-65535, rst=50"),
            ("Short Listen Cycles", 0x14, 17450, 16, "0-65535, rst=17450"),
            ("Chirps Per Elevation", 0x15, 32,    6, "1-32, clamped"),
        ]
        for label, opcode, default, bits, hint in wf_params:
            self._add_fpga_param_row(wf_layout, label, opcode, default, bits, hint)

        center_layout.addWidget(grp_wf)
        center_layout.addStretch()
        outer_layout.addWidget(center, stretch=1)

        # ── Right column: Detection (CFAR) + Custom Command ───────────
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)

        grp_cfar = QGroupBox("Detection (CFAR)")
        cfar_layout = QVBoxLayout(grp_cfar)

        cfar_params = [
            ("CFAR Enable",       0x25, 0,  1,  "0=off, 1=on"),
            ("CFAR Guard Cells",  0x21, 2,  4,  "0-15, rst=2"),
            ("CFAR Train Cells",  0x22, 8,  5,  "1-31, rst=8"),
            ("CFAR Alpha (Q4.4)", 0x23, 48, 8,  "0-255, rst=0x30=3.0"),
            ("CFAR Mode",         0x24, 0,  2,  "0=CA 1=GO 2=SO"),
        ]
        for label, opcode, default, bits, hint in cfar_params:
            self._add_fpga_param_row(cfar_layout, label, opcode, default, bits, hint)

        # CFAR quick toggles
        cfar_row = QHBoxLayout()
        btn_cfar_on = QPushButton("Enable CFAR")
        btn_cfar_on.clicked.connect(lambda: self._send_fpga_cmd(0x25, 1))
        cfar_row.addWidget(btn_cfar_on)
        btn_cfar_off = QPushButton("Disable CFAR")
        btn_cfar_off.clicked.connect(lambda: self._send_fpga_cmd(0x25, 0))
        cfar_row.addWidget(btn_cfar_off)
        cfar_layout.addLayout(cfar_row)

        right_layout.addWidget(grp_cfar)

        # ── AGC (Automatic Gain Control) ──────────────────────────────
        grp_agc = QGroupBox("AGC (Auto Gain)")
        agc_layout = QVBoxLayout(grp_agc)

        agc_params = [
            ("AGC Enable",  0x28,   0,  1, "0=manual, 1=auto"),
            ("AGC Target",  0x29, 200,  8, "0-255, peak target"),
            ("AGC Attack",  0x2A,   1,  4, "0-15, atten step"),
            ("AGC Decay",   0x2B,   1,  4, "0-15, gain-up step"),
            ("AGC Holdoff", 0x2C,   4,  4, "0-15, frames"),
        ]
        for label, opcode, default, bits, hint in agc_params:
            self._add_fpga_param_row(agc_layout, label, opcode, default, bits, hint)

        # AGC quick toggles
        agc_row = QHBoxLayout()
        btn_agc_on = QPushButton("Enable AGC")
        btn_agc_on.clicked.connect(lambda: self._send_fpga_cmd(0x28, 1))
        agc_row.addWidget(btn_agc_on)
        btn_agc_off = QPushButton("Disable AGC")
        btn_agc_off.clicked.connect(lambda: self._send_fpga_cmd(0x28, 0))
        agc_row.addWidget(btn_agc_off)
        agc_layout.addLayout(agc_row)

        # AGC status readback labels
        agc_st_group = QGroupBox("AGC Status")
        agc_st_layout = QVBoxLayout(agc_st_group)
        self._agc_labels: dict[str, QLabel] = {}
        for name, default_text in [
            ("enable", "AGC: --"),
            ("gain",   "Gain: --"),
            ("peak",   "Peak: --"),
            ("sat",    "Sat Count: --"),
        ]:
            lbl = QLabel(default_text)
            lbl.setStyleSheet(f"color: {DARK_INFO}; font-size: 10px;")
            agc_st_layout.addWidget(lbl)
            self._agc_labels[name] = lbl
        agc_layout.addWidget(agc_st_group)

        right_layout.addWidget(grp_agc)

        # Custom Command
        grp_custom = QGroupBox("Custom Command")
        cust_layout = QGridLayout(grp_custom)

        cust_layout.addWidget(QLabel("Opcode (hex):"), 0, 0)
        self._custom_opcode = QLineEdit("01")
        self._custom_opcode.setMaximumWidth(80)
        cust_layout.addWidget(self._custom_opcode, 0, 1)

        cust_layout.addWidget(QLabel("Value (dec):"), 1, 0)
        self._custom_value = QLineEdit("0")
        self._custom_value.setMaximumWidth(80)
        cust_layout.addWidget(self._custom_value, 1, 1)

        btn_send_custom = QPushButton("Send")
        btn_send_custom.clicked.connect(self._send_custom_command)
        cust_layout.addWidget(btn_send_custom, 2, 0, 1, 2)

        right_layout.addWidget(grp_custom)
        right_layout.addStretch()
        outer_layout.addWidget(right, stretch=1)

        scroll.setWidget(inner)
        tab_layout = QVBoxLayout(tab)
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.addWidget(scroll)

        self._tabs.addTab(tab, "FPGA Control")

    def _add_fpga_param_row(self, parent_layout: QVBoxLayout, label: str,
                            opcode: int, default: int, bits: int, hint: str):
        """Add a single FPGA parameter row: label + spinbox + hint + Set button."""
        row = QHBoxLayout()

        lbl = QLabel(label)
        lbl.setMinimumWidth(140)
        row.addWidget(lbl)

        max_val = (1 << bits) - 1
        spin = QSpinBox()
        spin.setRange(0, max_val)
        spin.setValue(default)
        spin.setMinimumWidth(80)
        row.addWidget(spin)
        self._param_spins[f"0x{opcode:02X}"] = spin

        hint_lbl = QLabel(hint)
        hint_lbl.setStyleSheet(f"color: {DARK_INFO}; font-size: 10px;")
        row.addWidget(hint_lbl)

        btn = QPushButton("Set")
        btn.setMaximumWidth(60)
        # Capture opcode and spin by value
        btn.clicked.connect(lambda _, op=opcode, sp=spin, b=bits:
                            self._send_fpga_validated(op, sp.value(), b))
        row.addWidget(btn)

        parent_layout.addLayout(row)

    # -----------------------------------------------------------------
    # TAB 4: AGC Monitor
    # -----------------------------------------------------------------

    def _create_agc_monitor_tab(self):
        """AGC Monitor — real-time strip charts for FPGA inner-loop AGC."""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)

        # ---- Top indicator row ---------------------------------------------
        indicator = QFrame()
        indicator.setStyleSheet(
            f"background-color: {DARK_ACCENT}; border-radius: 4px;")
        ind_layout = QHBoxLayout(indicator)
        ind_layout.setContentsMargins(12, 8, 12, 8)

        self._agc_mode_lbl = QLabel("AGC: --")
        self._agc_mode_lbl.setStyleSheet(
            f"color: {DARK_FG}; font-size: 16px; font-weight: bold;")
        ind_layout.addWidget(self._agc_mode_lbl)

        self._agc_gain_lbl = QLabel("Gain: --")
        self._agc_gain_lbl.setStyleSheet(
            f"color: {DARK_INFO}; font-size: 14px;")
        ind_layout.addWidget(self._agc_gain_lbl)

        self._agc_peak_lbl = QLabel("Peak: --")
        self._agc_peak_lbl.setStyleSheet(
            f"color: {DARK_INFO}; font-size: 14px;")
        ind_layout.addWidget(self._agc_peak_lbl)

        self._agc_sat_total_lbl = QLabel("Total Saturations: 0")
        self._agc_sat_total_lbl.setStyleSheet(
            f"color: {DARK_SUCCESS}; font-size: 14px; font-weight: bold;")
        ind_layout.addWidget(self._agc_sat_total_lbl)

        ind_layout.addStretch()
        layout.addWidget(indicator)

        # ---- Matplotlib figure with 3 subplots -----------------------------
        agc_fig = Figure(figsize=(12, 7), facecolor=DARK_BG)
        agc_fig.subplots_adjust(
            left=0.07, right=0.96, top=0.95, bottom=0.07,
            hspace=0.32)

        # Subplot 1: Gain history (4-bit, 0-15)
        self._agc_ax_gain = agc_fig.add_subplot(3, 1, 1)
        self._agc_ax_gain.set_facecolor(DARK_ACCENT)
        self._agc_ax_gain.set_ylabel("Gain Code", color=DARK_FG, fontsize=10)
        self._agc_ax_gain.set_title(
            "FPGA Inner-Loop Gain (4-bit)", color=DARK_FG, fontsize=11)
        self._agc_ax_gain.set_ylim(-0.5, 15.5)
        self._agc_ax_gain.tick_params(colors=DARK_FG, labelsize=9)
        self._agc_ax_gain.set_xlim(0, self._agc_history_len)
        for spine in self._agc_ax_gain.spines.values():
            spine.set_color(DARK_BORDER)
        self._agc_gain_line, = self._agc_ax_gain.plot(
            [], [], color="#89b4fa", linewidth=1.5, label="Gain")
        self._agc_ax_gain.axhline(y=7.5, color=DARK_WARNING, linestyle="--",
                                  linewidth=0.8, alpha=0.5, label="Midpoint")
        self._agc_ax_gain.legend(
            loc="upper right", fontsize=8,
            facecolor=DARK_ACCENT, edgecolor=DARK_BORDER,
            labelcolor=DARK_FG)

        # Subplot 2: Peak magnitude (8-bit, 0-255)
        self._agc_ax_peak = agc_fig.add_subplot(
            3, 1, 2, sharex=self._agc_ax_gain)
        self._agc_ax_peak.set_facecolor(DARK_ACCENT)
        self._agc_ax_peak.set_ylabel("Peak Mag", color=DARK_FG, fontsize=10)
        self._agc_ax_peak.set_title(
            "ADC Peak Magnitude (8-bit)", color=DARK_FG, fontsize=11)
        self._agc_ax_peak.set_ylim(-5, 260)
        self._agc_ax_peak.tick_params(colors=DARK_FG, labelsize=9)
        for spine in self._agc_ax_peak.spines.values():
            spine.set_color(DARK_BORDER)
        self._agc_peak_line, = self._agc_ax_peak.plot(
            [], [], color=DARK_SUCCESS, linewidth=1.5, label="Peak")
        self._agc_ax_peak.axhline(y=200, color=DARK_WARNING, linestyle="--",
                                  linewidth=0.8, alpha=0.5,
                                  label="Target (200)")
        self._agc_ax_peak.axhspan(240, 255, alpha=0.15, color=DARK_ERROR,
                                  label="Sat Zone")
        self._agc_ax_peak.legend(
            loc="upper right", fontsize=8,
            facecolor=DARK_ACCENT, edgecolor=DARK_BORDER,
            labelcolor=DARK_FG)

        # Subplot 3: Saturation count per update (8-bit, 0-255)
        self._agc_ax_sat = agc_fig.add_subplot(
            3, 1, 3, sharex=self._agc_ax_gain)
        self._agc_ax_sat.set_facecolor(DARK_ACCENT)
        self._agc_ax_sat.set_ylabel("Sat Count", color=DARK_FG, fontsize=10)
        self._agc_ax_sat.set_xlabel(
            "Sample (newest right)", color=DARK_FG, fontsize=10)
        self._agc_ax_sat.set_title(
            "Saturation Events per Update", color=DARK_FG, fontsize=11)
        self._agc_ax_sat.set_ylim(-1, 10)
        self._agc_ax_sat.tick_params(colors=DARK_FG, labelsize=9)
        for spine in self._agc_ax_sat.spines.values():
            spine.set_color(DARK_BORDER)
        self._agc_sat_line, = self._agc_ax_sat.plot(
            [], [], color=DARK_ERROR, linewidth=1.0, label="Saturation")
        self._agc_sat_fill_artist = None
        self._agc_ax_sat.legend(
            loc="upper right", fontsize=8,
            facecolor=DARK_ACCENT, edgecolor=DARK_BORDER,
            labelcolor=DARK_FG)

        self._agc_canvas = FigureCanvasQTAgg(agc_fig)
        layout.addWidget(self._agc_canvas, stretch=1)

        self._tabs.addTab(tab, "AGC Monitor")

    # -----------------------------------------------------------------
    # TAB 5: Diagnostics
    # -----------------------------------------------------------------

    def _create_diagnostics_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)

        top_row = QHBoxLayout()

        # Connection status
        conn_group = QGroupBox("Connection Status")
        conn_layout = QGridLayout(conn_group)

        self._conn_ft2232h = self._make_status_label("FT2232H")
        self._conn_stm32 = self._make_status_label("STM32 USB")

        conn_layout.addWidget(QLabel("FT2232H:"), 0, 0)
        conn_layout.addWidget(self._conn_ft2232h, 0, 1)
        conn_layout.addWidget(QLabel("STM32 USB:"), 1, 0)
        conn_layout.addWidget(self._conn_stm32, 1, 1)

        top_row.addWidget(conn_group)

        # Frame statistics
        stats_group = QGroupBox("Statistics")
        stats_layout = QGridLayout(stats_group)

        labels = [
            "Frames:", "Detections:", "GPS Packets:",
            "Errors:", "Uptime:", "Frame Rate:",
        ]
        self._diag_values: list = []
        for i, text in enumerate(labels):
            r, c = divmod(i, 2)
            stats_layout.addWidget(QLabel(text), r, c * 2)
            val = QLabel("0")
            val.setStyleSheet(f"color: {DARK_INFO}; font-weight: bold;")
            stats_layout.addWidget(val, r, c * 2 + 1)
            self._diag_values.append(val)

        top_row.addWidget(stats_group)

        # FPGA Status readback
        fpga_group = QGroupBox("FPGA Status Readback")
        fpga_layout = QVBoxLayout(fpga_group)
        self._fpga_status_label = QLabel("No status received yet")
        self._fpga_status_label.setWordWrap(True)
        self._fpga_status_label.setStyleSheet(
            "font-family: 'Courier New', monospace; font-size: 11px; padding: 4px;")
        fpga_layout.addWidget(self._fpga_status_label)

        top_row.addWidget(fpga_group)

        # Dependency status
        dep_group = QGroupBox("Optional Dependencies")
        dep_layout = QGridLayout(dep_group)

        deps = [
            ("pyusb", USB_AVAILABLE),
            ("pyftdi", FTDI_AVAILABLE),
            ("scipy", SCIPY_AVAILABLE),
            ("sklearn", SKLEARN_AVAILABLE),
            ("filterpy", FILTERPY_AVAILABLE),
        ]
        for i, (name, avail) in enumerate(deps):
            dep_layout.addWidget(QLabel(name), i, 0)
            lbl = QLabel("Available" if avail else "Missing")
            lbl.setStyleSheet(
                f"color: {DARK_SUCCESS}; font-weight: bold;"
                if avail else
                f"color: {DARK_WARNING}; font-weight: bold;"
            )
            dep_layout.addWidget(lbl, i, 1)

        top_row.addWidget(dep_group)

        layout.addLayout(top_row)

        # Log viewer
        log_group = QGroupBox("System Log")
        log_layout = QVBoxLayout(log_group)

        self._log_text = QPlainTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setMaximumBlockCount(500)
        log_layout.addWidget(self._log_text)

        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self._log_text.clear)
        log_layout.addWidget(clear_btn)

        layout.addWidget(log_group, stretch=1)

        self._tabs.addTab(tab, "Diagnostics")

    # -----------------------------------------------------------------
    # TAB 5: Settings (host-side DSP)
    # -----------------------------------------------------------------

    def _create_settings_tab(self):
        tab = QWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setContentsMargins(8, 8, 8, 8)

        # ---- Host-side DSP group -------------------------------------------
        proc_group = QGroupBox("Host-Side Signal Processing (post-FPGA)")
        p_layout = QGridLayout(proc_group)
        row = 0

        note = QLabel(
            "These settings control host-side DSP that runs AFTER the FPGA "
            "processing pipeline. FPGA-side MTI, CFAR, and DC notch are "
            "controlled from the FPGA Control tab."
        )
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {DARK_WARNING}; padding: 6px;")
        p_layout.addWidget(note, row, 0, 1, 2)
        row += 1

        # Clustering
        self._cluster_check = QCheckBox("DBSCAN Clustering")
        self._cluster_check.setChecked(self._processing_config.clustering_enabled)
        if not SKLEARN_AVAILABLE:
            self._cluster_check.setEnabled(False)
            self._cluster_check.setToolTip("Requires scikit-learn")
        p_layout.addWidget(self._cluster_check, row, 0, 1, 2)
        row += 1

        p_layout.addWidget(QLabel("DBSCAN eps:"), row, 0)
        self._cluster_eps_spin = _make_dspin()
        self._cluster_eps_spin.setRange(1.0, 5000.0)
        self._cluster_eps_spin.setDecimals(1)
        self._cluster_eps_spin.setValue(self._processing_config.clustering_eps)
        self._cluster_eps_spin.setSingleStep(10.0)
        p_layout.addWidget(self._cluster_eps_spin, row, 1)
        row += 1

        p_layout.addWidget(QLabel("Min Samples:"), row, 0)
        self._cluster_min_spin = QSpinBox()
        self._cluster_min_spin.setRange(1, 20)
        self._cluster_min_spin.setValue(self._processing_config.clustering_min_samples)
        p_layout.addWidget(self._cluster_min_spin, row, 1)
        row += 1

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f"color: {DARK_BORDER};")
        p_layout.addWidget(sep, row, 0, 1, 2)
        row += 1

        # Kalman Tracking
        self._tracking_check = QCheckBox("Kalman Tracking")
        self._tracking_check.setChecked(self._processing_config.tracking_enabled)
        if not FILTERPY_AVAILABLE:
            self._tracking_check.setEnabled(False)
            self._tracking_check.setToolTip("Requires filterpy")
        p_layout.addWidget(self._tracking_check, row, 0, 1, 2)
        row += 1

        # Apply
        apply_proc_btn = QPushButton("Apply Host DSP Settings")
        apply_proc_btn.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_SUCCESS}; color: white; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: #66BB6A; }}"
        )
        apply_proc_btn.clicked.connect(self._apply_processing_config)
        p_layout.addWidget(apply_proc_btn, row, 0, 1, 2)

        layout.addWidget(proc_group)

        # ---- About group ---------------------------------------------------
        about_group = QGroupBox("About")
        about_layout = QVBoxLayout(about_group)
        about_lbl = QLabel(
            "<b>AERIS-10 Radar System V7</b><br>"
            "PyQt6 Edition with Embedded Leaflet Map<br><br>"
            "<b>Data Interface:</b> FT2232H USB 2.0 (production protocol)<br>"
            "<b>FPGA Protocol:</b> 4-byte register commands, 0xAA/0xBB packets<br>"
            "<b>Map:</b> OpenStreetMap + Leaflet.js<br>"
            "<b>Framework:</b> PyQt6 + QWebEngine<br>"
            "<b>Version:</b> 7.1.0 (production protocol)"
        )
        about_lbl.setStyleSheet(f"color: {DARK_TEXT}; padding: 12px;")
        about_layout.addWidget(about_lbl)

        layout.addWidget(about_group)
        layout.addStretch()

        scroll.setWidget(inner)
        tab_layout = QVBoxLayout(tab)
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.addWidget(scroll)

        self._tabs.addTab(tab, "Settings")

    # =====================================================================
    # Status bar
    # =====================================================================

    def _setup_statusbar(self):
        bar = QStatusBar()
        self.setStatusBar(bar)

        self._sb_status = QLabel("Ready")
        bar.addWidget(self._sb_status)

        self._sb_targets = QLabel("Targets: 0")
        bar.addPermanentWidget(self._sb_targets)

        self._sb_mode = QLabel("Idle")
        self._sb_mode.setStyleSheet(f"color: {DARK_INFO}; font-weight: bold;")
        bar.addPermanentWidget(self._sb_mode)

    # =====================================================================
    # Device management
    # =====================================================================

    def _refresh_devices(self):
        # STM32 GPS
        self._stm32_devices = self._stm32.list_devices()
        self._stm32_combo.clear()
        for d in self._stm32_devices:
            self._stm32_combo.addItem(d["description"])
        if self._stm32_devices:
            self._stm32_combo.setCurrentIndex(0)

        logger.info(f"Devices refreshed: {len(self._stm32_devices)} STM32")

    # =====================================================================
    # FPGA command sending
    # =====================================================================

    def _send_fpga_cmd(self, opcode: int, value: int):
        """Send a 4-byte register command to the FPGA via FT2232H."""
        if self._connection is None or not self._connection.is_open:
            logger.warning(f"Cannot send 0x{opcode:02X}={value}: no connection")
            return
        cmd = RadarProtocol.build_command(opcode, value)
        ok = self._connection.write(cmd)
        if ok:
            logger.info(f"Sent FPGA cmd: 0x{opcode:02X} = {value}")
        else:
            logger.error(f"Failed to send FPGA cmd: 0x{opcode:02X}")

    def _send_fpga_validated(self, opcode: int, value: int, bits: int):
        """Clamp value to bit-width and send."""
        max_val = (1 << bits) - 1
        clamped = max(0, min(value, max_val))
        if clamped != value:
            logger.warning(f"Value {value} clamped to {clamped} "
                           f"({bits}-bit max={max_val}) for opcode 0x{opcode:02X}")
            # Update the spinbox
            key = f"0x{opcode:02X}"
            if key in self._param_spins:
                self._param_spins[key].setValue(clamped)
        self._send_fpga_cmd(opcode, clamped)

    def _send_custom_command(self):
        """Send custom opcode + value from the FPGA Control tab."""
        try:
            opcode = int(self._custom_opcode.text(), 16)
            value = int(self._custom_value.text())
            self._send_fpga_cmd(opcode, value)
        except ValueError:
            logger.error("Invalid custom command: check opcode (hex) and value (dec)")

    # =====================================================================
    # Start / Stop radar
    # =====================================================================

    def _start_radar(self):
        """Start radar data acquisition using production protocol."""
        # Mutual exclusion: stop demo if running
        if self._demo_mode:
            self._stop_demo()

        try:
            mode = self._mode_combo.currentText()

            if "Raw IQ" in mode:
                self._start_raw_iq_replay()
                return

            if "Mock" in mode:
                self._connection = FT2232HConnection(mock=True)
                if not self._connection.open():
                    QMessageBox.critical(self, "Error", "Failed to open mock connection.")
                    return
            elif "Live" in mode:
                self._connection = FT2232HConnection(mock=False)
                if not self._connection.open():
                    QMessageBox.critical(self, "Error",
                                         "Failed to open FT2232H. Check USB connection.")
                    return
            elif "Replay" in mode:
                from PyQt6.QtWidgets import QFileDialog
                npy_dir = QFileDialog.getExistingDirectory(
                    self, "Select .npy replay directory")
                if not npy_dir:
                    return
                self._connection = ReplayConnection(npy_dir)
                if not self._connection.open():
                    QMessageBox.critical(self, "Error",
                                         "Failed to open replay connection.")
                    return
            else:
                QMessageBox.warning(self, "Warning", "Unknown connection mode.")
                return

            # Start radar worker
            self._radar_worker = RadarDataWorker(
                connection=self._connection,
                processor=self._processor,
                recorder=self._recorder if self._recorder.recording else None,
                gps_data_ref=self._radar_position,
                settings=self._settings,
            )
            self._radar_worker.frameReady.connect(self._on_frame_ready)
            self._radar_worker.statusReceived.connect(self._on_status_received)
            self._radar_worker.targetsUpdated.connect(self._on_radar_targets)
            self._radar_worker.statsUpdated.connect(self._on_radar_stats)
            self._radar_worker.errorOccurred.connect(self._on_worker_error)
            self._radar_worker.finished.connect(self._on_worker_finished)
            self._radar_worker.start()

            # Optionally start GPS worker
            idx = self._stm32_combo.currentIndex()
            if (idx >= 0 and idx < len(self._stm32_devices)
                    and self._stm32.open_device(self._stm32_devices[idx])):
                self._gps_worker = GPSDataWorker(
                    stm32=self._stm32,
                    usb_parser=self._usb_parser,
                )
                self._gps_worker.gpsReceived.connect(self._on_gps_received)
                self._gps_worker.errorOccurred.connect(self._on_worker_error)
                self._gps_worker.start()

            # UI state
            self._running = True
            self._start_time = time.time()
            self._frame_count = 0
            self._start_btn.setEnabled(False)
            self._stop_btn.setEnabled(True)
            self._mode_combo.setEnabled(False)
            self._demo_btn_main.setEnabled(False)
            self._demo_btn_map.setEnabled(False)
            self._status_label_main.setText(f"Status: Running ({mode})")
            self._sb_status.setText(f"Running ({mode})")
            self._sb_mode.setText(mode)
            logger.info(f"Radar started: {mode}")

        except RuntimeError as e:
            QMessageBox.critical(self, "Error", f"Failed to start radar: {e}")
            logger.error(f"Start radar error: {e}")

    def _stop_radar(self):
        self._running = False
        self._replay_status_override = None

        # Stop demo simulator if active (prevents cross-mode interference)
        if self._simulator:
            self._simulator.stop()
            self._simulator = None
            self._demo_mode = False
            self._demo_btn_main.setText("Start Demo")
            self._demo_btn_map.setText("Start Demo")
            self._demo_btn_map.setChecked(False)

        if self._radar_worker:
            self._radar_worker.stop()
            self._radar_worker.wait(2000)
            self._radar_worker = None

        # Raw IQ Replay cleanup
        if self._replay_controller is not None:
            self._replay_controller.stop()
        if self._replay_worker is not None:
            self._replay_worker.stop()
            self._replay_worker.wait(2000)
            self._replay_worker = None
        self._replay_controller = None
        self._iq_processor = None

        if self._gps_worker:
            self._gps_worker.stop()
            self._gps_worker.wait(2000)
            self._gps_worker = None

        if self._connection:
            self._connection.close()
            self._connection = None

        self._stm32.close()

        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._mode_combo.setEnabled(True)
        self._demo_btn_main.setEnabled(True)
        self._demo_btn_map.setEnabled(True)
        self._playback_frame.setVisible(False)
        self._waveform_frame.setVisible(False)
        self._pb_play_btn.setText("Play")
        self._pb_frame_label.setText("Frame: 0 / 0")
        self._pb_file_label.setText("")
        self._status_label_main.setText("Status: Radar stopped")
        self._sb_status.setText("Radar stopped")
        self._sb_mode.setText("Idle")
        logger.info("Radar system stopped")

    # =====================================================================
    # Raw IQ Replay
    # =====================================================================

    def _start_raw_iq_replay(self):
        """Start raw IQ replay mode: load .npy file and begin playback."""
        from PyQt6.QtWidgets import QFileDialog

        npy_path, _ = QFileDialog.getOpenFileName(
            self, "Select Raw IQ .npy file", "",
            "NumPy files (*.npy);;All files (*)")
        if not npy_path:
            return

        try:
            # Create controller and load file
            self._replay_controller = RawIQReplayController()
            info = self._replay_controller.load_file(npy_path)

            # -- Waveform calibration: try to parse from filename -----------
            parsed_wf = _parse_waveform_from_filename(Path(npy_path).name)
            if parsed_wf is not None:
                self._wf_fs_spin.setValue(parsed_wf.sample_rate_hz / 1e6)
                self._wf_bw_spin.setValue(parsed_wf.bandwidth_hz / 1e6)
                self._wf_chirp_spin.setValue(parsed_wf.chirp_duration_s / 1e-6)
                self._wf_fc_spin.setValue(parsed_wf.center_freq_hz / 1e9)
                logger.info("Waveform params parsed from filename: %s", parsed_wf)

            # Build waveform config from (possibly updated) spinboxes
            wfc = self._waveform_config_from_ui()
            range_res = wfc.range_resolution(info.n_samples)
            vel_res = wfc.velocity_resolution(info.n_samples, info.n_chirps)
            n_range_out = min(64, info.n_samples)
            max_range = range_res * n_range_out

            # Create replay-specific RadarSettings with correct calibration
            replay_settings = RadarSettings(
                system_frequency=wfc.center_freq_hz,
                range_resolution=range_res,
                velocity_resolution=vel_res,
                max_distance=max_range,
                map_size=max_range * 1.2,
                coverage_radius=max_range * 1.2,
            )
            logger.info(
                "Replay calibration: range_res=%.4f m/bin, vel_res=%.4f m/s/bin, "
                "max_range=%.1f m",
                range_res, vel_res, max_range,
            )

            # Update coverage/map spinboxes to match replay scale
            self._coverage_spin.setValue(replay_settings.coverage_radius / 1000)
            self._update_waveform_res_label(info.n_samples, info.n_chirps)

            # Create frame processor
            self._iq_processor = RawIQFrameProcessor(
                n_range_out=n_range_out,
                n_doppler_out=min(32, info.n_chirps),
            )

            # Apply current AGC settings from FPGA Control tab
            agc_enable = self._param_spins.get("0x28")
            agc_target = self._param_spins.get("0x29")
            agc_attack = self._param_spins.get("0x2A")
            agc_decay = self._param_spins.get("0x2B")
            agc_holdoff = self._param_spins.get("0x2C")
            self._iq_processor.set_agc_config(AGCConfig(
                enabled=bool(agc_enable.value()) if agc_enable else False,
                target=agc_target.value() if agc_target else 200,
                attack=agc_attack.value() if agc_attack else 1,
                decay=agc_decay.value() if agc_decay else 1,
                holdoff=agc_holdoff.value() if agc_holdoff else 4,
            ))

            # Apply CFAR settings
            cfar_en = self._param_spins.get("0x25")
            cfar_guard = self._param_spins.get("0x21")
            cfar_train = self._param_spins.get("0x22")
            cfar_alpha = self._param_spins.get("0x23")
            cfar_mode = self._param_spins.get("0x24")
            self._iq_processor.set_cfar_params(
                enabled=bool(cfar_en.value()) if cfar_en else False,
                guard=cfar_guard.value() if cfar_guard else 2,
                train=cfar_train.value() if cfar_train else 8,
                alpha_q44=cfar_alpha.value() if cfar_alpha else 0x30,
                mode=cfar_mode.value() if cfar_mode else 0,
            )

            # Apply MTI / DC notch
            mti_en = self._param_spins.get("0x26")
            dc_notch = self._param_spins.get("0x27")
            self._iq_processor.set_mti_enabled(
                bool(mti_en.value()) if mti_en else False)
            self._iq_processor.set_dc_notch_width(
                dc_notch.value() if dc_notch else 0)

            # Threshold
            thresh = self._param_spins.get("0x03")
            self._iq_processor.set_detect_threshold(
                thresh.value() if thresh else 10000)

            # Create worker
            self._replay_worker = RawIQReplayWorker(
                controller=self._replay_controller,
                processor=self._iq_processor,
                host_processor=self._processor,
                settings=replay_settings,
                gps_data_ref=self._radar_position,
            )
            self._replay_worker.frameReady.connect(self._on_frame_ready)
            self._replay_worker.statusReceived.connect(self._on_status_received)
            self._replay_worker.targetsUpdated.connect(self._on_radar_targets)
            self._replay_worker.statsUpdated.connect(self._on_radar_stats)
            self._replay_worker.errorOccurred.connect(self._on_worker_error)
            self._replay_worker.finished.connect(self._on_worker_finished)
            self._replay_worker.playbackStateChanged.connect(
                self._on_playback_state_changed)
            self._replay_worker.frameIndexChanged.connect(
                self._on_frame_index_changed)

            # Start worker (paused initially)
            self._replay_worker.start()

            # UI state
            self._running = True
            self._replay_status_override = "paused"
            self._start_time = time.time()
            self._frame_count = 0
            self._start_btn.setEnabled(False)
            self._stop_btn.setEnabled(True)
            self._mode_combo.setEnabled(False)
            self._demo_btn_main.setEnabled(False)
            self._demo_btn_map.setEnabled(False)
            self._playback_frame.setVisible(True)
            self._waveform_frame.setVisible(True)
            self._pb_frame_label.setText(f"Frame: 0 / {info.n_frames}")
            self._pb_file_label.setText(
                f"{Path(npy_path).name} "
                f"({info.n_chirps}x{info.n_samples}, "
                f"{info.file_size_mb:.1f} MB)")
            self._status_label_main.setText("Status: Raw IQ Replay (paused)")
            self._sb_status.setText("Raw IQ Replay")
            self._sb_mode.setText("Raw IQ Replay")
            logger.info(f"Raw IQ Replay started: {npy_path}")

        except (ValueError, OSError) as e:
            # Clean up any partially-created objects
            if self._replay_worker is not None:
                self._replay_worker.stop()
                self._replay_worker.wait(1000)
                self._replay_worker = None
            self._replay_controller = None
            self._iq_processor = None
            QMessageBox.critical(self, "Error",
                                 f"Failed to load raw IQ file:\n{e}")
            logger.error(f"Raw IQ load error: {e}")

    # ---- Playback control slots --------------------------------------------

    def _pb_play_pause(self):
        """Toggle play/pause for raw IQ replay."""
        if self._replay_controller is None:
            return
        state = self._replay_controller.state
        if state == PlaybackState.PLAYING:
            self._replay_controller.pause()
            self._pb_play_btn.setText("Play")
        else:
            self._replay_controller.play()
            self._pb_play_btn.setText("Pause")

    def _pb_step(self):
        """Step one frame forward in raw IQ replay."""
        if self._replay_controller is not None:
            self._replay_controller.step_forward()

    def _pb_fps_changed(self, value: float):
        if self._replay_controller is not None:
            self._replay_controller.set_fps(value)

    def _pb_loop_changed(self, checked: bool):
        if self._replay_controller is not None:
            self._replay_controller.set_loop(checked)

    def _waveform_config_from_ui(self) -> WaveformConfig:
        """Build a WaveformConfig from the waveform spinboxes."""
        return WaveformConfig(
            sample_rate_hz=self._wf_fs_spin.value() * 1e6,
            bandwidth_hz=self._wf_bw_spin.value() * 1e6,
            chirp_duration_s=self._wf_chirp_spin.value() * 1e-6,
            center_freq_hz=self._wf_fc_spin.value() * 1e9,
        )

    def _update_waveform_res_label(self, n_samples: int, n_chirps: int) -> None:
        """Update the waveform resolution info label."""
        wfc = self._waveform_config_from_ui()
        r_res = wfc.range_resolution(n_samples)
        v_res = wfc.velocity_resolution(n_samples, n_chirps)
        n_r = min(64, n_samples)
        max_r = r_res * n_r
        self._wf_res_label.setText(
            f"Range: {r_res:.3f} m/bin  |  Vel: {v_res:.3f} m/s/bin  |  "
            f"Max range: {max_r:.1f} m ({n_r} bins)"
        )

    @pyqtSlot(str)
    def _on_playback_state_changed(self, state_str: str):
        if state_str == "playing":
            self._pb_play_btn.setText("Pause")
            self._replay_status_override = "playing"
        elif state_str == "paused":
            self._pb_play_btn.setText("Play")
            self._replay_status_override = "paused"
        elif state_str == "stopped":
            self._replay_status_override = None
            self._stop_radar()

    @pyqtSlot(int, int)
    def _on_frame_index_changed(self, current: int, total: int):
        self._pb_frame_label.setText(f"Frame: {current} / {total}")

    # =====================================================================
    # Demo mode
    # =====================================================================

    def _start_demo(self):
        if self._simulator:
            return
        # Mutual exclusion: do not start demo while radar/replay is running
        if self._running:
            logger.warning("Cannot start demo while radar is running")
            return
        self._simulator = TargetSimulator(self._radar_position, self)
        self._simulator.targetsUpdated.connect(self._on_demo_targets)
        self._simulator.start(500)
        self._demo_mode = True
        if not self._running:
            self._sb_mode.setText("Demo Mode")
            self._sb_status.setText("Demo mode active")
        self._demo_btn_main.setText("Stop Demo")
        self._demo_btn_map.setText("Stop Demo")
        self._demo_btn_map.setChecked(True)
        logger.info("Demo mode started")

    def _stop_demo(self):
        if self._simulator:
            self._simulator.stop()
            self._simulator = None
        self._demo_mode = False
        if not self._running:
            mode = "Idle"
        elif self._replay_controller is not None:
            mode = "Raw IQ Replay"
        elif isinstance(self._connection, ReplayConnection):
            mode = "Replay"
        else:
            # Use mode combo text for Mock vs Live distinction
            mode = self._mode_combo.currentText()
            if "Mock" not in mode and "Live" not in mode:
                mode = "Live"
        self._sb_mode.setText(mode)
        if not self._running:
            self._sb_status.setText("Demo stopped")
        self._demo_btn_main.setText("Start Demo")
        self._demo_btn_map.setText("Start Demo")
        self._demo_btn_map.setChecked(False)
        logger.info("Demo mode stopped")

    def _toggle_demo_main(self):
        if self._demo_mode:
            self._stop_demo()
        else:
            self._start_demo()

    def _toggle_demo_map(self, checked: bool):
        if checked:
            self._start_demo()
        else:
            self._stop_demo()

    def _add_demo_target(self):
        if self._simulator:
            self._simulator.add_random_target()
            logger.info("Added random demo target")

    # =====================================================================
    # Slots — data from workers / simulator
    # =====================================================================

    @pyqtSlot(object)
    def _on_frame_ready(self, frame: RadarFrame):
        """Handle a complete 64x32 radar frame from production acquisition."""
        self._current_frame = frame
        self._frame_count += 1

    @pyqtSlot(object)
    def _on_status_received(self, status: StatusResponse):
        """Handle FPGA status readback."""
        self._last_status = status
        self._update_status_display(status)

    @pyqtSlot(list)
    def _on_radar_targets(self, targets: list):
        self._current_targets = targets
        self._map_widget.set_targets(targets)

    @pyqtSlot(dict)
    def _on_radar_stats(self, stats: dict):
        self._last_stats = stats

    @pyqtSlot(str)
    def _on_worker_error(self, msg: str):
        logger.error(f"Worker error: {msg}")

    def _on_worker_finished(self):
        """Handle unexpected worker thread exit — recover UI to stopped state."""
        if self._running:
            logger.warning("Worker thread exited unexpectedly, resetting UI")
            self._stop_radar()

    @pyqtSlot(object)
    def _on_gps_received(self, gps: GPSData):
        self._gps_packet_count += 1
        self._radar_position.latitude = gps.latitude
        self._radar_position.longitude = gps.longitude
        self._radar_position.altitude = gps.altitude
        self._radar_position.pitch = gps.pitch
        self._radar_position.timestamp = gps.timestamp

        self._map_widget.set_radar_position(self._radar_position)

        if self._simulator:
            self._simulator.set_radar_position(self._radar_position)

    @pyqtSlot(list)
    def _on_demo_targets(self, targets: list):
        self._current_targets = targets
        self._map_widget.set_targets(targets)
        self._sb_targets.setText(f"Targets: {len(targets)}")

    def _on_target_selected(self, target_id: int):
        for t in self._current_targets:
            if t.id == target_id:
                self._show_target_info(t)
                break

    def _show_target_info(self, target: RadarTarget):
        status = ("Approaching" if target.velocity > 1
                  else ("Receding" if target.velocity < -1 else "Stationary"))
        color = (DARK_ERROR if status == "Approaching"
                 else (DARK_INFO if status == "Receding" else DARK_TEXT))
        info = (
            f"<b>Target #{target.id}</b><br><br>"
            f"<b>Track ID:</b> {target.track_id}<br>"
            f"<b>Range:</b> {target.range:.1f} m<br>"
            f"<b>Velocity:</b> {target.velocity:+.1f} m/s<br>"
            f"<b>Azimuth:</b> {target.azimuth:.1f}\u00b0<br>"
            f"<b>Elevation:</b> {target.elevation:.1f}\u00b0<br>"
            f"<b>SNR:</b> {target.snr:.1f} dB<br>"
            f"<b>Class:</b> {target.classification}<br>"
            f'<b>Status:</b> <span style="color: {color}">{status}</span>'
        )
        self._target_info_label.setText(info)

    # =====================================================================
    # FPGA Status display
    # =====================================================================

    def _update_status_display(self, st: StatusResponse):
        """Update FPGA status readback labels."""
        # Diagnostics tab
        lines = [
            f"Mode: {st.radar_mode}  Stream: {st.stream_ctrl:03b}  "
            f"Thresh: {st.cfar_threshold}",
            f"Long Chirp: {st.long_chirp}  Listen: {st.long_listen}",
            f"Guard: {st.guard}  Short Chirp: {st.short_chirp}  "
            f"Listen: {st.short_listen}",
            f"Chirps/Elev: {st.chirps_per_elev}  Range Mode: {st.range_mode}",
        ]
        self._fpga_status_label.setText("\n".join(lines))

        # Self-test labels
        if st.self_test_busy or st.self_test_flags:
            flags = st.self_test_flags
            self._st_labels["busy"].setText(
                f"Busy: {'YES' if st.self_test_busy else 'no'}")
            self._st_labels["flags"].setText(
                f"Flags: {flags:05b}")
            self._st_labels["detail"].setText(
                f"Detail: 0x{st.self_test_detail:02X}")
            self._st_labels["t0"].setText(
                f"T0 BRAM: {'PASS' if flags & 0x01 else 'FAIL'}")
            self._st_labels["t1"].setText(
                f"T1 CIC:  {'PASS' if flags & 0x02 else 'FAIL'}")
            self._st_labels["t2"].setText(
                f"T2 FFT:  {'PASS' if flags & 0x04 else 'FAIL'}")
            self._st_labels["t3"].setText(
                f"T3 Arith: {'PASS' if flags & 0x08 else 'FAIL'}")
            self._st_labels["t4"].setText(
                f"T4 ADC:  {'PASS' if flags & 0x10 else 'FAIL'}")

        # AGC status readback
        if hasattr(self, '_agc_labels'):
            agc_str = "AUTO" if st.agc_enable else "MANUAL"
            agc_color = DARK_SUCCESS if st.agc_enable else DARK_INFO
            self._agc_labels["enable"].setStyleSheet(
                f"color: {agc_color}; font-weight: bold;")
            self._agc_labels["enable"].setText(f"AGC: {agc_str}")
            self._agc_labels["gain"].setText(
                f"Gain: {st.agc_current_gain}")
            self._agc_labels["peak"].setText(
                f"Peak: {st.agc_peak_magnitude}")
            sat_color = DARK_ERROR if st.agc_saturation_count > 0 else DARK_INFO
            self._agc_labels["sat"].setStyleSheet(
                f"color: {sat_color}; font-weight: bold;")
            self._agc_labels["sat"].setText(
                f"Sat Count: {st.agc_saturation_count}")

        # AGC Monitor tab visualization
        self._update_agc_visualization(st)

    def _update_agc_visualization(self, st: StatusResponse):
        """Push AGC metrics into ring buffers and redraw AGC Monitor charts.

        Data is always accumulated (cheap), but matplotlib redraws are
        throttled to ``_AGC_REDRAW_INTERVAL`` seconds to avoid saturating
        the GUI event-loop when status packets arrive at 20 Hz.
        """
        if not hasattr(self, '_agc_canvas'):
            return

        # Push data into ring buffers (always — O(1))
        self._agc_gain_history.append(st.agc_current_gain)
        self._agc_peak_history.append(st.agc_peak_magnitude)
        self._agc_sat_history.append(st.agc_saturation_count)

        # Update indicator labels (cheap Qt calls)
        agc_str = "AUTO" if st.agc_enable else "MANUAL"
        agc_color = DARK_SUCCESS if st.agc_enable else DARK_INFO
        self._agc_mode_lbl.setStyleSheet(
            f"color: {agc_color}; font-size: 16px; font-weight: bold;")
        self._agc_mode_lbl.setText(f"AGC: {agc_str}")
        self._agc_gain_lbl.setText(f"Gain: {st.agc_current_gain}")
        self._agc_peak_lbl.setText(f"Peak: {st.agc_peak_magnitude}")

        total_sat = sum(self._agc_sat_history)
        if total_sat > 10:
            sat_color = DARK_ERROR
        elif total_sat > 0:
            sat_color = DARK_WARNING
        else:
            sat_color = DARK_SUCCESS
        self._agc_sat_total_lbl.setStyleSheet(
            f"color: {sat_color}; font-size: 14px; font-weight: bold;")
        self._agc_sat_total_lbl.setText(f"Total Saturations: {total_sat}")

        # ---- Throttle matplotlib redraws ---------------------------------
        now = time.monotonic()
        if now - self._agc_last_redraw < self._AGC_REDRAW_INTERVAL:
            return
        self._agc_last_redraw = now

        n = len(self._agc_gain_history)
        xs = list(range(n))

        # Update line plots
        gain_data = list(self._agc_gain_history)
        peak_data = list(self._agc_peak_history)
        sat_data = list(self._agc_sat_history)

        self._agc_gain_line.set_data(xs, gain_data)
        self._agc_peak_line.set_data(xs, peak_data)
        self._agc_sat_line.set_data(xs, sat_data)

        # Update saturation fill
        if self._agc_sat_fill_artist is not None:
            self._agc_sat_fill_artist.remove()
        if n > 0:
            self._agc_sat_fill_artist = self._agc_ax_sat.fill_between(
                xs, sat_data, color=DARK_ERROR, alpha=0.4)
        else:
            self._agc_sat_fill_artist = None

        # Auto-scale saturation y-axis
        max_sat = max(sat_data) if sat_data else 1
        self._agc_ax_sat.set_ylim(-1, max(max_sat * 1.3, 5))

        # Scroll x-axis
        self._agc_ax_gain.set_xlim(max(0, n - self._agc_history_len), n)

        self._agc_canvas.draw_idle()

    # =====================================================================
    # Position / coverage callbacks (map sidebar)
    # =====================================================================

    def _on_position_changed(self):
        self._radar_position.latitude = self._lat_spin.value()
        self._radar_position.longitude = self._lon_spin.value()
        self._radar_position.altitude = self._alt_spin.value()
        self._radar_position.heading = self._heading_spin.value()
        self._map_widget.set_radar_position(self._radar_position)
        if self._simulator:
            self._simulator.set_radar_position(self._radar_position)

    def _on_coverage_changed(self, value: float):
        radius_m = value * 1000
        self._settings.coverage_radius = radius_m
        self._map_widget.set_coverage_radius(radius_m)

    # =====================================================================
    # Settings
    # =====================================================================

    def _apply_processing_config(self):
        """Read host-side DSP controls into ProcessingConfig."""
        try:
            cfg = ProcessingConfig(
                clustering_enabled=self._cluster_check.isChecked(),
                clustering_eps=self._cluster_eps_spin.value(),
                clustering_min_samples=self._cluster_min_spin.value(),
                tracking_enabled=self._tracking_check.isChecked(),
            )
            self._processing_config = cfg
            self._processor.set_config(cfg)
            logger.info(
                f"Host DSP config: Clustering={cfg.clustering_enabled}, "
                f"Tracking={cfg.tracking_enabled}"
            )
            QMessageBox.information(self, "Settings", "Host DSP settings applied.")
        except RuntimeError as e:
            QMessageBox.critical(self, "Error",
                                 f"Failed to apply DSP settings: {e}")
            logger.error(f"DSP config error: {e}")

    # =====================================================================
    # Periodic GUI refresh (100 ms timer)
    # =====================================================================

    def _refresh_gui(self):
        try:
            # GPS label
            gps = self._radar_position
            self._gps_label.setText(
                f"GPS: Lat {gps.latitude:.6f}, Lon {gps.longitude:.6f}, "
                f"Alt {gps.altitude:.1f}m"
            )

            # Pitch label with colour coding
            pitch_text = f"Pitch: {gps.pitch:+.1f}\u00b0"
            self._pitch_label.setText(pitch_text)
            if abs(gps.pitch) > 10:
                self._pitch_label.setStyleSheet(
                    f"color: {DARK_ERROR}; font-weight: bold;")
            elif abs(gps.pitch) > 5:
                self._pitch_label.setStyleSheet(
                    f"color: {DARK_WARNING}; font-weight: bold;")
            else:
                self._pitch_label.setStyleSheet(
                    f"color: {DARK_SUCCESS}; font-weight: bold;")

            # Range-Doppler map from current frame
            if self._current_frame is not None:
                self._rdm_canvas.update_map(
                    self._current_frame.magnitude,
                    self._current_frame.detections,
                )

            # Targets table (main tab)
            self._update_main_targets_table()

            # Status label (main tab)
            if self._running:
                det = (self._current_frame.detection_count
                       if self._current_frame else 0)
                # Preserve replay-specific status (paused/playing)
                if self._replay_status_override == "paused":
                    self._status_label_main.setText(
                        f"Status: Raw IQ Replay (paused) \u2014 "
                        f"Frames: {self._frame_count} \u2014 Detections: {det}"
                    )
                else:
                    self._status_label_main.setText(
                        f"Status: Running \u2014 Frames: {self._frame_count} "
                        f"\u2014 Detections: {det}"
                    )

            # Diagnostics values
            self._update_diagnostics()

            # Status-bar target count
            self._sb_targets.setText(f"Targets: {len(self._current_targets)}")

        except (RuntimeError, ValueError, IndexError) as e:
            logger.error(f"GUI refresh error: {e}")

    def _update_main_targets_table(self):
        targets = self._current_targets[-20:]  # last 20
        self._targets_table_main.setRowCount(len(targets))

        for row, t in enumerate(targets):
            self._targets_table_main.setItem(
                row, 0, QTableWidgetItem(f"{t.range:.0f}"))
            self._targets_table_main.setItem(
                row, 1, QTableWidgetItem(f"{t.velocity:.0f}"))

            mag_val = 10 ** (t.snr / 10) if t.snr > 0 else 0
            self._targets_table_main.setItem(
                row, 2, QTableWidgetItem(f"{mag_val:.0f}"))
            self._targets_table_main.setItem(
                row, 3, QTableWidgetItem(f"{t.snr:.1f}"))
            self._targets_table_main.setItem(
                row, 4, QTableWidgetItem(str(t.track_id)))

    def _update_diagnostics(self):
        # Connection indicators
        conn_open = (self._connection is not None and self._connection.is_open)
        self._set_conn_indicator(self._conn_ft2232h, conn_open)
        self._set_conn_indicator(self._conn_stm32, self._stm32.is_open)

        gps_count = self._gps_packet_count
        if self._gps_worker:
            gps_count = self._gps_worker.gps_count

        uptime = time.time() - self._start_time
        frame_rate = self._frame_count / max(uptime, 1)
        det = (self._current_frame.detection_count
               if self._current_frame else 0)

        vals = [
            str(self._frame_count),
            str(det),
            str(gps_count),
            str(self._last_stats.get("errors", 0)),
            f"{uptime:.0f}s",
            f"{frame_rate:.1f}/s",
        ]
        for lbl, v in zip(self._diag_values, vals, strict=False):
            lbl.setText(v)

    # =====================================================================
    # Helpers
    # =====================================================================

    @staticmethod
    def _make_status_label(_name: str) -> QLabel:
        lbl = QLabel("Disconnected")
        lbl.setStyleSheet(f"color: {DARK_ERROR}; font-weight: bold;")
        return lbl

    @staticmethod
    def _set_conn_indicator(label: QLabel, connected: bool):
        if connected:
            label.setText("Connected")
            label.setStyleSheet(f"color: {DARK_SUCCESS}; font-weight: bold;")
        else:
            label.setText("Disconnected")
            label.setStyleSheet(f"color: {DARK_ERROR}; font-weight: bold;")

    def _log_append(self, message: str):
        """Append a log message to the diagnostics log viewer."""
        self._log_text.appendPlainText(message)

    # =====================================================================
    # Close event
    # =====================================================================

    def closeEvent(self, event):
        if self._simulator:
            self._simulator.stop()
        if self._replay_controller is not None:
            self._replay_controller.stop()
        if self._replay_worker is not None:
            self._replay_worker.stop()
            self._replay_worker.wait(1000)
        if self._radar_worker:
            self._radar_worker.stop()
            self._radar_worker.wait(1000)
        if self._gps_worker:
            self._gps_worker.stop()
            self._gps_worker.wait(1000)
        if self._connection:
            self._connection.close()
        self._stm32.close()
        logging.getLogger().removeHandler(self._log_handler)
        event.accept()


# =============================================================================
# Qt-compatible log handler (routes Python logging -> QTextEdit via signal)
# =============================================================================


class _LogSignalBridge(QObject):
    """Thread-safe bridge: emits a Qt signal so the slot runs on the GUI thread."""

    log_message = pyqtSignal(str)


class _QtLogHandler(logging.Handler):
    """Sends log records to a QObject signal (safe from any thread)."""

    def __init__(self, bridge: _LogSignalBridge):
        super().__init__()
        self._bridge = bridge
        self.setFormatter(logging.Formatter(
            "%(asctime)s  %(levelname)-8s  %(message)s",
            datefmt="%H:%M:%S",
        ))

    def emit(self, record):
        try:
            msg = self.format(record)
            self._bridge.log_message.emit(msg)
        except RuntimeError:
            pass
