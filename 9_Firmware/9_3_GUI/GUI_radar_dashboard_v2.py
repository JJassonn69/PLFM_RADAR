#!/usr/bin/env python3
"""
AERIS-10 Radar Dashboard V2
============================
Unified dashboard combining the best features from all legacy GUI versions
(V1–V6, V4_2_CSV) with the radar_protocol.py FT601 pipeline.

Features ported from legacy GUIs:
  - Kalman filter tracking with DBSCAN clustering (V2–V6)
  - GPS + IMU pitch correction on elevation angles (V3–V6)
  - Google Maps HTML overlay with target plotting (V4–V6)
  - Multi-PRF velocity unwrapping via CRT (V2–V6)
  - Settings validation with firmware-mirrored range checks (V5)
  - CSV offline analysis: host-side range FFT, Doppler FFT, MTI, CFAR (V4_2)

Features from radar_dashboard.py (V1):
  - FT601 USB reader with packet parsing (matches usb_data_interface.v)
  - Real-time range-Doppler magnitude heatmap (64×32)
  - CFAR detection overlay (flagged cells highlighted)
  - Range profile waterfall plot (range vs. time)
  - Host command sender (opcodes 0x01–0x27, 0x30, 0xFF)
  - HDF5 data recording for offline analysis
  - Mock / Replay / Live connection modes
  - Catppuccin Mocha dark theme

Usage:
  python GUI_radar_dashboard_v2.py              # Mock mode
  python GUI_radar_dashboard_v2.py --live       # FT601 hardware
  python GUI_radar_dashboard_v2.py --replay DIR # Replay .npy data
  python GUI_radar_dashboard_v2.py --record     # Auto-start HDF5 recording
"""

import sys
import os
import time
import queue
import logging
import argparse
import threading
import math
import struct
import tempfile
import webbrowser
from typing import Optional, Dict, List, Tuple, Any
from collections import deque
from dataclasses import dataclass, field

import numpy as np

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from radar_protocol import (
    RadarProtocol, FT601Connection, ReplayConnection,
    DataRecorder, RadarAcquisition,
    RadarFrame, StatusResponse, Opcode,
    NUM_RANGE_BINS, NUM_DOPPLER_BINS, WATERFALL_DEPTH,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("radar_dashboard_v2")


# ============================================================================
# Data Classes (ported from V2–V6)
# ============================================================================

@dataclass
class GPSData:
    """GPS + IMU data received from STM32 via USB CDC (V3–V6)."""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    pitch: float = 0.0
    timestamp: float = 0.0


@dataclass
class RadarTarget:
    """Tracked radar target (V2–V6)."""
    id: int = 0
    range_m: float = 0.0
    velocity: float = 0.0
    azimuth: int = 0
    elevation: int = 0
    corrected_elevation: float = 0.0
    snr: float = 0.0
    timestamp: float = 0.0
    track_id: int = -1


# ============================================================================
# Settings Validation (ported from V5)
# ============================================================================

RADAR_SETTINGS_LIMITS = {
    "system_frequency": (1e9, 20e9),
    "chirp_duration": (0.1e-6, 1e-3),
    "chirps_per_position": (1, 1024),
    "freq_min": (1e6, 500e6),
    "freq_max": (1e6, 500e6),
    "prf1": (100, 100000),
    "prf2": (100, 100000),
    "max_distance": (100, 200000),
}


def validate_radar_settings(settings: Dict[str, float]) -> List[str]:
    """
    Validate radar settings against firmware-side range checks (V5).
    Returns list of error strings; empty list means valid.
    """
    errors = []
    for field_name, (minimum, maximum) in RADAR_SETTINGS_LIMITS.items():
        if field_name in settings:
            value = settings[field_name]
            if value < minimum or value > maximum:
                errors.append(
                    f"{field_name} must be between {minimum:g} and {maximum:g} "
                    f"(got {value:g})")
    if "freq_max" in settings and "freq_min" in settings:
        if settings["freq_max"] <= settings["freq_min"]:
            errors.append("freq_max must be greater than freq_min")
    return errors


# ============================================================================
# Multi-PRF Velocity Unwrapping (ported from V2–V6)
# ============================================================================

def multi_prf_unwrap(doppler_measurements: List[float],
                     prf1: float, prf2: float,
                     center_freq: float = 10.5e9) -> List[float]:
    """
    Multi-PRF velocity unwrapping using Chinese Remainder Theorem (V2–V6).
    Resolves Doppler ambiguity by comparing measurements at two PRFs.
    """
    C = 3e8
    wavelength = C / center_freq
    v_max1 = prf1 * wavelength / 2
    v_max2 = prf2 * wavelength / 2

    unwrapped = []
    for doppler in doppler_measurements:
        v1 = doppler * wavelength / 2
        v2 = doppler * wavelength / 2
        velocity = _solve_chinese_remainder(v1, v2, v_max1, v_max2)
        unwrapped.append(velocity)
    return unwrapped


def _solve_chinese_remainder(v1: float, v2: float,
                             max1: float, max2: float) -> float:
    """CRT solver for velocity ambiguity (V2–V6)."""
    for k in range(-5, 6):
        candidate = v1 + k * max1
        if abs(candidate - v2) < max2 / 2:
            return candidate
    return v1


# ============================================================================
# Pitch Correction (ported from V3–V6)
# ============================================================================

def apply_pitch_correction(raw_elevation: float, pitch_angle: float) -> float:
    """
    Apply IMU pitch correction to elevation angle (V3–V6).
    raw_elevation: measured elevation from radar (degrees)
    pitch_angle: antenna pitch angle from IMU (degrees)
    Returns corrected elevation angle (degrees).
    """
    raw_elev_rad = math.radians(raw_elevation)
    pitch_rad = math.radians(pitch_angle)
    corrected_rad = raw_elev_rad - pitch_rad
    corrected_deg = math.degrees(corrected_rad)
    corrected_deg = corrected_deg % 180
    if corrected_deg < 0:
        corrected_deg += 180
    return corrected_deg


# ============================================================================
# Target Tracker — DBSCAN + Kalman Filter (ported from V2–V6)
# ============================================================================

class TargetTracker:
    """
    Host-side target tracker using DBSCAN clustering and Kalman filtering.
    Ported from V2–V6 RadarProcessor. Uses numpy-only Kalman (no filterpy dep).
    """

    def __init__(self):
        self.tracks: Dict[int, Dict[str, Any]] = {}
        self._next_track_id = 0

    def update(self, detections: List[RadarTarget],
               current_gps: Optional[GPSData] = None) -> List[RadarTarget]:
        """
        Process new detections: cluster, associate to existing tracks,
        and update Kalman filters. Returns list with track_id assigned.
        """
        if not detections:
            self._prune_stale()
            return []

        # Apply pitch correction if GPS data available
        if current_gps and current_gps.pitch != 0.0:
            for d in detections:
                d.corrected_elevation = apply_pitch_correction(
                    d.elevation, current_gps.pitch)

        # Cluster detections (simple proximity-based, no sklearn dependency)
        clusters = self._cluster_detections(detections)

        # Associate clusters to existing tracks
        associated = self._associate(clusters)

        # Update Kalman filters
        self._kalman_update(associated)

        self._prune_stale()
        return associated

    def _cluster_detections(self, detections: List[RadarTarget],
                            eps: float = 50.0) -> List[RadarTarget]:
        """Simple proximity clustering (replaces DBSCAN to avoid sklearn dep)."""
        if len(detections) <= 1:
            return detections

        # Greedy nearest-neighbor merge
        used = [False] * len(detections)
        merged = []
        for i, d in enumerate(detections):
            if used[i]:
                continue
            cluster_range = [d.range_m]
            cluster_vel = [d.velocity]
            cluster_snr = [d.snr]
            used[i] = True
            for j in range(i + 1, len(detections)):
                if used[j]:
                    continue
                dist = math.sqrt((d.range_m - detections[j].range_m) ** 2 +
                                 (d.velocity - detections[j].velocity) ** 2)
                if dist < eps:
                    cluster_range.append(detections[j].range_m)
                    cluster_vel.append(detections[j].velocity)
                    cluster_snr.append(detections[j].snr)
                    used[j] = True
            centroid = RadarTarget(
                range_m=float(np.mean(cluster_range)),
                velocity=float(np.mean(cluster_vel)),
                snr=float(np.max(cluster_snr)),
                azimuth=d.azimuth,
                elevation=d.elevation,
                corrected_elevation=d.corrected_elevation,
                timestamp=d.timestamp,
            )
            merged.append(centroid)
        return merged

    def _associate(self, detections: List[RadarTarget]) -> List[RadarTarget]:
        """Nearest-neighbor track association (V2–V6)."""
        for det in detections:
            best_track_id = None
            min_dist = float("inf")
            for tid, track in self.tracks.items():
                state = track["state"]
                dist = math.sqrt((det.range_m - state[0]) ** 2 +
                                 (det.velocity - state[2]) ** 2)
                if dist < min_dist and dist < 500:
                    min_dist = dist
                    best_track_id = tid
            if best_track_id is not None:
                det.track_id = best_track_id
            else:
                det.track_id = self._next_track_id
                self._next_track_id += 1
        return detections

    def _kalman_update(self, detections: List[RadarTarget]):
        """4-state Kalman filter update (V2–V6, numpy-only)."""
        current_time = time.time()
        F = np.array([[1, 1, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 1],
                      [0, 0, 0, 1]], dtype=np.float64)
        H = np.array([[1, 0, 0, 0],
                      [0, 0, 1, 0]], dtype=np.float64)
        R = np.diag([10.0, 1.0])
        Q = np.eye(4) * 0.1

        for det in detections:
            tid = det.track_id
            if tid not in self.tracks:
                x = np.array([det.range_m, 0.0, det.velocity, 0.0])
                P = np.eye(4) * 1000.0
                self.tracks[tid] = {
                    "state": x, "P": P,
                    "last_update": current_time, "hits": 1
                }
            else:
                track = self.tracks[tid]
                x = track["state"]
                P = track["P"]
                # Predict
                x = F @ x
                P = F @ P @ F.T + Q
                # Update
                z = np.array([det.range_m, det.velocity])
                y = z - H @ x
                S = H @ P @ H.T + R
                K = P @ H.T @ np.linalg.inv(S)
                x = x + K @ y
                P = (np.eye(4) - K @ H) @ P
                track["state"] = x
                track["P"] = P
                track["last_update"] = current_time
                track["hits"] += 1

    def _prune_stale(self, timeout: float = 5.0):
        """Remove tracks not updated within timeout (V2–V6)."""
        now = time.time()
        stale = [tid for tid, t in self.tracks.items()
                 if now - t["last_update"] > timeout]
        for tid in stale:
            del self.tracks[tid]


# ============================================================================
# Google Maps Generator (ported from V4–V6)
# ============================================================================

class MapGenerator:
    """Generate Google Maps HTML overlay with radar position and targets (V4–V6)."""

    MAP_TEMPLATE = """<!DOCTYPE html>
<html><head><title>AERIS-10 Radar Map</title>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>#map {{ height: 100vh; width: 100%; }}</style>
</head><body><div id="map"></div>
<script>
function initMap() {{
  var pos = {{lat: {lat}, lng: {lon}}};
  var map = new google.maps.Map(document.getElementById('map'), {{
    center: pos, zoom: 12, mapTypeId: google.maps.MapTypeId.ROADMAP
  }});
  new google.maps.Marker({{position: pos, map: map, title: 'Radar',
    icon: {{path: google.maps.SymbolPath.CIRCLE, scale: 8,
            fillColor: '#FF0000', fillOpacity: 1,
            strokeColor: '#FFFFFF', strokeWeight: 2}}
  }});
  new google.maps.Circle({{strokeColor: '#FF0000', strokeOpacity: 0.8,
    strokeWeight: 2, fillColor: '#FF0000', fillOpacity: 0.1,
    map: map, center: pos, radius: {radius}
  }});
  var info = new google.maps.InfoWindow({{
    content: '<b>AERIS-10</b><br>Lat: {lat:.6f}<br>Lon: {lon:.6f}<br>' +
             'Alt: {alt:.1f}m<br>Pitch: {pitch:+.1f}&deg;'
  }});
  {targets_js}
}}
</script>
<script async defer src="https://maps.googleapis.com/maps/api/js?key={api_key}&callback=initMap"></script>
</body></html>"""

    @staticmethod
    def generate(gps: GPSData, targets: List[RadarTarget],
                 coverage_radius: float = 50000.0,
                 api_key: str = "YOUR_KEY") -> str:
        """Generate HTML string for Google Maps with radar + targets."""
        targets_js = ""
        for i, t in enumerate(targets):
            # Approximate target lat/lon from range + azimuth
            bearing_rad = math.radians(t.azimuth)
            d_lat = (t.range_m * math.cos(bearing_rad)) / 111320.0
            d_lon = (t.range_m * math.sin(bearing_rad)) / (
                111320.0 * math.cos(math.radians(gps.latitude)))
            t_lat = gps.latitude + d_lat
            t_lon = gps.longitude + d_lon
            targets_js += (
                f"new google.maps.Marker({{position: {{lat:{t_lat:.6f},"
                f"lng:{t_lon:.6f}}}, map: map, title: 'T{i}: "
                f"{t.range_m:.0f}m {t.velocity:.1f}m/s', "
                f"icon: {{path: google.maps.SymbolPath.CIRCLE, scale: 5, "
                f"fillColor: '#0000FF', fillOpacity: 0.8, "
                f"strokeColor: '#FFF', strokeWeight: 1}} }});\n"
            )
        return MapGenerator.MAP_TEMPLATE.format(
            lat=gps.latitude, lon=gps.longitude, alt=gps.altitude,
            pitch=gps.pitch, radius=coverage_radius,
            api_key=api_key, targets_js=targets_js)


# ============================================================================
# CSV Signal Processor (ported from V4_2_CSV)
# ============================================================================

class CSVSignalProcessor:
    """
    Offline signal processor for CSV data files (V4_2_CSV).
    Provides host-side range FFT, Doppler FFT, MTI, and CFAR.
    """

    @staticmethod
    def range_fft(iq_data: np.ndarray, bw: float = 500e6
                  ) -> Tuple[np.ndarray, np.ndarray]:
        """Range FFT on complex IQ data. Returns (range_axis_m, magnitude)."""
        N = len(iq_data)
        window = np.hanning(N)
        windowed = iq_data * window
        spectrum = np.fft.fft(windowed)
        C = 3e8
        range_max = (C * N) / (2 * bw)
        range_axis = np.linspace(0, range_max, N)
        return range_axis, np.abs(spectrum)

    @staticmethod
    def doppler_fft(iq_data: np.ndarray, fs: float = 100e6,
                    center_freq: float = 10.5e9
                    ) -> Tuple[np.ndarray, np.ndarray]:
        """Doppler FFT on complex IQ data. Returns (velocity_axis, magnitude)."""
        N = len(iq_data)
        window = np.hanning(N)
        windowed = iq_data * window
        spectrum = np.fft.fftshift(np.fft.fft(windowed))
        freq_axis = np.linspace(-fs / 2, fs / 2, N)
        wavelength = 3e8 / center_freq
        velocity_axis = freq_axis * wavelength / 2
        return velocity_axis, np.abs(spectrum)

    @staticmethod
    def mti_filter(iq_data: np.ndarray,
                   mode: str = "single") -> np.ndarray:
        """MTI clutter canceller (V4_2_CSV). Returns filtered complex data."""
        if iq_data is None or len(iq_data) < 2:
            return np.array([], dtype=complex)
        data = iq_data.astype(complex)
        if mode == "single":
            return data[1:] - data[:-1]
        elif mode == "double":
            if len(data) < 3:
                return np.array([], dtype=complex)
            return data[2:] - 2 * data[1:-1] + data[:-2]
        return data

    @staticmethod
    def cfar_detect(profile: np.ndarray, guard: int = 2,
                    train: int = 10, alpha: float = 3.0
                    ) -> List[Tuple[int, float]]:
        """CA-CFAR detection on 1D profile (V4_2_CSV). Returns [(bin, mag)]."""
        N = len(profile)
        detections = []
        window = guard + train
        for i in range(window, N - window):
            lead = profile[i - guard - train:i - guard]
            lag = profile[i + guard + 1:i + guard + train + 1]
            noise = np.mean(np.concatenate([lead, lag]))
            threshold = noise * alpha
            if profile[i] > threshold:
                detections.append((i, float(profile[i])))
        return detections


# ============================================================================
# Dark Theme Colors (Catppuccin Mocha)
# ============================================================================

BG = "#1e1e2e"
BG2 = "#282840"
FG = "#cdd6f4"
ACCENT = "#89b4fa"
GREEN = "#a6e3a1"
RED = "#f38ba8"
YELLOW = "#f9e2af"
SURFACE = "#313244"
PEACH = "#fab387"


# ============================================================================
# Main Dashboard V2
# ============================================================================

class RadarDashboardV2:
    """
    Unified radar dashboard combining all legacy GUI features
    with the FT601 protocol pipeline.
    """

    UPDATE_INTERVAL_MS = 100  # 10 Hz display refresh

    AERIS10_CONFIG = {
        "sample_rate": 400e6,
        "bandwidth": 500e6,
        "ramp_time": 300e-6,
        "center_freq": 10.5e9,
        "fft_size": 1024,
        "decimation": 16,
        "num_chirps": 32,
        "range_formula": "if",
    }

    C = 3e8

    def __init__(self, root: tk.Tk, connection, recorder: DataRecorder):
        self.root = root
        self.conn = connection
        self.recorder = recorder

        if hasattr(connection, "radar_config"):
            self._radar_cfg = connection.radar_config
        else:
            self._radar_cfg = dict(self.AERIS10_CONFIG)

        self.root.title("AERIS-10 Radar Dashboard V2")
        self.root.geometry("1700x1000")
        self.root.configure(bg=BG)

        # Frame queue (acquisition → display)
        self.frame_queue: queue.Queue[RadarFrame] = queue.Queue(maxsize=8)
        self._acq_thread: Optional[RadarAcquisition] = None

        # Display state
        self._current_frame = RadarFrame()
        self._waterfall = deque(maxlen=WATERFALL_DEPTH)
        for _ in range(WATERFALL_DEPTH):
            self._waterfall.append(np.zeros(NUM_RANGE_BINS))

        self._frame_count = 0
        self._fps_ts = time.time()
        self._fps = 0.0
        self._vmax_ema = 1000.0
        self._vmax_alpha = 0.15
        self._pending_status: Optional[StatusResponse] = None

        # New V2 state
        self._tracker = TargetTracker()
        self._current_gps = GPSData(latitude=41.9028, longitude=12.4964)
        self._tracked_targets: List[RadarTarget] = []
        self._map_file_path: Optional[str] = None
        self._google_maps_api_key = "YOUR_GOOGLE_MAPS_API_KEY"

        self._build_ui()
        self._schedule_update()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=FG, fieldbackground=SURFACE)
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=FG)
        style.configure("TButton", background=SURFACE, foreground=FG)
        style.configure("TLabelframe", background=BG, foreground=ACCENT)
        style.configure("TLabelframe.Label", background=BG, foreground=ACCENT)
        style.configure("Accent.TButton", background=ACCENT, foreground=BG)
        style.configure("TNotebook", background=BG)
        style.configure("TNotebook.Tab", background=SURFACE, foreground=FG,
                         padding=[12, 4])
        style.map("TNotebook.Tab", background=[("selected", ACCENT)],
                  foreground=[("selected", BG)])

        # Top bar
        top = ttk.Frame(self.root)
        top.pack(fill="x", padx=8, pady=(8, 0))

        self.lbl_status = ttk.Label(top, text="DISCONNECTED", foreground=RED,
                                     font=("Menlo", 11, "bold"))
        self.lbl_status.pack(side="left", padx=8)

        self.lbl_fps = ttk.Label(top, text="0.0 fps", font=("Menlo", 10))
        self.lbl_fps.pack(side="left", padx=16)

        self.lbl_detections = ttk.Label(top, text="Det: 0", font=("Menlo", 10))
        self.lbl_detections.pack(side="left", padx=16)

        self.lbl_frame = ttk.Label(top, text="Frame: 0", font=("Menlo", 10))
        self.lbl_frame.pack(side="left", padx=16)

        self.lbl_tracks = ttk.Label(top, text="Tracks: 0", font=("Menlo", 10))
        self.lbl_tracks.pack(side="left", padx=16)

        self.lbl_gps = ttk.Label(top, text="GPS: --", font=("Menlo", 9))
        self.lbl_gps.pack(side="left", padx=16)

        self.lbl_pitch = ttk.Label(top, text="Pitch: --", font=("Menlo", 9))
        self.lbl_pitch.pack(side="left", padx=8)

        self.btn_connect = ttk.Button(top, text="Connect",
                                       command=self._on_connect,
                                       style="Accent.TButton")
        self.btn_connect.pack(side="right", padx=4)

        self.btn_record = ttk.Button(top, text="Record", command=self._on_record)
        self.btn_record.pack(side="right", padx=4)

        # Notebook (tabs)
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=8, pady=8)

        tab_display = ttk.Frame(nb)
        tab_targets = ttk.Frame(nb)
        tab_map = ttk.Frame(nb)
        tab_control = ttk.Frame(nb)
        tab_csv = ttk.Frame(nb)
        tab_log = ttk.Frame(nb)
        nb.add(tab_display, text="  Display  ")
        nb.add(tab_targets, text="  Targets  ")
        nb.add(tab_map, text="  Map  ")
        nb.add(tab_control, text="  Control  ")
        nb.add(tab_csv, text="  CSV Analysis  ")
        nb.add(tab_log, text="  Log  ")

        self._build_display_tab(tab_display)
        self._build_targets_tab(tab_targets)
        self._build_map_tab(tab_map)
        self._build_control_tab(tab_control)
        self._build_csv_tab(tab_csv)
        self._build_log_tab(tab_log)

    # -------------------------------------------------------- Display Tab
    def _build_display_tab(self, parent):
        cfg = self._radar_cfg
        sample_rate = cfg["sample_rate"]
        bandwidth = cfg["bandwidth"]
        ramp_time = cfg["ramp_time"]
        center_freq = cfg["center_freq"]
        fft_size = cfg["fft_size"]
        decimation = cfg["decimation"]
        formula = cfg.get("range_formula", "baseband")

        if formula == "if":
            range_per_fft_bin = self.C / (2.0 * bandwidth)
        else:
            range_per_fft_bin = (sample_rate / fft_size) * self.C * ramp_time / (2.0 * bandwidth)

        range_per_bin = range_per_fft_bin * decimation
        max_range = range_per_bin * NUM_RANGE_BINS
        wavelength = self.C / center_freq
        max_vel = wavelength / (4.0 * ramp_time)
        vel_per_bin = 2.0 * max_vel / NUM_DOPPLER_BINS
        vel_lo = -max_vel
        vel_hi = max_vel

        log.info(f"Axis: range_per_bin={range_per_bin:.3f}m, "
                 f"max_range={max_range:.1f}m, max_vel={max_vel:.1f}m/s")

        self.fig = Figure(figsize=(14, 7), facecolor=BG)
        self.fig.subplots_adjust(left=0.07, right=0.98, top=0.94, bottom=0.10,
                                  wspace=0.30, hspace=0.35)

        # Range-Doppler heatmap
        self.ax_rd = self.fig.add_subplot(1, 3, (1, 2))
        self.ax_rd.set_facecolor(BG2)
        self._rd_img = self.ax_rd.imshow(
            np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS)),
            aspect="auto", cmap="inferno", origin="lower",
            extent=[vel_lo, vel_hi, 0, max_range], vmin=0, vmax=1000)
        self.ax_rd.set_title("Range-Doppler Map", color=FG, fontsize=12)
        self.ax_rd.set_xlabel("Velocity (m/s)", color=FG)
        self.ax_rd.set_ylabel("Range (m)", color=FG)
        self.ax_rd.tick_params(colors=FG)

        self._vel_lo = vel_lo
        self._vel_hi = vel_hi
        self._max_range = max_range
        self._range_per_bin = range_per_bin
        self._vel_per_bin = vel_per_bin

        self._det_scatter = self.ax_rd.scatter([], [], s=30, c=GREEN,
                                                marker="x", linewidths=1.5,
                                                zorder=5, label="CFAR Det")
        # Tracked targets overlay (larger circles, different color)
        self._track_scatter = self.ax_rd.scatter([], [], s=80, c=PEACH,
                                                  marker="o", linewidths=1.5,
                                                  edgecolors=PEACH,
                                                  facecolors="none",
                                                  zorder=6, label="Tracked")

        # Waterfall
        self.ax_wf = self.fig.add_subplot(1, 3, 3)
        self.ax_wf.set_facecolor(BG2)
        wf_init = np.zeros((WATERFALL_DEPTH, NUM_RANGE_BINS))
        self._wf_img = self.ax_wf.imshow(
            wf_init, aspect="auto", cmap="viridis", origin="lower",
            extent=[0, max_range, 0, WATERFALL_DEPTH], vmin=0, vmax=5000)
        self.ax_wf.set_title("Range Waterfall", color=FG, fontsize=12)
        self.ax_wf.set_xlabel("Range (m)", color=FG)
        self.ax_wf.set_ylabel("Frame", color=FG)
        self.ax_wf.tick_params(colors=FG)

        canvas = FigureCanvasTkAgg(self.fig, master=parent)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas

    # -------------------------------------------------------- Targets Tab
    def _build_targets_tab(self, parent):
        """Target list with tracking info and corrected elevation (V3–V6)."""
        cols = ("TrackID", "Range", "Velocity", "Azimuth", "RawElev",
                "CorrElev", "SNR", "Hits")
        self._targets_tree = ttk.Treeview(parent, columns=cols,
                                           show="headings", height=20)
        headings = {
            "TrackID": ("Track", 60), "Range": ("Range (m)", 90),
            "Velocity": ("Vel (m/s)", 90), "Azimuth": ("Az", 60),
            "RawElev": ("Raw El", 60), "CorrElev": ("Corr El", 70),
            "SNR": ("SNR (dB)", 70), "Hits": ("Hits", 50),
        }
        for col, (text, width) in headings.items():
            self._targets_tree.heading(col, text=text)
            self._targets_tree.column(col, width=width, anchor="center")

        scroll = ttk.Scrollbar(parent, orient="vertical",
                                command=self._targets_tree.yview)
        self._targets_tree.configure(yscrollcommand=scroll.set)
        self._targets_tree.pack(side="left", fill="both", expand=True,
                                 padx=8, pady=8)
        scroll.pack(side="right", fill="y", padx=(0, 8), pady=8)

    # -------------------------------------------------------- Map Tab
    def _build_map_tab(self, parent):
        """Google Maps tab (V4–V6)."""
        controls = ttk.Frame(parent)
        controls.pack(fill="x", padx=10, pady=5)

        ttk.Button(controls, text="Open Map in Browser",
                   command=self._open_map_browser).pack(side="left", padx=5)
        ttk.Button(controls, text="Refresh Map",
                   command=self._generate_map).pack(side="left", padx=5)

        self._map_status = ttk.Label(controls, text="Map: Ready")
        self._map_status.pack(side="left", padx=20)

        # GPS entry (for mock/testing)
        gps_frame = ttk.LabelFrame(parent, text="GPS Position (manual)", padding=8)
        gps_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(gps_frame, text="Lat:").grid(row=0, column=0, padx=2)
        self._var_lat = tk.StringVar(value="41.9028")
        ttk.Entry(gps_frame, textvariable=self._var_lat, width=12).grid(
            row=0, column=1, padx=2)
        ttk.Label(gps_frame, text="Lon:").grid(row=0, column=2, padx=2)
        self._var_lon = tk.StringVar(value="12.4964")
        ttk.Entry(gps_frame, textvariable=self._var_lon, width=12).grid(
            row=0, column=3, padx=2)
        ttk.Label(gps_frame, text="API Key:").grid(row=0, column=4, padx=2)
        self._var_api_key = tk.StringVar(value="YOUR_GOOGLE_MAPS_API_KEY")
        ttk.Entry(gps_frame, textvariable=self._var_api_key, width=30).grid(
            row=0, column=5, padx=2)
        ttk.Button(gps_frame, text="Update GPS",
                   command=self._update_gps_manual).grid(row=0, column=6, padx=5)

        info = ttk.Frame(parent)
        info.pack(fill="x", padx=10, pady=5)
        self._map_info = ttk.Label(info, text="No GPS data yet", font=("Menlo", 10))
        self._map_info.pack()

    # -------------------------------------------------------- Control Tab
    def _build_control_tab(self, parent):
        """Host command sender and configuration panel (from V1)."""
        outer = ttk.Frame(parent)
        outer.pack(fill="both", expand=True, padx=16, pady=16)

        # Left: Quick actions
        left = ttk.LabelFrame(outer, text="Quick Actions", padding=12)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        buttons = [
            ("Trigger Chirp (0x01)", 0x01, 1),
            ("Enable MTI (0x26)", 0x26, 1),
            ("Disable MTI (0x26)", 0x26, 0),
            ("Enable CFAR (0x25)", 0x25, 1),
            ("Disable CFAR (0x25)", 0x25, 0),
            ("Request Status (0xFF)", 0xFF, 0),
        ]
        for text, op, val in buttons:
            ttk.Button(left, text=text,
                       command=lambda o=op, v=val: self._send_cmd(o, v)
                       ).pack(fill="x", pady=3)

        ttk.Separator(left, orient="horizontal").pack(fill="x", pady=6)
        ttk.Label(left, text="FPGA Self-Test", font=("Menlo", 10, "bold")).pack(
            anchor="w", pady=(2, 0))
        ttk.Button(left, text="Run Self-Test (0x30)",
                   command=lambda: self._send_cmd(0x30, 1)).pack(fill="x", pady=3)
        ttk.Button(left, text="Read Result (0x31)",
                   command=lambda: self._send_cmd(0x31, 0)).pack(fill="x", pady=3)

        # Self-test display
        st_frame = ttk.LabelFrame(left, text="Self-Test Results", padding=6)
        st_frame.pack(fill="x", pady=(6, 0))
        self._st_labels = {}
        for name, text in [("busy", "Busy: --"), ("flags", "Flags: -----"),
                           ("detail", "Detail: 0x--"),
                           ("t0", "T0 BRAM: --"), ("t1", "T1 CIC:  --"),
                           ("t2", "T2 FFT:  --"), ("t3", "T3 Arith: --"),
                           ("t4", "T4 ADC:  --")]:
            lbl = ttk.Label(st_frame, text=text, font=("Menlo", 9))
            lbl.pack(anchor="w")
            self._st_labels[name] = lbl

        # Right: Parameter configuration
        right = ttk.LabelFrame(outer, text="Parameter Configuration", padding=12)
        right.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        self._param_vars: Dict[str, tk.StringVar] = {}
        params = [
            ("CFAR Guard (0x21)", 0x21, "2"),
            ("CFAR Train (0x22)", 0x22, "8"),
            ("CFAR Alpha Q4.4 (0x23)", 0x23, "48"),
            ("CFAR Mode (0x24)", 0x24, "0"),
            ("Threshold (0x10)", 0x10, "500"),
            ("Gain Shift (0x06)", 0x06, "0"),
            ("DC Notch Width (0x27)", 0x27, "0"),
            ("Range Mode (0x20)", 0x20, "0"),
            ("Stream Enable (0x05)", 0x05, "7"),
        ]
        for row_idx, (label, opcode, default) in enumerate(params):
            ttk.Label(right, text=label).grid(row=row_idx, column=0,
                                               sticky="w", pady=2)
            var = tk.StringVar(value=default)
            self._param_vars[str(opcode)] = var
            ttk.Entry(right, textvariable=var, width=10).grid(
                row=row_idx, column=1, padx=8, pady=2)
            ttk.Button(right, text="Set",
                       command=lambda op=opcode, v=var: self._send_cmd(
                           op, int(v.get()))
                       ).grid(row=row_idx, column=2, pady=2)

        # Custom command
        ttk.Separator(right, orient="horizontal").grid(
            row=len(params), column=0, columnspan=3, sticky="ew", pady=8)
        ttk.Label(right, text="Custom Opcode (hex)").grid(
            row=len(params) + 1, column=0, sticky="w")
        self._custom_op = tk.StringVar(value="01")
        ttk.Entry(right, textvariable=self._custom_op, width=10).grid(
            row=len(params) + 1, column=1, padx=8)
        ttk.Label(right, text="Value (dec)").grid(
            row=len(params) + 2, column=0, sticky="w")
        self._custom_val = tk.StringVar(value="0")
        ttk.Entry(right, textvariable=self._custom_val, width=10).grid(
            row=len(params) + 2, column=1, padx=8)
        ttk.Button(right, text="Send Custom",
                   command=self._send_custom).grid(
            row=len(params) + 2, column=2, pady=2)

        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=2)
        outer.rowconfigure(0, weight=1)

    # -------------------------------------------------------- CSV Tab
    def _build_csv_tab(self, parent):
        """CSV offline analysis tab (ported from V4_2_CSV)."""
        controls = ttk.Frame(parent)
        controls.pack(fill="x", padx=10, pady=5)

        ttk.Button(controls, text="Load CSV",
                   command=self._load_csv).pack(side="left", padx=5)
        ttk.Button(controls, text="Process",
                   command=self._process_csv).pack(side="left", padx=5)
        ttk.Button(controls, text="Run CFAR",
                   command=self._run_csv_cfar).pack(side="left", padx=5)

        self._csv_status = ttk.Label(controls, text="No file loaded",
                                      font=("Menlo", 9))
        self._csv_status.pack(side="left", padx=20)

        # 4 subplots like V4_2
        self._csv_fig = Figure(figsize=(12, 7), facecolor=BG)
        self._csv_fig.subplots_adjust(hspace=0.35, wspace=0.30)
        self._csv_ax1 = self._csv_fig.add_subplot(221)
        self._csv_ax2 = self._csv_fig.add_subplot(222)
        self._csv_ax3 = self._csv_fig.add_subplot(223)
        self._csv_ax4 = self._csv_fig.add_subplot(224)
        for ax in [self._csv_ax1, self._csv_ax2, self._csv_ax3, self._csv_ax4]:
            ax.set_facecolor(BG2)
            ax.tick_params(colors=FG)
            ax.xaxis.label.set_color(FG)
            ax.yaxis.label.set_color(FG)
            ax.title.set_color(FG)
        self._csv_ax1.set_title("Range Profile")
        self._csv_ax2.set_title("Doppler Spectrum")
        self._csv_ax3.set_title("Range-Doppler Map")
        self._csv_ax4.set_title("MTI Filtered")

        canvas = FigureCanvasTkAgg(self._csv_fig, master=parent)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)
        self._csv_canvas = canvas
        self._csv_data = None

    # -------------------------------------------------------- Log Tab
    def _build_log_tab(self, parent):
        self.log_text = tk.Text(parent, bg=BG2, fg=FG, font=("Menlo", 10),
                                 insertbackground=FG, wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=8, pady=8)
        handler = _TextHandler(self.log_text, self.root)
        handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S"))
        logging.getLogger().addHandler(handler)

    # ----------------------------------------------------------- Actions
    def _on_connect(self):
        if self.conn.is_open:
            if self._acq_thread is not None:
                self._acq_thread.stop()
                self._acq_thread.join(timeout=2)
                self._acq_thread = None
            self.conn.close()
            self.lbl_status.config(text="DISCONNECTED", foreground=RED)
            self.btn_connect.config(text="Connect")
            log.info("Disconnected")
            return

        self.lbl_status.config(text="CONNECTING...", foreground=YELLOW)
        self.btn_connect.config(state="disabled")
        self.root.update_idletasks()
        self._connect_result: Optional[bool] = None

        def _do_connect():
            ok = self.conn.open()
            self._connect_result = ok

        threading.Thread(target=_do_connect, daemon=True).start()
        self._poll_connect_result()

    def _poll_connect_result(self):
        if self._connect_result is not None:
            self._on_connect_done(self._connect_result)
            self._connect_result = None
        else:
            self.root.after(50, self._poll_connect_result)

    def _on_connect_done(self, success: bool):
        self.btn_connect.config(state="normal")
        if success:
            self.lbl_status.config(text="CONNECTED", foreground=GREEN)
            self.btn_connect.config(text="Disconnect")
            self._acq_thread = RadarAcquisition(
                self.conn, self.frame_queue, self.recorder,
                status_callback=self._on_status_received)
            self._acq_thread.start()
            log.info("Connected and acquisition started")
        else:
            self.lbl_status.config(text="CONNECT FAILED", foreground=RED)
            self.btn_connect.config(text="Connect")

    def _on_record(self):
        if self.recorder.recording:
            self.recorder.stop()
            self.btn_record.config(text="Record")
            return
        filepath = filedialog.asksaveasfilename(
            defaultextension=".h5",
            filetypes=[("HDF5", "*.h5"), ("All", "*.*")],
            initialfile=f"radar_{time.strftime('%Y%m%d_%H%M%S')}.h5")
        if filepath:
            self.recorder.start(filepath)
            self.btn_record.config(text="Stop Rec")

    def _send_cmd(self, opcode: int, value: int):
        cmd = RadarProtocol.build_command(opcode, value)
        ok = self.conn.write(cmd)
        log.info(f"CMD 0x{opcode:02X} val={value} ({'OK' if ok else 'FAIL'})")

    def _send_custom(self):
        try:
            op = int(self._custom_op.get(), 16)
            val = int(self._custom_val.get())
            self._send_cmd(op, val)
        except ValueError:
            log.error("Invalid custom command values")

    def _on_status_received(self, status: StatusResponse):
        self._pending_status = status

    def _update_self_test_labels(self, status: StatusResponse):
        if not hasattr(self, "_st_labels"):
            return
        flags = status.self_test_flags
        detail = status.self_test_detail
        busy = status.self_test_busy
        busy_str = "RUNNING" if busy else "IDLE"
        busy_color = YELLOW if busy else FG
        self._st_labels["busy"].config(text=f"Busy: {busy_str}",
                                        foreground=busy_color)
        self._st_labels["flags"].config(text=f"Flags: {flags:05b}")
        self._st_labels["detail"].config(text=f"Detail: 0x{detail:02X}")
        tests = [("t0", "T0 BRAM"), ("t1", "T1 CIC"), ("t2", "T2 FFT"),
                 ("t3", "T3 Arith"), ("t4", "T4 ADC")]
        for i, (key, name) in enumerate(tests):
            if busy:
                r, c = "...", YELLOW
            elif flags & (1 << i):
                r, c = "PASS", GREEN
            else:
                r, c = "FAIL", RED
            self._st_labels[key].config(text=f"{name}: {r}", foreground=c)

    # -------------------------------------------------------- Map actions
    def _update_gps_manual(self):
        try:
            self._current_gps.latitude = float(self._var_lat.get())
            self._current_gps.longitude = float(self._var_lon.get())
            self._google_maps_api_key = self._var_api_key.get()
            self.lbl_gps.config(
                text=f"GPS: {self._current_gps.latitude:.4f}, "
                     f"{self._current_gps.longitude:.4f}")
            log.info(f"GPS updated manually: {self._current_gps.latitude:.6f}, "
                     f"{self._current_gps.longitude:.6f}")
        except ValueError:
            log.error("Invalid GPS coordinates")

    def _generate_map(self):
        if self._current_gps.latitude == 0 and self._current_gps.longitude == 0:
            self._map_status.config(text="Map: No GPS data")
            return
        try:
            html = MapGenerator.generate(
                self._current_gps, self._tracked_targets,
                coverage_radius=self._max_range,
                api_key=self._google_maps_api_key)
            with tempfile.NamedTemporaryFile(
                    mode="w", suffix=".html", delete=False, encoding="utf-8") as f:
                f.write(html)
                self._map_file_path = f.name
            self._map_status.config(text=f"Map: Generated")
            self._map_info.config(
                text=f"Radar: {self._current_gps.latitude:.6f}, "
                     f"{self._current_gps.longitude:.6f} | "
                     f"Targets: {len(self._tracked_targets)}")
            log.info(f"Map generated: {self._map_file_path}")
        except Exception as e:
            log.error(f"Map generation error: {e}")
            self._map_status.config(text=f"Map: Error")

    def _open_map_browser(self):
        if self._map_file_path and os.path.exists(self._map_file_path):
            webbrowser.open("file://" + os.path.abspath(self._map_file_path))
        else:
            self._generate_map()
            if self._map_file_path:
                webbrowser.open("file://" + os.path.abspath(self._map_file_path))

    # -------------------------------------------------------- CSV actions
    def _load_csv(self):
        filepath = filedialog.askopenfilename(
            title="Select CSV file",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if not filepath:
            return
        try:
            import csv
            with open(filepath, "r") as f:
                reader = csv.DictReader(f)
                rows = list(reader)
            if not rows:
                self._csv_status.config(text="Empty CSV file")
                return
            # Build complex IQ array
            i_vals = np.array([float(r.get("I_value", r.get("i", 0)))
                               for r in rows])
            q_vals = np.array([float(r.get("Q_value", r.get("q", 0)))
                               for r in rows])
            self._csv_data = i_vals + 1j * q_vals
            self._csv_status.config(
                text=f"Loaded: {os.path.basename(filepath)} "
                     f"({len(rows)} samples)")
            log.info(f"CSV loaded: {filepath} ({len(rows)} samples)")
        except Exception as e:
            log.error(f"CSV load error: {e}")
            self._csv_status.config(text=f"Load error: {e}")

    def _process_csv(self):
        if self._csv_data is None:
            messagebox.showwarning("Warning", "Load a CSV file first")
            return
        try:
            proc = CSVSignalProcessor
            iq = self._csv_data

            for ax in [self._csv_ax1, self._csv_ax2, self._csv_ax3, self._csv_ax4]:
                ax.clear()
                ax.set_facecolor(BG2)

            # Range profile
            r_axis, r_mag = proc.range_fft(iq)
            self._csv_ax1.plot(r_axis[:len(r_axis)//2],
                               r_mag[:len(r_mag)//2], color=ACCENT)
            self._csv_ax1.set_title("Range Profile", color=FG)
            self._csv_ax1.set_xlabel("Range (m)", color=FG)
            self._csv_ax1.grid(True, alpha=0.3)

            # Doppler
            v_axis, d_mag = proc.doppler_fft(iq)
            self._csv_ax2.plot(v_axis, d_mag, color=GREEN)
            self._csv_ax2.set_title("Doppler Spectrum", color=FG)
            self._csv_ax2.set_xlabel("Velocity (m/s)", color=FG)
            self._csv_ax2.grid(True, alpha=0.3)

            # MTI comparison
            mti = proc.mti_filter(iq, "single")
            self._csv_ax4.plot(np.abs(iq[:100]), color=ACCENT, alpha=0.7,
                               label="Original")
            if len(mti) > 0:
                self._csv_ax4.plot(np.abs(mti[:100]), color=RED, alpha=0.7,
                                   label="MTI")
            self._csv_ax4.set_title("MTI Comparison", color=FG)
            self._csv_ax4.legend(fontsize=8)
            self._csv_ax4.grid(True, alpha=0.3)

            # R-D map placeholder (need chirp structure for proper 2D map)
            self._csv_ax3.set_title("Range-Doppler (need chirp structure)", color=FG)
            self._csv_ax3.text(0.5, 0.5, "Load multi-chirp CSV\nfor 2D R-D map",
                               ha="center", va="center", color=FG,
                               transform=self._csv_ax3.transAxes)

            self._csv_canvas.draw_idle()
            self._csv_status.config(text="Processing complete")
        except Exception as e:
            log.error(f"CSV processing error: {e}")
            self._csv_status.config(text=f"Error: {e}")

    def _run_csv_cfar(self):
        if self._csv_data is None:
            messagebox.showwarning("Warning", "Load a CSV file first")
            return
        try:
            proc = CSVSignalProcessor
            r_axis, r_mag = proc.range_fft(self._csv_data)
            half = len(r_mag) // 2
            detections = proc.cfar_detect(r_mag[:half])

            self._csv_ax1.clear()
            self._csv_ax1.set_facecolor(BG2)
            self._csv_ax1.plot(r_axis[:half], r_mag[:half], color=ACCENT)
            if detections:
                det_r = [r_axis[b] for b, _ in detections if b < half]
                det_m = [m for b, m in detections if b < half]
                self._csv_ax1.plot(det_r, det_m, "o", color=RED, markersize=6,
                                   label=f"CFAR ({len(detections)} det)")
            self._csv_ax1.set_title("Range + CFAR Detections", color=FG)
            self._csv_ax1.legend(fontsize=8)
            self._csv_ax1.grid(True, alpha=0.3)
            self._csv_canvas.draw_idle()
            self._csv_status.config(
                text=f"CFAR: {len(detections)} detections")
        except Exception as e:
            log.error(f"CSV CFAR error: {e}")

    # -------------------------------------------------------- Display loop
    def _schedule_update(self):
        try:
            self._update_display()
        except Exception as e:
            log.error(f"Display update error: {e}", exc_info=True)
        self.root.after(self.UPDATE_INTERVAL_MS, self._schedule_update)

    def _update_display(self):
        # Status packet
        status = self._pending_status
        if status is not None:
            self._pending_status = None
            self._update_self_test_labels(status)

        frame = None
        while True:
            try:
                frame = self.frame_queue.get_nowait()
            except queue.Empty:
                break

        if frame is None:
            return

        self._current_frame = frame
        self._frame_count += 1

        # FPS
        now = time.time()
        dt = now - self._fps_ts
        if dt > 0.5:
            self._fps = self._frame_count / dt
            self._frame_count = 0
            self._fps_ts = now

        # fftshift Doppler axis FIRST so all downstream code uses shifted indices
        mag = np.fft.fftshift(frame.magnitude, axes=1)
        det_shifted = np.fft.fftshift(frame.detections, axes=1)

        # Extract detections for tracking from SHIFTED arrays
        det_coords = np.argwhere(det_shifted > 0)
        raw_targets = []
        for rbin, dbin in det_coords:
            t = RadarTarget(
                range_m=(float(rbin) + 0.5) * self._range_per_bin,
                velocity=self._vel_lo + (float(dbin) + 0.5) * self._vel_per_bin,
                snr=float(mag[rbin, dbin]),
                elevation=0, azimuth=0,
                timestamp=frame.timestamp)
            raw_targets.append(t)

        # Run tracker
        self._tracked_targets = self._tracker.update(
            raw_targets, self._current_gps)

        # Update labels
        self.lbl_fps.config(text=f"{self._fps:.1f} fps")
        self.lbl_detections.config(text=f"Det: {frame.detection_count}")
        self.lbl_frame.config(text=f"Frame: {frame.frame_number}")
        self.lbl_tracks.config(text=f"Tracks: {len(self._tracker.tracks)}")

        frame_vmax = float(np.max(mag)) if np.max(mag) > 0 else 1.0
        self._vmax_ema = (self._vmax_alpha * frame_vmax +
                          (1.0 - self._vmax_alpha) * self._vmax_ema)
        stable_vmax = max(self._vmax_ema, 1.0)

        self._rd_img.set_data(mag)
        self._rd_img.set_clim(vmin=0, vmax=stable_vmax)

        # CFAR overlay
        det_idx = np.argwhere(det_shifted > 0)
        if len(det_idx) > 0:
            range_m = (det_idx[:, 0] + 0.5) * self._range_per_bin
            vel_ms = self._vel_lo + (det_idx[:, 1] + 0.5) * self._vel_per_bin
            self._det_scatter.set_offsets(np.column_stack([vel_ms, range_m]))
        else:
            self._det_scatter.set_offsets(np.empty((0, 2)))

        # Tracked targets overlay
        if self._tracked_targets:
            t_offsets = np.array([[t.velocity, t.range_m]
                                  for t in self._tracked_targets])
            self._track_scatter.set_offsets(t_offsets)
        else:
            self._track_scatter.set_offsets(np.empty((0, 2)))

        # Waterfall
        self._waterfall.append(frame.range_profile.copy())
        wf_arr = np.array(list(self._waterfall))
        wf_max = max(np.max(wf_arr), 1.0)
        self._wf_img.set_data(wf_arr)
        self._wf_img.set_clim(vmin=0, vmax=wf_max)

        self._canvas.draw_idle()

        # Update targets treeview (every 5th frame to reduce overhead)
        if frame.frame_number % 5 == 0:
            self._update_targets_tree()

    def _update_targets_tree(self):
        for item in self._targets_tree.get_children():
            self._targets_tree.delete(item)
        for t in self._tracked_targets:
            hits = self._tracker.tracks.get(t.track_id, {}).get("hits", 0)
            self._targets_tree.insert("", "end", values=(
                t.track_id, f"{t.range_m:.1f}", f"{t.velocity:.1f}",
                t.azimuth, t.elevation, f"{t.corrected_elevation:.1f}",
                f"{t.snr:.0f}", hits))


# ============================================================================
# Text Log Handler (same as V1)
# ============================================================================

class _TextHandler(logging.Handler):
    def __init__(self, text_widget: tk.Text, root: tk.Tk):
        super().__init__()
        self._text = text_widget
        self._root = root
        self._queue: queue.Queue = queue.Queue(maxsize=500)
        self._drain()

    def emit(self, record):
        msg = self.format(record)
        try:
            self._queue.put_nowait(msg)
        except queue.Full:
            pass

    def _drain(self):
        try:
            for _ in range(50):
                msg = self._queue.get_nowait()
                self._text.insert("end", msg + "\n")
            self._text.see("end")
            lines = int(self._text.index("end-1c").split(".")[0])
            if lines > 500:
                self._text.delete("1.0", f"{lines - 500}.0")
        except queue.Empty:
            pass
        except Exception:
            pass
        self._root.after(100, self._drain)


# ============================================================================
# Entry Point
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="AERIS-10 Radar Dashboard V2")
    parser.add_argument("--live", action="store_true",
                        help="Use real FT601 hardware (default: mock mode)")
    parser.add_argument("--replay", type=str, metavar="NPY_DIR",
                        help="Replay real data from .npy directory")
    parser.add_argument("--no-mti", action="store_true",
                        help="With --replay, use non-MTI Doppler data")
    parser.add_argument("--record", action="store_true",
                        help="Start HDF5 recording immediately")
    parser.add_argument("--device", type=int, default=0,
                        help="FT601 device index (default: 0)")
    parser.add_argument("--moving-target", action="store_true",
                        help="Mock mode: simulate approaching target")
    args = parser.parse_args()

    if args.replay:
        npy_dir = os.path.abspath(args.replay)
        conn = ReplayConnection(npy_dir, use_mti=not args.no_mti)
        mode_str = f"REPLAY ({npy_dir})"
    elif args.live:
        conn = FT601Connection(mock=False)
        mode_str = "LIVE"
    else:
        conn = FT601Connection(mock=True, moving_target=args.moving_target)
        mode_str = "MOCK" + (" (moving)" if args.moving_target else "")

    recorder = DataRecorder()
    root = tk.Tk()
    dashboard = RadarDashboardV2(root, conn, recorder)

    if args.record:
        filepath = os.path.join(
            os.getcwd(), f"radar_{time.strftime('%Y%m%d_%H%M%S')}.h5")
        recorder.start(filepath)

    def on_closing():
        if dashboard._acq_thread is not None:
            dashboard._acq_thread.stop()
            dashboard._acq_thread.join(timeout=2)
        if conn.is_open:
            conn.close()
        if recorder.recording:
            recorder.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    log.info(f"Dashboard V2 started (mode={mode_str})")

    if not args.live:
        root.after(200, dashboard._on_connect)

    root.mainloop()


if __name__ == "__main__":
    main()
