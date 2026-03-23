#!/usr/bin/env python3
"""
Tests for GUI_radar_dashboard_v2.py — new features ported from legacy GUIs.
Tests cover: settings validation, multi-PRF unwrap, pitch correction,
target tracker, map generator, CSV signal processor.
"""

import math
import time
import unittest
import numpy as np

from GUI_radar_dashboard_v2 import (
    GPSData,
    RadarTarget,
    validate_radar_settings,
    RADAR_SETTINGS_LIMITS,
    multi_prf_unwrap,
    _solve_chinese_remainder,
    apply_pitch_correction,
    TargetTracker,
    MapGenerator,
    CSVSignalProcessor,
)


# ============================================================================
# Settings Validation (from V5)
# ============================================================================

class TestSettingsValidation(unittest.TestCase):
    """Tests for firmware-mirrored settings validation (V5)."""

    def test_valid_settings_no_errors(self):
        settings = {
            "system_frequency": 10.5e9,
            "chirp_duration": 30e-6,
            "chirps_per_position": 32,
            "freq_min": 10e6,
            "freq_max": 30e6,
            "prf1": 1000,
            "prf2": 2000,
            "max_distance": 50000,
        }
        errors = validate_radar_settings(settings)
        self.assertEqual(errors, [])

    def test_frequency_out_of_range(self):
        settings = {"system_frequency": 0.5e9}  # below 1 GHz min
        errors = validate_radar_settings(settings)
        self.assertEqual(len(errors), 1)
        self.assertIn("system_frequency", errors[0])

    def test_freq_max_must_exceed_freq_min(self):
        settings = {"freq_min": 30e6, "freq_max": 10e6}
        errors = validate_radar_settings(settings)
        self.assertTrue(any("freq_max" in e for e in errors))

    def test_freq_max_equals_freq_min(self):
        settings = {"freq_min": 20e6, "freq_max": 20e6}
        errors = validate_radar_settings(settings)
        self.assertTrue(any("freq_max" in e for e in errors))

    def test_chirp_duration_too_short(self):
        settings = {"chirp_duration": 0.01e-6}  # 10 ns, below 0.1 us min
        errors = validate_radar_settings(settings)
        self.assertEqual(len(errors), 1)

    def test_chirp_duration_too_long(self):
        settings = {"chirp_duration": 10e-3}  # 10 ms, above 1 ms max
        errors = validate_radar_settings(settings)
        self.assertEqual(len(errors), 1)

    def test_partial_settings_only_validates_present(self):
        settings = {"prf1": 500}  # valid
        errors = validate_radar_settings(settings)
        self.assertEqual(errors, [])

    def test_multiple_errors(self):
        settings = {
            "system_frequency": 0.1e9,  # too low
            "max_distance": 1,          # too low
        }
        errors = validate_radar_settings(settings)
        self.assertEqual(len(errors), 2)


# ============================================================================
# Multi-PRF Velocity Unwrapping (from V2–V6)
# ============================================================================

class TestMultiPRFUnwrap(unittest.TestCase):
    """Tests for CRT-based velocity ambiguity resolution (V2–V6)."""

    def test_zero_doppler_returns_zero(self):
        result = multi_prf_unwrap([0.0], prf1=1000, prf2=2000)
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0], 0.0)

    def test_multiple_measurements(self):
        result = multi_prf_unwrap([0.0, 100.0, -50.0], prf1=1000, prf2=2000)
        self.assertEqual(len(result), 3)

    def test_crt_solver_basic(self):
        v = _solve_chinese_remainder(1.0, 1.0, 10.0, 15.0)
        self.assertAlmostEqual(v, 1.0)

    def test_crt_solver_wraps(self):
        # v1=1.0 with max1=2.0, v2=1.0 with max2=3.0
        # k=0 candidate: 1.0, |1.0-1.0|=0 < 1.5 → match
        v = _solve_chinese_remainder(1.0, 1.0, 2.0, 3.0)
        self.assertAlmostEqual(v, 1.0)

    def test_empty_input(self):
        result = multi_prf_unwrap([], prf1=1000, prf2=2000)
        self.assertEqual(result, [])


# ============================================================================
# Pitch Correction (from V3–V6)
# ============================================================================

class TestPitchCorrection(unittest.TestCase):
    """Tests for IMU pitch correction on elevation angles (V3–V6)."""

    def test_zero_pitch_no_change(self):
        result = apply_pitch_correction(45.0, 0.0)
        self.assertAlmostEqual(result, 45.0, places=3)

    def test_positive_pitch_reduces_elevation(self):
        # Antenna tilted up 10°, raw elevation 45° → corrected ~35°
        result = apply_pitch_correction(45.0, 10.0)
        self.assertAlmostEqual(result, 35.0, places=3)

    def test_negative_pitch_increases_elevation(self):
        result = apply_pitch_correction(45.0, -10.0)
        self.assertAlmostEqual(result, 55.0, places=3)

    def test_result_stays_in_range(self):
        # Large pitch shouldn't produce negative angles
        result = apply_pitch_correction(5.0, 50.0)
        self.assertGreaterEqual(result, 0.0)
        self.assertLess(result, 180.0)

    def test_zero_elevation_zero_pitch(self):
        result = apply_pitch_correction(0.0, 0.0)
        self.assertAlmostEqual(result, 0.0, places=3)


# ============================================================================
# Target Tracker (from V2–V6)
# ============================================================================

class TestTargetTracker(unittest.TestCase):
    """Tests for DBSCAN clustering + Kalman filter tracker (V2–V6)."""

    def setUp(self):
        self.tracker = TargetTracker()

    def test_empty_detections(self):
        result = self.tracker.update([])
        self.assertEqual(result, [])

    def test_single_detection_creates_track(self):
        targets = [RadarTarget(range_m=100.0, velocity=5.0, snr=20.0,
                               timestamp=time.time())]
        result = self.tracker.update(targets)
        self.assertEqual(len(result), 1)
        self.assertGreaterEqual(result[0].track_id, 0)
        self.assertEqual(len(self.tracker.tracks), 1)

    def test_repeated_detection_same_track(self):
        t1 = [RadarTarget(range_m=100.0, velocity=5.0, snr=20.0,
                           timestamp=time.time())]
        self.tracker.update(t1)
        tid1 = list(self.tracker.tracks.keys())[0]

        # Slightly moved detection should associate to same track
        t2 = [RadarTarget(range_m=102.0, velocity=5.1, snr=21.0,
                           timestamp=time.time())]
        result = self.tracker.update(t2)
        self.assertEqual(result[0].track_id, tid1)

    def test_distant_detection_new_track(self):
        t1 = [RadarTarget(range_m=100.0, velocity=0.0, snr=20.0,
                           timestamp=time.time())]
        self.tracker.update(t1)

        # Far away detection should create a new track
        t2 = [RadarTarget(range_m=5000.0, velocity=50.0, snr=15.0,
                           timestamp=time.time())]
        result = self.tracker.update(t2)
        self.assertEqual(len(self.tracker.tracks), 2)

    def test_stale_tracks_pruned(self):
        targets = [RadarTarget(range_m=100.0, velocity=0.0, snr=20.0,
                               timestamp=time.time())]
        self.tracker.update(targets)
        self.assertEqual(len(self.tracker.tracks), 1)

        # Manually age the track beyond timeout
        for tid in self.tracker.tracks:
            self.tracker.tracks[tid]["last_update"] = time.time() - 10.0

        self.tracker.update([])  # prune
        self.assertEqual(len(self.tracker.tracks), 0)

    def test_clustering_merges_nearby(self):
        # Two very close detections should merge into one
        targets = [
            RadarTarget(range_m=100.0, velocity=5.0, snr=20.0,
                        timestamp=time.time()),
            RadarTarget(range_m=101.0, velocity=5.1, snr=22.0,
                        timestamp=time.time()),
        ]
        result = self.tracker.update(targets)
        # Clustering with eps=50 should merge these
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0].range_m, 100.5, places=0)

    def test_pitch_correction_applied(self):
        gps = GPSData(latitude=41.0, longitude=12.0, pitch=10.0)
        targets = [RadarTarget(range_m=100.0, velocity=0.0, snr=20.0,
                               elevation=45, timestamp=time.time())]
        result = self.tracker.update(targets, current_gps=gps)
        self.assertAlmostEqual(result[0].corrected_elevation, 35.0, places=1)

    def test_kalman_hits_increment(self):
        t = [RadarTarget(range_m=100.0, velocity=0.0, snr=20.0,
                          timestamp=time.time())]
        self.tracker.update(t)
        tid = list(self.tracker.tracks.keys())[0]
        self.assertEqual(self.tracker.tracks[tid]["hits"], 1)

        # Same location again
        self.tracker.update(t)
        self.assertEqual(self.tracker.tracks[tid]["hits"], 2)

    def test_kalman_state_dimension(self):
        t = [RadarTarget(range_m=500.0, velocity=10.0, snr=30.0,
                          timestamp=time.time())]
        self.tracker.update(t)
        tid = list(self.tracker.tracks.keys())[0]
        state = self.tracker.tracks[tid]["state"]
        self.assertEqual(len(state), 4)  # [range, range_rate, vel, vel_rate]


# ============================================================================
# Google Maps Generator (from V4–V6)
# ============================================================================

class TestMapGenerator(unittest.TestCase):
    """Tests for Google Maps HTML generation (V4–V6)."""

    def test_generates_valid_html(self):
        gps = GPSData(latitude=41.9028, longitude=12.4964, altitude=50.0)
        html = MapGenerator.generate(gps, [], coverage_radius=3000.0)
        self.assertIn("<!DOCTYPE html>", html)
        self.assertIn("initMap", html)
        self.assertIn("41.9028", html)
        self.assertIn("12.4964", html)

    def test_includes_targets(self):
        gps = GPSData(latitude=41.9028, longitude=12.4964)
        targets = [
            RadarTarget(range_m=1000.0, velocity=5.0, azimuth=45),
            RadarTarget(range_m=2000.0, velocity=-10.0, azimuth=90),
        ]
        html = MapGenerator.generate(gps, targets)
        # Should have 2 target markers
        self.assertEqual(html.count("fillColor: '#0000FF'"), 2)

    def test_empty_targets(self):
        gps = GPSData(latitude=0.0, longitude=0.0)
        html = MapGenerator.generate(gps, [])
        self.assertIn("initMap", html)
        self.assertNotIn("fillColor: '#0000FF'", html)

    def test_api_key_substituted(self):
        gps = GPSData(latitude=41.0, longitude=12.0)
        html = MapGenerator.generate(gps, [], api_key="TEST_KEY_123")
        self.assertIn("TEST_KEY_123", html)

    def test_coverage_radius_in_html(self):
        gps = GPSData(latitude=41.0, longitude=12.0)
        html = MapGenerator.generate(gps, [], coverage_radius=5000.0)
        self.assertIn("5000", html)


# ============================================================================
# CSV Signal Processor (from V4_2_CSV)
# ============================================================================

class TestCSVSignalProcessor(unittest.TestCase):
    """Tests for offline signal processing functions (V4_2_CSV)."""

    def test_range_fft_output_shape(self):
        iq = np.random.randn(256) + 1j * np.random.randn(256)
        r_axis, mag = CSVSignalProcessor.range_fft(iq)
        self.assertEqual(len(r_axis), 256)
        self.assertEqual(len(mag), 256)

    def test_range_fft_peak_at_known_freq(self):
        N = 1024
        fs = 100e6
        bw = 500e6
        # Single tone at bin 10
        iq = np.exp(2j * np.pi * 10 * np.arange(N) / N)
        _, mag = CSVSignalProcessor.range_fft(iq, bw=bw)
        peak_bin = np.argmax(mag)
        # Peak should be near bin 10 (windowing spreads energy slightly)
        self.assertAlmostEqual(peak_bin, 10, delta=2)

    def test_doppler_fft_output_shape(self):
        iq = np.random.randn(128) + 1j * np.random.randn(128)
        v_axis, mag = CSVSignalProcessor.doppler_fft(iq)
        self.assertEqual(len(v_axis), 128)
        self.assertEqual(len(mag), 128)

    def test_mti_single_canceler(self):
        data = np.array([1.0, 1.0, 1.0, 1.0], dtype=complex)
        result = CSVSignalProcessor.mti_filter(data, "single")
        # Constant input → all zeros after MTI
        np.testing.assert_array_almost_equal(result, [0, 0, 0])

    def test_mti_single_canceler_ramp(self):
        data = np.array([0.0, 1.0, 2.0, 3.0], dtype=complex)
        result = CSVSignalProcessor.mti_filter(data, "single")
        np.testing.assert_array_almost_equal(result, [1, 1, 1])

    def test_mti_double_canceler(self):
        data = np.array([0.0, 1.0, 2.0, 3.0, 4.0], dtype=complex)
        result = CSVSignalProcessor.mti_filter(data, "double")
        # Linear ramp: double canceler gives all zeros
        np.testing.assert_array_almost_equal(result, [0, 0, 0])

    def test_mti_empty_input(self):
        result = CSVSignalProcessor.mti_filter(np.array([]), "single")
        self.assertEqual(len(result), 0)

    def test_mti_single_sample(self):
        result = CSVSignalProcessor.mti_filter(np.array([5.0+0j]), "single")
        self.assertEqual(len(result), 0)

    def test_cfar_detect_no_targets_in_noise(self):
        profile = np.ones(100) * 10.0  # flat noise
        dets = CSVSignalProcessor.cfar_detect(profile, guard=2, train=10, alpha=3.0)
        self.assertEqual(len(dets), 0)

    def test_cfar_detect_finds_strong_peak(self):
        profile = np.ones(100) * 10.0
        profile[50] = 1000.0  # strong peak
        dets = CSVSignalProcessor.cfar_detect(profile, guard=2, train=10, alpha=3.0)
        self.assertGreater(len(dets), 0)
        # Detection should be near bin 50
        det_bins = [b for b, _ in dets]
        self.assertIn(50, det_bins)

    def test_cfar_detect_multiple_peaks(self):
        profile = np.ones(200) * 5.0
        profile[50] = 500.0
        profile[150] = 500.0
        dets = CSVSignalProcessor.cfar_detect(profile, guard=2, train=10, alpha=3.0)
        self.assertGreaterEqual(len(dets), 2)

    def test_cfar_high_alpha_suppresses(self):
        profile = np.ones(100) * 10.0
        profile[50] = 50.0  # moderate peak
        dets = CSVSignalProcessor.cfar_detect(profile, guard=2, train=10, alpha=100.0)
        self.assertEqual(len(dets), 0)  # alpha too high, no detections


# ============================================================================
# GPSData + RadarTarget data classes
# ============================================================================

class TestDataClasses(unittest.TestCase):
    def test_gps_defaults(self):
        g = GPSData()
        self.assertEqual(g.latitude, 0.0)
        self.assertEqual(g.pitch, 0.0)

    def test_radar_target_defaults(self):
        t = RadarTarget()
        self.assertEqual(t.track_id, -1)
        self.assertEqual(t.range_m, 0.0)

    def test_gps_with_values(self):
        g = GPSData(latitude=41.9, longitude=12.5, altitude=100.0, pitch=5.0)
        self.assertAlmostEqual(g.latitude, 41.9)
        self.assertAlmostEqual(g.pitch, 5.0)


if __name__ == "__main__":
    unittest.main()
