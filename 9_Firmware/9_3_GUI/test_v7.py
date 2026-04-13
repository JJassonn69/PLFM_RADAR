"""
V7-specific unit tests for the PLFM Radar GUI V7 modules.

Tests cover:
  - v7.models: RadarTarget, RadarSettings, GPSData, ProcessingConfig
  - v7.processing: RadarProcessor, USBPacketParser, apply_pitch_correction
  - v7.workers: polar_to_geographic
  - v7.hardware: STM32USBInterface (basic), production protocol re-exports

Does NOT require a running Qt event loop — only unit-testable components.
Run with:  python -m unittest test_v7 -v
"""

import struct
import unittest
from dataclasses import asdict

import numpy as np


# =============================================================================
# Test: v7.models
# =============================================================================

class TestRadarTarget(unittest.TestCase):
    """RadarTarget dataclass."""

    def test_defaults(self):
        t = _models().RadarTarget(id=1, range=1000.0, velocity=5.0,
                                   azimuth=45.0, elevation=2.0)
        self.assertEqual(t.id, 1)
        self.assertEqual(t.range, 1000.0)
        self.assertEqual(t.snr, 0.0)
        self.assertEqual(t.track_id, -1)
        self.assertEqual(t.classification, "unknown")

    def test_to_dict(self):
        t = _models().RadarTarget(id=1, range=500.0, velocity=-10.0,
                                   azimuth=0.0, elevation=0.0, snr=15.0)
        d = t.to_dict()
        self.assertIsInstance(d, dict)
        self.assertEqual(d["range"], 500.0)
        self.assertEqual(d["snr"], 15.0)


class TestRadarSettings(unittest.TestCase):
    """RadarSettings — verify stale STM32 fields are removed."""

    def test_no_stale_fields(self):
        """chirp_duration, freq_min/max, prf1/2 must NOT exist."""
        s = _models().RadarSettings()
        d = asdict(s)
        for stale in ["chirp_duration_1", "chirp_duration_2",
                       "freq_min", "freq_max", "prf1", "prf2",
                       "chirps_per_position"]:
            self.assertNotIn(stale, d, f"Stale field '{stale}' still present")

    def test_has_physical_conversion_fields(self):
        s = _models().RadarSettings()
        self.assertIsInstance(s.range_resolution, float)
        self.assertIsInstance(s.velocity_resolution, float)
        self.assertGreater(s.range_resolution, 0)
        self.assertGreater(s.velocity_resolution, 0)

    def test_defaults(self):
        s = _models().RadarSettings()
        self.assertEqual(s.system_frequency, 10e9)
        self.assertEqual(s.coverage_radius, 50000)
        self.assertEqual(s.max_distance, 50000)


class TestWaveformConfig(unittest.TestCase):
    """WaveformConfig — range/velocity resolution from waveform params."""

    def test_adi_phaser_range_resolution(self):
        """ADI CN0566 defaults: 4MSPS, 500MHz BW, 300µs chirp, 1079 samples."""
        wf = _models().WaveformConfig()  # ADI phaser defaults
        r_res = wf.range_resolution(n_samples=1079)
        # Expected: c * fs / (2 * N * slope) = 3e8 * 4e6 / (2 * 1079 * 1.667e12)
        # ≈ 0.334 m/bin
        self.assertAlmostEqual(r_res, 0.334, places=2)

    def test_adi_phaser_velocity_resolution(self):
        """ADI phaser: 256 chirps, 1079 samples at 4 MSPS."""
        wf = _models().WaveformConfig()
        v_res = wf.velocity_resolution(n_samples=1079, n_chirps=256)
        # λ * fs / (2 * N * M) = 0.03 * 4e6 / (2 * 1079 * 256) ≈ 0.217 m/s/bin
        self.assertAlmostEqual(v_res, 0.217, places=2)

    def test_max_range(self):
        wf = _models().WaveformConfig()
        max_r = wf.max_range(n_range_bins=64, n_samples=1079)
        # 0.334 * 64 ≈ 21.4 m
        self.assertAlmostEqual(max_r, 21.4, places=0)

    def test_plfm_defaults_differ(self):
        """PLFM FPGA defaults (781.25 m/bin) must NOT equal ADI phaser."""
        default_settings = _models().RadarSettings()
        wf = _models().WaveformConfig()
        r_res = wf.range_resolution(n_samples=1079)
        self.assertNotAlmostEqual(default_settings.range_resolution, r_res,
                                  places=0)  # 781 vs 0.33


class TestGPSData(unittest.TestCase):
    def test_to_dict(self):
        g = _models().GPSData(latitude=41.9, longitude=12.5,
                               altitude=100.0, pitch=2.5)
        d = g.to_dict()
        self.assertAlmostEqual(d["latitude"], 41.9)
        self.assertAlmostEqual(d["pitch"], 2.5)


class TestProcessingConfig(unittest.TestCase):
    def test_defaults(self):
        cfg = _models().ProcessingConfig()
        self.assertTrue(cfg.clustering_enabled)
        self.assertTrue(cfg.tracking_enabled)
        self.assertFalse(cfg.mti_enabled)
        self.assertFalse(cfg.cfar_enabled)


class TestNoCrcmodDependency(unittest.TestCase):
    """crcmod was removed — verify it's not exported."""

    def test_no_crcmod_available(self):
        models = _models()
        self.assertFalse(hasattr(models, "CRCMOD_AVAILABLE"),
                         "CRCMOD_AVAILABLE should be removed from models")


# =============================================================================
# Test: v7.processing
# =============================================================================

class TestApplyPitchCorrection(unittest.TestCase):
    def test_positive_pitch(self):
        from v7.processing import apply_pitch_correction
        self.assertAlmostEqual(apply_pitch_correction(10.0, 3.0), 7.0)

    def test_zero_pitch(self):
        from v7.processing import apply_pitch_correction
        self.assertAlmostEqual(apply_pitch_correction(5.0, 0.0), 5.0)


class TestRadarProcessorMTI(unittest.TestCase):
    def test_mti_order1(self):
        from v7.processing import RadarProcessor
        from v7.models import ProcessingConfig
        proc = RadarProcessor()
        proc.set_config(ProcessingConfig(mti_enabled=True, mti_order=1))

        frame1 = np.ones((64, 32))
        frame2 = np.ones((64, 32)) * 3

        result1 = proc.mti_filter(frame1)
        np.testing.assert_array_equal(result1, np.zeros((64, 32)),
                                       err_msg="First frame should be zeros (no history)")

        result2 = proc.mti_filter(frame2)
        expected = frame2 - frame1
        np.testing.assert_array_almost_equal(result2, expected)

    def test_mti_order2(self):
        from v7.processing import RadarProcessor
        from v7.models import ProcessingConfig
        proc = RadarProcessor()
        proc.set_config(ProcessingConfig(mti_enabled=True, mti_order=2))

        f1 = np.ones((4, 4))
        f2 = np.ones((4, 4)) * 2
        f3 = np.ones((4, 4)) * 5

        proc.mti_filter(f1)  # zeros (need 3 frames)
        proc.mti_filter(f2)  # zeros
        result = proc.mti_filter(f3)
        # Order 2: x[n] - 2*x[n-1] + x[n-2] = 5 - 4 + 1 = 2
        np.testing.assert_array_almost_equal(result, np.ones((4, 4)) * 2)


class TestRadarProcessorCFAR(unittest.TestCase):
    def test_cfar_1d_detects_peak(self):
        from v7.processing import RadarProcessor
        signal = np.ones(64) * 10
        signal[32] = 500  # inject a strong target
        det = RadarProcessor.cfar_1d(signal, guard=2, train=4,
                                      threshold_factor=3.0, cfar_type="CA-CFAR")
        self.assertTrue(det[32], "Should detect strong peak at bin 32")

    def test_cfar_1d_no_false_alarm(self):
        from v7.processing import RadarProcessor
        signal = np.ones(64) * 10  # uniform — no target
        det = RadarProcessor.cfar_1d(signal, guard=2, train=4,
                                      threshold_factor=3.0)
        self.assertEqual(det.sum(), 0, "Should have no detections in flat noise")


class TestRadarProcessorProcessFrame(unittest.TestCase):
    def test_process_frame_returns_shapes(self):
        from v7.processing import RadarProcessor
        proc = RadarProcessor()
        frame = np.random.randn(64, 32) * 10
        frame[20, 8] = 5000  # inject a target
        power, mask = proc.process_frame(frame)
        self.assertEqual(power.shape, (64, 32))
        self.assertEqual(mask.shape, (64, 32))
        self.assertEqual(mask.dtype, bool)


class TestRadarProcessorWindowing(unittest.TestCase):
    def test_hann_window(self):
        from v7.processing import RadarProcessor
        data = np.ones((4, 32))
        windowed = RadarProcessor.apply_window(data, "Hann")
        # Hann window tapers to ~0 at edges
        self.assertLess(windowed[0, 0], 0.1)
        self.assertGreater(windowed[0, 16], 0.5)

    def test_none_window(self):
        from v7.processing import RadarProcessor
        data = np.ones((4, 32))
        result = RadarProcessor.apply_window(data, "None")
        np.testing.assert_array_equal(result, data)


class TestRadarProcessorDCNotch(unittest.TestCase):
    def test_dc_removal(self):
        from v7.processing import RadarProcessor
        data = np.ones((4, 8)) * 100
        data[0, :] += 50  # DC offset in range bin 0
        result = RadarProcessor.dc_notch(data)
        # Mean along axis=1 should be ~0
        row_means = np.mean(result, axis=1)
        for m in row_means:
            self.assertAlmostEqual(m, 0, places=10)


class TestRadarProcessorClustering(unittest.TestCase):
    def test_clustering_empty(self):
        from v7.processing import RadarProcessor
        result = RadarProcessor.clustering([], eps=100, min_samples=2)
        self.assertEqual(result, [])


class TestUSBPacketParser(unittest.TestCase):
    def test_parse_gps_text(self):
        from v7.processing import USBPacketParser
        parser = USBPacketParser()
        data = b"GPS:41.9028,12.4964,100.0,2.5\r\n"
        gps = parser.parse_gps_data(data)
        self.assertIsNotNone(gps)
        self.assertAlmostEqual(gps.latitude, 41.9028, places=3)
        self.assertAlmostEqual(gps.longitude, 12.4964, places=3)
        self.assertAlmostEqual(gps.altitude, 100.0)
        self.assertAlmostEqual(gps.pitch, 2.5)

    def test_parse_gps_text_invalid(self):
        from v7.processing import USBPacketParser
        parser = USBPacketParser()
        self.assertIsNone(parser.parse_gps_data(b"NOT_GPS_DATA"))
        self.assertIsNone(parser.parse_gps_data(b""))
        self.assertIsNone(parser.parse_gps_data(None))

    def test_parse_binary_gps(self):
        from v7.processing import USBPacketParser
        parser = USBPacketParser()
        # Build a valid binary GPS packet
        pkt = bytearray(b"GPSB")
        pkt += struct.pack(">d", 41.9028)     # lat
        pkt += struct.pack(">d", 12.4964)     # lon
        pkt += struct.pack(">f", 100.0)       # alt
        pkt += struct.pack(">f", 2.5)         # pitch
        # Simple checksum
        cksum = sum(pkt) & 0xFFFF
        pkt += struct.pack(">H", cksum)
        self.assertEqual(len(pkt), 30)

        gps = parser.parse_gps_data(bytes(pkt))
        self.assertIsNotNone(gps)
        self.assertAlmostEqual(gps.latitude, 41.9028, places=3)

    def test_no_crc16_func_attribute(self):
        """crcmod was removed — USBPacketParser should not have crc16_func."""
        from v7.processing import USBPacketParser
        parser = USBPacketParser()
        self.assertFalse(hasattr(parser, "crc16_func"),
                         "crc16_func should be removed (crcmod dead code)")

    def test_no_multi_prf_unwrap(self):
        """multi_prf_unwrap was removed (never called, prf fields removed)."""
        from v7.processing import RadarProcessor
        self.assertFalse(hasattr(RadarProcessor, "multi_prf_unwrap"),
                         "multi_prf_unwrap should be removed")


# =============================================================================
# Test: v7.workers — polar_to_geographic
# =============================================================================

def _pyqt6_available():
    try:
        import PyQt6.QtCore  # noqa: F401
        return True
    except ImportError:
        return False


@unittest.skipUnless(_pyqt6_available(), "PyQt6 not installed")
class TestPolarToGeographic(unittest.TestCase):
    def test_north_bearing(self):
        from v7.workers import polar_to_geographic
        lat, lon = polar_to_geographic(0.0, 0.0, 1000.0, 0.0)
        # Moving 1km north from equator
        self.assertGreater(lat, 0.0)
        self.assertAlmostEqual(lon, 0.0, places=4)

    def test_east_bearing(self):
        from v7.workers import polar_to_geographic
        lat, lon = polar_to_geographic(0.0, 0.0, 1000.0, 90.0)
        self.assertAlmostEqual(lat, 0.0, places=4)
        self.assertGreater(lon, 0.0)

    def test_zero_range(self):
        from v7.workers import polar_to_geographic
        lat, lon = polar_to_geographic(41.9, 12.5, 0.0, 0.0)
        self.assertAlmostEqual(lat, 41.9, places=6)
        self.assertAlmostEqual(lon, 12.5, places=6)


# =============================================================================
# Test: v7.hardware — production protocol re-exports
# =============================================================================

class TestHardwareReExports(unittest.TestCase):
    """Verify hardware.py re-exports all production protocol classes."""

    def test_exports(self):
        from v7.hardware import (
            FT2232HConnection,
            RadarProtocol,
            STM32USBInterface,
        )
        # Verify these are actual classes/types, not None
        self.assertTrue(callable(FT2232HConnection))
        self.assertTrue(callable(RadarProtocol))
        self.assertTrue(callable(STM32USBInterface))

    def test_stm32_list_devices_no_crash(self):
        from v7.hardware import STM32USBInterface
        stm = STM32USBInterface()
        self.assertFalse(stm.is_open)
        # list_devices should return empty list (no USB in test env), not crash
        devs = stm.list_devices()
        self.assertIsInstance(devs, list)


# =============================================================================
# Test: v7.__init__ — clean exports
# =============================================================================

class TestV7Init(unittest.TestCase):
    """Verify top-level v7 package exports."""

    def test_no_crcmod_export(self):
        import v7
        self.assertFalse(hasattr(v7, "CRCMOD_AVAILABLE"),
                         "CRCMOD_AVAILABLE should not be in v7.__all__")

    def test_key_exports(self):
        import v7
        # Core exports (no PyQt6 required)
        for name in ["RadarTarget", "RadarSettings", "GPSData",
                      "ProcessingConfig", "FT2232HConnection",
                      "RadarProtocol", "RadarProcessor"]:
            self.assertTrue(hasattr(v7, name), f"v7 missing export: {name}")
        # PyQt6-dependent exports — only present when PyQt6 is installed
        if _pyqt6_available():
            for name in ["RadarDataWorker", "RadarMapWidget",
                          "RadarDashboard"]:
                self.assertTrue(hasattr(v7, name), f"v7 missing export: {name}")


# =============================================================================
# Test: AGC Visualization data model
# =============================================================================

class TestAGCVisualizationV7(unittest.TestCase):
    """AGC visualization ring buffer and data model tests (no Qt required)."""

    def _make_deque(self, maxlen=256):
        from collections import deque
        return deque(maxlen=maxlen)

    def test_ring_buffer_basics(self):
        d = self._make_deque(maxlen=4)
        for i in range(6):
            d.append(i)
        self.assertEqual(list(d), [2, 3, 4, 5])

    def test_gain_range_4bit(self):
        """AGC gain is 4-bit (0-15)."""
        from radar_protocol import StatusResponse
        for g in [0, 7, 15]:
            sr = StatusResponse(agc_current_gain=g)
            self.assertEqual(sr.agc_current_gain, g)

    def test_peak_range_8bit(self):
        """Peak magnitude is 8-bit (0-255)."""
        from radar_protocol import StatusResponse
        for p in [0, 128, 255]:
            sr = StatusResponse(agc_peak_magnitude=p)
            self.assertEqual(sr.agc_peak_magnitude, p)

    def test_saturation_accumulation(self):
        """Saturation ring buffer sum tracks total events."""
        sat = self._make_deque(maxlen=256)
        for s in [0, 5, 0, 10, 3]:
            sat.append(s)
        self.assertEqual(sum(sat), 18)

    def test_mode_label_logic(self):
        """AGC mode string from enable field."""
        from radar_protocol import StatusResponse
        self.assertEqual(
            "AUTO" if StatusResponse(agc_enable=1).agc_enable else "MANUAL",
            "AUTO")
        self.assertEqual(
            "AUTO" if StatusResponse(agc_enable=0).agc_enable else "MANUAL",
            "MANUAL")

    def test_history_len_default(self):
        """Default history length should be 256."""
        d = self._make_deque(maxlen=256)
        self.assertEqual(d.maxlen, 256)

    def test_color_thresholds(self):
        """Saturation color: green=0, warning=1-10, error>10."""
        from v7.models import DARK_SUCCESS, DARK_WARNING, DARK_ERROR
        def pick_color(total):
            if total > 10:
                return DARK_ERROR
            if total > 0:
                return DARK_WARNING
            return DARK_SUCCESS
        self.assertEqual(pick_color(0), DARK_SUCCESS)
        self.assertEqual(pick_color(5), DARK_WARNING)
        self.assertEqual(pick_color(11), DARK_ERROR)


# =============================================================================
# Helper: lazy import of v7.models
# =============================================================================

def _models():
    import v7.models
    return v7.models


if __name__ == "__main__":
    unittest.main()
