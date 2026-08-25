#!/usr/bin/env python3
"""Unit tests for Step 1 Sensor Calibration."""

import csv
import json
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

# Add src and root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from calibrate import fit_polynomial, read_measurements
from src.sensor_pipeline import CalibrationManager


class TestSensorCalibration(unittest.TestCase):
    def setUp(self):
        self.temp_dir = Path(tempfile.mkdtemp())
        self.csv_path = self.temp_dir / "test_measurements.csv"

        # Create dummy calibration data
        with self.csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["sensor", "raw_value", "reference_mm", "sample_id"])
            writer.writerow(["sharp_left", "100", "200", "1"])
            writer.writerow(["sharp_left", "200", "150", "2"])
            writer.writerow(["sharp_left", "300", "100", "3"])
            writer.writerow(["sharp_left", "400", "70", "4"])
            writer.writerow(["sharp_left", "500", "50", "5"])

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    def test_read_measurements(self):
        rows = read_measurements(str(self.csv_path))
        self.assertEqual(len(rows), 5)
        self.assertEqual(rows[0][0], "sharp_left")

    def test_polynomial_fitting(self):
        rows = read_measurements(str(self.csv_path))
        fit = fit_polynomial(rows, degree=2)
        self.assertEqual(fit["degree"], 2)
        self.assertEqual(len(fit["coefficients"]), 3)
        self.assertGreater(fit["r2"], 0.95)
        self.assertLess(fit["rmse_mm"], 10.0)


if __name__ == "__main__":
    unittest.main()
