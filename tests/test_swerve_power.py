"""Run the swerve electrical tests with the same module paths as the standalone replay."""
from pathlib import Path
import subprocess
import sys
import unittest


class SwervePowerReplay(unittest.TestCase):
    def test_electrical_suite(self):
        suite = Path(__file__).with_name("swerve_power_suite.py")
        completed = subprocess.run(
            [sys.executable, str(suite)], capture_output=True, text=True, check=False
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
