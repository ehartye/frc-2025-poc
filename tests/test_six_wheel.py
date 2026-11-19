"""Expose the preserved campaign's regression suite to normal project discovery."""
import unittest
from pathlib import Path


def load_tests(loader, tests, pattern):
    suite = Path(__file__).resolve().parents[1] / "experiments/six-wheel/tests"
    return unittest.TestLoader().discover(str(suite), pattern=pattern or "test*.py", top_level_dir=str(suite))

