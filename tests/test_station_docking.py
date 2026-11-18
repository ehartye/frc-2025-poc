"""Compare the structural and finite-pipe evidence used by the student docking lab."""
import hashlib
import json
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
SUPPLY = ROOT / "station-supply"


class StationDockingTest(unittest.TestCase):
    def test_same_assembly_different_catch(self):
        dock = json.loads((SUPPLY / "dock-tolerance.json").read_bytes())
        comp = json.loads((SUPPLY / "compensated-dock.json").read_bytes())
        reference = ROOT / "design/competition-package/shop-duplex/red/receive.step"
        self.assertEqual(comp["reference_step_sha256"], hashlib.sha256(reference.read_bytes()).hexdigest())
        self.assertEqual(comp["reference_step_sha256"], dock["receive_step_sha256"])
        self.assertEqual(dock["body_count"], comp["body_count"])
        self.assertEqual((dock["body_count"], dock["station_occurrences"]), (545, 47))
        self.assertTrue(dock["native_dock"][1]["intersections_mm3"])
        self.assertGreater(comp["structural_error_box"]["gap_lower_bound_mm"], 8.33)
        travel = comp["high_extension_self_sweep"]
        self.assertEqual(travel["end"], [940, 270])
        self.assertFalse(travel["unresolved"] or travel["reference_conflicts_mm3"])
        self.assertFalse(any(travel["exact_swept_common_mm3"].values()))
        centered = next(r for r in dock["finite_pipe_flight"] if r["station_standoff_mm"] == 40)
        self.assertEqual(len(centered["first_positive_material_sample"]["overlaps_mm3"]), 3)
        nominal, left, right, *_ = comp["finite_pipe_samples"]
        self.assertIsNone(nominal["first_positive_material_sample"])
        for row in (left, right):
            self.assertTrue(any("angle-board" in key for key in row["first_positive_material_sample"]["overlaps_mm3"]))

    def test_offline_lab_matches_native_receipt(self):
        html = (SUPPLY / "dock-lab.html").read_text(encoding="utf8")
        embedded = json.loads(re.search(r"const data=(.+?),ns=", html).group(1))
        self.assertEqual([(c["standoff"], c["stroke"]) for c in embedded["cases"]], [(10, 240), (40, 240), (40, 270)])
        self.assertEqual([c["pads"] for c in embedded["cases"]], [False, True, False])
        self.assertAlmostEqual(embedded["rigidGap"], 0.2493, places=4)
