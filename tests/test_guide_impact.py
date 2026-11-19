"""Protect coupling, real issued geometry, independent clocks and conditional custody."""

import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "guide-impact"
sys.path.insert(0, str(HERE))
import beam
from backing import GuideResponse


def read(name):
    return json.loads((HERE / name).read_bytes())


class GuideImpactTests(unittest.TestCase):
    def test_two_installed_faces_are_registered(self):
        s = read("section.json")
        self.assertEqual(s["parts"], 58)
        self.assertEqual(
            s["stepSha256"], beam.digest(ROOT / "guide-mounts/guide-mounts.step")
        )
        self.assertEqual(
            s["alternateThroatDatum"], beam.digest(ROOT / "guide-mounts/datum.json")
        )
        self.assertEqual(s["faceZmm"], [625, 813])
        self.assertEqual(s["yCenterPlaneMm"], 150.5125)
        self.assertEqual(
            sum(r["inside"] + r["outside"] for r in s["sides"].values()), 260
        )

    def test_live_point_compliance_and_mass(self):
        b = beam.Beam(ROOT / "guide-mounts", "folded")
        self.assertTrue(100 < b.w[0] / (2 * np.pi) < 120)
        self.assertTrue(0.025 < b.distributed < 0.035)
        self.assertEqual(len(b.w), 5)
        for reference, full, retained in b.check.values():
            self.assertAlmostEqual(reference, full, delta=0.000002)
            self.assertLess(abs(reference - retained) / reference, 0.01)
        self.assertGreater(b.check["625"][0], b.check["687"][0] * 6)

    def test_transient_work_and_release(self):
        b = beam.Beam(ROOT / "guide-mounts", "folded")
        state = GuideResponse(b)
        force = np.zeros_like(state.q)
        force[0, 0] = 0.4
        state.first_loaded = 0.0
        for _ in range(32):
            state.advance(0.00003125, force)
        positive = state.energy()[0]
        self.assertGreater(positive, 0)
        self.assertEqual(state.energy()[1], 0.0)
        np.testing.assert_allclose(
            state.input, state.dissipated + state.energy(), atol=1e-15, rtol=1e-9
        )
        state.advance(0.2, np.zeros_like(force))
        self.assertLess(state.energy()[0], positive * 0.002)
        self.assertGreater(state.dissipated[0], 0.0)

    def test_layout_and_width_have_independent_inputs(self):
        d = read("results.json")
        rows = d["cases"]
        nominal = [
            r
            for r in rows
            if r["layout"] == "folded"
            and not r["outwardPerSideMm"]
            and not r["carriageSupport"]
            and r["guideModel"][1] == 1800
            and r["guideModel"][2] == 0.236
        ]
        self.assertEqual(
            {(r["case"], r["stepUs"]) for r in nominal},
            {
                (side, dt)
                for side in ("leftSmall", "rightSmall")
                for dt in (15.625, 31.25, 62.5)
            },
        )
        for r in nominal:
            self.assertEqual(r["lastState"], "IN_BAND")
            self.assertEqual(r["lastJavaMs"], 5460)
            self.assertTrue(r["eligibleForStationImageReplay"])
        for name in ("flat", "half-couple", "unribbed", "rigid"):
            self.assertTrue(any(r["layout"] == name for r in rows))
        outward = [r for r in rows if r["outwardPerSideMm"] == 0.315]
        self.assertEqual({r["case"] for r in outward}, {"leftSmall", "rightSmall"})
        rigid = [
            r
            for r in rows
            if r["layout"] == "rigid"
            and r["stepUs"] == 31.25
            and not r["carriageSupport"]
        ]
        self.assertEqual(
            {r["case"]: r["trajectorySha256"] for r in rigid}, d["upstreamTrajectories"]
        )

    def test_force_transfer_and_raw_files_are_identified(self):
        for row in read("results.json")["cases"]:
            self.assertEqual(len(row["trajectorySha256"]), 64)
            if row["layout"] == "rigid":
                continue
            g = row["guide"]
            for key in ("peakSideNormalN", "peakAbsSideTravelMm", "frictionHeatJ"):
                self.assertGreater(max(g[key]), 0)
                self.assertGreaterEqual(min(g[key]), 0)
            self.assertEqual(len(g["signedPipeImpulseNs"]), 2)
            np.testing.assert_allclose(
                g["supportWorkJ"],
                np.array(g["supportEnergyAtExitJ"]) + g["supportDissipationJ"],
                atol=1e-12,
            )

    def test_optical_handoff_retains_unknown_observations(self):
        r = read("results.json")
        packets = read("station-frames.json")
        self.assertEqual(packets["source"], beam.digest(HERE / "results.json"))
        byid = {v["id"]: v for v in r["cases"]}
        self.assertTrue(packets["packets"])
        for p in packets["packets"]:
            row = byid[p["id"]]
            self.assertTrue(row["eligibleForStationImageReplay"])
            self.assertIsNone(p["actualOpticalCustody"])
            self.assertEqual(row["trajectorySha256"], p["trajectorySha256"])
            self.assertEqual(p["last100msRows"], 3201)
            self.assertEqual(
                [f["javaMs"] for f in p["frames"]],
                [5360, 5372, 5389, 5406, 5423, 5440, 5457, 5460],
            )
        self.assertIsNone(r["measuredAcquisition"])
