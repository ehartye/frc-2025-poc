"""Protect coupling, real issued geometry, independent clocks and conditional custody."""

import json
import hashlib
from functools import lru_cache
import subprocess
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "guide-impact"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@lru_cache(maxsize=1)
def flexure_check():
    # Run the project plant through its own executable import path. Other study
    # directories also contain modules named replay and feed.
    program = """
import json
import numpy as np
from beam import Beam,ROOT
from backing import GuideResponse
b=Beam(ROOT/'guide-mounts','folded')
s=GuideResponse(b)
f=np.zeros_like(s.q)
f[0,0]=.4
s.first_loaded=0.
for _ in range(32):
    s.advance(.00003125,f)
energy=s.energy().copy()
error=s.input-s.dissipated-energy
s.advance(.2,np.zeros_like(f))
print(json.dumps([b.receipt(),energy.tolist(),error.tolist(),s.energy().tolist(),s.dissipated.tolist()]))
"""
    result = subprocess.run(
        [sys.executable, "-c", program],
        cwd=HERE,
        text=True,
        capture_output=True,
        check=True,
    )
    return json.loads(result.stdout)


def read(name):
    return json.loads((HERE / name).read_bytes())


class GuideImpactTests(unittest.TestCase):
    def test_two_installed_faces_are_registered(self):
        s = read("section.json")
        self.assertEqual(s["parts"], 58)
        self.assertEqual(
            s["stepSha256"], digest(ROOT / "guide-mounts/guide-mounts.step")
        )
        self.assertEqual(
            s["alternateThroatDatum"], digest(ROOT / "guide-mounts/datum.json")
        )
        self.assertEqual(s["faceZmm"], [625, 813])
        self.assertEqual(s["yCenterPlaneMm"], 150.5125)
        self.assertEqual(
            sum(r["inside"] + r["outside"] for r in s["sides"].values()), 260
        )

    def test_live_point_compliance_and_mass(self):
        b = flexure_check()[0]
        self.assertTrue(100 < b["frequenciesHz"][0] < 120)
        self.assertTrue(0.025 < b["distributedMassPerSideKg"] < 0.035)
        self.assertEqual(len(b["frequenciesHz"]), 5)
        checks = b["staticMmPerN_referenceFullRetained"]
        for reference, full, retained in checks.values():
            self.assertAlmostEqual(reference, full, delta=0.000002)
            self.assertLess(abs(reference - retained) / reference, 0.01)
        self.assertGreater(checks["625"][0], checks["687"][0] * 6)

    def test_transient_work_and_release(self):
        _, peak, error, final, dissipation = flexure_check()
        self.assertGreater(peak[0], 0)
        self.assertEqual(peak[1], 0)
        np.testing.assert_allclose(error, [0.0, 0.0], atol=1e-15)
        self.assertLess(final[0], peak[0] * 0.002)
        self.assertGreater(dissipation[0], 0)

    def test_layout_and_width_have_independent_inputs(self):
        d = read("results.json")
        rows = d["cases"]
        nominal = [
            r
            for r in rows
            if r["layout"] == "folded"
            and not r["outwardPerSideMm"]
            and not r["carriageSupport"]
            and r["deadlineMs"] == 5460
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

    def test_sensor_fault_survives_the_two_stage_support(self):
        source = json.loads((ROOT / "station-landmark/feed.json").read_bytes())[
            "programSha256"
        ]
        rows = [
            r
            for r in read("results.json")["cases"]
            if r["carriageSupport"]
            and r["case"] == "leftSmall"
            and r["programSha256"] == source
        ]
        self.assertEqual({r["stepUs"] for r in rows}, {31.25, 15.625})
        for r in rows:
            self.assertEqual(
                (r["lastState"], r["lastJavaMs"], r["reason"]),
                ("FAULT", 1020, "incoming-edges-unresolved"),
            )
            self.assertFalse(r["eligibleForStationImageReplay"])
            self.assertEqual(r["portTransitions"][-1][2:4], [False, True])
            self.assertIn("shelfDisplacementSlopeTable", r["modalTables"])

    def test_recovery_keeps_the_same_physical_source(self):
        rows = read("results.json")["cases"]
        recovery = json.loads(
            (ROOT / "split-guide-acquisition/signals.json").read_bytes()
        )["programSha256"]
        for dt in (15.625, 31.25):
            for side in ("leftSmall", "rightSmall"):
                pair = [
                    r
                    for r in rows
                    if r["carriageSupport"] and r["stepUs"] == dt and r["case"] == side
                ]
                old = next(r for r in pair if r["programSha256"] != recovery)
                new = next(r for r in pair if r["programSha256"] == recovery)
                self.assertEqual(old["guideHistorySha256"], new["guideHistorySha256"])
                self.assertEqual(old["modalTables"], new["modalTables"])
                self.assertEqual(old["case"], new["case"])
                self.assertEqual(new["lastJavaMs"], 5460)
                self.assertTrue(new["eligibleForStationImageReplay"])
                if side == "rightSmall":
                    self.assertEqual(old["trajectorySha256"], new["trajectorySha256"])
                else:
                    self.assertEqual(new["firstBandMs"], 1340)

    def test_root_reconstruction_and_belt_force_are_finite(self):
        identifier = digest(HERE / "results.json")
        root = read("root-loads.json")
        self.assertEqual(root["input"], identifier)
        self.assertEqual(len(root["cases"]), 4)
        for v in root["cases"]:
            self.assertLess(v["maxGuideReplayForceErrorN"], 1e-7)
            self.assertLess(v["maxGuideReplayWorkErrorJ"], 1e-10)
            self.assertGreater(np.max(v["peakPositiveSpringN"]), 0)
            self.assertLess(np.min(v["peakNegativeSpringN"]), 0)
        traction = read("traction.json")
        self.assertEqual(traction["register"], identifier)
        for r in traction["cases"]:
            self.assertGreater(
                r["completeTrajectoryAbsXUpperN"],
                max(abs(x) for x in r["postGuideSampledTotalXN"]),
            )
            self.assertIsNone(r["verifiedMotorForceLimitN"])

    def test_optical_handoff_retains_unknown_observations(self):
        r = read("results.json")
        packets = read("station-frames.json")
        self.assertEqual(packets["source"], digest(HERE / "results.json"))
        byid = {v["id"]: v for v in r["cases"]}
        self.assertTrue(packets["packets"])
        for p in packets["packets"]:
            row = byid[p["id"]]
            self.assertTrue(row["eligibleForStationImageReplay"])
            self.assertIsNone(p["actualOpticalCustody"])
            self.assertEqual(row["trajectorySha256"], p["trajectorySha256"])
            self.assertEqual(p["nominalGridRows"], int(100000 / p["stepUs"]) + 1)
            self.assertEqual(
                p["last100msRows"],
                p["nominalGridRows"] + len(p["extraBoundarySubstepAtJavaMs"]),
            )
            self.assertEqual(
                [f["javaMs"] for f in p["frames"]],
                [5360, 5372, 5389, 5406, 5423, 5440, 5457, 5460],
            )
        self.assertIsNone(r["measuredAcquisition"])
