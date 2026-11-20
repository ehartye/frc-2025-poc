"""Join the published climb variant, reopened parts, stroke and timed assessment."""

import gzip
import hashlib
import json
import struct
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "cad/shallow-cage"


def read(name):
    return json.loads((HERE / name).read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ShallowCageTest(unittest.TestCase):
    def test_download_and_orbit_draw_the_reopened_groups(self):
        native, orbit = read("reopen.json"), read("orbit.json")
        self.assertEqual(native["sha256"], sha(ROOT / native["source"]))
        self.assertEqual(native["sha256"], orbit["fusion"])
        self.assertEqual(
            orbit["baseSha256"], sha(ROOT / "docs/robot/cad/assembly.stl.gz")
        )
        self.assertEqual(orbit["kitSha256"], sha(HERE / "climb-kit.stl.gz"))
        self.assertEqual(
            {k for k, _, _ in orbit["kitBatches"]},
            set(native["added"]) | {"Illustrative cage"},
        )
        self.assertEqual(sum(len(r["bodies"]) for r in native["added"].values()), 29)
        for row in native["added"].values():
            self.assertEqual(sha(HERE / row["step"]), orbit["sourceSteps"][row["step"]])
        kit = gzip.decompress((HERE / "climb-kit.stl.gz").read_bytes())
        self.assertEqual(
            sum(r[-1] for r in orbit["kitBatches"]),
            struct.unpack_from("<I", kit, 80)[0],
        )
        self.assertEqual(orbit["poses"][0], [-650, 0, 0, 0, 0, 0])
        self.assertEqual(orbit["poses"][-1][:5], [0, 36, 0, 13.584, 319.4])
        self.assertEqual(orbit["poses"][-1], orbit["poses"][(153 - 130) * 8])

    def test_signed_contact_controls_and_earned_points(self):
        fixed, rejected, lift = (
            read("installed.json"),
            read("rear-stroke.json"),
            read("lift.json"),
        )
        self.assertEqual(fixed["materialInterferences"], [])
        self.assertGreater(len(rejected["materialInterferences"]), 0)
        self.assertGreater(fixed["accountedUpperIncrementKg"], 5.6)
        for row in read("contact.json")["cases"].values():
            self.assertEqual(len(row["clamped"]), 6)
            self.assertGreater(
                min(x["estimatedBearingAreaMm2"] for x in row["clamped"].values()), 250
            )
        clock = lift["clock"]
        self.assertEqual(clock["reserved"]["fixtureShallowIfAllContactsAndHoldMet"], 6)
        self.assertLess(clock["reserved"]["events"][-1][1], 153)
        for name, row in clock.items():
            self.assertEqual(
                (row["matchScore"], row["passiveAssessmentWallS"]), (0, 156)
            )
            if name != "reserved":
                self.assertEqual(row["fixtureShallowIfAllContactsAndHoldMet"], 0)
