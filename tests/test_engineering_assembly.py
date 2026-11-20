"""Prevent a different archive, losing fit control, or a duplicate physical charge."""

import csv
import gzip
import hashlib
import json
import struct
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1] / "cad/assembly-engineering"
ISSUE = HERE / "issue"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(path.read_bytes())


class EngineeringAssemblyTest(unittest.TestCase):
    def test_issue_has_exact_drawn_inventory_and_signed_mass(self):
        issuance = load(ISSUE / "interfaces.json")
        mass = load(ISSUE / "mass.json")
        self.assertEqual(
            mass["issue"], digest(ISSUE / "station-and-jack-interfaces.step")
        )
        self.assertEqual(mass["issue"], issuance["outputSha256"])
        self.assertEqual(mass["material"], digest(ISSUE / "material-register.csv"))
        with (ISSUE / "material-register.csv").open(encoding="utf8") as f:
            rows = list(csv.DictReader(f))
        with (ISSUE / "article-register.csv").open(encoding="utf8") as f:
            articles = list(csv.DictReader(f))
        self.assertEqual(len({r["id"] for r in rows}), 1184)
        self.assertEqual(sum(r["scope"] == "demonstration" for r in rows), 1)
        charged = sum(
            float(r["inspectionStockUpperKg"])
            for r in rows
            if r["scope"] == "inspection"
        )
        charged += sum(
            float(r["upperKg"]) for r in articles if r["instance"] != "np18-battery"
        )
        self.assertAlmostEqual(charged, mass["inspectionUpperKg"], places=6)
        self.assertGreater(52.16312255 - charged, 0.11)
        self.assertLess(52.16312255 - charged, 0.13)
        self.assertNotIn("Left drive and pawl envelope", {r["id"] for r in rows})
        self.assertEqual(
            [r["instance"] for r in articles].count("unitemized-receiving"), 1
        )
        orbit = load(ISSUE / "orbit.json")
        self.assertEqual(orbit["issue"], mass["issue"])
        self.assertEqual(orbit["installedBodies"], issuance["robotSolids"])
        self.assertEqual(orbit["baseSha256"], digest(ISSUE / "installed.stl.gz"))
        with gzip.open(ISSUE / "installed.stl.gz", "rb") as f:
            count = struct.unpack_from("<I", f.read(84), 80)[0]
        self.assertEqual(count, sum(b[3] for b in orbit["baseBatches"]))

    def test_real_mating_surfaces_and_rejected_tools(self):
        pads = load(HERE / "saddle-bearing.json")["stacks"]
        self.assertEqual(len(pads), 32)
        for r in pads.values():
            self.assertAlmostEqual(r["gapMm"], 0, places=6)
            self.assertGreater(r["press001Mm3"], 0.72)
            self.assertGreater(r["unseat001GapMm"], 0.0099)
        fits = load(ISSUE / "device-acceptance.json")
        self.assertEqual(
            fits["issue"], digest(ISSUE / "station-and-jack-interfaces.step")
        )
        for k, r in fits["stations"].items():
            if k.startswith("pdh-"):
                self.assertGreater(r["inward001mmBearingAreaMm2"], 43)
                self.assertEqual(r["thinSocketMaterialByODmm"]["9.4"], 0)
                self.assertGreater(r["socketDeviceMm3"], 3)
            else:
                self.assertFalse(r["assembledStraightToolHits"])
        connectivity = load(ISSUE / "connections.json")
        self.assertEqual(connectivity["installed"], 1183)
        self.assertIn(["slide-extended-switch"], connectivity["separateComponents"])
