"""Bound the installed arm, matched field controls and the unchanged interface claims."""
import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"guide-arm-route"


def read(file):
    return json.loads((HERE/file).read_bytes())


class RoutedGuideTests(unittest.TestCase):
    def test_geometry_and_access(self):
        record=read("revision.json")
        step=hashlib.sha256((HERE/"route.step").read_bytes()).hexdigest()
        self.assertEqual(step,record["stepSha256"])
        self.assertEqual(len(record["volumesMm3"]),6)
        self.assertAlmostEqual(record["armVolumeChangeMm3"],6344,delta=1)
        for file in ("clearance.json","inspection.json","removal.json","field.json","field-outward.json","accounting.json"):
            self.assertEqual(read(file)["stepSha256"],step,file)
        fit=read("clearance.json")
        self.assertEqual(fit["intersectionMm3"],{})
        self.assertGreater(min(fit["landAt0p1Mm3"].values()),10)
        self.assertGreaterEqual(min(fit["nearCableClearanceMm"].values(),default=0.),0.)
        contacts=read("inspection.json")["overlapWithRevisedArmMm3"]
        self.assertEqual(len(contacts),46)
        self.assertTrue(all(v==0 for v in contacts.values()))
        service=read("removal.json")
        self.assertGreater(service["addedMaterialCells"],100)
        self.assertEqual(service["overlappingCellBounds"],{"inward":{},"forward":{}})
        self.assertEqual(service["inheritedServiceSha256"],hashlib.sha256((ROOT/"guide-mounts/service.json").read_bytes()).hexdigest())

    def test_controls_and_scope(self):
        all_layouts=read("layouts.json")
        self.assertEqual(len(all_layouts),18)
        for name,permits in (("rise-+0",(False,False)),("rise-+73",(True,True)),
                             ("shallow-corridor",(False,True)),("deep-return",(False,True)),("front-return",(True,True))):
            cases=all_layouts[name]["wheelCases"]
            self.assertEqual(tuple(cases[k]["permitted"] for k in ("leftSmall","rightSmall")),permits)
        self.assertTrue(any("NEO550" in k for k in all_layouts["rise--36"]["intersectionMm3"]))
        self.assertTrue(any("backing" in k for k in all_layouts["shallow-corridor"]["intersectionMm3"]))
        for file in ("field.json","field-outward.json"):
            rows=read(file)["runs"]
            self.assertEqual(len(rows),12)
            for r in rows:
                if r["mode"]=="nominal":
                    self.assertTrue(r["accepted"])
                    self.assertLess(abs(r["finalBodyErrorMmDeg"][0]),2)
                    self.assertLess(r["latestQualifiedInterval"][0],9)
                elif r["mode"]=="shared-survey-shift" and r["wheel"]=="rightSmall":
                    self.assertTrue(r["accepted"] and r["truthOutsideDeclaredCalibration"])
                    self.assertGreater(abs(r["finalBodyErrorMmDeg"][1]),13)
                else:
                    self.assertFalse(r["accepted"])

    def test_single_pupil_substitution(self):
        witness=next(r for r in read("pair-audit.json")["observations"] if r["ms"]==1820)["variants"]
        for key in ("reference","rightOnly"):
            self.assertEqual(witness[key]["acceptedRays"],[[0,0],[0,1]])
        for key in ("plate","leftOnly"):
            self.assertIsNone(witness[key]["acceptedRays"])
            self.assertTrue(witness[key]["ambiguous"])
        neighbors=witness["plate"]["neighbors"]["0/1"]
        self.assertEqual(len(neighbors["observed"]),2)
        self.assertEqual(neighbors["withFastenersHidden"],[neighbors["observed"][1]])

    def test_installed_budget(self):
        report=read("accounting.json")
        self.assertEqual(len(report["cases"]),8)
        for key,row in report["cases"].items():
            delta=report["deltaKg"][key.split("/")[0]]
            self.assertEqual(row["installed"],1189)
            self.assertIsNone(row["physicalMarginKg"])
            self.assertAlmostEqual(row["marginKg"][0],row["priorMarginKg"][0]-delta[1])
        self.assertAlmostEqual(report["cases"]["independentStock/noCutCreditKg/completeGroupDeltaKg"]["marginKg"][0],.05177,places=5)

