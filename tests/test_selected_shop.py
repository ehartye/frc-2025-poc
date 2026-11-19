import csv
import hashlib
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/"design/selected-shop"
SPEC=importlib.util.spec_from_file_location("selected_shop",PACKAGE/"reconcile.py")
shop=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(shop)


class SelectedShopTests(unittest.TestCase):
    def test_reconciled_mass_with_adverse_controls(self):
        review=shop.read(PACKAGE/"review.json")
        evidence=review["component_bounds"]
        for filename,sha in evidence["source_sha256"].items():
            self.assertEqual(hashlib.sha256((ROOT/filename).read_bytes()).hexdigest(),sha,filename)
        weights=[r for r in shop.rows(ROOT/"design/drivetrain/review/bom.csv") if r["id"].startswith("wheel-")]
        self.assertEqual(len(weights),6)
        old=sum(float(r["maxMassKg"]) for r in weights)
        initial=shop.read(ROOT/"frame-corners/accounting.json")["representedMarginKg"][0]
        clamp=shop.read(ROOT/"design/installed-procurement/clamp-review.json")["deltaKg"]
        final=initial + old - 6*.30 - 36*.005 - .105 - clamp
        cases=review["margin_at_upper_kg"]
        self.assertAlmostEqual(final,cases["accepted_receiving_caps"])
        self.assertAlmostEqual(final+.020,cases["unwelded_control_only"])
        self.assertLess(cases["old_550g_wheels_with_loose_items"],0)
        self.assertLess(cases["additional_1kg_loose_items"],0)
        self.assertIsNone(review["physical_margin_kg"])

    def test_installed_supply_ledger_and_kit_contents(self):
        with (PACKAGE/"supply.csv").open(newline="",encoding="utf8") as f:
            rows=list(csv.DictReader(f))
        sku={r["sku"]:r for r in rows if r["sku"]}
        self.assertEqual(len(sku),len([r for r in rows if r["sku"]]))
        self.assertEqual((sku["REV-21-1916-PK4"]["installed"],sku["REV-21-1916-PK4"]["packsIfZeroStock"]),("12","3"))
        self.assertEqual(sku["am-1592"]["installed"],"36")
        self.assertEqual(sku["10.025.100.0"]["installed"],"23")
        self.assertEqual(sku["DDR-30G-5"]["installed"],"3")
        self.assertEqual(sku["A00532"]["installed"],"2")
        self.assertEqual(sku["DM6010"]["installed"],"2")
        self.assertEqual([sku["REV-11-186"+str(i)+"-PK4"]["installed"] for i in range(4)],["3","3","1","3"])
        self.assertTrue(all(r["actualOrder"]=="" for r in rows))
        self.assertNotIn("REV-21-2104",sku)
        self.assertFalse(any("Pololu D24V50F5"==r["article"] for r in rows))
        self.assertEqual({sku[k]["installed"] for k in ("REV-25-2109","REV-21-2102","REV-21-2103")},{"2"})
        identities=[name for r in rows for name in r["installedIdentity"].split(" | ")]
        self.assertEqual(len(identities),len(set(identities)))
        review=shop.read(PACKAGE/"review.json")
        self.assertEqual(len(identities),review["buy_instances"])
        self.assertEqual(shop.sha(PACKAGE/"supply.csv"),review["supplies_sha256"])
        legacy=shop.rows(ROOT/"design/installed-procurement/loose-items.csv")
        selected=shop.rows(PACKAGE/"loose.csv")
        self.assertCountEqual([r["order or fabricate"] for r in legacy],[r["legacyItem"] for r in selected])

    def test_screw_head_receiving_gauge_and_negative_controls(self):
        fit=shop.read(PACKAGE/"wheel-fit.json")
        source=shop.read(ROOT/"camera-datum-shop/native-place.json")
        self.assertEqual(fit["source_sha256"],source["step_sha256"])
        self.assertEqual(fit["carrier_substitution"],[6,86])
        self.assertEqual(fit["retained_body_intersections_mm3"],{})
        for wheel in (pos+"-"+lr for pos in ("front","center","rear") for lr in "LR"):
            self.assertGreater(fit["wheel_sweeps"][wheel+"/receiving-head"]["gap_mm"],1.50)
            self.assertGreater(fit["wheel_sweeps"][wheel+"/shank"]["gap_mm"],4.4)
            self.assertGreater(fit["wheel_sweeps"][wheel+"/6mm-head-control"]["one_head_common_mm3"],25)
        self.assertGreater(fit["tip_beyond_inner_wheel_face_mm"],1.6)

    def test_unjoined_map_and_stale_map_rejected(self):
        with TemporaryDirectory() as temp:
            root=Path(temp)
            path=root/"baseline.json"
            shop.write(path,{"selected":["low-lock"],"configuration":"shop-duplex-545-red"})
            with self.assertRaisesRegex(ValueError,"issued #82"):
                shop.stage(root,path,root/"out")
            shop.write(path,{"selected":sorted(shop.EXPECTED),"configuration":"shop-duplex-545-red",
                             "sources":[{"id":"mass","file":"mass.json","sha256":"0"*64}]})
            shop.write(root/"mass.json",{})
            with self.assertRaisesRegex(ValueError,"Input pin changed"):
                shop.stage(root,path,root/"out")
