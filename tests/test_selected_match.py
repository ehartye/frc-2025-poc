"""Keep the interactive evidence tied to the original field and the actual shop exchange."""
import hashlib
import json
import unittest
from html.parser import HTMLParser
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "selected-match"


def receipt(name):
    return json.loads((STUDY / f"{name}.json").read_text(encoding="utf-8"))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ReviewPage(HTMLParser):
    def __init__(self):
        super().__init__()
        self.reading_source = False
        self.payload = ""
        self.links = []

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == "script":
            self.reading_source = attrs.get("id") == "source"
        if tag == "a":
            self.links.append(attrs["href"])

    def handle_data(self, data):
        if self.reading_source:
            self.payload += data

    def handle_endtag(self, tag):
        if tag == "script":
            self.reading_source = False


class TestSelectedMatch(unittest.TestCase):
    def test_fabrication_exchange_and_accounting_share_one_configuration(self):
        kit = receipt("kit")
        budget = receipt("clearance-mass")
        self.assertEqual(sha(STUDY / "receiver-clearance-kit.step"), kit["kitSha256"])
        self.assertEqual(budget["kitSha256"], kit["kitSha256"])
        made = set(kit["volumeMm3"])
        self.assertEqual(made, set(kit["originalVolumeMm3"]) | set(budget["fastenerUpperKg"]))
        self.assertEqual(len(made), kit["reopenedCount"])
        self.assertEqual((len(made), len(budget["fastenerUpperKg"])), (18, 12))
        self.assertFalse({"slide-position-SP1-25", "slide-position-line"} & made)
        self.assertLess(kit["maxVolumeRoundTripMm3"], 0.001)
        self.assertEqual(kit["materialOrUnresolved"], {})
        self.assertGreater(min(kit["rejectedLocators"].values()), 670)
        self.assertAlmostEqual(kit["beamScreen"]["retainedTopHoleLigamentMm"], 1.95)
        ledger = receipt("mass")
        self.assertFalse(any(ledger["coverage"].values()))
        self.assertIsNone(ledger["physicalMarginKg"])
        self.assertAlmostEqual(ledger["representedMarginKg"][0], .245377, places=5)

    def test_negative_controls_and_registered_sweep_are_different_evidence(self):
        original = receipt("reef-material")
        revised = receipt("corrected-clearance")
        self.assertEqual(original["fieldSha256"], revised["fieldSha256"])
        for path, expected in {**revised["sources"], **revised["configurationSources"]}.items():
            with self.subTest(source=path):
                self.assertEqual(sha(ROOT / path), expected)
        self.assertEqual(revised["nativeAndRotationalBodies"], 884)
        self.assertEqual(revised["additionalEnvelopeCount"], 74)
        self.assertTrue(revised["containedAtStow"])
        for dock, count in (("A", 3), ("B", 4)):
            self.assertEqual(original["cases"][f"coral-{dock}-0"]["native"], [])
            hits = original["cases"][f"coral-{dock}-370"]["native"]
            self.assertEqual(len(hits), count)
            rails = {hit["body"]: hit for hit in hits if hit["body"].endswith("-inner")}
            self.assertEqual(set(rails), {"drilled-slide-L-inner", "drilled-slide-R-inner"})
            for hit in rails.values():
                self.assertEqual(hit["field"], "NAUO1053/GE-25302")
                self.assertAlmostEqual(hit["commonMm3"], 1386.84, places=2)
            limit = revised["reefStructure"][f"coral-{dock}"]
            self.assertEqual(limit["pair"][1], "NAUO1053/GE-25302")
            self.assertAlmostEqual(limit["lowerMm"], 2.908115, places=5)
        self.assertGreater(revised["stationOverrunControl"]["commonMm3"], 65000)

    def test_offline_student_review_tracks_the_evidence(self):
        page = ReviewPage()
        page.feed((STUDY / "review.html").read_text(encoding="utf-8"))
        figure = json.loads(page.payload)
        kit = receipt("kit")
        self.assertEqual(figure["kit"], kit["kitSha256"])
        self.assertEqual(figure["leadHome"], kit["wireStowedMm"])
        self.assertEqual(figure["leadHome"] + figure["stroke"], kit["wireExtendedMm"])
        for dock in ("A", "B"):
            record = receipt("reef-material")["cases"][f"coral-{dock}-370"]["native"]
            left = next(hit for hit in record if hit["body"] == "drilled-slide-L-inner")
            self.assertEqual(figure["contact"][dock], left["commonMm3"])
            self.assertEqual(figure["lower"][dock], receipt("corrected-clearance")["reefStructure"][f"coral-{dock}"]["lowerMm"])
        for link in page.links:
            self.assertTrue((STUDY / link).is_file(), link)
