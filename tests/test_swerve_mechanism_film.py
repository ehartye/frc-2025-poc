import hashlib
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "experiments/swerve-drive/score-stroke"


class MechanismFilm(unittest.TestCase):
    def test_chapters_preserve_source_and_fixture_boundaries(self):
        track = json.loads((STUDY / "motion-track.json").read_bytes())
        candidate = json.loads((STUDY / "candidate.json").read_bytes())
        arrival = candidate["feeds"]["cam-station-plus-31.25us-0.1mm"]
        self.assertIn(arrival["pipeTrajectorySha256"], track["sources"].values())
        self.assertEqual(track["piece"], arrival["physicalPieceId"])
        self.assertEqual(track["firstInBandS"], arrival["javaFirstBandMs"] / 1000)
        self.assertAlmostEqual(track["station"][-1][0], arrival["javaLastMs"] / 1000)
        self.assertEqual(track["extend"][-1][1], 370)
        self.assertEqual(track["release"][-1][1], 0)
        self.assertGreater(track["release"][0][2] - track["extend"][-1][2], 80)
        self.assertGreater(track["landing"]["final_dwell_s"], 2)
        self.assertEqual(track["deliveryCredit"], 0)
        for item in ("candidate.json", "landing.json"):
            self.assertEqual(
                track["sources"][item],
                hashlib.sha256((STUDY / item).read_bytes()).hexdigest(),
            )

    def test_published_media_matches_reviewed_artifacts(self):
        page = json.loads((ROOT / "highlights.json").read_bytes())
        sources = {r["id"]: r for r in page["sources"]}
        film = next(r for r in page["media"] if r["asset"] == "receiver-film")
        for key in ("asset", "poster"):
            source = sources[film[key]]
            self.assertTrue(source["shareable"])
            self.assertEqual(
                source["sha256"],
                hashlib.sha256((ROOT / source["file"]).read_bytes()).hexdigest(),
            )
