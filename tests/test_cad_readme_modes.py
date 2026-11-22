"""The README checker features the current preliminary design release while no detailed design is current."""
import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location(
    "cad_readmes", Path(__file__).resolve().parents[1] / "scripts/cad-readmes.py")
cad_readmes = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cad_readmes)

CONFIG = {"viewerUrl": "https://example.invalid/site/"}


def sha(data):
    return hashlib.sha256(data).hexdigest()


class ReadmeModes(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)
        self.files = {
            "README.md": b"# Synthetic\n",
            "docs/robot/preliminary-layout-900.png": b"synthetic still",
            "docs/preliminary-design/releases.json": json.dumps(
                {"releases": ["layout-900"], "designStatus": {"layout-900": "in-progress"}, "current": "layout-900"}).encode(),
            "docs/preliminary-design/releases/layout-900/release.json": b"{}",
            "docs/preliminary-design/releases/layout-900/receipt.json": b'{"receipt": 1}',
            "docs/preliminary-design/releases/layout-900/review-scene.json.gz": b"scene",
        }
        release = "docs/preliminary-design/releases/layout-900/"
        self.files["docs/robot/preliminary-layout-900.json"] = json.dumps({
            "release": "layout-900", "imageSha256": sha(self.files["docs/robot/preliminary-layout-900.png"]),
            "releaseRecordSha256": sha(self.files[release + "release.json"]),
            "receiptSha256": sha(self.files[release + "receipt.json"]),
            "sceneSha256": sha(self.files[release + "review-scene.json.gz"]),
            "caption": {"title": "Synthetic layout", "alt": "Alt.", "pose": "Parked.", "scope": "Blocks only."}}).encode()

    def index(self, status="withdrawn", current=None, designs=("draft-900",)):
        return json.dumps({"current": current, "designs": list(designs),
                           "designStatus": {d: status for d in designs}}).encode()

    def feature(self, index=None, update=False):
        for name, data in self.files.items():
            (self.root / name).parent.mkdir(parents=True, exist_ok=True)
            (self.root / name).write_bytes(data)
        if index is not None:
            (self.root / "docs/detailed-design").mkdir(parents=True, exist_ok=True)
            (self.root / "docs/detailed-design/designs.json").write_bytes(index)
        return cad_readmes.preliminary_feature(self.root, CONFIG, update)

    def test_detailed_still_without_a_design_index(self):
        self.assertIsNone(self.feature())

    def test_detailed_still_while_a_detailed_design_is_shown(self):
        for status in ("effective", "in-progress"):
            self.assertIsNone(self.feature(self.index(status, current="draft-900")))
            self.assertIsNone(self.feature(self.index(status)))

    def test_damaged_index_is_an_error_not_a_mode(self):
        for damaged in (b"", b"{", b"[]", b"null", b'{"designs": ["draft-900"]}',
                        b'{"designs": "draft-900", "designStatus": {"draft-900": "withdrawn"}}',
                        b'{"designs": ["draft-900"], "designStatus": ["withdrawn"]}'):
            with self.assertRaisesRegex(ValueError, "Unreadable detailed design index"):
                self.feature(damaged)

    def test_no_current_detailed_design_features_the_preliminary_still(self):
        for index in (self.index(), self.index("superseded"), self.index(designs=())):
            feature = self.feature(index)
            self.assertEqual(feature["heading"], "Preliminary design · in progress · layout-900")
            self.assertEqual(feature["image"], self.root / "docs/robot/preliminary-layout-900.png")
            self.assertEqual(feature["review"], "https://example.invalid/site/preliminary-design/")
            self.assertEqual(feature["text"], "Synthetic layout. Parked. Blocks only.")

    def test_a_different_still_is_refused(self):
        self.files["docs/robot/preliminary-layout-900.png"] = b"another still"
        with self.assertRaisesRegex(ValueError, "differs from the current release"):
            self.feature(self.index())

    def test_a_still_of_another_release_state_is_refused(self):
        self.files["docs/preliminary-design/releases/layout-900/receipt.json"] = b'{"receipt": 2}'
        with self.assertRaisesRegex(ValueError, "differs from the current release"):
            self.feature(self.index())

    def test_a_release_without_a_still_is_refused(self):
        del self.files["docs/robot/preliminary-layout-900.json"]
        with self.assertRaisesRegex(ValueError, "Render the README still"):
            self.feature(self.index())


if __name__ == "__main__":
    unittest.main()
