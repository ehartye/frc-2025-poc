"""The README checker links the still to the right design review and refuses a still block that misleads a reader."""
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))     # cad-readmes.py imports its neighbour atomic_file.py, as it does when run as a script
SPEC = importlib.util.spec_from_file_location("cad_readmes", SCRIPTS / "cad-readmes.py")
cad_readmes = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cad_readmes)


class ReviewTarget(unittest.TestCase):
    """Where the still links when cad-showcase.json names no review: the current detailed design, else the preliminary one."""

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name).resolve()

    def write(self, name, data):
        (self.root / name).parent.mkdir(parents=True, exist_ok=True)
        (self.root / name).write_text(json.dumps(data) if not isinstance(data, str) else data, encoding="utf-8")

    def publish_preliminary(self):
        self.write("docs/preliminary-design/releases.json", {"releases": ["9.0"], "current": "9.0"})
        self.write("docs/preliminary-design/releases/9.0/index.html", "<!doctype html>")

    def test_no_detailed_design_links_the_current_preliminary_review(self):
        """Catches a still that links nowhere, or to a withdrawn page, while only a preliminary design is published."""
        self.publish_preliminary()
        for index in (None, {"designs": [], "designStatus": {}, "current": None}):
            if index is not None:
                self.write("docs/detailed-design/designs.json", index)
            target, _ = cad_readmes.review_of(self.root, {})
            self.assertEqual(target, self.root / "docs/preliminary-design/releases/9.0/index.html")

    def test_a_current_detailed_design_wins(self):
        """Catches a still that keeps pointing at the preliminary review after a detailed design becomes current."""
        self.publish_preliminary()
        self.write("docs/detailed-design/designs.json", {"designs": ["9.1"], "current": "9.1"})
        self.write("docs/detailed-design/designs/9.1/index.html", "<!doctype html>")
        target, _ = cad_readmes.review_of(self.root, {})
        self.assertEqual(target, self.root / "docs/detailed-design/designs/9.1/index.html")

    def test_a_damaged_index_falls_back_rather_than_crashing(self):
        """Catches a damaged detailed index stopping the check; control: the preliminary review is still found."""
        self.publish_preliminary()
        for damaged in ("", "{", "[]", "null"):
            self.write("docs/detailed-design/designs.json", damaged)
            target, _ = cad_readmes.review_of(self.root, {})
            self.assertEqual(target, self.root / "docs/preliminary-design/releases/9.0/index.html")

    def test_an_explicit_viewer_url_wins(self):
        self.publish_preliminary()
        target, _ = cad_readmes.review_of(self.root, {"viewerUrl": "https://example.invalid/site/"})
        self.assertEqual(target, "https://example.invalid/site/")


class StillBlock(unittest.TestCase):
    """The check refuses a block that is missing, misplaced, shows no image or names another design or status."""

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        (self.root / "still.png").write_bytes(b"synthetic still")
        self.file = self.root / "README.md"

    def check(self, text):
        cad_readmes.check_block(self.file, "README.md", text, "Synthetic layout 9.0", "in-progress")

    def block(self, image="still.png", caption="**Project CAD · Synthetic layout 9.0 · in-progress.** Blocks only."):
        return f"# Synthetic\n\n<!-- cad-still -->\n![Synthetic]({image})\n\n{caption}\n<!-- /cad-still -->\n\nText.\n"

    def test_a_current_block_passes(self):
        self.check(self.block())

    def test_a_block_not_below_the_title_is_refused(self):
        with self.assertRaisesRegex(ValueError, "directly below the title"):
            self.check("# Synthetic\n\nIntro first.\n\n" + self.block().split("\n\n", 1)[1])

    def test_a_missing_image_is_refused(self):
        with self.assertRaisesRegex(ValueError, "no image file that exists"):
            self.check(self.block(image="gone.png"))

    def test_a_caption_of_another_status_is_refused(self):
        with self.assertRaisesRegex(ValueError, "does not name the current design"):
            self.check(self.block(caption="**Project CAD · Synthetic layout 9.0 · effective.** Blocks only."))


if __name__ == "__main__":
    unittest.main()
