"""Build and check the preliminary design Fixed pocket 1.0 from this repository, with no hand edits.

    python iterations/002/design/preliminary-pocket/build_release.py PLUGIN_ROOT [--check-only | --layout-only | --records-only]
        [--processes N] [--jobs N] [--view-only]

Run from any directory with the CAD Python (OCP installed) and node on PATH. PLUGIN_ROOT is a copy of Agent FRC at the
revision .github/workflows/design-publication.yml names as AGENT_FRC_REF: an installed copy or an archive of that
commit, never a checkout another session can move.

Default: regenerate every file of cad/preliminary-design/releases/1.0 in order (field trials, the official field parts
at the two docks, structure sizing, blocks and STEP files, design record, motion study, budgets, premises, records, layout
plan, missions), run the layout check and write receipt.json and the review page. Each step stops the build on a non-zero exit code.

--layout-only: keep the recorded field trials and rebuild everything after them. Use it for a layout revision that does
not change a trial. --jobs N runs the layout check on N worker processes; --processes N does the same for the trials.

--records-only: keep the trials, sizing, STEP files and design record and rewrite the records, plan, receipt and page.

--view-only: with --records-only, rebuild the review page from the saved receipt after a presentation edit. No case reruns.

--check-only: rerun only the layout check on the committed inputs and compare its verdicts with the committed receipt.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PLUGIN = Path(sys.argv[1]).resolve()
REL = "cad/preliminary-design/releases/1.0"
R = ROOT / REL
SRC = HERE.relative_to(ROOT).as_posix()
VIEW = f"{REL}/package-motion.html"
procs = sys.argv[sys.argv.index("--processes") + 1] if "--processes" in sys.argv else "4"
jobs = ["--jobs", sys.argv[sys.argv.index("--jobs") + 1] if "--jobs" in sys.argv else "4"]
py = sys.executable


def run(*cmd, **kw):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], cwd=ROOT, check=True, **kw)


def plugin_identity():
    """The Agent FRC revision this run checked with: the one the workflow names, and the version of the copy used."""
    workflow = (ROOT / ".github/workflows/design-publication.yml").read_text(encoding="utf-8")
    ref = re.search(r"AGENT_FRC_REF:\s*([0-9a-f]{40})", workflow)
    version = json.loads((PLUGIN / "package.json").read_text(encoding="utf-8")).get("version")
    return dict(repository="ehartye/agent-frc", version=version, commit=ref[1] if ref else None)


def check(output, *mode):
    env = dict(os.environ, PYTHON=py)
    run("node", PLUGIN / "scripts/design.mjs", "preliminary-check", ".", f"{REL}/layout.json", "--output", output, *jobs, *mode, env=env)


if "--check-only" in sys.argv:
    with tempfile.TemporaryDirectory(dir=ROOT) as tmp:
        out = Path(tmp).relative_to(ROOT).as_posix() + "/receipt.json"
        check(out, "--physics-only")
        new, old = json.loads((ROOT / out).read_text(encoding="utf-8")), json.loads((R / "receipt.json").read_text(encoding="utf-8"))
    rows = lambda r: {c["case"]: c["status"] for c in r["cases"]}
    same = new["status"] == old["status"] and rows(new) == rows(old)
    print("status", new["status"], "| committed", old["status"], "| case results", "match" if same else "DIFFER")
    sys.exit(0 if same and new["status"] == "layout-checked" else 1)

TRIALS = {"evidence/field-trials.json"}
BUILT = TRIALS | {"evidence/structure-checks.json", "evidence/official-fixtures.json", "package.step", "scored-pipe.step", "layout-blocks.json",
                  "design.json"}
FIELD = "field/"        # one STEP per official field solid, written by cad/field/official/checked_fixtures.py
# HANDOFF.md is written by hand from the files here; the page, scene and receipt are kept for a --view-only rebuild.
HAND = {"HANDOFF.md"}
VIEW_FILES = {"package-motion.html", "review-scene.json.gz", "receipt.json"}


def clear(keep):
    for f in sorted(R.rglob("*"), reverse=True):
        name = f.relative_to(R).as_posix()
        if f.is_file() and name not in keep | HAND and not (name.startswith(FIELD) and BUILT <= keep):
            f.unlink()
        elif f.is_dir() and not any(f.iterdir()):
            f.rmdir()


view_only = "--view-only" in sys.argv
if "--records-only" in sys.argv:
    clear(BUILT | (VIEW_FILES if view_only else set()))
else:
    if "--layout-only" in sys.argv:
        clear(TRIALS)
    else:
        clear(set())
        (R / "evidence").mkdir(parents=True, exist_ok=True)
        run(py, "-I", "-B", f"{SRC}/field_trials.py", f"{REL}/evidence/field-trials.json", "--processes", procs)
    run(py, "-I", "-B", "cad/field/official/checked_fixtures.py", f"{REL}/field", f"{REL}/evidence/official-fixtures.json",
        stdout=subprocess.DEVNULL)
    run(py, "-I", "-B", f"{SRC}/structure_checks.py", f"{REL}/evidence/field-trials.json", f"{REL}/evidence/structure-checks.json")
    run(py, "-B", f"{SRC}/build_layout.py", ".", REL, PLUGIN / "scripts", f"{REL}/evidence/field-trials.json", stdout=subprocess.DEVNULL)
    run("node", f"{SRC}/design-inputs/revise-002.mjs", PLUGIN, f"{SRC}/design.json", f"{REL}/design.json")
run(py, "-B", f"{SRC}/write_package.py", ".", PLUGIN / "scripts", REL, VIEW)
check(f"{REL}/receipt.json", "--view-only" if view_only else "--write-view")
(R / "checked-with.json").write_text(json.dumps(dict(
    schema="frc-2025-poc/checked-with@1", receipt=f"{REL}/receipt.json", plugin=plugin_identity(),
    note="The Agent FRC revision that ran the layout check and wrote the review page of this design."), indent=2) + "\n", encoding="utf-8")
print(json.loads((R / "receipt.json").read_text(encoding="utf-8"))["status"])
