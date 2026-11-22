"""Build and check the Red 2 B'x preliminary release layout-002 from this repository, with no hand edits.

    python iterations/002/design/preliminary-pocket/build_release.py PLUGIN_ROOT [--check-only] [--processes N] [--view NAME.html]

Run from any directory with the CAD Python (OCP installed) and node on PATH. PLUGIN_ROOT is an Agent FRC checkout.

Default: regenerate every file of cad/preliminary-design/releases/layout-002 in order (field trials, structure sizing,
blocks and STEP files, design record, motion study, budgets, premises, records, layout plan, missions), then run the
layout check against the layout-001 receipt and save receipt.json. A rebuilt STEP carries a new timestamp, so its
SHA-256 and every pin that follows change; the committed release is one such build.

--check-only: rerun only the layout check on the committed inputs (what CI should run). It writes the receipt to a
temporary file and compares its status and case results with the committed receipt.json.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PLUGIN = Path(sys.argv[1]).resolve()
REL = "cad/preliminary-design/releases/layout-002"
R = ROOT / REL
SRC = HERE.relative_to(ROOT).as_posix()
PREVIOUS = "docs/preliminary-design/releases/layout-001/receipt.json"
procs = sys.argv[sys.argv.index("--processes") + 1] if "--processes" in sys.argv else str(max(2, (os.cpu_count() or 4) - 2))
view = sys.argv[sys.argv.index("--view") + 1] if "--view" in sys.argv else None
py = sys.executable


def run(*cmd, **kw):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], cwd=ROOT, check=True, **kw)


def check(output, write_view):
    env = dict(os.environ, PYTHON=py)
    cmd = ["node", PLUGIN / "scripts/design.mjs", "preliminary-check", ".", f"{REL}/layout.json", "--previous", PREVIOUS, "--output", output]
    run(*cmd, *(["--write-view"] if write_view else []), env=env)


if "--check-only" in sys.argv:
    with tempfile.TemporaryDirectory(dir=ROOT) as tmp:
        out = Path(tmp).relative_to(ROOT).as_posix() + "/receipt.json"
        check(out, False)
        new, old = json.loads((ROOT / out).read_text(encoding="utf-8")), json.loads((R / "receipt.json").read_text(encoding="utf-8"))
    rows = lambda r: {c["case"]: c["status"] for c in r["cases"]}
    same = new["status"] == old["status"] and rows(new) == rows(old)
    print("status", new["status"], "| committed", old["status"], "| case results", "match" if same else "DIFFER")
    sys.exit(0 if same and new["status"] == "layout-checked" else 1)

if R.exists():
    shutil.rmtree(R)
(R / "evidence").mkdir(parents=True)
run(py, "-I", "-B", f"{SRC}/field_trials.py", f"{REL}/evidence/field-trials.json", "--processes", procs)
run(py, "-I", "-B", f"{SRC}/structure_checks.py", f"{REL}/evidence/field-trials.json", f"{REL}/evidence/structure-checks.json")
run(py, "-B", f"{SRC}/build_layout.py", ".", REL, PLUGIN / "scripts", f"{REL}/evidence/field-trials.json", stdout=subprocess.DEVNULL)
run("node", f"{SRC}/design-inputs/revise-002.mjs", PLUGIN, f"{SRC}/design.json", f"{REL}/design.json")
run(py, "-B", f"{SRC}/write_package.py", ".", PLUGIN / "scripts", REL, *([view] if view else []))
check(f"{REL}/receipt.json", bool(view))
print(json.loads((R / "receipt.json").read_text(encoding="utf-8"))["status"])
