"""Copy the Red 2 design source from the former private assets tree into this repository (run once, 2026-11-22).

    python iterations/MIGRATION/migrate_private_tree.py SOURCE_CHECKOUT [--write]

Copies every git-tracked file under iterations/ and artifacts/ of SOURCE_CHECKOUT to the same path here, except caches,
logs and regenerable or duplicate files (SKIP below). Text files are scrubbed of workstation paths: a path inside the
source checkout or this repository becomes repository-relative, the plugin checkout becomes PLUGIN, the CAD Python
becomes python, and any other home-directory path becomes WORKSTATION/... . Each scrubbed file is listed in
manifest.json with its SHA-256 before and after, because records written before the migration pin the old bytes.
Without --write it only reports.

After the run the artifacts/ tree was moved to iterations/artifacts/ (manifest.json, relocated), because the project
layout check keeps a top-level artifacts folder for generated, untracked output. Do not rerun this script with --write:
it would restore the old location and overwrite files that have changed since.
"""
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

SRC = Path(sys.argv[1]).resolve()
DST = Path(__file__).resolve().parents[2]
WRITE = "--write" in sys.argv
ROOTS = ("iterations/", "artifacts/")
SKIP = [
    (re.compile(r"\.log$"), "log"),
    (re.compile(r"(^|/)__pycache__/|\.pyc$"), "cache"),
    (re.compile(r"^iterations/002/design/preliminary(-pocket)?/(review/)?package-motion\.html$"), "retired 41-73 MB player page, regenerable from its package"),
    (re.compile(r"^iterations/002/design/preliminary(-pocket)?/review/"), "LAN review bundle: copies of pages in the package"),
    (re.compile(r"^iterations/002/design/preliminary-pocket/field-interaction\.html$"), "generated player page (field_playback.py)"),
    (re.compile(r"^iterations/MIGRATION/"), "this folder"),
]
TEXT = {".json", ".py", ".md", ".mjs", ".js", ".txt", ".csv", ".sh", ".html", ".svg", ".jsonl", ".yml", ".ps1", ".scad", ".dxf", ""}
USER = r"[Cc]:[\\/]+Users[\\/]+[A-Za-z0-9_.-]+"
SEP = r"[\\/]+"
RULES = [
    (re.compile(USER + SEP + "repos" + SEP + r"(?:agent-frc-poc-assets|frc-2025-poc)(?:-worktrees" + SEP + r"[A-Za-z0-9_.-]+)?" + SEP), ""),
    (re.compile(USER + SEP + "repos" + SEP + r"(?:agent-frc-poc-assets|frc-2025-poc)(?:-worktrees" + SEP + r"[A-Za-z0-9_.-]+)?(?![A-Za-z0-9_.-])"), "."),
    (re.compile(USER + SEP + "repos" + SEP + r"agent-frc(?![A-Za-z0-9_-])"), "PLUGIN"),
    (re.compile(USER + SEP + r"\.venvs" + SEP + "frc-cad" + SEP + "Scripts" + SEP + r"python\.exe"), "python"),
    (re.compile(USER), "WORKSTATION"),
    (re.compile(r"/Users/[A-Za-z0-9_.-]+/repos/(?:frc-2025-poc/?)"), ""),
    (re.compile(r"/Users/[A-Za-z0-9_.-]+"), "WORKSTATION"),
    (re.compile(r"\b(?:192\.168|10\.0\.0)\.\d{1,3}(?:\.\d{1,3})?(?::\d+)?"), "LAN-HOST"),
]


def sha(b):
    return hashlib.sha256(b).hexdigest()


files = subprocess.run(["git", "ls-files", "-z", *[r.rstrip("/") for r in ROOTS]], cwd=SRC, capture_output=True, check=True).stdout.decode().split("\0")
copied, skipped, scrubbed, big = [], [], [], []
for f in filter(None, files):
    reason = next((why for rx, why in SKIP if rx.search(f)), None)
    if reason:
        skipped.append(dict(file=f, reason=reason))
        continue
    data = (SRC / f).read_bytes()
    out = data
    if Path(f).suffix.lower() in TEXT and len(data) < 20_000_000:
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            text = None
        if text is not None:
            new = text
            for rx, to in RULES:
                new = rx.sub(to, new)
            if new != text:
                out = new.encode("utf-8")
                scrubbed.append(dict(file=f, sha256Before=sha(data), sha256After=sha(out)))
    if len(out) > 95_000_000:
        big.append(f)
        continue
    copied.append(f)
    if WRITE:
        p = DST / f
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(out)
head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=SRC, capture_output=True, text=True).stdout.strip()
manifest = dict(schema="frc-2025-poc/private-tree-migration@1", sourceRepository="agent-frc-poc-assets (former private assets repository)",
                sourceBranch="private/red2-native-successor", sourceCommit=head, roots=list(ROOTS), copied=len(copied),
                skipped=skipped, scrubbed=scrubbed, over95MB=big,
                note="Records written before the migration pin the pre-scrub bytes of the scrubbed files; compare with sha256Before.")
if WRITE:
    (DST / "iterations/MIGRATION/manifest.json").write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8")
print("copied", len(copied), "skipped", len(skipped), "scrubbed", len(scrubbed), "over 95 MB", big)
