"""Minimal reproduction: the kickoff lane model deadlocks the red pair when the station duty starts from red slot 2.

    python -I -B make_repro.py PROJECT_ROOT
    node PLUGIN/scripts/rehearsal-series.mjs control.json control-out.json
    node PLUGIN/scripts/rehearsal-series.mjs swapped.json swapped-out.json

Both profiles are kickoff/live-profile.json with the series reduced to its "Qual 2" match. control.json is unchanged
(red slot 1 runs "station", slot 2 runs "keep-lane"). swapped.json only exchanges the red slot-1 and slot-2 plans and
preloads; nothing else differs.
"""
import copy
import json
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
HERE = Path(__file__).resolve().parent
base = json.loads((ROOT / "kickoff/live-profile.json").read_text(encoding="utf-8"))
q2 = next(m for m in base["rehearsal"]["series"]["matches"] if m["id"] == "Qual 2")
for name, swap in (("control.json", False), ("swapped.json", True)):
    prof = copy.deepcopy(base)
    match = copy.deepcopy(q2)
    if swap:
        red = {rb["slot"]: rb for rb in match["roster"] if rb["side"] == "red"}
        for key in ("plans", "preload"):
            red[1][key], red[2][key] = red[2].get(key), red[1].get(key)
    prof["rehearsal"]["series"] = dict(prof["rehearsal"]["series"], matches=[match])
    (HERE / name).write_text(json.dumps(prof, indent=1) + "\n", encoding="utf-8")
print("wrote control.json and swapped.json")
