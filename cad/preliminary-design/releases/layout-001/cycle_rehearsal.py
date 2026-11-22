"""Re-derive the receive-time requirement from the match cycle with the plugin's field rehearsal.

    python -I -B cycle_rehearsal.py PROJECT_ROOT PLUGIN_ROOT

Runs one-match variants of the project's kickoff/live-profile.json through the plugin's scripts/rehearsal-series.mjs.
The match is the profile's "Qual 2", its working station-cycling setup: one red robot cycles red-chute-low -> red L1
while its partner keeps the trough lane clear. The cycling robot runs a copy of the profile's station program with the
receive (acquire) dwell set to 0.50, 0.70, 0.75 (the profile's own value) and 1.00 s. Recorded per variant: credited L1
deliveries in TELEOP, the time of the second (the narrative needs two), the TELEOP time left after it, and the score.

Two variants, each with the plugin's --report-deadlocks wait-cycle report: "slot1" (the profile's working Qual 2 setup, the
station duty from red slot 1, default waitCycles) and "slot2-yield" (Red 2's real slot: the red slot-1 and slot-2 plans and
preloads exchanged, with rehearsal.waitCycles "yield" from Agent FRC 0.84.0). Before 0.84.0 the slot-2 setup deadlocked the
red pair permanently (deadlock-repro/).
The project's own role clock (iterations/002/design/strategy/pan-route-12mm.json: feed 21.1 s, carry start 27.032 s) gives
the role-specific receive window.
"""
import copy
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
PLUGIN = Path(sys.argv[2]).resolve()
REL = "iterations/002/design/preliminary-pocket/evidence/cycle-rehearsal"
OUT = ROOT / REL
SOURCE = "kickoff/live-profile.json"
DWELLS = (0.50, 0.70, 0.75, 1.00)
STATION_SLOT = 1          # the profile's working station-cycling robot in Qual 2


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


OUT.mkdir(parents=True, exist_ok=True)
base = json.loads((ROOT / SOURCE).read_text(encoding="utf-8"))
reh = base["rehearsal"]
qual2 = next(m for m in reh["series"]["matches"] if m["id"] == "Qual 2")
station = reh["programs"]["station"]
assert station["steps"][0]["operation"] == "acquire" and station["steps"][0]["dwell"] == 0.75
# variant: (tag, slot that runs the station duty, rehearsal waitCycles setting)
VARIANTS = (("slot1", 1, None), ("slot2-yield", 2, "yield"))
rows = []
for tag, slot, wait in VARIANTS:
    for d in DWELLS:
        prof = copy.deepcopy(base)
        r = prof["rehearsal"]
        if wait:
            r["waitCycles"] = wait
        prog = copy.deepcopy(station)
        prog["steps"][0]["dwell"] = d
        prog["steps"][0]["label"] = f"Receive into the B'x pocket ({d:.2f} s)"
        r["programs"]["red2-pocket"] = prog
        match = copy.deepcopy(qual2)
        match["id"] = f"Red 2 station-L1, receive {d:.2f} s, {tag}"
        match["basis"] = ("Selected role for Red 2 (kickoff/selected-role.md): station-to-L1 cycles in TELEOP; the partner keeps the trough lane clear. "
                          f"Receive dwell {d:.2f} s from the B'x pocket trials; station duty from red slot {slot}.")
        red = {rb["slot"]: rb for rb in match["roster"] if rb["side"] == "red"}
        if slot == 2:
            for key in ("plans", "preload"):
                red[1][key], red[2][key] = red[2].get(key), red[1].get(key)
            for rb in (red[1], red[2]):
                if rb.get("preload") is None:
                    rb.pop("preload", None)
        red[slot]["plans"] = dict(red[slot]["plans"], TELEOP="red2-pocket")
        r["series"] = dict(r["series"], matches=[match])
        name = f"profile-{tag}-receive-{int(round(d * 100)):03d}.json"
        (OUT / name).write_text(json.dumps(prof, indent=1) + "\n", encoding="utf-8")
        res_name = name.replace("profile-", "series-")
        rep_name = name.replace("profile-", "deadlocks-")
        run = subprocess.run(["node", str(PLUGIN / "scripts" / "rehearsal-series.mjs"), str(OUT / name), str(OUT / res_name), "--report-deadlocks"],
                             capture_output=True, text=True)
        if run.returncode:
            raise SystemExit(run.stderr)
        (OUT / rep_name).write_text(run.stdout, encoding="utf-8")
        report = json.loads(run.stdout)
        series = json.loads((OUT / res_name).read_text(encoding="utf-8"))
        m = series["matches"][0]
        game = m["game"]
        robot = f"R{slot}"
        tele = [e for e in game["events"] if e.get("robot") == robot and e.get("station") == "red-l1"
                and e.get("outcome") == "credited" and e["phase"] == "TELEOP"]
        tele_s = next(p_["seconds"] for p_ in m["periods"] if p_["phase"] == "TELEOP")
        second = tele[1]["timeS"] if len(tele) >= 2 else None
        rows.append(dict(variant=tag, receiveDwellS=d, stationRobot=robot, waitCycles=wait or "report",
                         profile={"file": f"{REL}/{name}", "sha256": sha(OUT / name)},
                         series={"file": f"{REL}/{res_name}", "sha256": sha(OUT / res_name)},
                         deadlocks={"file": f"{REL}/{rep_name}", "sha256": sha(OUT / rep_name),
                                    "cycleCount": report["cycleCount"], "permanentCount": report["permanentCount"]},
                         yields=[dl for p_ in m["periods"] for dl in p_.get("deadlocks", [])],
                         creditedL1Teleop=len(tele), secondDeliveryTeleopS=second,
                         teleopReserveAfterSecondS=None if second is None else round(tele_s - second, 3),
                         redScore=game["score"]["red"], blueScore=game["score"]["blue"], firstTwoTeleopS=[e["timeS"] for e in tele[:2]]))
summary = dict(schema="red2/cycle-rehearsal@1", source={"file": SOURCE, "sha256": sha(ROOT / SOURCE)},
               plugin={"script": "scripts/rehearsal-series.mjs", "version": json.loads((PLUGIN / "package.json").read_text())["version"]},
               generator={"file": "iterations/002/design/preliminary-pocket/cycle_rehearsal.py", "sha256": sha(Path(__file__))},
               method=__doc__.strip(), rows=rows)
(OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
for r_ in rows:
    print(r_["variant"], r_["receiveDwellS"], r_["deadlocks"]["cycleCount"], r_["deadlocks"]["permanentCount"], r_["creditedL1Teleop"], r_["firstTwoTeleopS"], r_["teleopReserveAfterSecondS"], r_["redScore"])
