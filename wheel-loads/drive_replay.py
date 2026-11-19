"""Replay the issued station clock at the selected mass with explicit wheel-load assumptions."""

import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np
from contact import HERE, ROOT, read

sys.path.insert(0, str(ROOT / "selected-drive"))
from route import wheel_steps
from electrical import screen


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def run(assets):
    info = read(HERE / "contact.json")
    mass = info["mass"]["noCutCreditKg"]["emptyFieldKg"][1]
    old = read(ROOT / "conditioned-match/slow-power.json")
    sources = {
        p: sha(ROOT / p)
        for p in (
            "selected-drive/electrical.py",
            "selected-drive/route.py",
            "conditioned-match/supply.py",
            "conditioned-match/slow-power.json",
            "selected-drive/wheels.json",
            "winchless-bill/straight-guide.json",
            "fixed-height-match/placements.json",
            "station-landmark/routes.json",
        )
    }
    schedule = read(ROOT / "fixed-height-match/placements.json")["placements"][0]
    runs = {}
    for sign in ("leftSmall", "rightSmall"):
        source = assets / "conditioned-match" / sign / "slow-field-poses.npz"
        with np.load(source) as d:
            t = d["t"]
            pose = d["pose"]
        arcs = wheel_steps(pose)
        drive = schedule["drive"] + 18
        events = {
            "coral-A": {"feed": 21.10, "seat": 21.571672635314284 + 1.4, "carry": drive}
        }
        labels = np.where(t[:-1] < 21.1, "opening approach", "later issued path")
        # Repeat every issued electrical comparison; only compare numeric pins here because
        # the descriptive phase labels are intentionally simplified.
        cases = [
            ("issued", info["referenceEmptyKg"], 0.44, 0.75),
            ("selected", mass, 0.44, 0.75),
        ]
        for name in (
            "equal-flat",
            "center-0.3048-short-80pct-rate",
            "center-half-rate",
            "ends-only",
        ):
            cases.append((name, mass, 0.44, info["rows"][name]["outerFraction"]))
        cases.append(
            (
                "lateral-40mm",
                mass,
                info["rows"]["lighter-side-outside"]["lightSideFraction"],
                0.75,
            )
        )
        signed = {}
        for name, kg, side, outer in cases:
            v = screen(
                t,
                arcs,
                labels,
                events,
                kg,
                "conditioned",
                (11.8, 0.027),
                assets,
                side=side,
                outer=outer,
            )
            keys = (
                "bodyKg",
                "onePipeKg",
                "peakFrictionRequired",
                "peakMotorPhaseA",
                "minTerminalV",
                "firstReferenceInhibitWallS",
                "limitingWallS",
                "floorBelowOrUnsolvedS",
                "minimumMotorHeadroomV",
            )
            r = {k: v[k] for k in keys}
            if name == "issued":
                for k in (
                    "peakMotorPhaseA",
                    "minTerminalV",
                    "bodyKg",
                    "onePipeKg",
                    "peakFrictionRequired",
                ):
                    if abs(r[k] - old[sign]["screened"][k]) > 1.0e-7:
                        raise ValueError("Issued path/rail changed: " + k)
            signed[name] = {**r, "sideFraction": side, "outerFraction": outer}
        # Derive equivalent lateral load-line shift for a CM offset up to 50 mm about
        # the chassis reference during the enabled opening. This is a body-load bound.
        dt = 0.005
        speed = arcs / dt
        w = (speed[:, 1] - speed[:, 0]) / 0.580
        alpha = np.maximum(
            abs(np.diff(w, prepend=0)) / dt, abs(np.diff(w, append=0)) / dt
        )
        lateral = abs(np.mean(speed, axis=1) * w) + 0.05 * (alpha + w * w)
        stop = t[:-1] < 21.10
        lat = lateral[stop].max()
        signed["openingLateralAcceleration"] = {
            "peakMS2": float(lat),
            "wallS": float(t[np.argmax(lateral[stop])]),
            "stationaryCgYAllowanceAtHeightsMm": {
                str(h): info["static44pctYmaxMm"] - h * float(lat) / 9.80665
                for h in (250, 350, 450)
            },
        }
        runs[sign] = {"poseSourceSha256": sha(source), "cases": signed}
    return {
        "sources": sources,
        "battery": {"openCircuitV": 11.8, "commonResistanceOhm": 0.027},
        "signed": runs,
        "historicalModel": "conditioned-match slow-power; subsequent numerical demands are conditional after any first inhibit",
        "fullNewMatchScoring": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    a = p.parse_args()
    (HERE / "power.json").write_text(
        json.dumps(run(a.assets), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
