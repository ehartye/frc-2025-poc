"""Register disjoint, selected mass parcels against the robot's stowed native geometry."""

import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "fixed-camera-route"))
import scene
from fixed_locator import CAD

sys.path.insert(0, str(ROOT / "design/stability"))
import stability


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def dump(path, data):
    path.write_text(
        json.dumps(data, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )


def prop(s):
    v, c = CAD.volume(s)
    return v, np.asarray(c), np.array(CAD.bounds(s))


def translated(original, current):
    v, c, b = original
    v2, c2, b2 = current
    shift = b2[:3] - b[:3]
    return (
        abs(v2 - v) < max(0.02, v * 1e-6)
        and np.allclose(c2 - c, shift, atol=0.002, rtol=0)
        and np.allclose(b2[3:] - b[3:], shift, atol=0.002, rtol=0)
    ), shift


def compile_parcels(assets, lamp, account):
    base = read(ROOT / "wheel-loads/moments.json")
    if digest(account) != base["inputMapSha256"]:
        raise ValueError("Selected no-credit robot required")
    bill = read(account)
    for s in bill["sources"]:
        if digest(account.parent / s["file"]) != s["sha256"]:
            raise ValueError("Selected source changed: " + s["id"])
    stowed, _ = scene.get(assets, lamp)
    source_scene = assets / "fixed-camera-route/scene.json"
    if digest(source_scene) != base["sourceSceneSha256"]:
        raise ValueError("Unexpected source geometry")
    registered = read(source_scene)["stowed"]
    if set(stowed) != set(registered) or any(
        not np.allclose(CAD.bounds(s), registered[k], rtol=0, atol=0.002)
        for k, s in stowed.items()
    ):
        raise ValueError(
            "Reopened scene differs from the issued geometric registration"
        )
    historical, pin = stability.loads_for("place")
    if pin != base["sourceStep"]:
        raise ValueError("Historical CAD changed")
    original = CAD.read_named(
        ROOT / "design/competition-package/poses/place.step",
        [{"id": i} for i in read(ROOT / "design/competition-package/part-frames.json")],
    )
    physical = {
        r["occurrence"]: r["instance"]
        for r in bill["assignments"]
        if r["export"] == "stowed" and r.get("instance")
    }
    retired = {
        i for r in bill["changes"] if r["id"] in bill["selected"] for i in r["remove"]
    }
    group = {
        "REV PDH + PH": ["rev-pdh", "rev-ph"],
        "four mechanism SPARK MAX units": [
            k for k in original if k.startswith("spark-")
        ],
        "long-range sensors and anchors": [
            k for k in original if k.startswith("qtrx-") or "harness-" in k
        ],
        "battery hold-down, panel, posts and brackets": [
            k
            for k in original
            if k.startswith(
                ("battery-cheek", "battery-hold", "battery-side-stop", "platform-post")
            )
        ]
        + ["distribution-bridge", "front-electronics-platform"],
    }
    parcels = {k: dict(v) for k, v in base["fixedParcelRows"].items()}
    used = {r["body"] for r in parcels.values()}
    skipped = {}
    for r in historical:
        if r.name in parcels:
            continue
        keys = group.get(r.name, [r.name])
        # Grouped installation rows require the entire original installation.
        if any(
            k not in original
            or k not in stowed
            or k not in physical
            or physical[k] in retired
            or k in used
            for k in keys
        ):
            skipped[r.name] = "missing, regrouped or superseded"
            continue
        old = [prop(original[k]) for k in keys]
        new = [prop(stowed[k]) for k in keys]
        if not all(translated(a, b)[0] for a, b in zip(old, new)):
            skipped[r.name] = "native volume, centroid or extents changed"
            continue
        # The two historically unseparated drive-chain envelopes were sold as their own
        # stock, and the integral covers subsequently altered the front chain path.
        if any(
            k.startswith("chain-") or k.startswith("cross-") or "rail-outer" in k
            for k in keys
        ):
            skipped[r.name] = "later same-label machining or chain repack"
            continue
        if len(keys) == 1 and np.allclose(
            r.bounds[:3], r.bounds[3:], atol=0.001, rtol=0
        ):
            c = new[0][1]
            support = np.r_[c, c]
        else:
            boxes = [row[2] for row in new]
            support = stability.union(boxes)
        parcels[r.name] = {
            "bodies": keys,
            "kg": list(r.mass),
            "positionMm": list(support),
        }
        used.update(keys)
    return {
        "accountSha256": digest(account),
        "sceneSha256": digest(source_scene),
        "sceneInputs": read(source_scene)["inputs"],
        "registeredBodies": len(stowed),
        "parcels": parcels,
        "excludedHistorical": skipped,
        "originalStepSha256": pin,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    p.add_argument("lamp", type=Path)
    p.add_argument("map", type=Path)
    a = p.parse_args()
    dump(HERE / "parcels.json", compile_parcels(a.assets, a.lamp, a.map))
