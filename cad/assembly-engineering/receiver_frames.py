"""Register the moving camera support from its supplier-independent 25-body source."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = ROOT / "cad/assembly-engineering"
OWNER = ROOT / "experiments/six-wheel/camera-disconnects"
CARRIER = "Outer slide"


def support_ids():
    receipt = json.loads((OWNER / "support.json").read_bytes())
    if (
        hashlib.sha256((OWNER / "service-support.step").read_bytes()).hexdigest()
        != receipt["step_sha256"]
    ):
        raise ValueError("Camera-support STEP revision changed")
    ids = sorted("support--" + k for k in receipt["parts"])
    if len(ids) != receipt["native_count"] or len(ids) != 25:
        raise ValueError("Incomplete camera support")
    return ids


def witness_ids():
    return sorted(
        support_ids()
        + ["moving-camera-public-envelope", "wire--data-plug", "wire--power-plug"]
    )


def inherited_frames():
    with (HERE / "issue/cold-comparison.csv").open(encoding="utf8", newline="") as f:
        return {r["id"]: r["occurrence"].split(":")[0] for r in csv.DictReader(f)}


def corrections():
    reg = inherited_frames()
    return {
        k: CARRIER
        for k in support_ids() + ["moving-camera-public-envelope"]
        if reg[k] != CARRIER
    }


def audit(scene, destination):
    reg = inherited_frames()
    original = json.loads(scene.read_bytes())
    lengths = {
        "Middle slide": 185,
        "Outer slide": 370,
        "Right capture yoke": 370,
        "Left capture yoke": 370,
    }
    checked = []
    disagreements = {}
    for raw, lo in original["stowed"].items():
        k = raw.replace("/", "--")
        if k not in reg:
            continue
        diff = [b - a for a, b in zip(lo, original["bounds"][raw])]
        travel = min((0, 185, 370), key=lambda x: abs(x - diff[3]))
        if abs(diff[3] - travel) > 0.02 or any(
            abs(diff[i]) > 0.01 for i in (0, 1, 2, 4, 5)
        ):
            raise ValueError(("non-translating bounding envelope", k, diff))
        checked.append(k)
        if travel != lengths.get(reg[k], 0):
            disagreements[k] = {"prior": reg[k], "maxXDeltaMm": travel}
    envelope_sources = {
        "moving-camera-public-envelope": "roof-head/",
        "fixed-camera-public-envelope": "fixed-head/",
        "signal-lamp-public-envelope": "am-3583",
    }
    alias = {}
    for display, prefix in envelope_sources.items():
        keys = [k for k in original["stowed"] if k == prefix or k.startswith(prefix)]
        delta = {
            round(original["bounds"][k][3] - original["stowed"][k][3], 5) for k in keys
        }
        expected = 370.0 if prefix == "roof-head/" else 0.0
        if not keys or delta != {expected}:
            raise ValueError(("Invalid source envelope frame", display, delta))
        alias[display] = {
            "nativeComponents": len(keys),
            "expectedTravelMm": expected,
            "originalParent": reg[display],
            "sourceIds": sorted(keys),
        }
        if expected != lengths.get(reg[display], 0):
            disagreements[display] = {"prior": reg[display], "maxXDeltaMm": expected}
    if set(disagreements) != set(corrections()) | {"sp1-live-cable"}:
        raise ValueError(("unreviewed inherited motion", disagreements))
    receipt = dict(
        sceneSha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        supportRegisterSha256=hashlib.sha256(
            (OWNER / "support.json").read_bytes()
        ).hexdigest(),
        publicEnvelopeSources=alias,
        hostUsbBoots={
            "source": "experiments/swerve-drive/packaging/rear_fit.py",
            "fixed": ["usb-A-roof-boot", "usb-A-fixed-boot"],
            "basis": "Both are rear host USB-A ends; the camera-side data/power plugs and tails have registered independent bounds.",
        },
        matchesReviewed=len(checked),
        ids=sorted(checked),
        inheritedDifferences=disagreements,
        variableLength={
            "sp1-live-cable": "Draw wire pays out from its fixed guide to the moving eye. The rigid public wire envelope depicts the zero-stroke length only."
        },
        movingSupportIds=support_ids(),
        corrected=corrections(),
    )
    destination.write_bytes((json.dumps(receipt, indent=2) + "\n").encode())


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("scene", type=Path)
    p.add_argument("destination", type=Path)
    a = p.parse_args()
    audit(a.scene, a.destination)
