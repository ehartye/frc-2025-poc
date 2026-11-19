"""Publish compact results, rejecting missing material conditions and unjoined camera records."""

import argparse
import hashlib
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_bytes())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(root):
    conditions = {
        (sign, rho, mu, step)
        for sign in ("plus", "minus")
        for rho, mu, step in (
            (950, 0.05, 31.25),
            (950, 0.2, 31.25),
            (960, 0.05, 31.25),
            (960, 0.2, 31.25),
            (955, 0.1, 31.25),
            (955, 0.1, 15.625),
        )
    }
    collected = []
    for path in sorted(root.glob("rho-*/result.json")):
        case = path.parent
        old = read(case / "qualified-core.json")
        new = read(case / "qualified-rims-core.json")
        identity = (
            new["case"],
            new["densityKgM3"],
            new["shelfFriction"],
            new["stepUs"],
        )
        conditions.remove(identity)
        for k in (
            "pipeTrajectorySha256",
            "terminalStateSha256",
            "physicalPipeAtCaptures",
            "dockMmMmDeg",
            "javaState",
        ):
            if new[k] != old[k]:
                raise ValueError("A different input reached the two solvers")
        controls = {}
        for mode, before in old["observations"].items():
            after = new["observations"][mode]
            if before["images"] != after["images"]:
                raise ValueError("A detector changed its source images")
            controls[mode] = {
                "originalPeak": before["start"],
                "projectedRims": after["start"],
                "projected45ms": after["start45ms"],
                "acceptedExposures": after["captureAccepted"],
                "imageSeriesSha256": hashlib.sha256(
                    " ".join(before["images"]).encode()
                ).hexdigest(),
            }
        source = read(path)["feed"]
        row = {
            k: v
            for k, v in new.items()
            if k not in ("observations", "physicalPipeAtCaptures")
        }
        row.update(
            firstCapture=new["physicalPipeAtCaptures"][0]
            if new["physicalPipeAtCaptures"]
            else None,
            driveTraceSha256=source["driveTraceSha256"],
            observations=controls,
        )
        full = case / "qualified-rims-all.json"
        if full.is_file():
            row["matchedControls"] = {
                k: [v["start"], v["start45ms"]]
                for k, v in read(full)["observations"].items()
            }
            detail = read(case / "photometry-rims-all.json")
            row["rimSupportAtNominal"] = [
                f.get("rimCheck") for f in detail["nominal"]["frames"]
            ]
        lp = case / "loaded/result.json"
        if lp.is_file():
            data = read(lp)
            if data["materialState"] != new["terminalStateSha256"]:
                raise ValueError("Wrong loaded pipe")
            row["loaded"] = {
                "contact": data["contact"],
                "drivenMm": data["drive"]["commandedTravelMm"],
                "brakeDriftMm": data["drive"]["driftAfterBrakeMm"],
                "massKg": data["drive"]["loadedMassKg"],
            }
            for images in sorted(lp.parent.glob("images-*.json")):
                optical = read(images)
                frames = optical.pop("captures")
                row["loaded"][images.stem] = {
                    **optical,
                    "count": len(frames),
                    "accepted": sum(f["accepted"] for f in frames),
                    "captureSeriesSha256": hashlib.sha256(
                        json.dumps(frames, sort_keys=True).encode()
                    ).hexdigest(),
                }
        collected.append(row)
    if conditions:
        raise ValueError(f"Missing conditions {conditions}")
    return {
        "schema": "reefscape-black-facing-custody/1",
        "source": read(root / "calibrators.json"),
        "cases": collected,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    a.output.write_text(
        json.dumps(summarize(a.root), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
