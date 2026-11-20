"""Collect only time-joined, identity-registered low-placement runs and their adverse controls."""

import argparse
from pathlib import Path
import hashlib
import json
import csv

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stage_mass():
    issue = ROOT / "cad/assembly-engineering/receiver-service/issue"
    native = read(issue / "engineering-reopen.json")
    owners = {r["id"]: r for r in read(issue / "mapping.json")["instances"]}
    groups = {}
    for row in native["bodies"]:
        k = row["occurrence"].split(":")[0]
        if k in (
            "Middle slide",
            "Outer slide",
            "Left capture yoke",
            "Right capture yoke",
        ):
            groups.setdefault(k, set()).add(row["instance"])
    if sum(len(x) for x in groups.values()) != len(set.union(*groups.values())):
        raise ValueError("One owner claimed by two moving groups")
    return {
        k: dict(
            physicalOwners=len(s),
            installedUpperKg=sum(owners[x]["mass"]["rangeKg"][1] for x in sorted(s)),
        )
        for k, s in groups.items()
    }


def collect(artifacts):
    root = artifacts / "low-placement"
    cohort = read(HERE / "cohort.json")
    out = dict(
        cohort=sha(HERE / "cohort.json"),
        installedMovingMass=stage_mass(),
        cases={},
        controls={},
    )
    for side in ("plus", "minus"):
        folder = root / "advance-25" / side
        chassis = read(folder / "chassis.json")
        piece = read(folder / "coral.json")
        if (
            chassis["sourcePipe"] != piece["source"]
            or chassis["route"] != piece["wheels"]
            or chassis["piece"] != piece["piece"]
            or abs(piece["endWallS"] - chassis["endWallS"]) > 1e-7
            or sha(folder / "coral.npz") != piece["state"]
            or sha(folder / "end-wheels.npz") != chassis["terminalWheels"]
        ):
            raise ValueError("Unregistered corrective dock")
        out["cases"][side] = dict(
            chassis={k: v for k, v in chassis.items() if k != "cornerReview"},
            cargo=piece,
            sourceChassis=sha(folder / "chassis.json"),
            sourceCargo=sha(folder / "coral.json"),
            rack=read(folder / "rack-337mm-31.25us.json"),
        )
        out["controls"][side] = [
            read(folder / (name + ".json")) for name in ("missing-eye", "lost-packets")
        ]
        refinements = []
        for f in sorted(folder.glob("coupled-*.json")):
            data = read(f)
            if (
                data["sourceState"] != piece["state"]
                or data["piece"] != piece["piece"]
                or sha(f.with_suffix(".npy")) != data["trace"]
            ):
                raise ValueError("Incompatible free-pipe continuation")
            refinements.append(data)
        out["cases"][side]["freePipe"] = refinements
    ref = cohort["cases"][2]
    inputs = root / "holdout" / "minus"
    optics = read(root / "heldout-station.json")
    station = read(inputs / "station-piece.json")
    if (
        optics["feed"] != ref["firstFeedReceipt"]
        or sha(inputs / "station-piece.npz") != station["state"]
    ):
        raise ValueError("Different held-out station")
    out["holdout"] = dict(
        input=ref["firstFeedReceipt"],
        optics=optics,
        station=station,
        routeWheels=read(inputs / "route/route.json"),
    )
    prior = inputs / "route-piece.json"
    if prior.exists():
        run = read(prior)
        if (
            run["before"] != station["state"]
            or sha(inputs / "route-piece.npz") != run["state"]
            or run["wheel"] != out["holdout"]["routeWheels"]["trajectory"]
        ):
            raise ValueError("Different held-out carried pipe or wheels")
        out["holdout"]["routePiece"] = run
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    a.output.write_bytes(
        (json.dumps(collect(a.artifacts), indent=2, allow_nan=False) + "\n").encode()
    )
