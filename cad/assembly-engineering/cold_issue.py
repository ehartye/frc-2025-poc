"""Compare the cold Fusion readback to each expected world-space part in the frozen issue."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from attachments import base, volume
from installed_audit import HERE, ROOT, reopen, sha
from issue import rows, document
from shop_sources import issue_shapes
from clearance import read_assembly


def precise(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    return list(box.CornerMin().Coord() + box.CornerMax().Coord())


def compare(folder):
    record = json.loads((folder / "interfaces.json").read_bytes())
    source = folder / "station-and-jack-interfaces.step"
    assert sha(source) == record["outputSha256"]
    original = {k: s for k, *_, s in reopen()}
    demo = next(
        s
        for g, k, s in read_assembly(ROOT / "cad/swerve-captive/public-robot.step")
        if g == "Demonstration CORAL"
    )
    original["Practice CORAL tube"] = demo
    actual = {
        r["identity"]: r
        for r in json.loads((folder / "engineering-reopen.json").read_bytes())["bodies"]
    }
    sorted_parts = {
        k: v
        for k, v in (original | issue_shapes(folder, record)).items()
        if k not in record["fixedHeight"]["removed"]
    }
    if len(sorted_parts) != len(actual):
        raise ValueError("Body inventory size changed")
    output = []
    for k, s in sorted(sorted_parts.items()):
        ident = hashlib.sha256(("team-1780-engineering-v1/" + k).encode()).hexdigest()
        r = actual[ident]
        v = volume(s)
        native = r["volumeCm3"] * 1000
        dx = max(abs(x - y) for x, y in zip(precise(s), r["boundsMm"]))
        ev = abs(native - v)
        brep = abs(1000 * r["brepVolumeCm3"] - v)
        default = abs(1000 * r["defaultProxyVolumeCm3"] - v)
        output.append(
            dict(
                id=k,
                occurrence=r["occurrence"],
                expectedMm3=v,
                worldBoundsMaxMm=dx,
                highAccuracyErrorMm3=ev,
                analyticalBRepErrorMm3=brep,
                defaultProxyErrorMm3=default,
                passes=dx < 0.10 and min(ev, brep) <= max(0.08, 0.0005 * v),
            )
        )
    with (folder / "cold-comparison.csv").open(
        "w", encoding="utf8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=output[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(output)
    joints = json.loads((folder / "engineering-joints.json").read_bytes())

    def moved(r):
        if "samples" not in r:
            return True
        a, b, c = r["samples"]
        delta = max(abs(v - w) for key in ("a", "b") for v, w in zip(a[key], b[key]))
        reset = max(abs(v - w) for key in ("a", "b") for v, w in zip(a[key], c[key]))
        return (
            delta > 0.000001
            and reset < 0.0001
            and abs(b["readback"] - b["command"]) < 0.0001
        )

    summary = dict(
        archiveSha256=sha(
            folder / record.get("filename", "Team-1780-engineering-shallow.f3d")
        ),
        interfaceSha256=sha(source),
        observationSha256=sha(folder / "engineering-reopen.json"),
        perBodySha256=sha(folder / "cold-comparison.csv"),
        measuredBodies=len(output),
        boundsToleranceMm=0.10,
        absoluteVolumeToleranceMm3=0.08,
        relativeTolerance=0.0005,
        minVolumeMethod="Use either the native solid-volume or very-high accuracy physical properties within the independent tolerance. Preserve the default cached proxy as a control.",
        failingBodies=[r for r in output if not r["passes"]],
        totalJoints=len(joints),
        movedJoints=sum("samples" in r for r in joints.values()),
        failedJointRuns=[k for k, r in joints.items() if not moved(r)],
    )
    document(folder / "cold-reopen.json", summary)
    print(
        json.dumps({k: v for k, v in summary.items() if k != "failingBodies"}),
        len(summary["failingBodies"]),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", type=Path)
    compare(parser.parse_args().folder)
