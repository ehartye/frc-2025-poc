"""Build and reopen the removable fixed-camera locating shoes and drilled cradle."""

import argparse
import json
import re
from pathlib import Path

import numpy as np
from OCP.STEPControl import STEPControl_Reader
from unittest.mock import patch

import mechanics as m
from camera_parts import box, cylinder, cut, unite, bore_hex

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CAD = m.vendor.cad
BASIS = np.column_stack((m.FORWARD, m.LEFT, m.UP))


def read_mount():
    base = ROOT / "release-observer"
    source = json.loads((base / "mount-check.json").read_bytes())
    if m.digest(base / "stereo-mount.step") != source["mount_sha256"]:
        raise ValueError("Changed fixed cradle")
    reader = STEPControl_Reader()
    reader.ReadFile(str(base / "stereo-mount.step"))
    if reader.TransferRoots() != len(source["part_order"]):
        raise ValueError("Unexpected fixed mount STEP roots")
    return {k: reader.Shape(i) for i, k in enumerate(source["part_order"], 1)}


def local(shape):
    return m.vendor.transform(shape, BASIS, m.FIXED)


def gap(a, b):
    return float(m.vendor.distance(a, b))


def run(folder, step, receipt):
    old = read_mount()
    cradle = old["slotted-cradle-weldment"]
    cases, data = m.registration(folder, cradle)
    native = cases["fixed-datum-up"][0]
    bodies = {}
    screws = {}
    driver = {}
    tools = []
    for side, y in (("L", 37.5), ("R", -37.5)):
        # Local X is forward; the seat was measured from the enclosure lens-axis midpoint.
        rear = box(-23, y - 5, -15, 9.78, 10, 21.2)
        foot = box(-23, y - 5, -15, 25, 10, 6.039)
        pilot = cylinder(-32, y, 0.539, 2.25, 24, (1, 0, 0))
        bodies["fixed-camera-shoe-" + side] = local(cut(unite(rear, foot), pilot))
        tools.append(local(pilot))
        # A rotational envelope for an M4 x 18 socket head. Hex socket included.
        screw = cut(
            unite(cylinder(0, 0, -18, 2, 18), cylinder(0, 0, 0, 3.5, 4)),
            bore_hex(0, 0, 2, 3, 2.1),
        )
        screw = m.vendor.transform(
            screw, [[0, 0, -1], [1, 0, 0], [0, -1, 0]], [-27, y, 0.539]
        )
        key = "fixed-camera-M4-" + side
        bodies[key] = local(screw)
        screws[key] = bodies[key]
        driver[key] = local(cylinder(-131, y, 0.539, 3.5, 100, (1, 0, 0)))
    bodies["fixed-camera-cradle-drilled"] = cut(cradle, *tools)
    if data["registrations"]["fixed-datum-up"]["cradleCaseOverlapMm3"] > 0.001:
        raise ValueError("Selected enclosure intersects cradle")
    labels = [{"id": k, "group": "fixed"} for k in bodies]
    writer = CAD.STEPCAFControl_Writer

    def plain():
        instance = writer()
        instance.SetColorMode(False)
        return instance

    with patch.object(CAD, "STEPCAFControl_Writer", plain):
        CAD.write_named(step, list(bodies.values()), labels, {"fixed": "#7799aa"})
    step.write_text(
        re.sub(
            r"(FILE_NAME\('Open CASCADE Shape Model',')[^']+",
            r"\g<1>2026-11-19T00:00:00",
            step.read_text(encoding="ascii"),
            count=1,
        ),
        encoding="ascii",
        newline="\n",
    )
    reopened = CAD.read_named(step, labels)
    interference = {}
    names = list(reopened)
    for i, name in enumerate(names):
        shape = reopened[name]
        for other in names[i + 1 :]:
            v = CAD.intersection(shape, reopened[other])
            if v > 0.001:
                interference[name + "/" + other] = v
        if name not in screws:
            for j, body in enumerate(native):
                if CAD.occupied(CAD.bounds(shape), CAD.bounds(body)):
                    v = CAD.intersection(shape, body)
                    if v > 0.001:
                        interference[name + "/case-" + str(j)] = v
    if interference:
        raise ValueError(interference)
    checks = {}
    for name, tool in driver.items():
        fixed = {
            **{
                k: v
                for k, v in old.items()
                if k not in ("slotted-cradle-weldment", "camera-proxy")
            },
            **{k: v for k, v in reopened.items() if k != name},
        }
        checks[name] = min((gap(tool, v), k) for k, v in fixed.items())
    if min(g for g, _ in checks.values()) <= 0:
        raise ValueError(checks)
    volumes = {k: CAD.volume(s)[0] for k, s in reopened.items()}
    data.update(
        {
            "sourceMountSha256": m.digest(ROOT / "release-observer/stereo-mount.step"),
            "stepSha256": m.digest(step),
            "reopenedParts": {
                k: {"volumeMm3": volumes[k], "boundsMm": CAD.bounds(s)}
                for k, s in reopened.items()
            },
            "interferenceAbove001Mm3": interference,
            "driverClearanceMmAndClosestPart": checks,
            "holderDeltaMm3": volumes["fixed-camera-cradle-drilled"]
            - CAD.volume(cradle)[0],
            "addedShoesMm3": sum(v for k, v in volumes.items() if "-shoe-" in k),
            "addedFastenerEnvelopeMm3": sum(
                v for k, v in volumes.items() if "-M4-" in k
            ),
            "nominalThreadEngagementMm": 4.22,
        }
    )
    receipt.write_text(
        json.dumps(data, indent=2) + "\n", encoding="utf-8", newline="\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("vendor_folder", type=Path)
    args = parser.parse_args()
    run(args.vendor_folder, HERE / "fixed-datum.step", HERE / "mount.json")
