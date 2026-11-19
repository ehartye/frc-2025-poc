"""Project the actual exposed robot facing into each empty camera at both attained docks."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "motion"))
import receiver_witness as witness

FACE = "r6-polymer-facing"


def top_face(parts):
    (part,) = [p for p in parts if p[0] == FACE]
    xyz = np.asarray(part[3]).reshape(-1, 3) / 10
    triangles = np.asarray(part[4]).reshape(-1, 3)
    at_top = (xyz[triangles][:, :, 2] == 500).all(axis=1)
    surface = triangles[at_top]
    corners = xyz[surface]
    area = (
        np.linalg.norm(
            np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]),
            axis=1,
        ).sum()
        / 2
    )
    if not len(surface) or not 0 < area <= 265 * 420:
        raise ValueError("Unregistered upper face")
    return (*part[:4], surface.ravel().tolist()), area


def inputs(docks=None):
    if docks is None:
        pairs = [
            (witness.HERE / "first-feed.json", witness.HERE / "receiver-witness.json")
        ]
    else:
        pairs = [
            (p.parent / "feed.json", p) for p in sorted(docks.glob("*us/station.json"))
        ]
    if not pairs:
        raise ValueError("No registered feeds and station images")
    results = []
    feeds = []
    pins = []
    expected_model = witness.read(witness.HERE / "receiver-witness.json")
    seen = set()
    for feed_path, frames_path in pairs:
        record = witness.read(frames_path)
        if record["firstFeedSha256"] != witness.digest(feed_path):
            raise ValueError("Station evidence did not use these feeds")
        for feed in witness.read(feed_path)["attempts"]:
            key = (feed["case"], feed["plantStepUs"])
            if key in seen or feed["state"] != "IN_BAND" or feed["javaLastMs"] != 5460:
                raise ValueError("Duplicate or incomplete station acquisition")
            seen.add(key)
            feeds.append(feed)
        if any(
            record[key] != expected_model[key]
            for key in (
                "stepSha256",
                "fieldSha256",
                "appearanceRoleMapSha256",
                "colorGroups",
                "cameraMountSha256",
            )
        ):
            raise ValueError("Unregistered robot, field, or cameras")
        results.extend(c for c in record["cases"] if c["finish"] == "matched")
        pins.append(witness.digest(frames_path))
        if record["stepSha256"] != witness.digest(
            witness.PROJECT / "cad/full-swerve-r4/full-robot.step"
        ):
            raise ValueError("Different robot registration")
    return feeds, results, pins


def build(field, output, docks=None):
    expected = witness.read(witness.LEGACY / "full-match/installed-stock.json")[
        "field_step_sha256"
    ]
    if witness.digest(field) != expected:
        raise ValueError("Changed station assembly")
    output.mkdir(parents=True, exist_ok=True)
    parts = witness.source_parts()
    surface, area = top_face(parts)
    roof = witness.render(parts, "roof")
    camera = witness.sensor.camera
    target = camera.images_for_pose({"parts": [surface]})
    marked = [(*p[:2], "#ffffff" if p[0] == FACE else "#000000", *p[3:]) for p in parts]
    identity = witness.render(marked, "roof")
    fixed = witness.render(parts, "fixed")
    fixed_target = camera.images_for_pose({"parts": [surface]})
    fixed_identity = witness.render(marked, "fixed")
    environment = witness.field_mesh(field)
    feeds, _, _ = inputs(docks)
    evidence = {}
    for name in sorted({f["case"] for f in feeds}):
        x, y, yaw = next(
            f["trueChassisAtFeedMmMmDeg"] for f in feeds if f["case"] == name
        )
        witness.sensor.scenes.fixed_pose(witness.O)
        fixed_scene = witness.scene(environment, [-x, -y, yaw], fixed)
        witness.sensor.optics.configure()
        roof_scene = witness.scene(environment, [-x, -y, yaw], roof)
        record = {}
        for eye in range(2):
            fg, fz = fixed_scene[eye]
            g, z = roof_scene[eye]
            _, rz = roof[eye]
            _, target_z = target[eye]
            idr, _ = identity[eye]
            # The target must win the complete robot ID pass as well as the field depth test.
            target_ok = (
                (idr == 167)
                & (target_z < 3490)
                & (abs(target_z - z) < 0.002)
                & (abs(rz - z) < 0.002)
            )
            safe_z = np.where(rz <= z + 0.01, rz, 3500.0)
            fixed_mask = (
                (fixed_identity[eye][0] == 167)
                & (fixed_target[eye][1] < 3490)
                & (abs(fixed_target[eye][1] - fz) < 0.002)
                & (abs(fixed[eye][1] - fz) < 0.002)
            )
            for k, a in dict(
                fg=fg,
                fz=fz,
                g=g,
                z=z,
                rz=safe_z,
                target=target_ok,
                fixedTarget=fixed_mask,
                allFaces=(idr == 167) & (abs(rz - z) < 0.002),
                fixedAll=(fixed_identity[eye][0] == 167)
                & (abs(fixed[eye][1] - fz) < 0.002),
            ).items():
                record[f"{k}{eye}"] = a
        path = output / (name + ".npz")
        np.savez_compressed(path, **record)
        evidence[name] = {
            "dockMmMmDeg": [x, y, yaw],
            "visibleTopPixels": [int(record[f"target{i}"].sum()) for i in range(2)],
            "fixedTopPixels": [int(record[f"fixedTarget{i}"].sum()) for i in range(2)],
            "roofAllPixels": [int(record[f"allFaces{i}"].sum()) for i in range(2)],
            "fixedAllPixels": [int(record[f"fixedAll{i}"].sum()) for i in range(2)],
        }
    metadata = dict(
        fieldSha256=expected,
        robotSha256=witness.digest(
            witness.PROJECT / "cad/full-swerve-r4/full-robot.step"
        ),
        sourceTopAreaMm2=area,
        physicalPart=FACE,
        docks=evidence,
    )
    (output / "surfaces.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    return metadata


if __name__ == "__main__":
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("field", type=Path)
    args.add_argument("output", type=Path)
    args.add_argument("--docks", type=Path)
    p = args.parse_args()
    print(json.dumps(build(p.field, p.output, p.docks), indent=2))
