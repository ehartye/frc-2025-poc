"""Place the field material in the moving camera view at the measured stations and scoring sectors."""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

from scenes import HERE, ROOT, rigid_mesh, combine

sys.path.insert(0, str(ROOT / "relieved-match"))
from cameras import mount, bridge, optics, camera
from registration import parts as station_parts
from sight import background, silhouette, bridge_fit
from exposures_check import frames, exposures, sha


def mesh(objects):
    out = []
    for k, shape in objects.items():
        vertices, tri = mount.original.triangulate(shape)
        out.append((k, np.asarray(vertices).reshape(-1, 3) / 10, tri))
    return out


def run(args):
    source = json.loads((args.scenes / "scenes.json").read_bytes())
    if sha(args.field) != source["sourceSha256"]["registered field STEP"]:
        raise ValueError("Unknown field")
    reef = mesh(
        bridge.mod("optical_field", ROOT / "moving-observer/clearance.py").reef_parts(
            args.field
        )
    )
    station = mesh(station_parts(args.field))
    base = np.array(
        json.loads((HERE / "mount.json").read_bytes())["registrations"]["roof"][
            "assumedRectifiedOriginMm"
        ]
    )
    with (ROOT / "relieved-match/landing-windows.csv").open() as f:
        settled = next(r for r in csv.DictReader(f) if r["case"] == "coral-A/250/115")
    prior = background(args.scenes / "barrel-moving-reference.npz")
    report = {}
    for piece in ("coral-A", "coral-B"):
        label, src, cargo, raw, pick, target = frames(piece, args.cache)
        released = json.loads(
            (ROOT / "relieved-match" / (piece + "-fixed.json")).read_bytes()
        )
        phase = source["operatingCases"][piece]
        cases = {}
        for kind, stroke in (("station", 0), ("reef", 370)):
            dx, dy, h = (
                phase["stationCorrectionMmDeg"]
                if kind == "station"
                else phase["reefRobotMmDeg"]
            )
            c, s = math.cos(math.radians(h)), math.sin(math.radians(h))

            def transform(v):
                if kind == "station":
                    x, y, z = 444.525 - dx - v[:, 0], -dy - v[:, 1], v[:, 2]
                else:
                    x, y, z = v[:, 0] - 3453.050980729168 - dx, -v[:, 2] - dy, v[:, 1]
                return np.column_stack((c * x + s * y, -s * x + c * y, z))

            material = rigid_mesh(
                station if kind == "station" else reef, transform, kind
            )
            optics.ORIGIN = base + [stroke, 0, 0]
            optics.configure()
            env = camera.images_for_pose({"parts": material})
            local = background(args.scenes / f"barrel-moving-{stroke}.npz")
            bg = combine(local, env)
            if kind == "reef" and piece == "coral-B":
                x, y = (
                    1000 * float(settled["x_m"]) - dx,
                    1000 * float(settled["y_m"]) - dy,
                )
                pos = np.array(
                    [c * x + s * y, -s * x + c * y, 1000 * float(settled["z_m"])]
                )
                axis = np.array([float(settled["axis_" + a]) for a in "xyz"])
                axis = np.array(
                    [c * axis[0] + s * axis[1], -s * axis[0] + c * axis[1], axis[2]]
                )
                images = silhouette(bg, pos, axis)
                # Exclude field/neighbor-contaminated backing from the optical inverse.
                bg = [
                    (im, np.where(im != g, np.minimum(z, base[2] - pos[2] - 162), z))
                    for im, (g, z) in zip(images, bg)
                ]
            ref = []
            excluded = []
            for (g, z), (new, znew), (_, z_stable) in zip(local, bg, prior):
                mask = (znew + 0.1 < z) & (z_stable < 3490)
                excluded.append(int(mask.sum()))
                # The field may obstruct the pipe. It cannot furnish robot registration edges or end backing.
                ref.append((g, np.where(mask, 3500, z_stable)))
            optics.ORIGIN = base.copy()
            optics.configure()
            if kind == "station":
                ticks = range(
                    17 * math.ceil((src["firstPortBandMs"] - 17) / 17),
                    int(src["firstPortBandMs"]) + 153,
                    17,
                )
            else:
                ticks = range(
                    17 * math.ceil(released["releaseCameraMs"] / 17),
                    int(released["touchCameraMs"] + 35) + 1,
                    17,
                )
            measurements = {}
            for t in ticks:
                ctr, axis, _ = exposures.state_at(
                    piece, t, raw, src, cargo, pick, target
                )
                images = silhouette(bg, ctr, axis)
                got = bridge_fit.fit(images, ref)
                fact = [ctr[0], ctr[1], ctr[2] - 557.15]
                bb = [v for k in ("x_mm", "y_mm", "bottom_mm") for v in got.get(k, [])]
                error = (
                    max(
                        (
                            max(bb[i] - x, x - bb[i + 1], 0.0)
                            for i, x in zip((0, 2, 4), fact)
                        ),
                        default=None,
                    )
                    if len(bb) == 6
                    else None
                )
                measurements[t] = {
                    "state": got["status"],
                    "boundsMm": bb,
                    "truthOutsideMm": error,
                }
            cases[kind] = {
                "excludedRobotBackingPixels": excluded,
                "fieldBodies": len(material),
                "frames": measurements,
            }
        report[piece] = cases
    (HERE / "roof-field.json").write_text(
        json.dumps({"fieldSha256": sha(args.field), "cases": report}, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("scenes", "cache", "field"):
        p.add_argument(k, type=Path)
    run(p.parse_args())
