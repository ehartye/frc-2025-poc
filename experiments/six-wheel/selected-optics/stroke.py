"""Bound uncertain moving parts in the roof-camera frame during the complete slide stroke."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from scenes import ROOT, HERE, load_assembly, selected_poses, store


def run(args, select_poses=selected_poses):
    model = load_assembly(args.assembly)
    sys.path.insert(0, str(ROOT / "relieved-match"))
    from cameras import box_near, camera, optics, receiver_drives
    from sight import background
    from mechanics import digest

    scene = json.loads((args.out / "scenes.json").read_bytes())
    if digest(args.assembly) != scene["sourceSha256"]["selected-match/clearance.py"]:
        raise ValueError("Selected assembly builder changed after rendering")
    stowed, placed = select_poses(model, args.artifacts, args.lamp)
    fixed, slide = model.wire_boxes()
    a = {k: np.array(model.cad.bounds(v)) for k, v in stowed.items()}
    b = {k: np.array(model.cad.bounds(v)) for k, v in placed.items()}
    shift = np.array([370.0, 0.0, 0.0, 370.0, 0.0, 0.0])
    for k, v in fixed.items():
        a["wire/" + k] = v
        b["wire/" + k] = v
    for k, v in slide.items():
        a["wire/" + k] = v
        b["wire/" + k] = v + shift
    # Include installed fixed case/shoes and screw heads in the world-to-slide sweep.
    report = json.loads((HERE / "mount.json").read_bytes())
    for k, data in report["reopenedParts"].items():
        a[k] = b[k] = np.array(data["boundsMm"])
    for name in (
        "observer-slotted-cradle-weldment",
        "observer-camera-proxy",
        "roof-camera-envelope",
        "wire/registered-camera-case",
    ):
        a.pop(name)
        b.pop(name)
    m = np.array(report["registrations"]["fixed-datum-up"]["assumedRectifiedOriginMm"])
    # All fixed housing corners in the 26-degree registration are within this sphere at the mount.
    a["fixed-native-housing"] = b["fixed-native-housing"] = np.r_[m - 55, m + 55]
    dynamic = {}
    for name, b0 in a.items():
        b1 = b[name] - shift
        grip = receiver_drives.translates_with_grip(name) or name.startswith("grip-")
        rigid = np.max(abs(b0 - b1)) < 0.001
        if rigid and not (grip or name.startswith(("wire/", "support/"))):
            continue
        pad = np.zeros(3)
        if "pinion" in name:
            pad[:] = 22.0
        if grip:
            pad[1] += 1.5
        if name == "carrier":
            pad[0] += 15.0
        dynamic[name] = np.r_[
            np.minimum(b0[:3], b1[:3]) - pad, np.maximum(b0[3:], b1[3:]) + pad
        ]
    source = background(args.out / "barrel-moving-reference.npz")
    optics.ORIGIN = np.array(
        report["registrations"]["roof"]["assumedRectifiedOriginMm"]
    )
    optics.configure()
    result = []
    counts = []
    effect_by_eye = []
    for eye, (g, z) in zip(camera.EYES, source):
        origin = optics.ORIGIN + [0, eye, 0]
        nearest = np.full(g.shape, np.inf)
        effects = {}
        for name, bounds in dynamic.items():
            if bounds[2] > origin[2] or bounds[5] < 195:
                continue
            d = box_near(origin, (camera.DX, camera.DY, camera.DZ), bounds)
            used = (d <= z + 0.1) & (z < 3490)
            if used.any():
                effects[name] = int(used.sum())
                nearest = np.minimum(nearest, d)
        remove = (nearest <= z + 0.1) & (z < 3490)
        result.append(
            (np.where(remove, 0, g).astype(np.uint8), np.where(remove, 3500, z))
        )
        effect_by_eye.append(set(effects))
        counts.append(
            {
                "guardedRays": int(remove.sum()),
                "originalBackedRays": int((z < 3490).sum()),
                "possibleOccluders": effects,
            }
        )
    out = {
        "selectedAssemblySha256": digest(args.assembly),
        "sourceReferenceSha256": scene["backgroundSha256"]["barrel-moving-reference"],
        "dynamicEnvelopes": len(dynamic),
        "eyeCounts": counts,
        "backgroundSha256": store(args.out, "barrel-moving-coarse", result),
    }
    # Refine the broad-box challenge using swept native triangle faces. An extruded
    # triangle prism contains the complete translation of that face, including
    # intermediate poses. Interior prism faces may over-occlude. Curved mesh
    # features retain the optical renderer's approximation and physical calibration gate.
    from registration import mount

    candidates = set.union(*effect_by_eye)
    bodies = []
    broad = {}

    def mesh_sweep(name, shape, direction, start):
        pts, indices = mount.original.triangulate(shape)
        points = np.asarray(pts, float).reshape(-1, 3) + 10 * start
        triangles = np.asarray(indices, int).reshape(-1, 3)
        if np.linalg.norm(direction) < 1e-5:
            return [
                name,
                "shop",
                "#737d85",
                points.ravel().tolist(),
                triangles.ravel().tolist(),
            ]
        n = len(points)
        both = np.vstack((points, points + 10 * direction))
        edges = np.vstack(
            [triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]]
        )
        edges = np.unique(np.sort(edges, axis=1), axis=0)
        i, j = edges.T
        faces = np.vstack(
            (
                triangles,
                triangles + n,
                np.column_stack((i, j, j + n)),
                np.column_stack((i, j + n, i + n)),
            )
        )
        return [name, "shop", "#737d85", both.ravel().tolist(), faces.ravel().tolist()]

    replaced = []
    for k in sorted(candidates):
        if k in ("wire/power-feed", "wire/data-feed"):
            # Accumulated sample positions within each section are no farther than PITCH/2
            # from the exact centerline. Each section translates -370 along shelf X.
            path, radius = (
                (model.power_feed(-329), 4.35)
                if k.endswith("power-feed")
                else (model.data_feed(-317), 3.5)
            )
            for n, (_, points) in enumerate(path):
                for j in range(0, len(points) - 1, 8):
                    chunk = np.asarray(points[j : j + 9])
                    pad = radius + model.PITCH / 2
                    lo, hi = chunk.min(axis=0) - pad, chunk.max(axis=0) + pad
                    broad[f"{k}/{n}/{j}"] = np.r_[lo - [370, 0, 0], hi]
            replaced.append(k)
            continue
        if k not in stowed or "pinion" in k or k.endswith("-spring"):
            broad[k] = dynamic[k]
            continue
        b0, b1 = a[k], b[k] - shift
        delta = (b1[:3] + b1[3:] - b0[:3] - b0[3:]) / 2
        if np.max(np.abs((b1[3:] - b1[:3]) - (b0[3:] - b0[:3]))) > 0.005:
            broad[k] = dynamic[k]
            continue
        # Coupled grip parts co-move with the slide; bracket their allowed
        # lateral movement in either direction. Static support feet and towers
        # keep their actual faces instead of inheriting the belt travel.
        start = np.zeros(3)
        if receiver_drives.translates_with_grip(k):
            if np.linalg.norm(delta) > 0.005:
                raise ValueError("Unexpected non-lateral grip motion " + k)
            start[1] = -1.5
            delta = np.array([0.0, 3.0, 0.0])
        bodies.append(mesh_sweep(k, stowed[k], delta, start))
    exact = camera.images_for_pose({"parts": bodies})
    guarded = []
    worstcase = []
    refined = []
    for eye, (g, z), (_, depth) in zip(camera.EYES, source, exact):
        origin = optics.ORIGIN + [0, eye, 0]
        nearest = depth
        for bounds in broad.values():
            if bounds[2] > origin[2] or bounds[5] < 195:
                continue
            nearest = np.minimum(
                nearest, box_near(origin, (camera.DX, camera.DY, camera.DZ), bounds)
            )
        mask = (nearest < z + 0.2) & (z < 3490)
        guarded.append((np.where(mask, 0, g).astype(np.uint8), np.where(mask, 3500, z)))
        # Keep only rays with opaque slide-fixed material behind the piece. Every uncertain
        # nearer component can occlude it; the union is the nearest possible depth.
        worstcase.append((g.copy(), np.where(z < 3490, np.minimum(z, nearest), 3500)))
        refined.append(
            {
                "guardedRays": int(mask.sum()),
                "backedRaysRemaining": int(((~mask) & (z < 3490)).sum()),
            }
        )
    out["broadBoxBackgroundSha256"] = out.pop("backgroundSha256")
    out["refinedCounts"] = refined
    out["translatedNativeBodies"] = len(bodies)
    out["refinedFeedEnvelopes"] = replaced
    out["backgroundSha256"] = store(args.out, "barrel-moving-swept", guarded)
    out["nearestPossibleSha256"] = store(args.out, "barrel-moving-nearest", worstcase)
    (args.out / "sweep.json").write_text(
        json.dumps(out, indent=2) + "\n", encoding="utf-8"
    )
    print([(v["guardedRays"], v["originalBackedRays"]) for v in counts], flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("artifacts", "lamp", "out"):
        p.add_argument(k, type=Path)
    p.add_argument(
        "--assembly", type=Path, default=ROOT / "selected-match/clearance.py"
    )
    run(p.parse_args())
