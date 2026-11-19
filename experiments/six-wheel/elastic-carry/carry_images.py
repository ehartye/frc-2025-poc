"""Read independent stereo exposures from the transported pipe and native robot backing."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "selected-optics"))
# The released parser imports the fixed observer by its short name.
from exposures_check import optics, camera, fit_row, background, silhouette
import scenes
from cameras import mount

sys.path.insert(0, str(ROOT / "shelf-structure"))
from custody_check import guard, cable_boxes
from tray import digest


def guide_layer():
    step = ROOT / "station-guide/narrow-face.step"
    solids = mount.cad.read_named(
        step, [{"id": "guide-positive-y"}, {"id": "guide-negative-y"}]
    )
    mesh = []
    for key, body in solids.items():
        points, triangles = mount.original.triangulate(body)
        mesh.append([key, "guide", "#a9a49c", points, triangles])
    return camera.images_for_pose({"parts": mesh})


def composite(scene, guide):
    return [
        (np.where(gz < z, gg, g), np.minimum(gz, z))
        for (g, z), (gg, gz) in zip(scene, guide)
    ]


def available(folder, name, register):
    p = folder / (name + ".npz")
    if digest(p) != register["backgroundSha256"][name]:
        raise ValueError("Changed native backing " + name)
    return background(p)


def shelf_guard(actual, reference, pupil_z, same_finish, magnitude=2.0):
    # A 2 mm change in Z moves a roof pixel by less than 1.9 pixels
    # at the 320 px horizontal edge and the closest allowed shelf plane.
    radius = math.ceil(320 * magnitude / (pupil_z - 500.8 - magnitude))
    output = []
    known = []
    for (gray, z), (back, guaranteed) in zip(actual, reference):
        h, w = z.shape
        padz = np.pad(z.astype(float), radius, mode="edge")
        padg = np.pad(gray, radius, mode="edge")
        touches = np.zeros(z.shape, dtype=bool)
        uniform = np.ones(z.shape, dtype=bool)
        near = z.astype(float).copy()
        for y in range(2 * radius + 1):
            for x in range(2 * radius + 1):
                d = padz[y : y + h, x : x + w]
                g = padg[y : y + h, x : x + w]
                possible = (d >= pupil_z - 502.8) & (d <= pupil_z - 482.9)
                touches |= possible
                uniform &= (g == gray) & (abs(d - z.astype(float)) < 0.5)
                near = np.where(possible, np.minimum(near, d - magnitude), near)
        shade = np.where(touches, 220, gray).astype(np.uint8) if same_finish else gray
        output.append((shade, near))
        known.append((shade, np.where(touches & ~uniform, 3500, guaranteed)))
    return output, known


def camera_views(assets, folder, head):
    reg = json.loads((ROOT / "selected-optics/mount.json").read_bytes())[
        "registrations"
    ]
    if head.startswith("roof"):
        tag = {
            "roof": "barrel",
            "roof-guard": "barrel",
            "roof-050": "barrel",
            "roof-same": "barrel",
            "roof-front": "front",
            "roof-rear": "rear",
        }[head]
        dz = {"barrel": 0.0, "front": -0.75, "rear": 0.75}[tag]
        optics.ORIGIN = np.array(reg["roof"]["assumedRectifiedOriginMm"]) + [0, 0, dz]
        optics.configure()
        out = folder / "marked"
        scene = json.loads((out / "scenes.json").read_bytes())
        guide = guide_layer()
        actual = composite(available(out, tag + "-moving-0", scene), guide)
        stable = available(out, tag + "-moving-reference", scene)
        reference = [
            (g, np.minimum(z, gz))
            for (g, _), (_, z), (_, gz) in zip(actual, stable, guide)
        ]
        if head in ("roof-guard", "roof-same", "roof-050"):
            return shelf_guard(
                actual,
                reference,
                optics.ORIGIN[2],
                head == "roof-same",
                0.05 if head == "roof-050" else 2.0,
            )
        return actual, reference
    scenes.camera = camera
    scenes.fixed_pose(reg["fixed-datum-up"]["assumedRectifiedOriginMm"])
    nominal = json.loads((ROOT / "shelf-structure/background.json").read_bytes())
    full = available(
        assets / "shelf-structure/native-background", "stowed-robot-fixed", nominal
    )
    if head == "fixed-marked":
        from shelf_marks import apply
        from paths import O, F, U

        full, _ = apply(full, O, F, U, 0)
    # Neutralize all rays that lack robot backing. This prevents stationary
    # field contours being used as evidence as the chassis travels.
    full = [(np.where(z < 3490, g, 220).astype(np.uint8), z) for g, z in full]
    lead = assets / "fixed-camera-route/scene.json"
    if (
        digest(lead)
        != "291d68c43e21f20c33885064e6bbe8c0e47b684d13a8a06e8af61f9fd9db5886"
    ):
        raise ValueError("Unknown harness geometry")
    boxes = cable_boxes(json.loads(lead.read_bytes())["stowed"])
    masked, masks = guard(full, 2.0, boxes)
    if head == "fixed-dark":
        full = masked
    elif head == "fixed-same":
        full = [
            (np.where(mask, 220, g).astype(np.uint8), z)
            for (g, z), mask in zip(masked, masks)
        ]
    elif head not in ("fixed-native", "fixed-marked"):
        raise ValueError(head)
    actual = composite(full, guide_layer())
    return actual, actual


def outcome(fit, xyz):
    label, *edges = fit
    return (
        label in ("inside", "position-witness")
        and len(edges) == 6
        and all(
            low <= value <= high
            for low, value, high in zip(edges[::2], xyz, edges[1::2])
        )
    )


def run(assets, folder, rows, head, selected_step=0.0000625):
    actual, reference = camera_views(assets, folder, head)
    results = []
    for r in rows:
        side = r["case"]
        if r["stepS"] != selected_step:
            continue
        stem = folder / "carry" / f"{side}-{round(selected_step * 1e9)}ns.npy"
        if digest(stem) != r["traceSha256"]:
            raise ValueError("Changed transferred pipe")
        data = np.load(stem, mmap_mode="r")
        feed = folder / "checkpoints" / (side + ".npy")
        d = next(
            d
            for d in json.loads((HERE / "departure.json").read_bytes())["cases"]
            if d["case"] == side
        )
        if digest(feed) != d["stationTrajectorySha256"]:
            raise ValueError("No matched stationary state")
        supply = np.load(feed, mmap_mode="r")
        prior = np.array([np.interp(5457, supply[:, 0], v) for v in supply[:, 1:7].T])
        clock = np.r_[5457.0, 5460.0 + data[:, 0] * 1000]
        vec = np.vstack([prior, np.column_stack((data[:, 1:4] * 1000, data[:, 4:7]))])
        last = int(17 * np.floor(clock[-1] / 17))
        memo = {}
        frames = []
        for t in range(5457, last + 1, 17):
            v = np.array([np.interp(t, clock, col) for col in vec.T])
            pair = silhouette(actual, v[:3], v[3:] / np.linalg.norm(v[3:]))
            h = hashlib.sha256(b"".join(img.tobytes() for img in pair)).hexdigest()
            if h not in memo:
                memo[h] = fit_row(pair, reference, head.startswith("roof"), 0)
            fit = memo[h]
            contained = outcome(fit, [v[0], v[1], v[2] - 557.15])
            frames.append((t, h, fit, contained))
            if not contained:
                break
        first = next((r for r in frames if not r[3]), None)
        results.append(
            {
                "case": side,
                "stepS": selected_step,
                "head": head,
                "sourceTrace": digest(stem),
                "tested": len(frames),
                "uniqueRasters": len(memo),
                "firstSampleJavaMs": 5457,
                "lastSampleJavaMs": frames[-1][0],
                "firstUnresolvedJavaMs": first[0] if first else None,
                "firstUnresolvedServiceWallS": r["startWallS"]
                + (5 * math.ceil((first[0] + 15) / 5) - 5460) / 1000
                if first
                else None,
                "lastVerdictAndIntervals": frames[-1][2],
                "frameReceiptSha256": hashlib.sha256(
                    json.dumps(frames, sort_keys=True).encode()
                ).hexdigest(),
            }
        )
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("artifacts", "folder", "rows", "out"):
        p.add_argument(k, type=Path)
    p.add_argument(
        "head",
        choices=[
            "roof",
            "roof-front",
            "roof-guard",
            "roof-050",
            "roof-same",
            "roof-rear",
            "fixed-native",
            "fixed-marked",
            "fixed-dark",
            "fixed-same",
        ],
    )
    p.add_argument("--step", type=float, default=0.0000625)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(
            run(a.artifacts, a.folder, json.loads(a.rows.read_bytes()), a.head, a.step),
            indent=2,
        )
        + "\n",
        encoding="utf8",
        newline="\n",
    )
