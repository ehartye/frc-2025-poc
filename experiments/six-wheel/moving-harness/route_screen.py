"""Bound cable length through the existing anchors and screen a rolling loop lane."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "design/fixed-no-air"))
from converter_panel import BODY_X, FILES, cad, lower_bound

ANCHORS = ("harness-fixed-anchor", "lift-harness-moving-anchor", "shelf-harness-moving-anchor")


def native_bounds(artifacts, name):
    step = artifacts / (name + ".step")
    if hashlib.sha256(step.read_bytes()).hexdigest() != FILES[name]:
        raise ValueError(f"Unrecognized exchange for {name}")
    receipt = json.loads(step.with_suffix(".json").read_bytes())
    manifest = json.loads((ROOT / "design/fixed-no-air/whole-robot.json").read_bytes())
    row = manifest["cases"][receipt["pose"]]
    names = sorted(set(row["identical_brep_sha256"]) | set(row["new_or_changed"]))
    parts = cad.read_named(step, [{"id": key} for key in names])
    return {key: np.array(cad.bounds(shape)) for key, shape in parts.items()}


def length_bounds(boxes):
    """Certify a lower bound by minimizing each linear support term over its box."""
    lo, hi = boxes[:, :3], boxes[:, 3:]
    points = (lo + hi) / 2
    for _ in range(601):
        segments = np.diff(points, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        units = segments / lengths[:, None]
        weights = np.vstack((-units[0], units[:-1] - units[1:], units[-1]))
        lower = np.minimum(weights * lo, weights * hi).sum()
        upper = lengths.sum()
        if upper - lower < 0.0001:
            return {"lowerMm": float(lower), "upperMm": float(upper)}
        points = np.clip(points - 8 * weights, lo, hi)
    raise ValueError("Route certificate did not converge")


def screen(artifacts):
    pickup = native_bounds(artifacts, "fixed-no-air/pickup")
    place = native_bounds(artifacts, "bridge-tolerance/place")
    # Use the entire top of the moving-camera converter as an optimistic terminal.
    x = BODY_X[-1]
    terminal = [x, -193, 277, x + 36, -134, 277]
    camera = np.array(json.loads((ROOT / "bridge-tolerance/mount.json").read_bytes())
                      ["parts"]["camera-envelope"]["bounds_mm"])
    routes = {}
    lanes = {}
    for stroke in (0, 185, 370):
        boxes = [terminal]
        for key in ANCHORS:
            boxes.append(pickup[key] + (place[key] - pickup[key]) * stroke / 370)
        boxes.append(camera + [stroke, 0, 0, stroke, 0, 0])
        routes[stroke] = length_bounds(np.array(boxes))
    for name, pose in (("pickup", pickup), ("placement", place)):
        lanes[name] = {}
        for y in (-270, -315):
            # Bounding volume for the candidate moving loop. Feed and tails are separate.
            lane = [-425, y - 10, 560, 455, y + 10, 840]
            gaps = sorted((lower_bound(lane, bounds), key) for key, bounds in pose.items())
            lanes[name][y] = {"minBoxGapMm": float(gaps[0][0]), "closest": gaps[0][1],
                              "overlappingBoxes": [key for gap, key in gaps if gap == 0]}
    return {"nativeSourceSha256": {k: FILES[k] for k in ("fixed-no-air/pickup", "bridge-tolerance/place")},
            "existingAnchors": routes, "candidateLoopEndpointBounds": lanes}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(screen(args.artifacts), indent=2) + "\n")
