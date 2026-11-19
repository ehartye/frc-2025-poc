"""Reopen the new robot background and carry its reached station pose into the field approach."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "sector-alignment"))
from dock import tracecase
from scene import context
from vision import C, S, F, EYES

sys.path.insert(0, str(ROOT / "full-match"))
from match import corrected


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scene(registered, relief):
    ctx = context(registered)
    record = read(ROOT / "return-station/relief-dock.json")
    bg = relief / "station-views-relief.npz"
    if sha(bg) != record["stowedCameraBackgroundSHA256"] or record["sourceRobotBodies"] != 597:
        raise ValueError("unregistered camera/robot")
    raw = np.load(bg)
    bounds = read(ROOT / "sector-alignment/scene.json")["carryBoxMm"]
    v, u = np.indices(raw["left"].shape, dtype=float)
    up = (240 - v) / F
    directions = (C + up * S, (320 - u) / F, -S + up * C)
    for eye, name in zip(EYES, ("left", "right")):
        depth = raw[name + "_z"]
        near = np.full(depth.shape, -np.inf)
        far = np.full(depth.shape, np.inf)
        for i, (origin, ray) in enumerate(zip((-315, eye, 970), directions)):
            with np.errstate(divide="ignore", invalid="ignore"):
                start = (bounds[i] - origin) / ray
                end = (bounds[i + 3] - origin) / ray
            near = np.maximum(near, np.minimum(start, end))
            far = np.minimum(far, np.maximum(start, end))
        blocked = (near > 0) & (far >= near) & (near < depth)
        ctx[name] = np.where(blocked, 170, raw[name]).astype(np.uint8)
        ctx[name + "_z"] = np.where(blocked, near, depth)
    return ctx


def placed_route(pipe, delta):
    f = corrected()
    _, nominal, _ = f.routes(-.18 if pipe == "coral-A" else .22)
    shift = f.rot((delta[0] / 1000, delta[1] / 1000), nominal.start[2])
    start = (nominal.start[0] + shift[0], nominal.start[1] + shift[1],
             nominal.start[2] + math.radians(delta[2]))
    actual = f.Route(start)
    for segment in nominal.segments[:-1]:
        actual.segment(segment["kind"], segment["demand"])
    marker = nominal.segments[-1]["start"]
    return nominal, actual, [(actual.end[0] - marker[0]) * 1000,
                             (actual.end[1] - marker[1]) * 1000, math.degrees(f.short(actual.end[2] - marker[2]))]


def return_after_a(reef_delta):
    f = corrected()
    reference = f.routes(-.18)[2]
    nominal = f.Route(reference.start)
    for segment in reference.segments[:-2]:
        nominal.segment(segment["kind"], segment["demand"])
    bias = read(ROOT / "return-station/return.json")["fixedStationLeftBiasMm"]
    correction = f.rot((0, bias / 1000), reference.end[2])
    nominal.go(f.add(reference.end, correction))
    nominal.face(reference.end[2])
    reached = f.Route((reference.start[0] + reef_delta[0]/1000, reference.start[1] + reef_delta[1]/1000,
                       reference.start[2] + math.radians(reef_delta[2])))
    for segment in nominal.segments:
        reached.segment(segment["kind"], segment["demand"])
    dx, dy = f.rot(f.sub(reached.end, reference.end), -reference.end[2])
    return [1000*dx, 1000*dy, math.degrees(f.short(reached.end[2] - reference.end[2]))]


def run(cache, prior, relief):
    ctx = scene(prior, relief)
    rows = []
    for piece, modes in (("coral-A", ["nominal", "dropout"]), ("coral-B", ["nominal"])):
        delta = [0, 0, 0] if piece == "coral-A" else return_after_a(rows[0]["finalBodyErrorMmDeg"])
        _, transit, residual = placed_route(piece, delta)
        for mode in modes:
            row = tracecase(piece, residual, mode, ctx, cache)
            row["actualStationMmDeg"] = delta
            row["stationToNavigationS"] = transit.duration
            rows.append(row)
            print(piece, mode, residual, row["accepted"], row["reason"], row["finalBodyErrorMmDeg"], flush=True)
    out = {"relievedBackgroundSha256": sha(relief / "station-views-relief.npz"),
           "carryBoxMm": read(ROOT / "sector-alignment/scene.json")["carryBoxMm"], "cases": rows}
    (HERE / "navigation.json").write_text(json.dumps(out, indent=2) + "\n", encoding="utf8", newline="\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("cache", "prior", "relief"):
        parser.add_argument(arg, type=Path)
    args = parser.parse_args()
    run(args.cache, args.prior, args.relief)
