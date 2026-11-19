"""Bound rays to the whole finite cylinder, then check registered interval and station transfers."""

import argparse
import itertools
from pathlib import Path
import sys
import numpy as np
from visibility import register, digest
from paths import F, U, L, O
from scene import ROOT, read, write

sys.path.insert(0, str(ROOT / "selected-optics"))
from exposures_check import exposures, frames
from evidence import table, survey_edges
from paths import Z

CORNERS = np.asarray(list(itertools.product((0, 1), repeat=3)))


def cones(boxes, eye, axes):
    b = np.asarray(boxes)
    pts = b[:, :3, None] + (b[:, 3:] - b[:, :3])[:, :, None] * CORNERS.T
    return (np.moveaxis(pts, 1, 2) - eye) @ axes.T


def possible(vertices, axes, origin, point, axis):
    center = axes @ (point - origin)
    a = axes @ axis
    extent = abs(a) * 150.8125 + 57.15 * np.sqrt(np.maximum(0, 1 - a * a))
    lo, hi = center - extent, center + extent
    if lo[0] <= 0:
        return -1
    sides = []
    for i in (1, 2):
        ratio = np.array([lo[i], hi[i]])[:, None] / [lo[0], hi[0]]
        sides.append([ratio.min(), ratio.max()])
    d = vertices[:, :, 0]
    eligible = (d.max(1) >= 0) & (d.min(1) < hi[0])
    for i, (a, b) in enumerate(sides, 1):
        eligible &= (vertices[:, :, i] - a * d).max(1) >= 0
        eligible &= (vertices[:, :, i] - b * d).min(1) <= 0
    return int(eligible.sum())


def run(args):
    sys.path.insert(0, str(ROOT / "station-landmark"))
    import panel

    b = read(args.bounds)["stowed"]
    # Removed opaque reservations cannot become foreground. Their lost backing is
    # handled separately in the grayscale/reference trials.
    from harness_fit import RETIRED

    cube = register(b)[: -len(RETIRED)]
    out = {
        "sourceMarkedSceneSha256": digest(args.source / "scenes.json"),
        "guideIssueSha256": digest(Path(__file__).with_name("guides.json")),
        "roofRegistration": survey_edges(args.images),
        "captures": {},
        "joined": {},
        "station": {},
    }
    for name in ("first-fixed", "return-fixed"):

        def detect(folder):
            v = np.load(folder / (name + ".npz"))
            return panel.observations([v[k] for k in ("left", "right")])

        prior, new = detect(args.source), detect(args.images)
        out["station"][name] = {
            "bothRectanglesUnique": prior is not None and new is not None,
            "maxShiftPx": float(abs(prior - new).max())
            if prior is not None and new is not None
            else None,
        }
    reg = read(ROOT / "selected-optics/mount.json")["registrations"]
    mats = {"moving": np.array([-Z, L, [1, 0, 0]]), "fixed": np.array([F, L, U])}
    bases = {"moving": np.array(reg["roof"]["assumedRectifiedOriginMm"]), "fixed": O}
    for piece in ("coral-A", "coral-B"):
        label, src, edge, raw, pick, target = frames(piece, args.raw)
        all_views = {}
        for head, suffix in (("moving", "all-bounded"), ("fixed", "critical"), ("seat", "all")):
            tag = f"{piece}-{head}-barrel-nominal-{suffix}"
            view = table(args.images, tag)
            previous = table(args.source, tag)
            filename = args.images / (tag + ".csv")
            report = read(args.images / (tag + ".json"))
            if digest(filename) != report["frameCsvSha256"]:
                raise ValueError("Changed capture CSV")
            count = 0
            unresolved = 0
            finite = 0
            outside = 0.0
            memo = {}
            origins = {}
            for tick, row in view.items():
                center, axis, stroke = exposures.state_at(piece, tick, raw, src, edge, pick, target)
                if head == "seat":
                    pos = np.array([np.interp(tick, pick[:, 0], s) for s in pick[:, 1:4].T])
                    axis = np.array([np.interp(tick, pick[:, 0], s) for s in pick[:, 4:7].T])
                    axis /= np.linalg.norm(axis)
                    stroke = 0
                    center = pos
                truth = [center[0], center[1], center[2] - 557.15]
                if len(row) == 7:
                    outside = max(
                        outside,
                        *[max(row[i + 1] - v, v - row[i + 2]) for i, v in zip((0, 2, 4), truth)],
                    )
                    finite += 1
                position = center.copy()
                position[0] += stroke
                kind = "moving" if head == "moving" else "fixed"
                mat = mats[kind]
                key = round(stroke, 6) if kind == "moving" else 0.0
                if key not in origins:
                    base = bases[kind] + [key, 0, 0]
                    origins[key] = [
                        (base + e * L, cones(cube, base + e * L, mat)) for e in (37.5, -37.5)
                    ]
                cache = (key, tuple(position.round(7)), tuple(axis.round(8)))
                if cache not in memo:
                    memo[cache] = [possible(vs, mat, o, position, axis) for o, vs in origins[key]]
                if any(n != 0 for n in memo[cache]):
                    unresolved += 1
                count += int(row == previous[tick])
            out["captures"][tag] = {
                "frames": len(view),
                "finite": finite,
                "outsideModelTruthMm": outside,
                "sameIntervalAndStatus": count,
                "wholePipeConeUnresolved": unresolved,
                "csvSha256": digest(filename),
                "statuses": report["statuses"],
            }
            all_views[head] = view
        key_times = sorted(all_views["fixed"])[2:6]
        joined = []
        for t in key_times:
            fixed = all_views["fixed"][t]
            moving = all_views["moving"][t]
            if (
                len(fixed) == len(moving) == 7
                and moving[0] == "inside"
                and all(
                    max(fixed[i], moving[i]) <= min(fixed[i + 1], moving[i + 1]) for i in (1, 3, 5)
                )
            ):
                joined.append(t)
        last = 26707 if piece == "coral-A" else 26401
        out["joined"][piece] = {
            "overlapMs": joined,
            "wholeFrontTimeMs": last,
            "wholeFrontBounds": all_views["fixed"][last],
        }
    controls = {}
    for fault in ("lost-eye", "swapped-eyes"):
        tag = "coral-B-moving-barrel-" + fault + "-critical"
        rows = table(args.images, tag)
        meta = read(args.images / (tag + ".json"))
        controls[fault] = {
            "captures": len(rows),
            "finite": sum(len(row) == 7 for row in rows.values()),
            "statuses": meta["statuses"],
            "csvSha256": digest(args.images / (tag + ".csv")),
        }
    out["adverseControls"] = controls
    write(args.out, out)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("bounds", "raw", "source", "images", "out"):
        p.add_argument(k, type=Path)
    run(p.parse_args())
