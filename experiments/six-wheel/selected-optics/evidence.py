"""Compile compact receipts from native backgrounds and complete timestamped optical observations."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from exposures_check import HERE, ROOT, MATCH, frames, exposures, sha, read
from sight import bridge_fit, background


def result(folder, name):
    q = read(folder / (name + ".json"))
    if sha(folder / (name + ".csv")) != q["frameCsvSha256"]:
        raise ValueError("Edited exposure CSV")
    return q


def table(folder, name):
    with (folder / (name + ".csv")).open() as f:
        return {
            int(row[2]): [row[3], *map(float, row[4:])]
            for row in list(csv.reader(f))[1:]
        }


def survey_edges(folder):
    bg = background(folder / "barrel-moving-swept.npz")
    images = [g for g, _ in bg]
    values = bridge_fit.registration_scores(images, bg)
    refs = bridge_fit._registered[tuple(id(g) for g, _ in bg)][1]
    stable = background(folder / "barrel-moving-reference.npz")
    contour_reference = stable
    bridge_fit.registration_scores(images, contour_reference)
    other = bridge_fit._registered[tuple(id(g) for g, _ in contour_reference)][1]
    equal = all(
        ia == ib
        and all(
            a[0] == b[0] and all(np.array_equal(x, y) for x, y in zip(a[1:], b[1:]))
            for a, b in zip(ga, gb)
        )
        for (ia, ga), (ib, gb) in zip(refs, other)
    )
    return {
        "sameWitnessCoordinatesAndSigns": equal,
        "scores": values,
        "witnessSamples": [sum(len(axis[1]) for axis in group) for _, group in refs],
    }


def evaluate(args):
    folder = args.issued / "marked"
    scene = read(folder / "scenes.json")
    sweep = read(folder / "sweep.json")
    mark = read(folder / "marks.json")
    sources = {
        **scene["sourceSha256"],
        "packet-recovery/events.csv": sha(ROOT / "packet-recovery/events.csv"),
        "relieved-match/controller-register.json": sha(
            MATCH / "controller-register.json"
        ),
    }
    views = {}
    joined = {}
    carry = {}
    clock = {}
    for piece in ("coral-A", "coral-B"):
        label, src, edge, raw, pick, target = frames(piece, args.original)
        clock[piece] = read(MATCH / (label + "-feed.json"))["epochWallUs"]
        clips = {}
        for head, suffix in (
            ("moving", "all-bounded"),
            ("fixed", "critical"),
            ("seat", "all"),
        ):
            name = f"{piece}-{head}-barrel-nominal-{suffix}"
            summary = result(folder, name)
            capture = table(folder, name)
            clips[head] = capture
            outside = 0.0
            finite = 0
            for t, fit in capture.items():
                if len(fit) != 7:
                    continue
                if head == "seat":
                    xyz = [np.interp(t, pick[:, 0], col) for col in pick[:, 1:4].T]
                else:
                    xyz, _, _ = exposures.state_at(
                        piece, t, raw, src, edge, pick, target
                    )
                real = [xyz[0], xyz[1], xyz[2] - 557.15]
                outside = max(
                    outside,
                    *[
                        max(fit[i + 1] - v, v - fit[i + 2])
                        for i, v in zip((0, 2, 4), real)
                    ],
                )
                finite += 1
            views[name] = {
                "frames": len(capture),
                "statuses": summary["statuses"],
                "finite": finite,
                "maxTruthOutsideMm": float(outside),
                "captureCsvSha256": summary["frameCsvSha256"],
            }
            if head == "moving":
                counts = summary["uncertainBrightPixelCounts"]
                carry[piece] = {
                    "uniqueBetweenEndpointStates": len(counts),
                    "statesWithAmbiguousPixels": sum(
                        sum(v) > 0 for v in counts.values()
                    ),
                    "maxAmbiguousPixels": max(sum(v) for v in counts.values()),
                    "allBinaryCombinationsChecked": sum(
                        1 << sum(v) for v in counts.values()
                    ),
                    "ambiguousUnresolvedFrames": summary["statuses"].get(
                        "unbounded-occlusion", 0
                    ),
                }
        key_times = sorted(clips["fixed"])[2:6]

        def overlap(a, b):
            return len(a) == len(b) == 7 and all(
                max(a[i], b[i]) <= min(a[i + 1], b[i + 1]) for i in (1, 3, 5)
            )

        point = 26707 if piece == "coral-A" else 26401
        accepted = [
            t
            for t in key_times
            if clips["moving"][t][0] == "inside"
            and overlap(clips["moving"][t], clips["fixed"][t])
        ]
        joined[piece] = {
            "joinedCapturesMs": accepted,
            "bothHeads": {
                t: {"roof": clips["moving"][t], "fixed": clips["fixed"][t]}
                for t in key_times
            },
            "nextRoof": clips["moving"][key_times[-1] + 17],
            "wholeFrontCaptureMs": point,
            "fixedWholeFront": clips["fixed"][point],
        }

    def b(key):
        return result(folder, "coral-B-moving-" + key + "-critical")["selectedFrames"][
            "26231"
        ]

    adverse = {
        "staleRoofTruthOutsideAt26231Mm": b("barrel-stale-roof")["outsideMm"],
        "staleReportedStatus": b("barrel-stale-roof")["bounds"][0],
        "lostEyeSuppliedBounds": len(b("barrel-lost-eye")["bounds"]) == 7,
        "swappedEyesSuppliedBounds": len(b("barrel-swapped-eyes")["bounds"]) == 7,
        "unmarkedRegistration": survey_edges(args.issued),
        "markedRegistration": survey_edges(folder),
    }
    variants = {"roof": {k: b(k + "-nominal") for k in ("front", "rear")}}
    for key in ("front", "rear"):
        variant = args.issued / ("fixed-" + key)
        cases = {}
        for piece in ("coral-A", "coral-B"):
            for head in ("fixed", "seat"):
                name = f"{piece}-{head}-barrel-nominal-critical"
                r = result(variant, name)
                cases[piece + "/" + head] = {
                    "frames": r["frames"],
                    "statuses": r["statuses"],
                    "maximumTruthOutsideSelectedMm": max(
                        (v["outsideMm"] or 0.0 for v in r["selectedFrames"].values()),
                        default=0.0,
                    ),
                    "frameCsvSha256": r["frameCsvSha256"],
                }
        variants["fixed-" + key] = {
            "forwardOffsetMm": read(variant / "scenes.json")["fixedDepthOffsetMm"],
            "cases": cases,
        }
    issued = {
        "sources": sources,
        "selectedScene": scene["hardware"],
        "stationAndFieldTransforms": scene["operatingCases"],
        "localEpochWallUs": clock,
        "markedSceneSha256": sha(folder / "scenes.json"),
        "flowingBackgroundSha256": sweep["nearestPossibleSha256"],
        "opaqueSlideFixedBackingParts": scene["stableRoofBackingParts"],
        "uncertainEnvelopeCount": sweep["dynamicEnvelopes"],
        "translatedNativeBodies": sweep["translatedNativeBodies"],
        "shelfMarkMaterialBacking": mark["areaBackingFraction"],
        "shelfMarkGuardedPixelCounts": mark["renderings"]["barrel-moving-swept"][
            "visibleMarkPixels"
        ],
        "completeViews": views,
        "intermediateStroke": carry,
        "pairedHandoff": joined,
        "adverse": adverse,
        "datumVariants": variants,
        "measuredOpticalAccuracy": None,
    }
    (HERE / "observations.json").write_text(
        json.dumps(issued, indent=2) + "\n", encoding="utf-8", newline="\n"
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("issued", type=Path)
    p.add_argument("original", type=Path)
    evaluate(p.parse_args())
