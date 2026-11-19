"""Recover actual four-eye station evidence under bounded lighting and deliberate faults."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from calibrators import LO, HI, witness, select_views
from radiometry import encode, samples, normalize
from model_arc import fit as fit_rims


def run_view(
    state, sign, views, surveyed, wheel, light, restore=True, challenge=None, rims=False
):
    fixed, (actual, ref), _ = views
    q, v = state
    if challenge == "empty":
        q = None
    elif challenge == "off-center":
        q = q + [0, 18, 0]
    elif challenge == "rotated":
        angle = math.radians(7)
        v = np.array([math.sin(angle), math.cos(angle), 0])
    if q is not None:
        extent = witness.sensor.camera.HALF * np.abs(
            v
        ) + witness.sensor.camera.R * np.sqrt(np.maximum(0, 1 - v * v))
        if np.any(q - extent < LO) or np.any(q + extent > HI):
            raise ValueError(
                "The measured trial leaves the calibration exclusion volume"
            )
        witness.sensor.scenes.fixed_pose(witness.O)
        f = witness.silhouette(
            fixed, q, v, intensity=165 if challenge == "pipe-shadow" else 220
        )
        witness.sensor.optics.configure()
        r = witness.sensor.silhouette(
            actual, q, v, intensity=165 if challenge == "pipe-shadow" else 220
        )
        if challenge in ("flat-roof", "flat-right"):
            # Deliberately present a bright rectangular stand-in to the roof while
            # the independent fixed camera still sees the real cylinder.
            for eye in [0, 1] if challenge == "flat-roof" else [1]:
                empty = actual[eye][0]
                occupied = (r[eye] != empty) & (r[eye] == 220)
                yy, xx = np.nonzero(occupied)
                r[eye] = empty.copy()
                if len(xx):
                    r[eye][yy.min() : yy.max() + 1, xx.min() : xx.max() + 1] = 220
    else:
        f = [g for g, _ in fixed]
        r = [g for g, _ in actual]
    ref_f = [g for g, _ in fixed]

    gain, offset = light
    settings = [(gain, offset)] * 4
    if challenge == "pupil-gains":
        settings = [(0.72, 4), (1.12, 6), (0.68, 8), (0.94, 1)]
    incoming = [encode(im, g, b) for im, (g, b) in zip(f + r, settings)]
    if challenge == "left-drop":
        incoming[0] = np.zeros_like(incoming[0])
    if challenge == "uneven-reference":
        yy, xx = np.indices(incoming[0].shape)
        crossed = (yy // 32 + xx // 32) % 2 == 1
        incoming[0] = np.where(crossed, encode(incoming[0], 1, -25), incoming[0])
    if challenge == "saturate":
        incoming = [np.full_like(im, 255) for im in incoming]
    decoded = []
    checks = []
    for g, groups in zip(incoming, surveyed):
        if restore:
            g, status = normalize(g, groups)
            checks.append(status)
            if g is None:
                return {
                    "accepted": False,
                    "why": "unregistered-exposure",
                    "calibration": checks,
                }
        decoded.append(g)
    rendered_sha = hashlib.sha256(b"".join(im.tobytes() for im in incoming)).hexdigest()
    f, rf = witness.mask_fixed_contour(decoded[:2], ref_f, wheel["fixed"], False)
    witness.sensor.scenes.fixed_pose(witness.O)
    pos = witness.read_pair(f, rf, 560, 0)
    witness.sensor.optics.configure()
    r = [np.where(m, 32, im) for m, im in zip(wheel["roof"], decoded[2:])]
    reference = [
        (np.where(m, 32, g), np.where(m, 3500.0, z))
        for m, (g, z) in zip(wheel["roof"], ref)
    ]
    roof = witness.measure(r, reference, peak_bound=witness.peak)
    witness.bridge_fit._registered.pop(tuple(id(g) for g, _ in reference), None)
    yaw = roof.get("independentYawIntervalRad")
    rim_result = (
        fit_rims(r, reference, witness.sensor.optics, pos, yaw) if rims else None
    )
    accepted = (
        pos["status"] == "position-witness"
        and (
            rim_result["status"] == "two-projected-rims"
            if rims
            else roof["status"] == "inside"
        )
        and yaw is not None
        and max(map(abs, yaw)) <= math.radians(2)
    )
    if accepted:
        truth = [q[0], q[1], q[2] - 557.15] if q is not None else None
        if truth is None or not (
            witness.enclose(
                pos,
                ("u_interval_mm", "y_interval_mm", "bottom_relative_to_shelf_mm"),
                truth,
            )
            and (rims or witness.enclose(roof, ("x_mm", "y_mm", "bottom_mm"), truth))
            and yaw[0] <= math.atan2(v[0], v[1]) <= yaw[1]
        ):
            raise ValueError("Granted invalid physical interval")
    return {
        "accepted": bool(accepted),
        "fixed": pos["status"],
        "roof": roof["status"],
        "rimCheck": rim_result,
        "registered": roof["registered"],
        "yawIntervalRad": yaw,
        "imageSha256": rendered_sha,
        "calibration": checks,
    }


def trial(feed, raw_dir, cache, wheel_path, masks, *, full=True, rims=False):
    sign = feed["case"]
    frames = witness.frames(feed, raw_dir)
    wheel = witness.load_wheel_mask(wheel_path)
    results = {}
    cases = [
        ("nominal", 1, 0, True, None),
        ("dim", 0.72, 4, True, None),
        ("bright", 1.12, 6, True, None),
        ("raw-dim", 0.72, 4, False, None),
        ("mixed-pupils", 1, 0, True, "pupil-gains"),
        ("matched", 1, 0, True, "matched"),
    ]
    if full:
        cases += [
            (challenge, 1, 0, True, challenge)
            for challenge in (
                "empty",
                "off-center",
                "pipe-shadow",
                "saturate",
                "left-drop",
                "uneven-reference",
                "opaque-guard",
                "flat-roof",
                "flat-right",
                "rotated",
            )
        ]
    for name, gain, offset, restore, challenge in cases:
        views = select_views(
            cache / (sign + ".npz"),
            220 if challenge == "matched" else 80,
            blocked=challenge == "opaque-guard",
        )
        fixed, (actual, reference), _ = views
        refs = [g for g, _ in fixed] + [g for g, _ in reference]
        grades = [
            samples(empty, masks[f"{sign}-{head}{eye}"])
            for empty, (head, eye) in zip(
                refs, [("fixed", 0), ("fixed", 1), ("roof", 0), ("roof", 1)]
            )
        ]
        outcomes = [
            {
                "captureJavaMs": stamp,
                **run_view(
                    (q, v),
                    sign,
                    views,
                    grades,
                    wheel,
                    (gain, offset),
                    restore,
                    challenge,
                    rims,
                ),
            }
            for stamp, q, v in frames
        ]
        results[name] = {
            "gate": witness.service_gate(outcomes, feed, 15),
            "late": witness.service_gate(outcomes, feed, 45),
            "frames": outcomes,
        }
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for arg in ("feed", "raw", "cache", "wheel", "masks", "output"):
        p.add_argument(arg, type=Path)
    p.add_argument("--short", action="store_true")
    a = p.parse_args()
    original = witness.read(a.feed)
    f = original.get("feed") or original["attempts"][0]
    with np.load(a.masks) as masks:
        results = trial(f, a.raw, a.cache, a.wheel, masks, full=not a.short)
    a.output.write_text(
        json.dumps(results, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
    print(
        {
            k: (
                v["gate"]["accepted"],
                v["frames"][0].get("why"),
                v["frames"][0].get("roof"),
            )
            for k, v in results.items()
        }
    )
