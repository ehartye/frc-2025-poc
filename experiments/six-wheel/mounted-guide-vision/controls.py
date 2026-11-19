"""Exercise the guarded roof fit with an empty shelf, misplaced pipe and damaged images."""

import argparse
import json
import math
from pathlib import Path
import numpy as np

from station import (
    HERE,
    CANONICAL_PACKETS,
    INSTALLATIONS,
    station_packets,
    sensor,
    views,
    bridge_fit,
    guarded_contour,
    yaw_interval,
)


def measure(images, ref, *, peak_bound=None):
    registration = bridge_fit.registration_scores(images, ref)
    if min(registration) < 0.90:
        return {"status": "robot-reference-lost", "registered": min(registration)}
    clean = [
        np.where((g >= 185) & (g.astype(int) - base.astype(int) > 36), g, base)
        for g, (base, _) in zip(images, ref)
    ]
    fit = guarded_contour.partial_fit(clean, ref, peak_bound=peak_bound)
    fit["registered"] = min(registration)
    fit["independentYawIntervalRad"] = yaw_interval(images, ref)
    return fit


def run(assets, packet_path=CANONICAL_PACKETS, *, peak_bound=None):
    signed = station_packets(packet_path)
    result = []
    for installation, shift in INSTALLATIONS.items():
        fixed, roof, register = views(assets, installation, write_labels=False)
        bg, ref = roof["guarded"]
        for packet in signed:
            if (
                packet["outwardPerSideMm"] != shift
                or packet["stepUs"] != 31.25
                or packet["guideModel"][1] != 1800
                or "serial-carriage" in packet["id"]
            ):
                continue
            state = next(f for f in packet["frames"] if f["javaMs"] == 5440)
            center = np.asarray(state["centerMm"])
            axis = np.asarray(state["axis"])
            trials = [("acquired", center, axis), ("empty", None, None)]
            for dim, offset in ((0, -25), (0, 35), (1, -12), (1, 12), (2, 18)):
                new = center.copy()
                new[dim] += offset
                trials.append((f"axis-{dim}-shift-{offset}", new, axis))
            for yaw in (-6, 6):
                a = math.radians(yaw)
                trials.append(
                    (f"yaw-{yaw}", center, np.array([math.sin(a), math.cos(a), 0.0]))
                )
            trials += [
                ("missing-left-packet", center, axis),
                ("wrong-field-reference", center, axis),
            ]
            for name, q, v in trials:
                images = (
                    [gray.copy() for gray, _ in bg]
                    if q is None
                    else sensor.silhouette(bg, q, v)
                )
                if name == "missing-left-packet":
                    images[0] = np.zeros_like(images[0])
                if name == "wrong-field-reference":
                    images = [np.full_like(x, 220) for x in images]
                fit = measure(images, ref, peak_bound=peak_bound)
                permit = (
                    fit["status"] == "inside"
                    and fit.get("independentYawIntervalRad") is not None
                    and max(abs(bound) for bound in fit["independentYawIntervalRad"])
                    <= math.radians(2)
                )
                truth = None if q is None else [*q.tolist(), *v.tolist()]
                inside = (
                    False
                    if q is None
                    else (
                        55 <= q[0] <= 100
                        and abs(q[1]) <= 8
                        and abs(q[2] - 557.15) <= 12
                        and abs(math.degrees(math.atan2(v[0], v[1]))) <= 2
                    )
                )
                result.append(
                    {
                        "installation": installation,
                        "wheelCase": packet["case"],
                        "challenge": name,
                        "sourcePipeSha256": packet["trajectorySha256"],
                        "centerAndAxis": truth,
                        "insideSourceGate": bool(inside),
                        "status": fit["status"],
                        "admitted": bool(permit),
                        "minReferenceScore": fit["registered"],
                    }
                )
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    p.add_argument("--packets", type=Path, default=CANONICAL_PACKETS)
    a = p.parse_args()
    (HERE / "controls.json").write_text(
        json.dumps(run(a.assets, a.packets), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
