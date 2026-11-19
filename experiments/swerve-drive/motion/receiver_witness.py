"""Replay independent optical custody on the registered R4 body and attained swerve feeds."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]
LEGACY = HERE.parents[1] / "six-wheel"
sys.path.insert(0, str(LEGACY / "straight-guide-match"))
from camera import sensor, measure, peak
from controls import bridge_fit
from station import field_mesh, read_pair, silhouette, O
from station_scene import scene
from station_clock import service_gate
from wheel_visibility import load_wheel_mask, mask_fixed_contour

sensor.scenes.camera = sensor.camera


def read(path):
    return json.loads(path.read_bytes())


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def source_parts():
    model = PROJECT / "cad/full-swerve-r4"
    receipt = read(PROJECT / "docs/robot/cad/receipt.json")
    manifest = read(model / "appearances.json")
    if not (
        digest(model / "full-robot.step")
        == manifest["sourceSha256"]
        == receipt["sourceSha256"]
    ):
        raise ValueError(
            "Selected R4 assembly differs from its public rendering record"
        )
    rows = manifest["solids"]
    bodies = sensor.mount.cad.read_named(model / "full-robot.step", rows)
    colors = {
        group["role"]: group["color"] for group in receipt["appearances"]["groups"]
    }
    parts = []
    for row in rows:
        points, faces = sensor.mount.original.triangulate(bodies[row["id"]])
        parts.append((row["id"], row["role"], colors[row["role"]], points, faces))
    return parts


def render(parts, head):
    if head == "fixed":
        sensor.scenes.fixed_pose(O)
    else:
        reg = read(LEGACY / "selected-optics/mount.json")["registrations"]["roof"]
        sensor.optics.ORIGIN = np.asarray(reg["assumedRectifiedOriginMm"])
        sensor.optics.configure()
    shell = ("moving" if head == "roof" else "fixed") + "-camera-public-envelope"
    if sum(part[0] == shell for part in parts) != 1:
        raise ValueError("Missing the registered emitting camera")
    return sensor.camera.images_for_pose(
        {"parts": [part for part in parts if part[0] != shell]}
    )


def frames(feed, source_dir):
    paths = list(
        (source_dir / feed["case"]).glob(f"*folded-{feed['plantStepUs']:g}us*/*.npy")
    )
    (path,) = [
        p for p in paths if p.name != "guide.npy" and p.name.startswith(p.parent.name)
    ]
    if digest(path) != feed["pipeTrajectorySha256"]:
        raise ValueError("Pipe trajectory differs from the selected station feed")
    trace = np.load(path, mmap_mode="r")
    result = []
    for stamp in (5372, 5389, 5406, 5423, 5440, 5457):
        i = int(np.abs(trace[:, 0] - stamp).argmin())
        if abs(trace[i, 0] - stamp) > 0.001:
            raise ValueError("Missing exposure time")
        result.append((stamp, trace[i, 1:4], trace[i, 4:7]))
    return result


def enclose(fit, keys, truth):
    return all(
        k in fit and fit[k][0] <= val <= fit[k][1] for k, val in zip(keys, truth)
    )


def observe(fixed, roof, center, axis, fault=None, wheel_mask=None):
    sensor.scenes.fixed_pose(O)
    empty = [gray for gray, _ in fixed]
    fixed_images = empty if center is None else silhouette(fixed, center, axis)
    if fault == "fixed-left-drop":
        fixed_images[0] = np.zeros_like(empty[0])
    if wheel_mask is not None:
        fixed_images, empty = mask_fixed_contour(
            fixed_images,
            empty,
            wheel_mask["fixed"],
            wheel_mask.get("wholeScanlines", False),
        )
    position = read_pair(fixed_images, empty, 560, 0)
    sensor.optics.configure()
    actual, reference = roof
    roof_images = (
        [g for g, _ in actual]
        if center is None
        else sensor.silhouette(actual, center, axis)
    )
    if fault == "roof-right-drop":
        roof_images[1] = np.zeros_like(actual[1][0])
    if wheel_mask is not None:
        # Uncertain received pixels cannot establish either a contour or a reference.
        roof_images = [
            np.where(mask, 32, im) for mask, im in zip(wheel_mask["roof"], roof_images)
        ]
        reference = [
            (np.where(mask, 32, gr), np.where(mask, 3500.0, depth))
            for mask, (gr, depth) in zip(wheel_mask["roof"], reference)
        ]
    fit = measure(roof_images, reference, peak_bound=peak)
    # Pose-specific calibration images are discarded after this measurement.
    # The historical registration cache otherwise keeps every full stereo pair.
    bridge_fit._registered.pop(tuple(id(gray) for gray, _ in reference), None)
    angle = fit.get("independentYawIntervalRad")
    accepted = (
        position["status"] == "position-witness"
        and fit["status"] == "inside"
        and angle is not None
        and max(abs(v) for v in angle) <= math.radians(2)
    )
    if accepted:
        if center is None:
            raise ValueError("Empty shelf accepted")
        truth = [center[0], center[1], center[2] - 557.15]
        if not (
            enclose(
                position,
                ("u_interval_mm", "y_interval_mm", "bottom_relative_to_shelf_mm"),
                truth,
            )
            and enclose(fit, ("x_mm", "y_mm", "bottom_mm"), truth)
            and angle[0] <= math.atan2(axis[0], axis[1]) <= angle[1]
        ):
            raise ValueError("Estimator accepted an interval that excludes the pipe")
    fixed_box = [
        position.get(k)
        for k in ("u_interval_mm", "y_interval_mm", "bottom_relative_to_shelf_mm")
    ]
    roof_box = [fit.get(k) for k in ("x_mm", "y_mm", "bottom_mm")]
    image_hash = hashlib.sha256(
        b"".join(im.tobytes() for im in [*fixed_images, *roof_images])
    ).hexdigest()
    return {
        "fixedBoxMm": fixed_box,
        "roofBoxMm": roof_box,
        "fourFramesSha256": image_hash,
        "fixed": position["status"],
        "roof": fit["status"],
        "reference": fit["registered"],
        "accepted": bool(accepted),
        "yawIntervalRad": angle,
    }


def run(
    source_dir,
    field,
    opening=None,
    feeds=None,
    wheel_mask_path=None,
    whole_scanlines=False,
):
    wheel_mask = None if wheel_mask_path is None else load_wheel_mask(wheel_mask_path)
    if wheel_mask is not None:
        wheel_mask["wholeScanlines"] = whole_scanlines
    opening = opening or HERE / "opening.json"
    feeds = feeds or HERE / "first-feed.json"
    field_hash = read(LEGACY / "full-match/installed-stock.json")["field_step_sha256"]
    if digest(field) != field_hash:
        raise ValueError("Unexpected field source")
    parts = source_parts()
    roof_robot = render(parts, "roof")
    fixed_robot = render(parts, "fixed")
    field_parts = field_mesh(field)
    results = []
    controls = []
    withheld = []
    openings = read(opening)["cases"]
    for feed in read(feeds)["attempts"]:
        run = openings[feed["case"]]
        if (
            run["csvSha256"] != feed["driveTraceSha256"]
            or run["actualTerminalMmMmDeg"] != feed["trueChassisAtFeedMmMmDeg"]
        ):
            raise ValueError("Station feed and wheel terminal are from different runs")
        if feed["state"] != "IN_BAND" or feed["javaLastMs"] != 5460:
            withheld.append(
                dict(
                    case=feed["case"],
                    stepUs=feed["plantStepUs"],
                    piece=feed["physicalPieceId"],
                    terminalStateSha256=feed["pipeStateSha256"],
                    state=feed["state"],
                    lastJavaMs=feed["javaLastMs"],
                    accepted=False,
                )
            )
            continue
        x, y, heading = feed["trueChassisAtFeedMmMmDeg"]
        # Station offsets are inward-normal/left; the docked robot faces the opposite direction.
        # The native-field projector subtracts its supplied offset and keeps signed body yaw.
        station_pose = [-x, -y, heading]
        sensor.scenes.fixed_pose(O)
        fixed = scene(field_parts, station_pose, fixed_robot)
        sensor.optics.configure()
        roof_actual = scene(field_parts, station_pose, roof_robot)
        # Field pixels outside the robot silhouette cannot serve as surveyed roof backing.
        ref = [
            (g, np.where(rz <= z + 0.01, rz, 3500.0))
            for (g, z), (_, rz) in zip(roof_actual, roof_robot)
        ]
        states = frames(feed, source_dir)
        modes = {"nominal": (roof_actual, ref)}
        for name, matched in [("guarded", False), ("matched", True)]:
            modes[name] = sensor.shelf_guard(
                roof_actual, ref, sensor.optics.ORIGIN[2], matched, 0.05
            )
        for mode, roof in modes.items():
            outcomes = []
            for stamp, center, axis in states:
                obs = observe(fixed, roof, center, axis, wheel_mask=wheel_mask)
                outcomes.append(
                    dict(
                        captureJavaMs=stamp,
                        trueCenterMm=center.tolist(),
                        trueAxis=axis.tolist(),
                        **obs,
                    )
                )
            results.append(
                dict(
                    case=feed["case"],
                    stepUs=feed["plantStepUs"],
                    finish=mode,
                    piece=feed["physicalPieceId"],
                    trace=feed["pipeTrajectorySha256"],
                    terminalStateSha256=feed["pipeStateSha256"],
                    dockMmDeg=[x, y, heading],
                    gate=service_gate(outcomes, feed, 15),
                    late=service_gate(outcomes, feed, 45),
                    frames=outcomes,
                )
            )
        _, center, axis = states[-2]
        for name, q, v in [
            ("empty", None, None),
            ("off-center", center + [0, 18, 0], axis),
            (
                "yaw-7-deg",
                center,
                np.array([math.sin(math.radians(7)), math.cos(math.radians(7)), 0]),
            ),
            ("fixed-left-drop", center, axis),
            ("roof-right-drop", center, axis),
        ]:
            outcome = observe(fixed, modes["guarded"], q, v, name, wheel_mask)
            if outcome["accepted"]:
                raise ValueError("A failed challenge was granted: " + name)
            controls.append(
                dict(
                    case=feed["case"],
                    stepUs=feed["plantStepUs"],
                    challenge=name,
                    **outcome,
                )
            )
    return dict(
        schema="reefscape-r4-station-custody/1",
        fieldSha256=field_hash,
        stepSha256=digest(PROJECT / "cad/full-swerve-r4/full-robot.step"),
        firstFeedSha256=digest(feeds),
        appearanceRoleMapSha256=digest(PROJECT / "cad/full-swerve-r4/appearances.json"),
        colorGroups=read(PROJECT / "docs/robot/cad/receipt.json")["appearances"][
            "paletteSha256"
        ],
        cameraMountSha256=digest(LEGACY / "selected-optics/mount.json"),
        cases=results,
        controls=controls,
        **(
            {"wheelMaskRegisterSha256": digest(wheel_mask_path)}
            if wheel_mask_path
            else {}
        ),
        **({"withheld": withheld} if withheld else {}),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("feeds", type=Path)
    parser.add_argument("field", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--opening", type=Path)
    parser.add_argument("--feed-register", type=Path)
    parser.add_argument("--wheel-mask", type=Path)
    parser.add_argument(
        "--whole-scanlines",
        action="store_true",
        help="Conservative entire-row control for the fixed stereo",
    )
    args = parser.parse_args()
    args.output.write_text(
        json.dumps(
            run(
                args.feeds,
                args.field,
                args.opening,
                args.feed_register,
                args.wheel_mask,
                args.whole_scanlines,
            ),
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf8",
        newline="\n",
    )
