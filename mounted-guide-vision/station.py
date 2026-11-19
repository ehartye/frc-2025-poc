"""Project the fitted guide kit and two registered signed station feeds into both cameras."""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "elastic-carry"))
import carry_images as sensor
from custody import field_mesh, scene, read_pair, silhouette
from paths import O

sys.path.insert(0, str(ROOT / "observed-reef-a"))
from roof_trim import yaw_interval
from exposures_check import bridge_fit
import guarded_contour

CANONICAL_PACKETS = ROOT / "guide-impact/station-frames.json"
PACKET_SHA = "29b1631ee94959c01e273e30cd257d26f0c716feabe0489f296984998f0b064f"
INSTALLATIONS = {"nominal": 0.0, "outward": 0.315}


def station_packets(path):
    if digest(path) != PACKET_SHA:
        raise ValueError("Changed registered station exposure packet")
    return read(path)["packets"]


def read(p):
    return json.loads(p.read_bytes())


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def mesh(installation, setting=None):
    fit = read(ROOT / "guide-mounts/fit.json")
    datum = read(ROOT / "guide-mounts/datum.json")
    step = ROOT / "guide-mounts/guide-mounts.step"
    if digest(step) != fit["stepSha256"] or datum["stepSha256"] != fit["stepSha256"]:
        raise ValueError("Guide kit exchange changed")
    solids = sensor.mount.cad.read_named(step, [{"id": k} for k in fit["volumesMm3"]])
    setting_sha = None
    if setting is not None:
        revised = read(setting / "fit.json")
        setting_sha = digest(setting / "setting.step")
        if (
            setting_sha != revised["stepSha256"]
            or revised["guideSha256"] != fit["stepSha256"]
        ):
            raise ValueError("Changed setup-only repair")
        candidate = sensor.mount.cad.read_named(
            setting / "setting.step", [{"id": k} for k in revised["volumesMm3"]]
        )
        if set(revised["changedInstalled"]) & set(revised["pitOnly"]):
            raise ValueError("Pit tools enter match")
        for key in revised["changedInstalled"]:
            if key not in solids:
                raise ValueError("Unregistered replacement")
            solids[key] = candidate[key]
    if len(solids) != 58:
        raise ValueError("Incomplete guide inventory")
    offsets = datum["outboardOffsetsMm"]
    parts = []
    for key, shape in solids.items():
        if key.endswith("-machined-mast"):
            # The source robot already contains the opaque mast. Leaving its old bore material
            # in the depth map is conservative for line-of-sight to the field or the pipe.
            continue
        label = (
            "sleeve"
            if key.endswith("-sleeve")
            else "bolt"
            if "-root-" in key
            else "guide"
        )
        shift = offsets[label] if installation == "outward" else 0.0
        sign = 1 if key.startswith("guide-L-") else -1
        # The archived mesh encoder uses ten units/mm. Translation is applied in native
        # millimetres before tessellation to retain exactly the datum study's transform.
        if shift:
            shape = sensor.mount.cad.translated(shape, (0, sign * shift, 0))
        v, t = sensor.mount.original.triangulate(shape)
        parts.append([key, "guide", "#a9a49c", v, t])
    return {"parts": parts}, fit["stepSha256"], setting_sha


def views(assets, installation, setting=None, *, write_labels=True):
    kit, sha, setting_sha = mesh(installation, setting)
    station_base = assets / "observed-reef-a/machined-fixed.npz"
    if (
        digest(station_base)
        != read(ROOT / "observed-reef-a/station-bridge.json")["leftSmall"][
            "machinedFieldBackgroundSha256"
        ]
    ):
        raise ValueError("Changed painted station source or fixed harness")
    sensor.scenes.camera = sensor.camera
    sensor.scenes.fixed_pose(O)
    baseline = sensor.background(station_base)
    layer = sensor.camera.images_for_pose(kit)
    coded = []
    for index, (key, group, color, vertices, triangles) in enumerate(kit["parts"]):
        # Undiluted per-cell gray is 40 plus half the source gray, without lighting.
        shade = f"{2 * (index + 1):02x}" * 3
        coded.append([key, group, "#" + shade, vertices, triangles])
    masks = sensor.camera.images_for_pose({"parts": coded})
    key = ("set-" if setting else "") + installation
    labels = assets / "mounted-guide-vision" / (key + "-labels.npz")
    if write_labels:
        np.savez_compressed(
            labels,
            left=masks[0][0],
            left_z=masks[0][1],
            right=masks[1][0],
            right_z=masks[1][1],
        )
    fixed = sensor.composite(baseline, layer)
    differences = [
        int(np.count_nonzero(gz < z)) for (_, z), (_, gz) in zip(baseline, layer)
    ]
    reg = read(ROOT / "selected-optics/mount.json")["registrations"]["roof"]
    sensor.optics.ORIGIN = np.array(reg["assumedRectifiedOriginMm"])
    sensor.optics.configure()
    folder = assets / "elastic-carry/marked"
    marks = read(folder / "scenes.json")
    baseline = sensor.available(folder, "barrel-moving-0", marks)
    prior = sensor.available(folder, "barrel-moving-reference", marks)
    layer = sensor.camera.images_for_pose(kit)
    moving = sensor.composite(baseline, layer)
    ref = [
        (g, np.minimum(z, gz)) for (g, _), (_, z), (_, gz) in zip(moving, prior, layer)
    ]
    moving_differences = [
        int(np.count_nonzero(gz < z)) for (_, z), (_, gz) in zip(baseline, layer)
    ]
    roof = {
        "nominal": (moving, ref),
        "guarded": sensor.shelf_guard(
            moving, ref, sensor.optics.ORIGIN[2], False, 0.05
        ),
        "matched-finish": sensor.shelf_guard(
            moving, ref, sensor.optics.ORIGIN[2], True, 0.05
        ),
    }
    return (
        fixed,
        roof,
        {
            "guideStepSha256": sha,
            "settingStepSha256": setting_sha,
            "reopenedParts": 58,
            "addedToRaster": 56,
            "oldOpaqueMastRetained": 2,
            "bodyLabelSha256": digest(labels),
            "bodyPerGray": {
                str(i + 41): body[0] for i, body in enumerate(kit["parts"])
            },
            "newForegroundPixels": {"fixed": differences, "roof": moving_differences},
            "stationBaseSha256": digest(station_base),
            "roofSourceSha256": digest(folder / "scenes.json"),
        },
    )


def run(assets, field, setting=None, packet_path=CANONICAL_PACKETS):
    field_sha = read(ROOT / "full-match/installed-stock.json")["field_step_sha256"]
    if digest(field) != field_sha:
        raise ValueError("Different field")
    field_shape = field_mesh(field)
    arrivals = read(ROOT / "conditioned-match/opening.json")
    folder = assets / "mounted-guide-vision"
    folder.mkdir(exist_ok=True)
    out = {}
    signed = station_packets(packet_path)
    for installation, fit_shift in INSTALLATIONS.items():
        packets = [case for case in signed if case["outwardPerSideMm"] == fit_shift]
        fixed, roof, register = views(assets, installation, setting)
        key = ("set-" if setting else "") + installation
        target = folder / (key + "-fixed.npz")
        np.savez_compressed(
            target,
            left=fixed[0][0],
            left_z=fixed[0][1],
            right=fixed[1][0],
            right_z=fixed[1][1],
        )
        register.update(
            fixedBackgroundSha256=digest(target),
            sourcePacketSha256=PACKET_SHA,
            fieldSha256=field_sha,
        )
        cases = []
        for packet in packets:
            side = packet["case"]
            if packet["outwardPerSideMm"] != (
                0.315 if installation == "outward" else 0
            ):
                raise ValueError("Wrong kit datum")
            sensor.scenes.fixed_pose(O)
            background = scene(field_shape, arrivals[side]["sourceRobotMmDeg"], fixed)
            empty = [g for g, _ in background]
            stationary = []
            moving = []
            partial = []
            sparse = []
            for frame in packet["frames"]:
                stamp = frame["javaMs"]
                if stamp not in (5372, 5389, 5406, 5423, 5440, 5457):
                    continue
                center = np.asarray(frame["centerMm"])
                axis = np.asarray(frame["axis"])
                axis /= np.linalg.norm(axis)
                actual = (center[0], center[1], center[2] - 557.15)
                if stamp < 5457:
                    im = silhouette(background, center, axis)
                    fit = read_pair(im, empty, 560, 0)
                    bounded = all(
                        name in fit and fit[name][0] <= v <= fit[name][1]
                        for name, v in zip(
                            (
                                "u_interval_mm",
                                "y_interval_mm",
                                "bottom_relative_to_shelf_mm",
                            ),
                            actual,
                        )
                    )
                    stationary.append([stamp, fit["status"], bounded])
                sensor.optics.configure()
                for mode, (bg, ref) in roof.items():
                    im = sensor.silhouette(bg, center, axis)
                    fit = sensor.fit_row(im, ref, True, 0)
                    registration = bridge_fit.registration_scores(im, ref)
                    clean = [
                        np.where(
                            (g >= 185) & (g.astype(int) - gray.astype(int) > 36),
                            g,
                            gray,
                        )
                        for g, (gray, _) in zip(im, ref)
                    ]
                    alt = (
                        bridge_fit.partial_fit(clean, ref)
                        if min(registration) >= 0.90
                        else {"status": "robot-reference-lost"}
                    )
                    intervals = [alt.get(k) for k in ("x_mm", "y_mm", "bottom_mm")]
                    enclosed = all(
                        v is not None and v[0] <= value <= v[1]
                        for v, value in zip(intervals, actual)
                    )
                    partial.append(
                        [
                            stamp,
                            mode,
                            alt["status"],
                            enclosed,
                            intervals,
                            min(registration),
                        ]
                    )
                    candidate = (
                        guarded_contour.partial_fit(clean, ref)
                        if min(registration) >= 0.90
                        else {"status": "robot-reference-lost"}
                    )
                    box = [candidate.get(k) for k in ("x_mm", "y_mm", "bottom_mm")]
                    angle = yaw_interval(im, ref)
                    sparse.append(
                        [
                            stamp,
                            mode,
                            candidate["status"],
                            all(
                                v is not None and v[0] <= value <= v[1]
                                for v, value in zip(box, actual)
                            ),
                            box,
                            list(angle) if angle is not None else None,
                        ]
                    )
                    yaw = np.arctan2(axis[0], axis[1])
                    moving.append(
                        [
                            stamp,
                            mode,
                            fit[0],
                            sensor.outcome(fit, actual),
                            bool(angle is not None and angle[0] <= yaw <= angle[1]),
                        ]
                    )
                sensor.scenes.fixed_pose(O)
            cases.append(
                {
                    "wheelCase": side,
                    "source": packet["id"],
                    "pipeSha256": packet["trajectorySha256"],
                    "modalStateSha256": packet["modeStateSha256"],
                    "controllerSha256": packet.get("programSha256"),
                    "bearingSupport": packet["carriageSupport"],
                    "packetSha256": PACKET_SHA,
                    "packetFile": "guide-impact/station-frames.json",
                    "maxFreeGuideDeflectionMm": max(
                        max(f["allFaceModeledAbsUpperMm"]) for f in packet["frames"]
                    ),
                    "fixed": stationary,
                    "roof": moving,
                    "partialContour": partial,
                    "boundedGaps": sparse,
                }
            )
        out[key] = {"register": register, "cases": cases}
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    p.add_argument("field", type=Path)
    p.add_argument("--setting", type=Path)
    p.add_argument("--packets", type=Path, default=CANONICAL_PACKETS)
    a = p.parse_args()
    (HERE / ("setting-station.json" if a.setting else "station.json")).write_text(
        json.dumps(run(a.assets, a.field, a.setting, a.packets), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
