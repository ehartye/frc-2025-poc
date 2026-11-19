"""Render selected robot and field geometry at the two station and placement poses."""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def load_assembly(path):
    spec = importlib.util.spec_from_file_location("selected_shop_scene", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def selected_poses(model, artifacts, lamp):
    stowed, placed = model.poses(artifacts, lamp)
    q = model.ROOT / "selected-match"
    kit = json.loads((q / "kit.json").read_bytes())
    native = q / "receiver-clearance-kit.step"
    if model.digest(native) != kit["kitSha256"]:
        raise ValueError("Selected receiver machining STEP no longer matches its issue")
    joint = json.loads((model.ROOT / "drive-side-attachment/fit.json").read_bytes())
    sys.path.insert(0, str(q))
    from rail_profile import revised

    for stroke, p in ((0, stowed), (370, placed)):
        model.add_exchange(
            p,
            model.ROOT / "drive-side-attachment/inner-rail-joints.step",
            joint["volumeMm3"],
            joint["stepSha256"],
        )
        machined = revised(p, stroke)
        if not set(kit["volumeMm3"]) <= set(machined):
            raise ValueError("Missing manufactured receiver components")
        for k, v in kit["volumeMm3"].items():
            if abs(model.cad.volume(machined[k])[0] - v) > 0.002:
                raise ValueError("Stale receiver volume: " + k)
        p.update(machined)
    return stowed, placed


def combine(a, b):
    return [
        (np.where(z1 < z2, g1, g2), np.minimum(z1, z2))
        for (g1, z1), (g2, z2) in zip(a, b)
    ]


def paint_world(parts):
    robot = []
    for name, shape in sorted(parts.items()):
        v, t = mount.original.triangulate(shape)
        robot.append([name, "shop", colors.get(name, "#737d85"), v, t])
    return robot


def rigid_mesh(components, transform, kind):
    result = []
    for name, vertices, triangles in components:
        local = transform(vertices)
        if kind == "reef" and (local[:, 0].max() < -315 or local[:, 0].min() > 1700):
            continue
        shade = (
            ("#808080" if "/GE-" in name else "#ababab")
            if kind == "station"
            else "#a0a0a0"
        )
        result.append(
            [
                name,
                kind,
                shade,
                np.rint(local * 10).astype(int).ravel().tolist(),
                triangles,
            ]
        )
    return result


def store(folder, name, scene):
    target = folder / (name + ".npz")
    np.savez_compressed(
        target,
        left=scene[0][0],
        left_z=scene[0][1],
        right=scene[1][0],
        right_z=scene[1][1],
    )
    return hashlib.sha256(target.read_bytes()).hexdigest()


def fixed_pose(origin):
    from imaging import stereo

    camera.C, camera.S = stereo.C, stereo.S
    camera.ORIGIN = np.array(origin, float)
    camera.DX = camera.C + camera.Q * camera.S
    camera.DY = -(camera.U - camera.W / 2) / camera.F
    camera.DZ = -camera.S + camera.Q * camera.C


def render(args, select_poses=selected_poses):
    model = load_assembly(args.assembly)
    sys.path.insert(0, str(ROOT / "relieved-match"))
    from cameras import (
        mount as reference_mount,
        optics as roof,
        camera as im,
        receiver_drives,
    )
    from images import silhouette
    from registration import parts as station_parts

    global mount, optics, camera, colors
    mount, optics, camera = reference_mount, roof, im
    from mechanics import source, pose, digest, mapped, vendor, FORWARD

    manifest = json.loads((HERE / "mount.json").read_bytes())
    if digest(HERE / "fixed-datum.step") != manifest["stepSha256"]:
        raise ValueError("Changed locator STEP")
    stowed, placed = select_poses(model, args.artifacts, args.lamp)
    fixed_wire, moving_wire = model.wire_boxes()
    wire = {
        k: model.native_box(v)
        for k, v in {**fixed_wire, **moving_wire}.items()
        if k != "registered-camera-case"
    }
    datum = vendor.cad.read_named(
        HERE / "fixed-datum.step", [{"id": k} for k in manifest["reopenedParts"]]
    )
    raw, eyes, case_box, _ = source(args.artifacts / "camera-disconnects")
    result = {
        "sourceSha256": {},
        "hardware": {},
        "operatingCases": {},
        "backgroundSha256": {},
    }
    for path, key in (
        (args.assembly, "selected-match/clearance.py"),
        (args.lamp, "supplied lamp STEP"),
        (args.field, "registered field STEP"),
        (
            model.ROOT / "selected-match/rail_profile.py",
            "selected-match/rail_profile.py",
        ),
        (
            model.ROOT / "selected-match/receiver-clearance-kit.step",
            "selected-match/receiver-clearance-kit.step",
        ),
        (
            model.ROOT / "drive-side-attachment/inner-rail-joints.step",
            "drive-side-attachment/inner-rail-joints.step",
        ),
        (HERE / "mount.json", "selected-optics/mount.json"),
        (HERE / "fixed-datum.step", "selected-optics/fixed-datum.step"),
        (
            args.artifacts / "camera-datum-shop/place.step",
            "camera-datum-shop/place.step",
        ),
        (args.artifacts / "camera-disconnects/camera.stp", "DM9095 enclosure"),
    ):
        result["sourceSha256"][key] = digest(path)
    if (
        result["sourceSha256"]["registered field STEP"]
        != json.loads((ROOT / "relieved-match/native-register.json").read_bytes())[
            "fieldSha256"
        ]
    ):
        raise ValueError("Unknown field geometry")
    _, groups, colors, _ = mount.pose_layers(560, 0, "red")
    witness = eyes.mean(axis=0)
    transforms = {
        key: pose(key, case_box, witness) for key in ("fixed-datum-up", "roof")
    }
    origins = {key: mapped(t, witness) for key, t in transforms.items()}
    cases = {
        key: {
            f"{key}-vendor-{i}": vendor.transform(v, *transforms[key])
            for i, v in enumerate(raw)
        }
        for key in transforms
    }
    # The emitting case is removed after the local baffle check. The other camera stays as an opaque body.
    orig = stowed.copy()
    for stroke, frame in ((0, stowed), (370, placed)):
        for key in (
            "roof-camera-envelope",
            "observer-camera-proxy",
            "observer-slotted-cradle-weldment",
        ):
            del frame[key]
        frame.update(datum)
        frame.update(cases["fixed-datum-up"])
        frame.update(
            {
                k: vendor.cad.translated(v, [stroke, 0, 0])
                for k, v in cases["roof"].items()
            }
        )
        for k, v in wire.items():
            frame["wire/" + k] = vendor.cad.translated(
                v, [stroke if k in moving_wire else 0, 0, 0]
            )
    result["hardware"] = {
        "sourceParts": len(orig),
        "selectedPartsIncludingWire": len(stowed),
        "wireEnvelopes": len(wire),
        "replacementParts": len(datum),
        "sourceCaseSolidsPerHead": len(raw),
    }
    args.out.mkdir(parents=True, exist_ok=True)

    def save(name, bg):
        result["backgroundSha256"][name] = store(args.out, name, bg)

    stable = {}
    for name, part in stowed.items():
        a = np.array(vendor.cad.bounds(part))
        b = np.array(vendor.cad.bounds(placed[name])) - [370, 0, 0, 370, 0, 0]
        if np.max(abs(a - b)) < 0.001 and not (
            name.startswith(("wire/", "roof-vendor-", "support/"))
            or receiver_drives.translates_with_grip(name)
            or name.startswith("grip-")
        ):
            stable[name] = part
    print("Triangulating selected endpoint bodies", len(stowed), flush=True)
    meshes = {0: paint_world(stowed), 370: paint_world(placed)}
    rect = origins["fixed-datum-up"] + args.fixed_depth * FORWARD
    result["fixedDepthOffsetMm"] = args.fixed_depth
    fixed_pose(rect)
    if args.stowed_robot_only:
        parts = [r for r in meshes[0] if not r[0].startswith("fixed-datum-up-vendor-")]
        save("stowed-robot-fixed", camera.images_for_pose({"parts": parts}))
        (args.out / "station-background.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
        return
    fixed = {}
    for stroke, mesh in meshes.items():
        parts = [r for r in mesh if not r[0].startswith("fixed-datum-up-vendor-")]
        fixed[stroke] = camera.images_for_pose({"parts": parts})

    def to_mesh(items):
        return [
            (k, np.asarray(v, dtype=float).reshape(-1, 3) / 10, t)
            for k, _, _, v, t in paint_world(items)
        ]

    from registration import bridge

    reef = bridge.mod("optics_reef_field", ROOT / "moving-observer/clearance.py")
    station = to_mesh(station_parts(args.field))
    field = to_mesh(reef.reef_parts(args.field))
    nav = json.loads((ROOT / "relieved-match/navigation.json").read_bytes())["cases"]
    with (ROOT / "relieved-match/landing-windows.csv").open() as fh:
        settled = next(r for r in csv.DictReader(fh) if r["case"] == "coral-A/250/115")
    for piece, label in (("coral-A", "first"), ("coral-B", "return")):
        feed = json.loads(
            (ROOT / "relieved-match" / (label + "-feed.json")).read_bytes()
        )["cases"][0]
        dx, dy, h = feed["actualReturn"]
        c, s = math.cos(math.radians(h)), math.sin(math.radians(h))

        def station_to_robot(v):
            x, y = 444.525 - dx - v[:, 0], -dy - v[:, 1]
            return np.column_stack((x * c + y * s, -x * s + y * c, v[:, 2]))

        env = camera.images_for_pose(
            {"parts": rigid_mesh(station, station_to_robot, "station")}
        )
        save(label + "-fixed", combine(fixed[0], env))
        ex, ey, h = next(
            r["finalBodyErrorMmDeg"]
            for r in nav
            if r["pipe"] == piece and r["mode"] == "nominal"
        )
        c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
        cx, cy = -490 + ex, (-180 if piece == "coral-A" else 220) + ey

        def reef_to_robot(v):
            x, y = v[:, 0] - 3453.050980729168 - cx, -v[:, 2] - cy
            return np.column_stack((x * c + y * s, -x * s + y * c, v[:, 1]))

        env = camera.images_for_pose(
            {"parts": rigid_mesh(field, reef_to_robot, "reef")}
        )
        bg = combine(fixed[370], env)
        if piece == "coral-B":
            x, y = 1000 * float(settled["x_m"]) - cx, 1000 * float(settled["y_m"]) - cy
            pos = np.array(
                [c * x + s * y, -s * x + c * y, 1000 * float(settled["z_m"])]
            )
            a = np.array([float(settled["axis_" + d]) for d in "xyz"])
            axis = np.array([c * a[0] + s * a[1], -s * a[0] + c * a[1], a[2]])
            im = silhouette(bg, pos, axis)
            near = (pos - rect) @ FORWARD - 162
            bg = [
                (img, np.where(img != g, np.minimum(z, near), z))
                for img, (g, z) in zip(im, bg)
            ]
        save(piece + "-fixed", bg)
        result["operatingCases"][piece] = {
            "stationCorrectionMmDeg": [dx, dy, feed["actualReturn"][2]],
            "reefRobotMmDeg": [cx, cy, h],
            "firstPipeIncludedAtReef": piece == "coral-B",
        }
    if args.fixed_depth:
        from shelf_marks import apply
        from sight import background

        for name in list(result["backgroundSha256"]):
            stroke = 370 if name.startswith("coral-") else 0
            bg = background(args.out / (name + ".npz"))
            marked, _ = apply(
                bg,
                rect,
                FORWARD,
                np.array([math.sin(math.radians(26)), 0.0, math.cos(math.radians(26))]),
                stroke,
            )
            save(name, marked)
        (args.out / "scenes.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
        return
    stable_mesh = [r for r in meshes[0] if r[0] in stable]
    for label, height in (("barrel", 0.0), ("front", -0.75), ("rear", 0.75)):
        base = origins["roof"] + [0, 0, height]
        optics.ORIGIN = base.copy()
        optics.configure()
        save(
            label + "-moving-reference", camera.images_for_pose({"parts": stable_mesh})
        )
        for stroke, mesh in meshes.items():
            optics.ORIGIN = base + [stroke, 0, 0]
            optics.configure()
            components = [r for r in mesh if not r[0].startswith("roof-vendor-")]
            save(
                label + "-moving-" + str(stroke),
                camera.images_for_pose({"parts": components}),
            )
    result["stableRoofBackingParts"] = len(stable_mesh)
    (args.out / "scenes.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    print("Selected field and robot backgrounds rendered", flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for arg in ("artifacts", "lamp", "field", "out"):
        p.add_argument(arg, type=Path)
    p.add_argument("--stowed-robot-only", action="store_true")
    p.add_argument(
        "--assembly", type=Path, default=ROOT / "selected-match/clearance.py"
    )
    p.add_argument("--fixed-depth", type=float, choices=(-0.75, 0.0, 0.75), default=0.0)
    render(p.parse_args())
