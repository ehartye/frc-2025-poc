"""Four fixed side-rail trays and service boxes for the eight registered controllers."""

import argparse
from pathlib import Path
import sys
import numpy as np
from electrical import HERE, ROOT, IDS, PACKAGE
from screen import write, sha

sys.path.insert(0, str(ROOT / "guide-arm-route"))
from arm_route import carriage, j, inputs
from OCP.BRepCheck import BRepCheck_Analyzer


def cyl(x, y, z, h, r):
    return j.cad.brep(["cyl", [x, y, z, 0, 0, 1, r, h]])


def tray(i, initial_rr=False):
    sx, sy = ((1, 1), (1, -1), (-1, 1), (-1, -1))[i]
    x = -99.0 if IDS[i] == "RR" and not initial_rr else sx * 141.0

    def rect(a, b, l, r, z, Z):
        return j.box(x + a, x + b, sy * l, sy * r, z, Z)

    plate = rect(-41, 41, 249, 347, 155, 157)
    # The inboard edge folds up; both side returns keep the thin floor stiff.
    for bound in [(-41, -39, 249, 347), (39, 41, 249, 347)]:
        plate = j.fuse(plate, rect(*bound, 157, 164))
    plate = j.fuse(plate, rect(-39, 39, 249, 251, 157, 161))
    pad = rect(-40, 40, 328, 346, 150, 155)
    plate = j.fuse(plate, pad)
    for dx in (-26, 26):
        plate = j.cut(plate, cyl(x + dx, sy * 337, 149, 10, 2.25))
    ctr = {"drive": (x, sy * 274), "steer": (x, sy * 313)}
    for cx, cy in ctr.values():
        for dx in (-22, 22):
            for dy in (-19, 19):
                plate = j.cut(
                    plate,
                    j.box(cx + dx - 2.7, cx + dx + 2.7, cy + dy - 0.85, cy + dy + 0.85, 154.5, 158),
                )
    housings = {
        key: j.box(cx - 35, cx + 35, cy - 17.5, cy + 17.5, 157, 182.5)
        for key, (cx, cy) in ctr.items()
    }
    svc = {
        key: j.box(cx - 35, cx + 35, cy - 17.5, cy + 17.5, 157, 222.5)
        for key, (cx, cy) in ctr.items()
    }
    return plate, housings, svc


def retained(assets):
    registry = assets / "fixed-camera-route/scene.json"
    scene, edges, _ = carriage.scene(registry, ROOT / "fixed-hoist")
    native, _ = j.read_shop(assets) if hasattr(j, "read_shop") else (None, None)
    if native is None:
        from continuous_screen import read_shop

        native, _ = read_shop(assets)
    for k in list(scene):
        if k.startswith(("bumper-", "drive-rail-outer")) and k in native:
            scene[k] = native[k]
    # Both old drive controller proxies move into these proposed locations.
    retired = {
        k
        for k in scene
        if k.startswith(("wheel-rear-", "wheel-center-", "wheel-front-", "drive-controller-"))
    }
    for k in retired:
        scene.pop(k)
    package = PACKAGE
    import json

    frame = json.loads((package / "frame.json").read_bytes())
    if sha(package / "swerve-frame.step") != frame["stepSha256"]:
        raise ValueError("Changed chassis mounting kit")
    frame_bodies = j.cad.read_named(
        package / "swerve-frame.step", [{"id": k} for k in frame["parts"]]
    )
    scene.update({"swerve/" + k: v for k, v in frame_bodies.items()})
    return scene, edges


def run(assets, output):
    scene, edges = retained(assets)
    pieces = {}
    results = {}
    kit = PACKAGE / "module.json"
    modules = __import__("json").loads(kit.read_bytes())["modules"]
    for i, site in enumerate(IDS):
        part, housing, service = tray(i)
        sx, sy = np.sign([modules[site]["xM"], modules[site]["yM"]])
        pieces["pair-tray-" + site] = part
        side = "L" if sy > 0 else "R"
        rail = scene["drive-rail-outer-" + side]
        # Cylindrical bore to be installed in retained rail top wall. Require both
        # the native stock and a hollow cavity for the blind fastener projection.
        holes = []
        for dx in (-26, 26):
            x = (-99.0 if site == "RR" else sx * 141) + dx
            y = sy * 337
            tool = cyl(x, y, 144.0, 8.0, 3.1)
            holes.append(
                dict(
                    xMm=x,
                    yMm=y,
                    outerWallCutMm3=j.cad.intersection(rail, tool),
                    bodyClearBelowMm3=j.cad.intersection(rail, cyl(x, y, 136.0, 10.0, 3.1)),
                )
            )
        obj = {**housing, **{k + "-service": v for k, v in service.items()}, "pair-tray": part}
        mxy = np.array([modules[site]["xM"], modules[site]["yM"]]) * 1000
        minModuleX = abs(mxy[0]) - 88.646
        fit = {}
        for name, s in obj.items():
            bb = np.array(j.cad.bounds(s))
            others = {}

            def inspect(k, body, b):
                if (np.maximum(np.array(b[:3]) - bb[3:], bb[:3] - b[3:]) < 6.0).all():
                    distance = j.distance(s, body)
                    common = j.cad.intersection(s, body) if distance < 0.002 else 0.0
                    others[k] = [float(distance), float(common)]

            for k, other in scene.items():
                inspect(k, other, j.cad.bounds(other))
            cable = {}
            for key, edge, b, radius in edges:
                gap = float(np.max(np.maximum(np.array(b[:3]) - bb[3:], bb[:3] - b[3:])))
                if gap < radius + 6:
                    cable[key] = min(cable.get(key, 1e9), j.distance(s, edge) - radius)
            fit[name] = dict(
                boundsMm=bb.tolist(),
                near=others,
                nearCableMarginMm=cable,
                nativeModuleXSeparationLowerMm=float(minModuleX - max(abs(bb[0]), abs(bb[3]))),
            )
        results[site] = dict(volumeMm3=j.cad.volume(part)[0], holes=holes, fit=fit)
    j.write_step(output, pieces)
    j.finalize_step(output)
    reopen = j.cad.read_named(output, [{"id": k} for k in pieces])
    for k, v in reopen.items():
        if (
            not BRepCheck_Analyzer(v).IsValid()
            or j.cad.solid_count(v) != 1
            or abs(j.cad.volume(v)[0] - j.cad.volume(pieces[k])[0]) > 0.01
        ):
            raise ValueError("Tray export changed " + k)
    _, earlier, earlier_service = tray(3, True)
    adverse = {
        k: j.cad.intersection(v, scene["data-trough-14"])
        for k, v in (("housingMm3", earlier["drive"]), ("serviceMm3", earlier_service["drive"]))
    }
    if min(adverse.values()) <= 0:
        raise ValueError("Original trough counterexample changed")
    if any(
        pair[1] > 0.0001
        for row in results.values()
        for bound in row["fit"].values()
        for pair in bound["near"].values()
    ):
        raise ValueError("Selected controller frames intersect")
    if any(
        x <= 0
        for row in results.values()
        for bound in row["fit"].values()
        for x in bound["nearCableMarginMm"].values()
    ):
        raise ValueError("Selected controller frames enter an existing cable")
    return dict(
        stepSha256=sha(output),
        registrySha256=sha(assets / "fixed-camera-route/scene.json"),
        moduleSha256=sha(kit),
        initialRearRightAtMinus141=adverse,
        sweptOriginalObstacles=len(scene),
        cableEdges=len(edges),
        corners=results,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    p.add_argument("step", type=Path)
    p.add_argument("receipts", type=Path)
    a = p.parse_args()
    write(a.receipts, run(a.assets, a.step))
