"""Project issued occupied volumes into every selected eye with both pupil-depth controls."""

import argparse
import itertools
from pathlib import Path
import numpy as np
from harness_fit import ROOT, read, write, CAD, RADIUS
from hardware import solids, RODS, WALL
from paths import routes, points, F, L, U
from hashlib import sha256
from sys import path as sys_path

sys_path.insert(0, str(ROOT / "selected-optics"))
from scenes import store


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def register(bounds):
    output = []
    for name, route in routes()[0].items():
        a = points(route)
        for k in range(0, len(a) - 1, 16):
            chunk = a[k : k + 17]
            low = chunk.min(0) - RADIUS[name] - WALL - 1.05
            high = chunk.max(0) + RADIUS[name] + WALL + 1.05
            output.append(np.r_[low, high])
    for s in solids()[0].values():
        if s not in RODS:
            output.append(np.asarray(CAD.bounds(s)) + [-1, -1, -1, 1, 1, 1])
            continue
        a, b, r = RODS[s]
        poly = np.linspace(a, b, 1 + int(np.ceil(np.linalg.norm(b - a) / 5)))
        for s, t in zip(poly, poly[1:]):
            output.append(np.r_[np.minimum(s, t) - r - 1, np.maximum(s, t) + r + 1])
    # No ray can obtain new backing credit from a removed reserve. The older broad
    # data-feed and Pi reservation were both opaque in the predecessor rendering.
    from harness_fit import RETIRED

    output.extend(np.asarray(bounds[k]) for k in RETIRED)
    return output


def depths(boxes, o, f, up):
    v, u = np.mgrid[:480, :640] + 0.5
    focal = 320 / np.tan(np.deg2rad(36.5))
    ray = (
        f[:, None, None]
        - (u - 320) / focal * L[:, None, None]
        + (240 - v) / focal * up[:, None, None]
    )
    result = np.full((480, 640), np.inf)
    for b in boxes:
        vs = np.array(list(itertools.product(*zip(b[:3], b[3:])))) - o
        z = vs @ f
        if max(z) <= 0:
            continue
        if min(z) > 0:
            uu = 320 - focal * (vs @ L) / z
            vv = 240 - focal * (vs @ up) / z
            left, right = np.clip([np.floor(uu.min()) - 1, np.ceil(uu.max()) + 1], 0, 640).astype(
                int
            )
            top, bot = np.clip([np.floor(vv.min()) - 1, np.ceil(vv.max()) + 1], 0, 480).astype(int)
        else:
            left, right, top, bot = 0, 640, 0, 480
        if left >= right or top >= bot:
            continue
        region = np.s_[top:bot, left:right]
        nr = np.full((bot - top, right - left), -np.inf)
        far = np.full_like(nr, np.inf)
        for i, r in enumerate(ray):
            rr = r[region]
            with np.errstate(divide="ignore", invalid="ignore"):
                z0 = (b[i] - o[i]) / rr
                z1 = (b[i + 3] - o[i]) / rr
            nr = np.maximum(nr, np.minimum(z0, z1))
            far = np.minimum(far, np.maximum(z0, z1))
        hit = (far >= nr) & (far > 0)
        result[region] = np.minimum(result[region], np.where(hit, np.maximum(0, nr), np.inf))
    return result


def run(args):
    bound = read(args.bounds)["stowed"]
    boxes = register(bound)
    scenes = read(args.source / "scenes.json")
    sweep = read(args.source / "sweep.json")
    if scenes["hardware"]["selectedPartsIncludingWire"] != 991:
        raise ValueError("Expected #94 installed hardware")
    mount = read(ROOT / "selected-optics/mount.json")["registrations"]
    table = dict(scenes["backgroundSha256"])
    table["barrel-moving-swept"] = sweep["backgroundSha256"]
    table["barrel-moving-nearest"] = sweep["nearestPossibleSha256"]
    data = {}
    memo = {}
    args.out.mkdir(parents=True, exist_ok=True)
    for name, h in table.items():
        path = args.source / (name + ".npz")
        if digest(path) != h:
            raise ValueError("Changed reference " + name)
        source = np.load(path)
        fixed = name.endswith("-fixed")
        f = F if fixed else np.array([0, 0, -1])
        up = U if fixed else np.array([1, 0, 0])
        dz = (
            scenes.get("fixedDepthOffsetMm", 0.0)
            if fixed
            else {"front": -0.75, "barrel": 0.0, "rear": 0.75}[name.split("-")[0]]
        )
        pos = np.array(mount["fixed-datum-up" if fixed else "roof"]["assumedRectifiedOriginMm"])
        pos += f * dz if fixed else [370 if name.endswith("-370") else 0, 0, dz]
        sweep_bounds = name.endswith(("-nearest", "-swept", "-reference"))
        volumes = [b - np.array([370, 0, 0, 0, 0, 0]) for b in boxes] if sweep_bounds else boxes
        key = tuple(pos) + (fixed, sweep_bounds)
        if key not in memo:
            memo[key] = [depths(volumes, pos + [0, eye, 0], f, up) for eye in (37.5, -37.5)]
        bg = []
        count = []
        for side, near in zip(("left", "right"), memo[key]):
            g = source[side]
            z = source[side + "_z"]
            invalid = (near < z + 0.2) & (z < 3490)
            # Refuse all potentially altered backing. Retain foreground obstacles in the
            # nearest-possible sweep so a new body cannot make the pipe visible through itself.
            if name.endswith("-nearest"):
                bg.append((g.copy(), np.where(z < 3490, np.minimum(z, near), z)))
            else:
                bg.append((np.where(invalid, 0, g).astype(np.uint8), np.where(invalid, 3500, z)))
            count.append(int(invalid.sum()))
        new = store(args.out, name, bg)
        data[name] = {"source": h, "guardedPixels": count, "output": new}
        if name.endswith("-nearest"):
            sweep["nearestPossibleSha256"] = new
        elif name.endswith("-swept"):
            sweep["backgroundSha256"] = new
        else:
            scenes["backgroundSha256"][name] = new
    write(args.out / "scenes.json", scenes)
    write(args.out / "sweep.json", sweep)
    write(
        args.out / "route-visibility.json",
        {
            "source": digest(args.source / "scenes.json"),
            "nativeBodies": 991,
            "routeBounds": len(boxes),
            "images": data,
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("bounds", "source", "out"):
        p.add_argument(key, type=Path)
    run(p.parse_args())
