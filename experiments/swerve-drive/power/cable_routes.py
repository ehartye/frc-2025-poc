"""Route fixed harness reservations through the selected stowed and moving robot envelopes."""

import argparse
from heapq import heappop, heappush
from itertools import product
from pathlib import Path
import numpy as np
from electrical import HERE, ROOT, IDS, PACKAGE
from screen import write, sha
from controller_fit import retained, tray, j

ORIGIN = np.array([-330.0, -330.0, 168.0])
CELL = 10.0
DIRECTIONS = list(product((-1, 0, 1), repeat=3))
DIRECTIONS.remove((0, 0, 0))


def distance_box(a, b):
    return float(np.linalg.norm(np.maximum(0, np.maximum(a[:3] - b[3:], b[:3] - a[3:]))))


def edge(a, b):
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
    from OCP.gp import gp_Pnt

    return BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge()


def segbox(a, b):
    return np.r_[np.minimum(a, b), np.maximum(a, b)]


def fillet(points, radius):
    # Circular path with fixed radius. Reject turns whose tangency arcs overlap.
    path = [points[0]]
    last_tangent = 0.0
    for a, b, c in zip(points, points[1:], points[2:]):
        u = (b - a) / np.linalg.norm(b - a)
        v = (c - b) / np.linalg.norm(c - b)
        angle = np.arccos(np.clip(u @ v, -1.0, 1.0))
        if angle < 1e-6:
            continue
        length = radius * np.tan(angle / 2)
        if length + last_tangent > np.linalg.norm(b - a) or length > np.linalg.norm(c - b):
            raise ValueError("Insufficient straight run for selected bend")
        perp = (v - u * (u @ v)) / np.sin(angle)
        start = b - length * u
        center = start + radius * perp
        path.append(start)
        for theta in np.linspace(0, angle, max(3, int(np.ceil(angle / 0.12)) + 1))[1:]:
            path.append(center + radius * (-perp * np.cos(theta) + u * np.sin(theta)))
        last_tangent = length
    path.append(points[-1])
    return path


def robot(assets):
    scene, edges = retained(assets)
    for i, site in enumerate(IDS):
        plate, housings, _ = tray(i)
        scene["new-tray-" + site] = plate
        scene.update({site + "-" + k: v for k, v in housings.items()})
    mod = PACKAGE / "module.json"
    import json

    m = json.loads(mod.read_bytes())["modules"]
    # Reserve the complete module including 90 mm radial upper housing box up
    # to measured 171.5 mm. Wheel and gear sweep below is out of the route domain.
    for site, spec in m.items():
        x, y = 1000 * np.array([spec["xM"], spec["yM"]])
        scene["module-" + site] = j.box(x - 90, x + 90, y - 90, y + 90, 0, 171.5)
    parts = {k: (np.array(j.cad.bounds(v)), v, 0.0) for k, v in scene.items()}
    for i, (key, e, bb, r) in enumerate(edges):
        parts["prior/" + key + str(i)] = (np.array(bb), e, r)
    return parts


def search(parts, start, end, radius):
    shape = np.array([67, 67, 14])
    mask = np.zeros(shape, dtype=bool)
    sx, ex = (np.rint((p - ORIGIN) / CELL).astype(int) for p in (start, end))
    for k, (b, body, r) in parts.items():
        if k.startswith(("bumper-foam", "bumper-cover", "bumper-backing")):
            continue
        pad = radius + r + 6.0
        a = np.ceil((b[:3] - pad - ORIGIN) / CELL).astype(int)
        z = np.floor((b[3:] + pad - ORIGIN) / CELL).astype(int)
        a = np.maximum(0, a)
        z = np.minimum(shape - 1, z)
        if (z >= a).all():
            where = tuple(slice(i, jj + 1) for i, jj in zip(a, z))
            if k.startswith("loom-"):
                from OCP.BRepAdaptor import BRepAdaptor_Curve

                c = BRepAdaptor_Curve(body)
                p = np.array(c.Value(c.FirstParameter()).Coord())
                q = np.array(c.Value(c.LastParameter()).Coord())
                ix = np.moveaxis(np.indices(tuple(z - a + 1)), 0, -1) + a
                world = ix * CELL + ORIGIN
                v = q - p
                u = np.clip(((world - p) @ v) / (v @ v), 0.0, 1.0)
                mask[where] |= np.linalg.norm(world - p - u[..., None] * v, axis=-1) <= pad
            else:
                mask[where] = True
    for n in (sx, ex):
        if mask[tuple(n)]:
            x = n * CELL + ORIGIN
            overlaps = [
                k
                for k, (b, _, r) in parts.items()
                if (x >= b[:3] - r - radius - 6).all()
                and (x <= b[3:] + r + radius + 6).all()
                and not k.startswith(("bumper-foam", "bumper-cover", "bumper-backing"))
            ]
            raise ValueError(f"Blocked endpoint {x.tolist()} {overlaps}")

    def clear(a, b):
        n = max(2, int(np.linalg.norm(b - a) * 4) + 1)
        points = np.rint(np.linspace(a, b, n)).astype(int)
        return not mask[tuple(points.T)].any()

    origin = tuple(sx)
    goal = tuple(ex)
    que = [(0.0, 0.0, origin)]
    prev = {origin: None}
    cost = {origin: 0.0}
    while que:
        _, g, a = heappop(que)
        if g > cost[a]:
            continue
        if a == goal:
            break
        for dr in DIRECTIONS:
            b = tuple(i + k for i, k in zip(a, dr))
            if any(x < 0 or x >= y for x, y in zip(b, shape)) or mask[b]:
                continue
            v = g + sum(x * x for x in dr) ** 0.5
            if v < cost.get(b, 1e9):
                cost[b] = v
                prev[b] = a
                h = sum((x - y) ** 2 for x, y in zip(b, goal)) ** 0.5
                heappush(que, (v + h, v, b))
    if goal not in prev:
        raise ValueError("No free grid corridor")
    nodes = []
    p = goal
    while p is not None:
        nodes.append(np.array(p))
        p = prev[p]
    nodes.reverse()
    reduced = [nodes[0]]
    idx = 0
    while idx < len(nodes) - 1:
        j = next(j for j in range(len(nodes) - 1, idx, -1) if clear(nodes[idx], nodes[j]))
        reduced.append(nodes[j])
        idx = j
    return [n * CELL + ORIGIN for n in reduced]


def qualify(parts, points, radius):
    near = 999.0
    hit = None
    for a, b in zip(points, points[1:]):
        if np.linalg.norm(b - a) < 1e-6:
            continue
        bb = segbox(a, b)
        seg = None
        for key, (box, body, r) in parts.items():
            lower = distance_box(bb, box) - r - radius
            if lower >= near:
                continue
            if lower > 2.0:
                near = lower
                hit = key
                continue
            if seg is None:
                seg = edge(a, b)
            gap = j.distance(seg, body) - r - radius
            if gap < near:
                near = gap
                hit = key
    return near, hit


def run(assets):
    parts = robot(assets)
    rows = {}
    # Independent power+CAN trunks connect a PDH breakout reservation to fixed
    # connector/service handoffs above the controllers. End adapters are separate.
    terminals = {
        "FL-breakout": [-170, 230, 208],
        "RL-breakout": [-200, 230, 218],
        "FR-breakout": [-80, 120, 218],
        "RR-breakout": [-110, 80, 218],
        "FL": [140, 300, 198],
        "FR": [140, -300, 198],
        "RL": [-140, 300, 198],
        "RR": [-100, -300, 198],
    }
    for site in IDS:
        start = site + "-breakout"
        try:
            way = search(parts, np.array(terminals[start]), np.array(terminals[site]), 8.0)
            pts = fillet(way, 30.0)
            # The chord sagitta for at most 0.12 rad/sample at 30 mm is <0.055 mm.
            gap, k = qualify(parts, pts, 8.06)
            if gap > 0:
                for i, (a, b) in enumerate(zip(pts, pts[1:])):
                    if np.linalg.norm(b - a) > 0.001:
                        parts[f"loom-{site}-{i}"] = (segbox(a, b), edge(a, b), 8.06)
            rows[site] = dict(
                start=start,
                radiusMm=8.0,
                bendRadiusMm=30.0,
                polylineVerticesMm=[p.tolist() for p in way],
                lengthMm=sum(np.linalg.norm(b - a) for a, b in zip(pts, pts[1:])),
                lowerGapMm=gap,
                nearest=k,
            )
        except ValueError as e:
            rows[site] = dict(start=start, unresolved=str(e))
    phase = {}
    for site in IDS:
        cx = -99 if site == "RR" else (141 if site.startswith("F") else -141)
        sx = np.sign(cx)
        sy = 1 if site.endswith("L") else -1
        for motor, y in (("Drive", 274), ("Steer", 313)):
            start = np.array([round((cx + sx * 47) / 10) * 10, round(sy * y / 10) * 10, 198.0])
            center = np.array(
                [sx * (282 if motor == "Drive" else 247), sy * (277 if motor == "Drive" else 242)]
            )
            end = np.r_[np.round(center / 10) * 10, 188.0]
            if site == "RR" and motor == "Drive":
                start = np.array([-120.0, -270.0, 208.0])
            if site == "RR" and motor == "Steer":
                end[2] = 218.0
            key = site + motor
            try:
                chosen = None
                # Keep phase and sensor bundles in separate corridors from all
                # issued supply runs. Try higher breakouts when two paths cross.
                for lift in (0.0, 20.0, 40.0, 60.0):
                    for shift in (0.0, -15.0, 15.0):
                        a = start + [0.0, 0.0, lift]
                        b = end + [0.0, shift, lift]
                        gap, k = qualify(parts, [a, b], 5.06)
                        if gap > 1.0:
                            chosen = ([a, b], [a, b], gap, k)
                            break
                    if chosen:
                        break
                if chosen:
                    way, pts, gap, k = chosen
                else:
                    way = search(parts, start, end, 5.0)
                    pts = fillet(way, 25.0)
                    gap, k = qualify(parts, pts, 5.06)
                if gap > 0:
                    for i, (a, b) in enumerate(zip(pts, pts[1:])):
                        if np.linalg.norm(b - a) > 0.001:
                            parts[f"loom-{key}-{i}"] = (segbox(a, b), edge(a, b), 5.06)
                phase[key] = dict(
                    radiusMm=5.0,
                    bendRadiusMm=25.0,
                    polylineVerticesMm=[p.tolist() for p in way],
                    lengthMm=sum(np.linalg.norm(b - a) for a, b in zip(pts, pts[1:])),
                    lowerGapMm=gap,
                    nearest=k,
                )
            except ValueError as e:
                phase[key] = dict(unresolved=str(e), startMm=start.tolist(), endMm=end.tolist())
    return dict(
        groupedSupplyTargets=rows,
        motorAndSensorTargets=phase,
        sourceEndpointsMm=terminals,
        projectRouteMinimumBendMm=30.0,
        componentConnectorTailQualification=None,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    write(a.out, run(a.assets))
