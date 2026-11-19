"""Bound the continuous stationary harness, native fittings and their extraction paths."""

import argparse
from pathlib import Path
import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Pnt
from OCP.GC import GC_MakeArcOfCircle
from paths import routes, points
from scene import get, read, write, ROOT, CAD, m
from hardware import RADIUS, WALL, solids, service
from routing import power_feed
from itertools import combinations

HERE = Path(__file__).resolve().parent
RETIRED = {
    "observer-camera-5v-reserve",
    "observer-host-5v-reserve",
    "observer-pi4-service-envelope",
    "observer-usb-service-reserve",
    "observer-cable-trunk-reserve",
    "wire/data-feed",
}
NATIVE_BUMPERS = {"bumper-backing-rear", "bumper-cover-rear-red", "bumper-foam-rear"}
MANAGED = {"wire/carrier", "wire/power-feed"}
PAD = 0.50


def sample_box(pts, b):
    return np.linalg.norm(np.maximum(0, np.maximum(b[:3] - pts, pts - b[3:])), axis=1)


def box_gap(a, b):
    return float(np.linalg.norm(np.maximum(0, np.maximum(a[:3] - b[3:], b[:3] - a[3:]))))


def edges(parts, offset):
    result = []
    for _, ps, r in parts:
        p = ps + offset
        if r is None:
            e = BRepBuilderAPI_MakeEdge(gp_Pnt(*p[0]), gp_Pnt(*p[-1])).Edge()
        else:
            e = BRepBuilderAPI_MakeEdge(
                GC_MakeArcOfCircle(*(gp_Pnt(*row) for row in (p[0], p[len(p) // 2], p[-1]))).Value()
            ).Edge()
        result.append(e)
    return result


def nearest(a, b):
    x = BRepExtrema_DistShapeShape(a, b)
    if not x.IsDone():
        raise ValueError("Unresolved native separation")
    return x.Value()


def pair(a, b):
    return float(
        min(np.linalg.norm(a[k : k + 32, None, :] - b, axis=2).min() for k in range(0, len(a), 32))
    )


def jacket_and_form():
    rr, board = routes()
    paths = {}
    for key, p in rr.items():
        paths[key] = (p, np.zeros(3), RADIUS[key])
        # Keep the outer guide envelope over every station in the installed check.
        # The socket-service check opens both removable Pi-end sections.
        paths[key + "-form"] = (p, np.zeros(3), RADIUS[key] + WALL)
    return paths, board


def native_screen(args):
    start, end = get(args.artifacts, args.lamp)
    cached = read(args.bounds)
    current = {k: np.asarray(v) for k, v in cached["bounds"].items()}
    if set(current) != set(start):
        raise ValueError("Assembly bounds do not match registered scene")
    for name, h in cached["inputs"].items():
        if m.digest(ROOT / name) != h:
            raise ValueError("Changed registered source " + name)
    for name, body in start.items():
        if np.max(abs(np.asarray(CAD.bounds(body)) - cached["stowed"][name])) > 1e-5:
            raise ValueError("Stale installed pose " + name)
    if min(min(b[3:] - b[:3]) for b in current.values()) < -1e-7:
        raise ValueError("Inverted sweep bounds")
    parts, anchors = solids()
    paths, board = jacket_and_form()
    static = {k: np.asarray(CAD.bounds(v)) for k, v in parts.items()}
    all_gaps = {}
    pair_gaps = {}
    service_gaps = {}
    for name, (p, off, r) in paths.items():
        pts = points(p) + off
        es = edges(p, off)
        found = {}
        for k, b in current.items():
            if k in RETIRED | MANAGED:
                continue
            if name.startswith("power") and k == "wire/converter--140":
                continue
            gap = float(sample_box(pts, b).min()) - r - PAD
            if gap < 12 and k in NATIVE_BUMPERS:
                gap = min(nearest(e, start[k]) for e in es) - r
            found[k] = gap
        if name.startswith("roof-host"):
            # Carrier bottom run extends from x=-5 toward negative X only. The new
            # last straight meets its fixed entrance tangentially from positive X.
            other = np.vstack([s for _, s in power_feed(-329)])
            found["previous-roof-power-jacket"] = pair(pts, other) - r - 4.30 - 1.5
        for k, v in static.items():
            # Intended mates and guide attachment points are reported separately.
            support = k.startswith(
                ("mast-", "head-", "transition-", "base-guide-", "roof-guide-", "base-power")
            )
            if support and name.endswith("-form"):
                continue
            if k == "transition-capsule" and name in ("pigtail", "power"):
                continue
            if k.startswith("usb-A-") and (
                (name.startswith("roof-host") and "roof" in k)
                or (name.startswith("data") and "fixed" in k)
            ):
                continue
            if (name.startswith("data") and k == "usb-C-datum-side-exit") or (
                name.startswith("pigtail") and k == "usb-C-power-boot"
            ):
                continue
            found[k] = (
                min(nearest(e, parts[k]) for e in es) - r
                if sample_box(pts, v).min() < r + 10
                else float(sample_box(pts, v).min() - r - PAD)
            )
        closest = min(found, key=found.get)
        all_gaps[name] = {
            "body": closest,
            "lowerMm": round(float(found[closest]), 5),
            "concerns": {k: round(v, 5) for k, v in found.items() if v < 0.05},
        }
    for a, b in combinations(paths, 2):
        if a.split("-form")[0] == b.split("-form")[0]:
            continue
        pa, oa, ra = paths[a]
        pb, ob, rb = paths[b]
        gap = pair(points(pa) + oa, points(pb) + ob) - ra - rb - 1
        if gap < 1:
            gap = min(nearest(l, r) for l in edges(pa, oa) for r in edges(pb, ob)) - ra - rb
        pair_gaps[a + " / " + b] = round(gap, 5)
    for name, shape in {**parts, **service()}.items():
        b = np.array(CAD.bounds(shape))
        found = {}
        for k, b2 in current.items():
            if k in RETIRED | MANAGED:
                continue
            if k == anchors.get(name):
                continue
            gap = box_gap(b, b2)
            if gap < 6:
                # Narrow only stationary solids. Translating and rotating material stays enclosed.
                if (
                    np.max(abs(np.array(cached["stowed"][k]) - b2)) < 1e-5
                    and np.max(abs(np.array(CAD.bounds(end[k])) - b2)) < 1e-5
                ):
                    gap = nearest(shape, start[k])
            found[k] = gap
        peer_mates = {
            "fixed-C-data": "usb-C-datum-side-exit",
            "fixed-C-power": "usb-C-power-boot",
            "host-A-fixed": "usb-A-fixed-boot",
            "host-A-roof": "usb-A-roof-boot",
        }
        if name in peer_mates:
            for k, s in parts.items():
                if k == peer_mates[name] or (
                    name.startswith("host-A") and k == "host-board-and-components"
                ):
                    continue
                found[k] = (
                    nearest(shape, s) if box_gap(b, static[k]) < 10 else box_gap(b, static[k])
                )
        closest = min(found, key=found.get)
        service_gaps[name] = {
            "body": closest,
            "lowerMm": round(float(found[closest]), 5),
            "contacts": [k for k, v in found.items() if v < 0.05],
        }
    height = max(
        [CAD.bounds(s)[5] for s in parts.values()]
        + [np.max(points(p)[:, 2] + off[2]) + r for p, off, r in paths.values()]
    )
    return {
        "snapshotSources": cached["inputs"],
        "nativeBodies": len(start),
        "routeToHardware": all_gaps,
        "betweenUnrelatedRoutesMm": pair_gaps,
        "fittingsAndWithdrawal": service_gaps,
        "installedHighestMm": float(height),
        "sourceMatedShellGapMm": 8 - (7.5 + 8) / 2,
        "straightOldUpperMm": read(ROOT / "selected-optics/interfaces.json")["fit"][
            "fixedPlugGaugesBoundsMm"
        ]["power-with-transferred-straight-tail"][-1],
        "physical": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("artifacts", "lamp", "bounds", "out"):
        p.add_argument(k, type=Path)
    args = p.parse_args()
    write(args.out, native_screen(args))
