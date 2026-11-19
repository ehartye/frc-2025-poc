"""Static comparative unit-load screen of the routed +Y lower arm. Units N, mm."""

import json, math, sys
import numpy as np
import project_inputs

ROOT = project_inputs.project
sys.path.insert(0, str(ROOT / "guide-impact"))
from beam import support_source, bending_inertia, digest

ref = support_source(ROOT / "guide-mounts")
E = ref.E
G = E / (2 * 1.33)
UPPER = ref.arm(787.0)["totalMmPerN"]


def section(b, h, tb, th, opened=False):
    inertia = (h * b**3 - (h - 2 * th) * (b - 2 * tb) ** 3) / 12
    torsion = 4 * ((b - tb) * (h - th)) ** 2 / (2 * ((b - tb) / th + (h - th) / tb))
    if opened:
        torsion = 2 * ((b - tb) * th**3 + (h - th) * tb**3) / 3
    return inertia, torsion


def segments(name, opened=False):
    xc = 165 + 0.46 * (687 - 625)
    restored = name != "rise-+73"
    tip = np.array([xc, 687.0]) if restored else np.array([xc + 73 * 0.46, 760.0])
    offset, w, dep, wall = {
        "front-return": (10.0, 16.0, 10.6, 0.9),
        "central-return": (0.0, 22.0, 10.6, 0.9),
        "deep-return": (8.0, 20.0, 20.0, 1.2),
        "rise-+73": (0.0, 0.0, 0.0, 0.0),
    }[name]
    pieces = {}

    def piece(label, a, b, shape):
        a, b = np.array(a), np.array(b)
        length = np.linalg.norm(b - a)
        if length == 0:
            return
        t = (b - a) / length
        i, j = shape
        # Root-to-tip load at the actual face, integrated exactly for linear moments.
        r = tip - np.stack([a, b])
        bend = r @ t
        twist = -r[:, 1] * t[0] + r[:, 0] * t[1]

        def integral(m):
            return length * (m[0] ** 2 + m[0] * m[1] + m[1] ** 2) / 3

        pieces[label] = dict(
            bend=integral(bend) / (E * i), twist=integral(twist) / (G * j)
        )

    xhi = xc + 73 * 0.46 + offset
    c = 71 / math.hypot(71, 73)
    base = section(30, 12, 0.8, 0.8, opened)
    piece("stub", [-14, 687], [4, 687], base)
    piece("rising", [4, 687], [75, 760], section(30, 12 * c, 0.8, 0.8 * c, opened))
    piece("high", [75, 760], [xhi, 760], base)
    if restored:
        # Horizontal X-width projects onto the normal of the sloping return.
        c = 1 / math.hypot(1, 0.46)
        piece(
            "return",
            [xhi, 760],
            [xc + offset, 687],
            section(dep, w * c, wall, wall * c, opened),
        )
        # Full 73-mm centerline includes both fused joints. Pocket transfer over the
        # horizontal offset assumed rigid; free edge / weld compliance is unmeasured.
    root = ref.arm(687.0)
    lever = tip[0] + 14
    fac = (lever / root["lengthMm"]) ** 2
    root_terms = {
        k: v * (1 if k == "rootTension" else fac)
        for k, v in root["termsMmPerN"].items()
        if k != "tubeBending"
    }
    bolt_shear = 200000 / (2 * 1.3) * 8.78 / 56
    root_terms["boltRollUnclamped"] = (tip[1] - 687) ** 2 / (
        bolt_shear * (14**2 + 10**2) / 2
    )
    return dict(
        compliance=sum(sum(d.values()) for d in pieces.values())
        + sum(root_terms.values()),
        members=pieces,
        root=root_terms,
    )


def screens(rows, lo, compliance):
    values = {}
    for side in (-1, 1):
        h = rows[rows[:, 1] == side]
        z = h[:, 3]
        f = h[:, 2]
        a = (z - (687 if side == -1 else lo)) * math.hypot(1, 0.46)
        span = (787 - (687 if side == -1 else lo)) * math.hypot(1, 0.46)
        b = span - a
        c = ref.arm(687.0)["totalMmPerN"] if side == -1 else compliance
        e = np.maximum(0, -np.minimum(a, b))
        spine = np.where(
            np.minimum(a, b) >= 0, a * a * b * b / span, e * e * (span + e)
        ) / (3 * E * bending_inertia(4))
        spring = f * (b * b * c + a * a * UPPER) / span**2
        rl = f * b / span
        ru = f * a / span

        def worst(u):
            i = np.argmax(abs(u))
            return {
                "mm": float(u[i]),
                "timeMs": float(h[i, 0]),
                "forceN": float(f[i]),
                "zMm": float(z[i]),
                "lowerN": float(rl[i]),
                "upperN": float(ru[i]),
            }

        values[str(side)] = {
            "peakMm": worst(spring + f * spine),
            "rigidSpine": worst(spring),
            "peakLoN": max(abs(rl)),
            "peakHiN": max(abs(ru)),
            "minLoN": min(rl),
            "minHiN": min(ru),
        }
    return values


def run(raw, routes, out):
    reports = {}
    layouts = {}
    for k in ("rise-+73", "front-return", "central-return", "deep-return"):
        for opened in (False, True):
            layouts[k + ("-open" if opened else "")] = segments(k, opened)
    layouts["straight"] = {"compliance": ref.arm(687.0)["totalMmPerN"]}
    paths = {
        k: digest(routes / k / "route.step")
        for k in ("rise-+73", "front-return", "central-return", "deep-return")
    }
    for name, value in paths.items():
        if project_inputs.routes[name] != value:
            raise ValueError("Different optical alternative: " + name)
    reg = json.loads((ROOT / "guide-impact/results.json").read_bytes())
    for case in ("leftSmall", "rightSmall"):
        for family in ("originalRigid", "serialRecovered"):
            if family == "originalRigid":
                loc = next(raw.glob(f"{case}-rigid-31.25us*"))
                file = loc / (loc.name + ".npy")
                hist = json.loads(
                    (ROOT / "conditioned-match/elastic.json").read_bytes()
                )
                source_hash = digest(file)
                if not any(
                    v["case"] == case and v["trajectorySha256"] == source_hash
                    for v in hist["cases"]
                ):
                    raise ValueError("Unknown signed rigid trajectory")
                rows, _ = ref.contacts(np.load(file, mmap_mode="r"))
            else:
                loc = next(raw.glob(f"{case}-folded-31.25us*serial-carriage-recover"))
                file = loc / "guide.npy"
                source_hash = digest(file)
                if not any(
                    v["case"] == case and v["guideHistorySha256"] == source_hash
                    for v in reg["cases"]
                ):
                    raise ValueError("Unknown signed serial trajectory")
                h = np.load(file, mmap_mode="r")
                arr = []
                for i, side in enumerate((-1, 1)):
                    active = h[:, 1 + i] > 1e-8
                    arr.extend(
                        np.column_stack(
                            [
                                100 + 1000 * h[active, 0],
                                np.full(np.sum(active), side),
                                h[active, 1 + i],
                                1000 * h[active, 3 + i],
                            ]
                        )
                    )
                rows = np.array(arr)
            reports[case + "-" + family] = {
                "source": digest(file),
                "layouts": {
                    k: screens(
                        rows, 760 if k.startswith("rise") else 687, v["compliance"]
                    )
                    for k, v in layouts.items()
                },
            }
    report = {
        "sources": paths,
        "layouts": layouts,
        "cases": reports,
        "basis": "Static planar unit Y load; original issued normal/COP substituted for lateral force. Fused joints, centered transfer, linear small strain, no clamp slip or root opening. No physical measurements.",
    }
    (out / "screen.json").write_text(
        json.dumps(report, indent=2, allow_nan=False), encoding="utf8"
    )
    for k, r in layouts.items():
        print(
            k,
            round(r["compliance"], 6),
            [
                (
                    n,
                    round(v["layouts"][k]["1"]["peakMm"]["mm"], 3),
                    round(v["layouts"][k]["1"]["rigidSpine"]["mm"], 3),
                )
                for n, v in reports.items()
            ],
        )
