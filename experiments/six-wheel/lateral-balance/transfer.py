"""Evaluate signed lateral and fore-aft load lines at the registered CG, on both opening clocks."""

import argparse
import hashlib
import itertools
import json
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "selected-drive"))


def read(p):
    return json.loads(p.read_bytes())


def write(p, v):
    p.write_text(
        json.dumps(v, indent=2, allow_nan=False) + "\n", encoding="utf8", newline="\n"
    )


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def components():
    inventory = {}
    for recipe in ("parcels", "kits"):
        values = read(HERE / (recipe + ".json"))["parcels"]
        if set(values) & set(inventory):
            raise ValueError("A mass parcel occurs in both inventories")
        inventory.update(values)
    return inventory


def pack_fill(parts, density):
    row = parts["NP18-12 battery"].copy()
    box = np.array(row["positionMm"])
    spans = box[3:] - box[:3]
    areas = np.prod(spans) / spans
    mass = row["kg"][0]
    # Densest permitted phase is packed against the face; the minimum ten per cent
    # electrolyte follows it. This rearrangement minimizes the signed first moment.
    inset = mass * 1e6 / (2 * areas) * (0.99 / density + 0.01 / 1.35)
    if np.any(2 * inset >= spans):
        raise ValueError("Constituent volumes exceed the available slab")
    row["positionMm"] = np.r_[box[:3] + inset, box[3:] - inset].tolist()
    return {**parts, "NP18-12 battery": row}


def paired(parts):
    """Enclose each pair's possible position, retaining the catalog interval on total mass."""
    out = dict(parts)
    for name, mismatch in [
        (f"wheel-{ax}", 0.020) for ax in ("front", "center", "rear")
    ] + [("gearbox", 0.025)]:
        left, right = [out.pop(name + "-" + sign) for sign in ("L", "R")]
        lo, hi = left["kg"]
        if [lo, hi] != right["kg"] or not lo < hi or mismatch > hi - lo:
            raise ValueError(name)
        vertices = (
            (lo, lo),
            (hi, hi),
            (lo, lo + mismatch),
            (lo + mismatch, lo),
            (hi, hi - mismatch),
            (hi - mismatch, hi),
        )
        loc = [
            (min if j < 3 else max)(
                (a * left["positionMm"][j] + b * right["positionMm"][j]) / (a + b)
                for a, b in vertices
            )
            for j in range(6)
        ]
        out["matched/" + name] = {"kg": [2 * lo, 2 * hi], "positionMm": loc}
    return out


def cg_box(parts, masses, residual):
    """Exact axial extrema over independent mass intervals, a total mass interval and a residual CG box."""
    lo, hi = masses
    if sum(p["kg"][1] for p in parts.values()) > lo:
        raise ValueError("Intervals require a constrained residual-mass solve")
    ends = []
    for j in range(6):
        u = residual[j]
        moment = sum(
            (min if j < 3 else max)(k * (p["positionMm"][j] - u) for k in p["kg"])
            for p in parts.values()
        )
        ends.append((min if j < 3 else max)(u + moment / M for M in masses))
    return ends


def traces(assets):
    from route import wheel_steps

    d = read(ROOT / "wheel-loads/opening.json")
    out = {}
    for sign, row in d["signed"].items():
        path = assets / "straight-guide-match" / (sign + "-revised-opening.npz")
        if digest(path) != row["clockSha256"]:
            raise ValueError("Different opening")
        with np.load(path) as frames:
            t = frames["t"].copy()
            pose = frames["pose"].copy()
        dt = np.diff(t)
        if len(dt) != 4220 or not np.allclose(dt, 0.005):
            raise ValueError("Different cadence")
        speeds = wheel_steps(pose) / dt[:, None]
        v = speeds.mean(axis=1)
        w = (speeds[:, 1] - speeds[:, 0]) / 0.580
        # Permit either adjacent acceleration on the finite interval. Pad with rest at both ends.
        out[sign] = (
            t[:-1],
            v,
            w,
            np.column_stack((np.diff(v, prepend=0), np.diff(v, append=0)))
            / dt[:, None],
            np.column_stack((np.diff(w, prepend=0), np.diff(w, append=0)))
            / dt[:, None],
        )
    return out


def extrema(box, sources):
    extremes = {}
    for sign, (t, v, w, accel, alpha) in sources.items():
        worst = {axis: {"absMm": -1} for axis in ("lateral", "longitudinal")}
        for x, y, h in itertools.product(*[(box[j], box[j + 3]) for j in range(3)]):
            ay = (v * w)[:, None] + (alpha * x - w[:, None] ** 2 * y) / 1000
            ax = accel - (alpha * y + w[:, None] ** 2 * x) / 1000
            for key, offset in (
                ("lateral", y - h * ay / 9.80665),
                ("longitudinal", x - h * ax / 9.80665),
            ):
                ind = np.unravel_index(np.argmax(abs(offset)), offset.shape)
                val = float(offset[ind])
                if abs(val) > worst[key]["absMm"]:
                    worst[key] = {
                        "absMm": abs(val),
                        "signedMm": val,
                        "wallS": float(t[ind[0]]),
                        "cgMm": [x, y, h],
                        "accelerationEdge": ("preceding", "following")[ind[1]],
                    }
        worst["minimumSideFraction"] = 0.5 - worst["lateral"]["absMm"] / 580
        extremes[sign] = worst
    return extremes


def evaluate(parts, masses, raw, sources):
    box = cg_box(parts, masses, raw)
    signed = extrema(box, sources)
    lower = min(r["minimumSideFraction"] for r in signed.values())
    return {
        "residualCentroidBoxMm": raw,
        "cgOuterBoxMm": box,
        "staticMinimumSideFraction": 0.5 - max(abs(box[1]), abs(box[4])) / 580,
        "signed": signed,
        "minimumSideFraction": max(0, lower),
        "twoSideSupportBound": lower > 0,
    }


def independent_witness(parts, masses, residual, sources):
    """Choose one common mass per parcel and one consistent XYZ for each clock trial."""
    worst = None
    for m, ys, xs, zs in itertools.product(masses, (0, 1), (0, 1), (0, 1)):
        ids = [3 * xs, 1 + 3 * ys, 2 + 3 * zs]
        r = np.array(residual)[ids]
        known = 0.0
        moment = np.zeros(3)
        for part in parts.values():
            q = np.array(part["positionMm"])[ids]
            heavier = (q[1] > r[1]) if ys else (q[1] < r[1])
            kg = part["kg"][int(heavier)]
            known += kg
            moment += kg * q
        cg = ((m - known) * r + moment) / m
        result = extrema(np.r_[cg, cg], sources)
        which = min(result, key=lambda s: result[s]["minimumSideFraction"])
        value = result[which]["minimumSideFraction"]
        if worst is None or value < worst["minimumSideFraction"]:
            worst = {
                "minimumSideFraction": value,
                "signedClock": which,
                "massKg": m,
                "signedBoxCorner": [xs, ys, zs],
                "knownMassKg": known,
                "residualMassKg": m - known,
                "residualCenterMm": r.tolist(),
                "cgMm": cg.tolist(),
                "atWallS": result[which]["lateral"]["wallS"],
            }
    return worst


def run(assets):
    parts = components()
    masses = read(ROOT / "wheel-loads/contact.json")["mass"]["noCutCreditKg"][
        "emptyFieldKg"
    ]
    sources = traces(assets)
    cases = {}
    for name, ylo, yhi in (
        ("centered", 0, 0),
        ("twenty", -20, 20),
        ("ten", -10, 10),
        ("right-bias", -25, -25),
        ("left-bias", 25, 25),
    ):
        cases[name] = evaluate(parts, masses, [-60, ylo, 150, 60, yhi, 450], sources)
    cases["gross"] = evaluate(parts, masses, [-425, -425, 0, 425, 425, 1100], sources)
    treated = paired(pack_fill(parts, 12))
    for name, candidate, y in (
        ("density-control", pack_fill(parts, 24), 0),
        ("density-12", pack_fill(parts, 12), 0),
        ("paired-only", paired(parts), 10),
        ("paired-and-packed", treated, 10),
        ("paired-and-packed-adverse", treated, 30),
    ):
        cases[name] = evaluate(candidate, masses, [-60, -y, 150, 60, y, 450], sources)
    for name, p in (
        ("centered", parts),
        ("twenty", parts),
        ("paired-and-packed-adverse", treated),
    ):
        cases[name]["commonMassWitness"] = independent_witness(
            p, masses, cases[name]["residualCentroidBoxMm"], sources
        )
    endpoints = {
        str(m): evaluate(treated, [m, m], [-60, -10, 150, 60, 10, 450], sources)
        for m in masses
    }
    # The heavy end of the released bill includes seven possible inherited pieces and fourteen
    # through sets. Study a location for that *same* charge only; subtract it from the residual.
    stock = read(ROOT / "winchless-bill/stock.json")
    fully_retained = treated.copy()
    fastener = stock["throughSetUpperKg"] * stock["throughSetsPerSite"]
    for k, r in stock["retainedStock"].items():
        b = np.array(r["robotBoundsMm"])
        fully_retained["stock/" + k] = {
            "kg": [r["finishedStockUpperKg"]] * 2,
            "positionMm": (b + [-1, -1, -1, 1, 1, 1]).tolist(),
        }
        fully_retained["through-sets/" + k] = {
            "kg": [fastener] * 2,
            "positionMm": (b + [-65, -65, -65, 65, 65, 65]).tolist(),
        }
    added = sum(r["kg"][0] for k, r in fully_retained.items() if k not in treated)
    if abs(added - stock["stockAndFastenerUpperKg"]) > 1.0e-6:
        raise ValueError("Inherited material was counted twice")
    heavy_stock = evaluate(
        fully_retained, [masses[1]] * 2, [-60, -10, 150, 60, 10, 450], sources
    )
    heavy_stock["explicitlyPartitionedKg"] = added

    # Solve both boundaries independently; the worst sign can change with the residual offset.
    def minimum(y):
        return evaluate(treated, masses, [-60, y, 150, 60, y, 450], sources)[
            "minimumSideFraction"
        ]

    safe = []
    for direction in (-1, 1):
        left, right = 0, 75
        if minimum(0) < 0.44:
            break
        for _ in range(26):
            mid = (left + right) / 2
            if minimum(direction * mid) >= 0.44:
                left = mid
            else:
                right = mid
        safe.append(direction * left)
    return {
        "sourceClocks": {
            k: v["clockSha256"]
            for k, v in read(ROOT / "wheel-loads/opening.json")["signed"].items()
        },
        "fieldMassKg": masses,
        "parcelCount": len(parts),
        "registeredMassKg": [sum(r["kg"][i] for r in parts.values()) for i in range(2)],
        "residualMassOuterKg": [
            masses[0] - sum(r["kg"][1] for r in parts.values()),
            masses[1] - sum(r["kg"][0] for r in parts.values()),
        ],
        "cases": cases,
        "selectedEndpoints": endpoints,
        "allLegacyStockAtNamedSites": heavy_stock,
        "limitingResidualCentroidYmmAtSpecifiedXandZ": safe,
        "actualWheelLoadN": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    a = p.parse_args()
    write(HERE / "transfer.json", run(a.assets))
