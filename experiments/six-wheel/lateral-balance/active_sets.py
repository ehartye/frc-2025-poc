"""Optimize the unilateral six-wheel system over every feasible active contact set."""

from itertools import product
import numpy as np
from scipy.optimize import linprog


def solve(wheels, xy, gap, ratios, largest):
    names = list(wheels)
    coords = np.array([wheels[n]["contactCenterMm"][:2] for n in names])
    scale = np.max(abs(coords), axis=0)
    a = np.column_stack((np.ones(6), coords / scale))
    mid = np.array([n.startswith("center") for n in names])
    unit = np.eye(15)
    force = unit[9:]
    heave = np.column_stack((a, -np.eye(6), np.zeros((6, 6))))
    # Force unit = one sixth of total weight; displacement unit = uniform static compression.
    limits = [(None, None)] * 3 + [(-gap, gap)] * 6 + [(0, None)] * 6
    total = force.sum(axis=0)
    mx = coords[:, 0] @ force
    my = coords[:, 1] @ force
    base = [mx, -mx, my, -my]
    rhs = [6 * xy[0], 6 * xy[0], 6 * xy[1], 6 * xy[1]]
    if largest:
        for end in np.flatnonzero(~mid):
            center = next(
                i for i in np.flatnonzero(mid) if names[i][-1] == names[end][-1]
            )
            base.append(unit[3 + center] - unit[3 + end])
            rhs.append(0)
    normal = force[~mid].sum(axis=0)
    feasible = 0
    highest = -1
    witness = None
    sets = []
    for contacts in product((False, True), repeat=6):
        if sum(contacts) < 2:
            continue
        ub = base.copy()
        b = rhs.copy()
        eq = [total]
        val = [6]
        for i, active in enumerate(contacts):
            if active:
                ub.extend(
                    (ratios[0] * heave[i] - force[i], force[i] - ratios[1] * heave[i])
                )
                b.extend((0, 0))
            else:
                eq.append(force[i])
                val.append(0)
                ub.append(heave[i])
                b.append(0)
        solution = linprog(
            -normal, A_ub=ub, b_ub=b, A_eq=eq, b_eq=val, bounds=limits, method="highs"
        )
        if solution.status == 2:
            continue
        if not solution.success:
            raise ValueError(solution.message)
        feasible += 1
        sets.append(sum(contacts))
        x = solution.x
        fraction = -solution.fun / 6
        if fraction > highest:
            highest = fraction
            deflection = heave @ x
            stiffness = np.where(
                deflection > 1e-8, x[9:] / np.maximum(deflection, 1e-8), np.mean(ratios)
            )
            if np.max(abs(stiffness * np.maximum(deflection, 0) - x[9:])) > 1e-6:
                raise ValueError("Failed independent force reconstruction")
            witness = {
                "normalFractions": (x[9:] / 6).tolist(),
                "stiffnessRatios": stiffness.tolist(),
                "gaps": x[3:9].tolist(),
                "deflection": deflection.tolist(),
                "loadLineMm": (x[9:] @ coords / 6).tolist(),
            }
    if not feasible:
        raise ValueError("Unsupported load-line bounds")
    return {
        "wheelOrder": names,
        "radiusErrorPerReferenceCompression": gap,
        "centerLargestOnEachSide": largest,
        "independentRateRange": list(ratios),
        "loadLineBoundsMm": xy,
        "feasibleContactSets": feasible,
        "minimumActiveSetSize": min(sets),
        "maximumEndFraction": highest,
        "witness": witness,
    }


def cases():
    from transfer import HERE, ROOT, read

    w = read(ROOT / "selected-drive/wheels.json")["wheels"]
    signed = read(HERE / "transfer.json")["cases"]["paired-and-packed"]["signed"]
    xy = [
        max(s[k]["absMm"] for s in signed.values()) for k in ("longitudinal", "lateral")
    ]
    return {
        name: solve(w, xy, spread, rates, order)
        for name, order, rates, spread in (
            ("unsorted", False, (0.9, 1.1), 0.1524),
            ("centers-largest", True, (0.9, 1.1), 0.1524),
            ("wide-rates", True, (0.75, 1.25), 0.1524),
            ("double-height-error", True, (0.9, 1.1), 0.3048),
        )
    }


if __name__ == "__main__":
    from transfer import HERE, write

    write(
        HERE / "order.json",
        {
            "minimumReferenceCompressionMm": 1,
            "stockRadiusErrorMm": 0.1524,
            "cases": cases(),
            "installedRateNPerMm": None,
        },
    )
