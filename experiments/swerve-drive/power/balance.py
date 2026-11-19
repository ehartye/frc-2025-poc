"""Rebuild four-contact CG and load requirements from physical parcels and gross mass."""

from itertools import product
import numpy as np
from electrical import ROOT, HERE, IDS, PACKAGE
from loads import G, load_bound, corner_loads, preloaded_bounds
from mass import read
from screen import write, sha


def centers():
    module = PACKAGE / "module.json"
    if module is None:
        raise ValueError("Missing project module installation")
    data = read(module)
    if data["order"] != list(IDS) or data["wheelDiameterM"] != 0.0762:
        raise ValueError("Changed corner registration")
    return np.array([[data["modules"][i]["xM"], data["modules"][i]["yM"]] for i in IDS])


def parcel_bounds(mass, xy, residual_box, loaded=False):
    known = read(ROOT / "wheel-loads/moments.json")["fixedParcelRows"]
    items = []
    for key, r in known.items():
        if key == "NP18-12 battery" or key.startswith("bumper-"):
            items.append((r["kg"], np.array(r["positionMm"]) / 1000))
    for x, y in xy:
        # Original supplier STEP spans -44.45..+88.646 in source X/Z and
        # rises 171.46 mm above the wheel ground datum. Widen to reserve stock.
        items.append(
            (
                [1.72 * 0.95, 1.72 * 1.05],
                np.array([x - 0.09, y - 0.09, 0.0, x + 0.09, y + 0.09, 0.18]),
            )
        )
    if loaded:
        items.append(([0.816466266] * 2, np.array([0.015, -0.155, 0.498, 0.136, 0.155, 0.617])))
    # At fixed total mass eliminate the residual mass, then extremize each
    # coefficient using an independent admitted endpoint of each physical parcel.
    if sum(v[0][1] for v in items) >= mass:
        raise ValueError("Nonnumeric residual budget")
    total = []
    for ax in range(3):
        bounds = []
        for high in (False, True):
            residual = residual_box[ax + 3 * high]
            moment = mass * residual
            for limits, pos in items:
                lever = pos[ax + 3 * high] - residual
                which = bool((lever >= 0) == high)
                moment += limits[int(which)] * lever
            bounds.append(float(moment / mass))
        total.append(bounds)
    return total


def make():
    xy = centers()
    m = read(HERE / "mass.json")
    # Distinguish robot inspection from battery+bumper field mass. A further
    # 2 kg scenario reserves physical integration weight without presuming legality.
    mass = m["projectedBareKg"][1] + 6.17 + 7.50 + 2.0
    pipe = 0.816466266
    boxes = {
        "centeredRemainder": [-0.06, -0.04, 0.15, 0.06, 0.04, 0.45],
        "broadRemainder": [-0.425, -0.425, 0, 0.425, 0.425, 1.1],
    }
    cg = {k: parcel_bounds(mass + pipe, xy, b, True) for k, b in boxes.items()}
    cases = {}
    for key, acc, omega, alpha, height, kerr in (
        ("standing", [0, 0], 0, 0, 0.00025, 0.10),
        ("forward", [2, 0], 0, 0, 0.00025, 0.10),
        ("diagonal", [2, 1], 0.7, 1.0, 0.00025, 0.10),
        ("unevenDiagonal", [2, 1], 0.7, 1.0, 0.001, 0.20),
        ("broadStanding", [0, 0], 0, 0, 0.00025, 0.10),
    ):
        c = cg["broadRemainder" if key == "broadStanding" else "centeredRemainder"]
        lo, hi = load_bound(xy, mass + pipe, c, acc, omega, alpha, kerr, height)
        cases[key] = dict(
            bodyOriginAccelMps2=acc,
            yawRadS=omega,
            yawAccelRadS2=alpha,
            heightErrorMm=height * 1000,
            verticalRateNperM=150000.0,
            rateFraction=kerr,
            fourContactLowerN=lo.tolist(),
            fourContactUpperN=hi.tolist(),
            allFourGuaranteed=bool(min(lo) > 0),
        )
        pmin, pmax = preloaded_bounds(xy, mass + pipe, c, acc, omega, alpha, 0.22, kerr)
        cases[key]["after22pctStaticAcceptanceLowerN"] = pmin.tolist()
        cases[key]["after22pctStaticAcceptanceUpperN"] = pmax.tolist()
    # Concrete equilibrium counterexample: diagonally uneven floor at stow.
    h = np.array([0.001, -0.001, -0.001, 0.001])
    cg0 = np.mean(cg["centeredRemainder"], axis=1)
    tripod = corner_loads(xy, mass + pipe, cg0, [2, 1], 0.7, 1.0, np.full(4, 150000.0), h)
    return dict(
        order=IDS,
        centersM=xy.tolist(),
        historicalParcelSourceSha256=sha(ROOT / "wheel-loads/moments.json"),
        fieldMassAtBareUpperKg=mass - 2.0,
        fieldMassDesignUpperPlus2kgIntegrationKg=mass,
        ladenDesignCaseKg=mass + pipe,
        residualPositionsM=boxes,
        allBodyCGBoundsM=cg,
        contactCases=cases,
        unevenExample=dict(
            cgM=cg0.tolist(), signedHeightMm=(1000 * h).tolist(), normalN=tripod.tolist()
        ),
        physicalCG=None,
    )


if __name__ == "__main__":
    write(HERE / "balance.json", make())
