"""Static unilateral six-wheel contact using the registered coordinates and declared compliance."""

import itertools
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
G = 9.80665


def read(p):
    return json.loads(p.read_bytes())


def contacts(wheels, mass_kg, load_xy_mm, spring_n_mm, gap_mm):
    """Solve vertical force, pitch and roll on level ground. Positive gap is a shorter wheel."""
    ids = list(wheels)
    p = np.array([wheels[n]["contactCenterMm"][:2] for n in ids])
    spans = np.max(np.abs(p), axis=0)
    A = np.column_stack((np.ones(len(ids)), p / spans))
    k = np.array([spring_n_mm[n] for n in ids], dtype=float)
    gap = np.array([gap_mm[n] for n in ids], dtype=float)
    target = mass_kg * G * np.array([1, *list(np.asarray(load_xy_mm) / spans)])
    if (
        not all(np.isfinite(v).all() for v in (k, gap, target))
        or (k <= 0).any()
        or mass_kg <= 0
    ):
        raise ValueError(
            "Finite positive mass and stiffness, and finite wheel gaps required"
        )
    for count in range(len(ids), 1, -1):
        for active in itertools.combinations(range(len(ids)), count):
            index = list(active)
            a = A[index]
            kk = k[index]
            q = np.linalg.lstsq(
                a.T @ (kk[:, None] * a), target + a.T @ (kk * gap[index]), rcond=None
            )[0]
            compression = A @ q - gap
            n = k * np.maximum(compression, 0)
            if np.max(np.abs(A.T @ n - target)) > 1e-7:
                continue
            if min(compression[index]) < -1e-7:
                continue
            weight = sum(n)
            front = sum(v for name, v in zip(ids, n) if name.startswith("front"))
            rear = sum(v for name, v in zip(ids, n) if name.startswith("rear"))
            left = sum(v for name, v in zip(ids, n) if name.endswith("L"))
            return {
                "wheelN": {key: float(v) for key, v in zip(ids, n)},
                "outerFraction": float((front + rear) / weight),
                "lightSideFraction": float(min(left, weight - left) / weight),
                "zeroLoad": [key for key, v in zip(ids, n) if v < 1e-5],
                "maxEquilibriumResidualN": float(max(abs(A.T @ n - target))),
            }
    raise ValueError("Applied load line is unsupported by these unilateral contacts")


def simulate():
    wheels = read(ROOT / "selected-drive/wheels.json")["wheels"]
    tread = {name: row["wheelBoundsMm"][2] for name, row in wheels.items()}
    if any(abs(v) > 1e-7 for v in tread.values()):
        raise ValueError("Different nominal wheel drop")
    bill = read(ROOT / "winchless-bill/straight-guide.json")
    inspection = read(ROOT / "design/electrical-inspection/inspection.json")
    gyro = read(ROOT / "conditioned-match/slow-power.json")["leftSmall"]["screened"]
    bumper = [5.9, 7.5]
    # Both removable bumper limits are the same end-specific allowances used by the shop bill.
    b = read(ROOT / "winchless-bill/original.json")
    if b["bumperKg"] != bumper or b["batteryKg"] != 6.17:
        raise ValueError("Changed field-mass basis")
    battery = b["batteryKg"]
    payload = gyro["onePipeKg"] - gyro["bodyKg"]
    phases = {}
    for shelf, row in bill.items():
        r = row["outcomes"]["allStocksAndFourteenSetsKg"]["estimatedR103Kg"]
        dressed = [r[i] + bumper[i] for i in (0, 1)]
        empty = [v + battery for v in dressed]
        loaded = [v + payload for v in empty]
        phases[shelf] = {
            "R103Kg": r,
            "R408Kg": dressed,
            "R408MarginUpperKg": inspection["R408LimitKg"] - dressed[1],
            "emptyFieldKg": empty,
            "loadedFieldKg": loaded,
            "oldReferenceEmptyMarginKg": gyro["bodyKg"] - empty[1],
            "oldReferenceLoadedMarginKg": gyro["onePipeKg"] - loaded[1],
        }
    width = (
        wheels["center-L"]["contactCenterMm"][1]
        - wheels["center-R"]["contactCenterMm"][1]
    )
    span = (
        wheels["front-L"]["contactCenterMm"][0] - wheels["rear-L"]["contactCenterMm"][0]
    )
    empty = phases["noCutCreditKg"]["emptyFieldKg"][1]
    # A stiffness chosen by *defining* 1 mm uniform static compression at this reference mass.
    # The ratio threshold below covers other compressions without assigning a vendor modulus.
    reference_k = empty * G / 6
    profile = {
        "equal-flat": ((0, 0), 1.0, 0.0),
        "issued-side-minimum": ((0, width * (0.5 - 0.44)), 1.0, 0.0),
        "lighter-side-outside": ((0, 40), 1.0, 0.0),
        "forward-offset": ((50, 0), 1.0, 0.0),
        "center-0.3048-short": ((0, 0), 1.0, 0.3048),
        "center-half-rate": ((0, 0), 0.5, 0.0),
        "center-0.3048-short-80pct-rate": ((0, 0), 0.8, 0.3048),
        "ends-only": ((0, 0), 1.0, 2.0),
    }
    results = {}
    for name, (xy, r, gap) in profile.items():
        k = {w: reference_k * (r if w.startswith("center") else 1.0) for w in wheels}
        h = {w: (gap if w.startswith("center") else 0.0) for w in wheels}
        results[name] = {
            **contacts(wheels, empty, xy, k, h),
            "loadLineMm": xy,
            "centerStiffnessRatio": r,
            "centerGapMm": gap,
        }
    return {
        "wheelBottomMm": tread,
        "trackMm": width,
        "outerWheelbaseMm": span,
        "springReferenceNPerMm": reference_k,
        "springReferenceCompressionMm": 1,
        "mass": phases,
        "referenceEmptyKg": gyro["bodyKg"],
        "referenceLoadedKg": gyro["onePipeKg"],
        "payloadKg": payload,
        "tireDiameterCalibrationFraction": 0.002,
        "independentRadiusGapSensitivityMm": 2
        * 0.002
        * wheels["center-L"]["outerRadiusMm"],
        "static44pctYmaxMm": width * (0.5 - 0.44),
        "static4521927pctYmaxMm": width * (0.5 - 0.4521927151096343),
        "centerGapMaxPerReferenceCompression": {
            str(rate): 1.125 - 0.75 / rate for rate in (0.5, 2 / 3, 0.8, 1.0, 1.2)
        },
        "rows": results,
        "qualifiedTireRateNPerMm": None,
        "selectedCenterOfGravityMm": None,
        "physicalWheelLoadsN": None,
    }


if __name__ == "__main__":
    (HERE / "contact.json").write_text(
        json.dumps(simulate(), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
