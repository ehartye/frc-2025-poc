"""Cable lengths, screened 5 V drop and qualified purchase/installation limits."""

from pathlib import Path
from itertools import product
import sys
import numpy as np
from paths import routes, lengths, points, DATA, POWER, PI, T, B, host, fillet, O, F, L, U
from scene import read, write, ROOT, CAD, m
from hardware import solids
from camera_parts import box, cut
from harness_fit import nearest
from routing import data_feed

HERE = Path(__file__).resolve().parent


def run(folder):
    rr, board = routes()
    # The old 2 m comparison started at a board envelope, without an A tip.
    # Carry its independently bounded moving end and 701.5 mm carrier length unchanged.
    baseline = read(ROOT / "camera-disconnects/route-proof.json")
    old = sum(s[0] for s in data_feed(-317))
    new = lengths(rr["roof-host"])
    flex = np.array(baseline["data_tip_to_tip_mm"]) - old + new + 38 + 12
    jacket = lengths(rr["data"])
    # Published nominal total is measured from the A tip to the far side of the C boot.
    fixed = jacket + 35 + 12 + 34.5
    # Declare an installation allowance on each intact factory cable, separate from
    # the as-measured jacket witness. Never coil or cut a SuperSpeed cable to hide excess.
    # Voltage at maximum load uses the room-to-hot uplift carried in the preceding harness.
    copper = (lengths(rr["power"]) + 36) / 1000
    tail = (lengths(rr["pigtail"]) + 24.95 + 10) / 1000
    metal = copper * 0.0393585 / 1.90 + tail * 2 * 1.25 * 0.008286 * 92 ** (2 * (26 - 14) / 39)
    voltages = {}
    for key, low, high, connections in (
        ("selected", 5.16, 5.20, 0.050),
        ("prior-band", 5.13, 5.20, 0.055),
        ("worn", 5.16, 5.20, 0.100),
        ("overtrim", 5.20, 5.25, 0.050),
    ):
        low_result = low - 0.010 - 1.20 * (metal + connections)
        voltages[key] = {
            "loadedLowerV": low_result,
            "unloadedUpperV": high + 0.010,
            "withinWindow": low_result >= 5 and high + 0.010 <= 5.25,
        }
    raw, eyes, bounds, _ = m.source(folder)
    rot, offset = m.pose("fixed-datum-up", bounds, eyes.mean(0))
    rel = rot @ m.ROOF_M.T
    trans = offset - rel @ m.ROOF_T
    receipt = read(ROOT / "camera-disconnects/interfaces.json")
    datum = receipt["disconnects"]["data-J1"]["mouth_mm"][0] + 6.5
    housing = list(m.vendor.solids(m.vendor.load(folder / "adapter.stp")))
    trim = box(-14, -7, -5, 28, 14, 2.35)
    outside = {}
    for i in (3, 4):
        relieved = cut(housing[i], trim)
        roof = m.vendor.transform(relieved, m.vendor.ADAPTER, (datum, -15.55, 847.78))
        outside[str(i)] = m.vendor.transform(roof, rel, trans)
    own = solids()[0]
    mating = {}
    for k in ("usb-C-power-boot", "usb-C-datum-side-exit"):
        body = own[k]
        mating[k] = {
            "caseGapMm": min(nearest(body, s) for s in outside.values()),
            "nativeInterferenceMm3": sum(CAD.intersection(body, s) for s in outside.values()),
        }
    expected = receipt["disconnects"]["power-J3"]["mouth_mm"]
    moved = rel @ np.array(expected) + trans
    if np.linalg.norm(moved - POWER) > 0.002:
        raise ValueError("Power lead is off the supplier mating axis")
    # A direct vertical drop beside the right fixed eye clears the enclosure but
    # its centerline itself projects inside that camera, independent of an AABB.
    old_top = board - [0, 0, 22]
    bad = fillet([old_top, old_top - [0, 0, 32], [-338, -29, 875], [-338, -29, 310]], [36, 36])
    origin = O - 37.5 * L
    focal = 320 / np.tan(np.deg2rad(36.5))
    candidates = {}
    for key, r in (("straight-down", bad), ("selected-outboard", rr["power"])):
        ps = points(r)
        rel = ps - origin
        d = np.maximum(0, rel @ F)
        distance = np.linalg.norm(rel - d[:, None] * F, axis=1)
        i = int(np.argmin(distance))
        u = 320 - focal * (rel[i] @ L) / d[i]
        v = 240 - focal * (rel[i] @ U) / d[i]
        candidates[key] = {
            "axisDistanceMm": float(distance[i]),
            "pointMm": ps[i].tolist(),
            "centralRayDepthMm": float(d[i]),
            "witnessPx": [float(u), float(v)],
        }
    return {
        "sourceSha256": {str(k): m.digest(folder / k) for k in m.vendor.EXPECTED},
        "piYawDeg": -15,
        "fixedMouthsMm": {"J1": DATA.tolist(), "J3": POWER.tolist()},
        "nominalPiPortsMm": {
            "lower": host(-202, -200, 169).tolist(),
            "upper": host(-202, -200, 177).tolist(),
        },
        "piHoleXYmm": [
            (PI + x * T + y * B)[:2].tolist() for x, y in product((-39, 19), (-24.5, 24.5))
        ],
        "minRadiusMm": {k: min(r for _, _, r in p if r) for k, p in rr.items()},
        "routedJacketMm": {k: lengths(p) for k, p in rr.items()},
        "tipOrDrawingLengthMm": {
            "roof2m": flex.tolist(),
            "fixedCustom": fixed,
            "fixedStockMaximum": 1020,
            "roofStockInterval": [1970, 2030],
            "originalRoofFeedMinimumRadiusMm": 50,
        },
        "terminalMetalMm": {"powerPair": 1000 * copper, "26AWGOneWay": 1000 * tail},
        "estimatedHotMetalLoopOhm": metal,
        "voltageAt1p2A": voltages,
        "fitToRelievedAdapter": mating,
        "rightEyePowerRouteControl": candidates,
        "onRobotUSB3AndPower": None,
    }


if __name__ == "__main__":
    write(Path(sys.argv[2]), run(Path(sys.argv[1])))
