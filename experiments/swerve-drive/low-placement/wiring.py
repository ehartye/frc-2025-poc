"""Check the full moving jackets and each revised installed support against the fixed jack."""

from pathlib import Path
import hashlib
import json
import math
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "experiments/six-wheel/camera-disconnects"))
from routes import derivative_bound, values

sys.path.insert(0, str(ROOT / "cad/assembly-engineering"))
from attachments import base, common, cyl, dist
from cold_issue import precise
from installed_audit import box_gap
from clearance import move
from OCP.gp import gp_Pnt
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def body_sweep(poly, kind, limit, radius, motor, connector):
    n = 4096
    end = 1.0 if kind == "data" else 0.978
    t = np.linspace(0, end, n + 1)
    points = values(poly, t)
    cx, cy = (motor[:2] + motor[3:5]) / 2
    rad = (motor[3] - motor[0]) / 2
    inflation = derivative_bound(poly, 1) * end / (2 * n)

    def gap(stroke, include_sweep):
        offset = np.clip(cx - points[:, 0], 0, stroke) if include_sweep else stroke
        x = points[:, 0] + offset
        radial = np.hypot(x - cx, points[:, 1] - cy)
        dz = np.maximum(0, np.maximum(motor[2] - points[:, 2], points[:, 2] - motor[5]))
        return np.hypot(np.maximum(0, radial - rad), dz) - radius

    at_end = gap(limit, False)
    ix = int(at_end.argmin())
    q = points[ix] + [limit, 0, 0]
    within_motor = bool(
        motor[2] < q[2] < motor[5] and math.hypot(q[0] - cx, q[1] - cy) < rad
    )
    x = points[:, 0] + np.clip(connector[0] - points[:, 0], 0, limit)
    p = np.column_stack((x, points[:, 1:]))
    clearance = (
        np.linalg.norm(
            np.maximum(0, np.maximum(connector[:3] - p, p - connector[3:])), axis=1
        )
        - radius
    )
    first_contact = None
    for stroke in np.arange(0, limit + 0.001, 0.25):
        if gap(stroke, False).min() < -0.001:
            first_contact = float(stroke)
            break
    return dict(
        radiusMm=radius,
        curvePointErrorUpperMm=inflation,
        sweepMotorLowerMm=float(gap(limit, True).min() - inflation),
        sweepConnectorBoxLowerMm=float(clearance.min() - inflation),
        firstSampledInterferenceMm=first_contact,
        strokeEndMm=limit,
        endWitnessT=float(t[ix]),
        endWitnessXYZmm=q.tolist(),
        centerInsideMotor=within_motor,
    )


def verify_hardware(shape, bounds, obstacle, obs_bounds, limit):
    swept = bounds.copy()
    swept[3] += limit
    broad = box_gap(swept, obs_bounds)
    if broad > 0:
        return broad
    intervals = [(0.0, limit)]
    bound = math.inf
    while intervals:
        a, b = intervals.pop()
        mid = (a + b) / 2
        placed = move(shape, (mid, 0, 0))
        clearance = dist(placed, obstacle)
        conservative = clearance - (b - a) / 2
        if conservative > 0.05:
            bound = min(bound, conservative)
        elif clearance < 0.001 or b - a < 0.01:
            return -common(placed, obstacle)
        else:
            intervals.extend([(a, mid), (mid, b)])
    return bound


def run():
    proposal = read(HERE / "candidate-wiring.json")
    model = HERE / "candidate-power-and-data.step"
    fixed = HERE / "fixed-power-clearance-gauge.step"
    if (
        sha(model) != proposal["movingAssembly"]["output"]
        or sha(fixed) != proposal["stationaryPower"]["gaugeSha256"]
    ):
        raise ValueError("Wrong camera wiring models")
    reg = (
        ROOT / "cad/assembly-engineering/receiver-service/issue/engineering-reopen.json"
    )
    rows = [
        r for r in read(reg)["bodies"] if r["instance"] == "Right NEO550 REV-21-1651"
    ]
    body, connector = sorted(
        (np.asarray(r["boundsMm"]) for r in rows), key=lambda bb: bb[0]
    )
    # The selected public motor is a cylinder and the connector a box relieved by that barrel.
    motor = cyl(305, -321, 773.25, 17.5, 44.5)
    plug = BRepAlgoAPI_Cut(
        BRepPrimAPI_MakeBox(gp_Pnt(*connector[:3]), gp_Pnt(*connector[3:])).Shape(),
        cyl(305, -321, 799, 17.5, 25),
    ).Shape()
    for sh, bb in [(motor, body), (plug, connector)]:
        if not np.allclose(precise(sh), bb, atol=1e-6, rtol=0):
            raise ValueError("Jack datum differs from the reopened assembly")
    old = ROOT / "experiments/six-wheel/camera-disconnects/route-controls.json"
    result = {
        "nativeRegister": sha(reg),
        "originalTail": sha(old),
        "candidateKit": sha(model),
        "fixedPower": sha(fixed),
        "layouts": {},
    }
    for name, curves in [
        ("original", read(old)),
        ("revised", proposal["tailControlVerticesMm"]),
    ]:
        result["layouts"][name] = {
            stroke: {
                kind: body_sweep(
                    p,
                    kind,
                    stroke,
                    proposal["independentlyBoundMaxJacketRadiusMm"][kind],
                    body,
                    connector,
                )
                for kind, p in curves.items()
            }
            for stroke in [345, 370]
        }
    kit = base.named(model)
    if len(kit) != proposal["movingAssembly"]["articleCount"]:
        raise ValueError("Incomplete 21-body neutral")
    result["supportToJack345Mm"] = {
        k: {
            name: verify_hardware(shape, precise(shape), other, bb, 345)
            for name, other, bb in [
                ("motor", motor, body),
                ("connector", plug, connector),
            ]
        }
        for k, shape in kit.items()
        if not k.endswith("-jacket-gauge")
    }
    result["minimumSupportGapLowerMm"] = min(
        v for r in result["supportToJack345Mm"].values() for v in r.values()
    )
    (HERE / "wiring-clearance.json").write_bytes(
        (json.dumps(result, indent=2, allow_nan=False) + "\n").encode()
    )


if __name__ == "__main__":
    run()
