"""Check rigid translations, proper damper work, guide ordering and legacy CAD pin clearance."""

import numpy as np
from model import HERE, ROOT, read, build
from boundary import write


def run():
    nominal = read(HERE / "boundary.json")
    C = np.array(nominal["complianceMmPerN"])
    coords = np.array(nominal["positionsMm"]) / 1000
    pins = read(ROOT / "fixed-hoist/lock-transfer.json")
    gravity = pins["centeredReferenceDeadLoadN"]
    checks = {}
    for stages in (1, 2):
        m = build(1.0, 5e6, 1.0, stages)
        N = m["nm"]
        Q = m["basis"]
        n = len(Q)
        # A 1 mm whole-platform heave or rigid pitch moves both damper ends together.
        pure = []
        for base in (np.ones(4) * 0.001, 0.005 * coords[:, 0], -0.005 * coords[:, 1]):
            x = np.zeros(n)
            x[N : N + 4] = base
            z = np.linalg.solve(Q, x)
            pure.append(float(np.max(abs(m["damperRel"] @ z))))
        x = np.zeros(n)
        x[-2:] = 0.001
        z = np.linalg.solve(Q, x)
        local = m["damperRel"] @ z
        shape = m["linearStiffness"]
        M = m["linearMass"]
        resid = np.linalg.norm(
            shape @ Q - (M @ Q) * (m["omega"] ** 2), ord="fro"
        ) / np.linalg.norm(shape @ Q, ord="fro")
        if (
            max(pure) > 1e-11
            or not np.allclose(local, 0.001, atol=1e-11)
            or resid > 1e-8
        ):
            raise ValueError("Incorrect relative frame or modal decomposition")
        checks[str(stages)] = {
            "massIdentityError": m["modalClosure"],
            "relativeEigenResidual": float(resid),
            "heavePitchRollFalseStrokeM": pure,
            "actual1mmGuideTravelAtBothDampersM": local.tolist(),
            "lowestModeHz": float(m["omega"][0] / (2 * np.pi)),
            "transformedModes": n,
        }
    # Vary the location of the earlier 140.21 N modeled weight, using the same
    # four installed lands and the matrix before contact with either old pin.
    vals = []
    H = np.array([[1, -0.087, y, -0.087 * y] for y in [0.2225, -0.2225]]) @ m["nodal"]
    for front in (0.35, 0.5, 0.65):
        for left in (0.35, 0.5, 0.65):
            f = gravity * np.array(
                [
                    front * left,
                    front * (1 - left),
                    (1 - front) * left,
                    (1 - front) * (1 - left),
                ]
            )
            vals.append(H @ C @ f)
    result = {
        "mechanismAssumedGravityN": gravity,
        "modalChecks": checks,
        "staticPinDisplacementMmAt9WeightDistributions": np.asarray(vals).tolist(),
        "measuredLockPreloadN": None,
    }
    write(HERE / "basis-checks.json", result)


if __name__ == "__main__":
    run()
