"""Derive four vertical support compliances from the manufactured carriage brace pair."""

from pathlib import Path
import json
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "carriage-support"))
from reactions import E, P, section
from support import j, BOLTS


def write(p, data):
    p.write_text(json.dumps(data, indent=2) + "\n", encoding="utf8", newline="\n")


def run():
    fit = j.read(ROOT / "carriage-support/fit.json")
    step = ROOT / "carriage-support/carriage-support.step"
    if j.digest(step) != fit["stepSha256"]:
        raise ValueError("Different brace")
    parts = j.cad.read_named(step, [{"id": k} for k in fit["volumesMm3"]])
    brace = parts["support-L-ribbed-bracket"]
    root = np.array([-44.0, 210.0, 374.5])
    # Rigid bolt-group hub, unit vertical force at each bearing land, with eight
    # positive shear/normal centroids (four bolts for each side of the robot).
    K = np.zeros((6, 6))
    grip = 34.0
    area = 20.1
    for x, z in BOLTS:
        dx, dy, dz = np.array([x, 210, z]) - root
        W = np.array(
            [[1, 0, 0, 0, dz, -dy], [0, 1, 0, -dz, 0, dx], [0, 0, 1, dy, -dx, 0]]
        )
        K += (
            W.T
            @ np.diag([77000 * area / grip, 200000 * area / grip, 77000 * area / grip])
            @ W
        )
    local = P[[0, 2]] - root
    J = np.array([[0, 0, 1, dy, -dx, 0] for dx, dy, dz in local])
    hub = J @ np.linalg.inv(K) @ J.T
    # Normal bending energy only. The machined mast-contact band is a rigid
    # root on this boundary; unmodeled shearing, torsion, and slip only add compliance.
    arms = []
    for site, coords in ((P[0], range(-23, 157, 2)), (P[2], range(-86, -61, 2))):
        energy = []
        for x in coords:
            cut = section(brace, x)
            lever = site[0] - x
            energy.append(lever**2 * cut["inv"][1, 1] / E)
        arms.append(float(np.trapezoid(energy, x=list(coords))))
    side = hub + np.diag(arms)
    # Compliance is in mm/N; inverse converted to N/m. Mirrored side order is FL, FR, RL, RR.
    full = np.zeros((4, 4))
    for ids in ([0, 2], [1, 3]):
        full[np.ix_(ids, ids)] = side
    output = {
        "sourceStepSha256": fit["stepSha256"],
        "positionsMm": P.tolist(),
        "rootReferenceMm": root.tolist(),
        "materialE_MPa": E,
        "boltGripMm": grip,
        "armFlexureMmPerN": arms,
        "boltHubMmPerN": hub.tolist(),
        "complianceMmPerN": full.tolist(),
        "stiffnessNPerM": (1000 * np.linalg.inv(full)).tolist(),
        "physicalCalibration": None,
    }
    write(HERE / "boundary.json", output)


if __name__ == "__main__":
    run()
