"""Map the two-plane optical survey into each independently registered robot mounting frame."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from mechanics import FORWARD, UP, LEFT

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "camera-datum-shop"))
import metrology


def fixture(head, stroke):
    doc = json.loads((HERE / "mount.json").read_bytes())["registrations"]
    row = doc["roof" if head == "roof" else "fixed-datum-up"]
    # Optical +X in the metrology fixture means image-up; +Y means robot-left;
    # +Z points back from target toward camera. The fixed frame uses its 26 degree mount.
    basis = np.eye(3) if head == "roof" else np.column_stack((UP, LEFT, -FORWARD))
    center = np.array(row["assumedRectifiedOriginMm"]) + (
        [stroke, 0, 0] if head == "roof" else [0, 0, 0]
    )
    datum = center - basis @ np.array([48.0, 0.0, 842.0])
    return basis, datum


def targets(head, stroke):
    basis, datum = fixture(head, stroke)
    return [
        [[float(i) for i in datum + basis @ np.r_[xy, z]] for xy in metrology.DOTS]
        for z in metrology.Z
    ]


def measure(head, stroke, record):
    basis, datum = fixture(head, stroke)
    if record.get("head") != head or record.get("stroke_mm") != stroke:
        raise ValueError("Camera head or slide location differs from survey")
    for observed in record["eyes"].values():
        for plane in observed["planes"]:
            points = np.asarray(plane["robot_dots_xyz_mm"], float)
            if points.shape != (5, 3) or not np.isfinite(points).all():
                raise ValueError("Need five surveyed robot-frame points per plane")
            local = (points - datum) @ basis
            nominal = float(np.mean(local[:, 2]))
            if (
                abs(local[:, 2] - nominal).max()
                > record["uncertainties"]["survey_z_mm"]
            ):
                raise ValueError("Target plane exceeds stated height uncertainty")
            plane["z_mm"] = nominal
            plane["surveyed_xy_mm"] = local[:, :2].tolist()
    result = metrology.evaluate(record)
    for item in result["observations"].values():
        item["pupil_robot_mm"] = (datum + basis @ np.array(item["pupil_mm"])).tolist()
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("head", choices=["fixed", "roof"])
    p.add_argument("stroke", type=float, choices=(0, 185, 370))
    p.add_argument("measurement", type=Path, nargs="?")
    a = p.parse_args()
    if a.measurement:
        result = measure(a.head, a.stroke, json.loads(a.measurement.read_bytes()))
    else:
        result = {
            "proposedTargetDotsRobotMm": targets(a.head, a.stroke),
            "physicalMeasurements": None,
        }
    print(json.dumps(result, indent=2, allow_nan=False))
