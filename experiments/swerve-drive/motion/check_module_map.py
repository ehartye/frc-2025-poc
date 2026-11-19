"""Translate four simulated signed rotor/absolute encoders into the portable swerve checker."""

import argparse
import json
import math
from pathlib import Path
import numpy as np
from kinematics import ORDER, EncoderMap, TAU, body_to_field, module_targets, wrapped

HERE = Path(__file__).resolve().parent


def exercise(raw_frames=False):
    module = json.loads((HERE.parent / "packaging/module.json").read_bytes())
    centers = np.array([[module["modules"][i][k] for k in ("xM", "yM")] for i in ORDER])
    sign = np.array([1, -1, 1, -1])
    steer = np.array([1, -1, -1, 1])
    zero = np.array([0, 1.5, -2.2, 2.8])
    coupling = sign.astype(float)
    mapping = EncoderMap(0.0762, 5.5, zero, steer, sign, coupling)
    modules = [
        dict(
            id=key,
            xM=x,
            yM=y,
            encoderAtForwardDeg=round(math.degrees(z), 8),
            driveSign=int(d),
        )
        for key, (x, y), z, d in zip(ORDER, centers, zero, sign)
    ]
    if raw_frames:
        for m, direction, coupled in zip(modules, steer, coupling):
            m.update(
                steerSign=int(direction), couplingMotorTurnsPerSteerTurn=float(coupled)
            )
    cases = []
    for label, twist, pivot, g, turn_rate in [
        ("Diagonal translation while turning", (0.5, 0.35, 0.25), (0.0, 0.0), 1.0, 1.1),
        ("Front left pivot", (0.0, 0.0, 1.4), centers[0], -0.7, -0.9),
        ("Sideways with reversed wheel", (0.0, -0.75, 0.0), (0.0, 0.0), 0.0, 0.0),
    ]:
        w, a = module_targets(
            twist, centers, np.array([0.4, 1.0, -0.6, -2.0]), 2.0, pivot
        )
        # Two raw shaft samples give speed; endpoint absolute is rounded to 10 bits.
        step = 0.02
        rotor0, absolute0 = mapping.raw(np.zeros(4), a - turn_rate * step, 1024)
        rotor1, absolute = mapping.raw(w * step, a, 1024)
        case = dict(
            name=label,
            fieldVelocityMps=body_to_field(twist[:2], g).tolist(),
            yawRateRadPerSec=twist[2],
            gyroDeg=math.degrees(g),
            pivotM=list(pivot),
        )
        if raw_frames:
            case["frames"] = [
                dict(
                    timeS=time_s,
                    readings=[
                        [key, float(count) * 360 / 1024, float(turns)]
                        for key, count, turns in zip(ORDER, phases, rotors)
                    ],
                )
                for time_s, phases, rotors in (
                    (0.0, absolute0, rotor0),
                    (step, absolute, rotor1),
                )
            ]
        else:
            angle = mapping.angle(absolute, 1024)
            raw_rpm = (rotor1 - rotor0) * 60 / step
            corrected = raw_rpm - sign * coupling * turn_rate * 60 / TAU
            case["samples"] = [
                [name, math.degrees(wrapped(q) + z), rpm]
                for name, q, z, rpm in zip(ORDER, angle, zero, corrected)
            ]
        cases.append(case)
    record = dict(
        schema="swerve-encoder-replay/v1",
        basis="Project native module centers, 5.50 overall gearing and 76.2 mm wheel. Simulated two-sample raw rotor turns, absolute quantization and removable sensor calibration. Both polarities and rotor-to-azimuth coupling are normalized before the portable vector check.",
        wheelDiameterM=0.0762,
        motorTurnsPerPulleyTurn=5.5,
        pulleyTurnsPerWheelTurn=1.0,
        velocityToleranceMps=0.003,
        modules=modules,
        cases=cases,
    )

    if raw_frames:
        record.update(
            schema="swerve-encoder-replay/v2",
            basis="Project module centers and gearing. Two synchronized simulated raw rotor and quantized absolute samples per module; encoder polarity and carrier coupling are applied by the checker.",
            maxSteerRateDegPerS=300,
            phaseResolutionDeg=360 / 1024,
        )
    return record


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out", type=Path)
    p.add_argument("--wrong-polarity", action="store_true")
    p.add_argument(
        "--raw",
        action="store_true",
        help="Export synchronized uncorrected frames for the v2 checker",
    )
    args = p.parse_args()
    result = exercise(raw_frames=args.raw)
    if args.wrong_polarity:
        result["modules"][1]["driveSign"] *= -1
    args.out.write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n"
    )
