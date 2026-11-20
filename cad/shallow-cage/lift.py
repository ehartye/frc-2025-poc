"""Quasi-static shallow-cage lift on a pivoting cage with a power-off hold."""

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
G = 9.80665
HEIGHT = 1508.064062
PLATE = 787.4
EYE_TO_PLATE = (HEIGHT - PLATE) / 1000
MASS = 74.0  # conservative loaded drive-sizing sensitivity; actual rack must be empty
CG_BOUNDS = (
    (-0.060, 0.060),
    (-0.040, 0.040),
    (0.150, 0.450),
)  # metres in chassis coordinates
JACK_X = 0.300
SEAT = 319.4
LEAD = 0.008
GEARING = 16
SCREW_ETA = 0.32
GEAR_ETA = 0.75


def stroke_state(lift_mm, cg):
    # Assumed rigid clamp/cage; suspension turns about the truss eye. Ignore cage
    # deadweight for the larger free equilibrium tilt. Before liftoff the tires
    # constrain the rotation; a remaining floor reaction balances the residual.
    dx, dy, dz = cg[0] - JACK_X, cg[1], cg[2] + lift_mm * 0.001 - HEIGHT * 0.001
    r = math.hypot(dx, dy)
    nx, ny = -dx / r, -dy / r
    free = math.atan2(r, -dz)

    def height(x, y, z, angle):
        lateral = (x - JACK_X) * nx + y * ny
        return (
            HEIGHT * 0.001
            + lateral * math.sin(angle)
            + (z + lift_mm * 0.001 - HEIGHT * 0.001) * math.cos(angle)
        )

    def ground(angle):
        # Full square enclosing each wheel and all other bodies, Z>=0. Bound
        # over all 4 lower hull corners is more conservative than just the wheels.
        return min(
            height(x, y, 0, angle) for x in (-0.425, 0.425) for y in (-0.425, 0.425)
        )

    lo, hi = 0, free
    for _ in range(42):
        mid = (lo + hi) / 2
        if ground(mid) < 0:
            hi = mid
        else:
            lo = mid
    a = lo
    return {
        "tiltDeg": math.degrees(a),
        "clearanceMm": 1000 * ground(a),
        "unconstrained": ground(free) > 0.0001,
        "highestHullMm": 1000
        * max(height(x, y, 1.059, a) for x in (-0.425, 0.425) for y in (-0.425, 0.425)),
        "plateTorqueNm": MASS * G * EYE_TO_PLATE * math.sin(a)
        if ground(free) > 0.0001
        else None,
    }


def evaluate():
    import itertools

    variants = list(itertools.product(*CG_BOUNDS))
    states = [stroke_state(i * SEAT / 320, cg) for i in range(321) for cg in variants]
    end = [stroke_state(SEAT, cg) for cg in variants]
    torque = max(s["plateTorqueNm"] or 0 for s in states)
    normal = MASS * G
    # Durability challenge includes 1.5x gravity mass load and one jack bearing
    # 65% of the vertical reaction. NEO 550 catalog stall and free constants.
    tension = 1.5 * 0.65 * normal
    screw_torque = tension * LEAD / (2 * math.pi * SCREW_ETA)
    motor_torque = screw_torque / (GEARING * GEAR_ETA)
    phase_current = motor_torque / (0.97 / 100) + 1.4
    speed = 40.0
    rpm = speed / 1000 / LEAD * 60 * GEARING
    R = 0.12
    kE = (12 - 1.4 * R) / (11000 * 2 * math.pi / 60)
    volts = rpm * 2 * math.pi / 60 * kE + R * phase_current
    lower = (
        torque / 0.055
    )  # lower front foot + upper centered shoes form opposing couple
    duty = volts / 9.5
    return dict(
        cgBoundsM=CG_BOUNDS,
        massKg=MASS,
        stepsPerCorner=321,
        coneDeg=max(e["tiltDeg"] for e in states),
        finalFullHullCarpetClearanceLowerMm=min(e["clearanceMm"] for e in end),
        maximumHullHeightMm=max(e["highestHullMm"] for e in states),
        allEightSuspended=all(e["unconstrained"] for e in end),
        jackDesign=dict(
            gearReduction=GEARING,
            screwLeadMm=LEAD * 1000,
            screwEfficiency=SCREW_ETA,
            gearEfficiency=GEAR_ETA,
            loadN=tension,
            screwTorqueNm=screw_torque,
            phaseCurrentA=phase_current,
            motorRpm=rpm,
            requiredV=volts,
            conditionalTerminalV=9.5,
            driveInputAForBoth=2 * phase_current * duty / 0.94,
            minimumPowerOffHoldingTorquePerScrewNm=tension * LEAD / (2 * math.pi),
        ),
        contact=dict(
            maxMomentAtCageNm=torque,
            lowerFootCompressionN=lower,
            totalUpperShoeReactionN=normal + lower,
            upperOnlyMomentBoundNm=normal * 0.011,
            materialQualification="Unresolved cage-plate local bending, retention proof, screws, frame anti-torque joints and power-off pawl capacity.",
        ),
        clock=trials(),
        scope="Eight whole-robot CG envelope corners with quasi-static motion and nominal locked cage. Point values are conditional fixture evaluations. Unverified frame and field strength, approach pose, inspection weight and closed-loop sensor response keep match climb credit at zero.",
    )


def trials():
    # Origin is an independently supplied final-approach stance 650 mm away.
    service = [
        ("approach at 0.25 m/s", 2.6),
        ("register bore, rack-empty and stowed", 0.5),
        ("raise 328 mm at 60 mm/s", 328 / 60),
        ("spread 36 mm", 0.6),
        ("seat both upper shoes", 8.6 / 20),
        ("raise lower pads 13.584 mm", 0.6),
        ("lift 319.4 mm at 40 mm/s", 319.4 / 40),
        ("verify and catch both passive pawls", 0.5),
    ]
    result = {}
    for key, start, failed in [
        ("reserved", 130, None),
        ("late", 140, None),
        ("registration-loss", 130, 1),
        ("single-shoe", 130, 4),
        ("pawl-missing", 130, 7),
    ]:
        t = start
        events = []
        for i, (label, seconds) in enumerate(service):
            if t + seconds > 153:
                events.append([label, 153, "cutoff"])
                break
            t += seconds
            if i == failed:
                events.append([label, t, "inhibit"])
                break
            events.append([label, t, "complete"])
        held = len(events) == len(service) and events[-1][-1] == "complete"
        result[key] = dict(
            startWallS=start,
            events=events,
            passiveAssessmentWallS=156,
            fixtureShallowIfAllContactsAndHoldMet=6 if held else 0,
            matchScore=0,
        )
    return result


if __name__ == "__main__":
    (HERE / "lift.json").write_bytes((json.dumps(evaluate(), indent=2) + "\n").encode())
