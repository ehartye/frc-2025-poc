"""Test a fully retracted bore pin against a finite annular pipe in stage coordinates, mm."""

import math
import numpy as np

INNER, OUTER, HALF = 50.8, 57.15, 150.8125
PIN_R = 4.0


def clearance(center, axis, x, side):
    # Bound the complete pin disk by its extent along the pipe's axis, then
    # minimize/maximize radial distance over just the admissible pin interval.
    base = np.array([x, side * 161.0, 579.0]) - center
    dy = np.array([0.0, float(side), 0.0])
    dot = float(base @ axis)
    slope = float(dy @ axis)
    cap = HALF + PIN_R * math.hypot(axis[0], axis[2])
    if abs(slope) < 1e-8:
        if abs(dot) > cap:
            return abs(dot) - cap
        lo, hi = 0.0, 60.0
    else:
        bounds = sorted(((-cap - dot) / slope, (cap - dot) / slope))
        lo, hi = max(0.0, bounds[0]), min(60.0, bounds[1])
        if lo > hi:
            return (lo - hi) * abs(slope)
    proj = base - dot * axis
    direction = dy - slope * axis
    diag = float(direction @ direction)
    critical = lo if diag < 1e-12 else np.clip(-float(proj @ direction) / diag, lo, hi)
    low = float(np.linalg.norm(proj + critical * direction))
    high = max(
        np.linalg.norm(proj + lo * direction), np.linalg.norm(proj + hi * direction)
    )
    return max(low - PIN_R - OUTER, INNER - high - PIN_R)


def exact(center, axis, x, side):
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape

    start = center - axis * HALF
    pipe_axis = gp_Ax2(gp_Pnt(*start), gp_Dir(*axis))
    tube = BRepAlgoAPI_Cut(
        BRepPrimAPI_MakeCylinder(pipe_axis, OUTER, 2 * HALF).Shape(),
        BRepPrimAPI_MakeCylinder(pipe_axis, INNER, 2 * HALF).Shape(),
    ).Shape()
    pin_axis = gp_Ax2(gp_Pnt(x, side * 161.0, 579.0), gp_Dir(0.0, float(side), 0.0))
    pin = BRepPrimAPI_MakeCylinder(pin_axis, PIN_R, 60.0).Shape()
    distance = BRepExtrema_DistShapeShape(tube, pin)
    if not distance.IsDone():
        raise ValueError("Failed pin/pipe distance")
    return distance.Value()


def minimum(center, axis):
    bound = math.inf
    for x in (39.0, 110.0):
        for side in (-1, 1):
            distance = clearance(center, axis, x, side)
            if distance < 0.1:
                distance = exact(center, axis, x, side)
            bound = min(bound, distance)
    return bound
