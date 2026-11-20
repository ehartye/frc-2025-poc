"""Screen real 45° rear chassis facets, continuous module yaw and matching backed bumpers."""

import math
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Pnt, gp_Vec
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps


def volume(s):
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(s, p)
    return abs(p.Mass())


def prism(points, side, low, high):
    w = BRepBuilderAPI_MakePolygon()
    for x, y in points:
        w.Add(gp_Pnt(-x, side * y, low))
    w.Close()
    return BRepPrimAPI_MakePrism(
        BRepBuilderAPI_MakeFace(w.Wire()).Face(), gp_Vec(0, 0, high - low)
    ).Shape()


def facet(c, side):
    outer1, outer2 = np.array([350.0, 350.0 - c]), np.array([350.0 - c, 350.0])
    inner_plane = 700 - c - 20 * math.sqrt(2)
    inner1, inner2 = (
        np.array([330.0, inner_plane - 330]),
        np.array([inner_plane - 325, 325.0]),
    )

    def at(a, b, f):
        return a + (b - a) * f

    shell = prism([outer1, outer2, inner2, inner1], side, 50, 150)
    cavity = prism(
        [
            at(outer1, inner1, 0.1),
            at(outer2, inner2, 0.1),
            at(outer2, inner2, 0.9),
            at(outer1, inner1, 0.9),
        ],
        side,
        52,
        148,
    )
    return BRepAlgoAPI_Cut(shell, cavity).Shape()


def box_gap(a, b):
    return float(
        np.linalg.norm(
            np.maximum(0, np.maximum(np.array(a[:3]) - b[3:], np.array(b[:3]) - a[3:]))
        )
    )


def distance(a, b):
    d = BRepExtrema_DistShapeShape(a, b)
    return d.Value()
