"""Register external OAK-D Lite mechanics to the two camera mounts (millimetres)."""

import hashlib
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "camera-disconnects"))
import connector_fit as vendor

FIXED = np.array([-315.0, 0.0, 970.0])
ANGLE = math.radians(26)
FORWARD = np.array([math.cos(ANGLE), 0.0, -math.sin(ANGLE)])
UP = np.array([math.sin(ANGLE), 0.0, math.cos(ANGLE)])
LEFT = np.array([0.0, 1.0, 0.0])
ROOF_M = np.array(vendor.CAM)
ROOF_T = np.array([32.5, 41.0, 851.0])


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cylinders(shape):
    result = []
    faces = TopExp_Explorer(shape, TopAbs_FACE)
    while faces.More():
        face = TopoDS.Face(faces.Current())
        surface = BRepAdaptor_Surface(face)
        if surface.GetType() == GeomAbs_Cylinder:
            cylinder = surface.Cylinder()
            axis = cylinder.Axis()
            p = np.array(axis.Location().Coord())
            if abs(axis.Direction().Z()) > 0.9999:
                b = vendor.cad.bounds(face)
                result.append((p, cylinder.Radius(), b[2], b[5]))
        faces.Next()
    return result


def source(folder):
    bodies = list(vendor.solids(vendor.load(folder / "camera.stp")))
    if len(bodies) != 13:
        raise ValueError("Unexpected enclosure inventory")
    bounds = np.array([vendor.cad.bounds(s) for s in bodies])
    center = []
    stops = []
    for index in (3, 4):
        cyl = [r for r in cylinders(bodies[index]) if abs(r[1] - 2.25) < 0.001]
        if len(cyl) < 1:
            raise ValueError("Missing cylindrical mono lens reference")
        p, _, low, high = cyl[0]
        center.append([*p[:2], (low + high) / 2])
        rings = {
            (round(radius, 5), round(zlow, 5))
            for q, radius, zlow, zhigh in cylinders(bodies[0])
            if np.linalg.norm(q[:2] - p[:2]) < 0.002 and radius < 4
        }
        if len(rings) != 4:
            raise ValueError("Unexpected molded lens aperture")
        stops.append(sorted(rings))
    points = np.array(sorted(center))
    if abs(np.linalg.norm(points[1] - points[0]) - 75) > 0.001:
        raise ValueError("Supplier mono axes no longer match their nominal baseline")
    return (
        bodies,
        points,
        np.r_[bounds[:, :3].min(axis=0), bounds[:, 3:].max(axis=0)],
        stops,
    )


def pose(name, bound, center):
    if name == "roof":
        return ROOF_M, ROOF_T
    # Seat the outside hull on the actual open cradle floor and back wall.
    # Supplier +Z is rearward. The USB mouth exits along supplier -Y.
    roll = -1 if name == "fixed-usb-down" else 1
    matrix = np.column_stack((roll * LEFT, -roll * UP, -FORWARD))
    if name == "fixed-datum-up":
        return matrix, FIXED - matrix @ center
    floor_y = bound[4] if roll == 1 else bound[1]
    anchor = np.array([(bound[0] + bound[3]) / 2, floor_y, bound[5]])
    return matrix, FIXED - 23 * FORWARD - 15 * UP - matrix @ anchor


def mapped(transform, point):
    m, t = transform
    return m @ point + t


def registration(folder, cradle):
    bodies, eyes, bounds, stops = source(folder)
    board = list(vendor.solids(vendor.load(folder / "board.step")))
    port = vendor.cad.bounds(board[240])
    # Reuse the published board connector's mouth and its center in the enclosure frame.
    mouth = np.array([56.55, port[1], -3.22])
    data = {}
    cases = {}
    for name in ("roof", "fixed-usb-up", "fixed-usb-down", "fixed-datum-up"):
        transform = pose(name, bounds, eyes.mean(axis=0))
        m, t = transform
        case = [vendor.transform(b, m, t) for b in bodies]
        origin = mapped(transform, eyes.mean(axis=0))
        axis = -m[:, 2]
        ports = mapped(transform, mouth)
        lead = -m[:, 1]
        gauge = vendor.cad.brep(["cyl", [*ports, *lead, 3.15, 30.0]])
        olds = np.array([48.0, 0.0, 842.0]) if name == "roof" else FIXED
        row = {
            "lensCylinderMidpointsMm": sorted(
                [mapped(transform, p).tolist() for p in eyes], key=lambda p: -p[1]
            ),
            "assumedRectifiedOriginMm": origin.tolist(),
            "oldOriginMm": olds.tolist(),
            "originDeltaMm": (origin - olds).tolist(),
            "outwardAxis": axis.tolist(),
            "usbMouthMm": ports.tolist(),
            "usbMouthDirection": lead.tolist(),
        }
        if name != "roof":
            row["cradleCaseOverlapMm3"] = sum(
                vendor.cad.intersection(b, cradle) for b in case
            )
            row["usbCorridorCradleOverlapMm3"] = vendor.cad.intersection(gauge, cradle)
        cases[name] = (case, origin)
        data[name] = row
    return cases, {
        "enclosureSha256": digest(folder / "camera.stp"),
        "pcbaSha256": digest(folder / "board.step"),
        "enclosureSolids": len(bodies),
        "pcbaSolids": len(board),
        "nativeCaseBoundsMm": bounds.tolist(),
        "lensCylinderMidpointsLocalMm": eyes.tolist(),
        "nativeMonoApertureRadiusAndForwardZMm": stops,
        "registrations": data,
        "measuredOpticalPupils": None,
    }
