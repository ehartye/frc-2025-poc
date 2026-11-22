"""Small planar-polyhedron helpers on OCP for the preliminary package layout."""
import math
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut, BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing,
                                BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Transform)
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism, BRepPrimAPI_MakeCylinder
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.ShapeFix import ShapeFix_Solid
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.TopAbs import TopAbs_SOLID, TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
try:
    from OCP.TopTools import TopTools_ListOfShape
except ImportError:
    from OCP.collections import List_TopoDS_Shape as TopTools_ListOfShape
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pnt, gp_Vec, gp_Ax1, gp_Ax2, gp_Dir, gp_Trsf
from OCP.Bnd import Bnd_Box, Bnd_OBB
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.TopAbs import TopAbs_FACE


def box(lo, hi):
    return BRepPrimAPI_MakeBox(gp_Pnt(*lo), gp_Pnt(*hi)).Shape()


def aabb(shape):
    b = Bnd_Box(); BRepBndLib.AddOptimal_s(shape, b, False, False)
    return [*b.CornerMin().Coord(), *b.CornerMax().Coord()]


def volume(shape):
    p = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, p); return p.Mass()


def solids(shape):
    out = []; e = TopExp_Explorer(shape, TopAbs_SOLID)
    while e.More():
        out.append((getattr(TopoDS, 'Solid_s', None) or TopoDS.Solid)(e.Current())); e.Next()
    return out


def unify(shape):
    u = ShapeUpgrade_UnifySameDomain(shape, True, True, True); u.Build(); return u.Shape()


def _bop(kind, first, others):
    if not others:
        return first
    a = TopTools_ListOfShape(); a.Append(first)
    t = TopTools_ListOfShape()
    for s in others:
        t.Append(s)
    op = kind(); op.SetArguments(a); op.SetTools(t); op.SetRunParallel(True); op.SetFuzzyValue(1e-5); op.Build()
    if not op.IsDone():
        raise RuntimeError("Boolean failed")
    return unify(op.Shape())


def fuse(shapes):
    shapes = list(shapes)
    return _bop(BRepAlgoAPI_Fuse, shapes[0], shapes[1:])


def common(shape, tool):
    return _bop(BRepAlgoAPI_Common, shape, [tool])


def cut(shape, tools):
    tools = [t for t in tools if t is not None]
    return _bop(BRepAlgoAPI_Cut, shape, tools)


def distance(a, b):
    d = BRepExtrema_DistShapeShape(a, b); d.Perform()
    if not d.IsDone():
        raise RuntimeError("distance failed")
    return d.Value()


def valid(shape):
    return BRepCheck_Analyzer(shape).IsValid()


def polyface(points):
    poly = BRepBuilderAPI_MakePolygon()
    for p in points:
        poly.Add(gp_Pnt(*p))
    poly.Close()
    f = BRepBuilderAPI_MakeFace(poly.Wire(), True)
    return f.Face() if f.IsDone() else None


def _dedupe(poly):
    out = []
    for p in poly:
        if not out or math.dist(out[-1], p) > 1e-6:
            out.append(tuple(p))
    if len(out) > 1 and math.dist(out[0], out[-1]) <= 1e-6:
        out.pop()
    return out


def loft_solid(a, b):
    """Planar-faced solid between two corresponding planar polygons (extrusion or ruled loft)."""
    a = _dedupe(a); b = _dedupe(b)
    if len(a) != len(b):
        raise ValueError("section vertex counts differ")
    faces = [polyface(a), polyface(b)]
    n = len(a)
    for i in range(n):
        j = (i + 1) % n
        quad = [a[i], a[j], b[j], b[i]]
        f = polyface(quad)
        if f is None:
            faces += [polyface([a[i], a[j], b[j]]), polyface([a[i], b[j], b[i]])]
        else:
            faces.append(f)
    if any(f is None for f in faces):
        raise ValueError("non-planar cap")
    sew = BRepBuilderAPI_Sewing(1e-4)
    for f in faces:
        sew.Add(f)
    sew.Perform()
    shell = sew.SewedShape()
    e = TopExp_Explorer(shell, TopAbs_SHELL)
    sh = (getattr(TopoDS, 'Shell_s', None) or TopoDS.Shell)(e.Current())
    solid = BRepBuilderAPI_MakeSolid(sh).Solid()
    fix = ShapeFix_Solid(solid); fix.Perform()
    solid = solids(fix.Solid())[0]
    if volume(solid) < 0:
        solid = solid.Reversed()
    return solid


def prism_xy(poly, z0, z1):
    """Vertical prism of a planar XY polygon."""
    a = [(x, y, z0) for x, y in poly]; b = [(x, y, z1) for x, y in poly]
    return loft_solid(a, b)


def ngon_prism(center_xy, apothem, z0, z1, n=32, phase=0.0):
    r = apothem / math.cos(math.pi / n)
    cx, cy = center_xy
    poly = [(cx + r*math.cos(phase + 2*math.pi*k/n), cy + r*math.sin(phase + 2*math.pi*k/n)) for k in range(n)]
    return prism_xy(poly, z0, z1)


def ngon_prism_y(center_xz, apothem, y0, y1, n=16, phase=0.0):
    """Polygon prism along +Y (for the pipe)."""
    r = apothem / math.cos(math.pi / n)
    cx, cz = center_xz
    ring = [(cx + r*math.cos(phase + 2*math.pi*k/n), cz + r*math.sin(phase + 2*math.pi*k/n)) for k in range(n)]
    return loft_solid([(x, y0, z) for x, z in ring], [(x, y1, z) for x, z in ring])


def mesh(shape, deflection=0.2):
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.3, True)


def nodes(shape):
    mesh(shape)
    out = []
    e = TopExp_Explorer(shape, TopAbs_FACE)
    while e.More():
        face = (getattr(TopoDS, 'Face_s', None) or TopoDS.Face)(e.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            tr = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                out.append(tri.Node(i).Transformed(tr).Coord())
        e.Next()
    return out


def obb_box(shape, grow=0.0):
    """Oriented box from the mesh plus a chord allowance; falls back to the axis-aligned box if not smaller."""
    mesh(shape)
    o = Bnd_OBB(); BRepBndLib.AddOBB_s(shape, o, True, True, True)
    hs = [o.XHSize() + grow + 0.25, o.YHSize() + grow + 0.25, o.ZHSize() + grow + 0.25]
    lo, hi = aabb(shape)[:3], aabb(shape)[3:]
    avol = math.prod(h - l + 2*grow for l, h in zip(lo, hi))
    ovol = 8 * math.prod(hs)
    if ovol >= 0.85 * avol:
        return box([l - grow for l in lo], [h + grow for h in hi]), "aabb"
    c = o.Center(); X = gp_Dir(o.XDirection()); Y = gp_Dir(o.YDirection()); Z = X.Crossed(Y)
    corner = gp_Pnt(c.X(), c.Y(), c.Z())
    for d, h in ((X, hs[0]), (Y, hs[1]), (Z, hs[2])):
        corner.Translate(gp_Vec(d).Multiplied(-h))
    ax = gp_Ax2(corner, Z, X)
    return BRepPrimAPI_MakeBox(ax, 2*hs[0], 2*hs[1], 2*hs[2]).Shape(), "obb"


def rotated(shape, origin, axis, deg):
    t = gp_Trsf(); t.SetRotation(gp_Ax1(gp_Pnt(*origin), gp_Dir(*axis)), math.radians(deg))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def moved(shape, v):
    t = gp_Trsf(); t.SetTranslation(gp_Vec(*v))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def offset_polygon(poly, d):
    """Mitred outward offset of a counterclockwise convex polygon."""
    n = len(poly); lines = []
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1; L = math.hypot(dx, dy); nx, ny = dy / L, -dx / L
        lines.append(((x1 + nx*d, y1 + ny*d), (dx, dy)))
    out = []
    for i in range(n):
        (p, u), (q, v) = lines[i - 1], lines[i]
        den = u[0]*v[1] - u[1]*v[0]
        t = ((q[0] - p[0])*v[1] - (q[1] - p[1])*v[0]) / den
        out.append((p[0] + u[0]*t, p[1] + u[1]*t))
    return out


def cylinder(base, axis, radius, length):
    """Right circular cylinder from a base-centre point along a unit axis."""
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*base), gp_Dir(*axis)), radius, length).Shape()


def revolved(axis_xy, profile):
    """Solid of revolution about the vertical axis through axis_xy. profile: [(z0, z1, r), ...] contiguous slabs."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
    cx, cy = axis_xy
    pts = [(cx, cy, profile[0][0])]
    for z0, z1, r in profile:
        pts += [(cx + r, cy, z0), (cx + r, cy, z1)]
    pts.append((cx, cy, profile[-1][1]))
    clean = [pts[0]]
    for q in pts[1:]:
        if math.dist(q, clean[-1]) > 1e-9:
            clean.append(q)
    face = polyface(clean)
    solid = BRepPrimAPI_MakeRevol(face, gp_Ax1(gp_Pnt(cx, cy, 0.0), gp_Dir(0, 0, 1)), 2 * math.pi).Shape()
    return solids(solid)[0] if solids(solid) else solid
