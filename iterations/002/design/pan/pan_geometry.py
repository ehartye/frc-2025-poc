"""Generate geometry.json for the iteration-002 guarded pan, its upper support and the inlet truss (revision r3).

Offline only (no Fusion). It builds every fabricated part as an OCP solid from the same
section data the native recipe sketches. It holds the moving pan to the simulation basis
(pan-mechanics.json) with a mass-neutral mechanically joined cheek, and measures pre-build
clearances against the accepted taps-02 STEP from the soft stop to past stow.

r1 applied the coordinator decisions OD-1..OD-5 plus the M1, M7 and M9 closures. r3 answers the second
independent review: a 1/4 in root-moment angle bolted through both legs on a relocated bulkhead, waterjet
plate stacks for the socket posts and the stop lug, a soft stop bolted through a keeper under the X250
cradle, an opening-direction limit cam and switch for the -3.5 deg inhibit, and a -9..70.1 deg sweep.

Run with the CAD interpreter from the project root:
  python -I iterations/002/design/pan/pan_geometry.py PLUGIN TAPS02_STEP
"""
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Section
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_Transform
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Ax1, gp_Ax2, gp_Dir, gp_Pln, gp_Pnt, gp_Trsf, gp_Vec

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
ART = ROOT / "artifacts/red2-integrated"
REVISION = "002-pan-r3"
PIVOT = (205.0, 580.0)            # sim settings.pivot, mm (X, Z); axis parallel to Y
STOW, DUMP, SOFT = 70.0, -2.0, -5.0
DENSITY = {"6061-T6": 2.70e-6, "5052-H32": 2.68e-6}
TUBE, WALL = 19.05, 1.5875        # 3/4 in square x 1/16 in (matches the existing upper pan stringers)
CTUBE, CWALL = 25.4, 1.5875       # 1 in square x 1/16 in cradles (stop-stiffness driven, see structure.json)
STUBE, SWALL = 12.7, 1.5875       # 1/2 in square x 1/16 in knee struts
PLATE, ANGLE_T, SHEET = 6.35, 3.175, 1.5875
PAD_T, BACK_T, SHIM_T = 6.0, 3.0, 0.5          # stow stop pad, bonded backing, nominal shim stack
SOFT_PAD_T = 18.0                               # soft-stop pad (r3: 18 mm keeps the F3 compression near 0.3 strain at the same 250 N/mm)
SWEEP_LO, STOW_LIMIT = -9.0, 70.1               # Fusion limits: F3 reaches about -8.5 deg on the pads; stow peak overtravel is 70.082 deg
CUT_DEG = -3.5                                  # opening-direction drive inhibit (limit cam leading edge)
CAM_R0, CAM_R1, CAM_Y0, CAM_T = 20.0, 23.5, 200.0, 3.175   # limit cam base and lobe radii, port drive shaft, 1/8 in plate
SWITCH_PSI = -90.0                              # switch roller world angle about the pivot (directly below the axis)
P1_THIN, P2_FRACTION = 1.0, 0.2                 # r3 applied lightening: inlet guides 3 -> 2 mm, feed panels 2.0 -> 1.6 mm (contact faces kept)
PAD_LEVER = 100.0
LUG_DEPTH, LUG_Y0 = 5.0, 161.0 - 22.225         # stop lug behind the lip back face, |Y| 138.775..161 (7/8 in waterjet plate; 5 mm keeps k_lug >> 300 N/mm)
PAD_Y = 149.5
SOFT_U = 90.0                                   # soft-stop strike on the lower stringer underside, pan-local u (mm); clear of the socket-post swing (r 61.8)
STRINGER_IN, STRINGER_Z0 = 232.475, 470.475
CR_Z0 = STRINGER_Z0 + 1.5                       # cradles clear the truss-member top faces by 1.5 mm (M9)
CR_TOP = CR_Z0 + CTUBE
CLIP_STATIONS = (140.0, 190.0, 260.0, 330.0, 400.0, 470.0, 530.0)   # 190 = socket-post / bulkhead tab, no clip
WEB_X0 = (115, 160, 205, 250, 295, 340, 385, 430, 475, 520, 541)   # 001 web start X, each 4 mm wide
RIVET_G, FHCS10_G, FHCS6_G, SHCS10_G, NUT10_G = 0.8, 2.4, 0.9, 3.0, 1.2   # estimated installed masses (g)
CLIP_LEG_B_X, CLIP_LEG_B_Z = 205.0, (528.0, 545.0, 562.0)        # leg B screws into the post inner plate (post Z 520..678)
CLIP_LEG_A_Y, CLIP_LEG_A_Z = 144.0, (512.0, 539.5, 567.0)        # leg A bolts through the bulkhead
POST_STACK_SCREWS = ((214.0, 540.0), (205.0, 597.0), (205.0, 669.0))
BACK_LINE = (np.array([115.0, 578.0]), np.array([102.99, 689.82]))  # flat-pose back-face edge of the rear lip


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def rel(p):
    try:
        return Path(p).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return "external/" + Path(p).parent.name + "/" + Path(p).name


# ---------------------------------------------------------------- solids
def wire(points):
    poly = BRepBuilderAPI_MakePolygon()
    for p in points:
        poly.Add(gp_Pnt(*map(float, p)))
    poly.Close()
    return poly.Wire()


def prism(sections, void=None):
    a, b = (np.array(s, float) for s in sections)
    shift = b - a
    normal = np.cross(a[1] - a[0], a[2] - a[0]); normal /= np.linalg.norm(normal)
    pure = np.allclose(shift, shift[0]) and np.linalg.norm(np.cross(shift[0], normal)) < 1e-6
    if pure:
        face = BRepBuilderAPI_MakeFace(wire(a), True)
        if void:
            face.Add(TopoDS.Wire(wire(void[0]).Reversed()))
        return BRepPrimAPI_MakePrism(face.Face(), gp_Vec(*shift[0])).Shape(), "extrude"

    def loft(x, y):
        t = BRepOffsetAPI_ThruSections(True, True); t.AddWire(wire(x)); t.AddWire(wire(y)); t.Build()
        return t.Shape()
    solid = loft(a, b)
    if void:
        solid = BRepAlgoAPI_Cut(solid, loft(*void)).Shape()
    return solid, "loft"


def build(part):
    solid, method = prism(part["sections"], part.get("void"))
    for j in part.get("joins", []):
        solid = BRepAlgoAPI_Fuse(solid, prism(j["sections"], j.get("void"))[0]).Shape()
    for c in part.get("cuts", []):
        solid = BRepAlgoAPI_Cut(solid, prism(c["sections"], c.get("void"))[0]).Shape()
    for b in part.get("bores", []):
        tool = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*b["start"]), gp_Dir(*b["direction"])), b["radius"], b["depth"]).Shape()
        solid = BRepAlgoAPI_Cut(solid, tool).Shape()
    for h in part.get("hexes", []):
        r = h["flats"] / math.sqrt(3)
        pts = [[h["center"][0] + r * math.cos(math.radians(30 + 60 * i)), h["y0"], h["center"][1] + r * math.sin(math.radians(30 + 60 * i))] for i in range(6)]
        tool = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire(pts), True).Face(), gp_Vec(0, h["y1"] - h["y0"], 0)).Shape()
        solid = BRepAlgoAPI_Cut(solid, tool).Shape()
    return solid, method


def props(shape, density):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, g)
    c = g.CentreOfMass()
    axis = gp_Ax1(gp_Pnt(PIVOT[0], 0, PIVOT[1]), gp_Dir(0, 1, 0))
    return dict(volumeMm3=g.Mass(), massKg=g.Mass() * density, centreMm=[c.X(), c.Y(), c.Z()],
                inertiaAboutPanAxisKgM2=g.MomentOfInertia(axis) * density * 1e-6)


def bbox(shape):
    b = Bnd_Box(); BRepBndLib.AddOptimal_s(shape, b, False, False)
    return [*b.CornerMin().Coord(), *b.CornerMax().Coord()]


def box_gap(a, b):
    return math.sqrt(sum(max(a[i] - b[i + 3], b[i] - a[i + 3], 0) ** 2 for i in range(3)))


def pose(shape, theta_deg):
    t = gp_Trsf(); t.SetRotation(gp_Ax1(gp_Pnt(PIVOT[0], 0, PIVOT[1]), gp_Dir(0, -1, 0)), math.radians(theta_deg))
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def world(u, w, theta):
    c, s = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    return np.array([PIVOT[0] + u * c - w * s, PIVOT[1] + u * s + w * c])


def gap(a, b):
    d = BRepExtrema_DistShapeShape(a, b); d.Perform()
    if not d.IsDone():
        raise RuntimeError("distance failed")
    v, common = d.Value(), 0.0
    if v < 1e-6:
        g = GProp_GProps(); BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(a, b).Shape(), g); common = g.Mass()
    return v, common


def section_vertices(solid, z):
    sec = BRepAlgoAPI_Section(solid, gp_Pln(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)), True)
    pts, e = [], TopExp_Explorer(sec.Shape(), TopAbs_VERTEX)
    while e.More():
        p = BRep_Tool.Pnt_s(TopoDS.Vertex(e.Current())); pts.append((p.X(), p.Y()))
        e.Next()
    uniq = []
    for p in pts:
        if all(math.dist(p, q) > 1e-6 for q in uniq):
            uniq.append(p)
    return uniq


def inner_square(solid, z):
    """Inner loop of a hollow square tube cut by a horizontal plane: the four vertices nearest its centroid."""
    v = section_vertices(solid, z)
    if len(v) != 8:
        raise ValueError(f"expected 8 section vertices at z={z}, got {len(v)}")
    c = np.mean(v, axis=0)
    inner = sorted(v, key=lambda p: math.dist(p, c))[:4]
    inner.sort(key=lambda p: math.atan2(p[1] - c[1], p[0] - c[0]))
    return [[x, y, z] for x, y in inner]


# ---------------------------------------------------------------- section helpers
def ins(axis, c, uv):
    out = []
    for u, v in uv:
        p = [float(u), float(v)]; p.insert(axis, float(c)); out.append(p)
    return out


def rect(axis, c, u0, u1, v0, v1):
    return ins(axis, c, [(u0, v0), (u1, v0), (u1, v1), (u0, v1)])


def ext(axis, c0, c1, uv, void_uv=None):
    return dict(axis=axis, sections=[ins(axis, c, uv) for c in (c0, c1)],
                void=[ins(axis, c, void_uv) for c in (c0, c1)] if void_uv else None)


def sq(u0, v0, s):
    return [(u0, v0), (u0 + s, v0), (u0 + s, v0 + s), (u0, v0 + s)]


def tube_along(axis, c0, c1, u0, v0, s, wall):
    return ext(axis, c0, c1, sq(u0, v0, s), sq(u0 + wall, v0 + wall, s - 2 * wall))


def oblique_tube(top, bottom, s, wall, z_top, z_bot):
    """Square tube between two centreline points, cut by horizontal planes z_top / z_bot (lofted oblique prism)."""
    top, bottom = np.array(top, float), np.array(bottom, float)
    d = top - bottom; d /= np.linalg.norm(d)
    # tube faces: one pair normal to the horizontal component, one pair normal to the horizontal perpendicular
    h = np.array([d[0], d[1], 0.0]); hn = np.linalg.norm(h)
    h = h / hn if hn > 1e-9 else np.array([1.0, 0, 0])
    q = np.array([-h[1], h[0], 0.0])
    cosa = abs(d[2])
    def loop(z, half):
        t = (z - bottom[2]) / (top[2] - bottom[2]); c = bottom + t * (top - bottom)
        a, b = half / cosa, half
        return [list(c + sa * a * h + sb * b * q) for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return dict(axis=2, sections=[loop(z_bot, s / 2), loop(z_top, s / 2)],
                void=[loop(z_bot, s / 2 - wall), loop(z_top, s / 2 - wall)]), loop


def mirror(spec):
    """Mirror a prism spec through Y=0."""
    def m(sec):
        return [[x, -y, z] for x, y, z in sec]
    out = dict(spec)
    out["sections"] = [m(s) for s in spec["sections"]]
    out["void"] = [m(s) for s in spec["void"]] if spec.get("void") else None
    for k in ("joins", "cuts"):
        if spec.get(k):
            out[k] = [mirror(j) for j in spec[k]]
    if spec.get("bores"):
        out["bores"] = [dict(b, start=[b["start"][0], -b["start"][1] - b["depth"] * b["direction"][1], b["start"][2]]) for b in spec["bores"]]
    return out


# ---------------------------------------------------------------- pan stop geometry (M1)
def stop_geometry():
    lo, hi = BACK_LINE
    d = (hi - lo) / np.linalg.norm(hi - lo)
    r0 = lo - np.array(PIVOT)
    s = PAD_LEVER - float(r0 @ d)
    n_flat = np.array([-d[1], d[0]])
    if n_flat[0] > 0:
        n_flat = -n_flat
    lip_point = lo + s * d
    lug_point = lip_point + LUG_DEPTH * n_flat
    u, w = lug_point - np.array(PIVOT)
    contact = world(u, w, STOW)
    th = math.radians(STOW)
    n_out = np.array([n_flat[0] * math.cos(th) - n_flat[1] * math.sin(th), n_flat[0] * math.sin(th) + n_flat[1] * math.cos(th)])
    n_in = -n_out
    lever = abs((contact[0] - PIVOT[0]) * n_in[1] - (contact[1] - PIVOT[1]) * n_in[0])
    lip_contact = world(*(lip_point - np.array(PIVOT)), STOW)
    return dict(wallDirectionFlat=d.tolist(), rearNormalFlat=n_flat.tolist(), patchArcMm=s,
                localWallHeightMm=float(lip_point[1] - PIVOT[1]), lugRearFaceFlatMm=lug_point.tolist(),
                contactAtStowMm=contact.tolist(), contactYmm=PAD_Y, backFaceOutwardNormalAtStow=n_out.tolist(),
                padPushDirection=n_in.tolist(), leverMm=lever,
                r0ContactOnLipBackFaceMm=lip_contact.tolist(),
                studyContactMm=[78, 522], studyNormal=[-0.34, -0.94])


def soft_geometry():
    contact = world(SOFT_U, 0.0, SOFT)                     # lower-flange underside (pan-local w = 0) at the soft stop
    th = math.radians(SOFT)
    n_in = np.array([-math.sin(th), math.cos(th)])           # pad pushes the pan along +w
    return dict(contactAtSoftStopMm=contact.tolist(), padPushDirection=n_in.tolist(), leverMm=SOFT_U, thetaDeg=SOFT)


# ---------------------------------------------------------------- moving pan (OD-4, mass-neutral)
SKIN_T = 3.175                                  # 1/8 in 6061-T6 plate (router or waterjet, 2D); panels between webs and stringers
SKIN_OUT = 161.0 + SKIN_T
WEB_X = (150.0, 340.0, 500.0)                   # 3/4 x 3/4 x 1/8 in web tubes
POST_X0, POST_X1, POST_Z0, POST_Z1 = 190.0, 220.0, 520.0, 678.0
POST_PLATES = (("inner", 9.525, "3/8 in"), ("middle", 9.525, "3/8 in"), ("outer", 4.7625, "3/16 in"))   # waterjet stack, skin side outward
POST_WINDOW = (194.0, 216.0, 606.0, 660.0)     # lightening window in every plate (2D), clear of the tapped holes at Z 595-597 and 668-669
BULK_T = 4.7625                                 # 3/16 in pivot bulkhead
BULK_X0, BULK_Z0 = POST_X1, 500.0               # r3: aft face on the post forward edge so the clip leg lands on the post
CLIP_T, CLIP_LEG, CLIP_Z = 6.35, 25.4, (502.0, 577.0)   # 1 x 1 x 1/4 in angle, 75 mm tall


def ladder_specs(stop):
    """Port-side riveted sheet-and-tube cheek ladder (r2). Every spec is an axis-normal prism a high-school shop can cut."""
    d = np.array(stop["wallDirectionFlat"]); n = np.array(stop["rearNormalFlat"]); lo = BACK_LINE[0]
    outline = [(115.0, 578.0)]
    for c in CLIP_STATIONS:
        if c == 190.0:
            outline += [(POST_X0, 578.0), (POST_X0, CLIP_Z[0]), (POST_X1, CLIP_Z[0]), (POST_X1, 578.0)]
        else:
            outline += [(c - 12.7, 578.0), (c - 12.7, 565.3), (c + 12.7, 565.3), (c + 12.7, 578.0)]
    sa, sb, sc = 72.0, 112.0, 100.0
    La, Lb, Lc = lo + sa * d, lo + sb * d, lo + sc * d
    A5, B5, C5 = La + LUG_DEPTH * n, Lb + LUG_DEPTH * n, Lc + LUG_DEPTH * n
    # the skin's aft lobe is the 1/8 in layer of the stop-lug plate stack (r2 machined it into a multi-face block)
    outline += [(545.0, 578.0), (545.0, 700.0), (115.0, 700.0), (115.0, float(B5[1])), tuple(map(float, B5)), tuple(map(float, A5)), (115.0, float(A5[1]))]
    specs = {}
    skin = ext(1, 161.0, SKIN_OUT, outline)
    skin.update(bores=[dict(start=[205, 160, 580], direction=[0, 1, 0], radius=8, depth=7)], hexes=[])
    specs["skin"] = (skin, "1/8 in 6061-T6 plate (ASTM B209), 2D router/waterjet")
    t = 1.5875
    specs["upper stringer"] = (tube_along(0, 115.0, 545.0, SKIN_OUT, 700.0 - TUBE, TUBE, t), "3/4 x 3/4 x 1/16 in 6061-T6 tube, 430 mm")
    specs["lower stringer rear"] = (tube_along(0, 115.0, POST_X0, SKIN_OUT, 580.0, TUBE, t), "3/4 x 3/4 x 1/16 in 6061-T6 tube, 75 mm")
    specs["lower stringer front"] = (tube_along(0, POST_X1, 545.0, SKIN_OUT, 580.0, TUBE, t), "3/4 x 3/4 x 1/16 in 6061-T6 tube, 325 mm")
    for x in WEB_X:
        specs[f"web {x:g}"] = (tube_along(2, 580.0 + TUBE, 700.0 - TUBE, x - TUBE / 2, SKIN_OUT, TUBE, ANGLE_T), "3/4 x 3/4 x 1/8 in 6061-T6 tube, 81.9 mm")
    inner = TUBE - 2 * t
    xs0 = PIVOT[0] + SOFT_U - 10.0
    specs["strike plug"] = (ext(0, xs0, xs0 + 20.0, sq(SKIN_OUT + t, 580.0 + t, inner)), "5/8 in 6061-T6 square bar, 20 mm (soft-stop strike backing)")
    specs["lower stringer rear end plug"] = (ext(0, POST_X0 - 12.7, POST_X0, sq(SKIN_OUT + t, 580.0 + t, inner)), "5/8 in 6061-T6 square bar, 12.7 mm")
    specs["lower stringer front end plug"] = (ext(0, POST_X1, POST_X1 + 12.7, sq(SKIN_OUT + t, 580.0 + t, inner)), "5/8 in 6061-T6 square bar, 12.7 mm")
    # stop lug (r3): a waterjet plate stack. The 7/8 in striking plate sits inside the skin, the skin lobe is the middle
    # layer and a 1/2 in outboard plate backs it; 2 x #10-32 screws and 2 dowels tie the stack. Striking face 5 mm behind the lip.
    specs["stop lug plate"] = (ext(1, LUG_Y0, 161.0, [tuple(map(float, q)) for q in (La, Lb, B5, A5)]), "7/8 in 6061-T6 plate, 2D waterjet profile")
    specs["stop lug outboard plate"] = (ext(1, SKIN_OUT, SKIN_OUT + 12.7, [tuple(map(float, A5)), tuple(map(float, C5)), (138.0, float(C5[1])), (138.0, float(A5[1]))]),
                                        "1/2 in 6061-T6 plate, 2D waterjet profile")
    return specs


def floor_clip(c):
    return ext(0, c - 9.525, c + 9.525, [(161, 578), (148.3, 578), (148.3, 578 - SHEET), (161 - SHEET, 578 - SHEET), (161 - SHEET, 565.3), (161, 565.3)])


def root_clip(stop):
    d = np.array(stop["wallDirectionFlat"]); n = np.array(stop["rearNormalFlat"]); c = BACK_LINE[0]
    p3 = c + 12.7 * d; p4 = p3 + SHEET * n
    q0 = c + SHEET * n; s = (578 - SHEET - q0[1]) / d[1]; q = q0 + s * d
    uv = [(127.7, 578.0), tuple(c), tuple(p3), tuple(p4), tuple(q), (127.7, 578 - SHEET)]
    return ext(1, 96.0, 161.0, uv)


def moving_hardware(stop):
    """Off-axis joining hardware as point masses (g, mm) for the mass properties; on-axis items listed separately."""
    d = np.array(stop["wallDirectionFlat"]); lo = BACK_LINE[0]
    rows = []
    for sy in (1, -1):
        for c in CLIP_STATIONS:
            if c == 190.0:
                continue
            rows.append(("floor clip rivet (dome head, floor, outside the pipe footprint)", RIVET_G, (c, sy * 154.7, 577.2)))
            rows.append(("floor clip rivet (dome head, skin tab)", RIVET_G, (c, sy * 163.5, 571.6)))
        for y in (110.0, 147.0):
            rows.append(("root clip rivet (countersunk in dimpled floor)", RIVET_G, (121.4, sy * y, 577.2)))
            p = lo + 6.0 * d
            rows.append(("root clip rivet (countersunk in dimpled lip)", RIVET_G, (p[0], sy * y, p[1])))
        for s in (82.0, 102.0):
            p = lo + s * d
            rows.append(("#6-32 FHCS lip to lug block", FHCS6_G, (p[0], sy * PAD_Y, p[1])))
        for z in (655.0, 672.0):
            rows.append(("#10-32 FHCS skin and lug stack into the tapped outboard plate", FHCS10_G, (127.0, sy * 166.0, z)))
        for x, z in ((121.0, 664.0), (133.0, 659.0)):
            rows.append(("4 mm steel dowel, skin to lug block", 1.6, (x, sy * 168.0, z)))
        for x in np.linspace(125.0, 535.0, 11):
            rows.append(("skin to upper stringer rivet", RIVET_G, (float(x), sy * 166.0, 690.5)))
        for x in [150.0, 180.0] + list(np.linspace(240.0, 530.0, 8)):
            rows.append(("skin to lower stringer rivet", RIVET_G, (float(x), sy * 166.0, 589.5)))
        for x in WEB_X:
            for z in (615.0, 665.0):
                rows.append(("skin to web rivet", RIVET_G, (x, sy * 166.0, z)))
        for x in (195.0, 215.0):
            for z in (595.0, 668.0):
                rows.append(("#10-32 FHCS skin to socket post", FHCS10_G, (x, sy * 166.0, z)))
        for z in CLIP_LEG_B_Z:
            rows.append(("#10-32 SHCS bulkhead clip leg B through skin into the post inner plate", SHCS10_G, (CLIP_LEG_B_X, sy * 157.0, z)))
        for z in CLIP_LEG_A_Z:
            rows.append(("#10-32 SHCS + nylon-insert locknut, bulkhead clip leg A through the bulkhead", SHCS10_G + NUT10_G, (BULK_X0 - 1.0, sy * CLIP_LEG_A_Y, z)))
        for x, z in POST_STACK_SCREWS:
            rows.append(("#10-32 FHCS post stack, outer plate into the tapped inner plate", FHCS10_G, (x, sy * 183.0, z)))
        for x in (POST_X0, POST_X1):
            rows.append(("#10-32 SHCS post to lower stringer end plug", FHCS10_G, (x, sy * 175.3, 589.5)))
            rows.append(("end plug rivet", RIVET_G, (x - 6 if x == POST_X0 else x + 6, sy * 175.3, 599.0)))
        for x in (250.0, 260.0):
            rows.append(("strike plug rivet", RIVET_G, (x, sy * 175.3, 599.0)))
    on_axis = [("1/2 in hex thrust collar (starboard outboard)", 6.0), ("hex spring driver / retaining collar", 6.0), ("1/2 in hex collar retaining the port limit cam", 6.0),
               ("6655K17 shaft washer (outboard, moving half)", 2.0), ("closing spring moving half", 7.5), ("arbor sleeve", 1.0)]
    return rows, on_axis


def moving_parts(shop, stop):
    by_id = {i["id"]: i for i in shop["items"]}
    parts = []

    def keep(i, **kw):
        it = by_id[i]
        base = dict(id=i, group="Guarded pan", moving=True, material="5052-H32" if "5052" in it["material"] else "6061-T6", axis=it["axis"],
                    sections=it["sections"], void=it["void"], bores=it["bores"], hexes=it["hexes"], role="mechanism", build=True,
                    source="artifacts/red2-integrated/pan-shop-plan.json#" + i, status="retained-from-001")
        base.update(kw); parts.append(base)
    keep("430 mm pan floor", status="retained-from-001 (carried-forward profile; riveted to clips with dome heads outside the pipe footprint and dimpled countersunk rivets under it)")
    for side in ("Port", "Starboard"):
        keep(f"{side} slotted rear lip", status="retained-from-001 (screwed to the stop lug plate stack, riveted to the dimpled root clip)")
    stub = by_id["Idler headed stub hex shank"]
    sec = stub["sections"]
    parts.append(dict(id="Idler headed stub hex shank", group="Guarded pan", moving=True, material="6061-T6", axis=1,
                      sections=[[[x, -229.0, z] for x, _, z in sec[0]], sec[1]], void=None, bores=[], hexes=[], role="mechanism", build=True,
                      source="pan-shop-plan.json#Idler headed stub hex shank, extended 27 mm outboard (OD-3)",
                      status="retained-from-001, lengthened to Y=-229 for the outboard thrust stack and torsion spring", stock="1/2 in hex 6061-T6"))
    specs = ladder_specs(stop)
    for side, sy in (("Port", 1), ("Starboard", -1)):
        for key, (spec, stock) in specs.items():
            parts.append(dict(id=f"{side} cheek {key}", **(spec if sy > 0 else mirror(spec)), group="Guarded pan", moving=True, material="6061-T6",
                              role="mechanism", build=True, stock=stock,
                              status="new 002 (r2 riveted sheet-and-tube cheek ladder; no welds, no monolith)",
                              source="pan-shop-plan.json cheek footprint; r2 shop redesign"))
        ya = SKIN_OUT
        wx0, wx1, wz0, wz1 = POST_WINDOW
        for label, t, stock in POST_PLATES:
            y0, y1 = sorted((sy * ya, sy * (ya + t)))
            cuts = [ext(1, y0, y1, [(wx0, wz0), (wx1, wz0), (wx1, wz1), (wx0, wz1)])]
            parts.append(dict(id=f"{side} socket post {label} plate", **ext(1, y0, y1, [(POST_X0, POST_Z0), (POST_X1, POST_Z0), (POST_X1, POST_Z1), (POST_X0, POST_Z1)]),
                              group="Guarded pan", moving=True, material="6061-T6", role="mechanism", build=True, cuts=cuts,
                              bores=[], hexes=[dict(y0=y0, y1=y1, center=[205, 580], flats=12.72)],
                              stock=f"{stock} 6061-T6 plate, 30 x 158 mm; 2D waterjet profile with the 1/2 in hex (12.72 mm across flats, fitted to a hex go gauge) and a lightening window",
                              status="new 002 (r3: waterjet plate stack replaces the r2 broached bar; the inner plate carries the tapped skin, clip and stack screws)",
                              source="pan-shop-plan.json#Port driven hex socket hex location; r3 load path"))
            ya += t
        for c in CLIP_STATIONS:
            if c == 190.0:
                continue
            spec = floor_clip(c)
            parts.append(dict(id=f"{side} floor clip {c:g}", **(spec if sy > 0 else mirror(spec)), group="Guarded pan", moving=True, material="6061-T6",
                              role="mechanism", build=True, bores=[], hexes=[], stock="1/2 x 1/2 x 1/16 in 6061-T6 angle, 19.05 mm",
                              status="new 002 (OD-4 floor joint)", source="new"))
        spec = root_clip(stop)
        parts.append(dict(id=f"{side} lip root clip", **(spec if sy > 0 else mirror(spec)), group="Guarded pan", moving=True, material="5052-H32",
                          role="mechanism", build=True, bores=[], hexes=[], stock="1/16 in 5052-H32 strip, bent 95.2 deg on a hand bending brake (registered route: fabrication-plan.json)",
                          status="new 002 (OD-4 lip-to-floor joint)", source="new"))
        xa = BULK_X0                                   # leg A on the bulkhead aft face, leg B on the skin over the post
        clip = ext(2, CLIP_Z[0], CLIP_Z[1], [(xa, 161.0 - CLIP_LEG), (xa, 161.0), (xa - CLIP_LEG, 161.0), (xa - CLIP_LEG, 161.0 - CLIP_T),
                                             (xa - CLIP_T, 161.0 - CLIP_T), (xa - CLIP_T, 161.0 - CLIP_LEG)])
        parts.append(dict(id=f"{side} bulkhead end clip", **(clip if sy > 0 else mirror(clip)), group="Guarded pan", moving=True, material="6061-T6",
                          role="mechanism", build=True, bores=[], hexes=[], stock="1 x 1 x 1/4 in 6061-T6 angle, 75 mm (saw cut, drilled)",
                          status="new 002 (r3 root-moment joint: 3 x #10-32 through leg A and the bulkhead, 3 x #10-32 through leg B and the skin into the post)", source="new"))
    bulk = ext(0, BULK_X0, BULK_X0 + BULK_T, [(-161.0, BULK_Z0), (161.0, BULK_Z0), (161.0, 578.0), (-161.0, 578.0)])
    bulk["cuts"] = [ext(0, BULK_X0, BULK_X0 + BULK_T, [(y0, BULK_Z0 + 12), (y1, BULK_Z0 + 12), (y1, 566.0), (y0, 566.0)]) for y0, y1 in ((-131.0, -10.0), (10.0, 131.0))]
    parts.append(dict(id="Pivot bulkhead", **bulk, group="Guarded pan", moving=True, material="6061-T6", role="mechanism", build=True, bores=[], hexes=[],
                      stock="3/16 in 6061-T6 plate, 322 x 78 mm, two windows (router or waterjet)",
                      status="new 002 (r3: moved to X 220-224.8 and deepened to Z 500 so the 1/4 in clip lands on the post; carries the cheek root moment across the pan)",
                      source="new"))
    # opening-direction limit cam on the port drive hex (r3, for the -3.5 deg inhibit): base circle plus one lobe
    psi = SWITCH_PSI
    a_on, a_off = psi - CUT_DEG, psi - (-14.0)                 # pan-local lobe edges: switch on the lobe for theta in [-14, -3.5]
    ramp = 1.0
    pts = []
    for k in range(360):
        a = -180.0 + k
        if a_on - ramp <= a <= a_on + ramp:
            r = CAM_R0 + (CAM_R1 - CAM_R0) * (a - (a_on - ramp)) / (2 * ramp)
        elif a_off - ramp <= a <= a_off + ramp:
            r = CAM_R1 - (CAM_R1 - CAM_R0) * (a - (a_off - ramp)) / (2 * ramp)
        elif a_on + ramp < a < a_off - ramp:
            r = CAM_R1
        else:
            r = CAM_R0
        pts.append((PIVOT[0] + r * math.cos(math.radians(a)), PIVOT[1] + r * math.sin(math.radians(a))))
    cam = ext(1, CAM_Y0, CAM_Y0 + CAM_T, pts)
    parts.append(dict(id="Port reverse-limit cam", **cam, group="Guarded pan", moving=True, material="6061-T6", role="mechanism", build=True, bores=[],
                      hexes=[dict(y0=CAM_Y0, y1=CAM_Y0 + CAM_T, center=[205, 580], flats=12.72)],
                      stock="1/8 in 6061-T6 plate, 2D waterjet profile with the 1/2 in hex", status="new 002 (r3 opening-direction inhibit: lobe leading edge at theta = -3.5 deg)",
                      source="new", camDeg=dict(switchWorldDeg=psi, lobeLocalDeg=[a_on, a_off], rampDeg=2 * ramp, baseRadiusMm=CAM_R0, lobeRadiusMm=CAM_R1)))
    return parts


def moving_properties(parts, hardware):
    m = mu = mw = inertia = 0.0
    for p in parts:
        s, _ = build(p); pr = props(s, DENSITY[p["material"]])
        cx, _, cz = pr["centreMm"]
        m += pr["massKg"]; mu += pr["massKg"] * (cx - PIVOT[0]); mw += pr["massKg"] * (cz - PIVOT[1]); inertia += pr["inertiaAboutPanAxisKgM2"]
    for _, g, (x, _, z) in hardware:
        k = g / 1000
        m += k; mu += k * (x - PIVOT[0]); mw += k * (z - PIVOT[1]); inertia += k * ((x - PIVOT[0]) ** 2 + (z - PIVOT[1]) ** 2) * 1e-6
    return np.array([m, mu, mw, inertia])


# ---------------------------------------------------------------- fixed parts
def fixed_parts(shop, stop, soft, chassis, ids):
    by_id = {i["id"]: i for i in shop["items"]}
    parts = []
    def add(i, spec, **kw):
        base = dict(id=i, group="Pan upper support", moving=False, material="6061-T6", role="structure", build=True,
                    bores=[], hexes=[], status="new 002", source="new")
        base.update(spec); base.update(kw); parts.append(base)
    # --- fixed inlet (001 surfaces unchanged; posts re-landed on the stringer extensions, OD-2)
    foot_top = STRINGER_Z0 + TUBE + 4.0
    panels = {i["id"]: i["sections"] for i in shop["items"] if "feed panel" in i["id"]}

    def panel_bottom_shift(x, z):
        """P2: raise a point that sits on a feed-panel bottom by 20 % of that panel's vertical thickness (2.0 -> 1.6 mm)."""
        for secs in panels.values():
            (a, b) = secs
            xa, xb = a[0][0], b[0][0]
            if not min(xa, xb) - 1e-6 <= x <= max(xa, xb) + 1e-6:
                continue
            f = (x - xa) / (xb - xa)
            za_bot, za_top = min(q[2] for q in a), max(q[2] for q in a)
            zb_bot, zb_top = min(q[2] for q in b), max(q[2] for q in b)
            zbot = za_bot + f * (zb_bot - za_bot); ztop = za_top + f * (zb_top - za_top)
            if abs(z - zbot) < 0.06:
                return P2_FRACTION * (ztop - zbot)
        return 0.0
    for item in shop["items"]:
        if item["moving"] or any(k in item["id"] for k in ("tower", "Gearbox reaction", "Six-bolt")):
            continue
        spec = dict(axis=item["axis"], sections=item["sections"], void=item["void"], bores=item["bores"], hexes=item["hexes"])
        light = None
        iid = item["id"]
        inward = lambda q, d=P1_THIN: [q[0], q[1] - d if q[1] > 0 else q[1] + d, q[2]]
        if "feed panel" in iid:                                   # P2: contact (top) face kept, bottom raised
            new = []
            for sec in spec["sections"]:
                zb = min(q[2] for q in sec); zt = max(q[2] for q in sec)
                new.append([[q[0], q[1], q[2] + P2_FRACTION * (zt - zb) if abs(q[2] - zb) < 1e-6 else q[2]] for q in sec])
            spec["sections"] = new; light = "P2: 2.0 -> 1.6 mm, top contact face kept"
        elif "inlet guide" in iid:                                # P1: contact (inner) face kept, outer face 1 mm inboard
            new = []
            for sec in spec["sections"]:
                outer = max(abs(q[1]) for q in sec)
                new.append([inward(q) if abs(abs(q[1]) - outer) < 1e-6 else q for q in sec])
            spec["sections"] = new; light = "P1: 3 -> 2 mm, inner contact face kept"
        elif "upper guide spine" in iid:                          # P1: spine follows the thinner guide
            spec["sections"] = [[inward(q) for q in sec] for sec in spec["sections"]]
            spec["void"] = [[inward(q) for q in sec] for sec in spec["void"]] if spec["void"] else None
            light = "P1: moved 1 mm inboard onto the 2 mm guide"
        elif "inlet post" in iid:                                 # P1: post tip follows the spine
            new = []
            for sec in spec["sections"]:
                tip = min(abs(q[1]) for q in sec)
                new.append([inward(q) if abs(abs(q[1]) - tip) < 1e-6 else q for q in sec])
            spec["sections"] = new; light = "P1: tip extended 1 mm inboard onto the moved spine"
        elif "chute crossmember" in iid:                          # P2: tops follow the raised panel bottoms
            spec["sections"] = [[[q[0], q[1], q[2] + panel_bottom_shift(q[0], q[2])] for q in sec] for sec in spec["sections"]]
            light = "P2: top raised onto the thinner panels"
        status = "retained-from-001"
        if "mounting foot" in item["id"]:
            spec["sections"] = [[[x, y, z + (STRINGER_Z0 + TUBE - 480.0)] for x, y, z in s] for s in item["sections"]]
            status = "retained-from-001 because it seats the retained post; raised 9.525 mm onto the OD-2 stringer extension top"
        elif "inlet post" in item["id"]:
            spec["sections"] = [clip_z(s, foot_top) for s in spec["sections"]]
            spec["void"] = [clip_z(s, foot_top) for s in item["void"]]
            status = "retained-from-001 because it holds the guide spines and inlet guides at the sim feed geometry; bottom trimmed to Z=%.3f to sit on the raised foot on the OD-2 stringer extension" % foot_top
        if light:
            status += "; r3 lightening " + light
        add(item["id"], spec, group="Fixed inlet", material="5052-H32" if "5052" in item["material"] else "6061-T6", role="guide",
            source="artifacts/red2-integrated/pan-shop-plan.json#" + item["id"], status=status, **({"lightening": light} if light else {}))
    # --- cradles (OD-1 A), end clips, carriers, gearbox plate, landing clips
    for label, xc in (("110", 110.0), ("250", 250.0)):
        add("Upper cradle " + label, tube_along(1, -STRINGER_IN, STRINGER_IN, xc - CTUBE / 2, CR_Z0, CTUBE, CWALL),
            stock="1 x 1 x 1/16 in wall 6061-T6 square tube, 464.95 mm", status="new 002 (OD-1 A)")
    for label, face, sign in (("110", 110.0 + CTUBE / 2, 1), ("250", 250.0 - CTUBE / 2, -1)):
        for side, sy in (("port", 1), ("starboard", -1)):
            a, b = face + sign * ANGLE_T, face + sign * TUBE
            yi, ya, yb = sy * (STRINGER_IN - TUBE), sy * STRINGER_IN, sy * (STRINGER_IN - ANGLE_T)
            add(f"Cradle {label} {side} end clip", ext(2, CR_Z0, STRINGER_Z0 + TUBE, [(face, yi), (a, yi), (a, yb), (b, yb), (b, ya), (face, ya)]),
                stock="3/4 x 3/4 x 1/8 in 6061-T6 angle, 17.55 mm")
    carrier = [(100.475, CR_TOP), (259.525, CR_TOP), (239.0, 605.0), (171.0, 605.0)]

    def windows(outline, y0, y1, ligament=10.0, z_top=556.0, boss=24.0):
        """Two lightening windows either side of a vertical bearing web, ligament-offset from the inclined edges (mass plan)."""
        (xl0, zb), (xr0, _), (xr1, zt), (xl1, _) = outline
        kl, kr = (xl1 - xl0) / (zt - zb), (xr1 - xr0) / (zt - zb)
        ol, orr = ligament * math.sqrt(1 + kl ** 2), ligament * math.sqrt(1 + kr ** 2)
        z0 = zb + ligament
        xl = lambda z: xl0 + kl * (z - zb) + ol
        xr = lambda z: xr0 + kr * (z - zb) - orr
        left = [(xl(z0), z0), (PIVOT[0] - boss, z0), (PIVOT[0] - boss, z_top), (xl(z_top), z_top)]
        right = [(PIVOT[0] + boss, z0), (xr(z0), z0), (xr(z_top), z_top), (PIVOT[0] + boss, z_top)]
        return [ext(1, y0, y1, left), ext(1, y0, y1, right)]
    for side, sy in (("Port", 1), ("Starboard", -1)):
        y0, y1 = sorted((sy * 191.85, sy * 198.15))
        add(side + " bearing carrier", ext(1, y0, y1, carrier), bores=[dict(start=[PIVOT[0], y0, PIVOT[1]], direction=[0, 1, 0], radius=14.2875, depth=PLATE)],
            cuts=windows(carrier, y0, y1), stock="1/4 in 6061-T6 plate, two lightening windows",
            status="new 002 (OD-1 A; replaces 001 tower, foot and knees)")
    gb = [(100.475, CR_TOP), (259.525, CR_TOP), (240.0, 606.0), (170.0, 606.0)]
    gbores = [dict(start=[205, 212, 580], direction=[0, 1, 0], radius=12.2, depth=4)]
    for k in range(6):
        a = math.radians(-30 + 60 * k)
        gbores.append(dict(start=[205 + 16 * math.cos(a), 212, 580 + 16 * math.sin(a)], direction=[0, 1, 0], radius=1.65, depth=4))
    add("Port gearbox plate", ext(1, 212.0, 216.0, gb), bores=gbores, cuts=windows(gb, 212.0, 216.0), stock="4 mm 6061-T6 plate, two lightening windows")
    for label, xc in (("110", 110.0), ("250", 250.0)):
        for name, yface, sy in (("Port carrier", 198.15, 1), ("Starboard carrier", -198.15, -1), ("Gearbox plate", 216.0, 1)):
            leg = 12.7
            uv = [(yface, CR_TOP), (yface + sy * leg, CR_TOP), (yface + sy * leg, CR_TOP + ANGLE_T), (yface + sy * ANGLE_T, CR_TOP + ANGLE_T),
                  (yface + sy * ANGLE_T, CR_TOP + leg), (yface, CR_TOP + leg)]
            add(f"{name} landing clip {label}", ext(0, xc - 9.525, xc + 9.525, uv), stock="1/2 x 1/2 x 1/8 in 6061-T6 angle, 19.05 mm")
    # --- starboard knee struts (axial load path; thrust is located on the idler side only)
    for xs in (160.0, 228.0):
        yt = -206.0
        for _ in range(30):
            alpha = math.atan2(abs(yt + 242.0), 560.0 - (STRINGER_Z0 + TUBE))
            hw = STUBE / 2 / math.cos(alpha)
            yt = -199.15 - hw
        spec, _ = oblique_tube((xs, yt, 560.0), (xs, -242.0, STRINGER_Z0 + TUBE), STUBE, SWALL, 560.0, STRINGER_Z0 + TUBE)
        add(f"Starboard knee strut {xs:g}", spec, stock="1/2 x 1/2 x 1/16 in 6061-T6 square tube", status="new 002 (OD-1 A axial path)")
        add(f"Starboard knee strut head clip {xs:g}", ext(0, xs - 9.525, xs + 9.525,
            [(-198.15, 560.0), (-217.2, 560.0), (-217.2, 560.0 + PLATE), (-198.15 - PLATE, 560.0 + PLATE), (-198.15 - PLATE, 579.05), (-198.15, 579.05)]),
            stock="3/4 x 3/4 x 1/4 in 6061-T6 angle, 19.05 mm (r2: 1/8 in leg overstressed)")
        for z0 in (560.0 - 12.7, STRINGER_Z0 + TUBE):
            ptop = inner_loop_oblique((xs, yt, 560.0), (xs, -242.0, STRINGER_Z0 + TUBE), STUBE / 2 - SWALL, z0, z0 + 12.7)
            add(f"Starboard knee strut plug {xs:g} {'top' if z0 > 520 else 'foot'}", ptop, stock="3/8 in 6061-T6 square bar, 12.7 mm")
    # --- stow stop (OD M1): brackets on the X110 cradle rear face, backings, at |Y| = PAD_Y under the cheek stop lugs
    c = np.array(stop["contactAtStowMm"]); n = np.array(stop["padPushDirection"]); t = np.array([n[1], -n[0]])
    seat = c - n * (PAD_T + BACK_T + SHIM_T); back = c - n * (PAD_T + SHIM_T)
    s1, s2 = sorted((seat - t * 14.0, seat + t * 14.0), key=lambda p: p[0])
    s1b, s2b = s1 - n * PLATE, s2 - n * PLATE
    face = 110.0 - CTUBE / 2; leg = face - PLATE
    zf = lambda x, p, q: p[1] + (x - p[0]) * (q[1] - p[1]) / (q[0] - p[0])
    bracket = [(leg, CR_Z0 + 1.5), (face, CR_Z0 + 1.5), (face, zf(face, s1, s2)), tuple(s2), tuple(s1), tuple(s1b), (leg, zf(leg, s1b, s2b))]
    bh = 11.0
    backing = [tuple(back - t * bh), tuple(back + t * bh), tuple(back + t * bh - n * BACK_T), tuple(back - t * bh - n * BACK_T)]
    for side, sy in (("Port", 1), ("Starboard", -1)):
        yc = sy * PAD_Y
        add(side + " stow stop bracket", ext(1, yc - 11.0, yc + 11.0, [tuple(map(float, p)) for p in bracket]),
            stock="machined from 1 x 1.5 in 6061-T6 bar (6.35 mm leg and seat)", status="new 002 (pan-stop-study proposal, relocated to the lug)")
        add(side + " stow stop pad backing", ext(1, yc - bh, yc + bh, [tuple(map(float, p)) for p in backing]), stock="3 mm 6061-T6 sheet, 22 x 22 mm")
    # --- soft stop (OD-5) on the X250 cradle top under the lower flange strike block
    SOFT_BEAM_X0, SOFT_KEEPER_X1 = 225.0, 275.0       # through-bolts at X 231 (aft of the cradle) and X 268 (forward of it)
    sc = np.array(soft["contactAtSoftStopMm"]); sn = np.array(soft["padPushDirection"]); st = np.array([sn[1], -sn[0]])
    sseat = sc - sn * (SOFT_PAD_T + SHIM_T)
    xp = sseat[0] - TUBE / 2
    x0c, x1c = sseat[0] - 12.7, sseat[0] + 12.7
    zs0, zs1 = (sseat[1] + (x - sseat[0]) * (st[1] / st[0]) for x in (x0c, x1c))
    zpost = min(zs0, zs1) - 3.0
    beam_top = CR_TOP + TUBE
    for side, sy in (("Port", 1), ("Starboard", -1)):
        y0, y1 = sorted((sy * 168.0, sy * (168.0 + TUBE)))
        add(side + " soft stop beam", ext(0, SOFT_BEAM_X0, xp + TUBE, sq(y0, CR_TOP, TUBE)),
            stock="3/4 x 3/4 in 6061-T6 square bar (solid), saw cut and drilled; bears on the X250 cradle top",
            status="new 002 (OD-5; r3: solid bar clamped to the cradle by two through-bolts and a keeper plate, no rivet nuts)")
        add(side + " soft stop keeper plate", ext(0, SOFT_BEAM_X0, SOFT_KEEPER_X1, [(y0, CR_Z0 - PLATE), (y1, CR_Z0 - PLATE), (y1, CR_Z0), (y0, CR_Z0)]),
            stock="1/4 in 6061-T6 plate, 2D waterjet or saw cut, drilled", status="new 002 (r3: reacts the soft-stop beam moment under the X250 cradle)")
        add(side + " soft stop post", tube_along(2, beam_top, zpost, xp, y0, TUBE, WALL),
            stock="3/4 x 3/4 x 1/16 in 6061-T6 square tube", status="new 002 (OD-5)")
        add(side + " soft stop cap", ext(1, y0, y1, [(x0c, zpost), (x1c, zpost), (x1c, zs1), (x0c, zs0)]),
            stock="machined 6061-T6 wedge cap, 25.4 x 19.05 mm, pad seat at -5 deg", status="new 002 (OD-5)")
        for z0, label in ((beam_top, "foot"), (zpost - 12.7, "top")):
            add(f"{side} soft stop post plug {label}", ext(2, z0, z0 + 12.7, sq(xp + WALL, y0 + WALL, TUBE - 2 * WALL)), stock="5/8 in 6061-T6 square bar, 12.7 mm")
    # --- inlet truss (OD-2): stringer extensions, splices, rear and side legs, landing pads, plugs
    x_ext0 = -307.95
    top_z, pad_top = STRINGER_Z0, 203.2 + PLATE
    for side, sy in (("Port", 1), ("Starboard", -1)):
        yl = sy * STRINGER_IN if sy > 0 else -(STRINGER_IN + TUBE)
        add(f"{side} upper stringer extension", tube_along(0, x_ext0, 100.475, yl, STRINGER_Z0, TUBE, WALL),
            stock="3/4 x 3/4 x 1/16 in 6061-T6 square tube, 408.4 mm", status="new 002 (OD-2)")
        y0, y1 = sorted((sy * (STRINGER_IN + TUBE), sy * (STRINGER_IN + TUBE + ANGLE_T)))
        add(f"{side} stringer splice", ext(1, y0, y1, [(50.475, CR_Z0), (150.475, CR_Z0), (150.475, STRINGER_Z0 + TUBE), (50.475, STRINGER_Z0 + TUBE)]),
            stock="1/8 in 6061-T6 plate, 100 x 19.05 mm", status="new 002 (OD-2)")
        legs = (("rear leg", (-298.0, sy * 242.0, top_z), (-351.0, sy * 242.0, pad_top), (-366.0, -336.0, sy * 227.0, sy * 257.0)),
                ("side leg", (-278.0, sy * 242.0, top_z), (-278.0, sy * 346.0, pad_top), (-291.0, -265.0, sy * 331.0, sy * 361.0)))
        for name, top, bot, padxy in legs:
            spec, _ = oblique_tube(top, bot, TUBE, WALL, top_z, pad_top)
            add(f"{side} inlet truss {name}", spec, stock="3/4 x 3/4 x 1/16 in 6061-T6 square tube", status="new 002 (OD-2 node)")
            xa, xb, ya, yb = padxy
            ya, yb = sorted((ya, yb))
            add(f"{side} inlet truss {name} landing pad", ext(2, 203.2, pad_top, [(xa, ya), (xb, ya), (xb, yb), (xa, yb)]), stock="1/4 in 6061-T6 plate")
            for z0, label in ((top_z - 12.7, "top"), (pad_top, "foot")):
                add(f"{side} inlet truss {name} plug {label}", inner_loop_oblique(top, bot, TUBE / 2 - WALL, z0, z0 + 12.7),
                    stock="5/8 in 6061-T6 square bar, 12.7 mm")
    # --- existing truss-to-stringer nodes (M9): plugged butt nodes inside the existing member tops
    for k in chassis:
        if not (k.startswith("Pan truss") and (k.endswith("member 0") or k.endswith("member 2"))):
            continue   # members 1/3/5 are coped onto members 0/2 below the stringer; only 0/2 bear on the stringer
        z1 = STRINGER_Z0; z0 = z1 - 12.7
        za, zb = z0 + 2.0, z1 - 2.0
        A, B = np.array(inner_square(chassis[k], za)), np.array(inner_square(chassis[k], zb))
        at = lambda z: (A + (B - A) * (z - za) / (zb - za)).tolist()
        a, b = at(z0), at(z1)
        add(f"Truss node plug {k.replace('Pan truss ', '')}", dict(axis=2, sections=[a, b], void=None),
            stock="5/8 in 6061-T6 square bar, 12.7 mm, machined to the member bore",
            status="new 002 (M9 node: plug, four blind rivets (two through each of two opposite member walls), one #10-32 SHCS through the stringer bottom wall; coped members 1/3/5 join members 0/2 below Z 468 with 1/8 in gussets on the rear X face, outside the cradle envelope)",
            source="existing member void sectioned from the taps-02 STEP; host " + "+".join(ids[k]["occurrencePath"][1:3]))
    return parts


def clip_z(section, zmin):
    """Clip a planar polygon (list of [x,y,z]) to z >= zmin (Sutherland-Hodgman)."""
    out = []
    n = len(section)
    for i in range(n):
        p, q = np.array(section[i]), np.array(section[(i + 1) % n])
        pin, qin = p[2] >= zmin, q[2] >= zmin
        if pin:
            out.append(p.tolist())
        if pin != qin:
            t = (zmin - p[2]) / (q[2] - p[2]); out.append((p + t * (q - p)).tolist())
    return out


def inner_loop_oblique(top, bottom, half, z0, z1):
    top, bottom = np.array(top, float), np.array(bottom, float)
    d = top - bottom; d /= np.linalg.norm(d)
    h = np.array([d[0], d[1], 0.0]); hn = np.linalg.norm(h)
    h = h / hn if hn > 1e-9 else np.array([1.0, 0, 0])
    q = np.array([-h[1], h[0], 0.0]); cosa = abs(d[2])
    def loop(z):
        t = (z - bottom[2]) / (top[2] - bottom[2]); c = bottom + t * (top - bottom)
        return [list(c + sa * half / cosa * h + sb * half * q) for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return dict(axis=2, sections=[loop(z0), loop(z1)], void=None)


# ---------------------------------------------------------------- main
def main():
    plugin, step = Path(sys.argv[1]), Path(sys.argv[2])
    sys.path.insert(0, str(plugin / "scripts"))
    from showcase_contacts import load_solids
    shop = json.loads((ART / "pan-shop-plan.json").read_bytes())
    mech = json.loads((ART / "pan-mechanics.json").read_bytes())
    stop, soft = stop_geometry(), soft_geometry()
    # sim-basis target: rebuild the 001 moving shop stock exactly
    base_parts = [dict(sections=i["sections"], void=i["void"], bores=i["bores"], hexes=i["hexes"],
                       material="5052-H32" if "5052" in i["material"] else "6061-T6") for i in shop["items"] if i["moving"]]
    target = moving_properties(base_parts, [])
    hardware, on_axis = moving_hardware(stop)
    ids = {}
    chassis, _ = load_solids(step, ids)
    parts = moving_parts(shop, stop) + fixed_parts(shop, stop, soft, chassis, ids)
    solids = {}
    for p in parts:
        s, method = build(p); solids[p["id"]] = s
        pr = props(s, DENSITY[p["material"]])
        p.update(method=method, boundsMm=[round(v, 4) for v in bbox(s)],
                 **{k: (round(v, 6) if isinstance(v, float) else [round(x, 4) for x in v]) for k, v in pr.items()})
    moving = [p for p in parts if p["moving"]]
    achieved = moving_properties(moving, hardware)
    on_axis_kg = sum(g for _, g in on_axis) / 1000
    # ---- clearance
    cboxes = {k: bbox(v) for k, v in chassis.items()}
    built = [p for p in parts if p["build"]]
    fixed_built = [p for p in built if not p["moving"]]
    near = 25.0
    static = []
    for p in fixed_built:
        for k, cb in cboxes.items():
            if box_gap(p["boundsMm"], cb) > near:
                continue
            v, common = gap(solids[p["id"]], chassis[k])
            static.append(dict(part=p["id"], chassis=k, occurrencePath=ids[k]["occurrencePath"][1:], gapMm=round(v, 4), commonMm3=round(common, 4)))
    internal = []
    for i, p in enumerate(fixed_built):
        for q in fixed_built[i + 1:]:
            if box_gap(p["boundsMm"], q["boundsMm"]) > 1.0:
                continue
            v, common = gap(solids[p["id"]], solids[q["id"]])
            if common > 1e-3:
                internal.append(dict(a=p["id"], b=q["id"], gapMm=round(v, 4), commonMm3=round(common, 4)))
    lo = SWEEP_LO
    angles = [round(lo + 0.5 * i, 3) for i in range(int((STOW - lo) / 0.5) + 1)] + [STOW + math.degrees(0.0014366689614586647), STOW_LIMIT]
    radius = {p["id"]: max(math.hypot(x - PIVOT[0], z - PIVOT[1]) for sec in p["sections"] for x, _, z in sec) for p in moving}
    fixed_all = {**{k: ("chassis", chassis[k], cboxes[k]) for k in chassis}, **{p["id"]: ("new", solids[p["id"]], p["boundsMm"]) for p in fixed_built}}
    sweep = {}
    for p in moving:
        for th in angles:
            s = pose(solids[p["id"]], th); sb = bbox(s)
            for k, (kind, f, fb) in fixed_all.items():
                if box_gap(sb, fb) > near:
                    continue
                v, common = gap(s, f)
                key = (p["id"], k); row = sweep.get(key)
                if row is None or v < row["minGapMm"] - 1e-9 or common > row["maxCommonMm3"]:
                    sweep[key] = dict(moving=p["id"], fixed=k, fixedKind=kind, minGapMm=round(v, 4), atThetaDeg=th,
                                      maxCommonMm3=round(max(common, row["maxCommonMm3"] if row else 0), 4),
                                      sampleBoundMm=round(radius[p["id"]] * math.radians(0.5) / 2, 3))
    sweep_rows = sorted(sweep.values(), key=lambda r: r["minGapMm"])
    elec = json.loads((ROOT / "iterations/002/design/chassis/inset-electrical-package.json").read_bytes())["candidatePlacements"]
    keep = {k: v["boundsMm"] for k, v in elec.items()}
    keep.update({"battery parked (gauge)": [-241.5, -39.0, 33.0, -58.5, 39.0, 202.0], "battery lift-out 250 mm (gauge)": [-255.0, -60.0, 33.0, -45.0, 60.0, 510.0]})
    keepout = sorted([dict(keepOut=k, part=min(fixed_built, key=lambda p: box_gap(p["boundsMm"], b))["id"],
                           minBoxGapMm=round(min(box_gap(p["boundsMm"], b) for p in fixed_built), 3)) for k, b in keep.items()], key=lambda r: r["minBoxGapMm"])
    stops = []
    for side in ("Port", "Starboard"):
        lug = solids[side + " cheek stop lug plate"]; strike = solids[side + " cheek lower stringer front"]
        for th, label in ((STOW, "stow"), (STOW - 0.05, "stow-0.05deg"), (angles[-2], "stow peak overtravel"), (STOW_LIMIT, "stow joint limit")):
            v, _ = gap(pose(lug, th), solids[side + " stow stop pad backing"])
            stops.append(dict(stop="stow", side=side, pose=label, thetaDeg=round(th, 5), strikeToSeatMm=round(v, 4), padPlusShimMm=PAD_T + SHIM_T))
        for th, label in ((DUMP, "dump"), (CUT_DEG, "drive inhibit"), (SOFT, "soft stop contact"), (SWEEP_LO, "fault-envelope joint limit")):
            v, _ = gap(pose(strike, th), solids[side + " soft stop cap"])
            stops.append(dict(stop="soft", side=side, pose=label, thetaDeg=th, strikeToSeatMm=round(v, 4), padPlusShimMm=SOFT_PAD_T + SHIM_T))
    envelopes = {}
    for label, th in (("stow", STOW), ("flat", 0.0), ("dump", DUMP), ("soft stop", SOFT)):
        boxes = [bbox(pose(solids[p["id"]], th)) for p in moving]
        envelopes[label] = [round(min(b[i] for b in boxes), 3) for i in range(3)] + [round(max(b[i] for b in boxes), 3) for i in range(3, 6)]
    corners = [(x, y, z) for x in (115.0, 545.0) for y in (-161.0, 161.0) for z in (578.0, 580.0)]
    turned = lambda th: [[round(world(x - PIVOT[0], z - PIVOT[1], th)[0], 5), y, round(world(x - PIVOT[0], z - PIVOT[1], th)[1], 5)] for x, y, z in corners]
    native_pairs = sorted({(r["part"], r["chassis"]) for r in static if r["gapMm"] < 2.0} |
                          {(r["moving"], r["fixed"]) for r in sweep_rows if r["minGapMm"] < 10.0})
    native_poses = [("stow joint limit", STOW_LIMIT), ("stow", STOW), ("mid", 35.0), ("flat", 0.0), ("dump", DUMP), ("soft stop", SOFT), ("fault-envelope joint limit", SWEEP_LO)]
    moving_ids = {p["id"] for p in moving}
    every = {**chassis, **solids}
    expected = []
    for label, th in native_poses:
        for a, b in native_pairs:
            sa = pose(every[a], th) if a in moving_ids else every[a]
            sb = pose(every[b], th) if b in moving_ids else every[b]
            v, common = gap(sa, sb)
            expected.append(dict(pair=[a, b], pose=label, thetaDeg=th, gapMm=round(v, 4), commonMm3=round(common, 4)))
    chassis_paths = {k: "+".join(ids[k]["occurrencePath"][1::2]) for k in chassis if not ids[k]["occurrencePath"][1].startswith("Matched SDS")}
    roots = {}
    for k, path in chassis_paths.items():
        roots[path.split("+")[0]] = roots.get(path.split("+")[0], 0) + 1
    seats = [
        dict(name="Seat REV-21-1915 port flanged hex bearing", pointMm=[205, 195.0, 580], axis=[0, 1, 0], host="Port bearing carrier", basis="carrier bore r14.2875, plate Y 191.85..198.15; radial only, axial float 0.25-0.5 mm"),
        dict(name="Seat REV-21-1915 starboard flanged hex bearing", pointMm=[205, -195.0, 580], axis=[0, 1, 0], host="Starboard bearing carrier", basis="mirror"),
        dict(name="Seat 6655K17 starboard inboard thrust stack", pointMm=[205, -189.925, 580], axis=[0, 1, 0], host="Starboard bearing carrier", basis="3.86 mm gap from the post outer plate face Y=-187.99 to the carrier face Y=-191.85"),
        dict(name="Seat 6655K17 starboard outboard thrust stack", pointMm=[205, -200.075, 580], axis=[0, 1, 0], host="Starboard bearing carrier", basis="carrier face Y=-198.15 to thrust collar Y=-202"),
        dict(name="Seat starboard hex thrust collar", pointMm=[205, -205.0, 580], axis=[0, 1, 0], host="Guarded pan", basis="Y -202..-208 on the extended idler stub"),
        dict(name="Seat closing torsion spring", pointMm=[205, -215.7, 580], axis=[0, 1, 0], host="Guarded pan", basis="body Y -208..-223.4 on an 16 mm arbor sleeve; see structure.json closingSpring"),
        dict(name="Seat spring driver / retaining collar", pointMm=[205, -226.0, 580], axis=[0, 1, 0], host="Guarded pan", basis="Y -223..-229, notched for the moving leg"),
        dict(name="Seat spring fixed-leg pin", pointMm=[190.0, -205.0, 600.0], axis=[0, -1, 0], host="Starboard bearing carrier", basis="#10 shoulder screw in the carrier, 17 mm proud of the outboard face"),
        dict(name="Seat UltraPlanetary output face", pointMm=[205, 216.0, 580], axis=[0, 1, 0], host="Port gearbox plate", basis="001 driveDatumY=216; 32 mm M3 bolt circle"),
        dict(name="Seat port drive 1/2 in hex shaft", pointMm=[205, 191.0, 580], axis=[0, 1, 0], host="Guarded pan", basis="gearbox output Y=216 through bearing Y=195 into the post plate stack Y=164.2..188.0; slip fit for axial float"),
        dict(name="Seat port limit-cam retaining collar", pointMm=[205, round(CAM_Y0 + CAM_T + 3.175, 4), 580], axis=[0, 1, 0], host="Guarded pan", basis="1/2 in hex collar Y 203.2..209.5 outboard of the cam; the bearing inner-race flange locates the cam inboard"),
        dict(name="Seat pan reverse-limit switch", pointMm=[205.0, 198.15, round(580.0 - CAM_R1 - 12.0, 4)], axis=[0, 1, 0], host="Port bearing carrier",
             basis="roller-lever snap-action switch on the carrier outboard face, body X 195..215, Z 535..556 on the solid bearing web; roller on the cam rim at world angle -90 deg, radius 20 (released) to 23.5 mm (operated); wired to the pan controller reverse-limit input"),
        dict(name="Seat port stow stop pad", pointMm=[round(v, 4) for v in (c_back(stop)[0], PAD_Y, c_back(stop)[1])], axis=[round(stop["padPushDirection"][0], 6), 0, round(stop["padPushDirection"][1], 6)], host="Port stow stop pad backing", basis="6 mm pad on 0.5 mm shims; contact on the cheek stop lug at stow"),
        dict(name="Seat starboard stow stop pad", pointMm=[round(v, 4) for v in (c_back(stop)[0], -PAD_Y, c_back(stop)[1])], axis=[round(stop["padPushDirection"][0], 6), 0, round(stop["padPushDirection"][1], 6)], host="Starboard stow stop pad backing", basis="mirror"),
        dict(name="Seat port soft stop pad", pointMm=[round(soft_seat(soft)[0], 4), 177.525, round(soft_seat(soft)[1], 4)], axis=[round(soft["padPushDirection"][0], 6), 0, round(soft["padPushDirection"][1], 6)], host="Port soft stop cap", basis="12 mm pad on 0.5 mm shims; contact on the lower-flange strike block at -5 deg"),
        dict(name="Seat starboard soft stop pad", pointMm=[round(soft_seat(soft)[0], 4), -177.525, round(soft_seat(soft)[1], 4)], axis=[round(soft["padPushDirection"][0], 6), 0, round(soft["padPushDirection"][1], 6)], host="Starboard soft stop cap", basis="mirror"),
    ]
    joints = [
        dict(name="Pan pivot", type="revolute", one="Guarded pan:1", two="Pan upper support:1", pointMm=[205, 0, 580], axis="Y",
             thetaLimitsDeg=[SWEEP_LO, STOW_LIMIT], savedThetaDeg=STOW, commandedThetaDeg=[DUMP, STOW], padFacesDeg=[SOFT, STOW], driveInhibitDeg=CUT_DEG,
             basis="minimum -9 deg covers the soft-stop fault envelope (F3 runaway about -8.5 deg on the compressed pads, structure.json softStop); maximum 70.1 deg covers the 70.082 deg stow peak overtravel; pad faces at -5 and 70 deg; -2 deg dump is the commanded software limit; -3.5 deg is the opening-direction drive inhibit"),
        dict(name="Pan upper support to chassis", type="rigid", one="Pan upper support:1", twoBody="Upper pan stringer 1",
             basis="anchor resolved from the occurrence that owns 'Upper pan stringer 1' (M7)"),
        dict(name="Fixed inlet to upper support", type="rigid", one="Fixed inlet:1", two="Pan upper support:1",
             basis="inlet feet bolt to the stringer extensions (OD-2)"),
    ]
    attachments = [
        dict(interface="Cradle ends to Upper pan stringer +/-1 inboard faces (four 3/4 in angle end clips)", worldMm=[[110, 232.475, 480], [110, -232.475, 480], [250, 232.475, 480], [250, -232.475, 480]], chassisBodies=["Upper pan stringer 1", "Upper pan stringer -1"]),
        dict(interface="Existing truss member tops to stringer underside: plugged butt nodes (M9)", worldMm=[[110, 242, 470.475], [250, 242, 470.475], [110, -242, 470.475], [250, -242, 470.475]], chassisBodies=[k for k in chassis if k.startswith("Pan truss") and (k.endswith("member 0") or k.endswith("member 2"))]),
        dict(interface="Stringer extensions spliced to Upper pan stringer +/-1 at X=100.475", worldMm=[[100.475, 242, 480], [100.475, -242, 480]], chassisBodies=["Upper pan stringer 1", "Upper pan stringer -1"]),
        dict(interface="Inlet truss rear legs on Structural perimeter 2 top", worldMm=[[-351, 242, 203.2], [-351, -242, 203.2]], chassisBodies=["Structural perimeter 2"]),
        dict(interface="Inlet truss side legs on Structural perimeter 0 / 4 tops", worldMm=[[-278, 346, 203.2], [-278, -346, 203.2]], chassisBodies=["Structural perimeter 0", "Structural perimeter 4"]),
        dict(interface="Starboard knee struts on Upper pan stringer -1 top", worldMm=[[160, -242, 489.525], [228, -242, 489.525]], chassisBodies=["Upper pan stringer -1"]),
        dict(interface="Carriers, gearbox plate (landing clips) on cradle tops", worldMm=[[110, 195, CR_TOP], [250, 195, CR_TOP], [110, -195, CR_TOP], [250, -195, CR_TOP], [110, 214, CR_TOP], [250, 214, CR_TOP]], chassisBodies=[]),
        dict(interface="Stow stop brackets on Upper cradle 110 rear face", worldMm=[[97.3, PAD_Y, 490], [97.3, -PAD_Y, 490]], chassisBodies=[]),
        dict(interface="Soft stop beams on Upper cradle 250 top, clamped through a keeper plate under the cradle by two #10-32 through-bolts each (X 231 and X 268, outside the cradle walls)", worldMm=[[231.0, 177.525, CR_TOP], [268.0, 177.525, CR_TOP], [231.0, -177.525, CR_TOP], [268.0, -177.525, CR_TOP]], chassisBodies=[]),
        dict(interface="Pan reverse-limit switch on the port carrier outboard face", worldMm=[[205.0, 198.15, 545.0]], chassisBodies=[]),
        dict(interface="Pan pivot journals", worldMm=[[205, 195, 580], [205, -195, 580]], chassisBodies=[]),
    ]
    inputs = {rel(f): sha(f) for f in [ART / "pan-shop-plan.json", ART / "pan-mechanics.json", step,
                                       ROOT / "iterations/002/design/references/pan-floor-430x322.step",
                                       ROOT / "iterations/002/design/chassis/inset-electrical-package.json", Path(__file__)]}
    upper_new = [p for p in parts if p["group"] == "Pan upper support"]
    rec = dict(
        target=dict(massKg=target[0], firstMomentUKgMm=target[1], firstMomentWKgMm=target[2], inertiaKgM2=target[3],
                    simMassKg=mech["movingShopStockKg"], simInertiaKgM2=mech["movingStockInertiaKgM2"],
                    simCentreMm=mech["movingShopCentreMm"]),
        achievedIncludingOffAxisJoiningHardware=dict(massKg=achieved[0], firstMomentUKgMm=achieved[1], firstMomentWKgMm=achieved[2], inertiaKgM2=achieved[3]),
        relativeError=dict(zip(("mass", "firstMomentU", "firstMomentW", "inertia"), ((achieved - target) / np.array([target[0], abs(target[1]), abs(target[2]), target[3]])).tolist())),
        stowHoldingArmMm=dict(target=float((target[1] * math.cos(math.radians(STOW)) - target[2] * math.sin(math.radians(STOW))) / target[0]),
                              achieved=float((achieved[1] * math.cos(math.radians(STOW)) - achieved[2] * math.sin(math.radians(STOW))) / achieved[0])),
        onAxisAddedKg=on_axis_kg, onAxisItems=on_axis, offAxisHardware=[dict(item=a, grams=g, atMm=list(p)) for a, g, p in hardware],
        disposition="r3 riveted ladder: contact surfaces (floor top, cheek inner faces, lip front faces) are unchanged. The moving properties differ from the sim basis by relativeError; a fresh 12-case replay with these properties is required. On-axis additions change only bearing load.",
        fixedNew=dict(upperSupportKg=round(sum(p["massKg"] for p in upper_new), 6),
                      fixedInletKg=round(sum(p["massKg"] for p in parts if p["group"] == "Fixed inlet"), 6)))
    findings = [
        dict(id="M1", kind="stop contact", text=f"The stow stop strikes a separate stop lug plate stack (7/8 in striking plate, skin lobe, 1/2 in outboard plate; screwed and pinned) whose striking face is {LUG_DEPTH:g} mm behind the lip back face at |Y|={PAD_Y}. The 2 mm lip panel alone gives about 12 N/mm (see structure.json). The stow contact is ({stop['contactAtStowMm'][0]:.2f}, +/-{PAD_Y}, {stop['contactAtStowMm'][1]:.2f}) mm with push direction ({stop['padPushDirection'][0]:.4f}, {stop['padPushDirection'][1]:.4f}). The lever is {stop['leverMm']:.2f} mm, so K = 2 x k_pad x 0.1^2 is unchanged."),
        dict(id="M7", kind="census", text=f"Chassis bodies by root occurrence: {roots}. The recipe resolves each referenced body to its own occurrence path and anchors to the occurrence that owns 'Upper pan stringer 1'."),
        dict(id="M9", kind="node", text="Existing truss-to-stringer nodes are plugged butt joints: a plug inside each member 0/2 top, four blind rivets (two through each of two opposite walls) and one #10-32 SHCS down through the stringer bottom wall. Coped members 1/3/5 join members 0/2 with 1/8 in rear-face gussets below Z 468. No weld. The cradles sit 1.5 mm above the member top faces."),
        dict(id="M8", kind="rules", text="See structure.json rulesCheck: start height and the 18 in extension limit both pass, so the geometry is unchanged for rules."),
    ]
    out = dict(
        schema="red2/pan-geometry@2", revision=REVISION, status="proposal r3 for review; offline OCP only, no Fusion",
        generator=rel(__file__), inputs=inputs,
        frame=dict(name="002 robot frame = taps-02 root", origin="chassis centre, floor Z=0", axes="+X forward, +Y port, +Z up", units="mm"),
        pivot=dict(pointMm=[PIVOT[0], 0.0, PIVOT[1]], direction=[0, 1, 0], thetaConvention="sim theta: floor direction [cos t, 0, sin t]",
                   fusionConvention="Fusion revolute about +Y: phi = -theta; the recipe verifies the sign on the floor landmark",
                   journalsMm={"portBearingPlaneY": 195.0, "starboardBearingPlaneY": -195.0, "driveDatumY": 216.0, "axialLocation": "starboard (idler) side only"}),
        poses=dict(stowDeg=STOW, dumpDeg=DUMP, softStopDeg=SOFT, driveInhibitDeg=CUT_DEG, flatAsAuthoredDeg=0.0, sweepFromDeg=lo, sweepToDeg=STOW_LIMIT, sweepStepDeg=0.5, sampled=len(angles)),
        stopGeometry=stop, softStopGeometry=soft, poseEnvelopesMm=envelopes, framePerimeterXYMm=[-370.0, -365.0, 370.0, 365.0],
        joints=joints, attachments=attachments, deferredSeats=seats, chassisOccurrences=dict(byRoot=roots, bodyPaths=chassis_paths), findings=findings,
        buildInput=dict(
            authority=dict(record=dict(status="pass", policy="registered-offline-archive", id="red2-two-sill-robot",
                                       configuration="Four parked matched SDS L2 modules and editable inset perimeter, electronics decks and pan support trusses",
                                       revision="002-connected-machining-654-r6", asOf="2026-11-21T14:01:40.849439+00:00",
                                       archiveSha256="e7c7386af0b1e775a7e0daba344f75082a42e6445ec0064b297497abc077834a",
                                       stepSha256="c430890bce46ceac46a57826cbc516d828073a147b5cf934a821621321d3fa77"),
                           lineage="issue-006/lineage.json"),
            parent=dict(stagedPath="external/taps-02/four-tapped-holes.f3d", sha256="e426e5768a25ab6038f69f5b43b6ab5ed80ad9aca997610c3217117731fa5471",
                        stepSha256=sha(step), receipt="external/taps-02/native-receipt.json",
                        receiptSha256="69ad4908e45caa26d87e2e9c710f81c19c841ffbf6a5fbbe17788695b728fa91", bodyCount=654,
                        expectedRootBodyCounts=roots, moduleOccurrences=[f"Matched SDS L2 module:{i}" for i in range(1, 5)]),
            outputs=dict(archive="pan-build.f3d", step="pan-build.step", receipt="native-receipt.json"),
            landmark=dict(body="430 mm pan floor", flatVerticesMm=[list(c) for c in corners], stowVerticesMm=turned(STOW), toleranceMm=0.01),
            nativeCheckPairs=[list(p) for p in native_pairs], nativePoses=[dict(label=l, thetaDeg=t) for l, t in native_poses],
            expectedGaps=expected, gapToleranceMm=0.05,
            materials={"6061-T6": "002 machined 6061-T6", "5052-H32": "002 5052-H32 sheet"},
            palette={"mechanism": "#007F78", "structure": "#496A81", "guide": "#B5C5D5"}),
        parts=parts,
        reconciliation=rec,
        clearance=dict(method="OCP BRepExtrema min distance + BRepAlgoAPI_Common, bbox-prefiltered (25 mm); moving parts rotated about the pivot",
                       chassisStep=rel(step), chassisSolids=len(chassis), staticNewFixedVsChassis=sorted(static, key=lambda r: r["gapMm"]),
                       newFixedInternalOverlaps=internal, sweepMovingVsFixed=sweep_rows, stops=stops, keepOuts=keepout))
    (HERE / "geometry.json").write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    print(json.dumps(dict(parts=len(parts), built=len(built), relErr=rec["relativeError"], arm=rec["stowHoldingArmMm"], achieved=rec["achievedIncludingOffAxisJoiningHardware"],
                          minSweep=sweep_rows[:6], minStaticCommon=[r for r in static if r["commonMm3"] > 0][:6], minStatic=sorted(static, key=lambda r: r["gapMm"])[:6],
                          internal=internal[:6], stops=stops, keep=keepout[:4], roots=roots, fixed=rec["fixedNew"], env=envelopes), indent=1))


def c_back(stop):
    c, n = np.array(stop["contactAtStowMm"]), np.array(stop["padPushDirection"])
    return c - n * (PAD_T + SHIM_T)


def soft_seat(soft):
    c, n = np.array(soft["contactAtSoftStopMm"]), np.array(soft["padPushDirection"])
    return c - n * (SOFT_PAD_T + SHIM_T)


if __name__ == "__main__":
    main()
