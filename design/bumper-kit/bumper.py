"""Editable two-module bumper kit. Dimensions in millimetres in the existing robot frame."""
from dataclasses import dataclass
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "design/station-l1"))
sys.path.insert(0, str(ROOT / "design/drivetrain"))
import build_assembly as cad
from mechanical import box, cylinder, cut
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt, gp_Vec
import math

REPLACED = {"bumper-side-L", "bumper-side-R", "bumper-end-front", "bumper-end-rear"}
FRAME = 350.0
INSIDE = 351.0
BACK = 355.0
FOAM = 424.5
OUTSIDE = 425.0
LOW, HIGH = 43.0, 181.0
HALF_SEAM = 1.0
AXIS_Z = 127.0


def brick(x1, x2, y1, y2, z1, z2):
    return box(min(x1, x2), min(y1, y2), min(z1, z2), abs(x2-x1), abs(y2-y1), abs(z2-z1))


def panel(end, left, inner, outer, stop, low, high):
    if left == 0:
        return brick(end*inner, end*outer, -stop, stop, low, high)
    return brick(end*HALF_SEAM, end*stop, left*inner, left*outer, low, high)


def joined(specs):
    return ["union", list(specs)]


def u_shape(end, inner, outer, low, high, seam=HALF_SEAM):
    return cut(brick(end*seam, end*outer, -outer, outer, low, high),
               brick(end*(seam-2), end*inner, -inner, inner, low-1, high+1))


@dataclass(frozen=True)
class Anchor:
    tag: str
    face: str
    sign: int
    tangent: float
    rail: str
    width: float

    def cylinder(self, radius, start, stop):
        axis = [self.sign, 0, 0] if self.face == "end" else [0, self.sign, 0]
        point = [self.sign*start, self.tangent] if self.face == "end" else [self.tangent, self.sign*start]
        return cylinder(*point, AXIS_Z, axis, radius, stop-start)


def anchors():
    for end, label in [(1, "front"), (-1, "rear")]:
        for side, lr in [(1,"L"), (-1,"R")]:
            yield Anchor(f"{label}-end-{lr}", "end", end, side*175, "cross-"+label, 20)
            yield Anchor(f"{label}-side-{lr}", "side", side, end*125, "drive-rail-outer-"+lr, 25)


def machined_rails(shapes):
    """Apply the drill schedule to existing same-ID rail shapes, including their earlier cuts."""
    changed = {}
    for mount in anchors():
        key = mount.rail
        cutter = cad.brep(mount.cylinder(6.0, FRAME-mount.width-1, FRAME+1))
        op = BRepAlgoAPI_Cut(changed.get(key, shapes[key]), cutter)
        if not op.IsDone():
            raise ValueError("Failed rail drilling at "+mount.tag)
        changed[key] = op.Shape()
    return changed


def specifications(alliance="blue"):
    if alliance not in {"red", "blue"}:
        raise ValueError("Alliance must be red or blue")
    items = {}

    def put(key, group, shape):
        items[key] = (group, shape)

    for end, label in [(1, "front"), (-1, "rear")]:
        fixed = [a for a in anchors() if a.tag.startswith(label)]
        back = u_shape(end, INSIDE, BACK, LOW, HIGH)
        foam = u_shape(end, BACK, FOAM, LOW, HIGH)
        for mount in fixed:
            flange = mount.cylinder(12, BACK, BACK+8)
            foam = cut(foam, flange)
            back = cut(back, mount.cylinder(4, INSIDE-1, BACK+1))
            # OD8 is the nominal thread envelope. Helix, socket and washer chamfer are suppressed.
            put("bumper-boss-"+mount.tag, "alloy", cut(flange, mount.cylinder(4, BACK-1, BACK+9)))
            inner = FRAME-mount.width
            put("bumper-sleeve-"+mount.tag, "alloy", cut(mount.cylinder(6, inner, FRAME),
                mount.cylinder(4.3, inner-1, FRAME+1)))
            put("bumper-washer-"+mount.tag, "steel", cut(mount.cylinder(8, inner-1.6, inner),
                mount.cylinder(4.2, inner-2, inner+1)))
            # DIN 912 / ISO 4762 M8. Length stock 35/40; final end projection <=2 mm.
            shoulder = inner-1.6
            reach = shoulder + (35 if mount.width == 20 else 40)
            screw = joined([mount.cylinder(6.5, shoulder-8, shoulder), mount.cylinder(4, shoulder, reach)])
            foam = cut(foam, mount.cylinder(4, BACK, reach+0.2))
            put("bumper-screw-"+mount.tag, "steel", screw)
        for side in [0, 1, -1]:
            key = "end" if side == 0 else ("left" if side == 1 else "right")
            contact = panel(end, side, FRAME, INSIDE, 324 if side == 0 else 349, 57, 143)
            for mount in fixed:
                contact = cut(contact, mount.cylinder(4.3, FRAME-1, INSIDE+1))
            put("bumper-contact-"+label+"-"+key, "alloy", contact)
        put("bumper-backing-"+label, "alloy", back)
        # One cut U plank plus pockets. It can also be made of plane-butted straight strips
        # with full square corner blocks, permanently bonded under the SAME continuous cover.
        put("bumper-foam-"+label, "foam", foam)
        # Subtract the exact U cavity, including its tips, for exposed cloth over the end returns.
        put("bumper-cover-"+label+"-"+alliance, alliance, None)
    return items


def bumper_parts(alliance="blue"):
    def clip(shape, end, intercept):
        for side in [-1, 1]:
            wire = BRepBuilderAPI_MakePolygon()
            for x, y in [(intercept-450,450),(450,intercept-450),(450,450)]:
                wire.Add(gp_Pnt(end*x,side*y,LOW-2))
            wire.Close()
            cutter = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),
                                           gp_Vec(0,0,HIGH-LOW+4)).Shape()
            shape = BRepAlgoAPI_Cut(shape,cutter).Shape()
        return shape

    parts = {}
    for key,(group,spec) in specifications(alliance).items():
        end = 1 if "front" in key else -1
        if group in {"red", "blue"}:
            outside = clip(cad.brep(u_shape(end, BACK, OUTSIDE, LOW-.5, HIGH+.5, HALF_SEAM-.5)),end,842)
            inside = clip(cad.brep(u_shape(end, BACK-.1, FOAM, LOW, HIGH)),end,842-.5*math.sqrt(2))
            parts[key] = BRepAlgoAPI_Cut(outside,inside).Shape()
        elif group == "foam":
            parts[key] = clip(cad.brep(spec),end,842-.5*math.sqrt(2))
        else:
            parts[key] = cad.brep(spec)
    return parts


def export_step(path, alliance, parts):
    colors = {"alloy":"#a3aebb", "steel":"#647181", "foam":"#e4e3d8",
                       "red":"#c82136", "blue":"#1b48a3"}
    labels = [{"id":k, "group":group} for k,(group,_) in specifications(alliance).items()]
    cad.write_named(path, list(parts.values()), labels, colors)
    return cad.read_named(path, labels)
