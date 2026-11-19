"""Register the actual signal light and its removable deck mount in the fixed-height robots."""
import argparse
import itertools
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "leave-park"))
from rehearse import KIT, cad, digest
from assembly import pose_shapes
from bumper import specifications
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.STEPControl import STEPControl_Reader
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Ax2, gp_Dir, gp_Lin, gp_Pnt, gp_Trsf

VENDOR = "6dc6d4c850e6cf5798d3b2082ffc15527ad5cdd0869b641439cf62905ee074ca"
LOCKS = ROOT / "design/fixed-height"
X, Y, PANEL = 145, -250, 293


def read(path):
    return json.loads(path.read_bytes())


def product(path):
    if digest(path) != VENDOR:
        raise ValueError("Expected the issued am-3583 generic STEP")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != cad.IFSelect_RetDone or reader.TransferRoots() != 1:
        raise ValueError("Could not reopen the product")
    shape = reader.OneShape()
    if cad.solid_count(shape) != 1:
        raise ValueError("Unexpected product solids")
    return shape


def mounted(shape, x, y, shoulder):
    # Native +Y points to the dome. The model's shoulder and nut faces sit at -19.2 and -26.2.
    tr = gp_Trsf()
    tr.SetValues(1, 0, 0, x, 0, 0, -1, y, 0, 1, 0, shoulder + 19.2)
    return shape.Moved(TopLoc_Location(tr))


def cylinder(x, y, z, radius, height):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z), gp_Dir(0,0,1)), radius, height).Shape()


def difference(a,b):
    op = BRepAlgoAPI_Cut(a,b)
    if not op.IsDone():
        raise ValueError("Could not machine the plate")
    return op.Shape()


def common(a,b):
    op = BRepAlgoAPI_Common(a,b)
    if not op.IsDone():
        raise ValueError("Intersection failed")
    return cad.volume(op.Shape())[0]


def overlaps(a,b):
    return all(a[i] <= b[i+3] + 1e-6 and b[i] <= a[i+3] + 1e-6 for i in range(3))


def hits(shape, others, boxes):
    box = cad.bounds(shape)
    positive = {}
    for key, other in others.items():
        if overlaps(box, boxes[key]):
            volume = common(shape,other)
            if volume > .05:
                positive[key] = volume
    return positive


def first(intersector, line, stop):
    intersector.Perform(line, 0, stop)
    if not intersector.IsDone():
        raise ValueError("Incomplete visibility ray")
    return min((intersector.WParameter(i) for i in range(1,intersector.NbPnt()+1)), default=math.inf)


def vision(assembly, boxes, lamp, side, xy=(X,Y)):
    cache = {}
    def query(key, shape, line, stop):
        if key not in cache:
            cache[key] = IntCurvesFace_ShapeIntersector()
            cache[key].Load(shape, 1e-6)
        return first(cache[key],line,stop)
    result = []
    for height, lateral in itertools.product((1200,1500,1800), (-300,0,300)):
        distance=425+914.4
        eye={"front":(distance,lateral,height),"rear":(-distance,lateral,height),
             "left":(lateral,distance,height),"right":(lateral,-distance,height)}[side]
        visible = 0
        blocked = {}
        for dz,dy in itertools.product((30,38,46), (-8,0,8)):
            target = (xy[0], xy[1]+dy, PANEL+dz)
            delta = [v-e for v,e in zip(target,eye)]
            line = gp_Lin(gp_Pnt(*eye), gp_Dir(*delta))
            stop = query("am-3583",lamp,line,math.dist(eye,target)+25)
            if not math.isfinite(stop):
                raise ValueError("Target ray missed the dome")
            length = math.dist(eye,target)
            near = [e+d*stop/length for e,d in zip(eye,delta)]
            if near[2] < PANEL+15:
                raise ValueError("First hit is below the dome sample region")
            bounds = [min(p,q) for p,q in zip(eye,near)] + [max(p,q) for p,q in zip(eye,near)]
            nearer = [k for k,s in assembly.items() if k!="am-3583" and overlaps(bounds,boxes[k])
                      and query(k,s,line,stop-.01) < stop]
            if nearer:
                for key in nearer:
                    blocked[key]=blocked.get(key,0)+1
            else:
                visible+=1
        result.append({"eye_mm":eye, "visible_of_nine":visible, "blockers":blocked})
    return result


def swept_view(assembly, boxes, lamp):
    frames=read(KIT/"part-frames.json")
    moving={}
    for k,b in boxes.items():
        if frames.get(k) not in ("middle","slide") and k not in ("slide-position-line","slide-power-pinion"):
            continue
        bb=list(b)
        if k in ("slide-power-pinion","slide-fixed-rack-pinion","slide-inner-rack-pinion"):
            for i in (0,2):
                bb[i]-=2
                bb[i+3]+=2
        if k!="slide-power-pinion":
            bb[3]+=185 if frames[k]=="middle" else 370
        moving[k]=bb
    if len(moving)!=157:
        raise ValueError("Fixed-low slide inventory changed")
    field=dict(assembly)
    for k,b in moving.items():
        field[k]=BRepPrimAPI_MakeBox(gp_Pnt(*b[:3]),gp_Pnt(*b[3:])).Shape()
    return {"moving_ids":len(moving),"right":vision(field,{**boxes,**moving},lamp,"right"),
            "lamp_envelope_pairs":[k for k,b in moving.items() if overlaps(b,boxes["am-3583"])]}


def configuration(mode):
    low = mode != "high"
    pose = "place" if low else "stowed"
    source = KIT / (pose+".step")
    record = read(LOCKS/"native.json")["low-red-place" if low else "red"]
    if digest(source) != record["revision"]:
        raise ValueError("Unregistered fixed-height base")
    ids = read(KIT/"part-frames.json")
    base = pose_shapes(560,0,"red") if mode=="low-receive" else cad.read_named(source,[{"id":k} for k in ids])
    kit = LOCKS/("low-locks.step" if low else "locks.step")
    edits, additions = set(record["editedVolumesMm3"]), set(record["addedVolumesMm3"])
    if digest(kit)!=record["exportSha256"] or not edits <= base.keys() or additions & base.keys():
        raise ValueError("Unregistered lock kit")
    changes = cad.read_named(kit,[{"id":k} for k in edits|additions])
    return {**base,**changes}


def inspect(mode, native, output):
    assembly = configuration(mode)
    old = assembly.pop("rsl-reserve")
    deck = assembly.pop("front-electronics-platform")
    # One contiguous three-millimetre replacement blank, with a rounded tab on robot right.
    neck = BRepPrimAPI_MakeBox(gp_Pnt(X-33,Y,290),66,40,3).Shape()
    tab = BRepAlgoAPI_Fuse(neck,cylinder(X,Y,290,33,3)).Shape()
    blank = BRepAlgoAPI_Fuse(deck,tab).Shape()
    bore = cylinder(X,Y,280,11.4,20)
    round_hole = difference(blank,bore)
    key_relief = BRepPrimAPI_MakeBox(gp_Pnt(X-11.6,Y-9.1,289),4.1,4.5,5).Shape()
    drilled = difference(round_hole,key_relief)
    spacer = difference(cylinder(X,Y,PANEL-7,16,4),bore)
    lamp = mounted(native,X,Y,PANEL)
    assembly.update({"front-electronics-platform":drilled,"light-spacer":spacer})
    boxes = {k:cad.bounds(s) for k,s in assembly.items()}
    fit = hits(lamp,assembly,boxes)
    spacer_hits = hits(spacer,{k:s for k,s in assembly.items() if k!="light-spacer"},boxes)
    no_bore = common(lamp,blank)
    # Displace the complete vendor model into each bearing surface to verify axial capture.
    down = common(cad.translated(lamp,[0,0,-.5]),drilled)
    up = common(cad.translated(lamp,[0,0,.5]),spacer)
    if fit or spacer_hits or no_bore<=.05 or min(down,up)<=.05:
        raise ValueError(("Mount geometry",mode,fit,spacer_hits,no_bore,down,up))
    assembly["am-3583"] = lamp
    boxes["am-3583"] = cad.bounds(lamp)
    # Project allowance for insulated terminals and lead turn, immediately under the blades.
    service = cylinder(X,Y,PANEL-70,16,45)
    service_hits = hits(service,{k:s for k,s in assembly.items() if k!="am-3583"},boxes)
    removable = {k for k in specifications("red") if not k.startswith("bumper-sleeve-")}
    if not removable <= assembly.keys():
        raise ValueError("Unexpected bumper inventory")
    bare = {k:b for k,b in boxes.items() if k not in removable}
    sides = [min(b[i] for b in bare.values()) for i in (0,1)] + [max(b[i] for b in bare.values()) for i in (3,4)]
    top = max(b[5] for b in boxes.values())
    corners = []
    for x,y in itertools.product((-350,350),repeat=2):
        witness = BRepPrimAPI_MakeBox(gp_Pnt(min(x,x*.998),min(y,y*.998),100),.7,.7,1).Shape()
        rail = assembly["drive-rail-outer-"+("L" if y>0 else "R")]
        corners.append(common(rail,witness))
    mast_trial = mounted(native,-47,221,1020)
    rays = vision(assembly,boxes,lamp,"right")
    counter={}
    if mode=="low-place":
        inside=mounted(native,215,-4,PANEL)
        counter=vision(assembly,boxes,inside,"front",(215,-4))
    travel=swept_view(assembly,boxes,lamp) if mode=="low-receive" else None
    if mode=="high" and output:
        from check_routing import write_step
        from duplex import finalize_step
        parts={"front-electronics-platform":drilled,"light-spacer":spacer}
        write_step(output,parts)
        finalize_step(output)
        restored=cad.read_named(output,[{"id":k} for k in parts])
        if any(abs(cad.volume(restored[k])[0]-cad.volume(s)[0])>.05 for k,s in parts.items()):
            raise ValueError("Mount changed on STEP round-trip")
    return {"installed_model_bodies":len(assembly),"perimeter_corner_witness_mm3":corners,
            "removable_bumper_bodies":len(removable),"bumper_off_xy_mm":sides,
            "top_mm":top,"height_margin_mm":1066.8-top,
            "tallest":sorted(k for k,b in boxes.items() if abs(b[5]-top)<.001),
            "prior_reserve_mm":cad.bounds(old),"mast_trial_top_mm":cad.bounds(mast_trial)[5],
            "mast_trial_interference_mm3":hits(mast_trial,{k:s for k,s in assembly.items() if k!="am-3583"},boxes),
            "mount_mm":cad.bounds(lamp),"uncut_deck_interference_mm3":no_bore,
            "round_hole_key_interference_mm3":common(lamp,round_hole),
            "bearing_witness_mm3":{"down_half_mm":down,"up_half_mm":up},
            "added_deck_volume_mm3":cad.volume(drilled)[0]-cad.volume(deck)[0],
            "spacer_volume_mm3":cad.volume(spacer)[0],"hardware_interference_mm3":fit,
            "spacer_interference_mm3":spacer_hits,"lead_reserve_interference_mm3":service_hits,
            "right_view":rays,"low_slide_sweep":travel,"inboard_front_trial":counter}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("vendor_step",type=Path)
    parser.add_argument("--output",type=Path,default=HERE/"review.json")
    parser.add_argument("--mount-step",type=Path)
    args=parser.parse_args()
    native=product(args.vendor_step)
    cases={key:inspect(key,native,args.mount_step) for key in ("high","low-receive","low-place")}
    results={"vendor_sha256":VENDOR,"vendor_native_bounds_mm":cad.bounds(native),
             "incremental_budget_kg":[0,.125],
             "lock_register_sha256":digest(LOCKS/"native.json"),"cases":cases}
    args.output.write_bytes((json.dumps(results,indent=2,allow_nan=False)+"\n").encode())
