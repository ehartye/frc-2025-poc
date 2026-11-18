"""Independent per-side backup haul lines. Original fit-check shapes, robot mm."""
import argparse
import hashlib
import json
import re
from pathlib import Path

from check_routing import Y, Z, BASE, box, cylinder, cut, brep, clashes, write_step, read_named, volume

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parent / "competition-package"
REPLACES = {"common-shaft-round-envelope", "lift-crossbar", "mast--235", "mast-210"}
ROPE_R = 3.175/2
RADIUS = 19.05/2 + ROPE_R
DRUM_X = -92
SHEAVE_X = DRUM_X - RADIUS + 20.5
TAIL_X = SHEAVE_X + 20.5
SHAFT_R = 6.35 / (3**.5 / 2)


MAST_HOLE_Z=(275,298,363,388,886,911,994,1008)


def machining_tools(name,height):
    if name=="lift-crossbar":
        return [cylinder(-87,side*y,height-149,Z,3.3,23) for side in (-1,1) for y in (214,232)]
    if name in {"mast--235","mast-210"}:
        side=1 if name=="mast-210" else -1
        return [cylinder(-80,side*222,z,[1,0,0],4.05,72) for z in MAST_HOLE_Z]
    raise ValueError("no retention machining on "+name)


def machine_existing(name,shape,height):
    """Add only the retention holes to a separately machined shop body in robot coordinates."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    for tool in machining_tools(name,height):
        cut_op=BRepAlgoAPI_Cut(shape,brep(tool))
        if not cut_op.IsDone(): raise ValueError("failed retention cut in "+name)
        shape=cut_op.Shape()
    return shape


def native(height):
    crossbar=box(-97,-240,height-148,20,480,21)
    holes=machining_tools("lift-crossbar",height)
    shapes = {"long-common-shaft": brep(["hex",[DRUM_X,-320,330,12.7,640]]),
              "lift-crossbar-drilled":brep(cut(crossbar,*holes))}
    for side, tag in ((1, "L"), (-1, "R")):
        yc = 292 * side
        def slab(x, inside, z, length, width, thick):
            return box(x, inside if side==1 else -inside-width, z, length, width, thick)
        def put(key, value):
            shapes["backup-"+tag+"-"+key] = brep(value)
        put("drum-core",cut(cylinder(DRUM_X,yc-13.462,330,Y,19.05/2,26.924),
                            ["hex",[DRUM_X,yc-14,330,12.72,28]]))
        put("upper-sheave", ring_descriptor(SHEAVE_X, yc, 975, 20.5-ROPE_R, 6.35, 7))
        for idx in (0, 1):
            for key, x, z, width, outside, bore in (("drum", DRUM_X, 330, 26.924, 20.7, SHAFT_R),
                                                    ("sheave", SHEAVE_X, 975, 7, 22, 6.35)):
                y = yc-width/2-2 if idx==0 else yc+width/2
                put(key+"-flange-"+str(idx),ring_descriptor(x,y+1,z,outside,bore,2))
        put("upper-pin", cylinder(SHEAVE_X,yc-12,975,Y,6.35,24))
        # The separate top hanger extends toward the forward mast face, clear of the first sheave.
        top_mount= ["union",[slab(-20,216,988,8,90,26), slab(-20,300,963,8,6,32),
                                     slab(SHEAVE_X-9,300,963, -12-(SHEAVE_X-9),6,24)]]
        def transverse(z, start=-80, length=72, radius=2.75):
            return cylinder(start,side*222, z,[1,0,0],radius,length)
        put("top-hanger",cut(top_mount,cylinder(SHEAVE_X,yc-16,975,Y,6.36,32),transverse(994),transverse(1008)))
        put("outboard-bearing",ring_descriptor(DRUM_X,side*314,330,14.285,SHAFT_R,8))
        support=["union",[slab(-20,215,268,8,15,37),slab(-20,216,281,8,102,10),slab(DRUM_X-16,310,281, -12-(DRUM_X-16),8,12),
                          slab(DRUM_X-16,310,286,32,8,60)]]
        put("bearing-support",cut(support,cylinder(DRUM_X,side*314-7,330,Y,14.3,14),transverse(275),transverse(298)))
        # Independent pit support: twin captive removable pins, working poses only.
        rail=slab(-124,273,350,12,18,555)
        bores=[cylinder(-125,side*282,h-165,[1,0,0],6,16) for h in (560,670,940)]
        put("service-pin-rail",cut(rail,*bores))
        for h in (560,670,940):
            put("service-pin-bushing-"+str(h),cut(cylinder(-124,side*282,h-165,[1,0,0],6,12),
                                                  cylinder(-125,side*282,h-165,[1,0,0],4.1,14)))
        for h in (368,891):
            mount=["union",[slab(-124,217,h,54,10,14), slab(-124,217,h,12,56,14),
                            slab(-78,214,h-11,8,16,38)]]
            put("service-rail-arm-"+str(h),cut(mount,transverse(h-5),transverse(h+20)))
        mast_y=210 if side>0 else -235
        put("drilled-mast",cut(box(-70,mast_y,154,50,25,866),box(-68,mast_y+2,153,46,21,868),
                               *machining_tools("mast-"+str(mast_y),height)))
        for z in MAST_HOLE_Z:
            front=z in (275,298,994,1008)
            put("mast-spacer-"+str(z),cut(transverse(z,-70,50,4),transverse(z,-71,52,2.65)))
            put("mast-bolt-"+str(z),transverse(z,-70 if front else -78,58,2.5))
        put("running-line", cylinder(DRUM_X-RADIUS,yc,330,Z,ROPE_R,645))
        # Bolt plate directly to underside of the carriage at its own two through locations.
        holes = [cylinder(-87,side*y,height-169,Z,3.3,28) for y in (214,232)]
        put("carriage-plate",cut(["union",[slab(-98,207,height-160,23,96,12),slab(-98,267,height-160,45,36,12)]],*holes))
        for y in (214,232):
            put("carriage-bolt-"+str(y),cylinder(-87,side*y,height-160,Z,3,33))
        fork = ["union",[slab(TAIL_X-7,282,height-148,14,5,24),
                         slab(TAIL_X-7,297,height-148,14,5,24),
                         slab(TAIL_X-7,282,height-148,14,20,3)]]
        put("clevis",cut(fork,cylinder(TAIL_X,yc-13,height-134,Y,4,26)))
        put("clevis-pin",cylinder(TAIL_X,yc-12,height-134,Y,4,24))
        put("thimble",ring_descriptor(TAIL_X,yc,height-134,8,4.1,6))
        put("spliced-eye-envelope",ring_descriptor(TAIL_X,yc,height-134,10.6,8,3.175))
        put("return-line", cylinder(TAIL_X,yc,height-123.4,Z,ROPE_R,975-(height-123.4)))
    return shapes


def ring_descriptor(x, y, z, r, bore, width):
    return cut(cylinder(x,y-width/2,z,Y,r,width),cylinder(x,y-width/2-1,z,Y,bore,width+2))


def finalize_step(path):
    text=path.read_text(encoding="ascii")
    # Only the exchange header timestamp is volatile. Entity records remain the native export.
    text=re.sub(r"(FILE_NAME\('[^']*',')[^']+(')",r"\g<1>1970-01-01T00:00:00\2",text,count=1)
    path.write_text("\n".join(line.rstrip() for line in text.splitlines())+"\n",encoding="ascii",newline="\n")


def check(step=None,only=None):
    frames=json.loads((PACKAGE/"part-frames.json").read_bytes())
    review=json.loads((PACKAGE/"review.json").read_bytes())
    poses=json.loads((BASE/"assembly.json").read_bytes())["poses"]
    result={}
    for pose, values in poses.items():
        if only and pose!=only: continue
        source=PACKAGE/"poses"/(pose+".step")
        sha=hashlib.sha256(source.read_bytes()).hexdigest()
        if sha!=review[pose]["export_sha256"]: raise ValueError("different pose input")
        base=read_named(source,[{"id":k} for k in frames])
        for old in REPLACES: del base[old]
        shapes=native(values["height"])
        keys=list(shapes)
        pairs={}
        for i,name in enumerate(keys):
            pairs.update(clashes({name:shapes[name]},{k:shapes[k] for k in keys[i+1:]}))
        hits=clashes(shapes,base)
        result[pose]={"sourceSha256":sha,"baseBodies":len(frames),"addedBodies":len(shapes),
                      "externalOverlapMm3":hits,"internalOverlapMm3":pairs}
        # A pin is withdrawn 65 mm toward robot rear from its inserted -131..-66 mm interval.
        pins={"service-pin-"+str(side):brep(cylinder(-196,side*282,values["height"]-165,[1,0,0],4,130))
              for side in (-1,1)}
        result[pose]["servicePinInsertionSweepOverlaps"]=clashes(pins,base|shapes)
        if step and pose=="place":
            write_step(step,shapes)
            finalize_step(step)
            restored=read_named(step,[{"id":k} for k in shapes])
            result[pose]["layerStepSha256"]=hashlib.sha256(step.read_bytes()).hexdigest()
            result[pose]["uniqueReopenedLayerBodies"]=len(restored)
    return result


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--step",type=Path)
    p.add_argument("--report",type=Path)
    p.add_argument("--pose",choices=["place","stowed","receive","transport"])
    a=p.parse_args()
    report=check(a.step,a.pose)
    text=json.dumps(report,indent=2)+"\n"
    if a.report: a.report.write_text(text,encoding="utf-8",newline="\n")
    else: print(text)
    if any(r["externalOverlapMm3"] or r["internalOverlapMm3"] or r["servicePinInsertionSweepOverlaps"] for r in report.values()):
        raise ValueError("positive material overlap in candidate geometry")
