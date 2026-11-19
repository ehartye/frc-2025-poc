"""Build original candidate camera bracket and check the reopened robot poses with Open Cascade."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from math import cos, sin, radians, dist
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.STEPControl import STEPControl_Writer, STEPControl_Reader, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.gp import gp_Ax1,gp_Pnt,gp_Dir,gp_Lin,gp_Trsf

HERE=Path(__file__).resolve().parent
ASSEMBLY=HERE.parent/"design"/"station-l1"
sys.path.insert(0,str(ASSEMBLY))
import build_assembly as native
from mechanical import box, cylinder, cut
from stereo import ORIGIN, BASELINE, TILT, RADIUS, endpoints


def angled(spec):
    shape=native.brep(spec)
    tr=gp_Trsf()
    tr.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,1,0)),TILT)
    return native.translated(BRepBuilderAPI_Transform(shape,tr,False).Shape(),ORIGIN)


def parts():
    # Lens plane at local X=0. Open front and top for optics; slots receive retention strap.
    foot=box(-346,-30,154,39,60,4)
    for x in (-341,-311):
        for y in (-23,23): foot=cut(foot,cylinder(x,y,153,(0,0,1),2.65,6))
    upright=cut(box(-337,-10,158,20,20,798),box(-335,-8,157,16,16,800))
    cradle=cut(box(-27,-49,-18,29,98,36),box(-23,-46,-15,28,92,35))
    for y in (-47.5,47.5):
        cradle=cut(cradle,box(-20,y-2.5,-5,5,5,11))
    # Bent rear attachment tab joins the cradle to the post. Rear cable loop is a reserved box.
    cradle_body=angled(cradle)
    post=native.brep(upright)
    # A project-fabricated lug joins the back wall to the outside of the upright.
    tab=native.brep(box(-337,10,951,5,4,30))
    weldment=native.BRepAlgoAPI_Fuse(cradle_body,tab).Shape()
    camera=angled(box(-22,-45.5,-14,22,91,28))
    weldment=native.BRepAlgoAPI_Cut(weldment,camera).Shape()
    fixed={"perforated-foot":native.brep(foot),"upright":post,
            "slotted-cradle-weldment":weldment,"camera-proxy":camera,
            "usb-service-reserve":angled(box(-26,-67,-12,23,18,24)),
            "cable-trunk-reserve":native.brep(box(-338,-50,158,8,9,810)),
            "rear-cable-raceway":native.brep(box(-338,-201,155,8,158,5)),
            "pi-and-converter-backplate":native.brep(box(-298,-232,154,152,65,3)),
            "pi4-service-envelope":native.brep(box(-294,-228,157,94,58,36)),
            "camera-5v-reserve":native.brep(box(-194,-226,157,28,27,14)),
            "host-5v-reserve":native.brep(box(-194,-194,157,28,24,14))}
    # Maximum installed laminate envelopes. As-exported position is the place pose.
    for i,y in enumerate((-70,0,70)):
        head=native.brep(cylinder(525,y,500,(0,0,1),3.6,.8))
        tail=native.brep(box(495,y-2.5,500,30,5,.8))
        fixed["moving-pad-"+str(i)]=native.BRepAlgoAPI_Fuse(head,tail).Shape()
    return fixed


def hits(shapes,start,end):
    distance=dist(start,end)
    d=gp_Dir(*(b-a for a,b in zip(start,end)))
    line=gp_Lin(gp_Pnt(*start),d)
    result=[]
    for name,shape in shapes.items():
        test=IntCurvesFace_ShapeIntersector()
        test.Load(shape,1e-6)
        test.Perform(line,0.02,distance-.05)
        if test.NbPnt(): result.append(name)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument("--output",type=Path,default=HERE)
    p.add_argument("--mission-root",type=Path,default=HERE.parent); args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(args.mission_root.resolve()/"scripts"))
    import station_l1_mission as mission
    result=mission.run("reference",.001,sample_every_step=True)
    event=next(e for e in result["events"] if e["event"]=="finite-pipe-envelope-clear-of-shelf-by-1mm")
    row=min(result["trace"],key=lambda r:abs(r["time_s"]-event["time_s"]))
    rows=[row,result["trace"][-1]]
    rows.insert(1,min(result["trace"],key=lambda r:abs(r["time_s"]-(rows[0]["time_s"]+rows[-1]["time_s"])/2)))
    shapes=parts()
    path=args.output/"stereo-mount.step"
    writer=STEPControl_Writer()
    for solid in shapes.values(): writer.Transfer(solid,STEPControl_AsIs)
    if writer.Write(str(path))!=IFSelect_RetDone: raise RuntimeError("write")
    # Canonical LF STEP text lets source pins survive a clean Git checkout.
    path.write_text("\n".join(s.rstrip() for s in path.read_text(encoding="ascii").splitlines())+"\n",
                    encoding="ascii",newline="\n")
    reader=STEPControl_Reader()
    if reader.ReadFile(str(path))!=IFSelect_RetDone or reader.TransferRoots()!=len(shapes):
        raise RuntimeError("reopen")
    reopened={key:reader.Shape(i) for i,key in enumerate(shapes,1)}
    if any(native.solid_count(s)!=1 for s in reopened.values()): raise RuntimeError("invalid output")
    info={"mount_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),"part_order":list(reopened),
          "sample_time_s":row["time_s"],"poses":{}}
    for pose in ("stowed","receive","transport","place"):
        src=ASSEMBLY/(pose+".step")
        inventory=native.read_named(src)
        collisions=[]
        for label,shape in reopened.items():
            if label.startswith("moving-pad-"):
                target=native.MODEL["poses"][pose]
                shape=native.translated(shape,(target["stroke"]-370,0,target["height"]-560))
            bounds=native.bounds(shape)
            for name,body in inventory.items():
                if native.occupied(bounds,native.bounds(body)):
                    volume=native.intersection(shape,body)
                    if volume>1e-4: collisions.append([label,name,round(volume,2)])
        rays=[]
        # Coordinator's pending guide layout. Envelope widths are provisional allocations.
        extras={}
        for sign in (-1,1):
            cy=sign*252
            extras["sheave-and-pin-"+str(sign)]=native.brep(box(-104.6125,cy-17,953,44,34,44))
            extras["upper-bracket-"+str(sign)]=native.brep(box(-92.6125,235 if sign>0 else -245,966,79,10,18))
            for rx in (-103.1125,-62.1125):
                extras["vertical-rope-"+str(sign)+"-"+str(rx)]=native.brep(
                    cylinder(rx,cy,154,(0,0,1),1.5875,821))
        if pose=="place":
            for trace_row in rows:
                for y in (-BASELINE/2,BASELINE/2):
                    eye=(ORIGIN[0],y,ORIGIN[2])
                    for i,(x,cy,z) in enumerate(endpoints(trace_row)):
                        for label,offset in (("axis-end",0),("upper-rim",RADIUS)):
                            rays.append(dict(time_s=trace_row["time_s"],eye_y_mm=y,pipe_end=i,landmark=label,
                                             blockers=hits(inventory,eye,(x,cy,z+offset)),
                                             provisional_guide_blockers=hits(extras,eye,(x,cy,z+offset))))
        info["poses"][pose]={"source_sha256":hashlib.sha256(src.read_bytes()).hexdigest(),
                             "solids":len(inventory),"interferences_mm3":collisions,"optical_rays":rays}
    (args.output/"mount-check.json").write_text(json.dumps(info,indent=2)+"\n",encoding="utf-8",newline="\n")

if __name__=="__main__": main()
