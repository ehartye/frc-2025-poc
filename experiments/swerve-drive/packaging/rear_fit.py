"""Local machined relief and an elevated removable board bridge for the two rear motors."""
from pathlib import Path
import math,sys
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from frame import HERE,ROOT,j,cylinder,transform,register
from module_placement import read,write
from envelopes import load_raw
from fit import indexed
RISE=23.
SITES=[(-288,-209),(-222,-209),(-288,-177),(-222,-177)]
def up(s,z=RISE):return j.cad.translated(s,[0,0,z])
def bore(s,x,y,z=135,r=2.25):
    return j.cut(s,cylinder([x,y,z],[0,0,1],r,55))
def bar(a,b,r):
    a=np.asarray(a);b=np.asarray(b);v=b-a
    return cylinder(a,v/np.linalg.norm(v),r,float(np.linalg.norm(v)))
def endpoints(t):
    ff=TopExp_Explorer(t,TopAbs_FACE)
    while ff.More():
        sf=BRepAdaptor_Surface(TopoDS.Face(ff.Current()));ff.Next()
        if sf.GetType()!=GeomAbs_Cylinder:continue
        ax=sf.Cylinder().Axis()
        p=np.array(ax.Location().Coord());u=np.array(ax.Direction().Coord())
        ends=sorted([p+sf.FirstVParameter()*u,p+sf.LastVParameter()*u],key=lambda x:x[2])
        return ends,sf.Cylinder().Radius()
    raise ValueError("Expected original guide strut")
def original_board():
    yaw=math.radians(-15)
    t=np.array([math.cos(yaw),math.sin(yaw),0]);b=np.array([-t[1],t[0],0]);z=np.array([0,0,1.])
    m=np.column_stack([t,b,z]);o=np.array([-249.,-200.,0]);mouth=o+45.5*t-b
    return {"host-board-and-components":transform(j.box(-42.5,45.5,-28,28,163.5,181.1),m,o),
      "usb-A-fixed-boot":transform(j.box(0,35,-7.75,7.75,-3.75,3.75),m,mouth+169*z),
      "usb-A-roof-boot":transform(j.box(0,38,-8,8,-4,4),m,mouth+177*z)}
def run(art,check_fit=True):
    base=indexed(art)
    old={k:base[k] for k in ("electrical-backplate","observer-pi-and-converter-backplate")}
    guides_meta=read(ROOT/"fixed-camera-route/guides.json")
    gs=j.cad.read_named(ROOT/"fixed-camera-route/harness-guides.step",[{"id":k} for k in guides_meta["volumeMm3"]])
    gear=read(HERE/"frame.json")
    frame=j.cad.read_named(HERE/"swerve-frame.step",[{"id":k} for k in gear["parts"]])
    deck=frame["swerve-relieved-ribbed-deck"]
    native=load_raw(art)
    motors={side:register(native[1][1],side) for side in ("RL","RR")}
    parts={}
    plate=j.cut(old["electrical-backplate"],j.box(-277,-228,219.8,229,153,159))
    parts["edge-pocket-electrical-backplate"]=plate
    lower=j.cut(old["observer-pi-and-converter-backplate"],j.box(-266,-225,-234,-223,153,158))
    upper=j.box(-299,-215,-234,-159.5,178,180)
    # Four existing board standoffs and guide seat move 23 mm, maintaining board datums.
    holes=read(ROOT/"fixed-camera-route/qualification.json")["piHoleXYmm"]
    for x,y in holes: upper=bore(upper,x,y,145,1.45)
    for x in (-280,-270):upper=bore(upper,x,-226,145,1.65)
    slots={}
    for idx,(x,y) in enumerate(SITES,1):
        lower=bore(lower,x,y)
        upper=bore(upper,x,y)
        deck=bore(deck,x,y)
        parts[f"host-upright-{idx}"]=bore(cylinder([x,y,157],[0,0,1],4.5,21),x,y)
        parts[f"host-through-screw-{idx}"]=j.fuse(cylinder([x,y,140],[0,0,1],2,40),cylinder([x,y,180],[0,0,1],3.75,3))
        parts[f"host-washer-{idx}"]=j.cut(cylinder([x,y,149.2],[0,0,1],4.5,.8),cylinder([x,y,149],[0,0,1],2.15,2))
        parts[f"host-nut-{idx}"]=j.cut(cylinder([x,y,145.2],[0,0,1],4.05,4),cylinder([x,y,145],[0,0,1],2.01,5))
    for key,s in gs.items():
        if key.startswith("host-post-") or key=="base-guide-foot":parts["lifted-"+key]=up(s)
        elif key.startswith("base-guide-rib-"):
            ends,r=endpoints(s)
            preferred=ends[0]+[0,0,RISE]
            parts["routed-"+key]=bar(preferred,ends[1],r)
            slots[key]={"oldBase":ends[0].tolist(),"newBase":preferred.tolist(),"unchangedLug":ends[1].tolist()}
    parts["edge-pocket-host-converter-backplate"]=lower
    parts["elevated-host-plate"]=upper
    parts["host-drilled-swerve-deck"]=deck
    boards={k:up(s) for k,s in original_board().items()}
    native_hits={};gaps={}
    for tag,m in (motors.items() if check_fit else []):
        vals={}
        rb=j.cad.bounds(m)
        mb=j.box(rb[0],rb[3],rb[1],rb[4],rb[2],rb[5])
        for k,s in {**parts,**boards}.items():
            if j.lower_bound(j.cad.bounds(s),j.cad.bounds(m))>95:continue
            bound=j.lower_bound(j.cad.bounds(s),j.cad.bounds(m))
            if bound>1.0:
                vals[k]=round(bound,4)
                continue
            # A strict separation for the entire motor AABB encloses the
            # supplier's body, leads and every intermediate downward pose.
            v=j.cad.intersection(mb,s);gap=j.distance(mb,s)
            if v>.01:
                v=j.cad.intersection(m,s);gap=j.distance(m,s)
            vals[k]=round(gap,4)
            if v>.01:native_hits[tag+"/"+k]=round(v,4)
        gaps[tag]=vals
    supports={}
    for i,(x,y) in enumerate(SITES,1):
        touch=bore(cylinder([x,y,153.8],[0,0,1],4.5,.2),x,y)
        supports[str(i)]=j.cad.intersection(deck,touch)
    out=HERE/"rear-electronics.step"
    j.write_step(out,parts);j.finalize_step(out)
    reopened=j.cad.read_named(out,[{"id":k} for k in parts])
    qty={k:round(j.cad.volume(s)[0],5) for k,s in reopened.items()}
    displaced={"edge-pocket-electrical-backplate":j.cad.volume(old["electrical-backplate"])[0],
               "edge-pocket-host-converter-backplate":j.cad.volume(old["observer-pi-and-converter-backplate"])[0],
               "host-drilled-swerve-deck":j.cad.volume(frame["swerve-relieved-ribbed-deck"])[0]}
    for k,s in gs.items():
        if "lifted-"+k in parts:displaced["lifted-"+k]=j.cad.volume(s)[0]
        if "routed-"+k in parts:displaced["routed-"+k]=j.cad.volume(s)[0]
    mass=sum((vol-displaced.get(k,0))*(7850 if k.startswith(("host-nut","host-washer","host-through","routed-")) else 2700)/1e9 for k,vol in qty.items())
    gross=sum(max(0,vol-displaced.get(k,0))*(7850 if k.startswith(("host-nut","host-washer","host-through","routed-")) else 2700)/1e9 for k,vol in qty.items())
    write((HERE/"rear-electronics.json") if check_fit else (art/"rear-provisional.json"),
        {"stepSha256":j.digest(out),"originalChassis":gear["stepSha256"],
        "materialHitsWithRearMotorsMm3":native_hits,"nativeGapMm":gaps,"existingDeckBearingPressMm3AtPointTwo":supports,
        "originalHardwareConflicts":read(HERE/"backplate-control.json"),"outboardPlateNotchesMm":{"RL":[-277,-228,219.8,229],"RR":[-266,-225,-234,-223]},
        "guideEndpoints":slots,"hostPortAndPostRiseMm":RISE,"partsMm3":qty,
        "boundsMm":{k:j.cad.bounds(s) for k,s in {**reopened,**boards}.items()},"matchedGeometryNominalAddedKg":mass,
        "extraCeilingKg":max(0,mass*1.10)+.08,"noChipUpperKg":gross*1.10+.08,
        "ceilingAssumptions":"10% material tolerance plus 80 g allocation for the two longer host data leads, termination and finishes. Converter and independent camera power outlets stay stationary.",
        "physicalScaleKg":None})
if __name__=="__main__":run(Path(sys.argv[1]),"--export-only" not in sys.argv[2:])
