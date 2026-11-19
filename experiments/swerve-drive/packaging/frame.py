"""Native fixed-height swerve chassis interfaces, in robot millimetres."""
from pathlib import Path
import sys
import numpy as np
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from module_placement import HERE, SITES, basis, read, write
ROOT=next(p for p in HERE.parents if (p/"mission/check_branch_exclusion.py").is_file())
sys.path.insert(0,str(ROOT/"drive-side-attachment"))
import joint as j

def transform(shape,m,t):
    mat=np.column_stack((m,t))
    f=gp_Trsf()
    f.SetValues(*mat.reshape(-1).tolist())
    return BRepBuilderAPI_Transform(shape,f,False).Shape()

def register(s,site):
    return transform(s,basis(site),[*SITES[site],38.1])

def cylinder(p,v,r,length):
    return j.cad.brep(["cyl",[*p,*v,r,length]])

def rail_parts(art):
    info=read(ROOT/"mast-attachment/fit.json")
    source=ROOT/"mast-attachment/mast-joints.step"
    if j.digest(source)!=info["stepSha256"]:raise ValueError("Mast rail revision")
    src=j.cad.read_named(source,[{"id":k} for k in info["volumeMm3"]])
    deck_src=ROOT/"mast-attachment/slotted-ribbed-deck.step"
    if j.digest(deck_src)!=info["ribbedSlottedSha256"]:raise ValueError("Deck revision")
    deck_old=j.cad.read_named(deck_src,[{"id":"mast-slotted-ribbed-deck"}])["mast-slotted-ribbed-deck"]
    deck=deck_old
    outputs={}
    delta={}
    for site,(cx,cy) in SITES.items():
        sx,sy=int(np.sign(cx)),int(np.sign(cy))
        # Only motor bodies reach the deck. Round motor apertures with lead allowance.
        # Keep the outer land and all six deck ties.
        for x,z,r in ((0,0,53),(29.5,36.6,34)):
            center=basis(site)@np.array([x,0,z])+np.array([cx,cy,0])
            deck=j.cut(deck,cylinder([center[0],center[1],142],[0,0,1],r,15))
    outputs["swerve-relieved-ribbed-deck"]=deck
    delta["deckRemovedMm3"]=j.cad.volume(deck_old)[0]-j.cad.volume(deck)[0]
    for side,sy in (("L",1),("R",-1)):
        source=src["mast-base-"+side+"-rail"]
        clipped=j.boolean(BRepAlgoAPI_Common,source,j.box(-145,145,-400,400,40,160))
        for end,sx in (("front",1),("rear",-1)):
            # Fixed original ear and drilled end shift inboard 75 mm. The end
            # retains the registered quarter-turn fasteners/pilot and original end clearance.
            inner=j.boolean(BRepAlgoAPI_Common,source,j.box(280*sx,331*sx,-400,400,40,160))
            inner=j.cad.translated(inner,[0,-75*sy,0])
            points=[(145,229),(183,142),(260,142),(280,154),(280,160),
                    (260,160),(183,160),(145,235)]
            inner_points=[(149,224.53),(184.31,144),(259.45,144),(276.66,154.33),
                          (276.66,158),(184.0,158),(149,225.27)]
            def shell(pts,z,h):
                return j.extrude([(sx*x,sy*y,z) for x,y in pts],(0,0,h))
            brace=j.cut(shell(points,50,100),shell(inner_points,52,96))
            clipped=j.fuse(j.fuse(clipped,brace),inner)
        key="swerve-cranked-mast-rail-"+side
        outputs[key]=clipped
        delta[key+"ChangedMm3"]=j.cad.volume(clipped)[0]-j.cad.volume(source)[0]
    return outputs,delta

def local_mount():
    # Source x is perpendicular to this rail, z runs along it, y is upward.
    sadd=j.box(-48,-23.5,31.80,82.50,52.0,87.7)
    mounting={}
    for a in (57.15,82.55):
        sadd=j.cut(sadd,cylinder([-31.75,30,a],[0,1,0],2.65,55))
        shaft=cylinder([-31.75,27.45,a],[0,1,0],2.413,66.68)
        head=cylinder([-31.75,24.75,a],[0,1,0],4.6,2.70)
        mounting["tab-screw-"+str(a)]=j.fuse(head,shaft)
        mounting["tab-locknut-"+str(a)]=j.cut(cylinder([-31.75,86.25,a],[0,1,0],4.55,4.5),
                                                cylinder([-31.75,86,a],[0,1,0],2.43,6))
    for uy in (46,67):
        sadd=j.cut(sadd,cylinder([-49,uy,69.85],[1,0,0],3.25,28))
        screw=j.fuse(cylinder([-59.5,uy,69.85],[1,0,0],3,36),
                    cylinder([-23.5,uy,69.85],[1,0,0],5,6))
        mounting["rail-screw-"+str(uy)]=screw
        mounting["welded-thread-boss-"+str(uy)]=j.cut(cylinder([-61,uy,69.85],[1,0,0],6.5,13),
                                              cylinder([-62,uy,69.85],[1,0,0],3.01,15))
    for a in (57.8,81.9):
        tool=cylinder([-49,57.15,a],[1,0,0],5.35,28)
        sadd=j.cut(sadd,tool)
    mounting["tab-saddle"]=sadd
    return mounting

def chassis(art):
    items,delta=rail_parts(art)
    ties=read(ROOT/"deck-joints/fit.json")
    tied=j.cad.read_named(ROOT/"deck-joints/deck-through-ties.step",[{"id":k} for k in ties["volumeMm3"]])
    sides=read(ROOT/"frame-corners/weld-fit.json")
    side_ids=["cross-front","cross-rear","drive-rail-outer-L","drive-rail-outer-R"]
    side_ids += ["corner-"+corner+"-"+seam for corner in sides["corners"]
                 for seam in ("fillet","groove","bottom-cap","top-cap")]
    frame=j.cad.read_named(ROOT/"frame-corners/frame-weldment.step",[{"id":k} for k in side_ids])
    tubes={k:s for k,s in frame.items() if k in ("drive-rail-outer-L","drive-rail-outer-R")}
    tubes.update({k:s for k,s in tied.items() if k.startswith("tie-machined-cross-")})
    originals={k:j.cad.volume(s)[0] for k,s in tubes.items()}
    for site in SITES:
        for tag,tr in (("A",np.eye(3)),("B",np.array([[0,0,1],[0,1,0],[1,0,0.]]))):
            axis=basis(site)@tr@np.array([1,0,0])
            tube=("tie-machined-cross-"+("front" if SITES[site][0]>0 else "rear") if abs(axis[0])>.9
                  else "drive-rail-outer-"+("L" if SITES[site][1]>0 else "R"))
            for uy in (46,67):
                bore=cylinder([-63,uy,69.85],[1,0,0],6.55,16)
                tubes[tube]=j.cut(tubes[tube],register(transform(bore,tr,[0,0,0]),site))
            for name,shape in local_mount().items():
                items[f"{site}-{tag}-{name}"]=register(transform(shape,tr,[0,0,0]),site)
    for k,s in tubes.items():
        items["swerve-bored-"+k]=s
        delta[k+"RemovedMm3"]=originals[k]-j.cad.volume(s)[0]
    return items,delta

def run(out,art):
    out.mkdir(parents=True,exist_ok=True)
    items,delta=chassis(art)
    step=out/"swerve-frame.step"
    doc,table=j.cad.xcaf_doc()
    for k,s in items.items():
        label=table.AddShape(s,False)
        j.cad.TDataStd_Name.Set_s(label,j.cad.TCollection_ExtendedString(k))
    writer=j.cad.STEPCAFControl_Writer()
    if not writer.Transfer(doc,j.cad.STEPControl_AsIs) or writer.Write(str(step))!=j.cad.IFSelect_RetDone:
        raise ValueError("Frame STEP export failed")
    j.finalize_step(step)
    actual=j.cad.read_named(step,[{"id":k} for k in items])
    for k,s in actual.items():
        if j.cad.solid_count(s)!=1 or abs(j.cad.volume(s)[0]-j.cad.volume(items[k])[0])>.005:
            raise ValueError("Independent STEP reopen mismatch: "+k)
    data={k:{"boundsMm":[round(x,5) for x in j.cad.bounds(s)],
             "volumeMm3":round(j.cad.volume(s)[0],4)} for k,s in actual.items()}
    write(out/"frame.json",{"stepSha256":j.digest(step),"parts":data,"stockChange":delta})
if __name__=="__main__":run(Path(sys.argv[1]),Path(sys.argv[2]))
