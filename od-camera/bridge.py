"""Project-owned rear-supported overhead camera bridge for the outside-diameter receiver."""
from assembly_review import *
EYE=(75.,0.,895.)
PITCH=0.
KEEP=("camera-envelope","camera-M4--37.5","camera-M4-37.5")
GATE_REMOVE={"anchor-L--80","anchor-R--80"}

def hollow(x,y,z,dx,dy,dz,axis,wall=.8):
    dims=[d-2*wall for d in (dx,dy,dz)];dims[axis]=[dx,dy,dz][axis]+2
    loc=[v+wall for v in (x,y,z)];loc[axis]-=1+wall
    return subtract(box(x,y,z,dx,dy,dz),box(*loc,*dims))

def camera_pose(shape):
    rot=rotate(shape,(98,0,842),AY,PITCH)
    return cad.translated(rot,(EYE[0]-98,0,EYE[2]-842))

def structure(roof):
    result={}
    for sign in (-1,1):
        side="L" if sign==1 else "R"
        y=sign*200.5
        raw=box(-121,min(sign*172,sign*207.5),489,51,35.5,2.5)
        foot=subtract(raw,rod(-80,sign*180,488,AZ,2.7,5))
        result["OD-bridge-foot-"+side]=foot
        result["OD-long-anchor-"+side]=gate.fuse(rod(-80,sign*180,474.5,AZ,2.45,17),rod(-80,sign*180,491.5,AZ,4,2.5))
        result["OD-bridge-post-"+side]=hollow(-119,y-7.5,491.5,15,15,923-491.5,2)
        result["OD-roof-boom-"+side]=hollow(-104,sign*49-6,923,192,12,12,0)
    result["OD-roof-crossbar"]=hollow(-119,-207.5,923,15,415,15,1)
    plate=box(70,-56,859.5,42,112,3)
    for y in (-37.5,37.5):plate=subtract(plate,rod(98,y,858,AZ,2.25,7))
    result["OD-camera-plate"]=camera_pose(plate)
    for sign in (-1,1):
        side="L" if sign==1 else "R"
        anchor=cad.bounds(result["OD-camera-plate"])
        # Cut the two vertical straps against their actual tilted plate faces.
        raw=box(74,sign*49-4,anchor[2],6,8,923-anchor[2])
        above=rotate(box(-100,-200,862.5,400,400,100),(98,0,842),AY,PITCH)
        above=cad.translated(above,(EYE[0]-98,0,EYE[2]-842))
        from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
        result["OD-camera-hanger-"+side]=BRepAlgoAPI_Common(raw,above).Shape()
    for k in KEEP:result["roof-"+k]=camera_pose(roof[k])
    return result

def configuration(stroke,opening,rise,kit):
    old=baseline(stroke,kit); allgate=gate.shapes(stroke,opening,rise)
    retired_set=retired(old)|{k for k in old if k.startswith("roof-")}
    unchanged={k:v for k,v in old.items() if k not in retired_set}
    add={k:v for k,v in allgate.items() if k not in GATE_REMOVE}
    bridge={k:cad.translated(v,(stroke,0,0)) for k,v in structure(kit[0]).items()}
    if set(unchanged)&(set(add)|set(bridge)) or set(add)&set(bridge):raise ValueError("Part ID reused")
    return unchanged|add|bridge

def evaluate(kit):
    layers=structure(kit[0]);ids=[{"id":k,"group":"camera"} for k in layers]
    if any(cad.solid_count(s)!=1 for s in layers.values()):raise ValueError("Disconnected bridge body")
    step=HERE/"revised-bridge.step"
    cad.write_named(step,list(layers.values()),ids,{"camera":"#398083"})
    finalize_step(step);restored=cad.read_named(step,ids)
    intersections={};surfaces={}
    for a,s in restored.items():
        for b,t in restored.items():
            if a<b and interior(cad.bounds(s),cad.bounds(t)):
                v=cad.intersection(s,t)
                if v>.05:intersections[a+" / "+b]=v
    for x in (0,185,370):
        allold=baseline(x,kit);surfaces[x]={}
        other={k:v for k,v in allold.items() if k not in retired(allold) and not k.startswith("roof-")}
        others={k:cad.bounds(s) for k,s in other.items()}
        for k,s in restored.items():
            at=cad.translated(s,(x,0,0));b=cad.bounds(at)
            for n,obj in other.items():
                if interior(b,others[n]) and (v:=cad.intersection(at,obj))>.05:surfaces[x][k+" / "+n]=v
    gatehits={}
    for state,op,h in ([("gate",q,0) for q in range(0,113,7)]+[("hood",0,q) for q in range(0,41,4)]):
        other={k:s for k,s in gate.shapes(0,op,h).items() if k not in GATE_REMOVE}
        boxes={k:cad.bounds(s) for k,s in other.items()}
        hits={}
        for k,s in restored.items():
            b=cad.bounds(s)
            for n,t in other.items():
                if interior(b,boxes[n]) and (v:=cad.intersection(s,t))>.05:hits[k+" / "+n]=v
        gatehits[f"{state}-{max(op,h)}"]=hits
    old=baseline(370,kit);new=configuration(370,112,0,kit)
    keys=set(old)&set(new)
    return {"old":len(old),"new":len(new),"identicalAtPlacement":sorted(k for k in keys if whole.fingerprint(old[k])==whole.fingerprint(new[k])),
            "retired":sorted(set(old)-set(new)),"added":sorted(set(new)-set(old)),
            "relocated":sorted(k for k in keys if whole.fingerprint(old[k])!=whole.fingerprint(new[k])),
            "gateExcluded":sorted(GATE_REMOVE),"nativeSha":sha(step),"nativeVolumesMm3":{k:cad.volume(s)[0] for k,s in restored.items()},
            "withinBridge":intersections,"on593Unchanged":surfaces,"onGate":gatehits}
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--lamp",type=Path,required=True);a=p.parse_args()
    (HERE/"native.json").write_text(json.dumps(evaluate(source(a.lamp)),indent=2)+"\n",encoding="utf8",newline="\n")
