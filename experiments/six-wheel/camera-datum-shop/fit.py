"""Check the new moving stock through travel, through service and against occupied reef balls."""
import argparse,json,math,sys
from pathlib import Path
from camera_parts import HERE,ROOT,cad,SHELF,X_BOLTS,cylinder,read,sha
from design import mount
from replay_candidate import model,mod
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

def assembly(lamp):
    old=model(lamp)
    removed={k for k in old if k.startswith("roof-") or k==SHELF}
    if len(removed)!=22:raise ValueError("Different base camera configuration")
    parts=cad.read_named(HERE/"camera-shelf.step",[{"id":k} for k in read(HERE/"native.json")["parts"]])
    base={k:v for k,v in old.items() if k not in removed}
    full={**base,**{("roof-"+k if k!=SHELF else k):cad.translated(v,[370,0,0]) for k,v in parts.items()}}
    return base,parts,full

def swept(box,axis,d):
    b=list(box);b[axis]+=min(0,d);b[axis+3]+=max(0,d);return b
def clean_distance(a,b):
    d=BRepExtrema_DistShapeShape(a,b)
    if not d.IsDone():raise ValueError("Native distance failed")
    return d.Value()
def relation(shape,group):
    b=cad.bounds(shape)
    nearest=math.inf;closest=None
    for k,v in sorted(group.items(),key=lambda kv:mount.limits(b,cad.bounds(kv[1]))):
        if mount.limits(b,cad.bounds(v))>nearest:break
        d=clean_distance(shape,v)
        if d<nearest:nearest=d;closest=k
    return {"gap_mm":nearest,"body":closest}

def sweep_clearance(shape, obstacle, upper, reserve):
    # Distance between two rigid shapes changes by at most the Cartesian translation.
    # A positive midpoint distance minus half-interval length certifies the whole interval.
    stack=[(0.,upper)]
    smallest=math.inf
    while stack:
        a,b=stack.pop();mid=(a+b)/2
        part=cad.translated(shape,[0,0,mid])
        d=clean_distance(part,obstacle)
        guaranteed=d-(b-a)/2-reserve
        if guaranteed>.01:
            smallest=min(smallest,guaranteed)
            continue
        if d<=reserve or b-a<.01:
            return {"certified":False,"lift_mm":mid,"distance_mm":d,"overlap_mm3":cad.intersection(part,obstacle)}
        stack.extend([(a,mid),(mid,b)])
    return {"certified":True,"lower_mm":smallest}


def run(lamp,output=None):
    base,kit,full=assembly(lamp)
    frames=read(ROOT/"design/competition-package/shop-duplex/red/part-frames.json")
    rots=read(ROOT/"design/competition-package/shop-duplex/red/rotation-bounds.json")
    obstacles={}
    for n,v in base.items():
        f={"slide":1.,"middle":.5}.get(frames.get(n),0.)
        box=list(cad.bounds(v))
        if n in rots:
            b=rots[n]["bounds_mm"];box=[b[0]+370*f,b[1],b[2]+560,b[3]+370*f,b[4],b[5]+560]
        init=cad.translated(v,[-370*f,0,0])
        box[0]-=370;box[3]-=370*f
        ey=1.5 if n.startswith(("grip-L-","grip-R-")) else 0
        box[1]-=ey;box[4]+=ey
        obstacles[n]=(f,init,box)
    crossings=[];positive=[];checks=0
    for k,v in kit.items():
        if k==SHELF:continue # All changes in the original shelf remove material.
        b=cad.bounds(v)
        for n,(f,init,sw) in obstacles.items():
            checks+=1
            if mount.limits(b,sw)>1e-6:continue
            if f!=1:crossings.append([k,n,"different moving frame"]);continue
            vol=cad.intersection(v,init)
            if vol>.002:positive.append([k,n,vol])
    flex=read(HERE/"mechanics.json")["derived_2g"]["summed_tip_mm"]
    field=mod("shop_reef",ROOT/"bridge-tolerance/field.py")
    earlier=read(ROOT/"bridge-tolerance/field.json")
    ox=read(ROOT/"full-match/installed-stock.json")["stock"]["NAUO201"]["native_center_mm"]
    sectors={}
    for mm in (-180,220):
        local=(ox[0]-(3453.050980729168-490),-ox[2]-mm,ox[1])
        gap,k=min((field.robust(cad.bounds(v),local,flex),k) for k,v in kit.items() if k!=SHELF)
        # All other robot bodies retain the more conservative complete-robot #71 lower bound.
        other=earlier["sectors"][str(mm/1000)]["whole_target_stock"]["reef-ball"]["after_translation_and_camera_allowance_mm"]
        sectors[mm]={"robust_kit_mm":gap,"closest":k,"combined_lower_mm":min(gap,other)}
    service={}
    base_at_stow={n:v for n,(_,v,_) in obstacles.items()}
    for tag,sign in (("left",1),("right",-1)):
        for x in X_BOLTS:
            key=f"{tag}-M4x8-{x:g}"
            tool=cylinder(x,sign*200,505.3,1.8,115)
            # The service hex enters the screw's socket. Its swept access starts at the screw head.
            service[key]=relation(tool,{**base_at_stow,**{n:v for n,v in kit.items() if n!=key}})
    for y in (-37.5,37.5):
        k=f"camera-M4-{y:g}"
        tool=cylinder(48,y,865,1.8,115)
        service[k]=relation(tool,{**base_at_stow,**{n:v for n,v in kit.items() if n!=k}})
    bolt_audit={}
    for k,v in kit.items():
        if "-M4" not in k:continue
        camera=k.startswith("camera-")
        clear=3.6 if camera else 5.1
        pins=[s for n,s in kit.items() if ("camera-pin-"+k.split("M4-")[-1]+"-") in n] if camera else []
        if not camera:
            tag=k.split("-M4")[0];x=k.split("M4x8-")[1]
            pins=[s for n,s in kit.items() if n.startswith(f"{tag}-foot-pin-{x}-")]
        if len(pins)!=2:raise ValueError(k)
        lifted=cad.translated(v,[0,0,clear])
        # Prove actual positive contact if the head is pulled above its captured limit.
        excessive=cad.translated(v,[0,0,clear+3])
        bolt_audit[k]={"free_unscrew_rise_mm":clear,"usable_pin_gap_mm":min(clean_distance(lifted,p) for p in pins),
            "extraction_counterexample_volume_mm3":sum(cad.intersection(excessive,p) for p in pins)}
    # The four feet withdraw along their vertical locator axes. Use the conservative complete 25 mm swept boxes
    # against all non-cartridge bodies. Same-frame exclusions that touch at the initial datum are checked separately.
    retained={n:v for n,v in kit.items() if "saddle" in n or "-M3x20-" in n or n==SHELF}
    cartridge={n:v for n,v in kit.items() if n not in retained}
    remove_hits=[]
    certificates=[]
    lift=math.ceil(max(cad.bounds(s)[5] for s in base_at_stow.values())-min(cad.bounds(s)[2] for s in cartridge.values()))+10
    for n,v in cartridge.items():
        released=cad.translated(v,[0,0,3.6 if n.startswith("camera-M4") else 5.1]) if "-M4x8" in n else v
        b=swept(cad.bounds(released),2,lift)
        for host,s in base_at_stow.items():
            reserve=1.5 if host.startswith(("grip-L-","grip-R-")) else 0.
            if mount.limits(b,cad.bounds(s))>reserve+.01:continue
            cert=sweep_clearance(released,s,lift,reserve)
            certificates.append({"pair":[n,host],**cert})
            if not cert["certified"]:remove_hits.append([n,host])
    source=cad.read_named(ROOT/"bridge-tolerance/moving-bridge.step",[{"id":k} for k in (*read(ROOT/"bridge-tolerance/mount.json")["parts"],SHELF)])
    counter={}
    for side,tag in (("left","L"),("right","R")):
        oldfoot=cad.translated(source[side+"-foot"],[0,0,25])
        counter[side]=cad.intersection(oldfoot,base_at_stow["grip-"+tag+"-dashpot-housing"])
    r={"combined_native_count":len(full),"retained_source_bodies":len(base),"registered_sweep_pairs":checks,
       "unresolved_different_frame":crossings,"positive_solid_intersections":positive,"tool_access":service,"capture":bolt_audit,
       "vertical_removal_mm":lift,"vertical_removal_unresolved":remove_hits,"refined_vertical_removal":certificates,
       "issued_foot_dashpot_overlap_at_25mm_mm3":counter,"ball_tolerance":sectors,"shelf_only_loses_material":True}
    (HERE/"fit.json").write_text(json.dumps(r,indent=2)+"\n",encoding="utf8",newline="\n")
    if output is not None:
        from tempfile import TemporaryDirectory
        export=mod("shop_native_export",ROOT/"design/fixed-no-air/export_robot.py")
        row=read(ROOT/"design/fixed-no-air/whole-robot.json")["cases"]["place"]
        output.parent.mkdir(parents=True,exist_ok=True)
        with TemporaryDirectory(dir=output.parent) as scratch:
            local=Path(scratch)/output.name
            export.write_assembly(local,full)
            native=export.verify(local,full,row,"place")
            local.replace(output)
        native["shop_kit_sha256"]=sha(HERE/"camera-shelf.step")
        (HERE/"native-place.json").write_text(json.dumps(native,indent=2)+"\n",encoding="utf8",newline="\n")
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--lamp",type=Path,required=True)
    p.add_argument("--place-step",type=Path)
    a=p.parse_args();run(a.lamp,a.place_step)
