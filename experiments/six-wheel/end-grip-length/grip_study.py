"""Use the low-pose Java optical gate with finite end patches on the two independently sprung shoes."""
from contextlib import ExitStack
from dataclasses import dataclass,replace
import argparse
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from unittest.mock import patch

from shoe import BACK, FRONT, Design, HALF, LIMITS, load

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"observable-acquisition"))
import study
import pipe_axial_contact as axial

@dataclass(frozen=True)
class Trial:
    name: str
    cut: float = 0
    side: float = 0
    yaw: float = 0
    depth: float = 0
    friction: float = .4
    spring: float = 24
    phase: int = 0
    step: float = .0005
    pad: float = 2.
    setup: float = 0
    original: bool = False
    cassette: bool = False
    pressure_cap: float = 0.67
    open_until_ms: float | None = None
    jaw_mass: float = .50
    drag_ns_m: float = 10.
    indexed_center: float | None = None
    indexed_in: float = .35

def contact_law(case,history,carried=None):
    design=Design(spring_preload=case.spring)
    half=HALF+case.cut/2
    # Precompute restoring force along the full supported 12 mm travel.
    table=[load(LIMITS[0]+j*.1,design) for j in range(121)]
    def spring(travel):
        if case.cassette:return case.spring
        j=min(119,max(0,math.floor((travel+6)*10)))
        fraction=(travel+6)*10-j
        return table[j]+fraction*(table[j+1]-table[j])
    lower=case.indexed_center-case.indexed_in if case.indexed_center is not None else -6.
    upper=case.indexed_center+1.4 if case.indexed_center is not None else 6.
    pos=[lower,lower] if case.indexed_center is not None else [6.,6.]
    vel=[0.,0.]
    if carried is not None:
        pos=list(carried["positionMm"])
        vel=list(carried["speedMps"])
    previous=[None]
    def normals(self,u,v,yaw,stage,instant,zrel=0):
        angle=math.radians(case.yaw)+yaw
        lateral=case.side+1000*v
        points=[]
        forces=[]
        offsets=[]
        dt=0 if previous[0] is None else instant-previous[0]
        previous[0]=instant
        for i,side in enumerate((-1,1)):
            # The sensor plane is centered; actual yaw/lateral are imposed orientations for
            # testing the contact and native clearance, not an actuated centering constraint.
            strip=axial.patches(u-side*half*.001*math.sin(angle),zrel,self.cfg)
            nodes=[(x,z,area,side*lateral-case.setup+half*math.cos(angle)-HALF+side*1000*x*math.sin(angle))
                   for x,z,area in strip]
            travel=-6.
            if nodes and case.open_until_ms is None:
                lo,hi=LIMITS
                for _ in range(12):
                    mid=(lo+hi)/2
                    reaction=sum(area*1e6*min(case.pressure_cap,case.pad*max(0,cap-mid)) for _,_,area,cap in nodes)
                    if reaction>spring(mid):lo=mid
                    else:hi=mid
                travel=(lo+hi)/2
            if case.open_until_ms is not None:
                travel=pos[i]
            weighted=[(x,z,area*1e6*min(case.pressure_cap,case.pad*max(0,cap-travel))) for x,z,area,cap in nodes]
            total=sum(n for _,_,n in weighted)
            if case.open_until_ms is not None and 1000*instant>=100+case.phase+case.open_until_ms:
                accel=(total-spring(travel)-case.drag_ns_m*vel[i])/case.jaw_mass
                vel[i]+=dt*accel
                pos[i]+=dt*1000*vel[i]
                if pos[i]>=upper or pos[i]<=lower:
                    pos[i]=min(upper,max(lower,pos[i]))
                    vel[i]=0.
            # An unresolved compliant face pressure at the inward stop cannot exceed its
            # instantaneous support from positive preload. Outward-stop loading is a hold.
            if travel>upper-.01 and total>spring(upper)+.15:history["outerStop"]=True
            points.append([(x,z,n) for x,z,n in weighted if n>0]);forces.append(total);offsets.append(travel)
            if nodes and total>.1:
                history["travel"][0]=min(history["travel"][0],travel)
                history["travel"][1]=max(history["travel"][1],travel)
                history["peakNormal"]=max(history["peakNormal"],total)
        self.points=tuple(points)
        if carried is None:
            history["samples"].append((instant,1000*u,500+1000*(study.R+zrel),*offsets))
        if stage=="place" and self.start is None:self.start=instant
        return *forces,self.cfg.belt_mu,1.05,self.cfg.shelf_mu
    return normals

def trials():
    out=[Trial("issued",original=True),Trial("nominal"),Trial("weak",friction=.08,spring=9.),
         Trial("low-preload-short",cut=-6.35,spring=17.),Trial("soft-coupon",pad=.5),
         Trial("hard-coupon",pad=6.),Trial("fine",step=.00025),Trial("late",phase=19)]
    for cut,side,yaw,depth in itertools.product((-6.35,0,6.35),(-1.,1.),(-.25,.25),(-5.,5.)):
        out.append(Trial(f"box-{len(out):02}",cut,side,yaw,depth,phase=19 if depth>0 else 0))
    for cut,side,yaw,depth in itertools.product((-6.35,0,6.35),(-1.,1.),(-.25,.25),(-5.,5.)):
        out.append(Trial(f"cassette-{len(out):02}",cut,side,yaw,depth,phase=19 if depth>0 else 0,
                         spring=(1.97+.32)*4.4482216152605,cassette=True))
    for name,params in (("center",{}),("fine",{"step":.00025}),("weak",{"friction":.08}),
                        ("stiff",{"pad":6.}),("high-pressure",{"pressure_cap":2.}),
                        ("short",{"cut":-6.35}),("long",{"cut":6.35}),
                        ("shortest-worst",{"cut":-6.35,"side":1.5,"yaw":.5,"setup":.5}),
                        ("longest-worst",{"cut":6.35,"side":1.5,"yaw":.5,"setup":-.5})):
        out.append(Trial("fixed-"+name,spring=(1.97+.32)*4.4482216152605,cassette=True,**params))
    for force in (7.,8.76,9.16,9.5,9.8,10.,10.2,10.5,11.,2.63*4.4482216152605):
        out.append(Trial("preload-"+str(round(force,3)),cassette=True,spring=force))
    for cut,ms in itertools.product((-6.35,0,6.35),(170,180,190,200,210,220)):
        out.append(Trial(f"timed-{cut:g}-{ms}",cut=cut,spring=(1.97+.32)*4.4482216152605,cassette=True,open_until_ms=ms))
    force=(1.97+.32)*4.4482216152605
    for cut,depth,side,yaw in itertools.product((-6.35,0,6.35),(-5.,5.),(-1.,1.),(-.25,.25)):
        out.append(Trial(f"indexed-box-{len(out):02}",cut,side,yaw,depth,spring=force,cassette=True,
                         open_until_ms=0,indexed_center=cut/2,pad=.5))
    for cut,idx,offset in itertools.product((-6.35,0,6.35),(-6.35,0,6.35),(-.5,0,.5)):
        out.append(Trial(f"gauge-{cut:g}-{idx:g}-{offset:g}",cut=cut,cassette=True,spring=force,open_until_ms=0,
                         indexed_center=(idx+offset)/2,pad=.5))
    for reach,pad in itertools.product((.35,.65,1.,1.4),(.10,.25,.5,1.,2.)):
        out.append(Trial(f"coupon-{reach:g}-{pad:g}",cassette=True,spring=force,open_until_ms=0,indexed_center=0,indexed_in=reach,pad=pad))
    for mass,drag in itertools.product((.42,.70),(10.,37.52)):
        out.append(Trial(f"mass-{mass}-{drag}",cassette=True,spring=force,open_until_ms=0,indexed_center=0,pad=.5,jaw_mass=mass,drag_ns_m=drag))
    return out

def work(jdk,cases,keep=False,speed=.65):
    suffix=".exe" if os.name=="nt" else ""
    gate=study.HERE/"PortAcquisition.java"
    baseline=json.loads((ROOT/"design/fixed-height/ports-065.json").read_bytes())
    rows=[]
    with tempfile.TemporaryDirectory() as tmp,ExitStack() as stack:
        java=Path(tmp)/gate.name
        java.write_text(gate.read_text().replace("speed = 0.45;",f"speed = {speed:.2f};"),encoding="utf-8",newline="\n")
        if speed==.65 and hashlib.sha256(java.read_bytes()).hexdigest()!=baseline["compiledSourceSha256"]:raise ValueError("Changed gate")
        subprocess.run([str(jdk/"bin"/("javac"+suffix)),"-d",tmp,str(java),str(study.HERE/"PortConsole.java")],check=True)
        driver=stack.enter_context(subprocess.Popen([str(jdk/"bin"/("java"+suffix)),"-cp",tmp,"org.reefscape.controls.PortConsole"],
                    stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1))
        stack.enter_context(patch.object(study.route,"SHELF",.5+study.R))
        stack.enter_context(patch.object(study.route,"U_ORIGIN",.444525))
        stack.enter_context(patch.object(study.route.station,"SHELF_TOP",.5))
        try:
            for case in cases:
                hist={"travel":[6.,-6.],"peakNormal":0.,"outerStop":False,"samples":[]}
                with ExitStack() as cfg:
                    if not case.original:
                        cfg.enter_context(patch.object(axial,"BELT_BACK",-.0845 if case.cassette else BACK/1000))
                        cfg.enter_context(patch.object(axial,"BELT_FRONT",.0955 if case.cassette else FRONT/1000))
                        cfg.enter_context(patch.object(study.AxialContact,"normals",contact_law(case,hist)))
                    cmd=study.Case(name=case.name,phase_ms=case.phase,grip_mu=case.friction,depth_mm=case.depth)
                    result=study.run(driver,cmd,case.step,[])
                if case.original and json.loads(json.dumps(result))!=baseline["cases"]["0ms-0.0005"]:raise ValueError("Baseline changed")
                ready=result.get("first_in_band")
                if ready and hist["samples"]:
                    stamp=(ready["from_lip_ms"]+100+case.phase)/1000
                    i=min(range(len(hist["samples"])),key=lambda j:abs(hist["samples"][j][0]-stamp))
                    a,b=hist["samples"][max(0,i-1)],hist["samples"][i]
                    hist["firstQualifiedJaws"]={"positionMm":list(b[3:]),"speedMps":[(v-u)/max(1e-9,b[0]-a[0])/1000 for u,v in zip(a[3:],b[3:])]}
                if not keep:hist.pop("samples")
                rows.append({"counterfeed":speed,"javaSha256":hashlib.sha256(java.read_bytes()).hexdigest(),"case":vars(case),"mechanical":hist,"optical":result})
        finally:driver.stdin.close()
        if driver.wait():raise ValueError("Controller error")
    return rows

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--jdk",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps(work(a.jdk,trials()),indent=2)+"\n",encoding="utf-8",newline="\n")
