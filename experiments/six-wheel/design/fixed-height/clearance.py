"""Check the finite transverse pipe in the low built robot for both actual Java counterfeed settings."""
import argparse
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import sys
import subprocess
import tempfile
from unittest.mock import patch
from native import ROOT,HERE,hardware
sys.path.insert(0,str(ROOT/"observable-acquisition"))
import study
sys.path.insert(0,str(ROOT/"station-supply"))
from transfer import corridor,hits
sys.path.insert(0,str(ROOT/"design/competition-package/shop-duplex"))
from assembly import pose_shapes,cad


def audit(jdk):
    model=pose_shapes(560,0,"red")
    modified,more=hardware(model,height=560)
    shapes={k:v for k,v in {**model,**modified,**more}.items() if k not in
            ("grip-L-420-3M-15","grip-R-420-3M-15","receiver-R6-ported")}
    bounds={k:cad.bounds(v) for k,v in shapes.items()}
    rows={}
    original=study.AxialContact.normals
    exe=".exe" if os.name=="nt" else ""
    for file in ("ports.json","ports-065.json"):
        ref=json.loads((HERE/file).read_bytes())
        speed=ref["counterfeedMps"]
        all_flight=[]
        low,high=1e6,-1e6
        ztouch=500+study.R*1000
        with tempfile.TemporaryDirectory() as tmp,ExitStack() as stack:
            java=Path(tmp)/"PortAcquisition.java"
            text=(study.HERE/"PortAcquisition.java").read_text(encoding="utf-8")
            java.write_text(text.replace("speed = 0.45;",f"speed = {speed:.2f};"),encoding="utf-8",newline="\n")
            if study.hashlib.sha256(java.read_bytes()).hexdigest()!=ref["compiledSourceSha256"]:
                raise ValueError("Compiled source changed")
            subprocess.run([str(jdk/"bin"/("javac"+exe)),"-d",tmp,str(java),str(study.HERE/"PortConsole.java")],check=True)
            proc=stack.enter_context(subprocess.Popen([str(jdk/"bin"/("java"+exe)),"-cp",tmp,"org.reefscape.controls.PortConsole"],
                    stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1))
            stack.enter_context(patch.object(study.route,"SHELF",.5+study.R))
            stack.enter_context(patch.object(study.route,"U_ORIGIN",.444525))
            stack.enter_context(patch.object(study.route.station,"SHELF_TOP",.5))
            try:
                for key,expected in ref["cases"].items():
                    phase,delta=key.split("ms-")
                    step=float(delta)
                    def observe(contact,u,v,yaw,stage,t,zrel=0):
                        nonlocal low,high
                        if zrel>1e-8:
                            all_flight.append((1000*u,ztouch+1000*zrel))
                        else:
                            low=min(low,1000*u)
                            high=max(high,1000*u)
                        return original(contact,u,v,yaw,stage,t,zrel)
                    with patch.object(study.AxialContact,"normals",observe):
                        actual=study.run(proc,study.Case(name="low",phase_ms=int(phase)),step,[])
                    if json.loads(json.dumps(actual))!=expected:
                        raise ValueError("Native sweep differs from optical replay "+key+" "+str([(k,actual[k],expected[k]) for k in actual if actual[k]!=expected[k]][:1]))
                    all_flight.append((actual["first_contact_ms_mm"][1],ztouch))
            finally:
                proc.stdin.close()
            if proc.wait(): raise ValueError("Java failure")
        supported=corridor(shapes,bounds,(math.floor(low),ztouch),(math.ceil(high),ztouch))
        probes=hits(shapes,bounds,low,ztouch)
        # Convex filled tube between end points plus distance of every integration sample
        # bounds its chord. Overlapping bins share the samples on either side of the boundary.
        ordered=sorted(all_flight,key=lambda p:p[1])
        upper=ordered[-1][1]
        cuts=[min(range(len(ordered)),key=lambda i:abs(ordered[i][1]-(ztouch+(upper-ztouch)*j/12))) for j in range(13)]
        cuts[0]=0;cuts[-1]=len(ordered)-1
        inbound={}
        for a,b in zip(cuts,cuts[1:]):
            points=[pt for pt in ordered if ordered[a][1]-2.5<=pt[1]<=ordered[b][1]+2.5]
            x0,z0=points[0];x1,z1=points[-1]
            dx,dz=x1-x0,z1-z0;ds=dx*dx+dz*dz
            def distance(p):
                x,z=p
                alpha=max(0,min(1,((x-x0)*dx+(z-z0)*dz)/ds))
                return math.hypot(x-x0-alpha*dx,z-z0-alpha*dz)
            padding=max(map(distance,points))+.002
            for k,v in corridor(shapes,bounds,(x0,z0),(x1,z1),padding).items():
                inbound[k]=max(v,inbound.get(k,0))
        rows[str(speed)]={"supportRangeMm":[low,high],"continuousSupportedBoundingTubeMm3":supported,
                          "annularMaterialAtMinimumMm3":probes,"inboundChordOverlapsMm3":inbound,
                          "posesReplayed":len(ref["cases"]),"nativeBodiesTested":len(shapes)}
    return rows


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--java-home",type=Path,required=True)
    a=p.parse_args()
    (HERE/"clearance.json").write_text(json.dumps(audit(a.java_home),indent=2)+"\n",encoding="utf-8",newline="\n")
