"""Materialize the issued low two-piece acquisition and both independent settlement branches."""
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
LOW=ROOT/"design/fixed-height"
sys.path.insert(0,str(LOW))
import handoff
import ports
sys.path.insert(0,str(ROOT/"observable-acquisition"))
import study as acquisition

REF=json.loads((LOW/"ports-065.json").read_bytes())
KEYS=["0ms-0.0005","19ms-0.00025"]


def inputs(jdk):
    result={}
    suffix=".exe" if os.name=="nt" else ""
    with tempfile.TemporaryDirectory() as tmp:
        control=ROOT/"observable-acquisition"
        file=Path(tmp)/"PortAcquisition.java"
        java=(control/"PortAcquisition.java").read_text(encoding="utf-8").replace("speed = 0.45;","speed = 0.65;")
        file.write_text(java,encoding="utf-8",newline="\n")
        if hashlib.sha256(file.read_bytes()).hexdigest()!=REF["compiledSourceSha256"]: raise ValueError("low control changed")
        subprocess.run([str(jdk/"bin"/("javac"+suffix)),"-d",tmp,str(file),str(control/"PortConsole.java")],check=True)
        with subprocess.Popen([str(jdk/"bin"/("java"+suffix)),"-cp",tmp,"org.reefscape.controls.PortConsole"],
                              stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) as pipe:
            try:
                for k,piece in zip(KEYS,handoff.spin.DATA["identified_transfers"]):
                    phase,step=k.split("ms-")
                    phase=int(phase)
                    events=[]
                    path=[]
                    original=acquisition.AxialContact.normals
                    def observe(contact,u,v,yaw,mode,time,zrel=0):
                        path.append((time*1000,u*1000,v*1000,557.15+zrel*1000,math.degrees(yaw),0))
                        return original(contact,u,v,yaw,mode,time,zrel)
                    with patch.object(ports.route,"SHELF",.5+ports.route.R),patch.object(ports.route,"U_ORIGIN",.444525), \
                         patch.object(ports.route.station,"SHELF_TOP",.5),patch.object(acquisition.AxialContact,"normals",observe):
                        port=acquisition.run(pipe,acquisition.Case(name="low",phase_ms=phase),float(step),events)
                    if json.loads(json.dumps(port))!=REF["cases"][k]:raise ValueError("low acquisition changed")
                    original_settle=handoff.spin.settle.continue_contact
                    trials={}
                    def record(**kw):
                        response=original_settle(**kw)
                        trials[kw["withdrawal"]]=(kw,response)
                        return response
                    with patch.object(handoff.spin.settle,"continue_contact",record):
                        report=handoff.one(piece,"RAMP_090",port)
                    kw,post=trials["after-separation"]
                    run=kw["upstream"]
                    origin=100+phase
                    resumed=[]
                    for r in run["trace"]:
                        x,y,z=r["xyz_m"]
                        cx,cy,a=r["chassis_xy_heading"]
                        dx,dy=x-cx,y-cy
                        u=1000*(dx*math.cos(a)+dy*math.sin(a))
                        v=1000*(-dx*math.sin(a)+dy*math.cos(a))
                        resumed.append((origin+1000*r["time_s"],u,v,1000*z,-math.degrees(r.get("robot_relative_yaw_rad",0)),1000*r["slide_m"]))
                    first=port["first_in_band"]["from_lip_ms"]+origin
                    both=[r for r in path if r[0]<first]+resumed
                    result[piece["id"]] = dict(source=piece["id"],phase=phase,origin=origin,first=first,
                        acquisition_ticks=events,trajectory=np.array(both),report=report,source_run=run,settle_kwargs=kw,
                        truth={e["event"]:origin+1000*e["time_s"] for e in run["events"]},
                        post=post,post_fixed=trials["fixed"][1],admitted_port=port)
            finally:
                pipe.stdin.close()
            if pipe.wait()!=0:raise ValueError("Java input did not close")
    return result
