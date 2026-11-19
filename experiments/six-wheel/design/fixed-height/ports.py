"""Drive the original two-port Java state machine with modeled returns at the lower locked pose."""
import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/"observable-acquisition"))
from study import Case,run,route
from hashlib import sha256


def main(jdk,speed):
    rows={}
    inputs=ROOT/"observable-acquisition"
    suffix=".exe" if os.name=="nt" else ""
    with tempfile.TemporaryDirectory() as tmp,ExitStack() as stack:
        text=(inputs/"PortAcquisition.java").read_text(encoding="utf-8")
        if text.count("speed = 0.45;") != 1:
            raise ValueError("Counterfeed implementation differs from registered version")
        variant=Path(tmp)/"PortAcquisition.java"
        variant.write_text(text.replace("speed = 0.45;",f"speed = {speed:.2f};"),encoding="utf-8",newline="\n")
        source_hash=sha256(variant.read_bytes()).hexdigest()
        subprocess.run([str(jdk/"bin"/("javac"+suffix)),"-d",tmp,str(variant),str(inputs/"PortConsole.java")],check=True)
        proc=stack.enter_context(subprocess.Popen([str(jdk/"bin"/("java"+suffix)),"-cp",tmp,"org.reefscape.controls.PortConsole"],
                                 stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1))
        stack.enter_context(patch.object(route,"SHELF",.5+route.R))
        stack.enter_context(patch.object(route,"U_ORIGIN",.444525))
        stack.enter_context(patch.object(route.station,"SHELF_TOP",.5))
        try:
            for phase in range(20):
                for dt in (.0005,.00025):
                    events=[]
                    case=Case(name="low",phase_ms=phase)
                    r=run(proc,case,dt,events)
                    rows[f"{phase}ms-{dt:g}"]=r
        finally:
            proc.stdin.close()
        if proc.wait()!=0:
            raise ValueError("Java controller exited unsuccessfully")
    return {"sourceSha256":{n:sha256((inputs/n).read_bytes()).hexdigest() for n in ("PortAcquisition.java","PortConsole.java","study.py")},
            "pose":{"heightMm":560,"stationStrokeMm":0,"shelfTopMm":500,"dockMm":10},
            "sensorModel":"Centered transverse pipe, successor geometry still requires physical range and dock calibration.",
            "counterfeedMps":speed,"compiledSourceSha256":source_hash,"cases":rows,"carryAuthorized":False}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--java-home",type=Path,required=True)
    p.add_argument("--counterfeed",type=float,choices=(.45,.65,.8),default=.45)
    p.add_argument("--output",type=Path,default=HERE/"ports.json")
    a=p.parse_args()
    a.output.write_text(json.dumps(main(a.java_home,a.counterfeed),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
