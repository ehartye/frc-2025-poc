"""Bind reconstructed fixed-head measurements, source stock events and the selected native motion to one HAL trial."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
def read(p): return json.loads(p.read_bytes())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def run(scratch,native,jdk,cache):
    registration=read(HERE/"optics.json")
    if not registration["movingRayPrism"]["allChangedBodiesOutside"]:raise ValueError("Moving view unqualified")
    source=read(scratch/"source.json")["pieces"]
    plans=read(ROOT/"fixed-height-match/placements.json")["placements"]
    with tempfile.TemporaryDirectory() as temp:
        tmp=Path(temp)
        for k in ("MovingWitnessTest","PacketClockTest","SlideFeedbackTest","OpticalWindow","PortAcquisition","CustodyEvidence","Retreat"):
            sub={"MovingWitnessTest":"moving-observer","PacketClockTest":"moving-frame-timing","SlideFeedbackTest":"slide-feedback",
                 "OpticalWindow":"moving-frame-timing","PortAcquisition":"observable-acquisition","CustodyEvidence":"stereo-custody","Retreat":"slide-feedback"}[k]
            s=(ROOT/sub/(k+".java")).read_text(encoding="utf-8")
            if k=="PortAcquisition":s=s.replace("speed = 0.45;","speed = 0.65;")
            if k=="CustodyEvidence":s=s.replace("6389d766ad3d4d65b885bccb615a23807cadd8ef4f14d846214cea8a4dd96f73",sha(native/"fixed-0.json"))
            if k=="Retreat":s=s.replace("c1242109e239d880d1562c3e7b4bbcf857d154d9a31e3403e35e182864d00ebe",sha(HERE/"optics.json"))
            (tmp/(k+".java")).write_text(s,encoding="utf-8",newline="\n")
        if sha(tmp/"PortAcquisition.java")!=read(ROOT/"design/fixed-height/ports-065.json")["compiledSourceSha256"]:raise ValueError("Low port control mismatch")
        shutil.copy(HERE/"IntegratedTest.java",tmp)
        for stroke,pose in ((0,"receive"),(370,"place")):
            if sha(native/f"fixed-{stroke}.json")!=registration["poses"][pose]["frameFileSha256"]:raise ValueError("Unregistered native scene")
        old=list(csv.DictReader((ROOT/"moving-frame-timing/exposures.csv").open()))
        rebuilt={k:dict(frames) for k,frames in read(native/"fixed-370.json").items()}
        fixed_parity=0
        for r in old:
            if r["head"]=="fixed":
                obs=rebuilt[r["piece"]][int(r["capture_ms"])]
                raw=[float(r[f]) for f in ("x_lo","x_hi","y_lo","y_hi","bottom_lo","bottom_hi") if r[f]]
                actual=[q for n in ("u_interval_mm","y_interval_mm","bottom_relative_to_shelf_mm") for q in obs.get(n,[])]
                if raw!=actual or obs["status"]!=r["status"]:raise ValueError("Requalify changed native release frame")
                fixed_parity+=1
        shutil.copy(ROOT/"moving-frame-timing/exposures.csv",tmp)
        shutil.copy(ROOT/"moving-observer/frames.csv",tmp)
        with (tmp/"inputs.csv").open("w",newline="") as f:
            w=csv.writer(f,lineterminator="\n");w.writerow(["kind","piece","time","values"])
            for s in plans:
                key=s["id"]; rec=source[key]
                # Lip is the physical project datum: do not separately align every downstream event.
                epoch=(18+s["lip"])*1e6-rec["lip_ms"]*1000
                w.writerow(["clock",key,round(epoch),rec["truth"]["place"],math.floor((18+s["drive"])*1000),0 if key=="coral-A" else 35,
                            rec["first_in_band_ms"],round((18+s["release"])*1000)])
                for tick in rec["acquisition_ticks"]:w.writerow(["port",key,*tick[2:11]])
                for t,obs in read(native/"fixed-0.json")[key]:
                    b=[i for n in ("u_interval_mm","y_interval_mm","bottom_relative_to_shelf_mm") for i in obs.get(n,[])]
                    w.writerow(["seat",key,t,obs["status"],*(b if b else [0]*6)])
        with (tmp/"registers.txt").open("w") as f:
            f.write("\n".join([sha(native/"fixed-0.json"),sha(native/"fixed-370.json"),sha(ROOT/"moving-observer/frames.csv"),sha(HERE/"optics.json")]))
        env=dict(os.environ,JAVA_HOME=str(jdk),GRADLE_USER_HOME=str(cache),CUSTODY_SOURCE=str(tmp),INTEGRATED_OUTPUT=str(native.resolve()),
                 REEF_CONTROLS_SOURCE=str(ROOT/"competition-controls"),REEF_CONTROLS_BUILD=str(tmp/"build"))
        env["PATH"]=str(jdk/"bin")+os.pathsep+env["PATH"]
        project=ROOT/"shared/examples/java-2026-sim"
        wrapper=project/("gradlew.bat" if os.name=="nt" else "gradlew")
        subprocess.run([str(wrapper),"--offline","--no-daemon","--rerun-tasks","--project-cache-dir",str(tmp/"gradle"),
                        "-I",str(ROOT/"competition-controls/init.gradle"),"-I",str(ROOT/"stereo-custody/init.gradle"),
                        "test","--tests","org.reefscape.controls.IntegratedTest"],cwd=project,env=env,check=True)
        (HERE/"events.csv").write_text((native/"events.csv").read_text(encoding="utf-8"),encoding="utf-8",newline="\n")
        pins={p.name:sha(p) for p in sorted(tmp.glob("*.java"))}
        out={"sceneRegister":sha(HERE/"optics.json"),"fixedReleaseFramesUnchanged":fixed_parity,"emittedEventsSha256":sha(HERE/"events.csv"),
             "sourceSha256":pins,"inputsSha256":{f:sha(tmp/f) for f in ("inputs.csv","frames.csv","exposures.csv","registers.txt")},
             "motionSha256":{p.name:sha(p) for p in sorted(native.glob("path-*.csv"))}}
        (HERE/"hal.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ("scratch","native","java_home","gradle_home"):p.add_argument(arg,type=Path)
    a=p.parse_args();run(a.scratch.resolve(),a.native.resolve(),a.java_home.resolve(),a.gradle_home.resolve())
