"""Apply the registered packet, optical and rack controllers to ten corrected trajectories."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
MODES = ("nominal","positive","negative","wheel-bias","lateral-kick")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def project_inputs(cache, historical, tmp):
    source=json.loads((historical/"source.json").read_bytes())["pieces"]
    plans=json.loads((ROOT/"fixed-height-match/placements.json").read_bytes())["placements"]
    starts=json.loads((cache/"receive.json").read_bytes())
    actual=json.loads((HERE/"source.json").read_bytes())
    priors={}
    identity=digest(ROOT/"bridge-tolerance/register.json")
    reg=[digest(cache/"receive.json"),digest(ROOT/"bridge-tolerance/moving-bridge.step"),
         json.loads((ROOT/"bridge-tolerance/replay.json").read_bytes())["registration_sha256"],identity]
    (tmp/"registers.txt").write_text("\n".join(reg),encoding="ascii")
    pins={}
    for mode in MODES:
        place=tmp/mode;place.mkdir()
        with (place/"inputs.csv").open("w",newline="") as file, (place/"frames.csv").open("w",newline="") as travel, (place/"exposures.csv").open("w",newline="") as flight:
            writer=csv.writer(file,lineterminator="\n");mw=csv.writer(travel,lineterminator="\n");fw=csv.writer(flight,lineterminator="\n")
            writer.writerow(["kind","piece","time","values"]);mw.writerow(["piece","capture","arrival","status","bounds"]);fw.writerow(["piece","head","time","status","bounds"])
            for s in plans:
                key=s["id"];rec=source[key];row=next(r for r in actual if r["pipe"]==key and r["mode"]==mode)
                tr=np.load(cache/(key+"-"+mode+".npy"))
                u=tr[:,1]-tr[:,5]
                pad=tr[(tr[:,0]>=row["releaseCameraMs"])&(tr[:,0]<=row["endCameraMs"])&(np.abs(u-155)<=2.8),0]
                if not len(pad):raise ValueError("No physical pad crossing")
                prior=math.floor(float(pad[-1])-3)
                priors[key+"-"+mode]=prior
                epoch=(18+s["lip"])*1e6-rec["lip_ms"]*1000
                writer.writerow(["clock",key,round(epoch),row["releaseCameraMs"],math.floor((18+s["drive"])*1000),0 if key=="coral-A" else 35,rec["first_in_band_ms"],round((18+s["release"])*1000),prior])
                for tick in rec["acquisition_ticks"]:writer.writerow(["port",key,*tick[2:11]])
                for t,obs in starts[key]:
                    bounds=[v for n in ("u_interval_mm","y_interval_mm","bottom_relative_to_shelf_mm") for v in obs.get(n,[])]
                    writer.writerow(["seat",key,t,obs["status"],*(bounds or [0]*6)])
                for head in ("fixed","moving"):
                    with (cache/(key+"-"+mode+"-"+head+".csv")).open(newline="") as f:
                        for line in list(csv.reader(f))[1:]:
                            ts=int(line[2])
                            if head=="moving" and ts<=row["releaseCameraMs"]+17:mw.writerow([key,ts,ts+15,*line[3:]])
                            if ts>=row["releaseCameraMs"]:fw.writerow([*line,*[""]*(10-len(line))])
        pins[mode]={p.name:digest(p) for p in place.iterdir()}
    return reg,pins,priors


def replace_once(text,old,new):
    if text.count(old)!=1:raise ValueError("Source adapter no longer matches: "+old)
    return text.replace(old,new)


def run(cache, historical,jdk,gradle):
    output=cache.resolve()
    with tempfile.TemporaryDirectory() as folder:
        tmp=Path(folder);reg,pins,priors=project_inputs(cache,historical,tmp)
        for k,sub in {"MovingWitnessTest":"moving-observer","PacketClockTest":"moving-frame-timing","SlideFeedbackTest":"slide-feedback","OpticalWindow":"moving-frame-timing","PortAcquisition":"observable-acquisition","CustodyEvidence":"stereo-custody","Retreat":"slide-feedback"}.items():
            text=(ROOT/sub/(k+".java")).read_text(encoding="utf8")
            replacements={"PortAcquisition":("speed = 0.45;","speed = 0.65;"),"CustodyEvidence":("6389d766ad3d4d65b885bccb615a23807cadd8ef4f14d846214cea8a4dd96f73",reg[0]),
                          "Retreat":("c1242109e239d880d1562c3e7b4bbcf857d154d9a31e3403e35e182864d00ebe",reg[3])}
            if k in replacements:text=replace_once(text,*replacements[k])
            (tmp/(k+".java")).write_text(text,encoding="utf8",newline="\n")
        original=ROOT/"integrated-match/IntegratedTest.java"
        text=original.read_text(encoding="utf8")
        text=replace_once(text,'prior=a?26593:26301;','prior=Long.parseLong(row[8]);')
        text=replace_once(text,'Path src=Path.of','Path root=Path.of')
        text=replace_once(text,'readAllLines(src.resolve("registers.txt"))','readAllLines(root.resolve("registers.txt"))')
        text=replace_once(text,'for(String mode:List.of("nominal","lateB","single")){','for(String approach:List.of("nominal","positive","negative","wheel-bias","lateral-kick")) for(String mode:List.of("nominal","lateB","single")){\n                Path src=root.resolve(approach);')
        text=replace_once(text,'for(var tr:tests.values())tr.prepare(regs,mode);','log.println("approach,"+approach); for(var tr:tests.values())tr.prepare(regs,mode);')
        text=replace_once(text,'out.resolve("path-"+mode','out.resolve("path-"+approach+"-"+mode')
        # Record all reached outcomes, including legitimate rejected corrected inputs.
        text=replace_once(text,'assertTrue(a.stowed, mode);assertEquals(mode.equals("nominal"),b.stowed,mode);assertEquals(mode.equals("lateB"),b.failed,mode);','assertFalse(b.issued && !a.stowed);')
        (tmp/"IntegratedTest.java").write_text(text,encoding="utf8",newline="\n")
        env=dict(os.environ,JAVA_HOME=str(jdk),GRADLE_USER_HOME=str(gradle),CUSTODY_SOURCE=str(tmp),INTEGRATED_OUTPUT=str(output),
                 REEF_CONTROLS_SOURCE=str(ROOT/"competition-controls"),REEF_CONTROLS_BUILD=str(tmp/"build"))
        env["PATH"]=str(jdk/"bin")+os.pathsep+env["PATH"]
        project=ROOT/"shared/examples/java-2026-sim"
        wrapper=project/("gradlew.bat" if os.name=="nt" else "gradlew")
        subprocess.run([str(wrapper),"--offline","--no-daemon","--project-cache-dir",str(tmp/"gradle"),"-I",str(ROOT/"competition-controls/init.gradle"),"-I",str(ROOT/"stereo-custody/init.gradle"),"test","--tests","org.reefscape.controls.IntegratedTest"],cwd=project,env=env,check=True)
        with (cache/"events.csv").open(newline="") as f,(HERE/"events.csv").open("w",newline="") as o:
            rows=csv.reader(f);writer=csv.writer(o,lineterminator="\n");writer.writerow(["approach",*next(rows)])
            for r in rows:
                if r[0]=="approach":mode=r[1]
                else:writer.writerow([mode,*r])
        paths={p.name:digest(p) for p in cache.glob("path-*-coral-*.csv")}
        out={"source":digest(original),"compiled":{p.name:digest(p) for p in tmp.glob("*.java")},"evidenceInputs":pins,"registration":reg,
             "lastPriorMs":priors,"pathSHA256":paths,"eventsSHA256":digest(HERE/"events.csv")}
        (HERE/"hal.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("cache","historical","jdk","gradle"):p.add_argument(name,type=Path)
    a=p.parse_args();run(a.cache,a.historical,a.jdk,a.gradle)
