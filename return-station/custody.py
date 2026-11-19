"""Replay the measured station pipe images and electronic port state in the desktop scheduler."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def run(cache,jdk,gradle):
    receipts={}
    policy=json.loads((HERE/"qualification.json").read_bytes())
    coupled=json.loads((HERE/"coupled-clearance.json").read_bytes())["cases"]
    pads=json.loads((HERE/"pads.json").read_bytes())["results"]
    native=json.loads((HERE/"probe-native.json").read_bytes())["bins"]
    unresolved={k for b in native for k in b["outerEnvelopeOverlapMm3"]} - {r["part"] for r in coupled} - {"observer-moving-pad-"+str(i) for i in range(3)}
    if unresolved:raise ValueError("Unchecked native materials: "+str(sorted(unresolved)))
    for variant in ("issued","probe","hold"):
        inp=HERE/("images.json" if variant=="issued" else variant+"-images.json")
        feed=HERE/("feed.json" if variant=="issued" else variant+"-feed.json")
        images=json.loads(inp.read_bytes());rows=json.loads(feed.read_bytes())
        register=sha(inp)
        with tempfile.TemporaryDirectory() as folder:
            tmp=Path(folder)
            (tmp/"register.txt").write_text(register,encoding="ascii")
            source=(ROOT/"stereo-custody/CustodyEvidence.java").read_text()
            old="6389d766ad3d4d65b885bccb615a23807cadd8ef4f14d846214cea8a4dd96f73"
            if source.count(old)!=1:raise ValueError("Unexpected custody register")
            (tmp/"CustodyEvidence.java").write_text(source.replace(old,register),encoding="utf8",newline="\n")
            (tmp/"ReturnStationTest.java").write_bytes((HERE/"ReturnStationTest.java").read_bytes())
            with (tmp/"cases.csv").open("w",newline="") as out:
                w=csv.writer(out,lineterminator="\n");w.writerow(["case","port_band_ms","desktop_domain","physical_station_registration"])
                for name,d in images["cases"].items():
                    row=next(r for r in rows if r["name"]==name)
                    comparable=name.replace("hold-bias-","probe-bias-")
                    envelope=[q["at025mmIndentationMm"] for q in coupled if q["case"]==comparable]
                    pad=[q["continuousLowerMm"] for q in pads if q["case"]==comparable]
                    domain=variant in ("probe","hold") and bool(envelope and pad) and min(envelope+pad)>0 and not row["hardStop"] and not row["loss"]
                    if variant=="hold":
                        blocks=json.loads((HERE/"hold-native.json").read_bytes())["bins"]
                        domain=domain and all(not b["outerEnvelopeOverlapMm3"] and b["nativeRigidLowerMm"]>0 for b in blocks)
                    with (tmp/(name+".ports.csv")).open("w",newline="") as pf:
                        csv.writer(pf,lineterminator="\n").writerows((t[0],t[1]) for t in row["portTicks"])
                    measured=all(v=="measured-and-accepted" for v in policy["physicalInputs"].values())
                    w.writerow([name,row["firstPortBandMs"] or -1,str(domain).lower(),str(measured).lower()])
                    with (tmp/(name+".csv")).open("w",newline="") as output:
                        writer=csv.writer(output,lineterminator="\n")
                        for t,status,*bounds in d["frames"]:writer.writerow([t,status,*(bounds or [0]*6)])
            env=dict(os.environ,JAVA_HOME=str(jdk),GRADLE_USER_HOME=str(gradle),CUSTODY_SOURCE=str(tmp),RETURN_LOG=str((cache/(variant+"-gate.csv")).resolve()),
                     REEF_CONTROLS_SOURCE=str(ROOT/"competition-controls"),REEF_CONTROLS_BUILD=str(tmp/"build"))
            env["PATH"]=str(jdk/"bin")+os.pathsep+env["PATH"]
            project=ROOT/"shared/examples/java-2026-sim";wrapper=project/("gradlew.bat" if os.name=="nt" else "gradlew")
            subprocess.run([str(wrapper),"--offline","--no-daemon","--project-cache-dir",str(tmp/"gradle"),"-I",str(ROOT/"competition-controls/init.gradle"),
                           "-I",str(ROOT/"stereo-custody/init.gradle"),"test","--tests","org.reefscape.controls.ReturnStationTest"],cwd=project,env=env,check=True)
            with (cache/(variant+"-gate.csv")).open(newline="") as f:results=list(csv.DictReader(f))
            receipts[variant]={"register":register,"qualification":sha(HERE/"qualification.json"),"feed":sha(feed),"source":sha(tmp/"CustodyEvidence.java"),"run":results}
    (HERE/"custody.json").write_text(json.dumps(receipts,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for n in ("cache","jdk","gradle"):p.add_argument(n,type=Path)
    a=p.parse_args();run(a.cache,a.jdk,a.gradle)
