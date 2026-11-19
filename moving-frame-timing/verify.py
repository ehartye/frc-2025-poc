"""Run the two real scheduler cadences, replay delivery faults, and summarize per-camera phase outcomes."""
import argparse
import csv
from collections import Counter,defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from xml.etree import ElementTree
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
PROJECT=ROOT/"shared/examples/java-2026-sim"


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def contexts():
    base=json.loads((ROOT/"low-observation/results.json").read_bytes())["pieces"]
    overhead=json.loads((ROOT/"moving-observer/replay.json").read_bytes())["registration_sha256"]
    # This fixture register pins both the full model introduced by the bridge and the fixed-head observation procedure.
    fixed=hashlib.sha256("".join(digest(ROOT/n) for n in ("moving-observer/moving-bridge.step","moving-observer/handoff.py",
        "low-observation/results.json","stereo-custody/observe.py","stereo-custody/imaging.py")).encode()).hexdigest()
    with (HERE/"contexts.csv").open("w",newline="",encoding="ascii") as file:
        writer=csv.writer(file,lineterminator="\n")
        writer.writerow(["piece","moving_register","fixed_register","release_ms","prior_lower_ms"])
        for k,v in base.items():writer.writerow([k,overhead,fixed,math.ceil(v["truth_ms"]["place"]),math.floor(v["ideal_prior_pad_ms"]-3)])


def run(jdk,cache,output):
    contexts()
    output.parent.mkdir(exist_ok=True,parents=True)
    with tempfile.TemporaryDirectory() as folder:
        tmp=Path(folder)
        env=dict(os.environ,JAVA_HOME=str(jdk),GRADLE_USER_HOME=str(cache),CUSTODY_SOURCE=str(HERE),
                 TIMING_RECEIPT=str(output),REEF_CONTROLS_SOURCE=str(ROOT/"competition-controls"),REEF_CONTROLS_BUILD=str(tmp/"build"))
        env["PATH"]=str(jdk/"bin")+os.pathsep+env["PATH"]
        subprocess.run([str(PROJECT/("gradlew.bat" if os.name=="nt" else "gradlew")),"--offline","--no-daemon","--rerun-tasks",
            "--project-cache-dir",str(tmp/"cache"),"-I",str(ROOT/"competition-controls/init.gradle"),"-I",
            str(ROOT/"stereo-custody/init.gradle"),"test","--tests","org.reefscape.controls.PacketClockTest"],
            cwd=PROJECT,env=env,check=True)
        root=ElementTree.parse(tmp/"build/test-results/test/TEST-org.reefscape.controls.PacketClockTest.xml").getroot()
        if [root.get(k) for k in ("tests","errors","skipped","failures")]!=["1","0","0","0"]:raise ValueError("incomplete HAL replay")
    tally=Counter()
    profiles=defaultdict(lambda:defaultdict(lambda:[0,0]))
    hal=defaultdict(lambda:defaultdict(dict))
    with output.open(encoding="ascii",newline="") as file:
        for r in csv.DictReader(file):
            p=int(r["period_ms"]); mode=r["fault"]; d=int(r["delay_ms"])
            eligible=r["result"]=="window"
            tally[r["process"]]+=1
            if mode in ("constant","fixed-delay"):
                v=profiles[f"{p}ms/{mode}/{r['piece']}"][d]
                v[0]+=int(eligible);v[1]+=1
            if r["process"]=="HAL":
                hal[f"{p}ms/{mode}"][r["piece"]][int(r["phase"])]={
                    "status":r["result"],"forward_ms":int(r["capture_ms"]),"application_ms":int(r["tick_ms"])}
    prior=json.loads((ROOT/"moving-observer/handoff.json").read_bytes())
    summary={"counts":dict(tally),"profiles":profiles,"source_sha256":{n:digest(HERE/n) for n in
        ("OpticalWindow.java","PacketClockTest.java","exposures.csv","contexts.csv","exposures.py","verify.py")},
        "primary_sha256":digest(ROOT/"moving-observer/handoff.json"),"complete_sweep_sha256":digest(output),"hal":{}}
    for label,by_piece in hal.items():
        summary["hal"][label]={}
        for piece,phases in by_piece.items():
            statuses=Counter(v["status"] for v in phases.values())
            yes={str(k):v["application_ms"] for k,v in phases.items() if v["status"]=="window"}
            summary["hal"][label][piece]={"status_counts":dict(statuses),"accepted_phase_ticks":yes,
                "accepted_capture_ms":{str(k):v["forward_ms"] for k,v in phases.items() if v["status"]=="window"}}
            if label=="20ms/constant":
                summary["hal"][label][piece]["historical_accepted_phase_ticks"]={str(r["phase_ms"]):r["first_tick_ms"] for r in prior[piece] if r["timely"]}
    (HERE/"results.json").write_text(json.dumps(summary,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--java-home",type=Path,required=True);p.add_argument("--gradle-home",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True);args=p.parse_args()
    run(args.java_home.resolve(),args.gradle_home.resolve(),args.output.resolve())
