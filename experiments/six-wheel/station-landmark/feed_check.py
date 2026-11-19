"""Replay the actual attained station pose through the finite pipe and the issued Java port controller."""
import argparse
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"return-station"))
from feed import port_program,trial


def run(cache,jdk):
    routes=json.loads((HERE/"routes.json").read_bytes())
    original=json.loads((ROOT/"relieved-match/first-feed.json").read_bytes())
    epoch=original["epochWallUs"]
    duration=int((original["scheduledDriveWallS"]-epoch/1e6)*1000+.5)
    answers={}
    cache.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        tmp=Path(tmp)
        file=tmp/"PortAcquisition.java"
        file.write_text(port_program(True),encoding="utf8",newline="\n")
        digest=hashlib.sha256(file.read_bytes()).hexdigest()
        if digest!=original["javaSourceSha256"]:raise ValueError("Different port controller")
        subprocess.run([str(jdk/"bin/javac"),"-d",str(tmp),str(file),str(ROOT/"observable-acquisition/PortConsole.java")],check=True)
        with subprocess.Popen([str(jdk/"bin/java"),"-cp",str(tmp),"org.reefscape.controls.PortConsole"],
                              stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) as proc:
            for name,r in routes["cases"].items():
                if not r["eligibleInRehearsal"]:continue
                n,y,h=r["observer"]["finalStationMmDeg"]
                # The earlier feed solver takes robot-forward and robot-left residuals.
                delta=[-n,-y,h]
                for label,human_shift in ([(name,0),("shiftedHumanSource",3)] if name=="nominal" else [(name,0)]):
                    v=trial(proc,delta,label,.00025,0,cache,feed_side=human_shift,relief=True,until_ms=duration)
                    answers[label]={k:v[k] for k in ("lastPortState","firstPortBandMs","hardStop","loss",
                        "peakAxisTiltDeg","sourceBeltPeakTravelMm","finalPositionMm","trajectorySHA256")}
                    answers[label]["actualRobotMmDeg"]=delta
                    answers[label]["humanFeedDisplacementMm"]=human_shift
                    print(label,v["lastPortState"],v["hardStop"],flush=True)
            proc.stdin.close();proc.wait()
    return {"routesSha256":hashlib.sha256((HERE/"routes.json").read_bytes()).hexdigest(),
            "programSha256":digest,"humanFeedWallS":original["humanReleaseWallS"],
            "lipWallS":original["lipWallS"],"epochWallUs":epoch,"throughScheduledDriveWallS":original["scheduledDriveWallS"],
            "outcomes":answers}


if __name__=="__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("output",type=Path);cli.add_argument("jdk",type=Path);a=cli.parse_args()
    (HERE/"feed.json").write_text(json.dumps(run(a.output,a.jdk),indent=2)+"\n",encoding="utf8",newline="\n")
