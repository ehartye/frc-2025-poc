"""Check the independent camera and the issued port controller together on the project's desktop HAL."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
from xml.etree import ElementTree

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
PROJECT=ROOT/"shared/examples/java-2026-sim"


def run(jdk,cache):
    with tempfile.TemporaryDirectory() as folder:
        tmp=Path(folder)
        env=dict(os.environ,JAVA_HOME=str(jdk),GRADLE_USER_HOME=str(cache),CUSTODY_SOURCE=str(HERE),
                 REEF_CONTROLS_SOURCE=str(ROOT/"competition-controls"),REEF_CONTROLS_BUILD=str(tmp/"build"),
                 BELT_CLOCK_SOURCE=str(ROOT/"observable-acquisition"))
        env["PATH"]=str(jdk/"bin")+os.pathsep+env["PATH"]
        wrapper=PROJECT/("gradlew.bat" if os.name=="nt" else "gradlew")
        subprocess.run([str(wrapper),"--offline","--no-daemon","--rerun-tasks","--project-cache-dir",str(tmp/"cache"),
                        "-I",str(ROOT/"competition-controls/init.gradle"),"-I",str(ROOT/"coupled-belt-clock/init.gradle"),
                        "-I",str(HERE/"init.gradle"),"test","--tests","org.reefscape.controls.CustodyClockTest"],
                       cwd=PROJECT,env=env,check=True)
        result=ElementTree.parse(tmp/"build/test-results/test/TEST-org.reefscape.controls.CustodyClockTest.xml").getroot()
        if any(int(result.get(k))!=v for k,v in (("tests",1),("skipped",0),("failures",0),("errors",0))):
            raise ValueError("Incomplete HAL result")
        with (HERE/"application.csv").open() as f: apps=list(csv.DictReader(f))
        with (HERE/"camera.csv").open() as f: cameras=sum(1 for _ in csv.DictReader(f))
        positive=sum(int(r["corroborated"]) for r in apps)
        if f"CUSTODY_TICKS={len(apps)} FRAMES={cameras} POSITIVE={positive}" not in result.findtext("system-out","").splitlines():
            raise ValueError("Trace inventory mismatch")
        sources=[HERE/n for n in ("CustodyEvidence.java","CustodyClockTest.java","verify_hal.py","init.gradle",
                                   "application.csv","camera.csv")]
        sources.extend([ROOT/"observable-acquisition/PortAcquisition.java",PROJECT/"gradle.lockfile"])
        output={"scheduler_ticks":len(apps),"camera_frames":cameras,"corroborated_ticks":positive,"period_us":20000,
                "source_sha256":{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
        (HERE/"hal.json").write_text(json.dumps(output,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--java-home",type=Path,required=True)
    p.add_argument("--gradle-home",type=Path,required=True)
    args=p.parse_args()
    run(args.java_home.resolve(),args.gradle_home.resolve())
