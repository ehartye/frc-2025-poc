"""Run a conditional moving-frame supervisor on the real WPILib desktop clock."""
import argparse
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
    with tempfile.TemporaryDirectory() as temp:
        tmp=Path(temp)
        env=dict(os.environ,JAVA_HOME=str(jdk),GRADLE_USER_HOME=str(cache),CUSTODY_SOURCE=str(HERE),
                 REEF_CONTROLS_SOURCE=str(ROOT/"competition-controls"),REEF_CONTROLS_BUILD=str(tmp/"build"))
        env["PATH"]=str(jdk/"bin")+os.pathsep+env["PATH"]
        subprocess.run([str(PROJECT/("gradlew.bat" if os.name=="nt" else "gradlew")),"--offline","--no-daemon","--rerun-tasks",
            "--project-cache-dir",str(tmp/"cache"),"-I",str(ROOT/"competition-controls/init.gradle"),"-I",
            str(ROOT/"stereo-custody/init.gradle"),"test","--tests","org.reefscape.controls.MovingWitnessTest"],
            cwd=PROJECT,env=env,check=True)
        xml=ElementTree.parse(tmp/"build/test-results/test/TEST-org.reefscape.controls.MovingWitnessTest.xml").getroot()
        events=[line.split() for line in xml.findtext("system-out","").splitlines() if line.startswith("MOVING ")]
        if len(events)!=10 or int(xml.get("tests"))!=1 or int(xml.get("failures"))!=0:raise ValueError("missing HAL outcomes")
        report={"period_us":20000,"cases":{k+"/"+m:{"eligible_drive_to_release_ticks":int(ok),"hold_ticks":int(hold)}
                                              for _,k,m,ok,hold in events},"inputs":{n:hashlib.sha256((HERE/n).read_bytes()).hexdigest()
                                              for n in ("MovingWitnessTest.java","frames.csv","replay.json")}}
        (HERE/"hal.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8",newline="\n")
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--java-home",type=Path,required=True)
    p.add_argument("--gradle-home",type=Path,required=True)
    args=p.parse_args()
    run(args.java_home.resolve(),args.gradle_home.resolve())
