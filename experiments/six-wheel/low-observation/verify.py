"""Build and run the low station port profile and co-registered cameras through the desktop scheduler."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from xml.etree import ElementTree

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
STEREO=ROOT/"stereo-custody"
PORTS=ROOT/"observable-acquisition"
PROJECT=ROOT/"shared/examples/java-2026-sim"


def digest(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def run(jdk,cache):
    result=json.loads((HERE/"results.json").read_bytes())
    with tempfile.TemporaryDirectory() as folder:
        tmp=Path(folder)
        fixtures=tmp/"fixtures"
        fixtures.mkdir()
        for name in ("application.csv","camera.csv","withdrawal.csv","immediate.csv","LowWindowTest.java"):
            shutil.copy(HERE/name,fixtures/name)
        shutil.copy(STEREO/"CustodyClockTest.java",fixtures)
        source=(STEREO/"CustodyEvidence.java").read_text(encoding="utf-8")
        original="6389d766ad3d4d65b885bccb615a23807cadd8ef4f14d846214cea8a4dd96f73"
        if source.count(original)!=1:raise ValueError("source binding changed")
        (fixtures/"CustodyEvidence.java").write_text(source.replace(original,result["poses"]["0"]["mesh_sha256"]),
                                                    encoding="utf-8",newline="\n")
        changed=(PORTS/"PortAcquisition.java").read_text(encoding="utf-8").replace("speed = 0.45;","speed = 0.65;")
        (fixtures/"PortAcquisition.java").write_text(changed,encoding="utf-8",newline="\n")
        issued=json.loads((ROOT/"design/fixed-height/ports-065.json").read_bytes())
        if digest(fixtures/"PortAcquisition.java")!=issued["compiledSourceSha256"]:raise ValueError("low port profile differs")
        env=dict(os.environ,JAVA_HOME=str(jdk),GRADLE_USER_HOME=str(cache),CUSTODY_SOURCE=str(fixtures),
                 REEF_CONTROLS_SOURCE=str(ROOT/"competition-controls"),REEF_CONTROLS_BUILD=str(tmp/"build"))
        env["PATH"]=str(jdk/"bin")+os.pathsep+env["PATH"]
        wrapper=PROJECT/("gradlew.bat" if os.name=="nt" else "gradlew")
        subprocess.run([str(wrapper),"--offline","--no-daemon","--rerun-tasks","--project-cache-dir",str(tmp/"cache"),
                        "-I",str(ROOT/"competition-controls/init.gradle"),"-I",str(STEREO/"init.gradle"),
                        "test","--tests","org.reefscape.controls.CustodyClockTest","--tests","org.reefscape.controls.LowWindowTest"],
                       cwd=PROJECT,env=env,check=True)
        logs={}
        for name in ("CustodyClockTest","LowWindowTest"):
            report=ElementTree.parse(tmp/f"build/test-results/test/TEST-org.reefscape.controls.{name}.xml").getroot()
            if any(int(report.get(key))!=v for key,v in (("tests",1),("skipped",0),("failures",0),("errors",0))):
                raise ValueError("scheduler verification incomplete")
            logs[name]=next(line for line in report.findtext("system-out","").splitlines() if line.startswith(("CUSTODY_","LOW_WINDOWS=")))
        data={"scheduler_period_us":20000,"results":logs,"compiled_source_sha256":
            {f.name:digest(f) for f in sorted(fixtures.glob("*.java"))},"input_sha256":
            {n:digest(HERE/n) for n in ("results.json","application.csv","camera.csv","withdrawal.csv","immediate.json","immediate.csv","contacts.json")}}
        (HERE/"hal.json").write_text(json.dumps(data,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--java-home",type=Path,required=True)
    p.add_argument("--gradle-home",type=Path,required=True)
    args=p.parse_args()
    run(args.java_home.resolve(),args.gradle_home.resolve())
