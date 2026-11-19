"""Re-run every coupled sample through the isolated WPILib HAL desktop test source set."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory
from xml.etree import ElementTree

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PROJECT = ROOT/"shared/examples/java-2026-sim"


def run(jdk, cache):
    trace = HERE/"ticks.csv"
    with trace.open() as file:
        rows = list(csv.DictReader(file))
    n = len(rows)
    trials = sum(r["ms"] == "0" for r in rows)
    with TemporaryDirectory() as tempdir:
        temp = Path(tempdir)
        env = dict(os.environ, JAVA_HOME=str(jdk), GRADLE_USER_HOME=str(cache), BELT_CLOCK_SOURCE=str(HERE),
                   REEF_CONTROLS_SOURCE=str(ROOT/"competition-controls"), REEF_CONTROLS_BUILD=str(temp/"build"),
                   PORT_TRACE=str(trace))
        env["PATH"] = str(jdk/"bin") + os.pathsep + env["PATH"]
        wrapper = PROJECT/("gradlew.bat" if os.name == "nt" else "gradlew")
        cmd = [str(wrapper), "--offline", "--no-daemon", "--rerun-tasks", "--project-cache-dir", str(temp/"cache"),
               "-I", str(ROOT/"competition-controls/init.gradle"), "-I", str(ROOT/"coupled-belt-clock/init.gradle"),
               "test", "--tests", "org.reefscape.controls.PortClockTest"]
        subprocess.run(cmd, cwd=PROJECT, env=env, check=True)
        xml = temp/"build/test-results/test/TEST-org.reefscape.controls.PortClockTest.xml"
        root = ElementTree.parse(xml).getroot()
        if any(int(root.get(k)) != v for k,v in (("tests",1),("errors",0),("failures",0),("skipped",0))):
            raise ValueError("Incomplete desktop trial")
        if f"PORT_TICKS={n} TRIALS={trials}" not in root.findtext("system-out","").splitlines():
            raise ValueError("Unexpected trace inventory")
        sources = [trace, HERE/"PortAcquisition.java", HERE/"PortClockTest.java", HERE/"verify_hal.py",
                   PROJECT/"gradle.lockfile", PROJECT/"build.gradle", ROOT/"coupled-belt-clock/init.gradle",
                   ROOT/"competition-controls/init.gradle"]
        proof = {"ticks": n, "trials": trials, "period_us": 20000,
                 "source_sha256": {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in sources}}
        (HERE/"hal.json").write_text(json.dumps(proof,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--java-home", required=True, type=Path)
    parser.add_argument("--gradle-home", required=True, type=Path)
    args = parser.parse_args()
    run(args.java_home.resolve(), args.gradle_home.resolve())
