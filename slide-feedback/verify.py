"""Compile the desktop controller with the project's pinned WPILib build and run actual HAL clocks."""
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

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PROJECT = ROOT / "shared/examples/java-2026-sim"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(jdk, cache, observer, output):
    output.mkdir(parents=True, exist_ok=True)
    phases = json.loads((observer / "handoff.json").read_bytes())
    original = list(csv.DictReader((ROOT / "low-observation/immediate.csv").open()))
    with tempfile.TemporaryDirectory() as folder:
        tmp = Path(folder)
        register = hashlib.sha256()
        for filename in ("moving-bridge.step", "handoff.json", "clearance.json"):
            register.update((observer / filename).read_bytes())
        (tmp / "register.txt").write_text(register.hexdigest(), encoding="ascii")
        for file in ("Retreat.java", "SlideFeedbackTest.java"):
            shutil.copy(HERE / file, tmp / file)
        with (tmp / "phases.csv").open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(["piece", "phase", "earliest_pad_ms", "capture_ms", "handoff"])
            for row in original:
                name, phase = row["piece"], int(row["phase"])
                witness = phases[name][phase]
                if witness["phase_ms"] != phase or witness["first_forward_ms"] != int(row["capture_ms"]):
                    raise ValueError("Handoff differs from issued exposure")
                joined = witness["overlap_joined_ms"]
                timely = joined is not None and joined < witness["first_forward_ms"] and not witness["fixed_track_lost"]
                writer.writerow([name, phase, row["prior_lower_ms"], row["capture_ms"], str(timely).lower()])
        env = dict(os.environ, JAVA_HOME=str(jdk), GRADLE_USER_HOME=str(cache), CUSTODY_SOURCE=str(tmp),
                   SLIDE_OUTPUT=str(output), REEF_CONTROLS_SOURCE=str(ROOT / "competition-controls"),
                   REEF_CONTROLS_BUILD=str(tmp / "build"))
        env["PATH"] = str(jdk / "bin") + os.pathsep + env["PATH"]
        wrapper = PROJECT / ("gradlew.bat" if os.name == "nt" else "gradlew")
        subprocess.run([str(wrapper), "--offline", "--no-daemon", "--rerun-tasks", "--project-cache-dir",
                        str(tmp / "cache"), "-I", str(ROOT / "competition-controls/init.gradle"), "-I",
                        str(ROOT / "stereo-custody/init.gradle"), "test", "--tests",
                        "org.reefscape.controls.SlideFeedbackTest"], cwd=PROJECT, env=env, check=True)
        result = ElementTree.parse(tmp/"build/test-results/test/TEST-org.reefscape.controls.SlideFeedbackTest.xml").getroot()
        if tuple(result.get(k) for k in ("tests","errors","skipped","failures")) != ("1","0","0","0"):
            raise ValueError("HAL regression incomplete")
        print("Verified native HAL replay", sha(output / "trials.csv"))


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--java-home",type=Path,required=True)
    p.add_argument("--gradle-home",type=Path,required=True)
    p.add_argument("--observer",type=Path,default=ROOT/"moving-observer")
    p.add_argument("--output",type=Path,default=ROOT/"artifacts/slide-feedback")
    a=p.parse_args()
    run(a.java_home.resolve(),a.gradle_home.resolve(),a.observer.resolve(),a.output.resolve())
