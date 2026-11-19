"""Run the isolated WPILib desktop study using the project's populated offline cache."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
from xml.etree import ElementTree

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PROJECT = ROOT / "shared/examples/java-2026-sim"


def capture(jdk, cache, output):
    with tempfile.TemporaryDirectory() as path:
        temp = Path(path)
        env = dict(os.environ, JAVA_HOME=str(jdk), GRADLE_USER_HOME=str(cache),
                   REEF_CONTROLS_SOURCE=str(ROOT / "competition-controls"),
                   REEF_CONTROLS_BUILD=str(temp / "build"), BELT_CLOCK_SOURCE=str(HERE),
                   BELT_CLOCK_TRACE=str(temp / "controller.csv"))
        env["PATH"] = str(jdk / "bin") + os.pathsep + env["PATH"]
        wrapper = PROJECT / ("gradlew.bat" if os.name == "nt" else "gradlew")
        subprocess.run([str(wrapper), "--offline", "--no-daemon", "--rerun-tasks",
                        "--project-cache-dir", str(temp / "cache"), "-I",
                        str(ROOT / "competition-controls/init.gradle"), "-I", str(HERE / "init.gradle"),
                        "test", "--tests", "org.reefscape.controls.DesktopClockTest"], cwd=PROJECT, env=env, check=True)
        xml = temp / "build/test-results/test/TEST-org.reefscape.controls.DesktopClockTest.xml"
        result = ElementTree.parse(xml).getroot()
        if any(int(result.get(k)) != v for k, v in (("tests", 1), ("failures", 0), ("errors", 0), ("skipped", 0))):
            raise ValueError("incomplete scheduler run")
        output.write_bytes((temp / "controller.csv").read_bytes())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--java-home", type=Path, required=True)
    parser.add_argument("--gradle-home", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=HERE / "controller.csv")
    args = parser.parse_args()
    capture(args.java_home.resolve(), args.gradle_home.resolve(), args.output)
