"""Exercise an observable lost-entry response through the issued motor ramp and two-port Java program."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "return-station"))
from feed import port_program, trial


def challenge(args):
    classes, jdk, cache, name, delta, command, dt = args
    label = f"{name}-{command}-{round(dt*1e6)}"
    with subprocess.Popen([str(jdk/"bin/java"), "-cp", str(classes), "org.reefscape.controls.PortMotion", str(command)],
                          stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1) as proc:
        r = trial(proc, delta, label, dt, 0, cache, relief=True, until_ms=480)
        proc.stdin.close()
        proc.wait()
    changed = next((row[0] for row in r["portTicks"] if row[4] == command), None)
    return {"case": name, "challengeMps": command, "dtS": dt, "firstChangedDemandMs": changed,
            "stopAtControllerMs": None if r["hardStop"] is None else round(100+1000*r["hardStop"][0], 6),
            "stopTravelMm": None if r["hardStop"] is None else r["hardStop"][2],
            "lastPortState": r["lastPortState"], "firstPortBandMs": r["firstPortBandMs"],
            "loss": r["loss"], "trajectorySha256": r["trajectorySHA256"]}


if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("cache", type=Path)
    cli.add_argument("jdk", type=Path)
    a = cli.parse_args()
    a.cache.mkdir(parents=True, exist_ok=True)
    poses = json.loads((ROOT/"station-landmark/feed.json").read_bytes())["outcomes"]
    with tempfile.TemporaryDirectory() as folder:
        classes = Path(folder)
        source = classes/"PortAcquisition.java"
        source.write_text(port_program(True), encoding="utf8", newline="\n")
        subprocess.run([str(a.jdk/"bin/javac"), "-d", str(classes), str(source), str(HERE/"PortMotion.java")], check=True)
        jobs = [(classes, a.jdk, a.cache, name, poses[name]["actualRobotMmDeg"], command, dt)
                for name in ("nominal","leftSmall","rightSmall") for command in (0.0,-0.65)
                for dt in (.00025,.000125,.0000625)]
        with ProcessPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(challenge, jobs))
    (HERE/"motion.json").write_text(json.dumps(results, indent=2)+"\n", encoding="utf8", newline="\n")
