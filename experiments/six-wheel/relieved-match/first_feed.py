"""Re-run the first human feed at the carriage actually present at the opening station."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "return-station"))
from feed import port_program, trial


def run(cache, jdk, piece, output=HERE):
    cache.mkdir(parents=True, exist_ok=True)
    plans = json.loads((ROOT / "fixed-height-match/placements.json").read_bytes())["placements"]
    plan = next(row for row in plans if row["id"] == piece)
    phase = 0 if piece == "coral-A" else 19
    label = "first" if piece == "coral-A" else "return"
    delta = [0, 0, 0]
    if piece == "coral-B":
        rows = json.loads((HERE / "navigation.json").read_bytes())["cases"]
        delta = next(r["actualStationMmDeg"] for r in rows if r["pipe"] == piece and r["mode"] == "nominal")
    epoch = round((18 + plan["lip"]) * 1e6) - (100+phase)*1000
    local = (18 + plan["drive"]) * 1000 - epoch / 1000
    records = []
    with tempfile.TemporaryDirectory() as folder:
        folder = Path(folder)
        program = folder / "PortAcquisition.java"
        program.write_text(port_program(True), encoding="utf8", newline="\n")
        program_digest = hashlib.sha256(program.read_bytes()).hexdigest()
        subprocess.run([str(jdk / "bin/javac"), "-d", str(folder), str(program),
                        str(ROOT / "observable-acquisition/PortConsole.java")], check=True)
        with subprocess.Popen([str(jdk / "bin/java"), "-cp", str(folder), "org.reefscape.controls.PortConsole"],
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1) as process:
            for mode, dt, side in [("nominal", .00025, 0), ("coarse", .0005, 0), ("displaced-source", .0005, 3)]:
                name = label + "-" + mode
                rec = trial(process, delta, name, dt, phase, cache, feed_side=side, relief=True, until_ms=5540)
                path = np.load(cache / (name + ".npy"))
                rec["scheduledDriveCameraMs"] = local
                rec["stateAtDrive"] = None
                if path[-1, 0] > local and rec["lastPortState"] == "IN_BAND":
                    sample = np.array([np.interp(local, path[:, 0], col) for col in path[:, 1:].T])
                    sample[3:6] /= np.linalg.norm(sample[3:6])
                    rec["stateAtDrive"] = sample.tolist()
                records.append(rec)
                print(name, rec["firstPortBandMs"], rec["hardStop"], rec["stateAtDrive"], flush=True)
            process.stdin.close()
            process.wait()
    result = {"identifiedPipe": piece, "stationChassisMmDeg": delta,
              "initialStationCondition": "registered AUTO arrives at issued datum with zero residual" if piece == "coral-A" else "wheel continuation from attained A reef pose",
              "epochWallUs": epoch, "humanReleaseWallS": 18 + plan["release"], "lipWallS": 18 + plan["lip"],
              "scheduledDriveWallS": 18 + plan["drive"], "javaSourceSha256": program_digest, "cases": records}
    (output / (label+"-feed.json")).write_text(json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cache", type=Path)
    parser.add_argument("jdk", type=Path)
    parser.add_argument("--piece", choices=["coral-A", "coral-B"], default="coral-A")
    args = parser.parse_args()
    run(args.cache, args.jdk, args.piece)
