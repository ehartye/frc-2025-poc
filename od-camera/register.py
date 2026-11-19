"""Bind the 593-body input and this candidate's native, optical and power records."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
INPUTS = (
    "design/fixed-no-air/whole-robot.json", "design/fixed-no-air/inspection.json",
    "moving-observer/mount.json", "moving-observer/moving-bridge.step",
    "od-gate/split-gate.step", "od-gate/package.json", "od-gate/engineering.json",
    "od-gate/observer.json", "od-gate/study.json", "od-gate/motion.py",
    "fixed-low-power/dual-camera.json", "fixed-low-power/dual_camera.py",
    "kickoff/rule-register-v0.json"
)
RECORDS = (
    "revised-bridge.step", "unmodified.json", "first-bridge-control.json",
    "foot-width-control.json", "rearward-aim-control.json", "native.json",
    "travel.json", "departure.json", "ranges.json", "images.json", "budget.json",
    "station.json"
)

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def build():
    return {
        "schema": "outside-diameter-dual-camera/1",
        "source": {p: digest(ROOT / p) for p in INPUTS},
        "evidence": {p: digest(HERE / p) for p in RECORDS},
        "localSourceDigests": {
            "January FIRST field": "19374bc5a48fda1b3ce0505ef604a47d227c904d1784b6514a2c7d654bf3d918",
            "mounted product lamp": "6dc6d4c850e6cf5798d3b2082ffc15527ad5cdd0869b641439cf62905ee074ca"
        }
    }

if __name__ == "__main__":
    (HERE / "register.json").write_text(json.dumps(build(), indent=2) + "\n", encoding="utf8", newline="\n")
