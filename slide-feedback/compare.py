"""Check the source-specific 593-body front transfer without issuing a new optical permit."""
import argparse
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PIN = "8b2420a0ac24468ed0246fd468af30128134e1fe223dede3d62fd6c477d0dbf1"


def compare(path):
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != PIN:
        raise ValueError("No-air installation requires a fresh review")
    register = json.loads(raw)
    for name, expected in register["source_sha256"].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise ValueError("Geometry source changed: "+name)
    cases = {}
    for name, fixture in register["cases"].items():
        stay = fixture["identical_brep_sha256"]
        reviewed = fixture["source_front"]
        if (len(stay) != 590 or reviewed != fixture["match_front"]
                or any(body not in stay for body in reviewed["bodies"])
                or fixture["new_fixed_change_front"]["x_mm"] > 425):
            raise ValueError("Native front transfer failed")
        cases[name] = {"strokeMm": fixture["stroke_mm"], "frontMm": reviewed["x_mm"],
                       "governingBodies":reviewed["bodies"], "unchangedNativeBodies":len(stay),
                       "sourceBodies":fixture["source_bodies"],"successorBodies":fixture["match_bodies"]}
    data = {"registerSha256":PIN,"cases":cases,"carriedCameraAllowanceKg":.440,
            "newOrChangedBaseFrontMaxMm":337,"opticalGateTransferred":False,
            "evidence":"Whole-robot front and rack moving-mass comparison on the fixed-low red installation."}
    (HERE/"no-air.json").write_text(json.dumps(data,indent=2)+"\n", encoding="utf-8",newline="\n")


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--register",type=Path,default=ROOT/"design/fixed-no-air/whole-robot.json")
    compare(p.parse_args().register)
