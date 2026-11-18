"""Bind the corrected field-start configuration to the exact red bumper inventory and its optical clock."""
import argparse
import json
from pathlib import Path

from match import ROOT, MARKS, corrected
from mission.review_combined_departure import POSES, SuppliedRobot, digest, prepare_extreme, review
from mission import short_vision_clock


def write(path,data):
    path.write_text(json.dumps(data,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


def run(poses,folder):
    folder.mkdir(parents=True,exist_ok=True)
    configuration=ROOT/"full-match/field-trial.json"
    actual=json.loads((poses/"review.json").read_text())
    paths={k:poses/(k+".step") for k in POSES}
    robot=SuppliedRobot(poses/"part-frames.json",paths,hashes={k:actual[k]["export_sha256"] for k in POSES},
                        radial=json.loads((poses/"rotation-bounds.json").read_text()),outer_error=15,grip_error=1.5)
    prepare_extreme(robot,ROOT/"design/receiver-drives/receiver_drives.py")
    motion=corrected()
    result=review(robot,delay=.11,contact_root=ROOT,motion=motion,field_obstacles=MARKS)
    result["full_match_configuration_sha256"]=digest(configuration)
    result["travel_basis"]="Independent static opening stack checks on completed red bumper geometry; see separate concurrent match scenarios."
    report=folder/"red-assembly.json"
    write(report,result)
    timing=short_vision_clock.run(ROOT,report,poses,motion=motion)
    timing["full_match_configuration_sha256"]=digest(configuration)
    write(folder/"red-withdrawal-clock.json",timing)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--poses",type=Path,required=True)
    p.add_argument("--output",type=Path,default=Path(__file__).resolve().parent)
    a=p.parse_args()
    run(a.poses,a.output)
