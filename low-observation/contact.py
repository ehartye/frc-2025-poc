"""Re-run the fixed-slide counterexample against the issued two-piece integrated plant."""
import argparse
import json
from pathlib import Path
from source import HERE, inputs


def run(jdk):
    receipts={}
    for piece,data in inputs(jdk).items():
        rows={}
        for label,source in (("slide-held",data["post_fixed"]),("110ms-after-modeled-separation",data["post"])):
            rows[label]={key:source[key] for key in ("elapsed_s","stop","ending_still_double_contact_s","dt_s",
                "min_shelf_clearance_mm","first_lip_after_s","initial_roll_rad_s","official_score")}
            rows[label]["end_teleop_s"]=data["report"]["after-separation"]["targetContactTeleopS"]+source["elapsed_s"]
        receipts[piece]=rows
    (HERE/"contacts.json").write_text(json.dumps(receipts,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--java-home",type=Path,required=True)
    run(p.parse_args().java_home)
