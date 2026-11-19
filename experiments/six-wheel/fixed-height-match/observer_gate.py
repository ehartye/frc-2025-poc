"""Read the independent optical clocks and keep physical and timing-only permissions distinct."""
import argparse
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def read_gate(folder):
    originals={k:(folder/(k+".json")).read_bytes() for k in ("results","immediate","contacts")}
    data={k:json.loads(v) for k,v in originals.items()}
    pieces={}
    for key,case in data["results"]["pieces"].items():
        alt=data["immediate"]["pieces"][key]
        phases=case["phases"]
        immediate=alt["phases"]
        if len(phases)!=17 or len(immediate)!=17 or alt["physically_calibrated"] or alt["continuous_track_established"]:
            raise ValueError("Observer evidence configuration changed; review the qualification decision")
        issued_count=sum(p["timely"] for p in phases)
        if issued_count:
            raise ValueError("Issued controller now has eligible phases; review the match gate")
        control=data["contacts"][key]["slide-held"]
        pieces[key]={"stationCorroboratedTeleopS":case["first_usable_station_teleop_s"],
                     "issuedTimelyPhases":issued_count,"immediateTimingOnlyPhases":sum(p["time_window_only"] for p in immediate),
                     "totalPhases":len(phases),"sourceThroughCarryObserved":alt["continuous_track_established"],
                     "physicallyCalibrated":alt["physically_calibrated"],"heldSlideOutcome":control["stop"],
                     "heldAfterContactS":control["elapsed_s"],"matchAuthorized":False}
    if set(pieces)!={"coral-A","coral-B"}:
        raise ValueError("Both identified handoffs are required")
    return {"sourceSha256":{k:hashlib.sha256(v).hexdigest() for k,v in originals.items()},"pieces":pieces}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("study",type=Path)
    args=p.parse_args()
    (HERE/"observer.json").write_text(json.dumps(read_gate(args.study),indent=2)+"\n",encoding="utf-8",newline="\n")
