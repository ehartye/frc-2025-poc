"""Test every discrete onboard withdrawal onset admitted by the two recorded image budgets."""
import json
from pathlib import Path

from replay_source_match import SECTORS, corrected, settle, transfer_model

HERE = Path(__file__).resolve().parent


def run():
    project = json.loads((HERE/"shop-545.json").read_bytes())
    pickup = json.loads((HERE/"source-545.json").read_bytes())
    reference = json.loads((HERE/"reference-401.json").read_bytes())
    field = corrected()
    results={}
    for transfer,case,prior in zip(pickup["identified_transfers"],project["case_studies"],reference["case_studies"],strict=True):
        state,upstream=transfer_model(pickup,transfer,field)
        piece=state["piece_id"]
        if case["images"]!=prior["images"] or state!=case["state"]:
            raise ValueError("Timing or initial state differs between issued models")
        ms=set()
        for phases in case["images"].values():
            for exposure in phases.values():
                if exposure["eligible"]:
                    lower,upper=exposure["onset_after_shelf_ms"]
                    ms.update(range(lower,upper+1))
        rows=[]
        for delay in sorted(ms):
            row=settle.continue_contact(sector=SECTORS[piece],upstream=upstream,piece_id=piece,
                   withdrawal="after-separation",dt=.0002,wait_after_true_separation_s=delay/1000)
            rows.append({"delay_ms":delay,"result":row["stop"],"min_shelf_gap_mm":row["min_shelf_clearance_mm"]})
        results[piece]=rows
    return {"basis":"Every integer-millisecond onset in the eligible zero-added-wait exposures of either pixel budget. Source-derived motion, zero axial spin, 0.2 ms integrator, 2 s support check. Intermediate untested delay values are not promoted.",
            "cases":results}


if __name__=="__main__":
    (HERE/"onset-sweep.json").write_text(json.dumps(run(),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
