"""Sweep unmeasured target axial spin using the two actual station-fed trajectories."""
import json
from pathlib import Path
from replay import SOURCE,corrected,transfer_model,settle,SECTORS


def run():
    data=json.loads(SOURCE.read_bytes())
    f=corrected()
    results={}
    for t in data["identified_transfers"]:
        key=t["id"]
        state,up=transfer_model(data,t,f)
        cases={}
        for delta in (-29.690655456963764,-20,-15,-10,-5,0,5,10,15,20,29.690655456963764):
            case=settle.continue_contact(sector=SECTORS[key],upstream=up,piece_id=key,withdrawal="after-separation",
                                  wait_after_true_separation_s=.110,dt=.0002,initial_roll=delta)
            cases[str(round(delta,6))]={"outcome":case["stop"],"duration_s":case["elapsed_s"],
                                        "min_shelf_gap_mm":case["min_shelf_clearance_mm"]}
        results[key]=cases
    return {"source":"station-supply/transfer.json","basis":"Target contact axial spin is unmeasured. Fixed delay 110 ms and 0.2 ms settling integration.",
            "runs":results}


if __name__=="__main__":
    Path(__file__).with_name("spin-screen.json").write_text(json.dumps(run(),indent=2)+"\n",encoding="utf8",newline="\n")
