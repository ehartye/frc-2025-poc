"""Rehearse clearance-witness latency, controller delay and actual retreat on two source approaches."""
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.registered_transit import RegisteredMission
from scripts.station_l1_mission import run
from scripts.station_l1_settle import continue_contact


def grid():
    delays=sorted(set(range(50,151,2))|{77,83,99,127,500,600})
    cases={}
    for label,path in (("centered-bench",None),("field-registered",RegisteredMission(-.18))):
        source=run(target_y=-.18,dt=.001,trajectory=path)
        rows=[]
        for ms in delays:
            follow=continue_contact(sector=-.18,upstream=source,withdrawal="after-separation",
                                    wait_after_true_separation_s=ms/1000)
            rows.append([ms,follow["stop"],round(follow["min_shelf_clearance_mm"],5),
                         round(follow["ending_still_double_contact_s"],5)])
        cases[label]=rows
    return {"schema":"receiver-latency-grid/1","columns":["delay_from_true_separation_ms","outcome",
            "min_shelf_gap_after_floor_mm","final_still_dual_contact_s"],"rows":cases,
            "limits":"Fixed 0.35 friction, zero entering roll, 0.1 ms contact step and 0.15 m/s retreat. Values between tested delays and unmodeled contact properties remain unqualified."}


if __name__=="__main__":
    print(json.dumps(grid(),indent=2,allow_nan=False))
