"""Check both recalculated moving pieces against installed stock and each admitted camera timing."""
import json
import numpy as np

from spin_cases import CANDIDATE
from pipe_spin_route import ROOT,HERE,DATA,SECTORS,trial,corrected,vision,settle,digest
from pipe_axial_contact import K
from carried_stock import closest,STOCK,STOCK_SHA,PIPE


def run():
    if digest(STOCK)!=STOCK_SHA:raise ValueError("Unexpected installed stock")
    installed=json.loads(STOCK.read_bytes())
    f=corrected()
    obstacles={}
    for occurrence,item in installed["stock"].items():
        x,h,z=item["native_center_mm"]
        wx,wy=f.field_from_native(x/25.4,z/25.4)
        radius=installed["ball_radius_mm"]/1000 if item["group"].endswith("ball") else PIPE
        obstacles[occurrence]=(item["group"],np.array((wx-f.LIP[0],wy-f.LIP[1],h/1000)),radius)
    results={}
    for entry in DATA["identified_transfers"]:
        key=entry["id"]
        result,up,_=trial(CANDIDATE,entry,.0005)
        pts=np.array([p["target_xyz_m"] for p in up["trace"]])
        times=np.array([p["time_s"]+result["lip_teleop_s"] for p in up["trace"]])
        bounds={}
        for occurrence,(group,center,radius) in obstacles.items():
            bound=closest(pts,times,center,PIPE+radius)
            if group not in bounds or bound["lower_bound_m"]<bounds[group]["lower_bound_m"]:
                bounds[group]={"occurrence":occurrence,**bound}
        x,z=result["target"]["final"][:2]
        obstacles[key]=("first-delivery",np.array((x,SECTORS[key],z)),PIPE)
        start=up["trace"][0]["time_s"]
        observed=dict(up,trace=[r for r in up["trace"] if abs((r["time_s"]-start)*1000-round((r["time_s"]-start)*1000))<.0001])
        budgets,_=vision.schedule(observed,537)
        intervals=[e["onset_after_shelf_ms"] for e in budgets["0.1"]["0"]["exposures"] if e["within_150ms"]]
        admissible=sorted({ms for low,high in intervals for ms in range(low,high+1)})
        outcomes=[]
        for delay in admissible:
            post=settle.continue_contact(sector=SECTORS[key],upstream=up,piece_id=key,initial_roll=result["release_spin_rad_s"],
                          inertia_ratio=K,withdrawal="after-separation",wait_after_true_separation_s=delay/1000,dt=.0002)
            outcomes.append([delay,post["stop"],post["min_shelf_clearance_mm"]])
        results[key]={"loaded_bounds_m":bounds,"tight_pixel_integer_ms_settling":outcomes}
    return {"installed_stock_sha256":STOCK_SHA,"study_sha256":digest(HERE/"study.json"),"cases":results}


if __name__=="__main__":
    (HERE/"field-timing.json").write_text(json.dumps(run(),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
