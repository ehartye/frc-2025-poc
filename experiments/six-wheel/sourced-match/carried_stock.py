"""Check each attained loaded centerline against every January starting piece and the first delivery."""
import json
import math
from pathlib import Path

import numpy as np

from replay_source_match import ROOT, SECTORS, corrected, digest, transfer_model
from mission.field_body_clearance import HALF, RAD

HERE = Path(__file__).resolve().parent
PIPE = math.hypot(HALF + 2, RAD + 2) / 1000
STOCK = ROOT / "full-match/installed-stock.json"
STOCK_SHA = "f4c4be4d987bb7cc01b579d699b62e9673fd05540c32fbd4326b78b8ab218589"


def closest(points, times, center, combined_radius):
    """Distance to every piecewise-linear segment, reduced by both enclosing radii."""
    segment = np.diff(points, axis=0)
    delta = center - points[:-1]
    denom = np.einsum("ij,ij->i", segment, segment)
    numer = np.einsum("ij,ij->i", delta, segment)
    fraction = np.clip(np.divide(numer, denom, out=np.zeros_like(numer), where=denom>0), 0, 1)
    residual = delta - segment * fraction[:, None]
    gap = np.linalg.norm(residual, axis=1) - combined_radius
    i = int(np.argmin(gap))
    return {"lower_bound_m": float(gap[i]), "teleop_s": float(times[i]+fraction[i]*(times[i+1]-times[i]))}


def run():
    if digest(STOCK) != STOCK_SHA:
        raise ValueError("Installed field roster differs from reviewed January edition")
    stock = json.loads(STOCK.read_bytes())
    source = json.loads((HERE/"source-545.json").read_bytes())
    replay = json.loads((HERE/"shop-545.json").read_bytes())
    if digest(HERE/"source-545.json") != replay["source_receipt_sha256"]:
        raise ValueError("Feeder and match source differ")
    f = corrected()
    obstacles = {}
    for occurrence, row in stock["stock"].items():
        x, up, z = row["native_center_mm"]
        wx, wy = f.field_from_native(x/25.4,z/25.4)
        loc = [wx-f.LIP[0],wy-f.LIP[1],up/1000]
        radius = stock["ball_radius_mm"]/1000 if row["group"].endswith("ball") else PIPE
        obstacles[occurrence] = (row["group"], np.array(loc), radius)
    if len(obstacles)!=24:
        raise ValueError("Incomplete installed stock")
    result={}
    for entry, case in zip(source["identified_transfers"], replay["case_studies"], strict=True):
        state, upstream = transfer_model(source,entry,f)
        key=state["piece_id"]
        rows=upstream["trace"]
        points=np.array([r["target_xyz_m"] for r in rows])
        times=np.array([state["lip_teleop_s"]+r["time_s"] for r in rows])
        if key!=case["state"]["piece_id"] or max(np.diff(times))>.001001:
            raise ValueError("Trace identity or maximum interval changed")
        result[key]={}
        for occurrence,(kind,center,radius) in obstacles.items():
            gap=closest(points,times,center,PIPE+radius)
            if kind not in result[key] or gap["lower_bound_m"]<result[key][kind]["lower_bound_m"]:
                result[key][kind]={"occurrence":occurrence,**gap}
        x,z=case["zero_spin"]["settled_xz_m"]
        obstacles[key]=("previously-supported-pipe",np.array((x,SECTORS[key],z)),PIPE)
    return {"source_pickup_sha256":digest(HERE/"source-545.json"),"installed_stock_sha256":STOCK_SHA,
            "stock_count":len(stock["stock"]),"enclosing_pipe_radius_m":PIPE,
            "window":"Actual 1 ms seat-to-first-trough-contact trajectories, linear between integration samples. Includes a 2 mm radial and axial end allowance. The first settled pipe is added for the second trip.",
            "scope":"Fixed stock; no setup shifts, post-impact spin or post-impact pipe trajectories included.",
            "negative_control_m":closest(np.array([[0.,0,0],[2.,0,0]]),np.array([0.,1]),np.array([1.,0,0]),2*PIPE)["lower_bound_m"],
            "nearest_by_group":result}


if __name__=="__main__":
    (HERE/"carried-stock.json").write_text(json.dumps(run(),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
