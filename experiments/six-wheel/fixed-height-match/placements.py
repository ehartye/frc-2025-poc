"""Replay the two issued low intakes to the buzzer, including the stock along each pipe path."""
import hashlib
import json
import math
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "design/fixed-height"))
import handoff
from handoff import spin
from carried_stock import PIPE, STOCK, STOCK_SHA, closest


def read(path):
    return json.loads(path.read_bytes())


def run():
    if hashlib.sha256(STOCK.read_bytes()).hexdigest() != STOCK_SHA:
        raise ValueError("Installed field inventory changed")
    model = read(STOCK)
    f = spin.corrected()
    obstacles = {}
    for name, item in model["stock"].items():
        x, h, z = item["native_center_mm"]
        wx, wy = f.field_from_native(x / 25.4, z / 25.4)
        radius = model["ball_radius_mm"] / 1000 if item["group"].endswith("ball") else PIPE
        obstacles[name] = (np.array([wx-f.LIP[0], wy-f.LIP[1], h/1000]), radius)
    if len(obstacles) != 24:
        raise ValueError("Expected 24 distinct originally installed field pieces")
    pins = read(ROOT / "design/fixed-height/ports-065.json")
    expected = read(ROOT / "design/fixed-height/handoff.json")
    branch = read(ROOT / "mission/branch-exclusion.json")
    sources = []
    for piece, key in zip(spin.DATA["identified_transfers"], ("0ms-0.0005", "19ms-0.00025"), strict=True):
        real_settle = spin.settle.continue_contact
        capture = {}
        def replay(**kwargs):
            post = real_settle(**kwargs)
            if kwargs["withdrawal"] == "after-separation":
                capture.update(kwargs=kwargs, post=post)
            return post
        with patch.object(spin.settle, "continue_contact", replay):
            identity = piece["id"]
            case = handoff.one(piece, "RAMP_090", pins["cases"][key])
        original = next(row for row in expected if row["command"] == "RAMP_090"
                        and row["acquisitionCounterfeedMps"] == .65 and row["sourceId"] == identity)
        if case["after-separation"] != original["after-separation"]:
            raise ValueError("Replay differs from the low registered outcome")
        kwargs, post = capture["kwargs"], capture["post"]
        upstream = kwargs["upstream"]
        impact = case["after-separation"]["targetContactTeleopS"]
        lip = impact - post["start_from_station_s"]
        end = real_settle(**kwargs, duration=138-impact)
        if end["stop"] != "sustained-paired-floor-and-lip":
            raise ValueError("Target support lost before assessment")
        events = {e["event"]: lip+e["time_s"] for e in upstream["events"]}
        settled = post["last_xz_vxvz_spin"][:2]
        final = end["last_xz_vxvz_spin"][:2]
        sector = spin.SECTORS[identity]
        points = np.array([r["target_xyz_m"] for r in upstream["trace"]])
        times = np.array([lip+r["time_s"] for r in upstream["trace"]])
        loaded=f.routes(sector)[1]
        drive=events["drive-and-turn"]
        hull_clearance=math.inf
        for t,location in zip(times,points):
            if drive<=t<=drive+loaded.duration:
                cx,cy,yaw=loaded.at(t-drive)[0]
                dx,dy=f.rot((location[0]+f.LIP[0]-cx,location[1]+f.LIP[1]-cy),-yaw)
                hull_clearance=min(hull_clearance,.425-max(abs(dx),abs(dy))-PIPE)
        bounds = {name: closest(points, times, point, PIPE+rad) for name,(point,rad) in obstacles.items()}
        opts = next(b for b in branch["trips"] if b["piece"] == identity)
        delta = float(np.linalg.norm(np.array((final[0],sector,final[1]))-opts["target_x_y_z_m"]))
        if len(opts["pipe_distances_mm"]) != 12:
            raise ValueError("Missing branch exclusion reference")
        branch_gap = min(opts["pipe_distances_mm"].values())-1000*delta
        obstacles[identity] = (np.array([final[0],sector,final[1]]), PIPE)
        sources.append(dict(id=identity, sector=sector, release=piece["human_release_teleop_s"],
                            lip=lip, seat=times[0], drive=events["drive-and-turn"], impact=impact,
                            supported=impact+post["elapsed_s"], settledXZ=settled, finalXZ=final,
                            supportAt138=end["stop"], stillSeconds=end["ending_still_double_contact_s"],
                            clear=lip+post["clear_witness_from_station_s"], retreatDelay=.110,
                            stockedPipeBounds=bounds, finalBranchLowerMm=branch_gap,
                            carriedPipeInsideMovingChassisMarginM=hull_clearance))
    return {"stockSha256": STOCK_SHA, "placements": sources}


if __name__ == "__main__":
    (HERE/"placements.json").write_text(json.dumps(run(),indent=2,allow_nan=False)+"\n", encoding="utf-8",newline="\n")
