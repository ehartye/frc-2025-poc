"""Replay both mission traces with explicit sampling phase, processing delay and pixel bounds."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys

from stereo import clearance, endpoints, image_pairs

ROOT=Path(__file__).resolve().parents[1]


def evaluate(mission, pixel_error, registration, size_margin, frame_ms, transport_ms):
    # Retain the interval after powered support ends. Every previous support sample can read
    # both receiver ports clear; it cannot earn this independent clearance.
    support=next(e["time_s"] for e in mission["events"] if e["event"]=="left-powered-shelf-with-same-piece")
    physical=next((e["time_s"] for e in mission["events"] if e["event"]=="finite-pipe-envelope-clear-of-shelf-by-1mm"),None)
    rows=[r for r in mission["trace"] if r["time_s"]>=support]
    frames=[]
    for r in rows:
        marks=image_pairs(endpoints(r))
        gap=clearance(marks,1000*r["shelf_front_robot_u_m"],1000*r["shelf_top_m"]+.8,
                      pixel_error,registration,size_margin)
        frames.append((r["time_s"],gap,marks))
    if not frames: raise ValueError("empty release interval")
    pad_times=[]
    for r in mission["trace"]:
        if r["phase"]=="place" and r["state"]=="supported":
            a,b=endpoints(r)
            # Center pad, transverse tube. Idealized flat tangent passing the active disk.
            y_fraction=-a[1]/(b[1]-a[1])
            u=a[0]+(b[0]-a[0])*y_fraction-1000*r["slide_m"]
            if 0<=y_fraction<=1 and abs(u-155)<=2.8: pad_times.append(r["time_s"])
    # 2 ms multi-channel sampling and 1 ms synchronization are design budgets.
    lower=(pad_times[-1]-.003) if pad_times else None
    phases=[]
    for phase in range(frame_ms):
        observed=next((t for t,gap,_ in frames
                       if round(1000*t)%frame_ms==phase and gap>=1),None)
        if observed is not None:
            phases.append(observed)
    uv=[p for _,_,marks in frames for mark in marks for p in mark]
    delays=[round(1000*(t-physical),2) for t in phases] if physical is not None else []
    return dict(case=mission["case"],piece=mission["piece_id"],pixel_bound=pixel_error,
                registration_bound_mm=registration,size_bound_mm=size_margin,truth_s=physical,
                frame_period_ms=frame_ms,processing_transport_budget_ms=transport_ms,
                eligible_phases=len(phases),total_phases=frame_ms,samples=len(frames),
                last_ideal_contact_pad_s=pad_times[-1] if pad_times else None,
                bounded_timing_phases=sum(t+.001+.050+.020<=lower+.150+1e-9 for t in phases) if lower else 0,
                best_gap_bound_mm=round(max(g for _,g,_ in frames),3),
                image_bounds_px=[round(f(p[i] for p in uv),2) for f,i in [(min,0),(max,0),(min,1),(max,1)]],
                capture_lag_ms=[min(delays),max(delays)] if delays else None,
                worst_controller_receipt_ms=round(max(delays)+transport_ms+20,2) if delays else None)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mission-root",type=Path,default=ROOT)
    p.add_argument("--output",type=Path)
    args=p.parse_args()
    scripts=args.mission_root.resolve()/"scripts"
    sys.path.insert(0,str(scripts))
    import station_l1_mission as producer
    from registered_transit import RegisteredMission
    reports=[]
    for approach,lane in (("centered-bench",None),("field-registered",RegisteredMission(-.18))):
        for case in ("reference","slow-release"):
            mission=producer.run(case,.001,sample_every_step=True,trajectory=lane,target_y=-.18 if lane else 0)
            for pixel in (.10,.25,.5,1.0):
                row=evaluate(mission,pixel,1,.5,17,8)
                reports.append(dict(approach=approach,**row))
    result=dict(classification="synthetic ideal feature correspondence, uncalibrated, no hardware authorization",
                mission_source_sha256=sha256((scripts/"station_l1_mission.py").read_bytes()).hexdigest(),reports=reports)
    out=json.dumps(result,indent=2,allow_nan=False)+"\n"
    if args.output: args.output.write_text(out,encoding="utf-8",newline="\n")
    else: print(out,end="")

if __name__=="__main__":
    main()
