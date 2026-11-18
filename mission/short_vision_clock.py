"""Measure command-on-first-verified whole-body clearance and delayed-command alternatives on the short receiver."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mission import end_cartridge_vision as vision, heavy_field_clock as heavy
from mission.review_combined_departure import load_source, digest, recorded_departure, SuppliedRobot, POSES
from scripts import station_l1_settle as settle


def frames(up,front):
    support=next(e["time_s"] for e in up["events"] if e["event"]=="left-powered-shelf-with-same-piece")
    truth=next((e["time_s"] for e in up["events"] if e["event"]=="finite-pipe-envelope-clear-of-shelf-by-1mm"),None)
    last_pad=None
    rows=[]
    for r in up["trace"]:
        if r["phase"]=="place" and r["state"]=="supported":
            a,b=vision.stereo.endpoints(r)
            frac=-a[1]/(b[1]-a[1])
            u=a[0]+(b[0]-a[0])*frac-1000*r["slide_m"]
            if 0<=frac<=1 and abs(u-155)<=2.8:
                last_pad=r["time_s"]
        if r["time_s"]>=support:
            rows.append((r["time_s"],vision.stereo.image_pairs(vision.stereo.endpoints(r))))
    return truth,last_pad,rows


def schedule(up,front):
    truth,last_pad,rows=frames(up,front)
    if truth is None or last_pad is None:
        return {},set()
    possible=set()
    budgets={}
    for px in (.1,.25):
        marks=[(t,vision.plane_gap(m,front,px)) for t,m in rows]
        phases=[next(((t,g) for t,g in marks if round(t*1000)%17==i and g>=1),None) for i in range(17)]
        options={}
        for pause in (0,50):
            bracketed=0
            arrivals=[]
            records=[]
            for phase,observation in enumerate(phases):
                if observation is None:
                    continue
                t,gap=observation
                # Exposure clock ±1 ms, 8 ms processing, scheduler alignment
                # 0–20 ms and physical actuation onset 0–10 ms (study allowance).
                earliest=round((t-truth)*1000)+7+pause
                latest=round((t-truth)*1000)+39+pause
                arrivals.extend([earliest,latest])
                possible.update(range(earliest,latest+1))
                # Earliest physical release can follow the last pad by as little
                # as its 2 ms acquisition and 1 ms sync uncertainty permit.
                upper_from_contact=1000*((t+(.039+pause/1000))-(last_pad-.003))
                passes=upper_from_contact<=150+1e-6
                if passes:
                    bracketed+=1
                records.append({"source_phase_mod17":phase,"whole_front_lower_gap_mm":gap,
                                "capture_from_lip_ms":t*1000,"prior_pad_from_lip_ms":last_pad*1000,
                                "exposure_after_shelf_ms":round((t-truth)*1000,3),
                                "onset_after_shelf_ms":[earliest,latest],"upper_onset_since_prior_contact_ms":round(upper_from_contact,3),
                                "within_150ms":passes})
            options[str(pause)]={"observed_phases":sum(t is not None for t in phases),"total_phases":17,
                                "phases_within_150ms_prior_contact_bracket":bracketed,
                                "onset_after_shelf_truth_ms":[min(arrivals),max(arrivals)] if arrivals else None,
                                "exposures":records}
        budgets[str(px)]=options
    return budgets,possible


def run(root,report,poses):
    producer=load_source("scripts.short_passive_trajectory",root/"scripts/station_l1_mission.py")
    law=load_source("short_passive_force_law",root/"design/receiver-drives/size_drives.py")
    motion=heavy.revised_motion()
    data=json.loads(report.read_text())
    front=data["departures"][0]["independent_front_plane_stereo"]["front_bound_mm"]
    robot=SuppliedRobot(poses/"part-frames.json",{k:poses/(k+".step") for k in POSES},hashes=data["pose_step_sha256"],
                        radial=json.loads((poses/"rotation-bounds.json").read_text()),outer_error=15,grip_error=1.5)
    runs=[]
    for sector,key in ((-.18,"coral-A"),(.22,"coral-B")):
        for mu in (.1,.2):
            up=producer.run(target_y=sector,trajectory=motion.RegisteredMission(sector),piece_id=key,
                            dt=.001,sample_every_step=True,grip_law=law.Cartridge(shelf_mu=mu))
            budgets,delays=schedule(up,front)
            window=budgets["0.1"]["0"]["onset_after_shelf_truth_ms"]
            midpoint=sum(window)/2000
            spread=(window[1]-window[0])/2000
            swept=recorded_departure(robot,(sector,key,up),midpoint,spread)
            whole={"modeled_onset_interval_after_shelf_ms":window,
                   "last_possible_contact_after_shelf_ms":round(1000*(swept["last_possible_contact_s"]-swept["first_shelf_witness_s"]),3),
                   "min_tail_separation_mm":swept["min_tail_gap_mm"],
                   "minimum_neighbor_sector_mm":swept["checks_next"][1]["min_swept_bound_gap_mm"] if key=="coral-A" else None}
            observations=[]
            for ms in sorted(delays|{500,600}):
                post=settle.continue_contact(sector=sector,upstream=up,piece_id=key,withdrawal="after-separation",
                                              wait_after_true_separation_s=ms/1000,dt=.0002)
                observations.append((ms,post["stop"],post["min_shelf_clearance_mm"]))
            tested=[r for r in observations if r[0] in delays]
            supported=[r for r in tested if r[1]=="sustained-paired-floor-and-lip"]
            runs.append({"piece":key,"sector_m":sector,"shelf_coast_mu":mu,"budgets":budgets,
                         "all_tight_zero_wait_onsets_combined_geometry":whole,
                         "tested_onsets_after_truth_ms":sorted(delays),"tested_onset_count":len(tested),
                         "sustained_count":len(supported),"minimum_shelf_gap_mm":min((r[2] for r in supported),default=None),
                         "failures":[list(r) for r in tested if r[1]!="sustained-paired-floor-and-lip"],
                         "late_controls":[list(r) for r in observations if r[0] in (500,600)]})
    return {"schema":"whole-front-vision-actuation/1","pose_receipt_sha256":digest(report),
            "grip_law_sha256":digest(root/"design/receiver-drives/size_drives.py"),
            "clock_uncertainty_ms":1,"prior_pad_bound_ms":3,"processing_transport_ms":8,"scheduler_bound_ms":[0,20],
            "assumed_actuator_deadtime_ms":[0,10],"integration_step_ms":.2,"runs":runs}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--contact-root",type=Path,required=True)
    p.add_argument("--pose-receipt",type=Path,required=True)
    p.add_argument("--poses",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    a.output.write_text(json.dumps(run(a.contact_root,a.pose_receipt,a.poses),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
