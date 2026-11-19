"""Run named source/receiver/target alternatives and audit the modeler's instantaneous spin reset."""
import argparse
import json
import math
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

from pipe_spin_route import HERE,ROOT,DATA,SOURCE,Contact,trial,digest
from pipe_axial_contact import I,K,M,R

CANDIDATE=Contact(drive="velocity-command",release_limit=.90)
CASES={
    "force-following-limited":(Contact(),.0005),
    "force-following-instant-command":(Contact(belt_accel=1e6),.0005),
    "velocity-command-high":(replace(CANDIDATE,release_limit=1.05),.0005),
    "velocity-command-low":(replace(CANDIDATE,release_limit=.75),.0005),
    "nominal":(CANDIDATE,.0005),
    "low-ramp-friction":(replace(CANDIDATE,ramp_mu=.15),.0005),
    "low-belt-preload":(replace(CANDIDATE,belt_mu=.08,preload=2),.0005),
    "lower-shelf-friction":(replace(CANDIDATE,shelf_mu=.02),.0005),
    "higher-shelf-friction":(replace(CANDIDATE,shelf_mu=.25),.0005),
    "coarse-time":(CANDIDATE,.001),
    "fine-time":(CANDIDATE,.00025),
    "fine-belt-height":(replace(CANDIDATE,slices=11),.0005),
    "faster-slip-saturation":(replace(CANDIDATE,slip_scale=.01),.0005),
    "slower-slip-saturation":(replace(CANDIDATE,slip_scale=.04),.0005),
}


def one(job):
    key,entry=job
    cfg,step=CASES[key]
    row,_,path=trial(cfg,entry,step,through_end=(key=="nominal"))
    release=row.pop("release_trace",[])
    if key=="nominal" and entry["id"]=="coral-A":
        row["sampled_receiver_spin"]=[[t,omega] for t,u,z,omega in path[::max(1,round(.002/step))]]
        row["sampled_release_spin"]=[[t-row["phase_entry_spin_rad_s"]["place"][0],omega] for t,omega,v in release]
    return key,entry["id"],row


def main(workers):
    jobs=[(name,entry) for name in CASES for entry in DATA["identified_transfers"]]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows=list(pool.map(one,jobs))
    cases={k:{} for k in CASES}
    for k,identity,row in rows:cases[k][identity]=row
    first=cases["nominal"]["coral-A"]["pickup"]
    roll=first["ramp"]["spin"]
    angular=I*(0-roll)
    return {"schema":"source-axial-custody/1","pickup_input_sha256":digest(SOURCE),"polar_inertia_kg_m2":I,
            "tube_polar_ratio":K,"robot_pose_sha256":json.loads((ROOT/"sourced-match/shop-545.json").read_bytes())["pose_sha256"],
            "model_sources":{p.relative_to(ROOT).as_posix():digest(p) for p in (HERE/"pipe_axial_contact.py",HERE/"pipe_spin_route.py",
                         ROOT/"scripts/station_l1_mission.py",ROOT/"scripts/station_l1_settle.py")},
            "unforced_reset":{"before_rad_s":roll,"after_rad_s":0,"declared_impulse_Nms":0,"residual_Nms":angular,
                "unexplained_kinetic_energy_J":.5*I*roll*roll,"scope":"Instantaneous removal at chute exit. Target spin is evaluated after finite, recorded contact impulses."},
            "upstream_rebounds":DATA["feed"]["first_rebound_alternatives"],"cases":cases,
            "limits":["Prescribed velocity feedback, slip regularization, pressure distribution and reaction loads are unmeasured.",
                      "Centered planar axial motion; gyroscopic pitch reactions at chassis turns, yaw/length variation, tire slip, hand release or sensor/hardware validation remain outside the model.",
                      "No official points or hardware authorization. Alternative acquisition/withdrawal trajectories require their own vision, geometry, timing and drive review."]}


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--workers",type=int,default=2)
    args=parser.parse_args()
    (HERE/"study.json").write_text(json.dumps(main(args.workers),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
