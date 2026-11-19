"""Intersect the attained rack trace with each propagated pipe, through three seconds of assessment."""
import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path
import sys
from unittest.mock import patch
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "low-observation"))
from source import inputs, handoff


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def settle_trace(source):
    points=[]
    def sample(x,z,front,top):
        points.append((x,z,top))
        return 1.0
    # The issued solver's shelf only terminates on recontact. Its floor/lip forces and
    # axial state therefore apply exactly up to the first calculated shelf intersection.
    with patch.object(handoff.spin.settle,"shelf_gap",sample):
        out=handoff.spin.settle.continue_contact(**source["settle_kwargs"],duration=3.)
    if out["stop"]!="sustained-paired-floor-and-lip":
        raise ValueError("Target-only reference fell outside supported state")
    pos=np.array(points)
    hit=source["truth"]["paired-trough-first-contact"]
    times=hit+np.arange(len(pos))*1000*out["dt_s"]
    return times,pos,out["ending_still_double_contact_s"]


def signed_gaps(pos,travel):
    x,z,top=pos.T
    front=.045-travel  # 535 mm from chassis center, target X origin 490 mm forward.
    dx=np.maximum.reduce([front-.265-x,np.zeros_like(x),x-front])
    dz=np.maximum.reduce([top-.015-z,np.zeros_like(z),z-top])
    rect=np.hypot(dx,dz)
    radius=handoff.spin.settle.mission.R
    return np.where((x>=front-.006)&(z>=top-.006),
                    np.hypot(x-(front-.006),z-(top-.006))-.006-radius,rect-radius)


def run(jdk,observer,folder):
    hashes=json.loads((HERE/"motion-traces.json").read_bytes())
    for filename,digest in hashes.items():
        if sha(folder/filename)!=digest:
            raise ValueError("HAL motion bytes changed: "+filename)
    raw=list(csv.DictReader((folder/"trials.csv").open()))
    fixed=json.loads((ROOT/"low-observation/immediate.json").read_bytes())["native_front_planes_by_stroke_factor"]
    kit=json.loads((observer/"mount.json").read_bytes())
    if kit["combined_count"]!=598:
        raise ValueError("Incorrect robot")
    new_front=max(part["bounds_mm"][3] for part in kit["parts"].values())
    rates={float(k):v["unextended_front_mm"] for k,v in fixed.items()}
    rates[1.]=max(rates[1.],new_front)
    half=57.65*math.cos(math.radians(1))+151.0625*math.sin(math.radians(1))
    result={"motionSourceSha256":sha(folder/"trials.csv"),"poseBodies":598,"addedSlideFrontMm":new_front,
            "sourceSha256":{},"cases":len(raw),"policies":{},"pieces":{}}
    for path in ["design/receiver-drives/sizing.json","fixed-low-power/retreat.json","low-observation/results.json",
                 "low-observation/immediate.csv","fixed-height-match/placements.json","slide-feedback/Retreat.java",
                 "slide-feedback/SlideFeedbackTest.java","slide-feedback/motion-traces.json","design/electrical-inspection/circuits.json"]:
        result["sourceSha256"][path]=sha(ROOT/path)
    for name in ("handoff.json","mount.json","mass.json","moving-bridge.step","clearance.json"):
        result["sourceSha256"]["moving-observer/"+name]=sha(observer/name)
    for policy in {x["policy"] for x in raw}:
        rows=[x for x in raw if x["policy"]==policy and x["drag_N"]=="25" and x["fault"] in ("normal","biasLow","biasHigh")]
        result["policies"][policy]={p:{"trials":len(v),"verifiedStow":sum(x["state"]=="STOWED" for x in v),
            "fullyPassingPhases":[i for i in range(17) if all(x["state"]=="STOWED" for x in v if int(x["phase"])==i)]}
            for p in ("coral-A","coral-B") for v in [[x for x in rows if x["piece"]==p]]}
    phase_rows=[]
    for name,source in inputs(jdk).items():
        t,pos,dwell=settle_trace(source)
        candidate=[x for x in raw if x["piece"]==name and x["policy"]=="extended5"]
        witness=json.loads((ROOT/"low-observation/results.json").read_bytes())["pieces"][name]
        prior=math.floor(witness["ideal_prior_pad_ms"]-3)
        contact={}
        filedata={}
        for row in candidate:
            kind=row["trace_id"]
            start=int(row["command_ms"])
            if kind=="none":continue
            if kind not in filedata:
                filedata[kind]=np.loadtxt(folder/(kind+".csv"),delimiter=",",skiprows=1,usecols=1)
            key=f"{kind}@{start}"
            if key in contact:continue
            trace=filedata[kind]
            travel=np.interp(t-start,np.arange(len(trace)),trace,left=0,right=trace[-1])
            gaps=signed_gaps(pos,travel)
            # Every commanded start follows first target contact for this registered low profile,
            # apart from any earlier start checked by the native trajectory prefix below.
            metric=490+1000*pos[:,0]-half-np.maximum.reduce([base+rate*(370-1000*travel) for rate,base in rates.items()])-1
            if start<t[0]:
                tr=source["trajectory"]; before=np.r_[start,tr[(tr[:,0]>start)&(tr[:,0]<t[0]),0]]
                travel0=np.interp(before-start,np.arange(len(trace)),trace,left=0,right=trace[-1])
                extra=np.interp(before,tr[:,0],tr[:,1])-half-np.maximum.reduce(
                    [base+rate*(370-1000*travel0) for rate,base in rates.items()])-1
                minimum=min(np.min(metric),np.min(extra))
            else:
                minimum=np.min(metric)
            bad=np.nonzero(gaps<=0)[0]
            contact[key]=(float(1000*gaps.min()),float(minimum),None if not len(bad) else round(float(t[bad[0]]),2))
        accepted=[x for x in candidate if x["state"]=="STOWED" and x["drag_N"]=="25"]
        if not accepted:raise ValueError("No admitted phase")
        allg=[contact[x["trace_id"]+"@"+x["command_ms"]] for x in accepted]
        feedback=[int(x["observed_motion_ms"])-int(x["command_ms"]) for x in accepted]
        scheduled = source["truth"]["finite-pipe-envelope-clear-of-shelf-by-1mm"] + 110 + 370/.15
        for phase in range(17):
            rows=[x for x in accepted if int(x["phase"])==phase]
            gaps=[contact[x["trace_id"]+"@"+x["command_ms"]] for x in rows]
            phase_rows.append([name,phase,len(rows),min(int(x["command_ms"]) for x in rows),
                max(int(x["observed_motion_ms"]) for x in rows),max(int(x["verified_stow_ms"]) for x in rows),
                round(min(g[0] for g in gaps),4),round(min(g[1] for g in gaps),4)])
        fixed_hold=signed_gaps(pos,np.zeros(len(pos)))
        failures = {}
        for fault in sorted(set(x["fault"] for x in candidate)-{"normal","biasHigh","biasLow"}):
            rows=[x for x in candidate if x["fault"]==fault]
            outcomes=[]
            for row in rows:
                if row["trace_id"]=="none":
                    first=t[np.flatnonzero(fixed_hold<0)[0]]
                else:
                    first=contact[row["trace_id"]+"@"+row["command_ms"]][2]
                outcomes.append(first)
            hits=[float(q) for q in outcomes if q is not None]
            failures[fault]={"attempts":len(rows),"physicalShelfRecontacts":len(hits),
                             "earliestRecontactLocalMs":min(hits) if hits else None}
        result["pieces"][name]={"trials":len(accepted),"uniqueAttainedPathsAndStarts":len(set((x["trace_id"],x["command_ms"]) for x in accepted)),
            "minimumShelfGapMm":min(g[0] for g in allg),"minimumWholeFrontMm":min(g[1] for g in allg),
            "allRetainTargetSupport":all(g[2] is None and g[1]>0 for g in allg),"stationaryPairedSupportS":dwell,
            "ackAfterCommandMs":[min(feedback),max(feedback)],"latestAckAfterPriorMs":max(int(x["observed_motion_ms"])-prior for x in accepted),
            "stowAfterCommandMs":sorted(set(int(x["verified_stow_ms"])-int(x["command_ms"]) for x in accepted)),
            "peakPhaseA":max(float(x["peak_phase_A"]) for x in accepted),"peakMotorV":max(float(x["peak_motor_V"]) for x in accepted),
            "peakStopSpeedMps":max(float(x["hard_stop_mps"]) for x in accepted),
            "heldShelfFirstHitMs":float(t[np.flatnonzero(fixed_hold<0)[0]]),
            "minimumTimeBeforeIssuedStowMs":scheduled-max(int(x["verified_stow_ms"]) for x in accepted),
            "faults":{f:dict(Counter(x["state"] for x in candidate if x["fault"]==f)) for f in sorted(set(x["fault"] for x in candidate)-{"normal","biasHigh","biasLow"})},
            "faultContact":failures,"dragControl":{d:dict(Counter(x["state"] for x in candidate if x["drag_N"]==d)) for d in ["15","35","80","140"]}}
    with (HERE/"phases.csv").open("w",newline="",encoding="utf-8") as file:
        writer=csv.writer(file,lineterminator="\n")
        writer.writerow(["piece","exposure_phase_ms","accepted_cases","earliest_command_ms","latest_observed_motion_ms",
                         "latest_verified_stow_ms","minimum_shelf_mm","minimum_whole_robot_front_mm"])
        writer.writerows(phase_rows)
    (HERE/"results.json").write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--java-home",type=Path,required=True)
    p.add_argument("--observer",type=Path,default=ROOT/"moving-observer")
    p.add_argument("--output",type=Path,default=ROOT/"artifacts/slide-feedback")
    a=p.parse_args()
    run(a.java_home.resolve(),a.observer.resolve(),a.output.resolve())
