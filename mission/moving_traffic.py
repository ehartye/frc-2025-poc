"""Couple an occupied six-robot trajectory to finite station stock and a single-file feed."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts import registered_transit as f
from scripts import assess_coral

INPUT=json.loads((ROOT/"mission/registered-field-receipt.json").read_text())
STARTS=f.PROFILE["rehearsal"]["starts"]
HALF=.425
RADIUS=HALF*math.sqrt(2)


class Program:
    def __init__(self,slot,side):
        xx,yy=STARTS[slot-1]
        self.pose=((xx if side=="blue" else f.PROFILE["field"]["length"]-xx)*f.IN,yy*f.IN,
                   0 if side=="blue" else math.pi)
        self.initial=self.pose
        self.clock=0.0
        self.intervals=[]
        self.ends=[]

    def append(self,dt,name,route=None):
        if dt<=0:
            return
        self.intervals.append((self.clock,self.pose,name,route))
        self.clock+=dt
        self.ends.append(self.clock)
        if route is not None:
            self.pose=route.end

    def polyline(self,points,name):
        r=f.Route(self.pose)
        for x,y in points:
            r.go((x*f.IN,y*f.IN))
        self.append(r.duration,name,r)

    def at(self,time):
        i=bisect.bisect_right(self.ends,time)
        if i>=len(self.intervals):
            return self.pose,(0,0),"hold"
        start,p,name,r=self.intervals[i]
        if r is None:
            return p,(0,0),name
        pose,w=r.at(time-start)
        return pose,w,name


def feeder(ready,stock,loader_free,piece,consumed):
    if piece in consumed or stock is None:
        return None
    # A hand-set pipe starts at rest on one transverse rolling lane. Thin-walled
    # ring inertia uses actual nominal bore and outside radius; slip is unmodeled.
    radius=.05715
    bore=.0508
    rotational_ratio=.5*(1+(bore/radius)**2)
    acceleration=9.80665*math.sin(math.radians(35))/(1+rotational_ratio)
    target_lip_speed=.5
    distance=target_lip_speed**2/(2*acceleration)
    release=max(ready,stock,loader_free)+1.1
    arrival=release+target_lip_speed/acceleration
    return {"piece_id":piece,"stock_available_s":stock,"insertion_started_s":release-1.1,
            "chute_roll_start_s":release,"lip_arrival_s":arrival,"travel_m":distance,
            "chute_acceleration_mps2":acceleration,"observed_lip_speed_basis":"assumed no-slip rolling from rest"}


def programs(start_hold=0,stock=(0,35)):
    team={side+str(i):Program(i,side) for side in ("red","blue") for i in (1,2,3)}
    focal=team["red2"]
    focal.append(start_hold,"traffic reservation")
    routes=[f.routes(y) for y in (-.18,.22)]
    focal.append(routes[0][0].duration,"first station approach",routes[0][0])
    arrival=[]
    pipe_states=[]
    consumed={}
    for i,p in enumerate(INPUT["trips"]):
        key=p["piece_id"]
        sector=p["sector_m"]
        focal.append(2,"extend empty receiver")
        feed=feeder(focal.clock,stock[i],arrival[-1]["lip_arrival_s"] if arrival else 0,key,consumed)
        if not feed:
            break
        arrival.append(feed)
        focal.append(feed["lip_arrival_s"]-focal.clock,"single file station feed")
        consumed[key]=True
        source_start=focal.clock
        carry=routes[i][1]
        e=INPUT["events"]
        # This bench stage lasts through the actual registered body capture and
        # lowering. Derive duration from the source receipt's first loaded leg.
        loaded_index=next(j for j,ev in enumerate(e) if ev["event"]=="loaded-"+key)
        d0=e[loaded_index-1]["elapsed_s"]
        focal.append(d0,"seat, retract and lower")
        focal.append(carry.duration,"loaded "+key,carry)
        t=source_start+(p["support_confirmation_s"]-p["feed_s"])
        focal.append(t-focal.clock,"place, withdraw and observe "+key)
        pipe_states.append({"id":key,"support_s":focal.clock,"sector_m":sector})
        end_index=next(j for j,ev in enumerate(e[loaded_index:],loaded_index) if ev["event"]=="empty-return-"+key)
        focal.append(e[end_index-2]["elapsed_s"]+e[end_index-1]["elapsed_s"],"empty mechanism recovery")
        ret=routes[i][2]
        focal.append(ret.duration,"empty return "+key,ret)
        focal.append(2,"raise empty station receiver")
    red=team["red3"]
    red.polyline([(440,245)],"partner takes shared corridor")
    red.append(5,"partner occupies intersection")
    red.polyline([(475,284),(500,287),(515,287)],"partner clears north lane")
    blue=team["blue3"]
    blue.append(8,"opponent stages before cage gap")
    blue.polyline([(295,264.375),(408,264.375),(417,278),(400,291)],"opponent crosses cage gap")
    return team,arrival,pipe_states


def separation(p,q,dimensions_a=(HALF,HALF),dimensions_b=(HALF,HALF)):
    dx,dy=f.sub(q,p)
    a,b=p[2],q[2]
    # Four unit separating axes; positive gap is a conservative Euclidean clearance.
    ca,sa,cb,sb=math.cos(a),math.sin(a),math.cos(b),math.sin(b)
    c,s=abs(ca*cb+sa*sb),abs(ca*sb-sa*cb)
    ax,ay=dimensions_a
    bx,by=dimensions_b
    return max(abs(dx*ca+dy*sa)-ax-bx*c-by*s,
               abs(-dx*sa+dy*ca)-ay-bx*s-by*c,
               abs(dx*cb+dy*sb)-bx-ax*c-ay*s,
               abs(-dx*sb+dy*cb)-by-ax*s-ay*c)


def motion_bound(w):
    return abs((w[0]+w[1])/2)+RADIUS*abs((w[1]-w[0])/f.DRIVE["trackM"])


def analyze(start_hold=0,stock=(0,35),deadline=135,step=.02):
    if (not math.isfinite(start_hold) or start_hold<0 or not math.isfinite(deadline) or not 0<deadline<=135
            or not math.isfinite(step) or not .005<=step<=.05 or len(stock)!=2
            or any(t is not None and (not math.isfinite(t) or t<0) for t in stock)):
        raise ValueError("Use finite nonnegative times, two stock arrivals and a 5–50 ms collision step")
    team,feeds,pipes=programs(start_hold,stock)
    for key in ("red3","blue3"):
        for _,_,_,route in team[key].intervals:
            if route:
                screen=min(f.audit(route,[-425,-425,0,425,425,1020]),key=lambda s:s["gap_m"])
                if screen["gap_m"]<=0:
                    raise ValueError(f"{key} hits field {screen}")
    keys=list(team)
    half=step/2
    min_gap=math.inf
    worst=None
    first=None
    direct=None
    prior_pipes=[(f.LIP[0]+p["departure"]["settled_target_xz_m"][0],f.LIP[1]+p["sector_m"],0)
                 for p in INPUT["trips"]]
    min_other_pipe=math.inf
    for tick in range(math.ceil(deadline/step)):
        t=tick*step+half
        states=[team[k].at(t) for k in keys]
        def extent(j):
            phase=states[j][2]
            if keys[j]=="red2" and (phase.startswith("place, withdraw") or phase=="empty mechanism recovery"):
                return (.535,.425)
            return (HALF,HALF)
        for i,(p,w1,_) in enumerate(states):
            if keys[i]!="red2":
                for pipe in prior_pipes:
                    gap=separation(p,pipe,dimensions_b=(.05725,.1509125))
                    min_other_pipe=min(min_other_pipe,gap-half*motion_bound(w1)-2*half**2)
            for j in range(i):
                q,w2,_=states[j]
                sample=separation(p,q,extent(i),extent(j))
                # Speed at any interior point may exceed midpoint speed. Wheel
                # acceleration bounds also cover junctions at rest-to-rest segment ends.
                acc=f.DRIVE["maximumWheelAccelMps2"]*(1+2*RADIUS/f.DRIVE["trackM"])
                bound=half*(motion_bound(w1)+motion_bound(w2))+acc*half**2
                gap=sample-bound
                if gap<min_gap:
                    min_gap=gap
                    worst=[keys[i],keys[j],round(t,3)]
                if gap<=0 and first is None:
                    first=[keys[i],keys[j],round(t,3)]
                if sample<0 and direct is None:
                    direct=[keys[i],keys[j],round(t,3),round(sample,5)]
    physical=not first
    observed=[p for p in pipes if physical and p["support_s"]<=deadline]
    exclusions=json.loads((ROOT/"mission/branch-exclusion.json").read_text())
    if exclusions["scoring_pipe_count"]!=12 or exclusions["native_material_control_gap_mm"]>1e-5:
        raise ValueError("Incomplete native branch witness")
    measured={}
    for trip,witness in zip(INPUT["trips"],exclusions["trips"],strict=True):
        x,z=trip["departure"]["settled_target_xz_m"]
        if (trip["piece_id"]!=witness["piece"] or math.dist([x,trip["sector_m"],z],witness["target_x_y_z_m"])>1e-8
                or len(witness["pipe_distances_mm"])!=12):
            raise ValueError("Branch witness differs from observed registered pipe")
        measured[witness["piece"]]=min(witness["pipe_distances_mm"].values())
    scores={}
    samples=[]
    for p in observed:
        ms=round(p["support_s"]*1000)
        samples.append({"timeMs":ms,"inventoryComplete":False,"robotCommandActive":True,
                        "pieces":[{"id":p["id"],"observedMs":ms,"touchesOwnTrough":True,
                                   "touchesAlliedRobot":False if min_other_pipe>0 else None,
                                   "touchesOpponentRobot":False if min_other_pipe>0 else None,
                                   "scoredOnAnotherReefLevel":False if measured[p["id"]]>0 else None}]})
    if samples:
        for edition in assess_coral.EDITIONS:
            scores[edition]=assess_coral.assess({"schema":"reefscape-coral-observations/1","phase":"TELEOP",
                "basis":"Registered candidate, continuous moving-roster occupancy, complete red native scoring-pipe exclusion.",
                "samples":samples},edition)
    return {"hold_before_start_s":start_hold,"stock_ready_s":list(stock),"active_clock_s":deadline,
            "separated_for_full_clock":physical,"min_separating_bound_m":min_gap,"worst":worst,
            "first_unresolved":first,"first_sampled_overlap":direct,"feeds":feeds,
            "source_arrivals_before_clock":[s["piece_id"] for s in feeds if physical and s["lip_arrival_s"]<=deadline],
            "support_observations":observed,"planned_empty_finish_s":team["red2"].clock if len(pipes)==2 else None,
            "other_robot_to_target_min_bound_m":min_other_pipe,"edition_assessments":scores,
            "empty_finish_available":physical and len(pipes)==2 and team["red2"].clock<=deadline}


SCENARIOS={
    "immediate":("Immediate departure",0,(0,35)),
    "opponent":("Ten-second hold",10,(0,35)),
    "reserved":("Reserved corridor",16,(0,35)),
    "late":("Late station stock",16,(0,111)),
    "depleted":("One available pipe",16,(0,None)),
}


def matrix():
    inputs=["mission/registered-field-receipt.json","mission/branch-exclusion.json",
            "mission/field-registration.json","kickoff/rule-register-v0.json"]
    return {"schema":"registered-moving-traffic/1",
            "input_sha256_lf_text":{p:hashlib.sha256((ROOT/p).read_bytes().replace(b"\r\n",b"\n")).hexdigest() for p in inputs},
            "clock_s":135,"step_s":.02,"robot_pairs":15,
            "scenarios":{k:analyze(hold,stock) for k,(_,hold,stock) in SCENARIOS.items()}}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--hold",type=float,default=0)
    p.add_argument("--stock",type=float,nargs=2,default=[0,35])
    p.add_argument("--matrix",type=Path,help="Write the five named project trials as JSON")
    a=p.parse_args()
    if a.matrix:
        a.matrix.write_text(json.dumps(matrix(),indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
    else:
        print(json.dumps(analyze(a.hold,a.stock),indent=2,allow_nan=False))
