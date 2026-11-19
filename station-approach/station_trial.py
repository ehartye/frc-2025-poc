"""Station-panel visibility and a separate, preselected, encoder-observed short AUTO."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

FOCAL_PX = 320/math.tan(math.radians(36.5))
PITCH = math.radians(26)
TRACK_M = 0.580
RADIUS_M = 0.0762
REDUCTION = 20
DT = 0.005
ACCEL = 0.45
MAX_SPEED = 0.35
MAX_AGE = 0.035
GOAL = 0.542925


def read(name):
    return json.loads((ROOT / name).read_bytes())


def project(points, station_pose, eye_y):
    """Station X is into the field. Robot forward at the dock is toward -X."""
    x, y, yaw = station_pose
    dx, dy = points[:, 0] - x, points[:, 1] - y
    forward = -(dx * math.cos(yaw) + dy * math.sin(yaw)) + 315
    left = dx * math.sin(yaw) - dy * math.cos(yaw) - eye_y
    above = points[:, 2] - 970
    depth = forward * math.cos(PITCH) - above * math.sin(PITCH)
    up = forward * math.sin(PITCH) + above * math.cos(PITCH)
    guarded = np.where(depth > 0.001, depth, np.nan)
    return np.column_stack((320 - FOCAL_PX * left / guarded, 240 - FOCAL_PX * up / guarded))


def visible(uv):
    return (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480)


def station(raw):
    sys.path.insert(0, str(ROOT / "selected-drive"))
    import route
    evidence = read("station-localization/sightlines.json")
    points = np.array([r["centerStationMm"] for r in evidence["boltFaces"]])
    # Include all corners of the physical ID2 panel. Its printed code is inside it.
    box = evidence["panelCadBounds"]
    panel = np.array([[x,y,z] for x in (box[0],box[3]) for y in (box[1],box[4])
                      for z in (box[2],box[5])])
    t, poses, arcs, _, _, f, legs = route.samples(raw, "nominal")
    end = np.searchsorted(t, legs[0][2].duration)
    data = {}
    for key, scales, track in (("nominal",(1,1),TRACK_M),("radius",(.998,1.002),TRACK_M),
                               ("track",(1,1),.582)):
        p = route.integrate(poses[0], arcs[:end], scales, track)
        contact = read("selected-drive/dock.json")["cases"][{"radius":"bias"}.get(key,key)]
        hit = contact["firstSampleWithPositiveVolume"]
        cutoff = end if hit is None else np.searchsorted(t, hit["wallS"])-1
        seen, terminal = [], []
        for i in range(cutoff+1):
            dx,dy = f.rot((p[i,0]-f.DOCK[0],p[i,1]-f.DOCK[1]), -f.NORMAL)
            pose = (444.525+1000*dx,1000*dy, f.short(p[i,2]-poses[end,2]))
            images = [project(points,pose,e) for e in (-37.5,37.5)]
            if all(visible(uv).all() for uv in images):
                seen.append(float(t[i]))
            if cutoff-i < 100:
                corners = project(panel,pose,0)
                terminal.append(float(np.nanmax(corners[:,1])))
        data[key] = {"lastUnimpededSampleS":float(t[cutoff]), "allBoltsStereoFrameCount":len(seen),
                     "lastAllBoltsInViewS":seen[-1] if seen else None,
                     "lastHalfSecondLargestPanelRowPx":max(terminal),
                     "firstNativeOverlapS":hit["wallS"] if hit else None}
    # The long top edge of GE-25007 can yield an identical line at different
    # transverse positions. The corner coordinates come from the native wall.
    toe = np.array([[9.525,-1012.825,304.8],[9.525,1012.825,304.8]])
    data["lowerEdgeOnly"] = {}
    for side in (-20,20):
        uv = project(toe,(444.525,side,0),0)
        data["lowerEdgeOnly"][str(side)] = {"pixelX":uv[:,0].tolist(),"pixelY":uv[:,1].tolist()}
    return data


def encoder_rotations(wheel_path):
    return wheel_path * REDUCTION / (2*math.pi*RADIUS_M)


def rollout(scales=(1.,1.), track=TRACK_M, fault=None, fault_at=.75, delay_ms=15):
    """Feedback receives only motor-shaft rotations and acquisition stamps. Truth is scored afterwards."""
    shafts = np.zeros(2)
    history = {}
    x = y = yaw = speed = 0.
    target_speed = 0.
    latched = False
    completed = False
    fault_x = None
    now = 0.
    for tick in range(3001):
        now = tick*DT
        if tick%2 == 0:
            history[tick] = tuple(shafts)
        capture = (tick - math.ceil(delay_ms/5))//2*2
        if fault in ("late","drop") and capture*DT >= fault_at:
            capture = (math.ceil(fault_at/(2*DT))-1)*2 if fault=="drop" else (tick-14)//2*2
        if tick%2 == 0:
            if capture < 0:
                target_speed = 0.
                if now >= MAX_AGE:
                    latched = True
            elif now-capture*DT > MAX_AGE+1e-9:
                latched = True
            else:
                counts = np.asarray(history[capture]).copy()
                if fault=="disagree" and capture*DT >= fault_at:
                    counts[0] += encoder_rotations(.04)
                wheel = counts / encoder_rotations(1.)
                if abs(wheel[0]-wheel[1])>.006:
                    latched = True
                remaining = GOAL-float(wheel.mean())
                if remaining <= .001:
                    completed = True
                # Reserve travel during measurement age and a full control cycle.
                reserve = MAX_SPEED * (MAX_AGE+.010)
                target_speed = min(MAX_SPEED, math.sqrt(2*ACCEL*max(0,remaining-reserve)))
                if remaining > .001:
                    target_speed = max(.025,target_speed)
            if latched or completed:
                target_speed = 0.
        if latched and fault_x is None:
            fault_x = x
        prior = speed
        speed += np.clip(target_speed-speed,-ACCEL*DT,ACCEL*DT)
        ds = (speed+prior)*DT/2
        shafts += encoder_rotations(ds)
        dl,dr = ds*scales[0],ds*scales[1]
        dh = (dr-dl)/track
        middle = yaw + dh/2
        # Stable midpoint chord; sinc is mathematically exact for the constant-curvature step.
        chord = (dl+dr)/2*np.sinc(dh/(2*np.pi))
        x += chord*math.cos(middle)
        y += chord*math.sin(middle)
        yaw += dh
        if (latched or completed) and speed==0:
            break
    return {"latchedFault":latched,"goalSensorConfirmed":completed and not latched,
            "measuredWheelTravelM":float(shafts.mean()/encoder_rotations(1.)),
            "settledS":now+DT,"forwardM":float(x),"lateralM":float(y),"yawDeg":math.degrees(yaw),
            "postInhibitTravelM":float(x-fault_x) if fault_x is not None else None}


def short():
    cases = {}
    for left,right,track in itertools.product((.998,1.002),(.998,1.002),(.578,.582)):
        for delay in (5,15,25):
            cases[f"{left}/{right}/{track}/{delay}"] = rollout((left,right),track,delay_ms=delay)
    faults = {f"{kind}-{t}":rollout((.998,1.002),.578,kind,t) for kind,t in
              itertools.product(("late","drop","disagree"),(.10,.75,1.5,2.25))}
    faults["noDataFromEnable"] = rollout(fault="drop",fault_at=0.)
    native = read("leave-park/near-tape.json")["cases"]["near-tape"]
    cage = read("leave-park/cage-motion.json")
    # Integrate the worst signed wheel scale mismatch over the actual arc.
    # Includes staging translation and yaw, the smallest effective track and
    # all interior wheel-scale combinations, including those not sampled.
    endpoint = max(abs(q["measuredWheelTravelM"]-GOAL) for q in cases.values())
    distance = GOAL + endpoint
    scale_error = .002
    initial_yaw = math.radians(.4)
    yaw_growth = 2*scale_error*distance/.578
    center_error = .005 + distance*scale_error + distance*(1+scale_error)*(initial_yaw+yaw_growth/2)
    hull_radius = math.hypot(.425,.425)
    error_mm = 1000*(endpoint+center_error+2*hull_radius*math.sin((initial_yaw+yaw_growth)/2))
    sway = cage["upper_height_mm"]*math.tan(math.radians(3))
    locked = cage["high_locked_robot"]
    return {"nominal":rollout(),"wheelTrackDelayCases":cases,"badPackets":faults,"sweptHullErrorMm":error_mm,
            "tapeLeaveMarginMm":native["tape_clearance_mm"]-error_mm,
            "cageMarginAt3DegMm":cage["cases"]["near-tape"]["upright_gap_mm"]-error_mm-sway,
            "staticHardwareMarginMm":native["static_field_lower_bound_mm"]-error_mm,
            "planningExclusionMarginMm":1000*native["planning_field"]["gap_m"]-error_mm,
            "stockMarginMm":min(r["lower_bound_mm"] for r in locked["stock_bounds"].values())-error_mm,
            "autoPeerMarginMm":1000*min(native["auto_focal_to_each_peer_m"].values())-error_mm,
            "parkEndpointPeerMarginMm":1000*native["teleop_pair_bound_m"]-error_mm,
            "maximumPostInhibitTravelMm":max(q["postInhibitTravelM"] for q in faults.values())*1000,
            "maxPossibleTravelAfterFreshnessLossMm":1000*(MAX_SPEED*(MAX_AGE+.010)+MAX_SPEED**2/(2*ACCEL))*1.002,
            "terminalXErrorMm":endpoint*1000,"stockCount":locked["stock_count"]}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("approach_traces",type=Path,help="Directory containing the issued relieved-match CSVs")
    args=parser.parse_args()
    result={"station":station(args.approach_traces),"short":short()}
    sources=["sector-alignment/vision.py","selected-drive/route.py","station-localization/sightlines.json","selected-drive/dock.json","selected-drive/results.json",
             "leave-park/cage-motion.json","leave-park/near-tape.json","leave-park/near-tape-review.json"]
    result["sources"]={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
    (HERE/"results.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
