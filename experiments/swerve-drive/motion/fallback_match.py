"""Drive a preselected empty R4 through AUTO, disable, TELEOP and endgame assessment."""
import argparse
from collections import deque
import json
import math
from pathlib import Path

import numpy as np

from opening import (
    HERE, ROOT, EncoderMap, Odometry, ORDER, Plant, Support, field_to_body, n,
    read, screen, sha, supply_review, velocity, wrapped,
)

DT = .005
END_S = 156
# Continue away from midfield on the red side before returning to the selected park pose.
AUTO_X_M = 10.60


def run(sign, output):
    module = read(HERE.parent / "packaging/module.json")
    centers = np.array([[module["modules"][corner][axis] for axis in ("xM", "yM")] for corner in ORDER])
    start = np.array(read(ROOT / "leave-park/near-tape.json")["cases"]["near-tape"]["start"])
    parked = np.array(read(HERE / "park-endpoint.json")["cases"]["offset-for-cage-swing"]["poseXYMetresYawRadians"])
    auto = np.array([AUTO_X_M, start[1], start[2]])
    actual_start = start + sign * np.array([.002,-.002,math.radians(.12)])
    coupling = np.array([1.,-1.,1.,-1.])
    enc = EncoderMap(.0762, 5.5, np.array([0.,1.5,-2.2,2.8]),np.array([1,-1,-1,1]),np.array([1,-1,1,-1]),coupling)
    mass = 65 if sign == 1 else 72
    shares = np.array([.26,.24,.26,.24])
    support = Support(centers,mass,shares,.38,150000*np.array([.9,.9,1.1,1.1]))
    plant = Plant(centers, actual_start, mass=mass, inertia=5. if sign == 1 else 8.,
                  radii=.0381*(1+sign*np.array([.001,-.001,.0005,-.0005])),
                  loads=mass*9.80665*shares, coupling=coupling,encoder_map=enc,support=support)
    gyro = start[2]
    odom = Odometry(plant,start,enc,gyro)
    delayed_gyro = deque()
    traces = [np.r_[0., plant.pose, plant.velocity, odom.pose]]
    events = [[0.,"south lane reserved, receiver stowed"]]
    held = set()
    battery = {}
    previous = odom.rotor.copy()
    previous_angles = odom.angles.copy()
    disable_max_v = 0.
    # Review one second of electrical samples at a time, preserving the full 1 ms sampling rate.
    for tick in range(int(END_S/DT)):
        wall = tick * DT
        if tick % 2 == 0:
            delayed_gyro.append((tick+2, start[2] + (plant.pose[2]-actual_start[2])*(1+sign*.0005) + math.radians(sign*.006*wall)))
        while delayed_gyro and delayed_gyro[0][0] <= tick:
            _, gyro = delayed_gyro.popleft()
        rotor,absolute = plant.sensors()
        odom.update(rotor,absolute,gyro)
        speed = enc.delta_distance(rotor-previous,wrapped(odom.angles-previous_angles))/DT
        previous, previous_angles = rotor.copy(), odom.angles.copy()
        phase = "auto" if wall < 15 else "teleop"
        disabled = 15 <= wall < 18 or wall >= 153
        target = auto if phase == "auto" else parked
        error = target[:2] - odom.pose[:2]
        if not disabled and phase not in held and np.linalg.norm(error) < .006 and max(abs(speed)) < .012:
            events.append([wall,phase+" stopped on encoder and gyro estimate"])
            held.add(phase)
        stopped = disabled or phase in held
        cmd = np.zeros(3)
        if not stopped:
            cmd[:2] = field_to_body(velocity(error,.40,1.6),odom.pose[2])
            cmd[2] = np.clip(2*wrapped(target[2]-odom.pose[2]),-.35,.35)
        r = plant.advance(cmd,odom.angles,stopped=stopped,disabled=disabled)
        if disabled:
            disable_max_v=max(disable_max_v,float(max(np.max(abs(r[k])) for k in ("drive_voltage_v","steer_voltage_v"))))
        traces.append(np.r_[(tick+1)*DT,plant.pose,plant.velocity,odom.pose])
        if (tick+1) % 200 == 0:
            window=supply_review(plant.power_samples)
            plant.power_samples.clear()
            for pack, row in window.items():
                if not row["supplied"]:
                    raise ValueError(f"{pack} cannot provide requested voltage at {wall}")
                if pack not in battery:
                    battery[pack]=row.copy()
                else:
                    for k in ("minBusV","minControllerV","minMotorHeadroomV"):
                        battery[pack][k]=min(battery[pack][k],row[k])
    a=np.array(traces)
    np.save(output.with_suffix(".npy"),a,allow_pickle=False)
    f,_,team,_=n.route.route.field.scenario(read(ROOT/"fixed-height-match/placements.json")["placements"],inboard=True)
    traffic=n.route.route.field.traffic
    usual_south=traffic.Program(1,"red")
    usual_gap=traffic.separation(a[0,1:4],usual_south.initial)
    team["red1"]=traffic.Program(2,"red")
    gaps=screen(f,a[:,0],a[:,1:4],read(ROOT/"full-match/installed-stock.json")["stock"],team,END_S)
    half=.42505
    tape_east=(690.875-144-65.5-88)*.0254
    at15=a[round(15/DT)]
    auto_clear=(at15[1]-half*(abs(math.sin(at15[3]))+abs(math.cos(at15[3])))-tape_east)*1000
    assessment=a[a[:,0]>=153]
    terminal_delta=np.max(np.abs(assessment[:,1:4]-parked),axis=0)
    record=dict(schema="reefscape-swerve-fallback/1",case=sign,massKg=mass,events=events,
                initialXYMetresYawRadians=a[0,1:4].tolist(),poseAtAutoEnd=at15[1:4].tolist(),
                autoEndCoverClearOfEastTapeMm=auto_clear,headingTrackingScope="Independent electronic gyro with 10 ms transport and signed bias",
                disabledMaxAbsLeadVoltageV=disable_max_v,unassignedSouthStartPeerMarginMm=usual_gap*1000,
                peakHoldSpeedMps=float(np.linalg.norm(assessment[:,4:6],axis=1).max()),
                peakHoldYawRateRadS=float(np.max(abs(assessment[:,6]))),assessmentDeviationMmMmDeg=[terminal_delta[0]*1000,terminal_delta[1]*1000,math.degrees(terminal_delta[2])],
                battery=battery,energy=plant.balance,clearance=gaps,stepCount=len(a)-1,durationS=END_S,
                trajectorySha256=sha(output.with_suffix(".npy")),sourceHashes={
                    str(p.relative_to(HERE.parents[2])).replace(chr(92),"/"):sha(p) for p in (
                      HERE/"park-endpoint.json",HERE/"plant.py",HERE/"model.json",HERE/"kinematics.py",HERE/"actual_supply.py",
                      HERE.parent/"power/electrical.py",ROOT/"leave-park/near-tape.json",ROOT/"full-match/installed-stock.json")})
    output.write_text(json.dumps(record,indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
    print(sign,events,record["assessmentDeviationMmMmDeg"],auto_clear,gaps["minimumPeerMm"])


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sign",choices=[-1,1],type=int)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    args.output.parent.mkdir(exist_ok=True,parents=True)
    run(args.sign,args.output)
