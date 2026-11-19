"""Propagate the attained hollow pipe in the accelerating and rotating physical receiver frame."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

from navigation import HERE, ROOT, corrected, placed_route, read, sha
from cargo import interpolate

sys.path.insert(0, str(ROOT / "return-station"))
from feed import Pipe, I, IP, HALF, R, EDGE, UP
from replay import Trace


def momentum(axis, omega):
    return IP * omega + (I - IP) * axis * (axis @ omega)


def fictitious(pipe, linear, omega, alpha, dt):
    """Correct relative linear/angular momentum in the moving frame before its contact impulse."""
    p, v, axis = pipe.p, pipe.v, pipe.a
    wx = np.array([0., 0., omega])
    derivative = np.array([0., 0., alpha])
    pipe.v = v - dt * (linear + 2*np.cross(wx, v) + np.cross(derivative, p) + np.cross(wx, np.cross(wx, p)))
    change = np.cross(pipe.angular(pipe.l), axis)
    rotating_tensor = (I-IP)*(change * (axis @ wx) + axis * (change @ wx))
    pipe.l -= dt*(np.cross(wx, pipe.l + momentum(axis, wx)) + rotating_tensor + momentum(axis, derivative))


def frames(piece, row, cache, dt):
    route, before, _ = placed_route(piece, row["actualStationMmDeg"])
    sector = -.18 if piece == "coral-A" else .22
    path = cache / (piece + "-nominal.csv")
    if sha(path) != row["motionSHA256"]:
        raise ValueError("navigation does not supply this path")
    samples = np.loadtxt(path, delimiter=",", skiprows=1)[:, 1:]
    goal = route.end
    release = route.duration + 4.5
    times = np.arange(-2*dt, release+1, dt)
    poses = []
    for t in times:
        if t < before.duration:
            chassis, _ = before.at(t)
        else:
            point = interpolate(samples, t - before.duration)
            chassis = [goal[0] + point[0]/1000, goal[1]+point[1]/1000-sector, point[2]]
        u = np.clip((t - route.duration - 1.5) / 3, 0, 1)
        slide = .370 * u**3 * (10 - 15*u + 6*u*u)
        x, y, heading = chassis
        poses.append((x + slide*math.cos(heading), y + slide*math.sin(heading), heading, slide))
    poses = np.asarray(poses)
    poses[:, 2] = np.unwrap(poses[:, 2])
    rates = np.gradient(poses[:, :3], dt, axis=0)
    accel = np.gradient(rates, dt, axis=0)
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    body_accel = np.column_stack((c*accel[:, 0]+s*accel[:, 1], -s*accel[:, 0]+c*accel[:, 1], np.zeros(len(c))))
    return times, poses, rates, accel[:, 2], body_accel, release


def run(piece, dt, cache, mode):
    label = "first" if piece == "coral-A" else "return"
    feed = read(HERE / (label+"-feed.json"))
    source = next(r for r in feed["cases"] if r["name"] == label + "-nominal")
    state = source["stateAtDrive"]
    if state is None:
        raise ValueError("source failed before the departure clock")
    row = next(r for r in read(HERE/"navigation.json")["cases"] if r["pipe"] == piece and r["mode"] == "nominal")
    if mode.startswith("pads"):
        from pad_contact import SensorShelf
        pipe=SensorShelf(row["actualStationMmDeg"],pad_friction=.15)
        pipe.resolve_edge=mode=="pads-edge"
    else:
        pipe = Pipe(row["actualStationMmDeg"], relief=True)
    pipe.p, pipe.a, pipe.v = np.array(state[:3])/1000, np.array(state[3:6]), np.array(state[6:9])
    pipe.l = momentum(pipe.a, np.array(state[9:]))
    pipe.touch = ["continued station support"]
    times, poses, rates, alpha, acc, release = frames(piece, row, cache, dt)
    controller = Trace("RAMP_090")
    samples = []
    result = {"pipe":piece, "dtS":dt, "mode":mode, "feedTrajectorySha256":source["trajectorySHA256"],
              "driveWallS": feed["scheduledDriveWallS"], "poseSha256":row["motionSHA256"],
              "navigationAccepted":row["accepted"], "releaseRequestedWallS": feed["scheduledDriveWallS"]+release}
    dt_ms = round(.001/dt)
    for i in range(2, len(times)-1):
        t = times[i]
        fictitious(pipe, acc[i], rates[i,2], alpha[i], dt)
        target = 0. if t < release else controller.held(t-release)
        pipe.moves(target, dt)
        if i % dt_ms == 0 or pipe.hard:
            samples.append([t+dt, *pipe.p, *pipe.a, *pipe.v, *pipe.angular(pipe.l), *poses[i+1]])
        if pipe.hard or (pipe.loss and t < release):
            result.update(outcome="receiver stop or shelf loss before confirmed discharge", hardStop=pipe.hard,
                          wallS=feed["scheduledDriveWallS"]+t+dt, relativePositionMm=(1000*pipe.p).tolist())
            break
        if t >= release:
            radial = -R*(UP - pipe.a*pipe.a[2])/math.sqrt(1-pipe.a[2]**2)
            last_foot = pipe.p[0] + radial[0] - HALF*abs(pipe.a[0])
            departed=last_foot>EDGE
            if mode=="pads-edge":
                # Beyond 30 mm of forward travel the source has passed the possible
                # corner recontact and has a separately checked positive gap to all shelf material.
                departed=last_foot>EDGE+.030
            if departed and pipe.v[0] > 0:
                pos, heading = poses[i+1,:2], poses[i+1,2]
                c, s = math.cos(heading), math.sin(heading)
                rot = np.array([[c,-s,0],[s,c,0],[0,0,1]])
                lip = np.array(corrected().LIP)
                center = rot @ pipe.p + np.r_[pos-lip, 0.]
                v = rot @ (pipe.v + np.cross([0,0,rates[i+1,2]], pipe.p)) + np.r_[rates[i+1,:2], 0.]
                w = rot @ (pipe.angular(pipe.l) + [0,0,rates[i+1,2]])
                result.update(outcome="resolved pad and shelf edge" if mode=="pads-edge" else "last flat-shelf generator leaves support", positionM=center.tolist(),
                              velocityMps=v.tolist(), axis=(rot@pipe.a).tolist(), angularRadps=w.tolist(),
                              wallS=feed["scheduledDriveWallS"]+t+dt, chassisAtReefXYm=(pos-lip-.37*np.array([c,s])).tolist(),
                              chassisHeadingRad=heading, edgeRobotMm=(1000*pipe.p).tolist())
                break
    else:
        raise ValueError("No terminal condition")
    sample = np.asarray(samples)
    if mode.startswith("pads"):
        surface=np.asarray(pipe.trace)
        history=cache/(piece+"-"+mode+"-"+str(round(dt*1e6))+"-forces.csv")
        np.savetxt(history,surface,delimiter=",",header="since_drive_s,pad0_N,pad1_N,pad2_N",comments="",fmt="%.9f")
        result.update(padLoadSha256=sha(history),padPeakN=surface[:,1:].max(axis=0).tolist() if len(surface) else [],
                      maxPadPenetrationMm=1000*pipe.max_pad_indent,edgeImpulseNs=pipe.edge_impulse,
                      edgeContactsSinceDriveS=pipe.edge_contacts)
    file = cache / (piece+"-"+mode+"-"+str(round(1e6*dt))+".npy")
    np.save(file, sample)
    result.update(rawSha256=sha(file), peakOutwardDemandMm=pipe.peak, peakTiltDeg=pipe.peak_tilt,
                  minMaxRetainedCenterMm=[(1000*x).tolist() for x in (sample[sample[:,0]<release,1:4].min(axis=0),
                                                                       sample[sample[:,0]<release,1:4].max(axis=0))])
    output = HERE / file.with_suffix(".json").name
    output.write_text(json.dumps(result, indent=2)+"\n", encoding="utf8", newline="\n")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("piece", choices=["coral-A","coral-B"])
    parser.add_argument("cache", type=Path)
    parser.add_argument("--dt", type=float, default=.0005)
    parser.add_argument("--mode", choices=["coupled","pads","pads-edge"], default="pads-edge")
    args = parser.parse_args()
    run(args.piece, args.dt, args.cache, args.mode)
