"""Transport gyro heading and wheel targets from the station on a 5 ms drive service."""
import math
import numpy as np
from route import wheel_steps

def drive(plan, start, scale, width_mm, bias_deg_s, gain_error, pace=1.):
    dt = .005
    duration=plan.duration/pace
    times = np.arange(math.ceil((duration+.55)/dt)+1)*dt
    nominal = np.array([plan.at(min(t*pace,plan.duration))[0] for t in times])
    nominal[:,2] = np.unwrap(nominal[:,2])
    feedforward = wheel_steps(nominal)*1000/dt
    pose = np.asarray(start, dtype=float)
    poses = [pose.copy()]
    wheels = np.zeros(2)
    wheel_history = [wheels.copy()]
    yaw = 0.
    last_yaw = 0.
    last_capture = None
    queued = []
    for i,t in enumerate(times[:-1]):
        if i%2 == 0:
            queued.append((t+.020, t, yaw*(1+gain_error)+math.radians(bias_deg_s)*t))
        if queued and queued[0][0] <= t+1e-10:
            _, last_capture, last_yaw = queued.pop(0)
        targets = np.zeros(2)
        if last_capture is not None and t-last_capture <= .060 and t <= duration+.30:
            # Zeroed at the modeled station orientation. Only the rate sensor and shaft speeds enter feedback.
            estimate = nominal[0,2]+last_yaw+(t-last_capture)*(wheels[1]-wheels[0])/580.
            yaw_rate = np.clip(6*(nominal[i,2]-estimate),-.45,.45)
            targets = np.clip(feedforward[i]+np.array([-290.,290.])*yaw_rate,-1250,1250)
        accel = 640 if abs(feedforward[i].mean())<1 else 1200
        previous = wheels.copy()
        wheels += np.clip(targets-wheels,-accel*dt,accel*dt)
        distances = (previous+wheels)*(.5*dt)*scale
        dh = (distances[1]-distances[0])/width_mm
        ds = distances.mean()*.001*np.sinc(dh/(2*np.pi))
        mid = pose[2]+dh/2
        pose = pose+np.array([ds*math.cos(mid),ds*math.sin(mid),dh])
        yaw += dh
        poses.append(pose.copy())
        wheel_history.append(wheels.copy())
    if max(abs(wheels)) > .001:
        raise ValueError("Nonstationary transition to field vision")
    return times, np.asarray(poses), np.asarray(wheel_history)
