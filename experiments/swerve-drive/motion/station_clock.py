"""Evaluate both camera witnesses on the original 5 ms swerve application clock."""

import math


def service_gate(frames, feed, latency_ms):
    if latency_ms < 0:
        raise ValueError("Packet transport cannot be negative")
    stamps = [f["captureJavaMs"] for f in frames]
    if stamps != sorted(set(stamps)):
        raise ValueError("Camera captures must be distinct and chronological")
    epoch = feed["javaEpochWallS"]
    service = lambda t: math.ceil(t * 200 - 1e-9) / 200
    first_drive = service(feed["endWallS"])
    by_stamp = {f["captureJavaMs"]: f for f in frames}
    required = (5372, 5389, 5406, 5423, 5440)
    arrivals = {
        stamp: service(epoch + (stamp + latency_ms) / 1000) for stamp in by_stamp
    }
    usable = [
        ms
        for ms in required
        if ms in by_stamp and by_stamp[ms]["accepted"] and arrivals[ms] <= first_drive
    ]
    return dict(
        firstEligibleWheelServiceWallS=first_drive,
        transportMs=latency_ms,
        serviceWallS=arrivals,
        eligibleCapturesJavaMs=usable,
        accepted=len(usable) == len(required),
    )


def first_hold(issued, captures, first, stop, latency=0.015, freshness=0.040):
    if not issued["gate"]["accepted"]:
        return first, "station custody withheld"
    records = [
        (issued["epoch"] + row["captureJavaMs"] * 0.001, row["accepted"])
        for row in issued["frames"]
    ]
    records += [(r["captureWallS"], r["accepted"]) for r in captures]
    records.sort()
    for tick in range(round(first * 200), round(stop * 200) + 1):
        now = tick * 0.005
        arrived = [r for r in records if r[0] + latency <= now + 1e-9]
        if not arrived or not arrived[-1][1] or now - arrived[-1][0] > freshness + 1e-9:
            return now, "invalid or expired custody observation"
    return stop, "end of departure command"
