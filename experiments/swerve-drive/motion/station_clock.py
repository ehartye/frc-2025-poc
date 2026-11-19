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
