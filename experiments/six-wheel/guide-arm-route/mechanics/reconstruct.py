"""Reconstruct the two sides' integration using each saved successor pipe trajectory."""

import numpy as np
from arm_contact import ContactResponse
from beam import guide


def check(arms, pose, trace, recorded, modes):
    response = ContactResponse(arms)
    pipe = guide.GuidedPipe(
        pose,
        0.0,
        guide.Guide(
            throat_mm=-0.3,
            stiffness_n_mm=10,
            damping_n_s_m=200,
            friction=0.1,
            sectors=128,
            radial_bins=4,
            x_width_mm=40,
            upper_mm=813,
            x_offset_mm=20,
        ),
    )
    az = trace[:, 6]
    radius = guide.feed.HALF * abs(az) + guide.feed.R * np.sqrt(
        np.maximum(0.0, 1.0 - az**2)
    )
    inside = (trace[:, 3] / 1000 + radius >= 0.625) & (
        trace[:, 3] / 1000 - radius <= 0.813
    )
    worst_q = worst_v = 0.0
    for i in np.flatnonzero(inside[:-1]):
        row = trace[i]
        pipe.time = (row[0] - 100) / 1000
        pipe.p = row[1:4] / 1000
        pipe.a = row[4:7].copy()
        pipe.v = row[7:10].copy()
        omega = row[10:13]
        pipe.l = (
            guide.feed.IP * omega
            + (guide.feed.I - guide.feed.IP) * (omega @ pipe.a) * pipe.a
        )
        count = len(response.trace)
        response.step(pipe, (trace[i + 1, 0] - row[0]) / 1000)
        if len(response.trace) > count:
            n = response.q.size
            worst_q = max(
                worst_q,
                float(np.max(abs(modes[count, 1 : 1 + n] - response.q.ravel()))),
            )
            worst_v = max(
                worst_v, float(np.max(abs(modes[count, 1 + n :] - response.v.ravel())))
            )
    result = np.asarray(response.trace)
    if result.shape != recorded.shape:
        raise ValueError("Contact occurred on a different time grid")
    dif = np.max(abs(result - recorded), axis=0)
    if (
        max(dif[1:3]) > 1e-7
        or max(dif[9:11]) > 1e-10
        or max(dif[-2:]) > 1e-10
        or worst_q > 1e-10
        or worst_v > 1e-6
    ):
        raise ValueError("Successor contact and modes did not reproduce")
    return dict(
        guideSamples=len(result),
        peakNormalErrorN=dif[1:3].tolist(),
        peakFaceErrorM=dif[9:11].tolist(),
        peakInputWorkErrorJ=dif[-2:].tolist(),
        peakModalQError=worst_q,
        peakModalVError=worst_v,
    )
