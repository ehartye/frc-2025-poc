"""Integrate the changing fore/aft rail supports; save surface coefficients for independent replay."""

import argparse
import itertools
import json
import os
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import numpy as np
from dynamics import (
    G,
    HERE,
    ROOT,
    button_gaps,
    contact_rows,
    digest,
    floors,
    matrices,
    profile,
    project_legacy,
    read,
    shape_model,
    surface,
)


def equilibrium(M, K, weight, J, gaps, stiffness):
    for indices in itertools.combinations(range(6), 4):
        ix = list(indices)
        A = J[ix]
        try:
            q = np.linalg.solve(
                K + A.T @ (stiffness[ix, None] * A), weight - A.T @ (stiffness[ix] * gaps[ix])
            )
        except np.linalg.LinAlgError:
            continue
        d = gaps + J @ q
        reaction = np.maximum(0, -d) * stiffness
        if np.max(abs(weight - K @ q + J.T @ reaction)) < 1e-5:
            return q
    raise ValueError("No consistent gravitational equilibrium at zero stroke")


def registration():
    geo = read(HERE / "geometry.json")
    keeper = read(HERE / "keepers.json")
    native = geo["sources"]
    for path, pin in native.items():
        if digest(ROOT / path) != pin:
            raise ValueError(f"Changed native source {path}")
    if digest(HERE / "rail-fixture.step") != keeper["stepSha256"]:
        raise ValueError("Changed captured rail STEP")
    t = np.array(geo["strokeMm"])
    spans = np.array(floors(t / 1000, geo)).T * 1000
    gaps = button_gaps(t / 1000) * 1000
    error = 0.0
    for j, s in enumerate(t):
        for side in "LR":
            sample = geo["contacts"][side][str(s)]
            expected = (
                sample["innerOnMiddle"]["downwardXMm"] + sample["middleOnOuter"]["downwardXMm"]
            )
            if not np.allclose(spans[j], expected, atol=0.0001, rtol=0):
                raise ValueError("Moving land formulas differ from native contacts")
        for i, seat in enumerate(geo["fixedButtons"][str(s)]):
            if seat["overhead"] != np.isfinite(gaps[j, i]):
                raise ValueError("Wrong button coverage")
            if seat["overhead"]:
                error = max(error, abs(gaps[j, i] - seat["firstGapMm"]))
    if error > 0.005:
        raise ValueError("Button skin section differs from reopened STEP")
    project = read(ROOT / "design/receiver-drives/sizing.json")["slide"][
        "place-extend outer drag 25 N"
    ]
    s, v, a = profile(np.array([0, 1.5, 3 * (3 - np.sqrt(3)) / 6, 3]))
    if not np.isclose(max(v), project["outer_v_mps"]) or not np.isclose(
        max(a), project["outer_acc_mps2"]
    ):
        raise ValueError("Changed forward design profile")
    return geo, keeper, error


def table():
    # Force is transferred vertically at the stationary button axis. At the
    # last few mm only the disk edge can touch; clamp the application point
    # to the actual rim and report this lever-arm/section approximation.
    s = np.arange(0, 0.22226, 0.00025)
    local = np.maximum(-0.100, 0.118 - s)
    buttons = []
    for cy in (-0.066, 0.042):
        B = surface(local, np.full(len(s), cy))
        B[:, 0] -= 1
        B[:, 1] -= 0.118
        buttons.append(B)
    return np.array(buttons).transpose(1, 0, 2)


def trial(
    state,
    label,
    out,
    dt,
    stiffness=1e7,
    gap=0.00035,
    dressed=True,
    cg=0.04,
    keepers=True,
    active_buttons=True,
    challenge=False,
    duration=3.0,
    motion=None,
    sample_us=1000.0,
):
    if (
        not np.isfinite([dt, stiffness, gap, cg, duration, sample_us]).all()
        or min(dt, stiffness, duration, sample_us) <= 0
    ):
        raise ValueError("Expected positive duration, timestep, sample interval and stiffness")
    for key, dim in (("p", 3), ("a", 3), ("z", 21), ("zdot", 21)):
        value = np.asarray(state[key])
        if value.shape != (dim,) or not np.isfinite(value).all():
            raise ValueError("Wrong incoming state dimension or nonfinite value for " + key)
    if abs(np.linalg.norm(state["a"]) - 1) > 0.00001:
        raise ValueError("Pipe axis must have unit length")
    geo, keeper, _ = registration()
    M, K, C, g, pitch, pipe, weights, masses = matrices(state, dressed, cg)
    n = len(g)
    read_payload = read(HERE / "balance.json")["pipeMassKg"]
    if motion is None:
        t = np.arange(round((duration + 0.2) / dt) + 1) * dt
        stroke, speed, accel = profile(t, duration)
    else:
        # The caller provides a registered time law for the achieved OUTWARD
        # rack. We do not silently reflect a withdrawal trace or synthesize
        # derivatives across a position jump.
        t = np.arange(int(np.floor(motion[-1, 0] / dt)) + 1) * dt
        stroke, speed, accel = [np.interp(t, motion[:, 0], motion[:, i]) for i in (1, 2, 3)]
    rows, derivatives = contact_rows(stroke, geo, keeper)
    floors_mm = keeper["lowerFloorTravelFromNominalMm"]
    gaps = np.array([floors_mm / 1000, floors_mm / 1000, gap] * 2)
    ks = np.full(6, stiffness)
    if not keepers:
        ks[[2, 5]] = 0
    J0 = np.pad(rows[0], ((0, 0), (0, n - 6)))
    q = equilibrium(M, K, g, J0, gaps, ks)
    v = np.zeros_like(q)
    projected, projected_v, roll, roll_v = project_legacy(state)
    if challenge:
        q += projected
        v += projected_v
    # Mass-proportional contact damping is local and unilateral. Free joint
    # travel transmits neither reaction nor damping force.
    inv = np.linalg.inv(M)
    mass_contact = 1 / np.einsum("ij,jk,ik->i", J0, inv, J0)
    damping = 2 * 0.06 * np.sqrt(ks * mass_contact)
    # Trapezoidal linear elastic/damping terms; contact and horizontal drive
    # moments are held for one step. Independent timestep replay tests them.
    inv_step = np.linalg.inv(M + 0.5 * dt * C + 0.25 * dt * dt * K)
    V = inv_step @ (M - 0.5 * dt * C - 0.25 * dt * dt * K)
    D = -dt * inv_step @ K
    G0 = dt * inv_step @ g
    P0 = dt * inv_step @ pitch
    force_map = dt * inv_step
    R = np.zeros((8, n))
    limits = np.r_[np.full(6, np.inf), [750.0, 750.0]]
    buttons = table()
    buttons_index = np.clip(np.rint(stroke / 0.00025).astype(int), 0, len(buttons) - 1)
    gap_profile = button_gaps(stroke)
    local_gap_grid = button_gaps(np.arange(len(buttons)) * 0.00025)
    out.mkdir(parents=True, exist_ok=True)
    initial_button = buttons[buttons_index[0]]
    plungers = np.maximum(0, -gap_profile[0] - initial_button @ q)
    lastgap = gap_profile[0]
    max_force = np.zeros(6)
    max_button = np.zeros(2)
    max_press = plungers.copy()
    edge_jump = np.zeros(2)
    neg_normal = np.inf
    history = []
    step_stride = round(sample_us * 1e-6 / dt)
    if not step_stride or not np.isclose(step_stride * dt, sample_us * 1e-6):
        raise ValueError("Trace interval must be an integer number of steps")
    peak_edge_state = [None, None]
    first_edge = None
    required_retract = np.zeros(2)
    normal_min_state = None
    local_min = np.inf
    sample_normal = None
    peak_abs_accel = 0.0
    probes = surface(np.array([-0.1, state["p"][0], 0.165]), np.zeros(3))
    slopes = surface(np.array([state["p"][0], 0.165]), np.zeros(2), True)
    start_z = probes @ q
    min_z = np.full(3, np.inf)
    max_z = -min_z
    max_tilt = np.zeros(2)
    first_separation = None
    lost = None
    for i, clock in enumerate(t):
        J = rows[i]
        speed_at = J @ v[:6] + (derivatives[i] @ q[:6]) * speed[i]
        depth = -(gaps + J @ q[:6])
        B = buttons[buttons_index[i]]
        bj = gap_profile[i]
        depth_button = -bj - B @ q
        # Flag side entry while the physical button has recovered into a deep
        # pocket. An axial damping model cannot clear a lateral obstruction.
        strike = np.isfinite(bj) & (lastgap > bj + 0.001)
        intrusion = np.where(strike, np.maximum(0, depth_button - plungers), 0.0)
        required_retract = np.maximum(required_retract, depth_button)
        for j in np.flatnonzero(intrusion > edge_jump):
            peak_edge_state[j] = dict(
                timeS=float(clock),
                strokeMm=float(stroke[i] * 1000),
                localButtonCenterXMm=float(118 - stroke[i] * 1000),
                localButtonCenterYMm=(-66, 42)[j],
                oldGapMm=float(lastgap[j] * 1000) if np.isfinite(lastgap[j]) else None,
                newGapMm=float(bj[j] * 1000),
                preEntryPlungerMm=float(plungers[j] * 1000),
                intrusionMm=float(intrusion[j] * 1000),
            )
            if active_buttons and first_edge is None:
                first_edge = peak_edge_state[j]
        edge_jump = np.maximum(edge_jump, intrusion)
        plungers = np.maximum(0, plungers - dt * 0.008 / 0.3)
        # Smooth spatial slope under a stationary finite button. At a step
        # retain the geometric strike instead of applying a fictitious impulse.
        bs = buttons_index[i]
        prev = max(bs - 1, 0)
        nxt = min(bs + 1, len(buttons) - 1)
        change = np.zeros(2)
        if speed[i] and nxt != prev:
            slope = (buttons[nxt] - buttons[prev]) @ q
            before, after = local_gap_grid[prev], local_gap_grid[nxt]
            smooth = np.isfinite(before) & np.isfinite(after)
            difference = np.zeros(2)
            difference[smooth] = after[smooth] - before[smooth]
            smooth &= abs(difference) < 0.001
            change[smooth] = (
                (slope[smooth] + difference[smooth]) * speed[i] / (0.00025 * (nxt - prev))
            )
        lastgap = bj
        z = probes @ q
        min_z = np.minimum(min_z, z)
        max_z = np.maximum(max_z, z)
        max_tilt = np.maximum(max_tilt, abs(slopes @ q))
        # Implicit contact solves for a common midpoint velocity. Evaluating
        # the rail reactions one step late injects energy on a loaded contact.
        R[:6, :6] = J
        R[6:] = B
        relative_rate = np.r_[derivatives[i] @ q[:6] * speed[i], change]
        coeff = np.r_[ks, np.zeros(2)]
        gap_now = np.r_[-depth, -depth_button]
        cd = np.r_[damping, [1500.0, 1500.0]]
        vfree = V @ v + D @ q + G0 + P0 * accel[i]
        rate_free = R @ (0.5 * (v + vfree)) + relative_rate
        compress = -gap_now - 0.5 * dt * rate_free
        can = np.r_[compress[:6] > 0, (compress[6:] >= plungers) & active_buttons]
        force = np.zeros(8)
        active = np.flatnonzero(can)
        # Subtract constrained rows whose reactions would be tensile and cap
        # saturated dampers, then solve the remaining station reactions again.
        coefficient = 0.25 * dt * coeff + 0.5 * cd
        rhs = np.zeros(8)
        rhs[:6] = -ks * gap_now[:6] - (0.5 * dt * ks + damping) * rate_free[:6]
        rhs[6:] = -1500 * rate_free[6:]
        for _ in range(9):
            if not active.size:
                break
            RR = R[active]
            system = np.eye(active.size) + (coefficient[active, None] * RR) @ force_map @ RR.T
            residual = rhs[active] - coefficient[active] * (RR @ (force_map @ (R.T @ force)))
            candidate = np.linalg.solve(system, residual)
            clipped = np.clip(candidate, 0, limits[active])
            constrained = candidate != clipped
            if not np.any(constrained):
                force[active] = candidate
                break
            force[active[constrained]] = clipped[constrained]
            active = active[~constrained]
        vnew = vfree + force_map @ (R.T @ force)
        if active_buttons:
            plungers = np.maximum(plungers, depth_button)
        max_press = np.maximum(max_press, plungers)
        button = force[6:]
        force = force[:6]
        max_force = np.maximum(max_force, force)
        max_button = np.maximum(max_button, button)
        normal_stations = G + pipe @ ((vnew - v) / dt)
        required = weights @ normal_stations
        local_min = min(local_min, float(normal_stations.min()))
        if required < neg_normal:
            neg_normal = float(required)
            normal_min_state = dict(
                timeS=float(clock),
                strokeMm=float(stroke[i] * 1000),
                requiredMeanNormalMps2=float(required),
                requiredVertForceN=float(required * read_payload),
                foreAftAccelerationMps2=float(accel[i]),
            )
        peak_abs_accel = max(peak_abs_accel, abs(accel[i]))
        if required < 0 and first_separation is None:
            first_separation = dict(timeS=float(clock), strokeMm=float(stroke[i] * 1000))
        if i % step_stride == 0:
            history.append(
                np.r_[clock, stroke[i], speed[i], accel[i], q, v, force, button, required]
            )
        if not np.isfinite(q).all() or max(abs(q[[1, 3, 5]])) > 0.1:
            lost = float(clock)
            break
        if i == len(t) - 1:
            break
        q += 0.5 * dt * (v + vnew)
        v = vnew
    data = np.array(history)
    dest = out / (label + ".npz")
    np.savez_compressed(
        dest,
        timeS=data[:, 0],
        strokeM=data[:, 1],
        speedMps=data[:, 2],
        accelMps2=data[:, 3],
        q=data[:, 4 : 4 + n],
        v=data[:, 4 + n : 4 + 2 * n],
        railForceN=data[:, -9:-3],
        buttonN=data[:, -3:-1],
        requiredMeanNormalMps2=data[:, -1],
        plateShape=shape_model()[0],
    )
    return dict(
        label=label,
        dtS=dt,
        durationS=duration,
        stiffnessNPerM=stiffness,
        upperGapMm=1000 * gap,
        dressed=dressed,
        unlocatedStageCGm=cg,
        keepers=keepers,
        activeButtons=active_buttons,
        projectedLegacyDisturbance=challenge,
        ignoredRollDisturbanceMm=roll * 1000,
        ignoredRollVelocityMmS=roll_v * 1000,
        stageMassKg=masses,
        traceSha256=digest(dest),
        startProbeMm=(1000 * start_z).tolist(),
        finalProbeMm=(1000 * (probes @ q)).tolist(),
        finalChangeFromOwnStartMm=(1000 * (probes @ q - start_z)).tolist(),
        minProbeMm=(1000 * min_z).tolist(),
        maxProbeMm=(1000 * max_z).tolist(),
        peakSlopeRad=max_tilt.tolist(),
        peakRailN=max_force.tolist(),
        peakButtonN=max_button.tolist(),
        peakButtonStrokeMm=(max_press * 1000).tolist() if active_buttons else None,
        possibleSharpEdgeInterferenceMm=(1000 * edge_jump).tolist() if active_buttons else None,
        minRequiredMeanNormalMps2=float(neg_normal),
        firstNegativeMeanNormalS=first_separation,
        sampledEveryUs=sample_us,
        peakPossibleEdgeState=peak_edge_state if active_buttons else None,
        firstPossibleEdgeState=first_edge,
        geometricRetractionFromFreeTopMm=(required_retract * 1000).tolist(),
        minimumMeanNormalState=normal_min_state,
        minimumLocalNormalMps2=local_min,
        peakPrescribedAccelerationMps2=peak_abs_accel,
        firstLargeRotationS=lost,
        lastStrokeMm=float(stroke[i] * 1000),
        lastTimeS=float(clock),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--case", choices=["leftSmall", "rightSmall"], default="leftSmall")
    parser.add_argument("--dt-us", type=float, default=62.5)
    parser.add_argument(
        "--variant",
        choices=[
            "nominal",
            "soft",
            "wide",
            "bare",
            "forward-cg",
            "no-keepers",
            "no-buttons",
            "legacy",
            "slow",
            "retracted-slow",
            "retracted-slower",
        ],
        default="nominal",
    )
    parser.add_argument("--sample-us", type=float, default=1000.0)
    parser.add_argument(
        "--state",
        type=Path,
        help="Source JSON containing a complete named pre-extension state under beforeFirstWheelService or at the root",
    )
    parser.add_argument("--state-sha256")
    parser.add_argument(
        "--motion",
        type=Path,
        help="CSV time_s,stroke_m,velocity_mps,acceleration_mps2, beginning at stow",
    )
    parser.add_argument("--motion-sha256")
    a = parser.parse_args()
    geo, keeper, err = registration()
    if a.state:
        if digest(a.state) != a.state_sha256:
            raise ValueError("Unregistered incoming state")
        source = read(a.state)
        state = source.get("beforeFirstWheelService", source)
        state_file = a.state
    else:
        state_file = ROOT / "split-guide-acquisition/restart.json"
        state = next(
            r["beforeFirstWheelService"]
            for r in read(state_file)["cases"]
            if r["side"] == a.case
            and np.isclose(r["stepS"], 15.625e-6, atol=1e-12)
            and r.get("beforeFirstWheelService")
        )
    motion = None
    if a.motion:
        if digest(a.motion) != a.motion_sha256:
            raise ValueError("Unregistered achieved-motion input")
        raw = np.genfromtxt(a.motion, delimiter=",", names=True)
        if raw.dtype.names != ("time_s", "stroke_m", "velocity_mps", "acceleration_mps2"):
            raise ValueError("Motion columns")
        motion = np.column_stack([raw[k] for k in raw.dtype.names])
        if (
            len(motion) < 2
            or not np.isfinite(motion).all()
            or abs(motion[0, 0]) > 1e-12
            or abs(motion[0, 1]) > 1e-6
            or np.min(np.diff(motion[:, 0])) <= 0
            or min(motion[:, 1]) < 0
            or max(motion[:, 1]) > 0.370001
            or np.any(np.diff(motion[:, 1]) < -1e-8)
        ):
            raise ValueError("Expected finite outbound history from stow")
        intervals = np.diff(motion[:, 0])
        displacement = np.diff(motion[:, 1])
        velocity = np.diff(motion[:, 2])
        if (
            abs(displacement - 0.5 * intervals * (motion[:-1, 2] + motion[1:, 2])).max() > 0.0001
            or abs(velocity - 0.5 * intervals * (motion[:-1, 3] + motion[1:, 3])).max() > 0.01
        ):
            raise ValueError(
                "Supplied velocity or acceleration is inconsistent with the achieved trajectory"
            )
    opts = {
        "nominal": {},
        "soft": {"stiffness": 1e6},
        "wide": {"gap": 0.00070},
        "bare": {"dressed": False},
        "forward-cg": {"cg": 0.12},
        "no-keepers": {"keepers": False},
        "legacy": {"challenge": True},
        "no-buttons": {"active_buttons": False},
        "slow": {"duration": 6.0},
        "retracted-slow": {"duration": 6.0, "active_buttons": False},
        "retracted-slower": {"duration": 12.0, "active_buttons": False},
    }
    label = f"{a.case}-{a.variant}-{a.dt_us:g}us"
    result = trial(
        state,
        label,
        a.output,
        a.dt_us * 1e-6,
        motion=motion,
        sample_us=a.sample_us,
        **opts[a.variant],
    )
    result.update(
        stateSourceSha256=digest(state_file),
        motionSha256=digest(a.motion) if a.motion else None,
        motionSource="external achieved stroke"
        if a.motion
        else "prescribed forward project quintic",
        localPipePositionM=state["p"],
        tubeAxis=state["a"],
        nativeButtonGapCheckMaxMm=err,
        keeperStepSha256=keeper["stepSha256"],
    )
    (a.output / (label + ".json")).write_bytes(
        (json.dumps(result, indent=2, allow_nan=False) + "\n").encode()
    )


if __name__ == "__main__":
    main()
