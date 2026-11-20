"""Advance the *retained* six-DOF pipe against a coupled telescoping receiver.

The initial contact surface is registered to the original solved stow datum.
The outer carriage follows a commanded, measured-displacement scenario. The
plate and rail rotations, vertical motion, pipe, yokes and end-belt contacts
exchange reactions at each integration step. The latter two are rigidly fixed
to the inner rails; differential deck strain affects the floor and ports.
"""

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "cam-scoring"))
sys.path.insert(0, str(HERE.parent / "score-stroke"))
from cam_model import plate

baseline_sections = plate.sections
import captivity
from loaded_rack import read, digest
from fingers import geometry, mechanics, observations
from retained_report import permission
import dynamics
import dynamics_run as rack
from rail_contact import RailContact
from open_pins import minimum as open_clearance
from cam_model import candidate_gaps


def enable_rack():
    # captivity imported the cam-modified plate exactly once. Reuse that material
    # section when registering the changed native stock masses and cap profile.
    original = dynamics.read
    live_sections = dynamics.plate.sections
    try:
        dynamics.plate.sections = baseline_sections
        geo, keep, error = rack.registration()
    finally:
        dynamics.plate.sections = live_sections
    for k, delta in captivity.CAM["volumeDeltaMm3"].items():
        item = geo["movingStock"][k]
        item["massKg"] += delta * item["densityKgM3"] * 1e-9
    dynamics.read = lambda p: (
        geo if p == dynamics.HERE / "geometry.json" else original(p)
    )
    rack.registration = lambda: (geo, keep, error)
    rack.button_gaps = candidate_gaps


def moment(omega, axis, feed):
    return feed.IP * omega + (feed.I - feed.IP) * axis * (axis @ omega)


def frame(rail, travel, speed):
    # Inner-stage x is longitudinal, +z is upward. Positive pitch raises nose.
    angle = rail.q[5] - rail.start[5]
    c, s = math.cos(angle), math.sin(angle)
    r = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])
    return (
        r,
        np.array([travel, 0.0, 0.467 + rail.q[4] - rail.start[4]]),
        np.array([speed, 0.0, rail.v[4]]),
        np.array([0.0, -rail.v[5], 0.0]),
    )


PIVOT = np.array([0.0, 0.0, 0.467])


def inertial(pipe, f, feed):
    r, o, od, w = f
    arm = r @ (pipe.p - PIVOT)
    axis = r @ pipe.a
    return (
        o + arm,
        r @ pipe.v + od + np.cross(w, arm),
        axis,
        r @ pipe.l + moment(w, axis, feed),
    )


def local(pipe, f, world, feed):
    r, o, od, w = f
    p, v, axis, mom = world
    pipe.p = PIVOT + r.T @ (p - o)
    pipe.v = r.T @ (v - od - np.cross(w, p - o))
    pipe.a = r.T @ axis
    pipe.l = r.T @ (mom - moment(w, axis, feed))


def install_floor(pipe, rail):
    old = pipe.table
    old_z = pipe.z.copy()
    x = np.arange(107) * 0.0025 - 0.1
    y = np.arange(145) * 0.0025 - 0.18
    X, Y = np.meshgrid(x, y)
    modes = dynamics.shape_model()[0]
    pieces = []
    for dx, dy in ((0, 0), (1, 0), (0, 1)):
        chunks = []
        for i in range(0, len(y), 8):
            b = dynamics.plate.basis(
                X[i : i + 8].ravel(), Y[i : i + 8].ravel(), 13, 20, dx, dy
            )
            chunks.append(b @ modes)
        pieces.append(np.concatenate(chunks).reshape(*X.shape, -1))
    # Constant last column preserves the complete pointwise incoming station
    # displacement, including its omitted side-to-side land disturbance.
    original = old @ old_z
    pipe.table = np.concatenate((np.stack(pieces, axis=2), original[..., None]), axis=3)

    def sync():
        pipe.z = np.r_[-(rail.q[6:] - rail.start[6:]), 1.0]
        pipe.zdot = np.r_[-rail.v[6:], 0.0]

    sync()
    pipe.advance_modes = lambda dt, q: setattr(pipe, "plate_force", q)
    return sync


def run(artifacts, side, stroke_mm, dt_us, control, end_time):
    out = artifacts / "low-placement/advance-25" / side
    src = artifacts / "r4-score-stroke"
    name = f"cam-station-{side}-31.25us-0.1mm"
    feed_input = read(src / (name + ".json"))
    pipe, _, _ = captivity.restore_station(src / name, feed_input)
    from threadpoolctl import threadpool_limits

    threadpool_limits(limits=1, user_api="blas")
    previous = read(out / "coral.json")
    state = out / "coral.npz"
    if digest(state) != previous["state"] or not previous["allGranted"]:
        raise ValueError("Missing retained source")
    captivity.restore(pipe, state)
    if abs(pipe.time + feed_input["lipWallS"] - previous["endWallS"]) > 1e-7:
        raise ValueError("Pipe clock mismatch")
    feed = captivity.engine.feed
    feed.cfg = replace(feed.cfg, slices=10)
    enable_rack()
    dt = dt_us * 1e-6
    times = np.arange(round(end_time / dt) + 1) * dt
    travel, speed, acc = (a * stroke_mm / 370 for a in dynamics.profile(times, 3.0))
    raw = {
        key: getattr(pipe, key).copy()
        for key in ("a", "p", "v", "l", "z", "zdot", "plunger")
    }
    rail = RailContact(raw, dt, travel, speed)
    f = frame(rail, 0.0, 0.0)
    world = (pipe.p.copy(), pipe.v.copy(), pipe.a.copy(), pipe.l.copy())
    local(pipe, f, world, feed)
    sync = install_floor(pipe, rail)
    max_pin = max_gap_ms = 0.0
    minimum = 1e6
    stride = round(0.005 / dt)
    if abs(stride * dt - 0.005) > 1e-12:
        raise ValueError("5 ms service requires a whole number of solver ticks")
    history = []
    permits = []
    reason = "end-of-window"
    event = None
    peak_work = 0.0
    first_free = None
    lost_s = 0.0
    begin = pipe.time
    issued = None
    min_open = math.inf
    prior_pin_limit = None
    for i, t in enumerate(times[:-1]):
        # Require at least five fresh 5 ms held states at the arrived stroke
        # before opening; a stuck yoke keeps the pusher disabled.
        if t >= 3.1 and issued is None and all(permits[-5:]) and len(permits) >= 5:
            issued = float(t)
        yoke = (
            max(0.0, 35.0 - 150.0 * (t - issued))
            if issued is not None and control != "stuck-yoke"
            else 35.0
        )
        belt = 1.05 if issued is not None and yoke < 0.001 else 0.0
        r = f[0]
        p0, v0, l0 = pipe.p.copy(), pipe.v.copy(), pipe.l.copy()
        if yoke == 0 and control != "legacy-bore":
            # The original spring formula assumes a pin INSIDE the bore. After
            # retraction the finite tube can pass outside the tip. Check that
            # separate configuration before allowing the unforced open state.
            rows = geometry(pipe.p * 1000, pipe.a, 0.0)
            gap = open_clearance(pipe.p * 1000, pipe.a)
            min_open = min(min_open, gap)
            legacy = any(
                p["overlap"] > 0 and p["clearance"] < 0 for s in rows for p in s["pins"]
            )
            if legacy and prior_pin_limit is None:
                prior_pin_limit = dict(
                    timeS=float(t),
                    finiteGapMm=gap,
                    centerMm=(pipe.p * 1000).tolist(),
                    axis=pipe.a.tolist(),
                )
            sweep = (
                dt
                * 1000
                * (np.linalg.norm(pipe.v) + 0.21 * np.linalg.norm(pipe.angular(pipe.l)))
                + 0.01
            )
            if gap <= sweep:
                reason = "finite open pin approaches pipe during ejection"
                break
            for s in rows:
                for pin_row in s["pins"]:
                    pin_row["forceN"] = 0.0
            if control == "zero-traction" and belt:
                feed.cfg = replace(feed.cfg, belt_mu=0.0)
        else:
            rows = mechanics(pipe, yoke, dt)
        # Include gravity in the rotating stage coordinate system. moves() already
        # applies vertical nominal gravity; replace its direction by this delta.
        actual_gravity = r.T @ np.array([0.0, 0.0, -feed.G])
        pipe.v += dt * (actual_gravity - np.array([0.0, 0.0, -feed.G]))
        pipe.plate_force = 0.0
        feed.Pipe.moves(pipe, belt, dt)
        force = feed.M * ((pipe.v - v0) / dt - actual_gravity)
        torque = (pipe.l - l0) / dt
        # Relative spring/actuator reactions against the receiver. Generalized
        # positive pitch is -Ry. Reactions include both bore pins and belt pads.
        load = np.zeros_like(rail.q)
        load[4] = -(r @ force)[2]
        arm = r @ (p0 - PIVOT)
        load[5] = (r @ torque)[1] + arm[2] * (r @ force)[0] - arm[0] * (r @ force)[2]
        plate = np.asarray(pipe.plate_force)
        if plate.ndim:
            load[6:] = -plate[:-1]
        rail.advance(i, acc[i], load)
        newframe = frame(rail, travel[i + 1], speed[i + 1])
        P, V, A, L = world
        V = V + r @ (pipe.v - v0)
        L = L + r @ (pipe.l - l0)
        P = P + dt * V
        omega = L / feed.IP + (1 / feed.I - 1 / feed.IP) * A * (A @ L)
        A = A + dt * np.cross(omega, A)
        A = A / np.linalg.norm(A)
        world = P, V, A, L
        f = newframe
        local(pipe, f, world, feed)
        sync()
        peak_work = max(peak_work, abs(float(force @ v0 + torque @ omega)) * dt)
        pin = max(pin["forceN"] for end in rows for pin in end["pins"])
        max_pin = max(max_pin, pin)
        if issued is None:
            minimum = min(
                minimum, min(pin["overlap"] for end in rows for pin in end["pins"])
            )
        if pipe.last_normal_n < 1e-8:
            lost_s += dt
            if first_free is None:
                first_free = float(t + dt)
            max_gap_ms = max(max_gap_ms, lost_s * 1000)
        else:
            lost_s = 0.0
        if (i + 1) % stride == 0:
            ports = [bool(v) for v in pipe.ports()]
            obs = observations(geometry(pipe.p * 1000, pipe.a, yoke), yoke, "nominal")
            grant = permission([v for _, v in obs], ports, [v for v, _ in obs])
            permits.append(grant)
            history.append(
                [
                    t + dt,
                    travel[i + 1],
                    *P,
                    *A,
                    *V,
                    *L,
                    yoke,
                    pipe.belt,
                    pipe.last_normal_n,
                    *ports,
                    grant,
                    *rail.q[:6],
                ]
            )
            if issued is None and not grant:
                reason = "closed-yoke service lost measured occupancy"
                break
        if pipe.hard or pin > 6 or any(end["compression"] >= 8 for end in rows):
            reason = "modeled retention force or end travel exceeded"
            break
        if rail.max_pitch > 0.04 or rail.max_heave > 0.005:
            reason = "carriage excursion outside bounded placement geometry"
            break
        if (
            issued is not None
            and yoke == 0
            and pipe.last_normal_n == 0
            and pipe.p[0] > 0.165
            and all(not pipe.footprint(s) for s in (-1, 1))
        ):
            # A separation candidate also needs complete finite geometry beyond the
            # lip, rear wall clearance, no renewed robot contact and target support.
            reason = "free payload candidate beyond last loaded shelf generator"
            event = dict(
                localP=pipe.p.tolist(),
                chassisP=P.tolist(),
                axis=A.tolist(),
                velocity=V.tolist(),
                angularMomentum=L.tolist(),
                omega=omega.tolist(),
                time=float(t + dt),
            )
            break
    trace = out / f"coupled-{stroke_mm:g}mm-{dt_us:g}us-{control}.npy"
    np.save(trace, np.asarray(history))
    result = dict(
        side=side,
        control=control,
        sourceState=previous["state"],
        piece=previous["piece"],
        sourceTimeS=begin,
        startWallS=previous["endWallS"],
        endWallS=previous["endWallS"] + float(t + dt),
        initialProjectionRollMm=1000 * rail.roll,
        initialProjectionRollVelocityMmS=1000 * rail.roll_rate,
        dtUs=dt_us,
        commandMm=stroke_mm,
        firstZeroFloorReactionS=first_free,
        longestZeroFloorMs=max_gap_ms,
        minimumClosedBoreMm=minimum,
        maximumPinN=max_pin,
        minimumOpenPinClearanceMm=None if not math.isfinite(min_open) else min_open,
        legacyPinCounterexample=prior_pin_limit,
        peakRailAndPlungerN=rail.peak.tolist(),
        peakAbsPitchRad=rail.max_pitch,
        peakAbsHeaveMm=rail.max_heave * 1000,
        openCommandS=issued,
        lastStrokeMm=travel[i + 1] * 1000,
        outcome=reason,
        terminalLocalP=pipe.p.tolist(),
        terminalAxis=pipe.a.tolist(),
        terminalHardStop=pipe.hard,
        solverWallSeconds=float(t + dt),
        event=event,
        trace=digest(trace),
        matchCredit=0,
    )
    trace.with_suffix(".json").write_bytes(
        (json.dumps(result, indent=2, allow_nan=False) + "\n").encode()
    )
    print(result, flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("side", choices=("plus", "minus"))
    p.add_argument("--stroke-mm", type=float, default=337.0)
    p.add_argument("--dt-us", type=float, default=31.25)
    p.add_argument("--until", type=float, default=4.0)
    p.add_argument(
        "--control",
        choices=("nominal", "legacy-bore", "zero-traction", "stuck-yoke"),
        default="nominal",
    )
    a = p.parse_args()
    run(a.artifacts, a.side, a.stroke_mm, a.dt_us, a.control, a.until)
