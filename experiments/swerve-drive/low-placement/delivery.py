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
from target_contact import Trough
from trough import forward_limit
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


def run(
    artifacts,
    side,
    stroke_mm,
    dt_us,
    control,
    end_time,
    resume=False,
    field=False,
    held_out=False,
):
    if held_out and side != "minus":
        raise ValueError("The registered held-out station uses minus calibration")
    gap_mm = 0.2 if held_out else 0.1
    out = (
        artifacts
        / "low-placement"
        / ("heldout-advance-25" if held_out else "advance-25")
        / side
    )
    src = artifacts / "r4-score-stroke"
    name = f"cam-station-{side}-31.25us-{gap_mm:g}mm"
    feed_input = read(src / (name + ".json"))
    if not math.isclose(feed_input.get("capGapMm", 0.1), gap_mm):
        raise ValueError("Different physical station plunger setting")
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
    if field:
        retract = times >= 4.0
        sr, vr, ar = dynamics.profile(times[retract] - 4.0, 3.0)
        travel[retract] = (0.37 - sr) * stroke_mm / 370
        speed[retract] = -vr * stroke_mm / 370
        acc[retract] = -ar * stroke_mm / 370
    if control == "stalled-rack":
        travel[:] = 0.0
        speed[:] = 0.0
        acc[:] = 0.0
    raw = {
        key: getattr(pipe, key).copy()
        for key in ("a", "p", "v", "l", "z", "zdot", "plunger")
    }
    rail = RailContact(raw, dt, travel, speed)
    rail.clear = rail.clear + (gap_mm - 0.1) / 1000
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
    target_path = HERE.parents[1] / "six-wheel/mission/target.json"
    dock = read(out / "chassis.json")["terminalMmMmRad"]
    target = Trough(read(target_path), dock, 17 if dt_us <= 31.25 else 9)
    idle_front = forward_limit(0.0)
    full_front = forward_limit(stroke_mm)
    first_field = first_dual = None
    peak_field = 0.0
    dwell = 0.0
    longest_dwell = 0.0
    port_clear_ticks = 0
    withdrawal = None
    min_open = math.inf
    prior_pin_limit = None
    start_index = 0
    checkpoint = out / f"extended-{stroke_mm:g}mm-{dt_us:g}us.npz"
    if resume:
        with np.load(checkpoint, allow_pickle=False) as z:
            if (
                str(z["source"]) != previous["state"]
                or str(z["code"]) != digest(Path(__file__))
                or str(z["railCode"]) != digest(HERE / "rail_contact.py")
            ):
                raise ValueError("Wrong stopped pipe or extension source")
            start_index = int(z["index"])
            if abs(times[start_index] - 3.1) > 1e-9:
                raise ValueError("Restart is not the release-service boundary")
            rail.q, rail.v, rail.plunger, rail.peak = [
                z[k].copy() for k in ("q", "qd", "plunger", "peak")
            ]
            rail.max_pitch, rail.max_heave = z["peakMotion"]
            world = tuple(z["world"].copy())
            pipe.time, pipe.belt = z["pipeClock"]
            history = z["history"].tolist()
            permits = z["permits"].astype(bool).tolist()
            max_pin, max_gap_ms, minimum, lost_s = z["bounds"]
            first_free = None if np.isnan(z["firstFree"]) else float(z["firstFree"])
            f = frame(rail, travel[start_index], speed[start_index])
            local(pipe, f, world, feed)
            sync()
    for i in range(start_index, len(times) - 1):
        t = times[i]
        if (
            not resume
            and i == round(3.1 / dt)
            and all(permits[-5:])
            and control == "nominal"
        ):
            np.savez_compressed(
                checkpoint,
                source=previous["state"],
                code=digest(Path(__file__)),
                railCode=digest(HERE / "rail_contact.py"),
                index=i,
                q=rail.q,
                qd=rail.v,
                plunger=rail.plunger,
                peak=rail.peak,
                peakMotion=[rail.max_pitch, rail.max_heave],
                world=np.array(world),
                pipeClock=[pipe.time, pipe.belt],
                history=history,
                permits=permits,
                bounds=[max_pin, max_gap_ms, minimum, lost_s],
                firstFree=np.nan if first_free is None else first_free,
            )
        # Require at least five fresh 5 ms held states at the arrived stroke
        # before opening; a stuck yoke keeps the pusher disabled.
        read_stroke = (
            round((travel[i] * 1000 + (0.2 if side == "plus" else -0.2)) / 0.1) * 0.1
        )
        at_end = abs(read_stroke - stroke_mm) <= 0.5 and abs(speed[i]) < 0.001
        if (
            t >= 3.1
            and issued is None
            and at_end
            and all(permits[-5:])
            and len(permits) >= 5
        ):
            issued = float(t)
        if issued is None and t >= 3.25:
            reason = "stroke service expired before measured dock extension"
            break
        if field and t >= 4.0 and withdrawal is None:
            if port_clear_ticks < 5 or issued is None or control == "stuck-yoke":
                reason = "withdrawal inhibited by measured occupancy or yoke state"
                break
            withdrawal = float(t)
        yoke = (
            max(0.0, 35.0 - 150.0 * (t - issued))
            if issued is not None and control != "stuck-yoke"
            else 35.0
        )
        # The open position is a modeled servo completion indication. The as-issued
        # closed switches alone cannot identify a partially retracted jam.
        if control == "backseat" and issued is not None:
            pulse = t - issued
            belt = -0.14 if 0.235 <= pulse < 0.335 else 1.05 if pulse >= 0.375 else 0.0
        elif control == "soft-start" and issued is not None and yoke < 0.001:
            belt = min(0.88, 3.0 * max(0, t - issued - 35.0 / 150.0))
        else:
            belt = 1.05 if issued is not None and yoke < 0.001 else 0.0
        if control == "stuck-yoke" and issued is not None and t - issued >= 0.275:
            reason = "open-yoke service expired with belt disabled"
            break
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
        external, external_torque, spans, normals, compression = (
            target.solve(world)
            if field
            else (np.zeros(3), np.zeros(3), [0.0, 0.0], [0.0, 0.0], 0.0)
        )
        pipe.v += dt * (r.T @ external) / feed.M
        pipe.l += dt * (r.T @ external_torque)
        pipe.v += dt * (actual_gravity - np.array([0.0, 0.0, -feed.G]))
        pipe.plate_force = 0.0
        feed.Pipe.moves(pipe, belt, dt)
        force = feed.M * ((pipe.v - v0) / dt - actual_gravity) - r.T @ external
        torque = (pipe.l - l0) / dt - r.T @ external_torque
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
        rear = P[0] - 0.1508125 * abs(A[0]) - 0.05715 * np.sqrt(max(0.0, 1 - A[0] ** 2))
        nose = idle_front + (full_front - idle_front) * travel[i + 1] * 1000 / stroke_mm
        front_gap = rear - nose
        if sum(normals) > 0:
            if first_field is None:
                first_field = float(t + dt)
            if pipe.last_normal_n > 0 and first_dual is None:
                first_dual = float(t + dt)
        peak_field = max(peak_field, compression)
        if (
            min(spans) >= 0.18
            and np.linalg.norm(V) < 0.01
            and np.linalg.norm(omega) < 0.2
            and front_gap > 0.001
        ):
            dwell += dt
        else:
            dwell = 0.0
        longest_dwell = max(longest_dwell, dwell)
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
            port_clear_ticks = port_clear_ticks + 1 if not any(ports) else 0
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
                    *spans,
                    *normals,
                    front_gap,
                    dwell,
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
        if field and target.state(world)[0][2] < 0.20:
            reason = "missed trough after losing field support"
            break
        if field and dwell >= 0.25:
            reason = (
                "finite trough support and whole-robot separation persisted for 250 ms"
            )
            break
        if (
            not field
            and issued is not None
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
    prefix = "joined" if field else "coupled"
    trace = out / f"{prefix}-{stroke_mm:g}mm-{dt_us:g}us-{control}.npy"
    np.save(trace, np.asarray(history))
    result = dict(
        side=side,
        control=control,
        sourceState=previous["state"],
        piece=previous["piece"],
        sourceTimeS=begin,
        stationReceiptSha256=digest(src / (name + ".json")),
        physicalGapMm=gap_mm,
        startWallS=previous["endWallS"],
        endWallS=previous["endWallS"] + pipe.time - begin,
        initialProjectionRollMm=1000 * rail.roll,
        initialProjectionRollVelocityMmS=1000 * rail.roll_rate,
        dtUs=dt_us,
        commandMm=stroke_mm,
        firstZeroFloorReactionS=first_free,
        troughSourceSha256=digest(target_path) if field else None,
        firstTargetContactS=first_field,
        firstSimultaneousFieldShelfContactS=first_dual,
        maximumFieldIndentationMm=peak_field * 1000,
        longestSeparatedSupportS=longest_dwell,
        withdrawalCommandS=withdrawal,
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
        solverWallSeconds=pipe.time - begin,
        extensionRestartSha256=digest(checkpoint) if resume else None,
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
    p.add_argument("--from-extension", action="store_true")
    p.add_argument("--field", action="store_true")
    p.add_argument("--held-out", action="store_true")
    p.add_argument(
        "--control",
        choices=(
            "nominal",
            "legacy-bore",
            "zero-traction",
            "stuck-yoke",
            "stalled-rack",
            "backseat",
            "soft-start",
        ),
        default="nominal",
    )
    a = p.parse_args()
    run(
        a.artifacts,
        a.side,
        a.stroke_mm,
        a.dt_us,
        a.control,
        a.until,
        a.from_extension,
        a.field,
        a.held_out,
    )
