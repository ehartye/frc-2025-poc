"""Coupled receiver and two free pipe coordinates with compression-only contact."""

from loaded_rack import SIX, digest, read
import argparse
import json
from pathlib import Path
import numpy as np
from dynamics import G, contact_rows, matrices, profile
import dynamics_run as rack
from dynamics_run import equilibrium

HALF = 0.1508125
RO = 0.05715
RI = 0.0508


def solve(state, dt, duration, skin, kick=0.0, rail_gap=0.00035, cam=False):
    geo, keeper, _ = rack.registration()
    M, K, C, grav, pitch, basis, weights, _ = matrices(state)
    mass = read(SIX / "extension-support/balance.json")["pipeMassKg"]
    n = len(grav)
    # Remove precisely the tied Gauss-point mass already present in the earlier model.
    M -= (mass * weights[:, None] * basis).T @ basis
    grav += G * mass * (weights @ basis)
    pitch[5] -= mass * (state["p"][2] - 0.467)
    M, K, C = (np.pad(a, [(0, 2), (0, 2)]) for a in (M, K, C))
    grav, pitch = np.r_[grav, -mass * G, 0.0], np.r_[pitch, 0.0, 0.0]
    M[n, n] = mass
    M[n + 1, n + 1] = mass * (3 * (RO**2 + RI**2) + (2 * HALF) ** 2) / 12
    if np.linalg.eigvalsh(M).min() <= 0:
        raise ValueError("Inconsistent free-body mass matrix")
    # Sign convention: a positive gap separates the tube from the shelf.
    nodes = np.polynomial.legendre.leggauss(len(weights))[0] * HALF
    pipe = np.column_stack((-basis, np.ones(len(nodes)), nodes))
    settle = 0.25
    t = np.arange(round((duration + settle + 0.12) / dt) + 1) * dt
    s, speed, acc = profile(t - settle, duration)
    rails, dr = contact_rows(s, geo, keeper)
    gaps = np.array([0.001, 0.001, rail_gap] * 2)
    ks = np.full(6, 1e7)
    r = np.zeros((8 if cam else 6, n + 2))
    r[:6, :6] = rails[0]
    base_m, _, _, imposed_g, _, _, _, _ = matrices(state)
    q0 = equilibrium(base_m, K[:n, :n], imposed_g, r[:6, :n], gaps, ks)
    tilted = nodes * state["a"][2]
    q = np.r_[q0, (basis @ q0 - tilted).max(), state["a"][2]]
    v = np.zeros(n + 2)
    inv = np.linalg.inv(M + 0.5 * dt * C + 0.25 * dt**2 * K)
    force_map = dt * inv
    V = inv @ (M - 0.5 * dt * C - 0.25 * dt**2 * K)
    D = -dt * inv @ K
    cd = 0.12 * np.sqrt(ks / np.einsum("ij,jk,ik->i", r[:6], np.linalg.inv(M), r[:6]))
    if cam:
        from cam_model import candidate_gaps
        from dynamics_run import table

        cap = table()
        grid = np.arange(len(cap)) * 0.00025
        cap_gaps = candidate_gaps(grid)
        cap_gaps[~np.isfinite(cap_gaps)] = 0.0031
        cap_indices = np.clip(np.rint(s / 0.00025).astype(int), 0, len(grid) - 1)
        cap_clear = candidate_gaps(s)
        plungers = np.maximum(0, -cap_clear[0] - cap[0] @ q[:n])
        last_plungers = plungers.copy()
        cd = np.r_[cd, 1500.0, 1500.0]
        ks = np.r_[ks, 0.0, 0.0]
    max_cap_force = 0.0
    fixed = dt * 0.25 * ks + 0.5 * cd
    gravity = force_map @ grav
    drive = force_map @ pitch
    min_force = np.inf
    max_gap = peakN = 0.0
    limits = []
    history = []
    stride = round(0.001 / dt)
    previously_free = False
    last_minute = []
    for i, clock in enumerate(t):
        if i == round(settle / dt):
            v[n] += kick
        r[:6, :6] = rails[i]
        clearance = pipe @ q
        rate = pipe @ v
        # Original shelf's 400 kN/m distributed spring and 4 s/m dissipation;
        # no tensile reaction, even as the contact rebounds. Weights sum to one.
        pf = skin * weights * np.maximum(0, -clearance) * np.maximum(0, 1.0 - 4 * rate)
        vf = V @ v + D @ q + gravity + drive * acc[i] + force_map @ (pipe.T @ pf)
        rel = dr[i] @ q[:6] * speed[i]
        gap = r[:6] @ q + gaps
        if cam:
            k = cap_indices[i]
            r[6:, :n] = cap[k]
            lo, hi = max(0, k - 1), min(k + 1, len(grid) - 1)
            slope = ((cap[hi] - cap[lo]) @ q[:n] + cap_gaps[hi] - cap_gaps[lo]) / (
                grid[hi] - grid[lo]
            )
            rel = np.r_[rel, slope * speed[i]]
            gap = np.r_[gap, r[6:] @ q + cap_clear[i]]
            plungers = np.maximum(0, plungers - dt * 0.008 / 0.3)
        old = r @ v + rel
        free = r @ vf + rel
        threshold = np.r_[np.zeros(6), plungers] if cam else 0.0
        active = np.flatnonzero(-(gap + 0.25 * dt * (old + free)) >= threshold)
        forces = np.zeros(len(r))
        rhs = -(0.5 * dt * ks + cd) * (0.5 * (old + free))
        rhs[:6] -= ks[:6] * gap[:6]
        # Midpoint effective compliance; inactive rails cannot pull the next stage.
        while len(active):
            R = r[active]
            system = np.eye(len(active)) + (fixed[active, None] * R) @ force_map @ R.T
            u = np.linalg.solve(
                system, rhs[active] - fixed[active] * (R @ (force_map @ (r.T @ forces)))
            )
            if cam and np.any((active >= 6) & (u > 750.0)):
                j = np.flatnonzero((active >= 6) & (u > 750.0))[0]
                forces[active[j]] = 750.0
                active = np.delete(active, j)
                continue
            if min(u) >= 0:
                forces[active] = u
                break
            active = np.delete(active, np.argmin(u))
        vn = vf + force_map @ (r.T @ forces)
        if cam:
            plungers = np.maximum(plungers, -gap[6:])
            last_plungers = np.maximum(last_plungers, plungers)
            max_cap_force = max(max_cap_force, float(max(forces[6:])))
        normal = float(pf.sum())
        gap_mm = float(clearance.min() * 1000)
        sec = clock - settle
        if sec >= 0:
            free_now = gap_mm > 0
            if free_now != previously_free:
                limits.append(
                    {
                        "event": "all-stations-clear"
                        if free_now
                        else "first-recontact",
                        "timeS": float(sec),
                        "strokeMm": float(s[i] * 1000),
                    }
                )
            previously_free = free_now
            min_force = min(min_force, normal)
            peakN = max(peakN, normal)
            max_gap = max(max_gap, gap_mm)
            if i % stride == 0:
                history.append(
                    [sec, s[i] * 1000, gap_mm, normal, 1000 * q[n], float(q[n + 1])]
                )
        if -0.02 <= sec <= 0:
            last_minute.append(normal)
        if max(abs(q[[1, 3, 5, n + 1]])) > 0.1 or not np.isfinite(q).all():
            raise ValueError(f"Exceeded the small-angle model at {sec}")
        q += 0.5 * dt * (v + vn)
        v = vn
    return dict(
        durationS=duration,
        stepUs=dt * 1e6,
        shelfStiffnessNPerM=skin,
        verticalKickMps=kick,
        railUpperGapMm=rail_gap * 1000,
        signedDeclaredPipeMassKg=mass,
        preStrokeReactionRangeN=[min(last_minute), max(last_minute)],
        weightN=mass * G,
        minNormalN=min_force,
        peakNormalN=peakN,
        peakCompleteSeparationMm=max_gap,
        events=limits,
        terminalNormalN=normal,
        terminalMinGapMm=gap_mm,
        capMode="continuous lands" if cam else "withdrawn",
        peakCapForceN=max_cap_force,
        largestCapCompressionMm=1000 * float(max(last_plungers)) if cam else None,
        trace=history,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("station", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("sign", choices=("plus", "minus"))
    p.add_argument("dt_us", type=float)
    p.add_argument("--duration", type=float, default=3.0)
    p.add_argument("--stiffness", type=float, default=400000.0)
    p.add_argument(
        "--kick",
        type=float,
        default=0.0,
        help="Vertical pipe speed increment at stroke start, detector control only",
    )
    p.add_argument(
        "--cam",
        action="store_true",
        help="Use the machined cam and its independently propagated first-service state",
    )
    a = p.parse_args()
    if a.cam:
        from cam_model import activate_rack

        state_dir = f"cam-station-{a.sign}-31.25us-0.1mm"
        receipt = read(a.station / (state_dir + "-service.json"))
        candidate = activate_rack()
        if receipt["candidateStepSha256"] != candidate["candidateStepSha256"]:
            raise ValueError("Service input is from another cam")
        path = a.station / state_dir / "first-service" / f"{a.sign}-31.25us-service.npz"
    else:
        path = a.station / (a.sign + "-15.625us") / (a.sign + "-15.625us-service.npz")
        receipt = read(path.parent / "service.json")
    if receipt["outgoing"] != digest(path):
        raise ValueError("Changed signed input")
    with np.load(path) as source:
        state = {k: source[k] for k in ("p", "a", "z", "zdot")}
    result = solve(state, a.dt_us * 1e-6, a.duration, a.stiffness, a.kick, cam=a.cam)
    result["stateSha256"] = digest(path)
    history = result.pop("trace")
    name = f"{a.sign}-free-{a.dt_us:g}us-{a.duration:g}s-{a.stiffness:g}"
    if a.kick:
        name += f"-kick-{a.kick:g}"
    if a.cam:
        name += "-cam"
        result["candidateStepSha256"] = candidate["candidateStepSha256"]
    path = a.out / (name + ".npz")
    np.savez_compressed(path, samples=np.array(history))
    result["traceSha256"] = digest(path)
    (a.out / (name + ".json")).write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf8"
    )
