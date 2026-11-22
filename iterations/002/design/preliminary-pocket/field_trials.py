"""Free-piece field trials for the B'x pocket, layout-002: station receive, drive-away, carry sled and eject into the L1 trough.

    python -I -B field_trials.py OUTPUT.json [--processes 8]

Planar (XZ) rigid pipe with spin against the pocket_geometry.py surfaces, in the robot frame. Every contact is a
compliant penalty contact: normal spring-damper with damping set from a restitution e, and stick-slip Coulomb
friction through a tangential spring capped at mu x normal force, so a wedged or resting pipe can hold statically.

Drive (layout-002): one NEO 550 (REV-21-1651: Kt 0.0097 N m/A, 11000 rpm free at 12 V, 30 A limit) through a REV
MAXPlanetary 4:1 x 3:1 x 3:1 (36:1, back-drivable, 0.9 per stage assumed) on the lower shaft, belted 1:1 to the upper.
The tread contact reaction is fed back into the shaft speed. 'hold' is the powered position hold used on receive and
carry (torque-limited at the same 30 A); busV None is the disabled robot (rollers free to back-drive through the
reflected motor inertia). The layout-001 self-locking worm (30:1, 40 %) is kept only as the comparison drive.

Receive starts at rest (or pushed) on the registered station chute (35 deg from horizontal, 419.1 mm ramp, exit sill
952.5 mm above the carpet), drops onto the robot inlet and runs to the nip. The released pipe continues through the
trough section (target.json). When it comes to rest the robot withdraws 120 mm at 0.3 m/s; the trough is assessed 2 s
after the pipe is again at rest, as the scoring observation.

Not modelled: yaw and pipe-end contact, ovality and wear, the 3D trough panel holes, the sector divider, peers, tread
hysteresis, lip and hood compliance (rigid), the station opening's upper guard, and measured e, mu, tread stiffness or
motor inertia (all swept or estimated).
"""
import json
import math
import sys
from itertools import product
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pocket_geometry as g  # noqa: E402

G = 9.80665
R = g.R / 1e3
M = g.PIPE_MASS
K = (g.R ** 2 + g.PIPE_RI ** 2) / (2 * g.R ** 2)
I = K * M * R * R
DT_DEFAULT = 4e-5
K_RIGID = 2.0e5                 # N/m: polycarbonate panel on its supports (about 0.3 mm under the pipe at 8 g)
KT_DEFAULT = 2.0e4              # N/m tangential (stick) stiffness
MOTOR = dict(Kt=0.0097, Rm=0.12, Ke=12.0 / (11000 * 2 * math.pi / 60), Jm=1.5e-6)   # Jm: estimate, unsourced
J_ROLLERS = 8.0e-5              # two shafts and eight 2 in wheels about their axes (estimate)
WORM = dict(kind="worm", N=30.0, eta=0.40, ilim=30.0)                 # layout-001 comparison drive
MAX36 = dict(kind="planetary", N=36.0, eta=0.729, ilim=30.0)           # selected: MAXPlanetary 4:1 x 3:1 x 3:1
HOLD_RAD = 0.035                # position hold reaches its torque limit at 2 deg of shaft error
# Registered station (red coral station at the dock): V0 manual 5.6 sill 37.5 in; chute 55 deg from vertical; GE-25014
# sheet 80 ramp 16.50 in. The exit sill sits on the station wall's field face.
STATION = dict(sillZ=952.5, slopeDeg=35.0, rampMm=419.1, rearBumperFaceX=g.FRAME_REAR - (g.BUMPER_FACE - g.FRAME_FRONT))
NOMINAL_DOCK_GAP = 10.0         # rear bumper face to station wall (field-registration.json stand-off class)


def m(p):
    return (p[0] / 1e3, p[1] / 1e3)


def damping(k, e):
    e = min(max(e, 1e-3), 0.999)
    z = -math.log(e) / math.sqrt(math.pi ** 2 + math.log(e) ** 2)
    return 2 * z * math.sqrt(k * M)


def rot_xz(q, o, deg):
    a = math.radians(deg)
    x, z = q[0] - o[0], q[1] - o[1]
    return (o[0] + x * math.cos(a) + z * math.sin(a), o[1] - x * math.sin(a) + z * math.cos(a))


# ---------------------------------------------------------------- scene
def station_chute(gap):
    """Station chute polyline in the robot frame (mm): top of the ramp to the exit sill."""
    a = math.radians(STATION["slopeDeg"])
    lip = (STATION["rearBumperFaceX"] - gap, STATION["sillZ"])
    return [(lip[0] - STATION["rampMm"] * math.cos(a), lip[1] + STATION["rampMm"] * math.sin(a)), lip]


def scene(p, lip="deployed", flap=True, hood=True, standoff=None, ramp_deg=g.LIP_RAMP_DEG, dock=None, lip_proj=None):
    robot, field = [], []
    pc = dict(k=K_RIGID, c=damping(K_RIGID, p["e"]), mu=p["mu"])
    lc = dict(pc, lip=True)
    if dock is not None:
        ch = station_chute(dock)
        robot.append((m(ch[0]), m(ch[1]), pc))
    for a, b in zip(g.INLET, g.INLET[1:]):
        robot.append((m(a), m(b), pc))
    if hood:
        robot.append((m(g.HOOD[0]), m(g.HOOD[1]), pc))
        if p.get("flare", True):
            robot.append((m(g.HOOD_FLARE[0]), m(g.HOOD_FLARE[1]), pc))
    if lip == "deployed":
        pr = g.lip_profile(ramp_deg) if lip_proj is None else short_lip(lip_proj)
        for a, b in zip(pr, pr[1:]):
            robot.append((m(a), m(b), lc))
    elif lip == "stowed":
        # latch failed: the lip stands stowed (folded up in front of the nip)
        pr = [rot_xz(q, g.LIP_HINGE, g.LIP_STOW_DEG) for q in g.lip_profile()]
        for a, b in zip(pr, pr[1:]):
            robot.append((m(a), m(b), lc))
    if standoff is not None:
        t = g.TROUGH
        xl = g.trough_x(standoff)
        xr = xl + t["riserX"]
        fl = lambda x: t["floor0"] + t["slope"] * (x - xl)
        ld = t["lipDepth"]
        for a, b in [((xl, 300.0), (xl, t["lipTop"])), ((xl, t["lipTop"]), (xl + ld, t["lipTop"])),
                     ((xl + ld, fl(xl + ld)), (xl + ld, t["lipTop"])), ((xl + ld, fl(xl + ld)), (xr, fl(xr))),
                     ((xr, fl(xr)), (xr, t["riserTop"])), ((xr, t["riserTop"]), (xr + t["riserT"], t["riserTop"]))]:
            field.append((m(a), m(b), pc))
    kt = p["kTread"]
    tread = dict(k=kt, c=damping(kt, p["eTread"]), mu=p["muTread"])
    dz = (g.R + g.ROLLER_R - p.get("squeeze", g.SQUEEZE)) / 1e3
    rollers = [dict(centre=(g.THROAT_X / 1e3, g.POCKET_Z / 1e3 - dz), r=g.ROLLER_R / 1e3, sign=-1.0, **tread),
               dict(centre=(g.THROAT_X / 1e3, g.POCKET_Z / 1e3 + dz), r=g.ROLLER_R / 1e3, sign=1.0, **tread)]
    flapd = None
    if flap:
        flapd = dict(h=m(g.FLAP_HINGE), L=g.FLAP_LENGTH / 1e3, I=p.get("flapI", 2.4e-4), tau0=p.get("flapTau0", 0.04),
                     kf=p.get("flapK", 0.06), cf=p.get("flapC", 2e-4), maxDeg=g.FLAP_OPEN_DEG, eStop=0.3, **pc)
    return robot, field, rollers, flapd


def short_lip(proj):
    """Control lip: same ramp angle and land, tip only proj mm beyond the frame line."""
    tip = (g.FRAME_FRONT + proj, g.LIP_TIP_SURFACE_Z)
    knee = (tip[0] - g.LIP_LAND, tip[1])
    return [(g.LIP_ROOT_X, knee[1] + (knee[0] - g.LIP_ROOT_X) * math.tan(math.radians(g.LIP_RAMP_DEG))), knee, tip]


def closest(px, pz, a, b):
    ax, az = a
    dx, dz = b[0] - ax, b[1] - az
    L2 = dx * dx + dz * dz
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (pz - az) * dz) / L2))
    return ax + u * dx, az + u * dz


def contact(key, tang, st, q, n, delta, props, vsurf, dt, kt):
    """Penalty normal force and stick-slip friction on the pipe at q (normal n into the pipe)."""
    x, z, vx, vz, w = st
    rx, rz = q[0] - x, q[1] - z
    relx, relz = vx + w * -rz - vsurf[0], vz + w * rx - vsurf[1]
    vn = relx * n[0] + relz * n[1]
    fn = props["k"] * delta - props["c"] * vn
    if fn <= 0:
        tang.pop(key, None)
        return 0.0, 0.0, 0.0, 0.0, 0.0
    tx, tz = -n[1], n[0]
    vt = relx * tx + relz * tz
    s = tang.get(key, 0.0) + vt * dt
    ct = math.sqrt(kt * M)
    ft = -kt * s - ct * vt
    cap = props["mu"] * fn
    if abs(ft) > cap:
        ft = math.copysign(cap, ft)
        s = -ft / kt
    tang[key] = s
    fx, fz = fn * n[0] + ft * tx, fn * n[1] + ft * tz
    return fx, fz, rx * fz - rz * fx, fn, ft


def simulate(state, p, sc, tmax, accel=None, stop=None, record=0.002, other=None):
    """p['rollers']: 'locked' (worm, unpowered), 'free', 'driven' (imposed speed p['nip'](t)), or 'motor'
    (p['drive'] reducer, p['busV'] volts, None for unpowered, 0 for shorted brake)."""
    robot, field, rollers, flapd = sc
    dt = p.get("dt", DT_DEFAULT)
    kt = p.get("kT", KT_DEFAULT)
    x, z, vx, vz, w = state
    phi, dphi = 0.0, 0.0
    ox, vox = 0.0, 0.0
    ws = 0.0
    th = 0.0
    o2 = list(other) if other else None      # second free pipe (already in the trough): field and pipe-pipe contact only
    hinge = m(g.LIP_HINGE)
    drive = p.get("drive")
    J = (MOTOR["Jm"] * drive["N"] ** 2 + J_ROLLERS) if drive else J_ROLLERS
    t = 0.0
    tang = {}
    trace, nxt = [], 0.0
    aux = dict(maxBiteN=0.0, maxShaftDemandNm=0.0, maxBackDriveNm=0.0, maxFlapDeg=0.0, minNipMps=None, maxCurrentA=0.0, phi=0.0,
               maxLipN=0.0, maxLipMomentNm=0.0, maxHoldDeg=0.0, maxRollerN=0.0, other=None)
    mode = p["rollers"]
    while t < tmax:
        fx, fz, tau = 0.0, -M * G, 0.0
        if accel:
            fx += -M * accel(t)
        st = (x, z, vx, vz, w)
        sr = (x - ox, z, vx - vox, vz, w)
        for i, (a, b, props) in enumerate(robot):
            qx, qz = closest(sr[0], z, a, b)
            dx, dz = sr[0] - qx, z - qz
            d = math.hypot(dx, dz)
            if d >= R or d == 0:
                tang.pop(("r", i), None)
                continue
            f = contact(("r", i), tang, sr, (qx, qz), (dx / d, dz / d), R - d, props, (0.0, 0.0), dt, kt)
            fx += f[0]; fz += f[1]; tau += f[2]
            if props.get("lip"):
                aux["maxLipN"] = max(aux["maxLipN"], math.hypot(f[0], f[1]))
                # moment about the hinge from the pipe's reaction (-f at the contact point)
                aux["maxLipMomentNm"] = max(aux["maxLipMomentNm"], abs((qz - hinge[1]) * -f[0] - (qx - hinge[0]) * -f[1]))
        if o2:
            f2x, f2z, t2 = 0.0, -M * G, 0.0
            s2 = tuple(o2)
            for i, (a, b, props) in enumerate(field):
                qx, qz = closest(s2[0], s2[1], a, b)
                dx, dz = s2[0] - qx, s2[1] - qz
                d = math.hypot(dx, dz)
                if d >= R or d == 0:
                    tang.pop(("f2", i), None)
                    continue
                f = contact(("f2", i), tang, s2, (qx, qz), (dx / d, dz / d), R - d, props, (0.0, 0.0), dt, kt)
                f2x += f[0]; f2z += f[1]; t2 += f[2]
            dx, dz = x - s2[0], z - s2[1]
            d = math.hypot(dx, dz)
            if 0 < d < 2 * R:
                n = (dx / d, dz / d)
                q = (s2[0] + n[0] * R, s2[1] + n[1] * R)
                vq = (s2[2] - s2[4] * (q[1] - s2[1]), s2[3] + s2[4] * (q[0] - s2[0]))
                f = contact(("pp", 0), tang, st, q, n, (2 * R - d) / 2, field[0][2], vq, dt, kt)
                fx += f[0]; fz += f[1]; tau += f[2]
                f2x -= f[0]; f2z -= f[1]; t2 -= (q[0] - s2[0]) * f[1] - (q[1] - s2[1]) * f[0]
            else:
                tang.pop(("pp", 0), None)
            o2[2] += f2x / M * dt; o2[3] += f2z / M * dt; o2[4] += t2 / I * dt
            o2[0] += o2[2] * dt; o2[1] += o2[3] * dt
            aux["other"] = tuple(o2)
        for i, (a, b, props) in enumerate(field):
            qx, qz = closest(x, z, a, b)
            dx, dz = x - qx, z - qz
            d = math.hypot(dx, dz)
            if d >= R or d == 0:
                tang.pop(("f", i), None)
                continue
            f = contact(("f", i), tang, st, (qx, qz), (dx / d, dz / d), R - d, props, (0.0, 0.0), dt, kt)
            fx += f[0]; fz += f[1]; tau += f[2]
        speed = p["nip"](t) if mode == "driven" else (ws if mode == "motor" else 0.0)
        bite, demand = 0.0, 0.0
        for i, rl in enumerate(rollers):
            cx, cz = rl["centre"]
            dx, dz = sr[0] - cx, z - cz
            d = math.hypot(dx, dz)
            delta = R + rl["r"] - d
            if delta <= 0:
                tang.pop(("n", i), None)
                continue
            n = (dx / d, dz / d)
            q = (cx + n[0] * rl["r"], cz + n[1] * rl["r"])
            props = dict(rl)
            if mode == "free":
                props["mu"] = 0.0
            ux, uz = rl["sign"] * -(q[1] - cz) / rl["r"], rl["sign"] * (q[0] - cx) / rl["r"]
            f = contact(("n", i), tang, sr, q, n, delta, props, (ux * speed, uz * speed), dt, kt)
            fx += f[0]; fz += f[1]; tau += f[2]
            bite += f[3]
            aux["maxRollerN"] = max(aux["maxRollerN"], f[3])
            demand += (f[0] * ux + f[1] * uz) * rl["r"]
        aux["maxBiteN"] = max(aux["maxBiteN"], bite)
        aux["maxShaftDemandNm"] = max(aux["maxShaftDemandNm"], demand)
        aux["maxBackDriveNm"] = max(aux["maxBackDriveNm"], -demand)
        if mode == "motor":
            V = p.get("busV")
            N, eta = drive["N"], drive["eta"]
            wm = ws / rollers[0]["r"] * N
            cur = 0.0 if V is None else max(-drive["ilim"], min(drive["ilim"], (V - MOTOR["Ke"] * wm) / MOTOR["Rm"]))
            aux["maxCurrentA"] = max(aux["maxCurrentA"], abs(cur))
            if drive["kind"] == "worm":
                if V is None:
                    ws = 0.0
                else:
                    tout = MOTOR["Kt"] * cur * N * eta
                    ws = max(0.0, ws + (tout - max(demand, 0.0)) / J * rollers[0]["r"] * dt)
            elif p.get("hold"):
                # powered position hold: PD on the shaft angle, saturating at the current limit
                th += ws / rollers[0]["r"] * dt
                tmax = MOTOR["Kt"] * drive["ilim"] * N * eta
                kp = tmax / HOLD_RAD
                tout = max(-tmax, min(tmax, -kp * th - 2.0 * math.sqrt(kp * J) * ws / rollers[0]["r"]))
                aux["maxCurrentA"] = max(aux["maxCurrentA"], abs(tout) / (MOTOR["Kt"] * N * eta))
                aux["maxHoldDeg"] = max(aux["maxHoldDeg"], abs(math.degrees(th)))
                ws += (tout - demand) / J * rollers[0]["r"] * dt
            else:
                tout = MOTOR["Kt"] * cur * N * eta
                ws += (tout - demand) / J * rollers[0]["r"] * dt
            if t > 0.02 and sr[0] * 1e3 < g.THROAT_X + 25:
                aux["minNipMps"] = ws if aux["minNipMps"] is None else min(aux["minNipMps"], ws)
        if flapd:
            hx, hz = flapd["h"]
            tipx, tipz = hx + flapd["L"] * math.sin(phi), hz - flapd["L"] * math.cos(phi)
            qx, qz = closest(sr[0], z, (hx, hz), (tipx, tipz))
            dx, dz = sr[0] - qx, z - qz
            d = math.hypot(dx, dz)
            tf = 0.0
            if 0 < d < R:
                s_ = math.hypot(qx - hx, qz - hz)
                vq = (dphi * s_ * math.cos(phi), dphi * s_ * math.sin(phi))
                f = contact(("flap", 0), tang, sr, (qx, qz), (dx / d, dz / d), R - d, flapd, vq, dt, kt)
                fx += f[0]; fz += f[1]; tau += f[2]
                tf = s_ * (-f[0] * math.cos(phi) - f[1] * math.sin(phi))
            else:
                tang.pop(("flap", 0), None)
            dphi += (tf - (flapd["tau0"] + flapd["kf"] * phi) - flapd["cf"] * dphi) / flapd["I"] * dt
            phi += dphi * dt
            if phi < 0.0:
                phi, dphi = 0.0, max(dphi, 0.0)
            pmax = math.radians(flapd["maxDeg"])
            if phi > pmax:
                phi, dphi = pmax, -flapd["eStop"] * max(dphi, 0.0)
            aux["maxFlapDeg"] = max(aux["maxFlapDeg"], math.degrees(phi))
        vx += fx / M * dt
        vz += fz / M * dt
        w += tau / I * dt
        x += vx * dt
        z += vz * dt
        if aux.get("withdrawAt") is not None:
            tw = t - aux["withdrawAt"]
            vox = -0.3 if 0 <= tw < 0.4 else 0.0
            ox += vox * dt
        t += dt
        aux["phi"] = phi
        if record and t >= nxt:
            row = [round(t, 4), round(x * 1e3, 2), round(z * 1e3, 2), round(w, 2), round(math.degrees(phi), 1),
                   round(ox * 1e3, 1), round(speed, 3)]
            if o2:
                row += [round(o2[0] * 1e3, 2), round(o2[1] * 1e3, 2)]
            trace.append(row)
            nxt += record
        if stop:
            out = stop(t, (x, z, vx, vz, w), aux)
            if out:
                return out, (x, z, vx, vz, w), t, trace, aux
    return "timeout", (x, z, vx, vz, w), t, trace, aux


NOMINAL = dict(e=0.2, mu=0.35, muTread=0.8, kTread=8.0e4, eTread=0.3, rollers="motor", drive=MAX36, busV=12.0, hold=True)
K_BAND = (4.0e4, 8.0e4, 1.6e5)  # trialled tread stiffness per roller: low, nominal, high (N/m)
ACCEPT_BAND = (40.0, 120.0)     # bench acceptance band, N/mm per roller at the 6 mm squeeze: inside the trialled span, and
                                # the steel shafts stay within a quarter of the squeeze at 120 (structure_checks.py S2)
SENSOR_LATENCY = 0.05           # s from flap closed to the drive-away command (G5 flags within 50 ms)


def P(**kw):
    d = dict(NOMINAL)
    d.update(kw)
    return d


def at_rest(st):
    return math.hypot(st[2], st[3]) < 0.005 and abs(st[4]) * R < 0.005


def settle(p, start, tmax=4.0):
    hold = dict(n=0.0)

    def stop(t, st, aux):
        hold["n"] = hold["n"] + p.get("dt", DT_DEFAULT) if at_rest(st) else 0.0
        return "settled" if t > 0.3 and hold["n"] > 0.3 else None
    out, st, t, _, _ = simulate(start, p, scene(p), tmax, stop=stop, record=0)
    return (st[0], st[1], 0.0, 0.0, 0.0), out


def dwell(p, seat, tdw=3.0):
    """Creep check: hold the seat with nothing applied and report the drift (mm)."""
    out, st, t, _, _ = simulate(seat, p, scene(p), tdw, record=0)
    return round(math.dist(seat[:2], st[:2]) * 1e3, 3)


def drive_label(p):
    if p["rollers"] != "motor":
        return p["rollers"]
    d = p["drive"]
    name = "MAXPlanetary 36:1" if d is MAX36 or d.get("N") == 36.0 else "worm 30:1"
    if p.get("busV") is None:
        return name + ", robot disabled"
    return name + (", powered hold" if p.get("hold") else f", {p['busV']:g} V")


def common(p):
    return dict(e=p["e"], mu=p["mu"], muTread=p["muTread"], kTreadNmm=p["kTread"] / 1e3, squeezeMm=p.get("squeeze", g.SQUEEZE),
                kT=p.get("kT", KT_DEFAULT), drive=drive_label(p), busV=p.get("busV"))


# ---------------------------------------------------------------- trials
def eject_trial(args):
    label, p, s, lip, opt, seat = args
    ramp_deg = opt.get("rampDeg", g.LIP_RAMP_DEG)
    proj = opt.get("lipProjMm")
    tip_x = g.LIP_TIP_X if proj is None else g.FRAME_FRONT + proj
    sc = scene(p, lip=lip, standoff=s, ramp_deg=ramp_deg, lip_proj=proj)
    t = g.TROUGH
    xl = g.trough_x(s) / 1e3
    xr = xl + t["riserX"] / 1e3
    first = opt.get("firstPipeFromLipMm")       # resting first pipe, [x from the trough lip face, z], mm
    other = None if first is None else (xl + first[0] / 1e3, first[1] / 1e3, 0.0, 0.0, 0.0)
    st_ = dict(rest=None, dep=None, phase="eject")

    def in_trough(x, z):
        return xl < x < xr and z < t["riserTop"] / 1e3 + 2 * R

    def stop(tt, st, aux):
        x, z, vx, vz, w = st
        if st_["dep"] is None and x * 1e3 > tip_x - 0.5 and lip == "deployed":
            st_["dep"] = dict(t=round(tt, 4), x=round(x * 1e3, 2), z=round(z * 1e3, 2), speed=round(math.hypot(vx, vz), 4), spin=round(w, 3))
        if x > xr + 0.004 and z > t["riserTop"] / 1e3 and (other is None or x > xr + R):
            return "lost over riser"
        if z < 0.38:
            return "fell short / out over lip" if x < xl + t["lipDepth"] / 1e3 + 0.02 else "fell"
        o = aux.get("other")
        if o is not None and (o[1] < 0.38 or not xl - R < o[0] < xr + R):
            return "first pipe knocked out of the trough"
        still = at_rest(st) and (o is None or at_rest(o))
        if still and tt > 0.3:
            st_["rest"] = st_["rest"] if st_["rest"] is not None else tt
        else:
            st_["rest"] = None
        if st_["phase"] == "eject":
            if st_["rest"] is not None and tt - st_["rest"] > 0.2:
                if lip == "stowed" and x * 1e3 < tip_x:
                    return "retained on robot: exit blocked by the stowed lip"
                if x * 1e3 < g.THROAT_X + 25:
                    return "retained in the bite: robot disabled" if p.get("busV") is None else "stalled in the nip"
                if x * 1e3 < tip_x + 5:
                    return "retained on robot"
                st_["phase"] = "withdraw"
                aux["withdrawAt"] = tt
                st_["rest"] = None
            if tt > 6.0 and x * 1e3 < g.THROAT_X + 25:
                return "retained on robot: exit blocked by the stowed lip" if lip == "stowed" else "stalled in the nip"
        elif tt - aux["withdrawAt"] > 0.4 and st_["rest"] is not None and tt - st_["rest"] >= 2.0:
            if other is not None:
                if in_trough(x, z) and in_trough(o[0], o[1]):
                    return "captured and supported 2 s after withdrawal; first pipe still in the trough"
                return "at rest outside trough"
            if xl < x < xr and z < (t["lipTop"] / 1e3 + R):
                return "captured and supported 2 s after withdrawal"
            return "at rest outside trough"
        return None
    if p["rollers"] == "driven":
        v = p["nipSpeed"]
        p = dict(p, nip=lambda tt: v * min(1.0, tt / 0.1))
    out, st, tt, trace, aux = simulate(seat, p, sc, 12.0, stop=stop, record=0.002, other=other)
    o = aux.get("other")
    return dict(label=label, standOffMm=s, lip=lip, rampDeg=ramp_deg, lipProjMm=(tip_x - g.FRAME_FRONT if lip == "deployed" else None),
                outcome=out, endS=round(tt, 3), departure=st_["dep"], final=[round(st[0] * 1e3, 2), round(st[1] * 1e3, 2)],
                finalFromLipMm=[round((st[0] - xl) * 1e3, 2), round(st[1] * 1e3, 2)],
                firstPipeFinalFromLipMm=None if o is None else [round((o[0] - xl) * 1e3, 2), round(o[1] * 1e3, 2)],
                maxShaftDemandNm=round(aux["maxShaftDemandNm"], 3),
                minNipMpsInNip=None if aux["minNipMps"] is None else round(aux["minNipMps"], 4), maxCurrentA=round(aux["maxCurrentA"], 2),
                maxBiteN=round(aux["maxBiteN"], 1), maxRollerN=round(aux["maxRollerN"], 1), maxLipN=round(aux["maxLipN"], 1),
                maxLipMomentNm=round(aux["maxLipMomentNm"], 3), trace=trace, **common(p))


def receive_trial(args):
    label, p, opt = args
    v0 = opt.get("pushMps", 0.0)
    dock = opt.get("dockGapMm", NOMINAL_DOCK_GAP)
    flap, hood, go = opt.get("flap", True), opt.get("hood", True), opt.get("go")
    sc = scene(p, lip="deployed", flap=flap, hood=hood, dock=dock)
    ch = station_chute(dock)
    a = math.radians(STATION["slopeDeg"])
    u = (math.cos(a), -math.sin(a))
    nrm = (math.sin(a), math.cos(a))
    s0 = (ch[0][0] + 10.0 * u[0], ch[0][1] + 10.0 * u[1])
    st0 = ((s0[0] + (g.R + 0.2) * nrm[0]) / 1e3, (s0[1] + (g.R + 0.2) * nrm[1]) / 1e3, v0 * u[0], v0 * u[1], -v0 / R)
    seat_z = g.POCKET_Z / 1e3
    info = dict(passedFlap=None, maxRiseMm=0.0, throatEntry=False, flapClosedS=None, rest=None, exit=None, peak=0.0, flapPass=None,
                goAt=None, stopAt=None, maxBack=0.0)
    flap_x = g.FLAP_X / 1e3
    sill_x = ch[1][0] / 1e3
    if go:
        ta, tc = go["accelS"], go["cruiseS"]
        aa = go["aMps2"]

        def accel(tt):
            if info["goAt"] is None:
                return 0.0
            d = tt - info["goAt"]
            return aa if 0 <= d < ta else (-aa if ta + tc <= d < 2 * ta + tc else 0.0)
    else:
        accel = None
    tmax = 4.0 if not go else 9.0

    def stop(tt, st, aux):
        x, z, vx, vz, w = st
        sp = math.hypot(vx, vz)
        if info["exit"] is None and x > sill_x:
            info["exit"] = round(sp, 4)
        if info["passedFlap"] is None:
            info["peak"] = max(info["peak"], sp)
            if x - R > flap_x + 0.002:
                info["passedFlap"] = tt
                info["flapPass"] = round(sp, 4)
        if info["passedFlap"] is not None:
            rest_z = (g.floor_z(min(x * 1e3, g.FLOOR_END_X)) + g.R / math.cos(math.radians(g.FLOOR_DEG))) / 1e3
            ref = rest_z if x * 1e3 <= g.BITE_X else seat_z
            info["maxRiseMm"] = max(info["maxRiseMm"], (z - ref) * 1e3)
            if aux.get("phi", 0) < math.radians(2) and info["flapClosedS"] is None and (flap or True):
                info["flapClosedS"] = tt
                if go:
                    info["goAt"] = tt + SENSOR_LATENCY
                    info["stopAt"] = info["goAt"] + 2 * go["accelS"] + go["cruiseS"]
            if x + R < flap_x - 0.005:
                return "rear escape past flap"
            info["maxBack"] = max(info["maxBack"], (g.BITE_X / 1e3 - x) * 1e3)
        if x * 1e3 > g.THROAT_X - 2.0:
            info["throatEntry"] = True
        if x * 1e3 > g.THROAT_X + 25.0:
            return "passed through the nip (lost forward)"
        if z < 0.3 or x < sill_x - 0.6:
            return "lost"
        if tt > 2.5 and info["passedFlap"] is None:
            return "did not reach the pocket"
        moving = go and (info["stopAt"] is None or tt < info["stopAt"])
        if info["passedFlap"] is not None and at_rest(st) and not moving:
            info["rest"] = info["rest"] if info["rest"] is not None else tt
            if tt - info["rest"] > 0.15:
                return "seated in the nip throat" if info["throatEntry"] else "seated"
        else:
            info["rest"] = None
        if go and info["stopAt"] is not None and tt > info["stopAt"] + 1.5:
            return "not seated 1.5 s after the stop"
        return None
    out, st, tt, trace, aux = simulate(st0, p, sc, tmax, accel=accel, stop=stop, record=0.002)
    seated_by = info["rest"] if out.startswith("seated") else None
    pf = info["passedFlap"]
    return dict(label=label, pushMps=v0, dockGapMm=dock, flap=flap, hood=hood, go=go, enteredThroat=info["throatEntry"],
                stationExitMps=info["exit"], peakBeforeFlapMps=round(info["peak"], 4), flapPassMps=info["flapPass"],
                outcome=out, seatedByS=seated_by,
                seatedAfterFlapS=(None if seated_by is None or pf is None else round(seated_by - pf, 4)),
                captiveAfterFlapS=(None if info["flapClosedS"] is None or pf is None else round(info["flapClosedS"] - pf, 4)),
                seatedAfterStopS=(None if seated_by is None or info["stopAt"] is None else round(seated_by - info["stopAt"], 4)),
                maxRiseMm=round(info["maxRiseMm"], 2), maxBackFromBiteMm=round(info["maxBack"], 1),
                final=[round(st[0] * 1e3, 2), round(st[1] * 1e3, 2)],
                maxBiteN=round(aux["maxBiteN"], 1), maxRollerN=round(aux["maxRollerN"], 1), maxBackDriveNm=round(aux["maxBackDriveNm"], 3),
                maxHoldDeg=round(aux["maxHoldDeg"], 2), maxCurrentA=round(aux["maxCurrentA"], 2), maxFlapDeg=round(aux["maxFlapDeg"], 1),
                passedFlapS=pf, flapClosedS=info["flapClosedS"], goAtS=info["goAt"], stopAtS=info["stopAt"], trace=trace, **common(p))


def sled_trial(args):
    label, p, gpk, sign, seat = args
    sc = scene(p, lip="deployed", flap=p.get("flap", True))
    T = 0.05
    acc = lambda tt: sign * gpk * G * math.sin(math.pi * (tt - 0.05) / T) if 0.05 <= tt <= 0.05 + T else 0.0
    info = dict(maxDx=0.0)

    def stop(tt, st, aux):
        x = st[0]
        info["maxDx"] = max(info["maxDx"], abs(x - seat[0]) * 1e3)
        if x * 1e3 > g.THROAT_X + 25:
            return "passed the throat (lost forward)"
        if x + R < g.FLAP_X / 1e3 - 0.005:
            return "past the flap (lost rearward)"
        if tt > 1.0:
            return "retained"
        return None
    out, st, tt, trace, aux = simulate(seat, p, sc, 1.1, accel=acc, stop=stop, record=0.002)
    return dict(label=label, flap=p.get("flap", True), g=gpk,
                direction=("chassis accelerates toward +X: pipe loads toward the flap" if sign > 0
                           else "chassis brakes from +X travel: pipe loads into the nip"),
                outcome=out, maxTravelMm=round(info["maxDx"], 2), finalOffsetMm=round((st[0] - seat[0]) * 1e3, 2),
                final=[round(st[0] * 1e3, 2), round(st[1] * 1e3, 2)], maxBiteN=round(aux["maxBiteN"], 1),
                maxBackDriveNm=round(aux["maxBackDriveNm"], 3), maxHoldDeg=round(aux["maxHoldDeg"], 2), trace=trace, **common(p))


def good(r):
    o = r["outcome"]
    return o.startswith(("captured", "retained")) and "disabled" not in o and "stowed" not in o or o == "seated"


def tally(rows, pred=lambda r: True, **want):
    sel = [r for r in rows if pred(r) and all(r.get(k) == v for k, v in want.items())]
    out = {}
    for r in sel:
        out[r["outcome"]] = out.get(r["outcome"], 0) + 1
    return dict(n=len(sel), ok=sum(good(r) for r in sel), outcomes=dict(sorted(out.items())))


def rng(rows, key, digits=3):
    v = [r[key] for r in rows if r.get(key) is not None]
    return [round(min(v), digits), round(max(v), digits)] if v else None


def main(out, processes):
    gentle_start = ((g.BITE_X - 3.0) / 1e3, (g.POCKET_Z + 2.0) / 1e3, 0.0, 0.0, 0.0)
    settled, _ = settle(P(), gentle_start)
    seats, creep = {}, {}
    for kt in (1.0e4, 2.0e4) + K_BAND + (1.1e5, 2.4e5, 3.2e5):
        r = receive_trial(("seat", P(kTread=kt), {}))
        if r["outcome"].startswith("seated"):
            seats[kt] = (r["final"][0] / 1e3, r["final"][1] / 1e3, 0.0, 0.0, 0.0)
        else:                       # below the band the nominal receive itself can fail: settle the pipe gently instead
            seats[kt], _ = settle(P(kTread=kt), gentle_start)
        if kt in K_BAND:
            creep[str(kt / 1e3)] = dwell(P(kTread=kt), seats[kt])
    disabled_seat, _ = settle(P(busV=None, hold=False), gentle_start)
    creep["robot disabled"] = dwell(P(busV=None, hold=False), disabled_seat)
    creep["gently settled"] = dwell(P(), settled)
    E = lambda **kw: P(hold=False, **kw)       # eject: open-loop bus voltage on the selected drive
    bus = (9.0, 10.5, 12.0, 13.0)
    ejects = []
    for k, V, s, e, mu, mut in product(K_BAND, bus, (0, 25, 50, 75, 100), (0.2, 0.5), (0.15, 0.35), (0.4, 1.0)):
        ejects.append(("G2 eject", E(e=e, mu=mu, muTread=mut, kTread=k, busV=V), float(s), "deployed", {}, seats[k]))
    for V, e, mu in product((9.0, 12.0), (0.2, 0.5), (0.15, 0.35)):
        ejects.append(("bracket: stand-off 125 mm", E(e=e, mu=mu, busV=V), 125.0, "deployed", {}, seats[8.0e4]))
    for s, e, mu in product((0, 50, 75, 100), (0.2, 0.5), (0.15, 0.35)):
        ejects.append(("isolation: lip removed", E(e=e, mu=mu), float(s), "none", {}, seats[8.0e4]))
    for s, e, mu, V in product((0, 50, 75, 100), (0.2, 0.5), (0.15, 0.35), (9.0, 12.0)):
        ejects.append(("control: lip tip 72 mm out (inside the bumper face)", E(e=e, mu=mu, busV=V), float(s), "deployed", dict(lipProjMm=72.0), seats[8.0e4]))
    for s in (0, 50, 100):
        ejects.append(("control: latch failed, lip stowed", E(), float(s), "stowed", {}, seats[8.0e4]))
    ejects.append(("control: robot disabled", P(busV=None, hold=False), 50.0, "deployed", {}, seats[8.0e4]))
    for k, V, s in product((4.0e4, 8.0e4, 1.1e5, 1.6e5), (9.0, 12.0), (0, 100)):
        ejects.append(("comparison: layout-001 worm 30:1", P(e=0.5, mu=0.15, kTread=k, drive=WORM, busV=V, hold=False), float(s), "deployed", {}, seats[k]))
    for k, V, s, mut in product((1.0e4, 2.0e4, 2.4e5, 3.2e5), (9.0, 12.0), (0, 100), (0.4, 1.0)):
        ejects.append(("bracket: tread stiffness", E(e=0.5, mu=0.15, muTread=mut, kTread=k, busV=V), float(s), "deployed", {}, seats[k]))
    for ramp, s in product((20.0, 40.0), (0, 50, 100)):
        ejects.append((f"lip ramp {ramp:.0f} deg", E(e=0.5, mu=0.15, busV=10.5), float(s), "deployed", dict(rampDeg=ramp), seats[8.0e4]))
    for sq, s in product((4.0, 8.0), (0, 100)):
        ejects.append((f"sensitivity: squeeze {sq:.0f} mm", E(e=0.5, mu=0.15, busV=9.0, squeeze=sq), float(s), "deployed", {}, seats[8.0e4]))
    for kT_, s in product((1.0e4, 4.0e4), (0, 100)):
        ejects.append((f"sensitivity: tangential stiffness {kT_ / 1e3:.0f} N/mm", E(e=0.5, mu=0.15, busV=9.0, kT=kT_), float(s), "deployed", {}, seats[8.0e4]))
    n_first = len(ejects)
    receives = []
    for k, dock, e, mu, mut in product(K_BAND, (0.0, 10.0, 50.0), (0.2, 0.5), (0.15, 0.35), (0.3, 0.6, 1.0)):
        receives.append(("G1 receive", P(e=e, mu=mu, muTread=mut, kTread=k), dict(dockGapMm=dock)))
    for k, push, e, mu, mut in product(K_BAND, (1.0, 2.0), (0.2, 0.5), (0.15, 0.35), (0.3, 1.0)):
        receives.append(("adverse: pushed feed", P(e=e, mu=mu, muTread=mut, kTread=k), dict(pushMps=push)))
    for k, e, mu, mut in product((1.0e4, 2.0e4, 3.0e4), (0.2, 0.5), (0.15, 0.35), (0.3, 0.6, 1.0)):
        receives.append(("bracket: tread stiffness", P(e=e, mu=mu, muTread=mut, kTread=k), {}))
    for e, mu in product((0.2, 0.5), (0.15, 0.35)):
        receives.append(("baseline: powered hold", P(e=e, mu=mu), {}))
        receives.append(("isolation: no flap", P(e=e, mu=mu), dict(flap=False)))
        receives.append(("isolation: no hood", P(e=e, mu=mu), dict(hood=False)))
        receives.append(("control: robot disabled (rollers back-drivable)", P(e=e, mu=mu, busV=None, hold=False), {}))
        receives.append(("comparison: layout-001 worm locked", P(e=e, mu=mu, rollers="locked"), {}))
        receives.append(("adverse: weak flap spring", P(e=e, mu=mu, flapTau0=0.01, flapK=0.015), {}))
        receives.append(("isolation: no hood flare", P(e=e, mu=mu, flare=False), {}))
    for sq in (4.0, 8.0):
        receives.append((f"sensitivity: squeeze {sq:.0f} mm", P(e=0.5, mu=0.15, muTread=0.6, squeeze=sq), {}))
    for kT_ in (1.0e4, 4.0e4):
        receives.append((f"sensitivity: tangential stiffness {kT_ / 1e3:.0f} N/mm", P(e=0.5, mu=0.15, muTread=0.6, kT=kT_), {}))
    for k, a_, e, mut in product(K_BAND, (3.0, 6.0), (0.2, 0.5), (0.3, 1.0)):
        receives.append(("receive and go", P(e=e, muTread=mut, kTread=k), dict(go=dict(aMps2=a_, accelS=0.5, cruiseS=0.5))))
    receives.append(("mission: receive and go, nominal", P(muTread=1.0), dict(go=dict(aMps2=3.0, accelS=0.5, cruiseS=0.5))))
    for e in (0.2, 0.5):
        receives.append(("control: receive and go, no flap", P(e=e, kTread=1.6e5), dict(flap=False, go=dict(aMps2=6.0, accelS=0.5, cruiseS=0.5))))
    sleds = []
    for gpk, sign, kt, mut, e in product((1.0, 3.0, 8.0), (1, -1), K_BAND, (0.3, 1.0), (0.2, 0.5)):
        sleds.append(("G3 powered hold", P(kTread=kt, muTread=mut, e=e), gpk, sign, seats[kt]))
    for gpk, sign, kt in product((1.0, 3.0, 8.0), (1, -1), K_BAND):
        sleds.append(("G3 robot disabled (back-drivable)", P(kTread=kt, busV=None, hold=False), gpk, sign, seats[kt]))
    for gpk, sign in product((1.0, 3.0, 8.0), (1, -1)):
        sleds.append(("G3 powered hold, gently settled seat", P(), gpk, sign, settled))
        sleds.append(("isolation: no check flap, gently settled seat", P(flap=False), gpk, sign, settled))
        sleds.append(("isolation: no check flap, received seat", P(flap=False), gpk, sign, seats[8.0e4]))
    for sq in (4.0, 8.0):
        sleds.append((f"sensitivity: squeeze {sq:.0f} mm", P(squeeze=sq, muTread=0.6, e=0.5), 8.0, -1, seats[8.0e4]))
    with Pool(processes) as pool:
        ej = pool.map(eject_trial, ejects, chunksize=1)
        # second trip: the first pipe rests where the nominal 12 V eject left it, relative to the trough lip
        first_rest = {}
        for s1 in (0.0, 50.0, 100.0):
            r = next(r for r in ej if r["label"] == "G2 eject" and r["standOffMm"] == s1 and r["busV"] == 12.0 and r["e"] == 0.2
                     and r["mu"] == 0.35 and r["muTread"] == 1.0 and r["kTreadNmm"] == 80.0)
            assert r["outcome"].startswith("captured"), r["outcome"]
            first_rest[s1] = r["finalFromLipMm"]
        second = []
        for s1, s2, V, e, mu in product((0.0, 50.0, 100.0), (0, 25, 50, 75, 100), (9.0, 12.0, 13.0), (0.2, 0.5), (0.15, 0.35)):
            second.append(("second trip: first pipe in the same section", E(e=e, mu=mu, busV=V), float(s2), "deployed",
                           dict(firstPipeFromLipMm=first_rest[s1], firstDockMm=s1), seats[8.0e4]))
        ejects += second
        ej += pool.map(eject_trial, second, chunksize=1)
        rc = pool.map(receive_trial, receives, chunksize=1)
        sl = pool.map(sled_trial, sleds, chunksize=1)
    for a, r in zip(ejects, ej):
        if "firstDockMm" in a[4]:
            r["firstDockMm"] = a[4]["firstDockMm"]
            r["firstPipeStartFromLipMm"] = a[4]["firstPipeFromLipMm"]

    def pick(rows, **want):
        for r in rows:
            if all(r.get(k) == v for k, v in want.items()):
                return r
        raise KeyError(want)

    def first(rows, label, **want):
        cands = [r for r in rows if r["label"] == label and all(r.get(k) == v for k, v in want.items())]
        return next((r for r in cands if not good(r)), cands[0])

    def worst(rows, key, label, **want):
        cands = [r for r in rows if r["label"] == label and r.get(key) is not None and all(r.get(k) == v for k, v in want.items())]
        return max(cands, key=lambda r: r[key])
    into, toward = "chassis brakes from +X travel: pipe loads into the nip", "chassis accelerates toward +X: pipe loads toward the flap"
    playback = [
        ("receive-mission", "Mission receive: released from rest, leave 50 ms after the flap closes at 3 m/s2, brake, seat (80 N/mm tread)",
         pick(rc, label="mission: receive and go, nominal")),
        ("receive-nominal", "Station receive: released from rest, 10 mm dock gap, 80 N/mm tread, e 0.2, mu 0.35, tread mu 1.0",
         pick(rc, label="G1 receive", dockGapMm=10.0, kTreadNmm=80.0, e=0.2, mu=0.35, muTread=1.0)),
        ("receive-adverse", "Station receive, adverse: pushed 2.0 m/s down the chute, 160 N/mm tread, e 0.5, mu 0.15, tread mu 0.3",
         pick(rc, label="adverse: pushed feed", pushMps=2.0, kTreadNmm=160.0, e=0.5, mu=0.15, muTread=0.3)),
        ("receive-slowest-seat", "Station receive: the slowest seat in the acceptance band", worst(rc, "seatedAfterFlapS", "G1 receive")),
        ("receive-soft-tread", "Bracket: tread softer than the acceptance band (first failing case)", first(rc, "bracket: tread stiffness")),
        ("receive-control-disabled", "Control: robot disabled, rollers back-drivable", first(rc, "control: robot disabled (rollers back-drivable)")),
        ("receive-isolation-no-flap", "Isolation: check flap removed", first(rc, "isolation: no flap")),
        ("receive-and-go", "Receive and go: leave 50 ms after the flap closes, 6 m/s2, 160 N/mm tread, e 0.5",
         worst(rc, "seatedAfterStopS", "receive and go", kTreadNmm=160.0)),
        ("receive-and-go-no-flap", "Control: receive and go with the check flap removed", first(rc, "control: receive and go, no flap")),
        ("eject-nominal", "Eject at 12 V, 50 mm stand-off, 80 N/mm tread, e 0.2, mu 0.35",
         pick(ej, label="G2 eject", busV=12.0, standOffMm=50.0, kTreadNmm=80.0, e=0.2, mu=0.35, muTread=1.0)),
        ("eject-adverse-far", "Eject at 9 V, 100 mm stand-off, 160 N/mm tread, e 0.5, mu 0.15, tread mu 0.4",
         pick(ej, label="G2 eject", busV=9.0, standOffMm=100.0, kTreadNmm=160.0, e=0.5, mu=0.15, muTread=0.4)),
        ("eject-adverse-near", "Eject at 13 V, bumper touching (0 mm), 40 N/mm tread, e 0.5, mu 0.15, then withdraw",
         pick(ej, label="G2 eject", busV=13.0, standOffMm=0.0, kTreadNmm=40.0, e=0.5, mu=0.15, muTread=1.0)),
        ("eject-standoff-125", "Bracket: 125 mm stand-off, outside the accepted dock (first failing case)", first(ej, "bracket: stand-off 125 mm")),
        ("eject-control-latch-failed", "Control: latch failed, lip still stowed, 12 V", pick(ej, label="control: latch failed, lip stowed", standOffMm=50.0)),
        ("eject-control-disabled", "Control: robot disabled at the reef", pick(ej, label="control: robot disabled")),
        ("eject-isolation-lip-removed", "Isolation: lip removed, 12 V (first failing stand-off)", first(ej, "isolation: lip removed")),
        ("eject-control-short-lip", "Control: lip tip pulled inside the bumper face (first failing stand-off)",
         first(ej, "control: lip tip 72 mm out (inside the bumper face)")),
        ("eject-worm-comparison", "Comparison: the layout-001 worm drive on a stiff tread (first failing case)", first(ej, "comparison: layout-001 worm 30:1")),
        ("eject-stiff-tread", "Bracket: tread stiffer than the acceptance band (first failing case)", first(ej, "bracket: tread stiffness")),
        ("eject-second-trip", "Second trip, adverse: the first pipe rests in the same trough section, 12 V, 50 mm stand-off",
         pick(ej, label="second trip: first pipe in the same section", busV=12.0, standOffMm=50.0, firstDockMm=50.0, e=0.2, mu=0.35)),
        ("eject-second-trip-worst", "Second trip, adverse: first non-captured case, else the fastest departure",
         next((r for r in ej if r["label"].startswith("second trip") and not good(r)),
              max((r for r in ej if r["label"].startswith("second trip")), key=lambda r: (r["departure"] or {}).get("speed", 0)))),
        ("sled-8g-into-nip", "Carry sled 8 g, 50 ms, pipe loads into the held nip, 40 N/mm tread, tread mu 0.3",
         pick(sl, label="G3 powered hold", g=8.0, kTreadNmm=40.0, muTread=0.3, e=0.5, direction=into)),
        ("sled-8g-toward-flap", "Carry sled 8 g, 50 ms, pipe loads toward the closed flap",
         pick(sl, label="G3 powered hold", g=8.0, kTreadNmm=160.0, muTread=0.3, e=0.5, direction=toward)),
        ("sled-disabled-8g", "Carry sled 8 g with the robot disabled (rollers back-drivable)",
         first(sl, "G3 robot disabled (back-drivable)", g=8.0, direction=into)),
        ("sled-isolation-no-flap", "Isolation: check flap removed, gently settled seat", first(sl, "isolation: no check flap, gently settled seat")),
    ]
    kept = {id(r) for _, _, r in playback}
    for rows in (ej, rc, sl):
        for r in rows:
            if id(r) not in kept:
                r.pop("trace", None)
            else:
                r["trace"] = r["trace"][::2]

    def rerun(pid, r):
        kind = "eject" if pid.startswith("eject") else "receive" if pid.startswith("receive") else "sled"
        src = {"eject": ejects, "receive": receives, "sled": sleds}[kind][{"eject": ej, "receive": rc, "sled": sl}[kind].index(r)]
        args = list(src)
        args[1] = dict(args[1], dt=DT_DEFAULT / 2)
        return (kind, tuple(args))
    reruns = [rerun(i, r) for i, _, r in playback]
    with Pool(processes) as pool:
        halves = pool.starmap(_run, reruns, chunksize=1)
    sensitivity = []
    for (pid, _, r), (kind, _), h in zip(playback, reruns, halves):
        keys = {"eject": ("outcome", "final"), "receive": ("outcome", "final", "captiveAfterFlapS", "seatedAfterFlapS"),
                "sled": ("outcome", "final", "maxTravelMm")}[kind]
        sensitivity.append(dict(id=pid, dtS=DT_DEFAULT / 2, base={k: r.get(k) for k in keys}, half={k: h.get(k) for k in keys},
                                sameOutcome=h["outcome"] == r["outcome"]))
    G2 = [r for r in ej if r["label"] == "G2 eject"]
    G1 = [r for r in rc if r["label"] == "G1 receive"]
    PUSH = [r for r in rc if r["label"] == "adverse: pushed feed"]
    GO = [r for r in rc if r["label"] == "receive and go"]
    G3 = [r for r in sl if r["label"] == "G3 powered hold"]
    SEC = [r for r in ej if r["label"].startswith("second trip")]
    lab = lambda rows, l: [r for r in rows if r["label"] == l]
    summary = dict(
        bandNmm=[k / 1e3 for k in K_BAND], acceptanceBandNmm=list(ACCEPT_BAND), nominalNmm=80.0,
        g2=tally(G2), g2ByStandOff={str(int(s)): tally(G2, standOffMm=float(s)) for s in (0, 25, 50, 75, 100)},
        g2ByStiffness={str(int(k / 1e3)): tally(G2, kTreadNmm=k / 1e3) for k in K_BAND},
        g2DepartureMps=rng([r["departure"] for r in G2 if r["departure"]], "speed"), g2MinNipMps=rng(G2, "minNipMpsInNip")[0],
        g2ShaftDemandNm=rng(G2, "maxShaftDemandNm")[1], g2RollerN=rng(G2, "maxRollerN", 1)[1], g2CurrentA=rng(G2, "maxCurrentA", 1)[1],
        lipN=rng(G2 + SEC, "maxLipN", 1)[1], lipMomentNm=rng(G2 + SEC, "maxLipMomentNm")[1],
        standOff125=tally(lab(ej, "bracket: stand-off 125 mm")),
        lipRemoved={str(int(s)): tally(lab(ej, "isolation: lip removed"), standOffMm=float(s)) for s in (0, 50, 75, 100)},
        shortLip={str(int(s)): tally(lab(ej, "control: lip tip 72 mm out (inside the bumper face)"), standOffMm=float(s)) for s in (0, 50, 75, 100)},
        latchFailed=tally(lab(ej, "control: latch failed, lip stowed")), ejectDisabled=tally(lab(ej, "control: robot disabled")),
        wormByStiffness={str(int(k)): tally(lab(ej, "comparison: layout-001 worm 30:1"), kTreadNmm=float(k)) for k in (40, 80, 110, 160)},
        ejectStiffnessBracket={str(int(k)): tally(lab(ej, "bracket: tread stiffness"), kTreadNmm=float(k)) for k in (10, 20, 240, 320)},
        ramp={str(int(a)): tally(lab(ej, f"lip ramp {a:.0f} deg")) for a in (20.0, 40.0)},
        secondTrip=tally(SEC), secondTripDepartureMps=rng([r["departure"] for r in SEC if r["departure"]], "speed"),
        g1=tally(G1), g1ByStiffness={str(int(k / 1e3)): tally(G1, kTreadNmm=k / 1e3) for k in K_BAND},
        g1ByDock={str(int(d)): tally(G1, dockGapMm=float(d)) for d in (0, 10, 50)},
        g1CaptiveS=rng(G1, "captiveAfterFlapS"), g1SeatS=rng(G1, "seatedAfterFlapS"), g1ExitMps=rng(G1, "stationExitMps"),
        g1PeakMps=rng(G1, "peakBeforeFlapMps"), g1FlapPassMps=rng(G1, "flapPassMps"), g1RiseMm=rng(G1, "maxRiseMm", 2)[1],
        g1RollerN=rng(G1 + PUSH, "maxRollerN", 1)[1], g1BackDriveNm=rng(G1 + PUSH, "maxBackDriveNm")[1], g1HoldDeg=rng(G1 + PUSH, "maxHoldDeg", 2)[1],
        g1CurrentA=rng(G1 + PUSH, "maxCurrentA", 1)[1],
        pushed={str(v): tally(PUSH, pushMps=v) for v in (1.0, 2.0)}, pushedPeakMps=rng(PUSH, "peakBeforeFlapMps"),
        pushedCaptiveS=rng(PUSH, "captiveAfterFlapS"), pushedSeatS=rng(PUSH, "seatedAfterFlapS"),
        receiveStiffnessBracket={str(int(k)): tally(lab(rc, "bracket: tread stiffness"), kTreadNmm=float(k)) for k in (10, 20, 30)},
        receiveIso={l: tally(lab(rc, l)) for l in ("baseline: powered hold", "isolation: no flap", "isolation: no hood",
                                                   "control: robot disabled (rollers back-drivable)", "comparison: layout-001 worm locked",
                                                   "adverse: weak flap spring", "isolation: no hood flare")},
        go=tally(GO), goByAccel={str(a): tally(GO, pred=lambda r, a=a: r["go"]["aMps2"] == a) for a in (3.0, 6.0)},
        goSeatAfterStopS=rng(GO, "seatedAfterStopS"), goMaxBackMm=rng(GO, "maxBackFromBiteMm", 1)[1],
        goNoFlap=tally(lab(rc, "control: receive and go, no flap")),
        g3=tally(G3), g3TravelMm=rng(G3, "maxTravelMm", 2)[1], g3BackDriveNm=rng(G3, "maxBackDriveNm")[1],
        g3Disabled=tally(lab(sl, "G3 robot disabled (back-drivable)")),
        sledIso={l: {str(int(gk)): tally(lab(sl, l), g=gk) for gk in (1.0, 3.0, 8.0)} for l in (
            "G3 powered hold, gently settled seat", "isolation: no check flap, gently settled seat", "isolation: no check flap, received seat")},
        sensitivity={l: tally(lab(ej + rc + sl, l)) for l in ("sensitivity: squeeze 4 mm", "sensitivity: squeeze 8 mm",
                                                               "sensitivity: tangential stiffness 10 N/mm", "sensitivity: tangential stiffness 40 N/mm")},
        timestepSameOutcome=sum(s_["sameOutcome"] for s_ in sensitivity), timestepCases=len(sensitivity),
    )
    result = dict(schema="red2/pocket-field-trials@3", summary=summary, timestepSensitivity=sensitivity,
                  playback=[dict(id=i, title=t_, kind=("eject" if i.startswith("eject") else "receive" if i.startswith("receive") else "sled"),
                                 index=(ej if i.startswith("eject") else rc if i.startswith("receive") else sl).index(r)) for i, t_, r in playback],
                  model=__doc__.strip(), dtS=DT_DEFAULT, rigidStiffnessNpm=K_RIGID, tangentialStiffnessNpm=KT_DEFAULT,
                  motor=MOTOR, rollerInertiaKgm2=J_ROLLERS, drive=MAX36, holdRad=HOLD_RAD, comparisonWorm=WORM, station=STATION,
                  nominalDockGapMm=NOMINAL_DOCK_GAP, sensorLatencyS=SENSOR_LATENCY,
                  pipe=dict(radiusMm=g.R, innerRadiusMm=g.PIPE_RI, massKg=M, rollingFactor=K),
                  nominal={k: (v if not isinstance(v, dict) else "MAXPlanetary 36:1") for k, v in NOMINAL.items()},
                  settledSeatMm=[settled[0] * 1e3, settled[1] * 1e3], disabledSeatMm=[disabled_seat[0] * 1e3, disabled_seat[1] * 1e3],
                  carrySeatsByTreadStiffness={str(k / 1e3): [v[0] * 1e3, v[1] * 1e3] for k, v in seats.items()},
                  seatCreepMmOver3s=creep, eject=ej, receive=rc, sled=sl)
    Path(out).write_text(json.dumps(result, separators=(",", ":"), allow_nan=False) + "\n", encoding="utf-8")


def _run(kind, args):
    return {"eject": eject_trial, "receive": receive_trial, "sled": sled_trial}[kind](args)


if __name__ == "__main__":
    procs = int(sys.argv[sys.argv.index("--processes") + 1]) if "--processes" in sys.argv else 8
    main(sys.argv[1], procs)
