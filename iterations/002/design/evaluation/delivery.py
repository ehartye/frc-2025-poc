"""Delivery screen r2: a planar rigid disc with impulse contacts against the low-trough section.

Revision 2 answers the adversarial review:
  * exit height is the REAL post-fence height for B (pocket centre + fence height), and launch
    angle is swept (-10, 0, +10 deg);
  * landing is no longer "normal velocity zeroed": every contact (lip outer face, lip top, lip inner
    face, 15 deg floor, riser face, riser top) is an impulse with restitution e and Coulomb friction mu,
    tracking spin, so bounces near the 46 mm riser are simulated;
  * option B' (same fenced pocket and nip, then a short down-sloping exit lip that rolls the pipe off
    at low speed with rolling spin) is screened the same way, frame-contained (B'0) and with a 100 mm
    one-time-deployed exit lip (B'x).

Planar XZ, pipe axis along Y, coral and trough from sources.py. The trough outer lip face sits at
x = bumper face + stand-off s; s is unknown for the real dock, so capture is reported over s = 0..100 mm.
Crossing the riser top is counted lost (conservative, matching the recorded "cleared the far lip").
Still NOT modelled: yaw, end contact on the trough panels, pipe ovality and wear, the 3D panel holes,
peers, and measured restitution/friction (swept). The pan (A) result is a spatial 12-case replay; this
is a planar screen, so the A/B comparison is asymmetric.
"""
import math
import sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sources import coral, trough  # noqa: E402

G = 9.80665
FRAME_FRONT = 0.370   # design.json frameLengthM / 2
BUMPER_DEPTH = 0.075  # design.json bumperDepthM
C, T = coral(), trough()
R, M, K = C["R"], C["M"], C["k"]
I = K * M * R * R


def segments(xl):
    t = T
    ld = t["lipDepth"]
    xr = xl + t["riserX"]
    fl = lambda x: t["floor0"] + t["slope"] * (x - xl)
    return xr, [
        ((xl, 0.30), (xl, t["lipTop"])),                    # lip outer face
        ((xl, t["lipTop"]), (xl + ld, t["lipTop"])),        # lip top
        ((xl + ld, fl(xl + ld)), (xl + ld, t["lipTop"])),   # lip inner face
        ((xl + ld, fl(xl + ld)), (xr, fl(xr))),             # 15 deg floor
        ((xr, fl(xr)), (xr, t["riserTop"])),                # riser face
        ((xr, t["riserTop"]), (xr + 0.00635, t["riserTop"])),  # riser top
    ]


def closest(px, pz, a, b):
    ax, az = a
    bx, bz = b
    dx, dz = bx - ax, bz - az
    L2 = dx * dx + dz * dz
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (pz - az) * dz) / L2))
    return ax + u * dx, az + u * dz


def simulate(x, z, vx, vz, w, s, e, mu, dt=2e-4, tmax=2.5):
    """Return (outcome, detail). w is spin with point velocity w*(-rz, rx)."""
    xl = FRAME_FRONT + BUMPER_DEPTH + s
    xr, segs = segments(xl)
    still = 0.0
    bounces = 0
    t = 0.0
    while t < tmax:
        vz -= G * dt
        x += vx * dt
        z += vz * dt
        t += dt
        for a, b in segs:
            qx, qz = closest(x, z, a, b)
            dx, dz = x - qx, z - qz
            d = math.hypot(dx, dz)
            if d >= R or d == 0:
                continue
            nx, nz = dx / d, dz / d
            x += nx * (R - d)
            z += nz * (R - d)
            vn = vx * nx + vz * nz
            if vn >= 0:
                continue
            ee = e if -vn > 0.15 else 0.0
            if ee:
                bounces += 1
            Jn = -(1 + ee) * vn * M
            rx, rz = -R * nx, -R * nz
            tx, tz = -nz, nx
            vt = (vx + w * -rz) * tx + (vz + w * rx) * tz
            Jt = -vt * M / (1 + 1 / K)
            Jt = max(-mu * Jn, min(mu * Jn, Jt))
            vx += (Jn * nx + Jt * tx) / M
            vz += (Jn * nz + Jt * tz) / M
            w += (rx * (Jt * tz) - rz * (Jt * tx)) / I
        if x > xr + 0.004 and z > T["riserTop"]:
            return "lost over riser", bounces
        if z < 0.40 and x < xl + T["lipDepth"]:
            return "fell short / out over lip", bounces
        if xl < x < xr and math.hypot(vx, vz) < 0.01 and abs(w) * R < 0.01:
            still += dt
            if still > 0.2:
                return "captured", bounces
        else:
            still = 0.0
    inside = xl < x < xr and z < T["lipTop"] + R
    return ("captured" if inside else "unsettled"), bounces


# ------------------------------------------------------------------ cases
E_SWEEP = (0.2, 0.5)
MU_SWEEP = (0.15, 0.35)
S_SWEEP = tuple(i * 0.010 for i in range(11))


def case(args):
    opt, z0, v, ang, x0, rolling, s, e, mu = args
    vx, vz = v * math.cos(math.radians(ang)), v * math.sin(math.radians(ang))
    w = -v / R if rolling else 0.0
    out, nb = simulate(x0, z0, vx, vz, w, s, e, mu)
    return (opt, z0, v, ang, round(s * 1e3), e, mu, out, nb)


def grid():
    jobs = []
    sp = lambda lo, hi: [round(lo + 0.1 * i, 2) for i in range(int(round((hi - lo) / 0.1)) + 1)]
    # B: two-roller nip (rollers drive top and bottom of the pipe the same way -> no spin) over the
    # 70 mm fence. Exit centre = pocket centre + 0.070: pocket centre 0.60 -> 0.67, 0.63 -> 0.70.
    for z0 in (0.67, 0.70):
        for v in sp(0.6, 2.0):
            for ang in (-10.0, 0.0, 10.0):
                jobs += [("B", z0, v, ang, FRAME_FRONT - 0.020, False, s, e, mu)
                         for s in S_SWEEP for e in E_SWEEP for mu in MU_SWEEP]
    # B'0: after the nip a short down-sloping exit lip rolls the pipe off the frame edge, rolling spin.
    for z0 in (0.60, 0.67):
        for v in sp(0.3, 1.5):
            for ang in (0.0, -15.0, -30.0):
                jobs += [("B'0", z0, v, ang, FRAME_FRONT - 0.005, True, s, e, mu)
                         for s in S_SWEEP for e in E_SWEEP for mu in MU_SWEEP]
    # B'x: the same with an exit lip deployed once at match start so the pipe centre leaves 100 mm
    # beyond the frame perimeter (R105 18 in extension limit; lip tip >= 33 mm above the trough lip).
    for z0 in (0.55, 0.60):
        for v in sp(0.3, 1.5):
            for ang in (0.0, -15.0, -30.0):
                jobs += [("B'x", z0, v, ang, FRAME_FRONT + 0.100, True, s, e, mu)
                         for s in S_SWEEP for e in E_SWEEP for mu in MU_SWEEP]
    return jobs


def speed_windows(points):
    """Contiguous launch-speed band that captures every stand-off under every e and mu."""
    out = []
    keys = sorted({(p["option"], p["exitCentreZ"], p["angleDeg"]) for p in points})
    for opt, z0, ang in keys:
        row = sorted((p for p in points if (p["option"], p["exitCentreZ"], p["angleDeg"]) == (opt, z0, ang)),
                     key=lambda p: p["launchMps"])
        full = [p["launchMps"] for p in row if p["robustCount"] == len(S_SWEEP)]
        best, cur = [], []
        for v in full:
            if cur and v - cur[-1] > 0.101:
                cur = []
            cur.append(v)
            if len(cur) > len(best):
                best = list(cur)
        out.append(dict(option=opt, exitCentreZ=z0, angleDeg=ang,
                        band=[best[0], best[-1]] if best else None,
                        halfWidthPct=(round(50 * (best[-1] - best[0]) / ((best[-1] + best[0]) / 2), 1) if len(best) > 1 else 0.0)))
    return out


def summarise(rows):
    from collections import defaultdict
    by = defaultdict(list)
    for opt, z0, v, ang, s, e, mu, out, nb in rows:
        by[(opt, z0, v, ang)].append((s, e, mu, out, nb))
    points = []
    for (opt, z0, v, ang), lst in sorted(by.items()):
        n = len(lst)
        ok = sum(1 for r in lst if r[3] == "captured")
        # stand-offs captured under EVERY e, mu combination
        robust = sorted({s for s, *_ in lst if all(r[3] == "captured" for r in lst if r[0] == s)})
        modes = {}
        for r in lst:
            modes[r[3]] = modes.get(r[3], 0) + 1
        points.append(dict(option=opt, exitCentreZ=z0, launchMps=v, angleDeg=ang, captured=ok, of=n,
                           robustStandOffsMm=robust, robustCount=len(robust), outcomes=modes,
                           maxBounces=max(r[4] for r in lst)))
    best = {}
    for p in points:
        cur = best.get(p["option"])
        if cur is None or (p["robustCount"], p["captured"]) > (cur["robustCount"], cur["captured"]):
            best[p["option"]] = p
    return points, best


def run(processes=8):
    jobs = grid()
    with Pool(processes) as pool:
        rows = pool.map(case, jobs, chunksize=32)
    points, best = summarise(rows)
    return dict(speedWindows=speed_windows(points), model=__doc__.strip().splitlines()[0], standOffsMm=[round(s * 1e3) for s in S_SWEEP],
                restitution=E_SWEEP, friction=MU_SWEEP, cases=len(rows), points=points, best=best,
                pan=dict(recordedMinSupportMarginMm=1.8282960000000001,
                         recordedDockSensitivity="At the original dock both nominal far deliveries cleared the far "
                                                 "lip; a 12 mm earlier stop passed 12/12 (commissioning.md)",
                         basis="spatial 12-case replay (single_pivot_spatial.mjs + limited_field.py)"))


if __name__ == "__main__":
    import json
    r = run()
    for p in r["points"]:
        print(p["option"], p["exitCentreZ"], p["launchMps"], p["angleDeg"], f"{p['captured']}/{p['of']}",
              "robust", p["robustStandOffsMm"], p["outcomes"])
    for w in r["speedWindows"]:
        print("window", w)
