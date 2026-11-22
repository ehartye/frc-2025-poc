"""Custody screens: receive energy, passive retention limits, and fairness of the fixed-path test.

Quasi-static rigid-body screens on the recorded pipe. Rolling factor k = I/(M R^2) from the
pan solver. A horizontal chassis acceleration a puts an inertial load -M a on the pipe.
These are screens to rank architectures, not contact replays.
"""
import math

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sources import coral, load  # noqa: E402

G = 9.80665


def step_mount_speed(h, R, k):
    """Minimum rolling speed for a cylinder to mount a step of height h < R.

    Angular momentum about the step corner is conserved at impact:
    v' = v (1 + k - h/R) / (1 + k); mounting needs (1+k) v'^2 / 2 >= g h.
    Returns inf for h >= R (a wall cannot be rolled over).
    """
    if h >= R:
        return math.inf
    v_after = math.sqrt(2 * G * h / (1 + k))
    return v_after * (1 + k) / (1 + k - h / R)


def rolling_energy(M, v, k):
    return 0.5 * M * v * v * (1 + k)


def lip_hold_accel(h, R, tilt_deg):
    """Largest along-floor chassis acceleration a rolling pipe resists at a lip of height h,
    floor tilted tilt_deg toward the lip (positive = downhill toward the lip)."""
    phi = math.acos((R - h) / R)  # contact-normal angle at the lip corner
    t = math.radians(tilt_deg)
    return G * (math.tan(phi) * math.cos(t) - math.sin(t)) / (math.cos(t) + math.tan(phi) * math.sin(t))


def _modes(variant):
    out = {}
    for cv in variant["cases"].values():
        ff = cv.get("firstFailure") or {}
        key = ff.get("phase", "passed")
        out[key] = out.get(key, 0) + 1
    return out


def wall_impact(v, w, beta_deg, mu, e, R, k):
    """Pipe moving +x at v with spin w strikes a fence face whose top leans toward the pocket by beta.
    Returns post-impact (vx, vz, w). Point velocity convention w*(-rz, rx)."""
    b = math.radians(beta_deg)
    nx, nz = -math.cos(b), -math.sin(b)          # wall normal, into the pocket and downward
    vx, vz = v, 0.0
    vn = vx * nx + vz * nz
    Jn = -(1 + e) * vn                             # per unit mass
    rx, rz = -R * nx, -R * nz
    tx, tz = -nz, nx
    vt = (vx + w * -rz) * tx + (vz + w * rx) * tz
    Jt = max(-mu * Jn, min(mu * Jn, -vt / (1 + 1 / k)))
    vx += Jn * nx + Jt * tx
    vz += Jn * nz + Jt * tz
    w += (rx * Jt * tz - rz * Jt * tx) / (k * R * R)
    return vx, vz, w


def neo550_brake(v_pipe, ratio, wheel_r=0.0254, eff=0.9):
    """Brake-mode (shorted winding) resisting force at the nip rim when the pipe back-drives a still nip.
    NEO 550 published 0.97 N m stall at 100 A, 12 V -> Kt 0.0097 N m/A, R 0.12 ohm (catalogue-class)."""
    kt, r_ohm = 0.97 / 100, 12.0 / 100
    w_motor = v_pipe / wheel_r * ratio
    t_motor = min(kt * kt * w_motor / r_ohm, 0.97)
    return t_motor * ratio * eff / wheel_r


def run():
    c = coral()
    R, M, k = c["R"], c["M"], c["k"]
    ds = load("panDownselect")["settings"]
    feed = ds["feed"]
    seg = [(math.degrees(math.atan2(feed[i][1] - feed[i + 1][1], feed[i + 1][0] - feed[i][0])))
           for i in range(len(feed) - 1)]
    fp = load("fixedPath")
    arrival = 2.465696950433656  # premise-measurements.json <- comparison.json receiveScreen.arrivals.adverse.speedMps
    v_arr = arrival
    E_arr = rolling_energy(M, v_arr, k)

    # --- Fairness of the fixed-path test (comparison-02) -------------------------------------
    var = fp["fixedPath"]["variants"]["fixed-notch-undamped"]["config"]
    lip = var["profile"][-1][1]            # 14 mm front lip
    bump = 0.008                           # 8 mm rear bump in the notch variants
    tilt = -var["thetaDeg"]                # floor 8 deg downhill toward the exit
    fairness = dict(
        tested="Pan geometry frozen at the -8 deg dump pose: 14 mm front lip, optional 8 mm notch bump, "
               "passive brush damping, no actuator. Release only by a scripted 9 m/s^2 chassis stop.",
        arrivalRollingEnergyJ=E_arr,
        lipBarrierJ=M * G * lip,
        notchBarrierJ=M * G * bump,
        arrivalOverLipBarrier=E_arr / (M * G * lip),
        lipMountSpeedMps=step_mount_speed(lip, R, k),
        passiveLipHoldMps2=lip_hold_accel(lip, R, tilt),
        releaseStopMps2=var["release"]["stopDecel"],
        verdict=("Not a fair test of a fixed-geometry receiver. The arrival carries about "
                 f"{E_arr / (M * G * lip):.0f}x the energy needed to roll over the 14 mm lip, so the undamped "
                 "variant and the lightly damped c6 and c10 lost the pipe at receive in 12 of 12 cases. The damped "
                 "notch variants c15 and c22 and the floor-brush variants c10, c15 and c22 lost it in adverse carry "
                 "in 6 of 12 cases each (notch c15 lost the other 6 parked before release; floor-brush c10 lost all "
                 "12 in carry). In the remaining cases notch c22 and floor-brush c15 and c22 kept the pipe but could "
                 "not release it, because no variant had a release actuator. The test rejects a zero-actuator frozen "
                 "pan. It does not test a fixed pocket with a closed exit and a powered release. The FIRST 2025 KitBot "
                 "citation supports only the role (a Coral Station-fed Level 1 scorer), not any exit mechanism."),
        lossModes={k_: _modes(v_) for k_, v_ in fp["fixedPath"]["variants"].items()},
        variantsPassing={k_: v_["passCount"] for k_, v_ in fp["fixedPath"]["variants"].items()},
    )

    # --- A: guarded pan at stow ---------------------------------------------------------------
    stow = ds["stowDeg"]
    a_lift = G / math.tan(math.radians(stow))   # open face: floor normal leaves the pipe
    A = dict(
        liftoffForwardMps2=a_lift,
        liftoffSource="computed g/tan(stow); retention-review.json R1 records 3.569 m/s^2",
        unpowered="kept to 2 g opening contact; lost at 3 g/50 ms, 4 g, 5 g and the front strike (pan-stop-study-05 holds)",
        powered="kept to 8 g/50 ms and the front strike at 30 A, but the support test is interrupted 24-114 ms in every event (study-06)",
        custodyDependsOnPower=True,
        receivePeakStopNm=load("panStop05")["hold"]["P5-K5420-unpowered"]["pan"]["trips"]["nominal/coral-A/30"]["stowStopPeakNm"],
    )
    A["receivePeakStopN"] = A["receivePeakStopNm"] / 0.100  # 100 mm stop lever (appliedChanges.stowStop.leverMm)

    # --- B: fixed pocket with a closed two-roller exit nip ------------------------------------
    pocket_center_z = 0.60   # pocket centre; the exit over the 70 mm fence is at 0.67 m (delivery.py)
    drop = feed[0][1] - pocket_center_z
    v_pocket = math.sqrt(v_arr ** 2 + 2 * G * drop / (1 + k))
    E_pocket = rolling_energy(M, v_pocket, k)
    pad_travel = 0.025
    fence_h, hood_gap = 0.070, 0.010
    # Receive: a rolling pipe has topspin w = -v/R and k/(1+k) of its energy is spin. Friction at the
    # fence face can turn that spin into upward velocity (the pipe climbs the wall).
    w_roll = -v_pocket / R
    climb = []
    for beta in (0.0, 10.0, 20.0):
        for mu in (0.15, 0.35, 0.6):
            for e in (0.2, 0.5):
                vx, vz, w = wall_impact(v_pocket, w_roll, beta, mu, e, R, k)
                rise = max(vz, 0) ** 2 / (2 * G)
                climb.append(dict(undercutDeg=beta, mu=mu, e=e, upMps=round(vz, 3), riseMm=round(rise * 1e3, 1),
                                  reboundMps=round(-vx, 3), reachesNip=rise > hood_gap, clearsFence=rise > fence_h))
    # Check flap: after the pipe's rear passes the flap tip it travels (Lp - D) to the fence and back.
    D = 2 * R
    flap = []
    for Lp in (0.15, 0.20, 0.25):
        for e in (0.2, 0.5):
            back = [r["reboundMps"] for r in climb if r["undercutDeg"] == 20.0 and r["e"] == e and r["mu"] == 0.35][0]
            t_close = (Lp - D) / v_pocket + (Lp - D) / max(back, 1e-6)
            flap.append(dict(pocketLengthM=Lp, e=e, reboundMps=back, closeWithinMs=round(t_close * 1e3, 1)))
    B = dict(
        geometry=f"floor, side walls, fence {fence_h*1e3:.0f} mm tall (>= R = {R*1e3:.1f} mm) under the lower roller, "
                 f"hood {hood_gap*1e3:.0f} mm above the pipe, spring check flap behind the pocket; pocket centre "
                 f"{pocket_center_z} m so the exit over the fence is {pocket_center_z + fence_h:.2f} m",
        fenceIsWall=fence_h >= R,
        trappedWithoutPower=(fence_h >= R) and (hood_gap < fence_h),
        rearRollbackWithoutFlapMps2=G * math.tan(math.radians(seg[1])),
        rearEscapeWithoutFlapMps2=G * math.tan(math.radians(seg[0])),
        feedSegmentAnglesDeg=seg,
        arrivalAtPocketMps=v_pocket,
        arrivalEnergyJ=E_pocket,
        spinShareOfArrivalEnergy=k / (1 + k),
        padTravelM=pad_travel,
        fencePeakN=2 * E_pocket / pad_travel,
        receiveClimb=climb,
        nipBackDriveN={f"{r}:1": {f"{v} m/s": round(neo550_brake(v, r), 1) for v in (0.25, 0.5, 1.0)} for r in (9, 12)},
        checkFlap=flap,
        inertialNipLoadN={g_: M * g_ * G for g_ in (1, 3, 8)},
        custodyDependsOnPower=False,
    )

    # --- C: fixed pocket with a passive reef-contact-tripped gate ------------------------------
    C = dict(
        trappedWithoutPower=True,
        falseTripCases="Any front-bumper contact that strokes the trigger: the recorded front strike (11.4 rad/s pan "
                       "opening in A) and 3-8 g peer pulses would open the gate away from the reef.",
        custodyDependsOnPower=False,
    )
    # --- D: pivoting roller end effector -----------------------------------------------------
    D = dict(
        trappedWithoutPower="Only if the roller nip is self-locking; the arm itself needs a held stow like A.",
        custodyDependsOnPower=True,
    )
    return dict(coral=c, fairness=fairness, A=A, B=B, C=C, D=D)


if __name__ == "__main__":
    import json
    print(json.dumps(run(), indent=1, default=float))
