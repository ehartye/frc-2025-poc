"""Structural, rules, stop, spring and drive-fault checks for pan revision r3 (offline; writes structure.json).

Run from the project root after pan_geometry.py:
  python -I iterations/002/design/pan/pan_structure.py
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
G = 9.80665
E_AL, G_AL = 68.9e3, 26.0e3                 # MPa, 6061-T6
WORKING, FTY_6061, FTY_5052 = 120.0, 241.0, 159.0   # project working limit; base-metal minimum yields (ASTM B209/B221 T6, H32)
POST_WELD_001 = 105.0
RIVET_SHEAR_N = 1500.0                       # minimum rated single shear required of the 3/16 in structural blind rivet (procurement spec)
RIVNUT_PULLOUT_N = 1500.0                    # minimum published pull-out required of the #10-32 blind rivet nut in 1/16 in 6061-T6 (procurement gate; no sourced value yet)
SCREW10_TENSION_N, SCREW10_SHEAR_N = 12500.0, 7500.0   # #10-32 alloy steel socket screws (project basis, fastenerJoints)
BEARING_ALLOW = 1.5 * WORKING                # bearing allowable on the thinnest wall
TAU_AL = 0.6 * WORKING                       # working shear for thread stripping in 6061-T6
PAD_K = 300.0                                # N/mm per stow pad (pan-stop-study)
SOFT_PAD_T = 18.0                            # r3 soft-stop pad thickness (geometry.json SOFT_PAD_T)
CUT_LATENCY_S = 0.005                        # design trip-to-neutral latency for the -3.5 deg inhibit until bench measured
PIVOT = np.array([205.0, 580.0])
LUG_WEB = 5.0                                # stop-lug striking web behind the lip back face (geometry LUG_DEPTH)
DRIVE_INHIBIT_TEXT = ("Required in every mode: an absolute-encoder soft limit at -2 deg, plus an opening-direction drive inhibit at -3.5 deg on the pan controller's "
                      "reverse-limit input, independent of the roboRIO. The inhibit zeroes opening output only, never cuts closing output and never latches. "
                      "Idle mode coast. Trip-to-neutral <= 5 ms measured on the bench. The soft stop, its beam, the stow lug and the gearbox protection are sized "
                      "for F2/F3 with the inhibit 5 ms late and for the bumper strike; inspect the pads after any fault event")
SOURCES = {
    "ultraPlanetaryLoadRating": "REV UltraPlanetary load ratings (docs.revrobotics.com/rev-crossover-products/ultraplanetary/load-ratings) as recorded in the FRC wiki, wiki/authored/frc/2025-poc/reference/reefscape-grip-slide-drive-sources.md:65: 40 N m output-stage FAILURE, reduced by shock and cantilever loads. A failure value, not a working rating",
    "sparkMaxLimitInputs": "REV SPARK MAX data port (docs.revrobotics.com/brushless/spark-max/specs/data-port) as recorded in wiki/authored/frc/reference/limit-sensor-interface-boundaries.md:25: forward/reverse limit inputs neutralize output in the affected direction only and allow the opposite direction. Documented behaviour, not an executed firmware test; latency is not published there",
    "neo550": "NEO 550 0.97 N m stall, 100 A stall, 1.4 A free (wiki reefscape-grip-slide-drive-sources.md:65); 11000 rpm free speed is the package basis",
    "rotorInertia": "OPEN: no published NEO 550 rotor inertia is held in the project records or the wiki. 3e-6 kg m2 is an assumption; pan-stop-study-04 driveFault shows the post-cut gearbox torque scales with it (x2, x3). The slip clutch bounds the gearbox regardless",
    "rivetNutPullOut": "OPEN: no sourced #10-32 rivet-nut pull-out rating in 1/16 in 6061-T6 is held in the project records or the wiki. 1500 N is a procurement requirement; the selected SKU's published value must meet it before release",
    "Kt": "Kt 2.5 at a rivet hole in the web tube wall: textbook basis, Pilkey and Pilkey, Peterson's Stress Concentration Factors (3rd ed., Wiley 2008), chapter 4, finite-width plate with a central circular hole in tension (net-section Kt about 2.2-2.6 for d/W 0.1-0.4). Not captured in the project wiki; d/W here is about 0.25",
}


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def tube(b, t):
    i = (b ** 4 - (b - 2 * t) ** 4) / 12
    am = (b - t) ** 2
    j = 4 * am ** 2 * t / (4 * (b - t))
    return dict(I=i, S=i / (b / 2), A=b ** 2 - (b - 2 * t) ** 2, Am=am, J=j, t=t, b=b)


def beam(L, loads, fixed):
    """Prismatic beam on [0, L]; point loads (a, P); returns sampled M(y) and the deflection function, pinned or fixed ends."""
    ys = np.linspace(0, L, 2001)
    M = np.zeros_like(ys)
    for a, P in loads:
        b = L - a
        if fixed:
            Ma, Mb = -P * a * b ** 2 / L ** 2, -P * a ** 2 * b / L ** 2
            Ra = P * b ** 2 * (3 * a + b) / L ** 3
            M += np.where(ys < a, Ma + Ra * ys, Ma + Ra * ys - P * (ys - a))
        else:
            Ra = P * b / L
            M += np.where(ys < a, Ra * ys, Ra * ys - P * (ys - a))
    return ys, M


def deflection(L, a, P, y, EI, fixed):
    b = L - a
    if fixed:
        if y <= a:
            return P * b ** 2 * y ** 2 * (3 * a * L - (3 * a + b) * y) / (6 * L ** 3 * EI)
        return deflection(L, L - a, P, L - y, EI, fixed)
    if y <= a:
        return P * b * y * (L ** 2 - b ** 2 - y ** 2) / (6 * L * EI)
    return P * a * (L - y) * (2 * L * y - a ** 2 - y ** 2) / (6 * L * EI)


def torsion_internal(L, torques, y):
    """Fixed-fixed torsion: internal torque at y from point torques (a, T)."""
    total = 0.0
    for a, T in torques:
        Ta = T * (L - a) / L
        total += Ta if y < a else Ta - T
    return total


def cradle_check(name, sec, L, y0, xc, zc, loads, end_moments=()):
    """loads: list of (y_world, x, z, Fx, Fy, Fz) applied at world points; beam axis along Y."""
    vert = [(y - y0, -Fz) for y, x, z, Fx, Fy, Fz in loads]
    horiz = [(y - y0, -Fx) for y, x, z, Fx, Fy, Fz in loads]
    torq = [(y - y0, (z - zc) * Fx - (x - xc) * Fz) for y, x, z, Fx, Fy, Fz in loads]
    axial = sum(abs(l[4]) for l in loads)
    worst = None
    for fixed in (False, True):
        ys, Mv = beam(L, vert, fixed)
        _, Mh = beam(L, horiz, fixed)
        Mv = Mv + sum(m for m in end_moments)                     # conservative: end moment carried along the span
        for i in range(0, len(ys), 10):
            T = torsion_internal(L, torq, ys[i])
            sigma = (abs(Mv[i]) + abs(Mh[i])) / sec["S"] + axial / sec["A"]
            tau = abs(T) / (2 * sec["Am"] * sec["t"])
            vm = math.sqrt(sigma ** 2 + 3 * tau ** 2)
            if worst is None or vm > worst["vonMisesMPa"]:
                worst = dict(ends="fixed" if fixed else "pinned", yMm=float(ys[i] + y0), bendingVerticalNmm=float(Mv[i]), bendingHorizontalNmm=float(Mh[i]),
                             torsionNmm=float(T), axialN=axial, sigmaMPa=sigma, tauMPa=tau, vonMisesMPa=vm)
    worst["marginOnWorking"] = WORKING / worst["vonMisesMPa"] - 1
    worst["marginOnYield"] = FTY_6061 / worst["vonMisesMPa"] - 1
    worst["cradle"] = name
    return worst


def thread_strip_n(engagement_mm, major_mm=4.83):
    """Internal-thread stripping capacity in 6061-T6 at the working shear: about half the major-diameter cylinder is in shear."""
    return math.pi * major_mm * engagement_mm * 0.5 * TAU_AL


def root_moment_joint(F, M_nm):
    """r3 bulkhead end clip: 1 x 1 x 1/4 in angle, Z 502..577. Leg A (on the bulkhead aft face, X 213.65..220) is bolted
    through the bulkhead by 3 x #10-32 SHCS with locknuts at Y 144, Z 512/539.5/567 (in-plane moment -> bolt shear).
    Leg B (on the skin inner face over the post, X 194.6..220) carries 3 x #10-32 SHCS at X 205, Z 528/545/562 through the
    skin into the tapped 3/8 in post inner plate (out-of-plane moment -> screw tension against a heel)."""
    M = M_nm * 1000.0
    d_hole, t_bulk, t_clip, t_inner = 4.83, 4.7625, 6.35, 9.525
    direct = F / 3.0
    # leg A: bolt group in shear
    zA = (512.0, 539.5, 567.0); zc = sum(zA) / 3
    J = sum((z - zc) ** 2 for z in zA)
    vA = M * max(abs(z - zc) for z in zA) / J + direct
    legA = dict(bolts=3, boltZMm=list(zA), boltYMm=144.0, outerBoltShearN=vA, boltShearMargin=SCREW10_SHEAR_N / vA - 1,
                bulkheadBearingMPa=vA / (d_hole * t_bulk), bulkheadBearingMargin=BEARING_ALLOW / (vA / (d_hole * t_bulk)) - 1,
                clipBearingMPa=vA / (d_hole * t_clip), clipBearingMargin=BEARING_ALLOW / (vA / (d_hole * t_clip)) - 1,
                edgeDistancesMm=dict(toClipEnds=10.0, toLegEnd=144.0 - (161.0 - 25.4), headToLegBFace=(161.0 - 6.35) - 144.0 - 7.94 / 2),
                nuts="nylon-insert locknuts on the bulkhead forward face (open, reachable from below the floor)")
    # leg B: screw tension couple about the worse of the two heels (post bottom Z 520 or clip top Z 577)
    zB = (528.0, 545.0, 562.0)
    worst = 0.0
    for heel in (520.0, 577.0):
        arms = [abs(z - heel) for z in zB]
        worst = max(worst, M * max(arms) / sum(a * a for a in arms))
    T = worst + direct
    prying = 1.33
    engagement = t_inner - 0.5
    strip = thread_strip_n(engagement)
    b_prime = (220.0 - t_clip - 205.0) - d_hole / 2          # screw centre to leg A inner face, less half the hole (AISC T-stub b')
    pitch = 17.0
    leg_sigma = 6 * T * b_prime / (pitch * t_clip ** 2)
    legB = dict(screws=3, screwXMm=205.0, screwZMm=list(zB), screwTensionN=T, pryingFactor=prying, screwTensionWithPryingN=T * prying,
                screwMargin=SCREW10_TENSION_N / (T * prying) - 1, threadEngagementMm=engagement, threadStripN=strip,
                threadStripMargin=strip / (T * prying) - 1, legBendingMPa=leg_sigma, legBendingMargin=WORKING / leg_sigma - 1,
                headClearances=dict(screwToLegAInnerFaceMm=220.0 - t_clip - 205.0, socketHeadRadiusMm=7.94 / 2,
                                    flatHeadWouldNeedMm=9.78 / 2, screwToPostAftEdgeMm=205.0 - 190.0, holeToHexCornerMm=math.hypot(0, 580 - 562.0) - 12.72 / math.sqrt(3) - d_hole / 2),
                edgeDistancesMm=dict(lowestRowToPostBottom=528.0 - 520.0, highestRowToClipTop=577.0 - 562.0))
    # post plate stack in bending about X at the clip (non-composite bound and composite value)
    S_sep = 30.0 * (9.525 ** 2 + 9.525 ** 2 + 4.7625 ** 2) / 6
    S_comp = 30.0 * 23.8125 ** 2 / 6
    post = dict(nonCompositeMPa=M / S_sep, nonCompositeMargin=WORKING / (M / S_sep) - 1, compositeMPa=M / S_comp,
                note="three waterjet plates tied by 3 x #10-32 FHCS; the non-composite value governs")
    # bulkhead in-plane bending through the windows (flanges 12 mm) and beside the clip (full 78 mm)
    fl = 12.0; h = 78.0
    I_win = 2 * (BULK_T_MM * fl ** 3 / 12 + BULK_T_MM * fl * (h / 2 - fl / 2) ** 2)
    sig_win = M * (h / 2) / I_win
    bulk = dict(section="3/16 in plate, Z 500..578, windows Z 512..566", stressMPa=sig_win, margin=WORKING / sig_win - 1,
                besideClipMPa=M / (BULK_T_MM * h ** 2 / 6))
    governing = min((legA["boltShearMargin"], "leg A bolt shear"), (legA["bulkheadBearingMargin"], "bulkhead bearing at leg A bolts"),
                    (legB["screwMargin"], "leg B screw tension"), (legB["threadStripMargin"], "post inner plate thread stripping"),
                    (legB["legBendingMargin"], "leg B bending at the screws"), (post["nonCompositeMargin"], "post stack bending"),
                    (bulk["margin"], "bulkhead in-plane bending"))
    return dict(design="1 x 1 x 1/4 in 6061-T6 angle, 75 mm (Z 502..577), bolted through both legs", momentNm=M_nm, directShareN=direct,
                legA=legA, legB=legB, postStack=post, bulkhead=bulk, governingMargin=governing[0], governing=governing[1],
                supersedes="r2 clip: 2 x 3/16 in rivets in shear (margin -0.35 to -0.54), a #10 flat head that did not fit beside the leg, and a row 1 mm from the clip end")


BULK_T_MM = 4.7625


def off_post_paths(F, t16):
    """685 N lateral load at the cheek top front end (X 545, Z 690). The lower stringer is split at the post, so the ladder's
    horizontal cantilever is the upper stringer with the skin (path A). The floor is a deep in-plane diaphragm that holds the
    cheek's lower edge; the moment F x h about that edge goes back to the post as cheek torsion (path C). Paths share by stiffness."""
    E, Gm = E_AL, G_AL
    L, h = 545.0 - 220.0, 690.0 - 578.0
    As, ys = 122 * 3.175, 161 + 3.175 / 2
    At, yt = t16["A"], 164.175 + 9.525
    na = (As * ys + At * yt) / (As + At)
    I_A = As * (ys - na) ** 2 + 122 * 3.175 ** 3 / 12 + t16["I"] + At * (yt - na) ** 2
    c = max(183.225 - na, na - 161.0)
    k_A = 3 * E * I_A / L ** 3
    J_tube, J_skin = t16["J"], 122 * 3.175 ** 3 / 3
    J_tot = 2 * J_tube + J_skin
    k_C = Gm * J_tot / (L * h ** 2)
    F_A = F * k_A / (k_A + k_C); F_C = F - F_A
    sig_A = F_A * L * c / I_A
    T = F_C * h
    tau_tube = T * J_tube / J_tot / (2 * t16["Am"] * t16["t"])
    tau_skin = T * J_skin / J_tot * 3.175 / J_skin
    vm = math.sqrt(3) * max(tau_tube, tau_skin)
    floor_t = 2.0
    M_f = F_C * (545.0 - 205.0)
    sig_f = M_f * 161.0 / (floor_t * 322.0 ** 3 / 12)
    tau_f = 1.5 * F_C / (floor_t * 322.0)
    clip_shear = F_C / 2
    all_A = F * L * c / I_A
    return dict(loadAt="cheek top front end (X 545, Z 690); lower stringer split at the post",
                pathA=dict(member="upper stringer + skin as a horizontal cantilever from the post", I_mm4=I_A, cMm=c, stiffnessNPerMm=k_A, shareN=F_A,
                           stressMPa=sig_A, margin=WORKING / sig_A - 1, ifItCarriedAllMPa=all_A),
                pathC=dict(member="cheek torsion (two 3/4 x 1/16 in stringer tubes + skin) about the floor-held lower edge, back to the post",
                           J_mm4=J_tot, stiffnessNPerMm=k_C, shareN=F_C, torqueNm=T / 1000, tubeShearMPa=tau_tube, skinShearMPa=tau_skin,
                           vonMisesMPa=vm, margin=WORKING / vm - 1,
                           floorDiaphragm=dict(thicknessMm=floor_t, depthMm=322.0, momentNm=M_f / 1000, bendingMPa=sig_f, shearMPa=tau_f),
                           floorClips=dict(nearLoad="clips 470 and 530 on the loaded side", shearEachN=clip_shear, rivetMargin=RIVET_SHEAR_N / clip_shear - 1,
                                           returnAtPost="clips 140 and 260 on both cheeks plus the root clips, about F_C / 8 each")),
                correctedScreen=dict(r2LadderI_mm4=37918, bothStringersI_mm4=29106, bothStringersMPa=127.0,
                                     note="the r2 package I (37,918 mm4) was wrong; both stringers give 29,106 mm4 and 127 MPa, but the lower stringer is split at the post so only path A's upper-stringer section is continuous"),
                governingMargin=min(WORKING / sig_A - 1, WORKING / vm - 1),
                note="Elastic share by stiffness; the floor diaphragm itself is far from critical. Contact logs place the guide impacts at the post; the replay must still report contact positions")


def soft_stop_structure(P, contact_x):
    """r3 soft stop on one side: solid 3/4 in square beam on the X250 cradle top, clamped by a #10-32 FHCS at X 231 (aft of the
    cradle) and a #10-32 SHCS at X 268 (forward of it) through a 1/4 in keeper plate under the cradle. Heel: cradle front-top edge X 262.7."""
    heel, x_aft, x_fwd, cr_aft = 250.0 + 12.7, 231.0, 268.0, 250.0 - 12.7
    b = 19.05
    M_heel = P * (contact_x - heel)
    S = b ** 3 / 6
    S_net = (b - 4.98) * b ** 2 / 6
    M_net = P * (contact_x - x_fwd)
    T_aft = M_heel / (heel - x_aft)
    keeper_M = T_aft * (cr_aft - x_aft)
    keeper_S = b * 6.35 ** 2 / 6
    R_heel = P + T_aft
    wall = b * 1.5875
    post_A = 19.05 ** 2 - (19.05 - 2 * 1.5875) ** 2
    return dict(designPadForceN=P, beam=dict(section="3/4 in square 6061-T6 bar", momentAtHeelNm=M_heel / 1000, stressMPa=M_heel / S, margin=WORKING / (M_heel / S) - 1,
                                             netAtFrontBoltMPa=M_net / S_net, netMargin=WORKING / (M_net / S_net) - 1),
                aftBolt=dict(tensionN=T_aft, margin=SCREW10_TENSION_N / T_aft - 1, nut="nylon-insert locknut under the keeper (open below the cradle)"),
                keeper=dict(section="1/4 in x 3/4 in 6061-T6 plate", momentNm=keeper_M / 1000, stressMPa=keeper_M / keeper_S, margin=WORKING / (keeper_M / keeper_S) - 1),
                cradleFrontWall=dict(reactionN=R_heel, bearingMPa=R_heel / wall, margin=BEARING_ALLOW / (R_heel / wall) - 1,
                                     note="beam heel on the cradle front-top corner, carried by the 1/16 in front wall in bearing; crippling to be confirmed in the native structural route"),
                post=dict(compressionMPa=P / post_A, margin=WORKING / (P / post_A) - 1),
                rivetNuts="none: r2's two rivet nuts per beam (margins -0.21 in F2 and -0.34 in F3) are removed",
                governingMargin=min(WORKING / (M_heel / S) - 1, WORKING / (M_net / S_net) - 1, SCREW10_TENSION_N / T_aft - 1,
                                    WORKING / (keeper_M / keeper_S) - 1, BEARING_ALLOW / (R_heel / wall) - 1))


def stow_bracket_rivnuts(F_pad, n_in, contact):
    """Stow-stop bracket on the X110 cradle rear face (X 97.3), two #10-32 screws into rivet nuts at Z 479 and 491 (heel at Z 473.5)."""
    face, heel = 97.3, 473.475
    e = face - contact[0]
    Fx, Fz = -F_pad * n_in[0], -F_pad * n_in[1]               # force from the pan on the pad
    M = abs(Fz) * e + abs(Fx) * (contact[1] - heel)
    zs = (479.0 - heel, 491.0 - heel)
    T = M * max(zs) / sum(z * z for z in zs)
    return dict(padForceN=F_pad, momentAboutHeelNmm=M, topScrewTensionN=T, rivetNutPullOutRequiredN=RIVNUT_PULLOUT_N,
                pullOutMargin=RIVNUT_PULLOUT_N / T - 1, shearEachN=abs(Fz) / 2, source=SOURCES["rivetNutPullOut"])


def main():
    geo = json.loads((HERE / "geometry.json").read_bytes())
    duty = json.loads((ROOT / "artifacts/red2-integrated/pan-duty-summary.json").read_bytes())
    mech = json.loads((ROOT / "artifacts/red2-integrated/pan-mechanics.json").read_bytes())
    rules = json.loads((ROOT / "kickoff/rules-v0.json").read_bytes())
    register = json.loads((ROOT / "kickoff/rule-register-v0.json").read_bytes())
    parts = {p["id"]: p for p in geo["parts"]}
    rec = geo["reconciliation"]
    stop, soft = geo["stopGeometry"], geo["softStopGeometry"]
    m_pan = rec["achievedIncludingOffAxisJoiningHardware"]["massKg"] + rec["onAxisAddedKg"]
    U = rec["achievedIncludingOffAxisJoiningHardware"]["firstMomentUKgMm"] / rec["achievedIncludingOffAxisJoiningHardware"]["massKg"]
    W = rec["achievedIncludingOffAxisJoiningHardware"]["firstMomentWKgMm"] / rec["achievedIncludingOffAxisJoiningHardware"]["massKg"]
    I_pan = rec["achievedIncludingOffAxisJoiningHardware"]["inertiaKgM2"]
    out = {"schema": "red2/pan-structure@1", "revision": geo["revision"], "inputs": {
        "iterations/002/design/pan/geometry.json": sha(HERE / "geometry.json"),
        "artifacts/red2-integrated/pan-duty-summary.json": sha(ROOT / "artifacts/red2-integrated/pan-duty-summary.json"),
        "artifacts/red2-integrated/pan-mechanics.json": sha(ROOT / "artifacts/red2-integrated/pan-mechanics.json"),
        "kickoff/rules-v0.json": sha(ROOT / "kickoff/rules-v0.json"),
        "kickoff/rule-register-v0.json": sha(ROOT / "kickoff/rule-register-v0.json"),
        "iterations/002/design/pan/pan_structure.py": sha(Path(__file__))},
        "method": "Closed-form beam, torsion and statics screens with base-metal minimum properties. These screens precede, and do not replace, the native structural-route review."}

    # ------------------------------------------------------------ rules (M8)
    lim = rules["limits"]
    facts = {f["id"]: f for f in register["facts"]}
    env = geo["poseEnvelopesMm"]
    fp = geo["framePerimeterXYMm"]
    chamfer = 57.8
    perim = 2 * (fp[2] - fp[0]) + 2 * (fp[3] - fp[1]) - 4 * chamfer + 2 * math.sqrt(2) * chamfer
    reach = max(e[3] for e in env.values())
    beyond = reach - fp[2]
    out["rulesCheck"] = dict(
        artifact="kickoff/rules-v0.json (identical bytes in frc-2025-poc/kickoff/rules-v0.json, sha256 b32ba544...)",
        startHeight=dict(entry="rules-v0.json /limits/height; rule-register-v0.json facts/start-height, locator R104", limitIn=lim["height"], limitMm=lim["height"] * 25.4,
                         stowTopMm=env["stow"][5], marginMm=lim["height"] * 25.4 - env["stow"][5], verdict="pass",
                         basis="the stowed pan is the starting configuration; floor Z=0 is the factory tire contact"),
        startPerimeter=dict(entry="rules-v0.json /limits/perimeter; rule-register facts/start-perimeter, locator R101-R104", limitIn=lim["perimeter"], frameOutlineMm=round(perim, 1),
                            frameOutlineIn=round(perim / 25.4, 2), stowInsideFrame=env["stow"][0] > fp[0] and env["stow"][3] < fp[2],
                            verdict="pass", basis="frame outline from the census perimeter bounds with two 57.8 mm rear chamfers; nothing outside it at stow"),
        extension=dict(entry="rule-register-v0.json facts/extension (locator R102, R105): '18-inch extension limit refers to the perimeter' (status unmodeled in the register)",
                       limitMm=18 * 25.4, maxReachXMm=reach, framePerimeterXMm=fp[2], beyondPerimeterMm=round(beyond, 3), poseOfMaxReach=max(env, key=lambda k: env[k][3]),
                       marginMm=round(18 * 25.4 - beyond, 3), verdict="pass" if beyond <= 18 * 25.4 else "fail"),
        notInArtifact="rules-v0.json and its register carry no in-match height limit and no direction-specific extension rules. Do not infer them from a historical game; record them in the rule register if the owner adds them.",
        geometryChanged=False)

    # ------------------------------------------------------------ sections
    cr = tube(25.4, 1.5875)
    L = 2 * 232.475
    zc = 471.975 + 12.7
    out["sections"] = dict(cradle1x1x116=cr, stringer34x34x116=tube(19.05, 1.5875), strut12x12x116=tube(12.7, 1.5875))

    # ------------------------------------------------------------ load cases on the cradles
    CR_TOP = 497.375
    w_half = m_pan * G / 2

    def carrier_reactions(F, y):
        """In-plane carrier statics: bearing force (Fx, Fz) at (205, 580) -> landing reactions at X=110 and 250 on the cradle tops."""
        Fx, Fz = F
        r250 = (95.0 * Fz - (580 - CR_TOP) * Fx) / 140.0
        return [(y, 110.0, CR_TOP, Fx / 2, 0.0, Fz - r250), (y, 250.0, CR_TOP, Fx / 2, 0.0, r250)]

    def gearbox(Td):
        f = Td * 1000 / 140.0
        return [(214.0, 110.0, CR_TOP, 0.0, 0.0, f), (214.0, 250.0, CR_TOP, 0.0, 0.0, -f)]

    strut_alpha = math.atan2(36.0, 560.0 - 489.525)

    def axial(Fy):
        """Starboard thrust Fy at the pivot height: strut horizontal share, base shear and strut vertical reaction on the carrier."""
        Sy = Fy * (580 - CR_TOP) / (560 - CR_TOP)
        base = Sy - Fy
        Sz = Sy / math.tan(strut_alpha)
        rows = [(-195.0, 110.0, CR_TOP, 0.0, -base / 2, 0.0), (-195.0, 250.0, CR_TOP, 0.0, -base / 2, 0.0)]
        for xs in (160.0, 228.0):                              # strut vertical reaction into the carrier, then to the landings
            r250 = Sz / 2 * (xs - 110.0) / 140.0
            rows += [(-195.0, 110.0, CR_TOP, 0.0, 0.0, -(Sz / 2 - r250)), (-195.0, 250.0, CR_TOP, 0.0, 0.0, -r250)]
        return rows, dict(strutHorizontalTotalN=Sy, carrierBaseShearN=base, strutVerticalTotalN=Sz, strutAxialEachN=Sy / 2 / math.sin(strut_alpha))

    cases = {}
    for row in duty["signedPeaks"]:
        for sgn in (1, -1):
            loads = carrier_reactions((sgn * row["portRadialXZ_N"][0], sgn * row["portRadialXZ_N"][1] - w_half), 195.0)
            loads += carrier_reactions((sgn * row["starboardRadialXZ_N"][0], sgn * row["starboardRadialXZ_N"][1] - w_half), -195.0)
            loads += gearbox(row["requiredActuatorTorqueY_Nm"])
            ax, _ = axial(sgn * row["pipeForceN"][1])
            loads += ax
            cases[f"feed {row['metric']} sign {sgn:+d}"] = loads
    for sgn in (1, -1):
        ax, info_axial = axial(sgn * mech["movingGuide"]["forceN"])
        cases[f"moving guide 685 N axial sign {sgn:+d}"] = ax + carrier_reactions((0, -w_half), 195.0) + carrier_reactions((0, -w_half), -195.0)
    pad_peak_nm = 10.196520230184463
    n_in = np.array(stop["padPushDirection"]); c = np.array(stop["contactAtStowMm"])
    fpad = pad_peak_nm / 0.1 / 2
    pad_loads = [(sy * stop["contactYmm"], c[0], c[1], -fpad * n_in[0], 0.0, -fpad * n_in[1]) for sy in (1, -1)]
    react = fpad * n_in
    cases["stow stop peak unpowered (pads only)"] = pad_loads + carrier_reactions((0, -w_half), 195.0) + carrier_reactions((0, -w_half), -195.0)
    cases["stow stop peak unpowered (pads + bearing reaction)"] = pad_loads + carrier_reactions((react[0], react[1] - w_half), 195.0) + carrier_reactions((react[0], react[1] - w_half), -195.0)
    # inertial cases: braking 1.50 m/s2 (commissioning), traction-limited 0.8 g longitudinal and lateral, tip acceleration, all with gravity.
    # The pan rotation torque they produce at stow is reacted by the stow pads (closing) or the drive (opening, <= 15.4 N m).
    cg_stow = (205.0 + U * math.cos(math.radians(70)) - W * math.sin(math.radians(70)), 580.0 + U * math.sin(math.radians(70)) + W * math.cos(math.radians(70)))
    inertial = {}
    for label, ax, ay in (("braking 1.50 m/s2", 1.50, 0.0), ("traction 0.8 g longitudinal", 0.8 * G, 0.0), ("traction 0.8 g lateral", 0.0, 0.8 * G)):
        for sgn in (1, -1):
            fx, fy = -sgn * m_pan * ax, -sgn * m_pan * ay
            torque = -(cg_stow[1] - 580.0) / 1000 * fx                  # theta-positive (closing) torque about the pivot from fx at CG height
            loads = carrier_reactions((fx / 2, -w_half), 195.0) + carrier_reactions((fx / 2, -w_half), -195.0)
            if torque > 0:                                              # closing: stow pads
                f = torque / 0.1 / 2
                loads += [(sy * stop["contactYmm"], c[0], c[1], -f * n_in[0], 0.0, -f * n_in[1]) for sy in (1, -1)]
            else:
                loads += gearbox(torque)
            if fy:
                ax_rows, _ = axial(fy)
                loads += ax_rows
            cases[f"inertial {label} sign {sgn:+d}"] = loads
            inertial[f"{label} sign {sgn:+d}"] = dict(panForceN=[fx, fy], panTorqueAtStowNm=torque,
                                                     reactedBy="stow pads" if torque > 0 else "drive (within the 15.4 N m stall capacity)")
    tip_alpha = 6 * math.radians(72) / 1.6 ** 2
    inertial["pan motion (tip)"] = dict(peakAlphaRadS2=tip_alpha, torqueNm=I_pan * tip_alpha, note="smoothstep tip, 72 deg in 1.6 s; small against the feed peaks")
    bumper_a = 12340.0 / 68.55
    bumper_torque = m_pan * bumper_a * (cg_stow[1] - 580.0) / 1000
    bumper_impulse = bumper_torque * 0.020
    bumper_omega = bumper_impulse / (I_pan + 3.0e-6 * 54.83289124668435 ** 2)
    inertial["abuse: 12.34 kN bumper strike (commissioning.md lower-frame case)"] = dict(
        decelerationMs2=bumper_a, panTorqueNm=bumper_torque, durationS=0.020, angularImpulseNms=bumper_impulse, panRateRadS=bumper_omega,
        kineticEnergyJ=0.5 * (I_pan + 3.0e-6 * 54.83289124668435 ** 2) * bumper_omega ** 2,
        finding="A front strike opens the stowed pan to about this rate, and it reaches the soft stop with energy comparable to fault F3. A rear strike drives it into the stow pads at about this torque for 20 ms. Treat it as an abuse case: inspect the pads and soft-stop posts after any bumper strike near this level. It is not a pan-support design case.")

    # ------------------------------------------------------------ drive fault and soft stop (OD-5)
    ratio, kt, i_lim = 54.83289124668435, 0.97 / 100, 30.0
    j_rotor = 3.0e-6
    I_tot = I_pan + j_rotor * ratio ** 2
    k_soft_pad = 250.0
    k_soft = 2 * k_soft_pad * 1e3 * (soft["leverMm"] / 1000) ** 2
    c_soft = 2 * 790.0 * (soft["leverMm"] / 1000) ** 2
    spring_rate = 1.5 / math.radians(72)

    R_motor, ke = 12.0 / 100.0, 12.0 / (11000 * 2 * math.pi / 60)      # NEO 550 winding resistance and back-EMF constant (stall and free-speed basis)

    def fault(theta0, omega0, eta, v_bus=11.0, label="", cut_deg=None, latency=0.0, drive=True, idle="coast"):
        """Rigid pan on the soft stop. Below cut_deg the opening-direction output is inhibited `latency` s after the crossing
        (non-latching, closing output allowed). idle='brake' shorts the windings once inhibited (rejected mode, for comparison)."""
        th, om, t, dt = math.radians(theta0), omega0, 0.0, 1e-5
        t_cross = None
        w_free = 11000 * 2 * math.pi / 60 * v_bus / 12
        peak = dict(stopNm=0.0, compressionMm=0.0, thetaMinDeg=theta0, contactSpeedRadS=None, gearboxOutputNm=0.0, rateAtCutRadS=None, cutToContactMs=None)
        t_contact = None
        while t < 2.0:
            wm = abs(om) * ratio
            t_v = 0.97 * v_bus / 12 * max(0.0, 1 - wm / w_free)
            if cut_deg is not None and t_cross is None and math.degrees(th) <= cut_deg:
                t_cross = t; peak["rateAtCutRadS"] = om
            inhibited = t_cross is not None and t >= t_cross + latency
            if not drive:
                tm = 0.0
            elif inhibited:
                # opening output is zeroed; coast leaves no motor torque, brake mode shorts the windings
                tm = (min(ke * wm / R_motor, 100.0) * kt * ratio * (1.0 if om < 0 else -1.0)) if idle == "brake" else 0.0
            else:
                tm = -min(kt * i_lim, t_v) * ratio * eta
            tg = -m_pan * G * ((U * math.cos(th) - W * math.sin(th)) / 1000)
            ts = spring_rate * max(0.0, th - math.radians(-2.0))
            d = math.radians(-5.0) - th
            tstop = max(0.0, k_soft * d - c_soft * om) if d > 0 else 0.0
            if d > 0 and peak["contactSpeedRadS"] is None:
                peak["contactSpeedRadS"] = om; t_contact = t
            alpha = (tm + tg + ts + tstop) / I_tot
            # gearbox output carries motor torque plus the rotor's decelerating torque
            tgear = abs(tm - j_rotor * ratio ** 2 * alpha)
            peak["gearboxOutputNm"] = max(peak["gearboxOutputNm"], tgear)
            if inhibited:
                peak["gearboxAfterCutNm"] = max(peak.get("gearboxAfterCutNm", 0.0), tgear)
            peak["stopNm"] = max(peak["stopNm"], tstop)
            peak["compressionMm"] = max(peak["compressionMm"], max(0.0, d) * soft["leverMm"])
            peak["thetaMinDeg"] = min(peak["thetaMinDeg"], math.degrees(th))
            om += alpha * dt; th += om * dt; t += dt
            if d > 0 and om > 0 and tstop == 0.0:
                break
        if t_cross is not None and t_contact is not None:
            peak["cutToContactMs"] = (t_contact - t_cross) * 1e3
        peak.update(label=label, startThetaDeg=theta0, startRateRadS=omega0, efficiency=eta, busV=v_bus, driveCutDeg=cut_deg, cutLatencyS=latency,
                    drive=drive, idleMode=idle, padForceEachN=peak["stopNm"] / (soft["leverMm"] / 1000) / 2, padStrain=peak["compressionMm"] / SOFT_PAD_T)
        return peak

    lat = CUT_LATENCY_S
    faults = [fault(-2.0, 0.0, 1.0, label="F1 loss of position control at the commanded dump; ideal opening inhibit at -3.5 deg, coast", cut_deg=-3.5),
              fault(-2.0, 0.0, 1.0, label=f"F1 with the inhibit acting {lat * 1e3:g} ms late", cut_deg=-3.5, latency=lat),
              fault(-2.0, 0.0, 1.0, label="F1 without the inhibit (control)"),
              fault(34.0, -1.18, 1.0, label="F2 loss of control at mid-tip; ideal inhibit", cut_deg=-3.5),
              fault(34.0, -1.18, 1.0, label=f"F2 loss of control at mid-tip; inhibit {lat * 1e3:g} ms late (design)", cut_deg=-3.5, latency=lat),
              fault(70.0, 0.0, 1.0, label="F3 runaway from stow; ideal inhibit", cut_deg=-3.5),
              fault(70.0, 0.0, 1.0, label=f"F3 runaway from stow; inhibit {lat * 1e3:g} ms late (design)", cut_deg=-3.5, latency=lat),
              fault(70.0, 0.0, 1.0, label="F3 runaway from stow; ideal inhibit with BRAKE idle mode (rejected)", cut_deg=-3.5, idle="brake"),
              fault(70.0, -bumper_omega, 1.0, label="B1 front bumper strike (12.34 kN) opening the stowed pan, drive unpowered", drive=False)]
    f1 = faults[0]
    design_faults = [faults[i] for i in (4, 6, 8)]
    soft_design = max(design_faults, key=lambda q: q["padForceEachN"])
    stall_required = (30.0 - 1.4) * 0.00984 * ratio
    worst_ideal = max(faults[i]["gearboxOutputNm"] for i in (0, 3, 5))
    worst_late = max(faults[i]["gearboxOutputNm"] for i in (1, 4, 6))
    clutch = dict(setMinNm=16.5, setMaxNm=18.0, breakawayToleranceFactor=1.2)
    out["gearbox"] = dict(
        decision="Gearbox protection (owner decision): fit a mandatory 1/2 in hex friction slip clutch set between the worst normal-duty torque and the output-stage rating, OR show a documented output-stage capacity of at least 26 N m (F3 with an ideal inhibit). r3 records both routes and the latency condition on the second",
        normalDutyCapNm=stall_required, formula="(30 - 1.4) A x 0.00984 N m/A x 54.83 (the cap the controller can hold in any stall, including the feed peak)",
        f1PeakBeforeCutNm=f1["gearboxOutputNm"], f1PeakAfterCutNm=f1.get("gearboxAfterCutNm", 0.0),
        worstFaultIdealInhibitNm=worst_ideal, worstFaultInhibitLateNm=worst_late, inhibitLatencyS=lat,
        brakeModeF3Nm=faults[7]["gearboxOutputNm"],
        clutchRoute=dict(setting=clutch, requiredDocumentedCapacityNm=clutch["setMaxNm"] * clutch["breakawayToleranceFactor"],
                         placement="between REV-41-1620 and the port drive hex; the gearbox plate stands off outboard by the clutch length with the six-hole pattern kept. Needs its own drive checked placement (open)",
                         basis="never slips at the 15.4 N m normal-duty cap; caps the gearbox at <= 18 N m in F1-F3 and B1 whatever the inhibit latency or rotor inertia"),
        capacityRoute=dict(requiredNm=26.0, validOnlyIf=f"measured inhibit trip-to-neutral latency <= {faults[5]['cutToContactMs']:.2f} ms (F3 travel from -3.5 deg to the pad); at {lat * 1e3:g} ms the worst fault reaches {worst_late:.1f} N m"),
        supplierRating=SOURCES["ultraPlanetaryLoadRating"],
        verdict=f"The published 40 N m is a failure value. It is above the 26 N m ideal-inhibit demand but below the {worst_late:.1f} N m late-inhibit demand, so the clutch route governs until the latency is measured. The roster carries the clutch as required",
        currentCap="unchanged at 30 A (lowering it breaks the 29.62 A feed peak)",
        idleMode=f"COAST. Brake mode shorts the windings at the inhibit and raises the F3 gearbox torque to {faults[7]['gearboxOutputNm']:.1f} N m (rotor inertia {j_rotor:g} kg m2); coast is also the unpowered state the replay passes",
        rotorInertiaAssumptionKgM2=j_rotor, rotorInertiaSource=SOURCES["rotorInertia"])
    sn = np.array(soft["padPushDirection"]); sc = np.array(soft["contactAtSoftStopMm"])
    t_motor = kt * i_lim * ratio

    def soft_case(pad_each, drive_torque):
        pads = [(sy * 177.525, sc[0], sc[1], -pad_each * sn[0], 0.0, -pad_each * sn[1]) for sy in (1, -1)]
        br = 2 * pad_each * sn
        return pads + (gearbox(-drive_torque) if drive_torque else []) + carrier_reactions((br[0] / 2, br[1] / 2 - w_half), 195.0) + carrier_reactions((br[0] / 2, br[1] / 2 - w_half), -195.0)
    cases["soft stop F1 drive fault"] = soft_case(f1["padForceEachN"], t_motor)
    cases["soft stop F3 runaway, inhibit late (design)"] = soft_case(faults[6]["padForceEachN"], min(t_motor, clutch["setMaxNm"]))
    cases["soft stop B1 front bumper strike, unpowered"] = soft_case(faults[8]["padForceEachN"], 0.0)
    rear_pad = bumper_torque / 0.1 / 2                       # rear strike drives the stowed pan into the stow pads for 20 ms
    rear_design = 2.0 * rear_pad
    cases["rear bumper strike into the stow pads (x2 dynamic)"] = [(sy * stop["contactYmm"], c[0], c[1], -rear_design * n_in[0], 0.0, -rear_design * n_in[1]) for sy in (1, -1)] \
        + carrier_reactions((rear_design * n_in[0], rear_design * n_in[1] - w_half), 195.0) + carrier_reactions((rear_design * n_in[0], rear_design * n_in[1] - w_half), -195.0)

    # inlet post torsion into the stringer extension, reacted as an end moment on cradle 110 (conservative unbraced bound)
    inlet_T = mech["fixedInlet"]["forceN"] * (750.0 - 493.525)
    results = []
    for name, loads in cases.items():
        for label, xc in (("Upper cradle 110", 110.0), ("Upper cradle 250", 250.0)):
            mine = [l for l in loads if (l[1] < 180.0) == (xc < 180.0)]
            if not mine:
                continue
            r = cradle_check(label, cr, L, -232.475, xc, zc, mine)
            r["case"] = name; results.append(r)
    r = cradle_check("Upper cradle 110", cr, L, -232.475, 110.0, zc, [], end_moments=(inlet_T,))
    r["case"] = "inlet post lateral 94 N, unbraced cantilever torsion as an end moment"; results.append(r)
    gov = max(results, key=lambda q: q["vonMisesMPa"])
    out["cradleCheck"] = dict(section="1 x 1 x 1/16 in 6061-T6, span 464.95 mm between the stringer inboard faces",
                              allowable=dict(workingMPa=WORKING, baseMetalFtyMPa=FTY_6061),
                              governing=gov, cases=sorted(results, key=lambda q: -q["vonMisesMPa"]),
                              faultCasesIncluded="r3 designs the cradles for F3 (inhibit late, clutch-limited drive reaction), B1 front bumper strike and the rear-strike stow-pad load",
                              limits="Pinned and fixed end bounds; torsion fixed at both ends; combined corner stress with von Mises. End-clip bolts, wall bearing and clip prying are screened below and remain open for the native structural route.")

    # ------------------------------------------------------------ stow-stop stiffness in series (M1)
    EI = E_AL * cr["I"]
    a = 232.475 - stop["contactYmm"]
    def stiffness(fixed):
        dv = deflection(L, a, 1.0, a, EI, fixed) + deflection(L, L - a, 1.0, a, EI, fixed)   # both pads loaded
        r_arm = math.hypot(c[0] - 110.0, c[1] - zc)
        tw = (r_arm * 1.0) * (a * (L - a) / L) / (G_AL * cr["J"]) * r_arm
        return 1.0 / (dv * abs(n_in[1]) + tw)
    seat_I = 22.0 * 6.35 ** 3 / 12
    k_bracket = 3 * E_AL * seat_I / 16.0 ** 3
    lug_I = 40.0 * 5.0 ** 3 / 12
    k_lug = 3 * E_AL * lug_I / 10.0 ** 3
    rows = {}
    for fixed in (False, True):
        k_cr = stiffness(fixed)
        k_series = 1 / (1 / PAD_K + 1 / k_bracket + 1 / k_cr + 1 / k_lug)
        rows["fixed ends" if fixed else "pinned ends"] = dict(padNPerMm=PAD_K, bracketNPerMm=k_bracket, cradleNPerMm=k_cr, lugNPerMm=k_lug,
                                                              seriesNPerMm=k_series, torsionalNmPerRad=2 * k_series * 1e3 * 0.01, softerThanDesignFactor=PAD_K / k_series)
    lip_I = 65.0 * 2.0 ** 3 / 12
    k_lip = 3 * 70.3e3 * lip_I / 90.0 ** 3
    out["stowStopStiffness"] = dict(bounds=rows, designTorsionalNmPerRad=6000,
                                    replayTolerance="pan-stop-study F4/F6 passed at 2x softer; inside 2x keeps the study's result. Re-replay only if the measured series stiffness is softer than 3000 N m/rad",
                                    rejectedLipPatch=dict(lipCantileverNPerMm=k_lip, basis="2 mm 5052 lip 65 mm wide loaded 90 mm above the root: the study's original patch would make the stop about 24x softer, which is why the stop moved to the cheek lug"))

    # ------------------------------------------------------------ carrier, knee struts, landing clips (OD-1 A)
    _, info = axial(mech["movingGuide"]["forceN"])
    st = out["sections"]["strut12x12x116"]
    Ls = math.hypot(36.0, 560 - 489.525)
    pcr = math.pi ** 2 * E_AL * st["I"] / Ls ** 2
    carrier_m = mech["movingGuide"]["forceN"] * 20.0
    out["carrierAndStruts"] = dict(axialLocation="thrust on the starboard carrier only; the port bearing floats axially on the drive hex",
                                   guideAxialN=mech["movingGuide"]["forceN"], **info,
                                   strutStressMPa=info["strutAxialEachN"] / st["A"], strutBucklingN=pcr, strutBucklingMargin=pcr / info["strutAxialEachN"] - 1,
                                   carrierOutOfPlaneAboveStrutMPa=carrier_m / (100 * 6.35 ** 2 / 6),
                                   landingClipBendingMPa=(abs(info["carrierBaseShearN"]) / 2 * 12.7) / (19.05 * 3.175 ** 2 / 6),
                                   basis="carrier pinned at its landings, propped by two knee struts at Z=560; without struts the 685 N axial load at 82.6 mm puts about 970 MPa into a 1/8 in landing clip")

    # ------------------------------------------------------------ r2 riveted cheek ladder: guide, root moment, ribs, fatigue, rivets (OD-4)
    F = mech["movingGuide"]["forceN"]
    impact = duty["signedPeaks"]
    t34_16, t34_18 = tube(19.05, 1.5875), tube(19.05, 3.175)
    z_load = 690.0                                        # top-edge load height (w ~ 110); panImpact contact at w ~ 120, u ~ -16 (x ~ 189)
    M0 = F * (z_load - 580.0) / 1000
    span = 140.5                                          # longest upper-stringer span between webs / post (X 359.5 -> 490.5 plus end)
    up_local = F * span / 4 / t34_16["S"]
    web_lever = z_load - (580.0 + 19.05)                   # one web carries all of it (3/4 x 3/4 x 1/8 in, 3.175 mm walls)
    web_nom = F * web_lever / t34_18["S"]
    Kt, q_notch = 2.5, 0.8
    Kf = 1 + q_notch * (Kt - 1)
    cycles = 2 * 2 * 400                                  # two trips x two guide contacts x 400 matches and practices (design life)
    Se = 200.0                                            # 6061-T6 R=-1 fully reversed strength at ~1e4 cycles (MIL-HDBK-5 trend, smooth)
    Su = 290.0
    sa = sm = Kf * web_nom / 2
    goodman = 1 / (sa / Se + sm / Su)
    # explicit root-moment path: post screws -> post -> bulkhead end clip -> bulkhead -> opposite post / bearing couple
    post_rows = 668.0 - 595.0
    post_screw = M0 * 1000 / post_rows / 2
    m_post_root = F * (z_load - 545.0) / 1000
    root_joint = root_moment_joint(F, m_post_root)
    sig_bulk = root_joint["bulkhead"]["stressMPa"]
    # off-post: a 685 N load at the front end, with the lower stringer split at the post and the floor as a diaphragm
    off_post = off_post_paths(F, t34_16)
    # floor clips: diaphragm shear plus a bounded share of the root moment (prying)
    clips = 6
    share = 0.05
    clip_arm = 578.0 - 571.6 + 1.0
    clip_tension = share * M0 * 1000 / clip_arm / 2
    prying_Q = 1.33
    rivet_tension_rating = 2000.0
    out["cheekLadder"] = dict(
        design="r2 riveted sheet-and-tube ladder: 1/8 in 6061-T6 skin, 3/4 x 3/4 x 1/16 in upper and lower stringers, three 3/4 x 3/4 x 1/8 in webs, a socket post stack of three waterjet plates and a 3/16 in pivot bulkhead under the floor, tied by a 1 x 1 x 1/4 in end clip. No welds and no monolith; every part is a 2D waterjet or router cut or a saw cut and drill, except the bent lip root clip (hand brake, fabrication-plan.json)",
        loadLocation="pan-duty-summary/pan-shop-plan impacts: panImpact at pan-local (u=-16, w=120), i.e. at the socket post; the 685 N moving-guide force is applied at w=110 on the post line (design) and at the front end (sensitivity)",
        upperStringerLocal=dict(spanMm=span, stressMPa=up_local, marginOnWorking=WORKING / up_local - 1),
        oneWebCarriesAll=dict(section="3/4 x 3/4 x 1/8 in 6061-T6 web tube", leverMm=web_lever, nominalMPa=web_nom, marginOnWorking=WORKING / web_nom - 1,
                              rootDetail="the web root is the rivet line in the skin-side wall (extrusion corner radius about 1.6 mm); Kt 2.5 at the rivet holes",
                              peakAtHoleMPa=Kt * web_nom, peakMarginOnYield=FTY_6061 / (Kt * web_nom) - 1),
        fatigue=dict(cycles=cycles, Kf=Kf, alternatingMPa=sa, meanMPa=sm, fullyReversedStrengthAt1e4CyclesMPa=Se, ultimateMPa=Su, goodmanFactor=goodman,
                     basis="R=0 pulsating guide load, notch factor from Kt 2.5 with q 0.8, modified Goodman. The 200 MPa is a smooth-specimen fully reversed strength at about 1e4 cycles (conservative for the 1.6e3-cycle life), not an endurance limit; aluminium has none",
                     KtSource=SOURCES["Kt"]),
        rootMomentPath=dict(rootMomentNm=M0, path="cheek top -> skin and upper stringer -> socket post plate stack (4 x #10-32 FHCS in rows Z 595/668) -> post root -> 1 x 1 x 1/4 in end clip (3 x #10-32 SHCS through leg B and the skin into the post inner plate; 3 x #10-32 SHCS through leg A and the bulkhead with locknuts) -> pivot bulkhead -> opposite post; the pan-level moment closes as a +/-193 N couple at the two bearings",
                            postScrewTensionN=post_screw, postScrewMargin=SCREW10_TENSION_N / post_screw - 1,
                            postRootMomentNm=m_post_root, joint=root_joint,
                            bulkheadStressMPa=sig_bulk, bulkheadMargin=WORKING / sig_bulk - 1,
                            bearingCoupleN=M0 * 1000 / 390.0),
        offPostSensitivity=off_post,
        floorClips=dict(clips=clips, lateralShearEachN=F / clips, momentShareAssumed=share, coupleArmMm=clip_arm, rivetTensionN=clip_tension,
                        pryingFactor=prying_Q, rivetTensionWithPryingN=clip_tension * prying_Q, requiredRivetTensileN=rivet_tension_rating,
                        margin=rivet_tension_rating / (clip_tension * prying_Q) - 1,
                        basis="The 1/16 in clips are far more flexible in rotation than the post and bulkhead path, so they carry the diaphragm shear plus at most about 5 % of the root moment. Their rivets are checked in tension with an AISC T-stub prying factor"),
        rivetSpec=dict(shearN=RIVET_SHEAR_N, tensileN=rivet_tension_rating, heads="dome heads outside the pipe footprint; dimpled-sheet countersunk rivets where flush is needed (no knife-edge countersink in 2 mm sheet)"),
        materials=dict(cheek="6061-T6 plate and tube, base metal (no heat input)", floorLipRootClip="5052-H32"))
    W_imp = json.loads((ROOT / "artifacts/red2-integrated/pan-shop-plan.json").read_bytes())["impacts"]["panImpact"]["normalN"]
    sig_panel = 3 * W_imp / (2 * math.pi * 3.175 ** 2) * (1.3 * math.log(40.0 / 15.0) + 1)
    out["cheekLadder"]["skinPanel"] = dict(patchLoadN=W_imp, source="pan-shop-plan impacts.panImpact normalN", model="clamped circular panel a=40 mm, patch r0=15 mm (Roark)",
                                           stressMPa=sig_panel, marginOnWorking=WORKING / sig_panel - 1)
    # stop lug plate stack under the stow impact and the rear bumper strike
    pad_peak = max(10.196520230184463, 7.1023623) / 0.1 / 2
    dyn = 2.0
    Fl = max(pad_peak, rear_pad) * dyn
    lug_M = Fl * 30.0 / 1000
    web_S = 40.0 * LUG_WEB ** 2 / 6
    out["stopLugBlock"] = dict(padForceEachN=pad_peak, rearBumperPadForceEachN=rear_pad, dynamicFactor=dyn, designForceN=Fl, momentAtFastenersNm=lug_M,
                               screwTensionN=lug_M * 1000 / 17.0, screwMargin=SCREW10_TENSION_N / (lug_M * 1000 / 17.0) - 1,
                               dowelShearN=Fl / 2, dowelMargin=8000 / (Fl / 2) - 1,
                               strikingWebMPa=Fl * 10.0 / web_S, strikingWebMargin=WORKING / (Fl * 10.0 / web_S) - 1,
                               designBasis="max(stow pad peak 51 N, rear bumper strike 296 N) x 2 dynamic; r2 used 102 N",
                               design="waterjet stack: 7/8 in striking plate, the 1/8 in skin lobe and a 1/2 in outboard plate, tied by 2 x #10-32 FHCS into the tapped outboard plate and 2 x 4 mm steel dowels through all three")
    out["stowStopBracket"] = stow_bracket_rivnuts(Fl, n_in, c)
    out["softStopStructure"] = soft_stop_structure(soft_design["padForceEachN"], sc[0])
    out["softStopStructure"]["designCase"] = soft_design["label"]

    # ------------------------------------------------------------ closing spring (OD-3)
    d, ID, legs = 2.5, 18.0, 20.0
    D = ID + d
    k_nmm = spring_rate * 1000
    Na = 207e3 * d ** 4 / (64 * D * k_nmm)
    Nb = Na - 2 * legs / (3 * math.pi * D)
    C = D / d
    Ki = (4 * C ** 2 - C - 1) / (4 * C * (C - 1))
    sig = Ki * 32 * 1500 / (math.pi * d ** 3)
    mts = 2211 / d ** 0.145
    defl = math.radians(72)
    Dw = D * Nb / (Nb + defl / (2 * math.pi))
    out["closingSpring"] = dict(
        law="1.5 N m at stow, linear to zero at theta=-2 deg (72 deg of opening); moving leg lifts off its driver beyond -2 deg so the soft-stop region carries no spring torque",
        rateNmPerRad=spring_rate, rateNmmPerDeg=k_nmm * math.pi / 180,
        candidate=dict(classOf="music-wire torsion spring, ASTM A228, made to print or the nearest catalog part within +/-10 % rate",
                       wireMm=d, insideDiameterMm=ID, meanDiameterMm=D, activeCoilsEquivalent=Na, bodyCoils=round(Nb, 2), legs="two tangential straight legs, 20 mm",
                       windDirection="wound so that closing the pan winds the spring (diameter reduces under load)",
                       bodyLengthMm=(Nb + 1) * d, insideDiameterAtStowMm=Dw - d, arborSleeveOdMm=16.0, bendingStressAtStowMPa=sig, wireMinTensileMPa=mts,
                       stressRatio=sig / mts, allowable="0.78 MTS static; 0.6 MTS light cyclic service; this design is at the ratio above",
                       procurementNote="No supplier catalog lookup was performed for this revision. Procurement must match the rate, ID, leg geometry and wind, or order made to print."),
        mounting="On the starboard idler stub extended to Y=-229: thrust collar Y -202..-208, spring body -208..-223.4 on a 16 mm arbor sleeve, notched retaining/driver collar -223..-229; fixed leg on a #10 shoulder screw in the starboard carrier.")

    # ------------------------------------------------------------ soft stop (OD-5)
    env_min = min(q["thetaMinDeg"] for q in faults if q["idleMode"] == "coast")
    out["softStop"] = dict(geometry=soft, padThicknessMm=SOFT_PAD_T, padStiffnessEachNPerMm=k_soft_pad, torsionalNmPerRad=k_soft, dampingNmsPerRad=c_soft,
                           gapAtCommandedDumpDeg=3.0, rotorInertiaAssumptionKgM2=j_rotor, reflectedRotorKgM2=j_rotor * ratio ** 2,
                           faults=faults,
                           designCase=dict(case=soft_design["label"], peakStopNm=soft_design["stopNm"], padForceEachN=soft_design["padForceEachN"], padStrain=soft_design["padStrain"],
                                           thetaMinDeg=soft_design["thetaMinDeg"], overTravelBeyondSoftDeg=-5.0 - soft_design["thetaMinDeg"]),
                           f1Case=dict(case=f1["label"], padForceEachN=f1["padForceEachN"], padStrain=f1["padStrain"], thetaMinDeg=f1["thetaMinDeg"],
                                       cutToContactMs=f1["cutToContactMs"], gearboxAfterCutNm=f1.get("gearboxAfterCutNm", 0.0)),
                           faultEnvelopeMinDeg=env_min, jointLimitDeg=-9.0, jointLimitMarginDeg=env_min - (-9.0),
                           replayLawUnchanged="4050 N m/rad and 12.8 N m s/rad at -5 deg, 90 mm lever: the 18 mm pad keeps 250 N/mm per pad, so the replay law is unchanged",
                           controlsRequirement=DRIVE_INHIBIT_TEXT,
                           sweepNote="geometry.json sweeps the moving pan from -9 to 70.1 deg, which covers the fault envelope and the stow peak overtravel")
    out["driveInhibit"] = dict(
        thetaDeg=-3.5, direction="opening only (theta decreasing); closing output always allowed; non-latching (re-arms as soon as the cam leaves the switch)",
        evidence="pan-stop-study-04: the directional inhibit never trips in normal running (results identical to the no-cut control C4); a non-directional cut fails 6 of 12 resets (fullCutControl)",
        hardware=dict(sensor="snap-action roller-lever limit switch (SPDT, normally-closed wiring to the controller reverse-limit input so an open lead also inhibits opening)",
                      flag="Port reverse-limit cam: 1/8 in waterjet plate on the port drive hex, base radius 20 mm, lobe 23.5 mm over pan theta -14..-3.5 deg with 2 deg ramps",
                      seat="Seat pan reverse-limit switch on the port bearing carrier outboard face (geometry.json deferredSeats); roller at world angle -90 deg about the pivot",
                      controller=SOURCES["sparkMaxLimitInputs"]),
        travel=dict(lobeRiseMm=3.5, rampDeg=2.0, rampMmAtRoller=round(2 * math.pi * 21.75 * 2 / 360, 3), tripToleranceDeg=0.4,
                    switchHeldOverDeg=[-14.0, -3.5], overtravelBeyondJointLimitDeg=5.0,
                    cutToPadAt90mmMm=round(math.radians(1.5) * 90, 3), note="the switch operating point must sit inside the 3.5 mm rise; the lobe holds it operated past the -9 deg joint limit, so overtravel cannot release it"),
        responseTime=dict(requiredMaxMs=lat * 1e3, designAssumptionMs=lat * 1e3,
                          f1TravelMs=f1["cutToContactMs"], f3TravelMs=faults[5]["cutToContactMs"],
                          basis="trip-to-neutral must be bench-measured (switch operate time plus controller limit response). The structure and the clutch are sized with the inhibit acting 5 ms late, so the inhibit is credited only for slow faults such as F1"),
        idleMode="coast (brake mode raises the F3 gearbox torque; see gearbox.idleMode)")

    # ------------------------------------------------------------ node forces for the current design (M9) and fastener joints
    def shares(loads, xc):
        """Pinned cradle between the stringer faces: the force each end passes to its stringer (port, starboard)."""
        yl, yr = -232.475, 232.475
        mine = [l for l in loads if (l[1] < 180.0) == (xc < 180.0)]
        out_ = {}
        for comp in (3, 5):
            tot = sum(l[comp] for l in mine)
            right = sum(l[comp] * (l[0] - yl) for l in mine) / (yr - yl)
            out_[comp] = (right, tot - right)
        fy = sum(l[4] for l in mine)
        return dict(port=np.array([out_[3][0], fy / 2, out_[5][0]]), starboard=np.array([out_[3][1], fy / 2, out_[5][1]]))

    def truss(sy, f0, f1):
        n0, n1 = np.array([110.0, 242.0 * sy, 480.0]), np.array([250.0, 242.0 * sy, 480.0])
        g = lambda x, y: np.array([x, y * sy, 187.325])
        members = [(0, g(110, 145)), (0, g(110, 212)), (1, g(250, 145)), (1, g(250, 212)), (0, n1), (0, g(250, 145))]
        A = np.zeros((6, 6))
        for j, (node, far) in enumerate(members):
            here = n0 if node == 0 else n1
            u = (far - here) / np.linalg.norm(far - here)
            A[node * 3:node * 3 + 3, j] += u
            if j == 4:
                A[3:6, j] -= u
        bvec = -np.concatenate([f0, f1])
        return np.linalg.solve(A, bvec)          # tension positive

    strut_feet = {}
    def strut_foot(loads_name):
        return strut_feet.get(loads_name, np.zeros(3))
    member_rows, clip_rows, landing_rows = [], [], []
    for name, loads in cases.items():
        sh110, sh250 = shares(loads, 110.0), shares(loads, 250.0)
        for side, sy in (("port", 1), ("starboard", -1)):
            f0, f1 = sh110[side].copy(), sh250[side].copy()
            if side == "starboard":
                fy = sum(l[4] for l in loads if abs(l[0] + 195.0) < 1)   # carrier base shear rows on the starboard carrier
                if fy:
                    Sy = -fy / 0.319 * 1.319 if abs(fy) > 0 else 0.0
                    Sz = Sy / math.tan(strut_alpha)
                    for xs in (160.0, 228.0):
                        k1 = (xs - 110.0) / 140.0
                        foot = np.array([0.0, Sy / 2, Sz / 2])
                        f0 += foot * (1 - k1); f1 += foot * k1
            forces = truss(sy, f0, f1)
            member_rows.append(dict(case=name, side=side, memberForcesN=forces.tolist(), maxAbsN=float(np.max(np.abs(forces)))))
            for xc, sh in ((110.0, f0), (250.0, f1)):
                clip_rows.append(dict(case=name, side=side, cradle=xc, endForceN=float(np.linalg.norm(sh))))
        for l in loads:
            if abs(l[2] - CR_TOP) < 1e-6:
                landing_rows.append(dict(case=name, y=l[0], x=l[1], forceN=float(math.hypot(l[3], l[5])), upliftN=float(max(0.0, l[5]))))
    worst_member = max(member_rows, key=lambda r: r["maxAbsN"])
    node_members = max((abs(r["memberForcesN"][i]) for r in member_rows for i in (0, 2)))
    member_force = max(node_members, max(abs(r["memberForcesN"][4]) for r in member_rows) * 0 + node_members)
    wall_bearing = 4.83 * 1.5875 * 1.5 * WORKING
    worst_clip = max(clip_rows, key=lambda r: r["endForceN"])
    worst_land = max(landing_rows, key=lambda r: r["forceN"])
    worst_uplift = max(landing_rows, key=lambda r: r["upliftN"])
    strut_each = info["strutAxialEachN"]
    head_leg_M = strut_each * math.cos(strut_alpha) * 6.3
    out["fastenerJoints"] = dict(
        basis="#10-32 alloy SHCS/FHCS: 12.5 kN tension, 7.5 kN single shear. Bearing allowable is 1.5 x 120 MPa on the thinnest wall. Every screw that lands in a closed 1/16 in tube wall goes into a #10-32 blind rivet nut (roster rows), never a locknut; nuts are used only where the far side is open (bulkhead clip leg A, soft-stop keeper)",
        rivetNut=dict(article="#10-32 aluminium or steel flanged blind rivet nut, 0.5-2.0 mm grip, for a 1/16 in 6061-T6 wall", requiredPullOutN=RIVNUT_PULLOUT_N, source=SOURCES["rivetNutPullOut"]),
        cradleEndClips=dict(screwsPerClip=4, host="2 into rivet nuts in the cradle side wall, 2 into rivet nuts in the stringer inboard wall", worst=worst_clip,
                            perScrewN=worst_clip["endForceN"] / 2, tubeWallBearingN=wall_bearing,
                            margin=wall_bearing / (worst_clip["endForceN"] / 2) - 1, pullOutMarginIfAllTension=RIVNUT_PULLOUT_N / (worst_clip["endForceN"] / 2) - 1),
        landingClips=dict(screwsPerClip=2, host="1 into the tapped carrier or gearbox plate, 1 into a rivet nut in the cradle top wall", worstShear=worst_land, worstUplift=worst_uplift, cradleWallBearingN=wall_bearing,
                          shearMargin=wall_bearing / max(worst_land["forceN"], 1e-9) - 1, rivnutPullOutRequiredN=RIVNUT_PULLOUT_N,
                          upliftMargin=RIVNUT_PULLOUT_N / max(worst_uplift["upliftN"], 1e-9) - 1),
        strutHeadClips=dict(strutAxialEachN=strut_each, clip="3/4 x 3/4 x 1/4 in 6061-T6 angle (r2; the r1 1/8 in leg reached about 175 MPa)",
                            legMomentNmm=head_leg_M, legStressMPa=head_leg_M / (19.05 * 6.35 ** 2 / 6), legMargin=WORKING / (head_leg_M / (19.05 * 6.35 ** 2 / 6)) - 1,
                            carrierScrewShearN=strut_each / 2, carrierBearingMargin=4.83 * 6.35 * 1.5 * WORKING / (strut_each / 2) - 1,
                            plugScrewN=strut_each, plugScrewMargin=12500 / strut_each - 1),
        strutFootPlugs=dict(rivets=2, shearEachN=strut_each / 2, margin=RIVET_SHEAR_N / (strut_each / 2) - 1))
    out["trussMemberForces"] = dict(method="pin-jointed side truss (members 0-5 of inset-pan-truss layout) loaded by the cradle end shares and the starboard strut feet for every cradle case above",
                                    worst=worst_member, nodeMembers0and2MaxN=node_members, previousOld2xDutyN=2418.0)
    out["trussNodes"] = dict(design="plugged butt node: a 12.7 mm plug fitted to the member bore, four 3/16 in blind rivets (two through each of two opposite walls), one #10-32 SHCS down through the stringer bottom wall (access hole in the top wall)",
                             memberForceN=member_force, source="trussMemberForces (current r2 cases; replaces the stale inset-pan-truss 2x-duty 2418 N)",
                             rivetShearMargin=4 * RIVET_SHEAR_N / member_force - 1, wallBearingMPa=member_force / 4 / (4.8 * 1.5875),
                             wallBearingMarginOn1p5Working=1.5 * WORKING / (member_force / 4 / (4.8 * 1.5875)) - 1,
                             screwTensionMargin=12500 / member_force - 1, compression="bears on the stringer underside through the member end face",
                             copedMembers="members 1/3/5 are coped onto members 0/2 below Z 468. Their gussets go on the rear X faces below the stringer, outside the cradle envelope (cradle bottom Z 471.975)")
    ext = out["sections"]["stringer34x34x116"]
    tau_ext = inlet_T / (2 * ext["Am"] * ext["t"])
    Mext = mech["fixedInlet"]["forceN"] * 0.2405 * 0.1575 / 0.398
    out["inletTruss"] = dict(postLateralN=mech["fixedInlet"]["forceN"], postMomentAtFootNmm=inlet_T,
                             extensionTorsionMPa=tau_ext, extensionBendingMPa=Mext * 1000 / ext["S"],
                             portalExpectationNmm=mech["fixedInlet"]["forceN"] * (750 - 493.525) / 4,
                             endClipNote="The unbraced bound puts the post moment into the X110 cradle end clip as roughly a 1.4 kN bolt couple. The two chute crossmembers make the inlet a portal frame (about 6 N m per foot). Qualify the end-clip bolt row for the portal value and keep the crossmembers in the release.",
                             legs="rear leg to Structural perimeter 2 top, side leg to Structural perimeter 0/4 top; landing pads 1/4 in with two #10-32 each; legs under 160 N")
    rows_m = [("off-post 685 N at the cheek front end", out["cheekLadder"]["offPostSensitivity"]["governingMargin"]),
              ("root-moment joint: " + root_joint["governing"], root_joint["governingMargin"]),
              ("soft-stop structure (F3, inhibit late)", out["softStopStructure"]["governingMargin"]),
              ("web tube peak at its rivet holes, on yield", out["cheekLadder"]["oneWebCarriesAll"]["peakMarginOnYield"]),
              ("cradles: " + gov["case"], gov["marginOnWorking"]),
              ("stow bracket rivet-nut pull-out (rear bumper x2)", out["stowStopBracket"]["pullOutMargin"]),
              ("landing-clip rivet-nut uplift", out["fastenerJoints"]["landingClips"]["upliftMargin"]),
              ("stop lug striking web (rear bumper x2)", out["stopLugBlock"]["strikingWebMargin"]),
              ("skin panel under the pan impact", out["cheekLadder"]["skinPanel"]["marginOnWorking"]),
              ("floor-clip rivets with prying", out["cheekLadder"]["floorClips"]["margin"])]
    out["worstMargins"] = [dict(check=k, margin=v) for k, v in sorted(rows_m, key=lambda r: r[1])]
    (HERE / "structure.json").write_text(json.dumps(out, indent=1, default=float) + "\n", encoding="utf-8")
    print(json.dumps(dict(rules={k: v.get("verdict") for k, v in out["rulesCheck"].items() if isinstance(v, dict)},
                          height=out["rulesCheck"]["startHeight"]["marginMm"], ext=out["rulesCheck"]["extension"]["marginMm"],
                          gov={k: gov[k] for k in ("cradle", "case", "ends", "vonMisesMPa", "marginOnWorking", "marginOnYield")},
                          worst=out["worstMargins"], soft=out["softStopStructure"], joint=root_joint, stiff=rows, f=[{k: q[k] for k in ("label", "stopNm", "padForceEachN", "padStrain", "thetaMinDeg", "gearboxOutputNm")} for q in faults],
                          ladder={k: v for k, v in out["cheekLadder"].items() if k not in ("design", "loadLocation", "materials", "rivetSpec")}, lug=out["stopLugBlock"], gearbox=out["gearbox"],
                          fasteners=out["fastenerJoints"], members=out["trussMemberForces"]["nodeMembers0and2MaxN"], nodes=out["trussNodes"]), indent=1, default=float))


if __name__ == "__main__":
    main()
