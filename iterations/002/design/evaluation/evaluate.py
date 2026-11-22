"""Run every screen and write evaluation.json (revision 2, after the adversarial review).

    python -I -B evaluate.py
"""
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import budgets  # noqa: E402
import custody  # noqa: E402
import delivery  # noqa: E402
from sources import identities, load  # noqa: E402

G = 9.80665

ARCH = {
    "A": dict(name="Guarded tipping pan (r3, current)",
              summary="Rotating 430 x 322 mm pan on a NEO 550 / 54.8:1 drive; receives at 70 deg stow against a "
                      "spring-preloaded stop, tips to -2 deg so the pipe slides off over the trough."),
    "B": dict(name="Fixed pocket, two-roller nip, low-speed launch",
              summary="Fixed tray and inlet; the pipe settles in an enclosed pocket (centre 0.60 m) against a 70 mm "
                      "fence leaning 20 deg toward the pocket, low-friction face on compliant backing, under a "
                      "two-roller nip; one NEO 550 (12:1) lifts it over the fence and launches it at 1.3 m/s "
                      "(closed loop) from 0.67 m, inside the frame perimeter."),
    "B'0": dict(name="B pocket and nip, roll-off from the frame edge (KitBot-style placement)",
                summary="As B, then a short down-sloping exit lip rolls the pipe off the frame edge at low speed."),
    "B'x": dict(name="B pocket and nip, roll-off from a 100 mm one-time-deployed exit lip",
                summary="As B'0, but a polycarbonate exit lip deploys once at match start so the pipe leaves 100 mm "
                        "beyond the frame perimeter, above the trough lip, at 0.55 m and 0.6-1.5 m/s."),
    "C": dict(name="Fixed pocket with a passive reef-tripped gate",
              summary="Same pocket; a spring gate is released by a plunger touching the reef; gravity roll-out."),
    "D": dict(name="Pivoting roller end effector",
              summary="Short arm with an intake/eject roller pair that receives at the station and swings over the trough."),
}


def gravity_exit_speed(z_store, z_exit, k):
    return math.sqrt(2 * G * max(z_store - z_exit, 0) / (1 + k))


def window_of(de, opt, z0, ang):
    for w in de["speedWindows"]:
        if (w["option"], w["exitCentreZ"], w["angleDeg"]) == (opt, z0, ang):
            return w
    return None


def point(de, opt, z0, v, ang):
    for p in de["points"]:
        if (p["option"], p["exitCentreZ"], p["launchMps"], p["angleDeg"]) == (opt, z0, v, ang):
            return p
    return None


def main():
    cu, de, bu = custody.run(), delivery.run(), budgets.run()
    c = cu["coral"]
    feed_top = load("panDownselect")["settings"]["feed"][0][1]
    c_exit = gravity_exit_speed(feed_top, 0.67, c["k"])
    rules = {f["id"]: f for f in load("ruleRegister")["facts"]}

    wB = window_of(de, "B", 0.67, 0.0)
    wBx = window_of(de, "B'x", 0.55, 0.0)
    best_B0 = max((w for w in de["speedWindows"] if w["option"] == "B'0"), key=lambda w: w["halfWidthPct"])
    p19 = point(de, "B", 0.67, 1.9, 0.0)
    p19up = point(de, "B", 0.67, 1.9, 10.0)

    table = {
        "custody": {
            "A": "powered only: unpowered lost at >= 3 g opening contact; open-face liftoff above "
                 f"{cu['A']['liftoffForwardMps2']:.2f} m/s^2; support interrupted 24-114 ms in every powered event",
            "B": "geometric if the fence leans 20 deg toward the pocket with mu <= 0.35: a vertical fence lets arrival "
                 "spin lift the pipe 73-92 mm (over the 70 mm fence). Brake-mode nip resists ~79 N at 0.5 m/s when "
                 "powered. Check flap must re-close within 38-91 ms (150-200 mm pocket)",
            "B'0": "as B", "B'x": "as B", "C": "as B at carry; lost on any false trip of the reef plunger",
            "D": "roller nip plus a held arm; arm opening under impact like A",
        },
        "deliveryWindow": {
            "A": "1.83 mm post-contact support margin; a 12 mm dock change flipped 2 of 12 cases (spatial replay)",
            "B": f"every 0-100 mm stand-off under every e, mu at {wB['band'][0]}-{wB['band'][1]} m/s "
                 f"(+/-{wB['halfWidthPct']} %); at 1.9 m/s only {p19['robustCount']}/11 stand-offs robust, "
                 f"{p19up['robustCount']}/11 at +10 deg (planar screen)",
            "B'0": f"best band +/-{best_B0['halfWidthPct']} % ({best_B0['band']} m/s): falls short of the lip; "
                   "does not remove stand-off sensitivity",
            "B'x": f"every 0-100 mm stand-off at {wBx['band'][0]}-{wBx['band'][1]} m/s (+/-{wBx['halfWidthPct']} %)",
            "C": f"lossless gravity from the {feed_top} m inlet gives {c_exit:.2f} m/s at the 0.67 m exit, but storing "
                 "at the inlet conflicts with receive and the trip is from rest: marginal",
            "D": "placed directly over the trough; not screened",
        },
    }
    m, v_ = bu["mass"], bu["voltage"]
    keys = ["A", "B", "B'x", "C", "D"]
    comparison = {}
    for k in keys:
        comparison[k] = dict(
            upperKg=round(m[k]["upperKg"], 2), r103MarginKg=round(m[k]["marginKg"], 2), massKind=m[k]["kind"],
            mechPeakA=v_[k]["mechPeakA"], dockedBusV=round(v_[k]["dockedBusV"], 2),
            movingBusV=round(v_[k]["movingBusV"], 2), governing=bu["loads"][k]["text"],
            mechanismS=round(bu["cycle"][k]["mechanismS"], 2),
            fabricated=bu["fabrication"][k]["fabricated"], purchased=bu["fabrication"][k]["purchased"],
            custody=table["custody"][k], delivery=table["deliveryWindow"][k])
    comparison["A"]["recordedConservativeImpactV"] = 8.81
    comparison["B'0"] = dict(screenedOut=True, delivery=table["deliveryWindow"]["B'0"])

    out = dict(
        schema="red2/architecture-evaluation@2",
        revision=2,
        supersedes="evaluation.json r1 (commit 66748a4)",
        scope="Offline preliminary comparison. No Fusion, no edits to preliminary/, pan/ or chassis/.",
        asymmetry=("A's custody and delivery come from a spatial 12-case field replay with peers, clock, current "
                   "limit and finite panels. B-D come from planar screens (impulse disc against the trough section) "
                   "and bottom-up estimates. A's figures are deeper evidence than B's; B's advantages are "
                   "predictions to be confirmed by the bench gate below, not equal-basis results."),
        date="2026-11-22",
        inputs=identities(),
        role=dict(
            mission="Two TELEOP trips (match s 18-153): receive one station-issued CORAL (4.5 in OD x 11.875 in pipe, "
                    f"{c['M']:.3f} kg upper bound) at the station face, carry it, place it in red L1 trough sector A then B, "
                    "keep it credited, observe the score and return to a ready, serviceable state.",
            requirements=[
                "R1 receive: accept a pipe arriving at up to 2.47 m/s (adverse admission) rolling axis-across from the station",
                "R2 carry: keep custody through route accelerations, 3-8 g peer pulses and bumper strikes",
                "R3 deliver: leave the pipe supported in the low trough (lip top 457.2 mm, 15 deg floor, riser 518 mm) over the real dock scatter",
                "R4 reset: ready for the next receive within the trip clock",
                "R5 fit: R103 mass, R101-R104 perimeter and 42 in start height, R102/R105 18 in extension, weak-pack bus >= 9 V",
                "R6 service: repair or swap at a pit in a typical student shop",
            ],
            throughput="Two pieces in 135 s. Mechanism time is not the discriminator; custody and delivery robustness are.",
        ),
        rules=dict(
            register="experiments/six-wheel/kickoff/rule-register-v0.json",
            G412=rules["launch"], extension=rules["extension"], startHeight=rules["start-height"],
            notes=["B launches the pipe (about 0.1 s airborne): G412 requires a bumper partially in our reef zone at "
                   "release, so the eject command must be interlocked on reef-zone pose; the register marks G412 "
                   "unmodeled.",
                   "B'x places by roll-off; whether a roll-off counts as launched under G412 is an open rules "
                   "question. Its deployed lip and A's 179 mm dumped reach both fall under R102/R105 (18 in).",
                   "Stowed height 0.9405 m is under the registered 42 in (1.067 m) start height."]),
        architectures=ARCH,
        fixedPathFairness=cu["fairness"],
        custody=cu, delivery=de, budgets=bu, table=table, comparison=comparison,
    )
    out["evidenceMaturity"] = {
        "A": "high modelled (spatial 12-case replays, stop studies 01-06, three reviews, r3 package); no physical test",
        "B": "planar screens in this folder only",
        "B'x": "planar screens in this folder only",
        "C": "planar screens only; no known practice for a reef-tripped latch",
        "D": "common FRC practice; no project evidence",
        "kitBotCitation": "wiki reference/ingestion-coverage-2026-11-16.md:63 supports the ROLE (a Coral Station-fed "
                          "Level 1 scorer) only; it is not evidence for a two-roller launch or any exit geometry.",
    }
    out["rankings"] = {
        "time": {"B": 1, "B'x": 2, "C": 3, "A": 4, "D": 5},
        "risk": {"B'x": 1, "B": 2, "A": 3, "D": 4, "C": 5},
        "complexity": {"C": 1, "B": 2, "B'x": 3, "D": 4, "A": 5},
        "bestPractice": {"B'x": 1, "D": 2, "B": 3, "A": 4, "C": 5},
        "maintainability": {"B": 1, "B'x": 2, "C": 3, "D": 4, "A": 5},
    }
    out["recommendation"] = dict(
        select="B family (fixed pocket + nip), exit chosen at the bench: B'x if the deployed lip passes, else B",
        confidence="about 55 % that the B family meets the role better than A; low-medium that it passes the bench "
                   "gate first time",
        doNotDiscard="Do NOT discard or stop maintaining the r3 pan package until bench gates G1-G4 pass. "
                     "If they fail, A remains the selected architecture and the r3 fixes continue.",
        invalidatesIfGatesPass="Most of the r3 pan package: moving pan, drive, clutch, cam and switch, spring, stow and "
                               "soft stops, root-moment joint, plate-stack posts and bulkhead; the C2/C3/P3/titanium "
                               "mass work becomes unnecessary.",
        mostConsequentialUntested={
            "A": "unpowered custody fails at >= 3 g opening contact; the selected K5420 support margin is +1.9 um over "
                 "its own criterion (-3.2 um unpowered, pan-stop-study-05 findings)",
            "B": "receive into the closed nip with arrival spin (planar screen says a vertical fence is climbed; the "
                 "20 deg undercut is the untested fix) and the 1.1-1.5 m/s launch band, which rests on unmeasured "
                 "restitution, friction and nip slip",
        },
    )
    out["benchGate"] = {
        "article": "Real B pocket: 70 mm fence leaning 20 deg toward the pocket, UHMW or polycarbonate face on 25 mm "
                   "compliant backing, 10 mm hood, 200 mm pocket with spring check flap, two-roller nip (2 in 35A "
                   "compliant wheels, 12 mm compression), NEO 550 12:1 with closed-loop velocity; for B'x add the "
                   "deployed lip with the pipe leaving 100 mm beyond the frame line at 0.55 m. Real 4.5 in PVC "
                   "pipes at 0.82 kg (new and worn). Trough mock per target.json: lip top 457.2 mm, lip 12.7 mm, "
                   "15 deg floor, riser top 517.5 mm at 248 mm.",
        "G1 receive": "Feed 30 pipes rolling axis-across at 2.85 +/- 0.10 m/s (measured by light gates or 240 fps "
                      "video), nip stalled in brake: PASS 30/30 seated within 0.5 s, 0 rises above 10 mm at the fence, "
                      "0 entries into the nip, 0 rearward escapes past the flap; repeat 10 with the controller "
                      "unpowered: 10/10. KILL on any climb over the fence.",
        "G2 eject into trough": "Stand-offs 0, 25, 50, 75, 100 mm x bus 10.0, 11.0, 12.0 V (loaded, measured) x 10 "
                                "ejects = 150 per exit variant, target 1.3 m/s (B) or 1.0 m/s (B'x): PASS >= 147/150 "
                                "captured and still supported 2 s later, every cell >= 9/10, measured pipe exit speed "
                                "inside 1.1-1.5 m/s (B) in 150/150. KILL the variant if any cell is <= 7/10.",
        "G3 custody sled": "Pocket loaded, nip unpowered, 1, 3 and 8 g / 50 ms pulses forward and rearward, 10 each: "
                           "PASS 60/60 retained in the pocket. KILL on any loss at <= 3 g.",
        "G4 end to end": "20 consecutive cycles: G1 feed, 5 m carry with 1.5 m/s^2 braking, G2 eject at a random "
                         "stand-off 0-100 mm and random 10-12 V: PASS >= 19/20 credited with no hand recovery.",
        "decision": "All of G1-G4 pass -> retire r3 and start B-family Fusion. G1 or G3 fails -> retain A. "
                    "G2 fails for B but passes for B'x -> select B'x after an R105/G412 ruling. Then run the fair "
                    "spatial 12-case replay with a pocket-and-nip element before shop release.",
    }
    (HERE / "evaluation.json").write_text(json.dumps(out, indent=1, default=float) + "\n")
    for k, v in comparison.items():
        print(k, {kk: vv for kk, vv in v.items() if kk not in ("governing", "custody")})


if __name__ == "__main__":
    main()
