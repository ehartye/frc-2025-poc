"""Generate operating-intent.json for pan revision r3 from geometry.json, structure.json and the roster.

Run from the project root after pan_geometry.py, pan_structure.py and pan_roster.py:
  python -I iterations/002/design/pan/pan_intent.py
"""
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    geo = json.loads((HERE / "geometry.json").read_bytes())
    st = json.loads((HERE / "structure.json").read_bytes())
    stop, soft = geo["stopGeometry"], geo["softStopGeometry"]
    gov = st["cradleCheck"]["governing"]
    intent = {
     "schema": "red2/pan-operating-intent@2", "revision": geo["revision"],
     "status": "Proposal r3 for review. Copy /operatingIntent into the independently approved commissioning plan; the reviewer holds that copy outside the candidate tree.",
     "decisions": {
      "openingOnlyDriveInhibit": "The -3.5 deg limit inhibits drive output in the opening direction only; it never cuts the closing drive and never latches. Evidence: pan-stop-study-04 (a full cut fails 6 of 12 resets; the opening-only inhibit never trips in normal running). Idle mode coast.",
      "gearboxProtection": "Fit a mandatory slip clutch set between the worst normal-duty torque (15.4 N m) and the output-stage rating, or show a documented output capacity of at least 26 N m (F3 25.9 N m with the inhibit; pre-inhibit F1 peak 14.3 N m). r3 finds the capacity route valid only with a measured inhibit latency of about 1.5 ms or less, so the clutch is carried as required.",
      "retentionFrom001": "The owner delegated mechanical-viability decisions. Retaining the 001 pan contact surfaces, axis and drive beyond the floor profile named in commissioning.md:3 is accepted, on the 12-case simulations built on them and the failed fixed-path comparison (retained-from-001.md)."
     },
     "operatingIntent": {
      "authorityId": "red2-two-sill-robot",
      "configuration": "002 guarded pan and upper support on taps-02 chassis (pan-build r3 proposal)",
      "mode": "articulated",
      "basis": "limited-pan-12mm / limited-field-four-terminals (12/12) on pan-mechanics 0b743fed, and pan-stop-study F2/F3 (12/12 at 30 A and unpowered). pan-stop-study-04 replayed the r2 properties; r3 changes them again (geometry.json reconciliation), so a fresh 12-case replay is required before release",
      "motionRanges": {"pan": {"unit": "deg", "convention": "sim theta (floor direction [cos t, 0, sin t])", "min": -2.0, "max": 70.0,
                               "physicalStops": {"open": -5.0, "stow": 70.0}}},
      "nativeCoordinates": {"pan": {"type": "revolute", "occurrence": "Guarded pan:1", "name": "Pan pivot", "coordinate": "rotationValue",
                                    "nativeToTheta": "phi = -theta (the recipe verifies the sign on the floor landmark and records phiPerTheta)",
                                    "nativeLimitRad": [-math.radians(geo["joints"][0]["thetaLimitsDeg"][1]), -math.radians(geo["joints"][0]["thetaLimitsDeg"][0])],
                                    "nativeCommandedRad": [-math.radians(70.0), math.radians(2.0)], "openingDriveInhibitThetaDeg": -3.5}},
      "transferMissions": [],
      "externalIssues": [],
      "articleFunctions": {"file": "iterations/002/design/pan/target-roster.json", "sha256": sha(HERE / "target-roster.json")}
     },
     "pivotAxis": {"pointMm": [205.0, 0.0, 580.0], "direction": [0, 1, 0], "frame": geo["frame"], "journalsMm": geo["pivot"]["journalsMm"]},
     "poses": {"stowDeg": 70.0, "dumpDeg": -2.0, "softStopDeg": -5.0, "flatAsAuthoredDeg": 0.0, "commandedTravelDeg": 72.0,
               "tip": {"startBeforeReleaseS": 3.5, "durationS": 1.6, "profile": "smoothstep"}, "savedPose": "stow"},
     "hardStops": {
      "stow": {"kind": "passive unilateral high-loss pad pair on the X110 cradle, striking integral cheek stop lugs", "thetaDeg": 70.0, "toleranceDeg": 0.05,
               "contactAtStowMm": stop["contactAtStowMm"], "contactYmm": stop["contactYmm"], "padPushDirection": stop["padPushDirection"], "leverMm": stop["leverMm"],
               "seriesStiffness": st["stowStopStiffness"]["bounds"], "peakDynamicOverTravelRad": 0.0014366689614586647},
      "open": {"kind": "passive high-loss soft stop pair (18 mm pads) on solid beams clamped to the X250 cradle through keeper plates", "thetaDeg": -5.0,
               "contactAtSoftStopMm": soft["contactAtSoftStopMm"], "contactYmm": 177.525, "leverMm": soft["leverMm"],
               "torsionalNmPerRad": st["softStop"]["torsionalNmPerRad"], "designCase": st["softStop"]["designCase"],
               "faultEnvelopeMinDeg": st["softStop"]["faultEnvelopeMinDeg"], "structure": st["softStopStructure"]},
      "fusionLimits": "revolute limits theta %g..%g deg: the -9 deg minimum covers the fault envelope (%.2f deg, F2/F3 with the inhibit late and the B1 bumper strike) beyond the -5 deg pad face; the 70.1 deg maximum covers the 70.082 deg stow peak overtravel; pad compliance is not modeled in Fusion" % (geo["joints"][0]["thetaLimitsDeg"][0], geo["joints"][0]["thetaLimitsDeg"][1], st["softStop"]["faultEnvelopeMinDeg"]),
      "openingDriveInhibit": st["driveInhibit"]
     },
     "stopReplayInput": {
      "for": "pan-stop-study replay (M1)",
      "contactAtStowMm": stop["contactAtStowMm"], "contactYmm": [stop["contactYmm"], -stop["contactYmm"]], "padPushDirection": stop["padPushDirection"],
      "leverMm": stop["leverMm"], "wallFaceNormalAtStow": stop["backFaceOutwardNormalAtStow"], "studyContactMm": stop["studyContactMm"],
      "note": "The study's lumped model (lever 100 mm, 6000 N m/rad, 15.8 N m s/rad) is unchanged. Only the series stiffness bound changes; if a native or bench value falls below 3000 N m/rad, replay."
     },
     "stopDuty": {
      "source": "resume-20261121-2025/pan-stop-01/pan-stop-study.json (1f371208...) and pan-stop-study-02.json",
      "sign": "+q (theta increasing) closes past stow; the stow stop reacts +q only, the soft stop -q only",
      "stopAt30A": {"peakNm": 9.270017174435251, "sustained100msNm": 2.423933295706352, "sustained500msNm": 2.0662915714489234},
      "stopUnpowered": {"peakNm": 10.196520230184463, "sustained100msNm": 2.856128539736673, "sustained500msNm": 2.8192928043111345},
      "returnImpactPeakNm": 7.1023623, "compliance": {"perPadNPerMm": 300, "torsionalNmPerRad": 6000, "dampingNmsPerRad": 15.8, "dampingRatio": 0.3007},
      "preload": {"closingNmAtStow": 1.5, "zeroAtThetaDeg": -2.0, "rateNmPerRad": st["closingSpring"]["rateNmPerRad"], "embodiment": "torsion spring on the extended idler stub (OD-3)"}
     },
     "drive": {"motor": "REV-21-1651 NEO 550", "ratio": 54.83289124668435, "phaseCurrentCapA": 30, "reserveA": 1.4, "efficiencyBasis": 0.6,
               "controller": {"gains": [65, 2.5, 15], "periodS": 0.02},
               "faultInterlock": st["softStop"]["controlsRequirement"], "idleMode": st["gearbox"]["idleMode"],
               "openingDriveInhibit": {"decision": "Owner decision: the -3.5 deg limit inhibits drive output in the opening direction only; it never cuts closing drive and never latches",
                                       "evidence": st["driveInhibit"]["evidence"], "hardware": st["driveInhibit"]["hardware"], "travel": st["driveInhibit"]["travel"],
                                       "responseTime": st["driveInhibit"]["responseTime"]},
               "outputStage": {k: st["gearbox"][k] for k in ("decision", "normalDutyCapNm", "formula", "worstFaultIdealInhibitNm", "worstFaultInhibitLateNm", "clutchRoute", "capacityRoute", "supplierRating", "verdict", "currentCap")},
               "peaks": {"phaseA": 29.62, "loadedDrivingA": 7.45, "minBranchV": 9.47, "gearboxOutputLoadBoundNm": 7.113667837887667}},
     "custodyCriteria": {"cases": 12, "passIf": "All 12 keep and score both pipes with >= 1.83 mm post-contact support and a non-negative reset margin, at the 30 A limit AND unpowered through receive and carry",
                         "minSupportMarginMm": 1.83, "resetDeadline": "reset_wall_s <= reset_deadline_wall_s; stop-proposal minimum margin 71.8 ms",
                         "stopPlacement": "stow contact within +/-0.05 deg of commanded stow", "dockStandOffMm": 12.0},
     "loads": {"pipeRadialPerSideN": 445.490116272023, "pipeAxialN": 396.2769979069151, "movingGuideN": 685.3026513761104, "fixedInletN": 94.26791084504448},
     "chassisInterfaces": {"anchor": "occurrence owning 'Upper pan stringer 1' (Red 2 inset structural machining:1); the 7-body Serviceable brace revision:1 is resolved separately",
                           "rootBodyCounts": geo["chassisOccurrences"]["byRoot"], "attachments": geo["attachments"],
                           "structuralScreens": {"cradles": dict(governingCase=gov["case"], vonMisesMPa=gov["vonMisesMPa"], marginOnWorking=gov["marginOnWorking"]),
                                                 "kneeStruts": {k: st["carrierAndStruts"][k] for k in ("strutAxialEachN", "strutStressMPa", "strutBucklingMargin")},
                                                 "trussNodes": {k: st["trussNodes"][k] for k in ("memberForceN", "rivetShearMargin", "wallBearingMarginOn1p5Working", "screwTensionMargin")},
                                                 "cheekRootMoment": st["cheekLadder"]["rootMomentPath"], "fasteners": st["fastenerJoints"]},
                           "structuralScreensR3": {"rootMomentJoint": st["cheekLadder"]["rootMomentPath"]["joint"], "offPost": st["cheekLadder"]["offPostSensitivity"],
                                                   "softStop": st["softStopStructure"], "stowBracket": st["stowStopBracket"], "worstMargins": st["worstMargins"]},
                           "structuralRoutesPending": ["native structural-route review of the pad, journal, strut and soft-stop paths into the cradles and stringers, including cradle front-wall crippling under the soft-stop beam heel",
                                                       "end-clip and splice bolt rows (attachment design)", "inlet portal-frame end moment at the X110 end clip",
                                                       "slip-clutch drive checked placement", "sourced rivet-nut pull-out rating"]},
     "rules": {k: v for k, v in st["rulesCheck"].items()}
    }
    (HERE / "operating-intent.json").write_text(json.dumps(intent, indent=1) + "\n", encoding="utf-8")
    print(intent["operatingIntent"]["articleFunctions"])


if __name__ == "__main__":
    main()
