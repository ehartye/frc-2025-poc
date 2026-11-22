"""Merge the five bounded parallel runs of seated_screw_study.py into one record, derive proposed interface
bounds and write it to a new output folder (refuses to overwrite).

usage: python -I -B merge_study.py PART_DIR OUT_DIR
"""
import hashlib, json, shutil, sys, time
from pathlib import Path


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main(part_dir, out_dir):
    part_dir, out_dir = Path(part_dir), Path(out_dir)
    if out_dir.exists(): raise SystemExit("Refusing to overwrite " + str(out_dir))
    have = [i for i in range(4) if (part_dir / f"part-{i}.json").exists()]
    parts = [json.loads((part_dir / f"part-{i}.json").read_bytes()) for i in have]
    ctrl = json.loads((part_dir / "part-ctrl.json").read_bytes())
    base = dict(parts[0]); base["screws"] = [p["screws"][0] for p in parts]
    base["openScrews"] = [dict(selectionRow=i, status="open", reason="bounded parallel run not finished at report time; geometry is the X-mirror of row 3 (266,158) but its own phase sweep, seat, tip and neighbor results are not issued") for i in range(4) if i not in have]; base["controls"] = ctrl["controls"]
    for p in parts + [ctrl]:
        if p["inputsSha256"] != base["inputsSha256"] or p["plugin"]["commit"] != base["plugin"]["commit"]: raise ValueError("Mixed inputs")
    base["execution"] = dict(mode="five bounded parallel processes of seated_screw_study.py (SCREW_ONLY=0..3 with SCREW_SKIP_CONV=1; SCREW_CONTROLS_PHASE=56.0)",
                             partSha256={f"part-{i}.json": sha(part_dir / f"part-{i}.json") for i in have} | {"part-ctrl.json": sha(part_dir / "part-ctrl.json")},
                             runtimeSeconds=[p["runtime"]["seconds"] for p in parts + [ctrl]])
    proposals = []
    for s in base["screws"]:
        ph = s["phase"]; ref = ph["refinement"]; d = ref["offsetsFromCoarseMinDeg"]; v = ref["commonMm3"]
        phi = ph["installPhaseDeg"]; i0 = min(range(len(d)), key=lambda i: abs((ph["coarseMinAtDeg"] + d[i]) % 360 - phi))
        win = lambda w: [v[i] for i in range(len(d)) if abs(d[i] - d[i0]) <= w + 1e-9]
        curve = ph["commonMm3ByDeg"]; c = int(round(phi)) % 360
        v_direct = ph["installPhaseDirectMm3"]
        TH = 0.05  # mm3: quadrature/BSpline noise floor (direct 0.003, plateau 0.015 mm3; no interior witness ball)
        lo = int(round(ph["coarseMinAtDeg"])); w = [lo]
        while curve[(w[0] - 1) % 360] <= TH and len(w) < 360: w.insert(0, w[0] - 1)
        while curve[(w[-1] + 1) % 360] <= TH and len(w) < 360: w.append(w[-1] + 1)
        centre = (w[0] + w[-1]) / 2 % 360
        mis5 = min(curve[int(round(centre - len(w) / 2 - 5)) % 360], curve[int(round(centre + len(w) / 2 + 5)) % 360])
        wit = ph["interiorWitnesses"]["atInstallPhase"]
        fit = dict(kind="fit", status="open", recommendedInstallPhaseDeg=centre, zeroCommonWindowDeg=[w[0] % 360, w[-1] % 360], windowWidthDeg=len(w),
                   measuredAtPhaseMm3=dict(direct=v_direct, phaseDeg=phi, helixRefined=ph["installPhaseHelixMm3"], interiorWitnessBalls=wit["balls"]),
                   alternativeProjectGate=dict(maxCommonMm3=TH, minDiscriminationAt5DegOutsideWindowMm3=mis5),
                   basis=("Correctly phased, the modeled threads share no resolvable material (<= %.3f mm3 over %d deg; no inscribed interior witness), so a plugin fit record "
                          "with positive min/max common has no engineering basis here. Install at the window centre %.1f deg; gate the thread pair with common <= %.2f mm3 "
                          "(a 5 deg mis-phase outside the window gives >= %.2f mm3; phase 0 gives %.1f mm3)." % (max(curve[k % 360] for k in w), len(w), centre, TH, mis5, ph["nominalMm3"])))
        seat = s["atInstallPhase"]["seat"]
        bearing = dict(kind="bearing", status="proposed", bodies=[s["label"] + " screw", seat["bearingBody"]], maxGapMm=0.01,
                       basis="Measured under-head gap %.2g mm, planar contact %.3f mm2 on %s. 0.01 mm admits numeric/native transform rounding and rejects the 0.5 mm lift control by 50x." %
                             (seat["underHeadFaceToBearingGapMm"], seat["planarContactAreaMm2"], seat["bearingBody"]))
        proposals.append(dict(screw=s["label"], host=s["host"], studiedPhaseDeg=phi, fit=fit, bearing=bearing,
                              misPhase5DegCommonMm3=mis5, nominalPhase0CommonMm3=ph["nominalMm3"]))
    base["proposedInterfaces"] = dict(schema_note="Fit records still need agent-frc/placement-fit@1 wrapping with acceptedSha256 and exact native body paths once the native occurrences exist.", rows=proposals)
    base["limits"] = [
        "Primary volume method is exact ray/face quadrature on both real BSpline-thread solids. OCP Booleans on this pair are unreliable (empty or invalid results at phases where quadrature shows tens of mm3); they are retained as cross-checks only and must not be used as the fit measure.",
        "Both threads are basic CAD profiles (male major = female major = 6.350 mm). Common volume at the best phase reflects helix BSpline approximation (measured axial shift per 90 deg differs from p/4 by ~0.005 mm), not a tolerance-class 2A/2B fit, thread strength or preload.",
        "Thread phase in Fusion: the native install must apply the recommendedInstallPhaseDeg rotation about the into-host axis through underHead in addition to the selection transform; the selection transform alone (phase 0) leaves ~15 mm3 of flank interference.",
        "BRepExtrema returned a spurious 0 mm gap (witness point off the screw) for one placed pair; distances here require a genuine witness pair or fall back face-by-face.",
        "Radial convergence (double radii) was skipped in the bounded parallel run (open). Seat, tip and neighbor checks were taken at the studied phase (a window edge), which lies inside the zero-common window.",
        "Controls use phase 56.0 deg from the earlier coarse run rather than the refined phase, and a coarser 21-radius band.",
        "Head bearing contact is the planar area against the truss landing; load path, preload, stripping and tightening access are separate reviews.",
        "No Fusion session was used; native occurrence transform should be checked against the STEP-derived source occurrence before using targetStepOccurrenceWorldCm. worldDeltaFromSourceCm = deltaWorldCm is the robust form.",
    ]
    base["created"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    out_dir.mkdir(parents=True)
    (out_dir / "seated-screw-study.json").write_text(json.dumps(base, indent=1) + "\n")
    for f in ["seated_screw_study.py", "merge_study.py"]:
        shutil.copy2(part_dir / f, out_dir / f)
    for i in have + ["ctrl"]:
        shutil.copy2(part_dir / f"part-{i}.log", out_dir / f"part-{i}.log")
    print(json.dumps(proposals, indent=1))


if __name__ == "__main__":
    main(*sys.argv[1:3])
