"""Compare the same incoming hollow pipe at independently declared step and belt-strip resolutions."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
import hashlib
import json
import math
import platform
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "return-station"))
import feed

WINDOW = .330
TIMESTEPS = [.0005/2**i for i in range(6)]
STRIPS = [5, 10, 20]
CASES = ("leftSmall", "rightSmall", "nominal")
POSES = json.loads((ROOT / "station-landmark/feed.json").read_bytes())["outcomes"]
OUTCOME = ("Stop on first outward travel above 1.4 mm or lost support. Contact at horizon "
           "requires a finite lower generator inside the shelf and within 0.025 mm of its "
           "plane throughout the last 5 ms. Classify all other cases unresolved.")


def sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def artifact(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf8", newline="\n")


def settings(case):
    fixed = asdict(feed.cfg)
    del fixed["slices"]
    delta = POSES[case]["actualRobotMmDeg"]
    pipe = feed.Pipe(delta, relief=True)
    pins = {
        "scenario": {"poseMmDeg": delta, "case": case, "source": artifact(ROOT/"station-landmark/feed.json")},
        "constitutiveLaw": {"feed": artifact(ROOT/"return-station/feed.py"), "cartridge": fixed,
                            "outwardMm": pipe.outward, "inwardMm": pipe.inward, "preloadN": pipe.preload},
        "initialState": {"p": pipe.p.tolist(), "v": pipe.v.tolist(), "axis": pipe.a.tolist(),
                         "angularMomentum": pipe.l.tolist(), "beltMps": .65},
        "inputs": {"beltCommandMps": .65, "intervalFromLipSeconds": [0, WINDOW]},
        "runtime": {"python": platform.python_version(), "numpy": np.__version__,
                    "screen": artifact(Path(__file__))},
        "outcomeDefinition": OUTCOME}
    return pins


def manifest(case):
    pins = settings(case)
    model = sha([pins["constitutiveLaw"], pins["runtime"]])
    studies = []
    jobs = []
    for axis, parameter, fixed, values in [
            ("timestep", "dtSeconds", {"strips": 5}, TIMESTEPS),
            ("contact-geometry", "strips", {"dtSeconds": .00025}, STRIPS)]:
        levels = []
        for i, value in enumerate(values):
            config = {**fixed, parameter: value}
            run = f"{axis}-{i+1}"
            levels.append({"runId": run, "resolution": [value], "modelSha256": model, "settingsSha256": sha(config)})
            jobs.append((case, run, config))
        studies.append({"id": axis, "axis": axis, "resolutionParameters": [parameter],
                        "fixedSettingsSha256": sha(fixed), "levels": levels})
    return {"schema": "agent-frc/contact-convergence-manifest@1",
            "comparisonPins": {k: sha(v) for k, v in pins.items()}, "windowEndSeconds": WINDOW,
            "metrics": [{"id": "peak-outward-travel", "units": "mm",
                         "definition": "Largest positive cartridge displacement through 330 ms from the chute lip.",
                         "absoluteTolerance": .05}], "studies": studies}, jobs


def supported(pipe):
    a = pipe.a
    radial = -feed.R*(feed.UP-a*a[2])/math.sqrt(1-a[2]*a[2])
    for side in (-1, 1):
        x, y, z = pipe.p + side*feed.HALF*a + radial
        if -.100 <= x <= feed.EDGE and abs(y) <= .175 and abs(z-feed.TOP) <= .000025:
            return True
    return False


def simulate(job):
    case, run, config = job
    old = feed.cfg
    try:
        feed.cfg = replace(old, slices=config["strips"])
        pipe = feed.Pipe(POSES[case]["actualRobotMmDeg"], relief=True)
        pipe.belt = .65
        gap_at = 0.
        while WINDOW-pipe.time > 1e-11 and pipe.hard is None and pipe.loss is None:
            pipe.moves(.65, min(WINDOW-pipe.time, config["dtSeconds"]))
            if not supported(pipe):
                gap_at = pipe.time
        return {"case": case, "run": run, **config, "endSeconds": pipe.time, "firstStop": pipe.hard,
                "loss": pipe.loss, "lastUnsupportedSeconds": gap_at, "observedPeakMm": float(pipe.peak)}
    finally:
        feed.cfg = old


def report(rows, doc):
    by_id = {r["run"]: r for r in rows}
    runs = []
    for study in doc["studies"]:
        for level in study["levels"]:
            row = by_id[level["runId"]]
            complete = abs(row["endSeconds"]-WINDOW)<1e-9 and not row["firstStop"] and not row["loss"]
            contact = complete and WINDOW-row["lastUnsupportedSeconds"] >= .005
            runs.append({"id": row["run"], "comparisonPins": doc["comparisonPins"],
                         "fixedSettingsSha256": study["fixedSettingsSha256"],
                         "modelSha256": level["modelSha256"], "settingsSha256": level["settingsSha256"],
                         "windowEndSeconds": WINDOW, "completion": "complete" if complete else "failed",
                         "termination": "common-horizon" if complete else "physical-domain-exceeded",
                         "outcome": "contact-at-horizon" if contact else "unresolved",
                         "reportedIntegrityPassed": True,
                         "metrics": [{"id": "peak-outward-travel", "units": "mm",
                                      "value": row["observedPeakMm"] if complete else None}]})
    return {"schema": "agent-frc/contact-convergence-receipt@1", "basis": "simulation-observed", "runs": runs}


def run(plugin, cache):
    cache.mkdir(parents=True, exist_ok=True)
    docs = {}
    jobs = []
    for case in CASES:
        docs[case], local_jobs = manifest(case)
        write(cache/(case+"-manifest.json"), docs[case])
        write(cache/(case+"-parameters.json"), settings(case))
        jobs.extend(local_jobs)
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows = list(pool.map(simulate, jobs))
    results = {}
    for case, doc in docs.items():
        case_rows = [r for r in rows if r["case"] == case]
        observed = cache/(case+"-receipt.json")
        write(observed, report(case_rows, doc))
        expected = cache/(case+"-manifest.json")
        command = subprocess.run(["node", str(plugin/"scripts/check-contact-convergence.mjs"), str(expected), str(observed)],
                                 capture_output=True, text=True)
        result = json.loads(command.stdout)
        if result["status"] == "invalid":
            raise ValueError(result)
        write(cache/(case+"-review.json"), result)
        results[case] = {"manifestSha256": artifact(expected), "receiptSha256": artifact(observed),
                         "checkerExit": command.returncode, "status": result["status"],
                         "engineeringAcceptance": result["engineeringAcceptance"], "runs": case_rows}
    write(HERE/"refinement.json", {"pluginVersion": json.loads((plugin/"package.json").read_bytes())["version"],
                                   "checkerSha256": artifact(plugin/"scripts/contact-convergence.mjs"),
                                   "windowSeconds": WINDOW, "cases": results})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin", type=Path)
    parser.add_argument("cache", type=Path)
    args = parser.parse_args()
    run(args.plugin, args.cache)
