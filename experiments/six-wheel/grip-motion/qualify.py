"""Run the two passive-idler veto against the issued acquisition clocks and worst-case fixtures."""

import argparse, csv, hashlib, json, subprocess, sys, tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "split-guide-acquisition"))
from split_controller import candidate
from split_checkpoint import coupled


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def read(p):
    return json.loads(p.read_bytes())


def write(p, d):
    p.write_text(
        json.dumps(d, indent=2, allow_nan=False) + "\n", encoding="utf8", newline="\n"
    )


def folded_histories(path):
    """Expand the source's lossless change-only register back to its 20 ms service."""
    for row in read(path)["cases"]:
        if not row["id"].endswith("-serial-carriage-recover"):
            continue
        transitions = {point[0]: point[1:] for point in row["portTransitions"]}
        ticks = []
        held = None
        for ms in range(0, row["lastJavaMs"] + 1, 20):
            held = transitions.get(ms, held)
            ticks.append([ms, *held])
        yield (
            f"folded-{row['case']}-{row['stepUs']:g}",
            {
                "portTicks": ticks,
                "sourceProgramSha256": row["programSha256"],
                "state": row["lastState"],
                "firstBandJavaMs": row["firstBandMs"],
            },
        )


def analyze(jdk, artifacts):
    artifacts.mkdir(parents=True, exist_ok=True)
    inputs = artifacts / "ports.csv"
    raw = artifacts / "sweep.csv"
    pins = {}
    sources = {}
    for file in ("candidate", "locked", "traction"):
        p = ROOT / "split-guide-acquisition" / (file + ".json")
        pins[p.relative_to(ROOT).as_posix()] = digest(p)
        for r in read(p)["cases"]:
            sources[f"{file}-{r['side']}-{r['stepS'] * 1e6:g}"] = r
    folded = ROOT / "guide-impact/results.json"
    pins[folded.relative_to(ROOT).as_posix()] = digest(folded)
    sources.update(folded_histories(folded))
    with inputs.open("w", encoding="ascii", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        for key, r in sources.items():
            for ms, state, entry, seat, belt in r["portTicks"]:
                w.writerow([key, ms, int(entry), int(seat), state, belt])
    with tempfile.TemporaryDirectory() as td:
        classes = Path(td)
        port = classes / "PortAcquisition.java"
        port.write_text(candidate(coupled.feed), encoding="utf8", newline="\n")
        if any(r["sourceProgramSha256"] != digest(port) for r in sources.values()):
            raise ValueError("Different station program")
        subprocess.run(
            [
                str(jdk / "bin/javac"),
                "-d",
                str(classes),
                str(port),
                *[str(p) for p in sorted(HERE.glob("*.java"))],
            ],
            check=True,
        )
        subprocess.run(
            [
                str(jdk / "bin/java"),
                "-cp",
                str(classes),
                "org.reefscape.controls.WatchBoundaryTest",
            ],
            check=True,
        )
        subprocess.run(
            [
                str(jdk / "bin/java"),
                "-cp",
                str(classes),
                "org.reefscape.controls.WatchFixture",
                str(inputs),
                str(raw),
            ],
            check=True,
        )
    with raw.open(encoding="utf8") as f:
        rows = list(csv.DictReader(f))
    outcomes = {}
    for r in rows:
        if r["fixture"] == "source":
            key = r["source"]
            source = sources[key]
            outcomes[key] = {
                k: r[k]
                for k in (
                    "probe_ms",
                    "confirmed_ms",
                    "band_ms",
                    "fault_ms",
                    "state_or_command_differences",
                    "outcome",
                    "reason",
                    "last_command",
                )
            }
            outcomes[key].update(
                originalOutcome=source["state"],
                originalFirstBandMs=source["firstBandJavaMs"],
                originalLastJavaMs=source["portTicks"][-1][0],
            )
    challenges = {
        2: "single locked idler",
        3: "wrong right polarity",
        4: "missing ADC channel",
        5: "stale conversion",
        6: "uncommissioned",
        7: "replayed angle with jammed ports",
        10: "empty with good port and angle replay",
        11: "disabled",
        12: "stale port",
        13: "ADC code outside range",
        14: "future conversion",
        15: "failed ADC transaction",
        16: "duplicated frame",
        17: "lost dock",
    }
    negatives = {
        challenges[int(r["mode"])]: {
            k: r[k]
            for k in ("fault_ms", "confirmed_ms", "band_ms", "reason", "outcome")
        }
        for r in rows
        if r["fixture"] == "negative"
    }
    span = {}
    for name in ("healthy", "locked"):
        subset = [r for r in rows if r["fixture"] == "range" and r["source"] == name]
        field = "confirmed_ms" if name == "healthy" else "fault_ms"
        values = [int(x[field]) for x in subset]
        span[name] = {
            "trials": len(subset),
            "millisecondsAfterProbe": [min(values) - 580, max(values) - 580],
            "outcomes": dict(
                (s, sum(x["outcome"] == s for x in subset))
                for s in sorted({x["outcome"] for x in subset})
            ),
        }
    late = [r for r in rows if r["fixture"] == "late"]
    if any(int(r["fault_ms"]) < 0 for r in late):
        raise ValueError("missed persistent post-start stall")
    late_delay = [int(r["fault_ms"]) - float(r["stall_us"]) / 1000 for r in late]
    single_error = 3 / 360 * 60 + 0.5 * 60 / 4095 + 0.5 * 60 / 1650
    # Continuous bounds, not just the sampled phase sweep. Account for two distinct
    # readouts, a closest-earlier baseline, asynchronous service and real dynamics.
    min_end = (60 - 4 - 5.25 - 5.25 - 3 - 1) / 1000
    min_duration = (
        40 - 3 - 2
    ) / 1000  # at most 3 ms filter skew and two 1 ms timing errors
    # The oldest permissible baseline may precede start. Charge a whole acceleration
    # ramp allowance even when both sample endpoints are already in steady rolling.
    min_physical = 0.14 * (min(min_duration, min_end) - 0.14 / (2 * 16)) * 1000
    min_read = min_physical - 0.40 - 2 * single_error
    return dict(
        registeredInputs=pins,
        watchSources={p.name: digest(p) for p in sorted(HERE.glob("*.java"))},
        compiledPortProgramSha256=next(iter(sources.values()))["sourceProgramSha256"],
        sweepSha256=digest(raw),
        sampledCases=len(rows),
        joinedSource=outcomes,
        digitalControls=negatives,
        onsetGrid=span,
        laterPersistentStall={
            "cases": len(late),
            "sampledDelayMs": [min(late_delay), max(late_delay)],
            "conservativeContinuousMs": 40 + 5.25 + 5.25 + 4 + 3 + 2 + 20,
        },
        guaranteedBounds={
            "angleErrorDegEach": 3,
            "quantized4095SensorAnd1650AnalogPairMm": 2 * single_error,
            "thresholdMm": 1.50,
            "minHealthyStationaryStartReadMm": min_read,
            "minRollingSpeedMps": 0.14,
            "minAccelerationMps2": 16,
            "differentialDriveLashMm": 0.40,
            "maxFrameGapMs": 5.25,
            "maxTransferMs": 4,
            "maxEffectiveSensorLagMs": 3,
            "perStampClockErrorMs": 1,
            "scope": "conditional on installed signal, motion and timing acceptance; stationary only inside the modeled supplier/coupling error envelope",
        },
        installedCalibration=None,
        chassisAuthority=False,
        sensorIncrementalMassKg=0.0,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("jdk", type=Path)
    p.add_argument("artifacts", type=Path)
    n = p.parse_args()
    write(HERE / "qualification.json", analyze(n.jdk, n.artifacts))
