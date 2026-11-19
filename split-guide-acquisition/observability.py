"""Exercise the compiled port controller on observed histories and false station signals."""

import argparse
import json
from pathlib import Path
import subprocess, tempfile
from split_checkpoint import HERE, ROOT, coupled
from split_controller import candidate, digest


def sample(
    t, entry=False, seat=False, valid=True, cal=True, dock=True, on=True, stamp=None
):
    return [
        t,
        t if stamp is None else stamp,
        int(valid),
        int(cal),
        int(dock),
        int(on),
        int(entry),
        int(seat),
    ]


def evaluate(classes, jdk, frames):
    stdin = "COUNTERFEED\n" + "".join(",".join(map(str, r)) + "\n" for r in frames)
    result = subprocess.run(
        [
            str(jdk / "bin/java"),
            "-cp",
            str(classes),
            "org.reefscape.controls.PortConsole",
        ],
        input=stdin,
        text=True,
        capture_output=True,
        check=True,
    )
    rows = []
    for f, line in zip(frames, result.stdout.splitlines(), strict=True):
        state, active, speed, reason = line.split(",")
        rows.append(
            dict(
                t=f[0],
                state=state,
                enabled=(active == "true"),
                command=float(speed),
                reason=reason,
            )
        )
    return rows


def signal(r, ms):
    row = next(s for s in reversed(r["portTransitions"]) if s[0] <= ms)
    return row[2:4]


def run(jdk, out):
    source = json.loads((ROOT / "compliant-carriage/series.json").read_bytes())
    left = next(
        r
        for r in source["cases"]
        if r["case"] == "leftSmall" and r["stepS"] == 15.625e-6
    )

    def actual(t):
        return sample(t, *signal(left, t))

    frames = {}
    for label in (
        "empty",
        "seat-first",
        "frozen-seat-only",
        "lost-entry",
        "one-dark-flicker",
        "both-too-soon",
        "stale",
        "docking-lost",
        "disabled",
        "uncalibrated",
        "not-valid",
        "gap-in-clock",
        "clear-after-probe",
        "part-way-jam",
    ):
        rows = []
        for t in range(0, 1601, 20):
            r = actual(t)
            if label == "empty":
                r = sample(t)
            if label == "seat-first":
                r = sample(t, False, t >= 320)
            if label == "frozen-seat-only" and t >= 500:
                r = sample(t, False, True)
            if label == "lost-entry":
                r = sample(t, False, r[-1])
            if label == "one-dark-flicker" and t >= 500:
                r = sample(t, False, t != 520)
            if label == "both-too-soon" and t >= 600:
                r = sample(t, True, True)
            if label == "stale" and t == 600:
                r[1] = 580
            if label == "docking-lost" and t == 600:
                r[4] = 0
            if label == "uncalibrated" and t == 600:
                r[3] = 0
            if label == "disabled" and t == 600:
                r[5] = 0
            if label == "not-valid" and t == 600:
                r[2] = 0
            if label == "gap-in-clock" and t == 600:
                continue
            if label == "clear-after-probe" and t >= 640:
                r = sample(t)
            if label == "part-way-jam" and t >= 640:
                r = sample(t, False, True)
            rows.append(r)
        frames[label] = rows
    with tempfile.TemporaryDirectory() as tmp:
        c = Path(tmp)
        j = c / "PortAcquisition.java"
        j.write_text(candidate(coupled.feed), encoding="utf8", newline="\n")
        subprocess.run(
            [
                str(jdk / "bin/javac"),
                "-d",
                str(c),
                str(j),
                str(ROOT / "observable-acquisition/PortConsole.java"),
            ],
            check=True,
        )
        reports = {}
        for label, seq in frames.items():
            data = evaluate(c, jdk, seq)
            if (
                data[-1]["state"] != "FAULT"
                or data[-1]["command"]
                or any(s["state"] == "IN_BAND" for s in data)
            ):
                raise ValueError(label)
            first = next(q for q in data if q["state"] == "FAULT")
            probes = [q["t"] for q in data if q["state"] == "PROBE"]
            reports[label] = {
                "firstFault": first,
                "positiveProbeTickCount": len(probes),
                "firstProbeMs": min(probes, default=None),
            }
        # A port-history replay on an empty apparatus is externally indistinguishable
        # from the physical source if the fabricated bits and timestamps are identical.
        donor = next(
            r
            for r in source["cases"]
            if r["case"] == "rightSmall" and r["stepS"] == 15.625e-6
        )
        replayed = evaluate(
            c, jdk, [sample(t, *signal(donor, t)) for t in range(0, 1701, 20)]
        )
        if replayed[-1]["state"] != "IN_BAND":
            raise ValueError("Replay ambiguity changed")
        reports["empty-with-replayed-good-bits"] = {
            "firstBandMs": next(x["t"] for x in replayed if x["state"] == "IN_BAND"),
            "independentCustody": False,
        }
        record = {
            "programSha256": digest(j),
            "portFixture": digest(ROOT / "compliant-carriage/series.json"),
            "cases": reports,
        }
    out.write_text(json.dumps(record, indent=2) + "\n", encoding="utf8", newline="\n")


if __name__ == "__main__":
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("jdk", type=Path)
    a.add_argument("out", type=Path)
    ns = a.parse_args()
    run(ns.jdk, ns.out)
