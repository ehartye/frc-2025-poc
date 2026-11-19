"""Prepare and audit the first loaded swerve movement with separate simulation processes."""

import argparse
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
CASES = ("plus", "minus")
STEPS = (31.25, 15.625)


def read(p):
    return json.loads(p.read_bytes())


def write(p, d):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(
        json.dumps(d, indent=2, allow_nan=False) + "\n", encoding="utf8", newline="\n"
    )


def sha(p):
    with p.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def feeds(raw, destination):
    energy = read(HERE / "energy.json")["cases"]
    rows = []
    for case in CASES:
        for step in STEPS:
            row = read(raw / f"{case}-{step:g}.json")
            if (
                row["case"],
                row["plantStepUs"],
                row["driveTraceSha256"],
                row["trueChassisAtFeedMmMmDeg"],
            ) != (
                case,
                step,
                energy[case]["csvSha256"],
                energy[case]["actualTerminalMmMmDeg"],
            ):
                raise ValueError("Mismatched signed feed or solver clock")
            rows.append(row)
    write(
        destination,
        dict(
            schema="reefscape-reciprocal-feed/1",
            openingSha256=sha(HERE / "energy.json"),
            attempts=rows,
        ),
    )


def main(args):
    case, step = args.case, args.step
    f = next(
        r
        for r in read(args.feeds)["attempts"]
        if (r["case"], r["plantStepUs"]) == (case, step)
    )
    optical = read(args.witness)
    if optical["firstFeedSha256"] != sha(args.feeds):
        raise ValueError("Optics refers to different feed records")
    initial = next(
        r
        for r in optical["cases"]
        if (r["case"], r["stepUs"], r["finish"]) == (case, step, "guarded")
    )
    trial = args.work / f"{case}-{step:g}us"
    trial.mkdir(parents=True, exist_ok=True)
    if (
        not initial["gate"]["accepted"]
        or initial["terminalStateSha256"] != f["pipeStateSha256"]
        or initial["trace"] != f["pipeTrajectorySha256"]
        or initial["piece"] != f["physicalPieceId"]
    ):
        raise ValueError("Current station custody does not permit departure")
    servicefile = trial / "service.json"
    lease = args.lease_ms / 1000
    if args.stage == "service":
        from first_service import propagate

        write(servicefile, propagate(args.raw, f, trial))
        return
    service = read(servicefile)
    ds = f"drive-{args.lease_ms}.json"
    cs = f"contact-{args.lease_ms}.json"
    if args.stage == "wheels":
        from departure_drive import run

        write(trial / ds, run(case, args.wheels, lease, trial, service))
    elif args.stage == "pipe":
        from departure_contact import run

        drive = read(trial / ds)
        checkpoint = trial / f"{case}-{step:g}us-service.npz"
        write(
            trial / cs,
            run(
                args.raw,
                f,
                service,
                checkpoint,
                (trial / f"{case}-{args.lease_ms}.npz", drive),
                trial,
            ),
        )
    else:
        from departure_vision import run

        decision = run(
            args.field, f, service, initial, read(trial / ds), read(trial / cs), trial
        )
        write(trial / f"images-{args.lease_ms}.json", decision)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="stage", required=True)
    collect = sub.add_parser("collect")
    collect.add_argument("raw", type=Path)
    collect.add_argument("output", type=Path)
    for stage in ("service", "wheels", "pipe", "vision"):
        a = sub.add_parser(stage)
        a.add_argument("case", choices=CASES)
        a.add_argument("step", type=float, choices=STEPS)
        for name in ("raw", "feeds", "witness", "work"):
            a.add_argument(name, type=Path)
        a.add_argument("--lease-ms", type=int, default=600)
        if stage == "wheels":
            a.add_argument("--wheels", type=Path, required=True)
        if stage == "vision":
            a.add_argument("--field", type=Path, required=True)
    a = p.parse_args()
    if a.stage == "collect":
        feeds(a.raw, a.output)
    else:
        main(a)
