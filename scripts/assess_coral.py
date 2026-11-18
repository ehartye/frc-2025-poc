"""Offline evidence assessment for one robot, one alliance reef and one period."""
import argparse
import hashlib
import json
from pathlib import Path

EDITIONS = ("kickoff-v0", "archive-compiled")
ACTIVE_MS = {"AUTO": 15_000, "TELEOP": 135_000}
OBSERVATIONS = {
    "fullySupportedByRobot", "stuckToRobot", "intentionalHerding",
    "pushesScoredL1WhilePlacing", "touchesAlliedRobot", "touchesOpponentRobot",
    "touchesOwnTrough", "scoredOnAnotherReefLevel",
}


def any_observed(values):
    if True in values:
        return True
    return None if None in values else False


def all_observed(values):
    if False in values:
        return False
    return None if None in values else True


def inverse(value):
    return None if value is None else not value


def label(value):
    return {True: "yes", False: "no", None: "unknown"}[value]


def integer(value):
    return type(value) is int and value >= 0


def identity(value):
    return isinstance(value, str) and bool(value.strip())


def witness_map(rows, time_ms):
    if not isinstance(rows, list):
        raise ValueError("pieces must be an array")
    pieces = {}
    for row in rows:
        if not isinstance(row, dict) or not identity(row.get("id")) or row["id"] in pieces:
            raise ValueError("each piece needs one unique nonempty identity per sample")
        if set(row) - OBSERVATIONS - {"id", "observedMs", "directSupporterIds"}:
            raise ValueError("unknown piece observation")
        observed = row.get("observedMs")
        if not integer(observed) or observed > time_ms:
            raise ValueError("piece observation needs a nonfuture timestamp")
        for key in OBSERVATIONS:
            if row.get(key) is not None and type(row[key]) is not bool:
                raise ValueError(key + " must be true, false or null")
        supports = row.get("directSupporterIds")
        if supports is not None and (
            not isinstance(supports, list) or not all(identity(s) for s in supports)
            or row["id"] in supports or len(supports) != len(set(supports))
        ):
            raise ValueError("invalid supporting identities")
        pieces[row["id"]] = row if observed == time_ms else {}
    return pieces


def assess_sample(sample, phase, edition):
    time_ms = sample["timeMs"]
    duration = ACTIVE_MS[phase]
    if type(sample.get("inventoryComplete")) is not bool:
        raise ValueError("declare inventoryComplete for the focal robot's CORAL")
    # Post-buzzer assessment permits a moving piece to settle. It adds no powered travel.
    if type(sample.get("robotCommandActive")) is not bool:
        raise ValueError("declare robotCommandActive")
    if time_ms >= duration and sample["robotCommandActive"]:
        raise ValueError("the active movement budget has expired")
    clock = "active" if time_ms < duration else (
        "assessment-window" if time_ms <= duration + 3000 else "outside-window"
    )
    pieces = witness_map(sample.get("pieces"), time_ms)
    results, control_counts = [], []
    for piece_id, row in pieces.items():
        fixed = any_observed([row.get("fullySupportedByRobot"), row.get("stuckToRobot")])
        herding = row.get("intentionalHerding")
        control = any_observed([fixed, herding])
        effective_herding = herding
        if edition == "archive-compiled":
            effective_herding = all_observed([herding, inverse(row.get("pushesScoredL1WhilePlacing"))])
        counts = any_observed([fixed, effective_herding])
        control_counts.append(counts)

        supports = row.get("directSupporterIds")
        supported = None if supports is None else any_observed([
            pieces.get(other, {}).get("touchesOwnTrough") for other in supports
        ])
        qualifies = [any_observed([row.get("touchesOwnTrough"), supported]),
                     inverse(row.get("touchesAlliedRobot"))]
        if edition == "kickoff-v0":
            qualifies.append(inverse(row.get("touchesOpponentRobot")))
        else:
            qualifies.append(inverse(row.get("scoredOnAnotherReefLevel")))
        results.append({"id": piece_id, "controlledByFocalRobot": label(control),
                        "countsTowardCoralLimit": label(counts),
                        "l1CriteriaAtInstant": label(all_observed(qualifies))
                        if clock != "outside-window" else "outside-window"})
    if control_counts.count(True) >= 2:
        capacity = "over"
    elif sample["inventoryComplete"] and control_counts.count(True) + control_counts.count(None) <= 1:
        capacity = "within"
    else:
        capacity = "unknown"
    return {"timeMs": time_ms, "clock": clock, "controlledCoralCountScreen": capacity,
            "pieces": results}


def assess(trace, edition):
    if edition not in EDITIONS or trace.get("schema") != "reefscape-coral-observations/1":
        raise ValueError("unsupported rules edition or trace schema")
    phase, source = trace.get("phase"), trace.get("basis")
    if phase not in ACTIVE_MS or not identity(source):
        raise ValueError("declare phase and observation basis")
    samples = trace.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("supply one or more chronological samples")
    reports = []
    previous = -1
    for sample in samples:
        if not isinstance(sample, dict) or set(sample) != {"timeMs","inventoryComplete","robotCommandActive","pieces"}:
            raise ValueError("invalid sample fields")
        now = sample["timeMs"]
        if not integer(now) or now <= previous:
            raise ValueError("sample times must be strictly increasing integer milliseconds")
        reports.append(assess_sample(sample, phase, edition))
        previous = now
    return {"edition": edition, "phase": phase, "basis": source, "samples": reports,
            "officialEarnedPoints": None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="project observation JSON")
    parser.add_argument("--edition", required=True, choices=EDITIONS)
    args = parser.parse_args()
    try:
        data = args.input.read_bytes()
        report = assess(json.loads(data), args.edition)
        report["inputSha256"] = hashlib.sha256(data).hexdigest()
        print(json.dumps(report, indent=2, allow_nan=False))
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        parser.exit(2, str(exc) + "\n")


if __name__ == "__main__":
    main()
