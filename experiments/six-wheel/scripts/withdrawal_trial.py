"""Verify the one archived desktop departure before supplying a sample to the HAL trial."""
import hashlib
import json


ASSEMBLY = "mission/combined-departure-short.json"
CLOCK = "mission/short-vision-clock.json"
ASSEMBLY_SHA = "4803dd1f9186acd4736e8b43927e5998f6513ae265882f4ca8dd74baaea3543e"
CLOCK_SHA = "32b747afd56ef99ddcc695655c3b84701bcba9f133a28589b9acdb5134d8acb5"


def digest(file):
    sha = hashlib.sha256()
    with file.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError("unqualified desktop withdrawal: " + message)


def reviewed_exposure(root):
    require(digest(root / ASSEMBLY) == ASSEMBLY_SHA and digest(root / CLOCK) == CLOCK_SHA, "study changed")
    shape = json.loads((root / ASSEMBLY).read_bytes())
    study = json.loads((root / CLOCK).read_bytes())
    package = root / "design/competition-package"
    frames = package / "part-frames.json"
    review = json.loads((package / "review.json").read_bytes())
    require(study["pose_receipt_sha256"] == ASSEMBLY_SHA and
            shape["inventory_sha256"] == digest(frames) and shape["body_count"] == 361 and
            set(json.loads(frames.read_bytes())) == set(shape["part_ids"]) and len(set(shape["part_ids"])) == 361,
            "different body set")
    require(set(shape["pose_step_sha256"]) == {"stowed", "receive", "transport", "place"}, "pose scope")
    for pose, expected in shape["pose_step_sha256"].items():
        require(review[pose]["bodies"] == 361 and review[pose]["export_sha256"] == expected and
                digest(package / "poses" / (pose + ".step")) == expected, "different geometry for " + pose)
    require(study["grip_law_sha256"] == shape["grip_law_sha256"] ==
            digest(root / "design/receiver-drives/size_drives.py") and
            shape["exact_displaced_receiver_source_sha256"] == digest(root / "design/receiver-drives/receiver_drives.py") and
            shape["source_trajectory_sha256"] == digest(root / "mission/registered-field-receipt.json"), "mechanism or route")
    require((study["clock_uncertainty_ms"], study["prior_pad_bound_ms"], study["processing_transport_ms"],
             study["scheduler_bound_ms"], study["assumed_actuator_deadtime_ms"]) == (1, 3, 8, [0, 20], [0, 10]),
            "timing allocations")
    run, = (r for r in study["runs"] if (r["piece"], r["sector_m"], r["shelf_coast_mu"]) == ("coral-A", -.180, .1))
    budget = run["budgets"]["0.1"]["0"]
    row, = (e for e in budget["exposures"] if e["source_phase_mod17"] == 0)
    lower, upper = row["onset_after_shelf_ms"]
    geom = run["all_tight_zero_wait_onsets_combined_geometry"]
    require(row["within_150ms"] and row["whole_front_lower_gap_mm"] >= 1 and row["upper_onset_since_prior_contact_ms"] <= 150 and
            geom["last_possible_contact_after_shelf_ms"] < lower and geom["min_tail_separation_mm"] > 0 and
            set(range(lower, upper + 1)) <= set(run["tested_onsets_after_truth_ms"]) and
            not run["failures"] and run["sustained_count"] == run["tested_onset_count"] and
            any(delay == 600 and result == "receiver-recontact" for delay, result, _ in run["late_controls"]),
            "dynamics, geometry or negative control")
    # Integer monotonic test clock preserves the shared fractional offset of these two source frames.
    capture = round(row["capture_from_lip_ms"])
    prior = round(row["prior_pad_from_lip_ms"]) - study["prior_pad_bound_ms"]
    fields = [ASSEMBLY_SHA, CLOCK_SHA, run["piece"], round(1000 * run["sector_m"]), row["source_phase_mod17"],
              capture - prior, row["whole_front_lower_gap_mm"], capture]
    evidence = {"assemblyReceipt": ASSEMBLY_SHA, "clockReceipt": CLOCK_SHA, "bodyCount": shape["body_count"],
                "sectorMm": fields[3], "pieceId": run["piece"], "coastFriction": run["shelf_coast_mu"],
                "pixelUncertainty": 0.1, "intentionalWaitMs": 0, "exposure": row}
    return ",".join(map(str, fields)), evidence
