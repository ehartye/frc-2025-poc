"""Bind archived simulated exposures to exact native robot, field configuration and clock."""
import json
from pathlib import Path

from withdrawal_trial import digest, require

ASSEMBLY_SHA = "94a118e30a52e1738cb588e830515c6aa2e3547f82019ab3be6c4c7609b544ea"
CLOCK_SHA = "98c7142e75a2e6b947ba7fa6b94e62801e6a866383d4dee1506f9bfba7b0f806"
EXTENT_SHA = "4d69635cf715543a9edbc75cb7e014a0f053d8c8fe1f6593b5f6fb6fa49e46df"
FIELD_SHA = "5cf57314c6cd699d56e711737c85df21fef78d39ff978a9f2b04c19b9b416350"


def checked(path, sha):
    require(digest(path) == sha, f"changed artifact {path.name}")
    return json.loads(path.read_bytes())


def matrix(root, red_dir):
    study_dir = root / "full-match"
    shape = checked(study_dir / "red-assembly.json", ASSEMBLY_SHA)
    study = checked(study_dir / "red-withdrawal-clock.json", CLOCK_SHA)
    checked(study_dir / "field-trial.json", FIELD_SHA)
    loose = checked(study_dir / "observer-extent.json", EXTENT_SHA)
    require((loose["clock_sha256"],loose["assembly_sha256"],loose["field_trial_sha256"]) ==
            (CLOCK_SHA, ASSEMBLY_SHA, FIELD_SHA) and loose["pose_step_sha256"] == shape["pose_step_sha256"],
            "loose estimate refers to a different configuration")
    require(study["pose_receipt_sha256"] == ASSEMBLY_SHA and shape["full_match_configuration_sha256"] ==
            study["full_match_configuration_sha256"] == FIELD_SHA, "unrelated field or assembly")
    parts = json.loads((red_dir / "part-frames.json").read_bytes())
    require(shape["body_count"] == len(set(shape["part_ids"])) == len(parts) == 401 and
            set(shape["part_ids"]) == set(parts) and digest(red_dir / "part-frames.json") == shape["inventory_sha256"],
            "red bumper inventory")
    for p in ("stowed", "receive", "transport", "place"):
        require(digest(red_dir / (p + ".step")) == shape["pose_step_sha256"][p], "different red " + p)
    require(shape["grip_law_sha256"] == study["grip_law_sha256"] == digest(root / "design/receiver-drives/size_drives.py")
            and shape["exact_displaced_receiver_source_sha256"] == digest(root / "design/receiver-drives/receiver_drives.py")
            and shape["source_trajectory_sha256"] == digest(root / "mission/registered-field-receipt.json"), "motion inputs")
    require((study["clock_uncertainty_ms"], study["prior_pad_bound_ms"], study["processing_transport_ms"],
             study["scheduler_bound_ms"], study["assumed_actuator_deadtime_ms"]) == (1,3,8,[0,20],[0,10]), "clock contract")
    for d in shape["departures"]:
        stereo = d["independent_front_plane_stereo"]
        require(stereo["front_bound_mm"] == 537 and stereo["gap_required_mm"] == 1, "observer plane")
    rows, reasons, late = [], {}, []
    for run in study["runs"]:
        require(not run["failures"] and run["sustained_count"] == run["tested_onset_count"] and
                any(delay == 600 and result == "receiver-recontact" for delay,result,_ in run["late_controls"]),
                "missing dynamic negative or failed motion")
        geometry = run["all_tight_zero_wait_onsets_combined_geometry"]
        stratum = ("piece", "sector_m", "shelf_coast_mu")
        wide, = (r for r in loose["runs"] if all(r[k]==run[k] for k in stratum) and r["pixel_bound"] == .25 and r["wait_ms"] == 0)
        geo_lo, geo_hi = geometry["modeled_onset_interval_after_shelf_ms"]
        tested = set(run["tested_onsets_after_truth_ms"])
        require(geometry["min_tail_separation_mm"] > 0 and geometry["last_possible_contact_after_shelf_ms"] < geo_lo,
                "unresolved robot contact")
        for error, waits in run["budgets"].items():
            window = geometry if error == "0.1" else wide
            geo_lo, geo_hi = window["modeled_onset_interval_after_shelf_ms"]
            require(window["min_tail_separation_mm"]>0 and window["last_possible_contact_after_shelf_ms"]<geo_lo, "contact")
            for wait, budget in waits.items():
                for e in budget["exposures"]:
                    lo, hi = e["onset_after_shelf_ms"]
                    reason = "eligible"
                    if int(wait):
                        reason = "intentional dwell"
                    elif not e["within_150ms"] or e["upper_onset_since_prior_contact_ms"] > 150:
                        reason = "late actuation"
                        late.append([run["piece"], run["shelf_coast_mu"], float(error), e["source_phase_mod17"]])
                    elif error == "0.25" and e["source_phase_mod17"] not in wide["eligible_phases"]:
                        reason = "outside native sweep"
                    elif not (geo_lo <= lo <= hi <= geo_hi):
                        reason = "outside native sweep"
                    elif not set(range(lo, hi+1)) <= tested or e["whole_front_lower_gap_mm"] < 1:
                        reason = "incomplete dynamic coverage or clearance"
                    reasons[reason] = reasons.get(reason, 0) + 1
                    if reason != "eligible":
                        continue
                    cap = round(e["capture_from_lip_ms"])
                    prior = round(e["prior_pad_from_lip_ms"]) - study["prior_pad_bound_ms"]
                    rows.append([ASSEMBLY_SHA, CLOCK_SHA, run["piece"], round(1000*run["sector_m"]),
                                 e["source_phase_mod17"], cap-prior, e["whole_front_lower_gap_mm"], cap,
                                 run["shelf_coast_mu"], float(error), FIELD_SHA])
    require(reasons == {"eligible":134, "intentional dwell":136, "late actuation":2} and
            late == [["coral-A",.2,.25,16],["coral-B",.2,.25,9]], "unexpected exposure coverage")
    return rows, {"extentReceipt":EXTENT_SHA, "lateCases":late, "assemblyReceipt":ASSEMBLY_SHA,"clockReceipt":CLOCK_SHA,"fieldConfiguration":FIELD_SHA,
                  "sourceBodyCount":401,"decisionsByReason":reasons,
                  "eligibilityBasis":"desktop synchronized image, scenario friction/pixel tier and measured motion envelope"}
