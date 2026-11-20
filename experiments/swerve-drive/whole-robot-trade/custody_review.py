"""Compare complete receiver alternatives from the same two measured vehicle stops."""

import hashlib
import json
import subprocess

from assess_robot import HERE, ROOT, write


def pin(path):
    return {"path": path.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def run(scripts):
    study = HERE.parent
    inputs = {"braked": study / "transit/stops.json", "captive": study / "retention/carry.json"}
    stops = json.loads(inputs["braked"].read_bytes())["cases"]
    carries = json.loads(inputs["captive"].read_bytes())
    executions = {name: {} for name in inputs}
    for sign, old in stops.items():
        pipe = carries[sign]["pipe"]
        stop = old["stopFeedback"]["wallS"]
        state = old["cargo"]["terminalStateSha256"]
        if (old["piece"], state, stop) != (pipe["piece"], pipe["sourcePipe"], pipe["startWallS"]):
            raise ValueError(f"Different incoming pipe or stop on {sign}")
        for concept, record in (
            ("braked", (int(old["resumed"]), old["deliveries"], stop, old["cargo"]["sha256"])),
            ("captive", (int(pipe["releasedWheelsAfterHalfSecond"]), pipe["matchDeliveries"], pipe["endWallS"], pipe["cargoTrace"])),
        ):
            permit, scores, end, trajectory = record
            executions[concept][sign] = {
                "runId": trajectory, "case": sign, "concept": concept,
                "samePiece": old["piece"], "incomingState": state, "stopWallS": stop,
                "lastObservedWallS": end, "resumed": permit, "observedL1": scores,
            }
    traces = HERE / "custody-executions.json"
    write(traces, {"sources": {k: pin(p) for k, p in inputs.items()}, "runs": executions})
    selection = HERE / "role-selection.json"
    write(selection, {
        "schema": "agent-frc/role-selection@1",
        "profile": pin(HERE / "refinement-profile.json"), "study": pin(HERE / "refinement-study.json"),
        "role": "upper-chute", "context": "ordinary-shallow", "phase": "TELEOP", "station": "red-l1",
    })
    concepts = []
    for name, description, from_part, exchange in (
        ("braked", "Swerve with an open receiver, stereo custody gate and stopped retry.", "two optical heads", "visible seated pipe on both heads"),
        ("captive", "Swerve with opposed pins, annular probes and a monitored departure after capture.", "two closing yokes", "two completed shafts and both rim probes in range"),
    ):
        runs = []
        for sign in stops:
            pointer = f"/runs/{name}/{sign}"
            runs.append({"case": sign, "source": pin(traces), "execution": pointer, "values": {
                key: f"{pointer}/{key}" for key in ("resumed", "observedL1")}})
        concepts.append({"id": name, "architecture": description, "runs": runs, "interfaces": [
            {"from": from_part, "to": "swerve drive", "exchange": exchange},
            {"from": "receiver", "to": "L1 trough", "exchange": "a supported, independently released pipe"}]})
    review = HERE / "custody-study.json"
    write(review, {
        "schema": "agent-frc/design-review@1", "concepts": concepts,
        "cases": [{"id": s, "condition": f"Actual {s} wheel calibration and retained pipe at measured stop"} for s in stops],
        "objectives": [{"id": "resumed", "unit": "authorized departures", "min": 1},
                       {"id": "observedL1", "unit": "supported L1 deliveries observed in this trace", "min": 1}],
        "roleHandoff": {"selection": pin(selection), "pairs": [{
            "id": s, "case": s, "before": "braked", "after": "captive",
            "premise": "Does the drive need a working stereo silhouette throughout transport?",
            "change": "Close the bore pins and check both end probes before driving.",
            "fieldPolicies": {"before": "trough-cycle", "after": "upper-chute"},
        } for s in stops]},
    })
    for program, source, result in (("role-handoff", selection, "role-demand"), ("design-review", review, "custody-review")):
        data = subprocess.check_output(["node", str(scripts / (program + ".mjs")), str(source), str(ROOT)])
        write(HERE / (result + ".json"), json.loads(data))
