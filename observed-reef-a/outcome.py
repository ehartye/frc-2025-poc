"""Join each actual arrival to its cargo, field-camera and stopped chassis records."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent

def read(name):
    return json.loads((HERE/(name+".json")).read_bytes())

def summary():
    roof=read("station-bridge")
    cameras=read("machined-field-trials")
    geometry=read("machined-stock")
    electrical=read("field")
    arrivals=read("machined-roof-limited-quantized-cargo-nominal-active")
    effective=read("limited-quantized-cargo-nominal-active")
    effort=read("effort")
    result={}
    for a,b in zip(arrivals,effective):
        side=a["wheelCase"]
        if side!=b["wheelCase"] or roof[side]["pipeSha256"]!=a["sourceFeedSha256"]:
            raise ValueError("Broken pipe or signed-approach identity")
        optical=next(r for r in cameras if r["mode"]=="nominal" and r["wheelCase"]==side)
        drive=next(r for r in electrical if r["sensor"]=="nominal" and r["case"]==side)
        if len({a["sourceFieldPathSha256"],b["sourceFieldPathSha256"],drive["routeSourceSha256"],optical["motionSHA256"]})!=1:
            raise ValueError("Different attained field route")
        torque=next(r for r in effort["trajectories"]["machined-roof-limited-quantized-cargo-nominal-active"] if r["case"]==side)
        field_ok=optical["accepted"] and drive["power"]["screened"]["firstReferenceInhibitWallS"] is None
        handoff=all(r[-1] for r in roof[side]["fixedMarked"]+roof[side]["roof"]["roof"])
        sensor_bound=all(r[-1] for r in roof[side]["roof"]["roof-050"])
        nominal=field_ok and handoff and a["plannedEndpoint"] and torque["insideAnalyticalInputsAtSamples"] and effort["driveAt006Mps"]["2.4"]["feasible"]
        if nominal and abs(a["finalWallS"]-drive["finalWallS"])>1.e-6:
            raise ValueError("Cargo and chassis clocks disagree")
        result[side]={"piece":"coral-A","stationSourceSha256":a["sourceFeedSha256"],"checkpointSha256":a["elasticStateSha256"],
            "routeSha256":optical["motionSHA256"],"nominalImageWheelPathReadyWallS":drive["cameraDecisionWallS"],
            "nominalHeldAtSectorOnJointClock":nominal,"nominalMaterialFault":a["firstFault"],"nominalCarryEndsWallS":a["finalWallS"],
            "firstUnobservedOrMissingReason":a["physicalStop"] or a["lostPipe"] or a["firstFault"],
            "lastPipePositionAxisVelocityOmega":a["lastPipe"][1:13],"lastCropEvidenceJavaMs":a["lastObservedJavaMs"],
            "lastPipeTrajectorySha256":a["trajectorySha256"],"nativeBallLowerMm":geometry[side]["minBoundWith10Mm1DegMm"],
            "arrivedOnEffectiveShelf":b["plannedEndpoint"],"guardedStationDeparture":handoff and sensor_bound,
            "guardedAction":"hold at occupied station" if not sensor_bound else "continue with fresh observations",
            "scoredPieces":[],"secondFeedIssued":False,"positivePointsUnderOpeningAssumptions":3,
            "separateSouthReservedAlternativePoints":5}
    return result

if __name__=="__main__":
    (HERE/"outcomes.json").write_text(json.dumps(summary(),indent=2)+"\n",encoding="utf8",newline="\n")
