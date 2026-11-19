"""Screen the actuator, impact path and incremental mass independently of contact success."""
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "design/receiver-drives"))
from size_drives import motor, UP4, UP5


def build():
    native = json.loads((HERE / "native.json").read_bytes())
    rows = [r for r in json.loads((HERE / "study.json").read_bytes())["rows"] if r["id"].startswith("factor-")]
    old_budget = json.loads((ROOT / "design/terminal-mounts/accounting.json").read_bytes())
    volumes = native["nativeVolumeMm3"]
    aluminum = sum(v for k, v in volumes.items() if not k.startswith("bearing-") or k.startswith("bearing-post-"))
    aluminum -= volumes["shaft"] + volumes["drive-reserve"]
    stock = aluminum * 2.70e-6 + volumes["shaft"] * 7.85e-6
    # kg: motor, one assembled single-stage gearbox, two extra cartridges, two bearings,
    # hardware/coupler/limits/wire/liner allowance. No unselected latch credit.
    low = stock + .142 + .095 + .068 + .006 + .080
    high = stock + .142 + .220 + .080 + .040 + .250
    upper_delta = high - native["retiredKg"][0]
    material = sum(v * 2.70e-6 for k,v in volumes.items() if k.startswith(("rail-", "hub-")))
    reach = .254
    accel = 6 * math.radians(20) / .35**2
    angular_speed = 1.5 * math.radians(20) / .35
    load = max(r["dumpTorqueNm"] for r in rows) + material * (9.81 * reach + reach**2 * accel) + .25
    impact = [min(r["captureTorqueRangeNm"][0] for r in rows), max(r["captureTorqueRangeNm"][1] for r in rows)]
    stop = max(abs(t) for t in impact)
    drive = {label: motor(load,angular_speed,1.,ratio,.70,accel,3e-6,8)
             for label,ratio in (("retainedSingle",UP5),("candidateThree",UP5**2*UP4))}
    configuration = old_budget["configurations"]["red"]
    return {"nativeStockKg":stock, "tiltedAluminumKg":material, "upperSwingRadiusM":reach,
            "additionRangeKg":[low,high], "removedRangeKg":native["retiredKg"],
            "worstIncrementKg":upper_delta, "earlierRepresentedMarginKg":configuration["representedMarginKg"][0],
            "remainingRepresentedMarginKg":configuration["representedMarginKg"][0]-upper_delta,
            "completeInspectionMarginKg":None, "dumpTorqueBudgetNm":load, "candidateRatio":UP5**2*UP4,
            "captureSignedTorqueNm":impact, "captureStopSingleSideAt2xN":2*stop/.206,
            "vendorStageFailureReferenceNm":40, "regulatedCandidateDrive":drive,
            "qualifiedForHardware":False}


if __name__ == "__main__":
    print(json.dumps(build(),indent=2,allow_nan=False))
