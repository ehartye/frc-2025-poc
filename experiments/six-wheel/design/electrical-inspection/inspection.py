"""Calculate rule-qualified mass intervals and paired sensing-distance witnesses."""
import csv
import json
import math
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
POUND=.45359237
BASE_REPLACED={"battery","battery-anchor-L","battery-anchor-R","battery-restraint","protected-distribution",
               "controller-radio","optical-seat","optical-entry","lift-drive","belt-cartridge-L","belt-cartridge-R","slide-drive"}
BASE_REPLACED.update(f"belt-riser-{side}-{x}" for side in "LR" for x in [-90,136])
BASE_REPLACED.update(f"shelf-screw-{side}-{x}" for side in "LR" for x in [-80,140])
# Per-subassembly total, kg. No source claims for provisional ranges. Auxiliary stock and hardware
# not itemized in the base reserve are deliberately included conservatively.
ADDITIONS={
    "NP18-12 battery":(6.17,6.17),"REV PDH + PH":(.591,.591),"radio, controller, RSL, main breaker":(.7,1.5),
    "four mechanism SPARK MAX units":(.36,.6),"battery hold-down, panel, posts and brackets":(1.45,2.1),
    "compressor, reservoir, valve, regulator, gauges and fittings":(1.8,2.6),
    "power cables, branches, moving service loops, leads and terminations":(.65,1.65),
    "long-range sensors and anchors":(.04,.18),"lift drive, ratchet, drums, lines, hardware and release":(1.8,3.9),
    "three receiver drives, guides and rack hardware":(2.04952,3.1125),
    "observer hardware, pole, support and pads":(.5,1.5)}
# Retain the original 2–5 kg non-itemized allowance. No credit for loose wire removed.
def study():
    with (ROOT/"design/drivetrain/review/bom.csv").open() as f: rows=list(csv.DictReader(f))
    present={r["id"] for r in rows}
    if not BASE_REPLACED<=present: raise ValueError("mass source changed")
    totals=[]
    for index,field in enumerate(["minMassKg","maxMassKg"]):
        base=sum(float(r[field]) for r in rows if r["id"] not in BASE_REPLACED)
        bump=sum(float(r[field]) for r in rows if r["group"]=="bumper")
        gross=base+sum(v[index] for v in ADDITIONS.values())
        # Conservative: subtract only the pack, keep all cable/lug mass.
        totals.append({"emptyInstalledKg":gross,"bumperSetKg":bump,"R103kg":gross-6.17-bump,"R408kg":gross-6.17})
    sensors={tag:22+57.15-math.sqrt(57.15**2-(80-u)**2) for tag,u in [("seat",40),("entry",100)]}
    return {"projectMassAllowancesKg":ADDITIONS,"excludedPriorIds":sorted(BASE_REPLACED),"minimum":totals[0],
            "maximum":totals[1],"R103LimitKg":115*POUND,"R408LimitKg":135*POUND,
            "I103InspectedInventoryLimitKg":150*POUND,"actualWeighedKg":None,"seatedPipeRangeMm":sensors,
            "sensorAssumptions":{"emitterBelowShelfTopMm":22,"transversePipeRadiusMm":57.15,"seatedCenterUmm":80},
            "framePerimeterMm":2800,"R104FramePerimeterMaxMm":3048,"stowedHighestEnvelopeMm":1059,
            "R104HeightMaxMm":1066.8,"R105PeakForwardExtensionMm":185,"R105MaximumMm":457.2}


if __name__=="__main__":
    result=study()
    (HERE/"inspection.json").write_text(json.dumps(result,indent=2)+"\n")
    print(result["minimum"],result["maximum"],result["seatedPipeRangeMm"])
