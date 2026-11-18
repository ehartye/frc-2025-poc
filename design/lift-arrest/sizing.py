"""Energy, effective-mass and stop-distance screens, SI except named mm fields."""
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE.parent/"receiver-lift"/"delivery-retention.json"
ASSEMBLY_MASS = HERE.parent/"competition-package"/"shop-duplex"/"mass.json"
G = 9.80665
STROKE = 0.0127
TOP = 0.940
ENGAGE = 0.5702
V_NORMAL = 0.253


def buffer(mass, fraction, start_height):
    v2 = V_NORMAL**2 + 2*G*(start_height-ENGAGE)
    energy = fraction*mass*(v2/2 + G*STROKE)
    effective = 2*energy/v2
    return {"impactSpeedMps":math.sqrt(v2),"share":fraction,"oneDamperJ":energy,
            "effectiveKg":effective,"idealMeanForceN":energy/STROKE,
            "fullStrokeCatalogScreen":energy <= 41 and 2.3 <= effective <= 25}


def midstroke(delay_s, trip=.350, net=2*G):
    radius = .012
    start = V_NORMAL
    trigger_distance = (trip**2-start**2)/(2*G)
    at_brake = trip+G*delay_s
    delay_distance = trip*delay_s+G*delay_s**2/2
    # A toothed dog is only one concept; indexing adds an uncertain phase interval.
    index = 2*math.pi*radius/24
    at_index = math.sqrt(at_brake**2+2*G*index)
    return {"lagS":delay_s,"tripMps":trip,"lagTravelMm":delay_distance*1000,
            "maxIndexMm":1000*index,"speedAtIndexMps":at_index,
            "distanceWithIndexMm":1000*(trigger_distance+delay_distance+index+at_index**2/(2*net)),
            "maxBrakeForceNPerKg":G+net}


def study():
    native = json.loads(BASE.read_bytes())
    assembly = json.loads(ASSEMBLY_MASS.read_bytes())
    hi = assembly["liftLoadedUpperKg"]
    shop_slide_delta = hi - native["candidateLoadedUpperKg"]
    # Low screen omits the 0.25 kg uncertainty allowance; avoids inflating
    # the minimum effective mass to make the supplier range look favorable.
    lo = native["registeredMoving"]["dryKg"][0] + sum(native["candidateMovingAddedKg"].values()) + shop_slide_delta
    return {"inputSha256":hashlib.sha256(BASE.read_bytes()).hexdigest(),
            "assemblyMassSha256":hashlib.sha256(ASSEMBLY_MASS.read_bytes()).hexdigest(),
            "massRangeKg":[lo,hi],"measuredMassKg":None,"axisOneDamperCapacityJ":41,
            "highBalanced":buffer(hi,.5,TOP),"lowBalanced":buffer(lo,.5,TOP),
            "high7030": [buffer(hi,f,TOP) for f in (.7,.3)],
            "low7030": [buffer(lo,f,TOP) for f in (.7,.3)],
            "fullLoadOneUnit":buffer(hi,1,TOP),
            "lowVelocityApproach":buffer(hi,.5,ENGAGE),
            "netStopForceNAtUpperMass":hi*3*G,
            "flyweightExampleN":{str(v):.012*.0125*(4*v/.012)**2 for v in (V_NORMAL,.350)},
            "midtravel":[midstroke(.010),midstroke(.050)],
            "operatingPermission":False}


if __name__ == "__main__":
    print(json.dumps(study(),indent=2))
