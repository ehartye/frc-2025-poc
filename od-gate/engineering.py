"""Material accounting, impact and motor upper screens for the installed replacement."""
import csv
import json
import math
from gate_cad import local,cad,ROOT,HERE,sha,ARM
from motion import Case

def read(path):return json.loads(path.read_bytes())

def aggregate():
    lo_by_id={};hi_by_id={}
    # Measured native volumes for stock. Hardware reserves use component masses, not housing fill.
    density={"Al":(2.65,2.82),"steel":(7.7,7.95),"foam":(.2,.5),"poly":(.92,.99),
             "rubber":(1.0,1.3),"bearing":(3.2,7.85),"sensor":(.8,2.0)}
    groups={};moving=0.
    for key,(frame,mat,shape) in local().items():
        vol=cad.volume(shape)[0]*1e-6
        if mat=="motor":lo,hi=.142,.15
        elif mat=="gearbox":lo,hi=.12,.22
        elif key.startswith("hood-wheel"):lo,hi=.083,.085
        elif key.endswith("537-3M-15"):lo,hi=.019,.023
        else:lo,hi=[vol*d for d in density[mat]]
        lo_by_id[key]=lo;hi_by_id[key]=hi
        total=groups.setdefault(mat,[0.,0.]);total[0]+=lo;total[1]+=hi
        if frame in ("L","R"):moving+=hi
    allowance={"fasteners-guards-covers":(.08,.22),"limit-inputs-hood-load-sensors-electronics":(.04,.14),
               "motor-couplers-bearings-retainers":(.08,.18),"sensor-and-moving-power-loom":(.09,.22)}
    low=sum(lo_by_id.values())+sum(v[0] for v in allowance.values())
    high=sum(hi_by_id.values())+sum(v[1] for v in allowance.values())
    prior=read(ROOT/"cradle-receiver/native.json")
    with (ROOT/"design/station-l1/bom.csv").open() as stream:
        shelf=next(row for row in csv.reader(stream) if row[0]=="receiver-R6-ported")
    retire=prior["retiredKg"][0]+float(shelf[4])
    margin=read(ROOT/"design/shared-height/inspection.json")["cases"]["red"]["low"]["R103UpperHeadroomKg"]
    no_air=ROOT/"design/fixed-no-air/inspection.json"
    receipts={"red-low-light-and-shared-masts":margin}
    if no_air.is_file():
        ids=set(read(no_air.with_name("deletion.json"))["removed"])
        if ids.intersection(set(prior["retiredIds"])|{"receiver-R6-ported"}):
            raise ValueError("Overlapping removal credit")
        receipts["red-low-light-dedicated-masts-physically-deleted-pneumatics"]=read(no_air)["cases"]["low-red"]["checker_upper_headroom_kg"]
    cases={k:{"startingRepresentedMarginKg":v,"afterUpperReplacementKg":v+retire-high} for k,v in receipts.items()}
    return {"representedStockByMaterialKg":groups,"nongeometricAllowanceKg":allowance,"newPartsKg":[low,high],
            "retiredOriginalGripMinimumKg":prior["retiredKg"][0],"retiredShelfMinimumKg":float(shelf[4]),
            "conservativeNetAdditionKg":high-retire,"cases":cases,"newCassetteMassUpperBothKg":moving,
            "finishedConfigurationMarginKg":None}

def motor(torque,speed,ratio,efficiency,current_limit):
    # NEO 550 constants from issued receiver-drive source. No-load current is charged.
    phase=1.4+abs(torque)/(ratio*efficiency)*98.6/.97
    voltage=abs(speed)*ratio/(11000*math.tau/60)*(12-.12*1.4)+.12*phase
    return {"phaseA":phase,"currentLimitA":current_limit,"terminalV":voltage,"voltageAllocationV":9.5,
            "withinAllocations":phase<=current_limit and voltage<=9.5}

def build():
    stock=aggregate();rows=[r for r in read(HERE/"study.json")["rows"] if r["id"].startswith("grid")]
    lower=max(r["peakNormalN"].get("belt",0) for r in rows)
    shear=max(r["peakSingleSurfaceTractionN"] for r in rows)
    opened=max(r["openingBeltNormalN"] for r in rows);hood=max(r["peakNormalN"].get("hood",0) for r in rows)
    base=Case()
    radius=.060/math.tau
    accel=6*.115/.28**2;speed=1.5*.115/.28
    f=opened*.7+8+(stock["newCassetteMassUpperBothKg"]+.12)*accel
    gate=motor(f*radius,speed/radius,(84/29)*(76/21),.72,15)
    arm=ARM
    hood_angle=math.asin(.04/arm);omega=1.5*hood_angle/.24
    hood_demand=hood*arm+.70*9.80665*arm+.18
    closing=motor(hood_demand,omega,(68/13)**2*3,.70,20)
    torque=shear*base.belt_r
    # Inscribed OD cylinder gives a conservative torsion section of the 12.7 mm AF hollow hex.
    stress=16*torque*12.7e-3/(math.pi*((.0127)**4-.008**4))
    inertia=math.pi*(.010**4-.0065**4)/64
    # Fault case: all normal load on one guide at midspan, instead of ideal four-guide sharing.
    L=.172;stress_b=lower*L/4*.005/inertia
    sag=lower*L**3/(48*200e9*inertia)
    allowance=80e6;shaft_margin=allowance/stress
    return {"studySha256":sha(HERE/"study.json"),"nativeSha256":sha(HERE/"split-gate.step"),
            "mass":stock,"opening":{"upperSlidingNormalN":opened,"lateralFrictionAssumption":.70,"forceN":f,"speedMps":speed,**gate},
            "hood":{"peakUpperRollerNormalN":hood,"torqueNm":hood_demand,**closing},
            "commonDrive":{"phaseLimitA":8,"simulatedPeakTerminalV":max(r["peakTerminalV"] for r in rows),"supplyReserveV":9.5},
            "shaft":{"shockTangentialN":shear,"shaftNmUpper":torque,"hollowHexTorsionMPa":stress/1e6,
                     "assumedWorkingShearMPa":80,"ratioToAssumedWorkingShear":shaft_margin},
            "guide":{"worstLowerNormalN":lower,"singleGuideBendingMPa":stress_b/1e6,"singleGuideMidspanSagMm":1000*sag,
                     "fourWayIdealMPa":stress_b/4e6,"fourWayIdealSagMm":1000*sag/4},
            "twoSampledODCompressionMm":"12 mm relieved tread plus 22 mm sectional rear pads. Contact curves, foam shear, adhesive peel, heat and single-sided roll are unmeasured.",
            "physicalAuthorization":False}
if __name__=="__main__":
    (HERE/"engineering.json").write_text(json.dumps(build(),indent=2)+"\n",encoding="utf8",newline="\n")
