"""Independent screw load, speed, mass and de-energized descent comparisons."""
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
DESIGN = HERE.parent
sys.path.insert(0,str(DESIGN/"receiver-lift"))
import size_lift

G, INCH, LB, FT = 9.80665, .0254, .45359237, .3048
LENGTH = .520
HALF_ANGLE = math.radians(14.5)
# nominal inch major, threads/inch, inch root, lb/ft, bronze efficiency
THREADS = {"half-8":(.5,8,.332,.489,.37), "five-eighths-8":(.625,8,.457,.820,.31),
           "three-quarters-6":(.75,6,.537,1.150,.34)}
SCHEDULE = (("receiveToCarry",.940,.670,2),("carryToPlace",.670,.560,1.5),
            ("placeToCarry",.560,.670,1),("carryToReceive",.670,.940,2))
BASE = DESIGN/"competition-package"/"shop-duplex"/"mass.json"
OLD = DESIGN/"receiver-lift"/"delivery-retention.json"
INSPECTION = DESIGN/"electrical-inspection"/"inspection.json"
PRIOR = DESIGN/"lift-arrest"/"study.json"
HEAVY_CARRIER = DESIGN/"installed-procurement"/"steel-review.json"
LIGHT_CARRIER = DESIGN/"installed-procurement"/"clamp-review.json"
ROOT = DESIGN.parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def shaft(major,tpi,root,linear_weight,eff):
    lead = INCH/tpi
    pitch_r = (major*INCH-lead/2)/2
    r, core_r = major*INCH/2, root*INCH/2
    unit_mass = linear_weight*LB/FT*LENGTH
    # Upper geometric inertia: a solid steel minor core plus all residual stock at the outer radius.
    core_mass = 7850*math.pi*core_r**2*LENGTH
    if core_mass > unit_mass:
        raise ValueError("Minor cylinder heavier than the screw")
    inertia = .5*core_mass*core_r**2+(unit_mass-core_mass)*r**2
    tan_lead = lead/(2*math.pi*pitch_r)
    return {"leadM":lead,"radiusM":lead/(2*math.pi),"pitchRadiusM":pitch_r,"unitMassKg":unit_mass,
            "singleInertiaUpperKgM2":inertia,"frictionHoldThreshold":tan_lead*math.cos(HALF_ANGLE),
            "forwardEfficiency":eff,"criticalFixedSupportedRpm":4.76e6*root*1.55/(2*(LENGTH/INCH)**2)}


def profile(u,a,b,duration):
    s=10*u**3-15*u**4+6*u**5
    vel=(b-a)*30*u**2*(1-u)**2/duration
    accel=(b-a)*60*u*(1-u)*(1-2*u)/duration**2
    return a+(b-a)*s,vel,accel


def drive(thread, motor_per_screw, mass):
    radius=thread["radiusM"]
    rotor_j=size_lift.ROTOR_STACK_INERTIA
    shaft_j=2*thread["singleInertiaUpperKgM2"]
    rpm=volts=amps=surface=0
    for _,a,b,duration in SCHEDULE:
        for i in range(1001):
            _,v,accel=profile(i/1000,a,b,duration)
            force=mass*(G+abs(accel))+20
            # The upper screen treats raising and lowering with the same positive demand.
            shaft_torque=force*radius/thread["forwardEfficiency"]+shaft_j*abs(accel)/radius
            motor_torque=shaft_torque/(motor_per_screw*.95)+rotor_j*abs(accel)*motor_per_screw/radius
            amp=1.8+(105-1.8)*motor_torque/2.6
            omega=abs(v)/radius
            voltage=omega*motor_per_screw/size_lift.OMEGA_FREE*(12-(12/105)*1.8)+amp*(12/105)
            rpm=max(rpm,omega*60/(2*math.pi))
            surface=max(surface,omega*thread["pitchRadiusM"]*60/FT)
            volts,amps=max(volts,voltage),max(amps,amp)
    return {"ratioMotorPerScrew":motor_per_screw,"peakScrewRpm":rpm,"peakSurfaceFtMin":surface,
            "upperInputV":volts,"upperMotorA":amps,"shaftCriticalScreen":rpm<thread["criticalFixedSupportedRpm"],
            "schedulingScreen":volts<=9.5 and amps<=35 and rpm<thread["criticalFixedSupportedRpm"]}


def coast(thread, mu, mass, ratio, *, coupled):
    pitch=thread["pitchRadiusM"]
    radius=thread["radiusM"]
    tan_lead=radius/pitch
    f=mu/math.cos(HALF_ANGLE)
    # Positive torque needed to continue descent; friction already includes the work of gravity.
    tau=mass*G*pitch*(f-tan_lead)/(1+f*tan_lead)
    if tau<=0:
        return {"mu":mu,"selfHolding":False,"stopDistanceMm":None}
    inertia=2*thread["singleInertiaUpperKgM2"]+mass*radius**2
    if coupled:
        inertia+=size_lift.ROTOR_STACK_INERTIA*ratio**2
    worst=None
    for name,a,b,duration in SCHEDULE[:2]:
        for i in range(1001):
            u=i/1000
            h,v,_=profile(u,a,b,duration)
            # No powered control, motor electrical brake or additional guide drag credit.
            distance=.5*inertia*(v/radius)**2/tau*radius
            margin=h-.555-distance
            if worst is None or margin < worst["marginToBaselineArmMm"]/1000:
                worst={"leg":name,"phase":u,"heightMm":1000*h,"entryMps":abs(v),
                       "coastMm":1000*distance,"marginToBaselineArmMm":1000*margin}
    return {"mu":mu,"selfHolding":True,"screwRetardingNm":tau,"motorCoupled":coupled,"worstPhase":worst}


def mass_budget(thread):
    mass=json.loads(BASE.read_bytes())
    base=json.loads(OLD.read_bytes())
    group=json.loads(INSPECTION.read_bytes())["projectMassAllowancesKg"]["lift drive, ratchet, drums, lines, hardware and release"]
    drop_fixed=["-drum-", "-sheave-", "upper-sheave", "upper-pin", "top-hanger","outboard-bearing","bearing-support","running-line","longer-steel"]
    drop_moving=["clevis","thimble","spliced-eye","return-line"]
    retired={k:v for key,terms in (("candidateFixedAddedKg",drop_fixed),("candidateMovingAddedKg",drop_moving))
             for k,v in base[key].items() if any(t in k for t in terms)}
    # Upper estimate models a bronze sleeve with its smallest bore at thread minor radius.
    # Mass of unmachined sleeves is only budgeted for the selected commercial 3/4-6 configuration.
    bronze_each=8600*math.pi*((1.25*INCH/2)**2-(.537*INCH/2)**2)*(1.50*INCH)
    heavy=json.loads(HEAVY_CARRIER.read_bytes())["deltaKg"]
    light=json.loads(LIGHT_CARRIER.read_bytes())["deltaKg"]
    room=mass["poses"]["stowed"]["R103_margin_at_high_kg"]-heavy
    shafts=2*thread["unitMassKg"]
    return {"baseR103HeadroomKg":mass["poses"]["stowed"]["R103_margin_at_high_kg"],"shopCarrierAllowanceKg":heavy,
            "lightCarrierAllowanceKg":light,"headroomWithLightCarrierKg":mass["poses"]["stowed"]["R103_margin_at_high_kg"]-light,
            "roomBeforeNewLiftKg":room,"addOnShaftsKg":shafts,"fourSleeveEstimateKg":4*bronze_each,
            "v1DrivetrainAllowanceRetiredKg":group,"explicitDuplexRetirementsKg":retired,
            "retiredDuplexMassKg":sum(retired.values()),"fullReplacementRemainderAtUpperKg":room+group[1]+sum(retired.values())-shafts-4*bronze_each,
            "sameRoomWithLowOriginalDriveCreditKg":room+group[0]+sum(retired.values())-shafts-4*bronze_each}


def run():
    from drag import evaluate
    baseline=json.loads(PRIOR.read_bytes())["size"]["massRangeKg"]
    cases={}
    for name,params in THREADS.items():
        th=shaft(*params)
        cases[name]={"screw":th,"drive":[drive(th,g,baseline[1]+1.5) for g in (.65,.80,1)]}
    th=cases["three-quarters-6"]["screw"]
    budget=mass_budget(th)
    friction_cases={}
    for name,m in (("light",baseline[0]),("loadedWithMovingAllowance",baseline[1]+1.5)):
        friction_cases[name]=[coast(th,mu,m,.8,coupled=c) for mu in (.05,.08,.10,.15,.20) for c in (False,True)]
    tilt_limit_mm=.8*510/58
    first=next(i for i in range(2001) if abs(profile(i/2000,.940,.670,2)[0]-.940)*1000>tilt_limit_mm)
    return {"schema":"independent-twin-screw-replacement/1",
            "sourceSha256":{p.relative_to(ROOT).as_posix():sha(p) for p in (
                BASE,OLD,INSPECTION,PRIOR,HEAVY_CARRIER,LIGHT_CARRIER,DESIGN/"lift-arrest"/"plate-relief.json",
                ROOT/"mission"/"candidate.json",ROOT/"scripts"/"station_l1_mission.py",
                ROOT/"scripts"/"station_l1_phase.py",ROOT/"sourced-match"/"shop-545.json")},
            "threads":cases,"mass":budget,"assumedExtraMovingKg":1.5,"coast":friction_cases,
            "passiveDragRange":evaluate(th,(baseline[0],baseline[1]+1.5)),
            "splitDrive":{"spanMm":510,"shoeLengthMm":58,"availableSideMm":.8,
                          "differentialMmAtGuideLimit":tilt_limit_mm,"commandTimeMsFromStopped":first,
                          "differentialProof":False},
            "qualifiedForInstallation":False}


if __name__=="__main__":
    print(json.dumps(run(),indent=2,allow_nan=False))
