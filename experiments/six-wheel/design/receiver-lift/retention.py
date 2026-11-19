"""Static load paths, differential payout and energy bounds for duplicate suspension lines."""
import json
import math

from delivery_load import audit
from duplex import native, volume
from layer_inventory import metadata
from size_lift import G, INCH, ROTOR_STACK_INERTIA, profile


def suspended_mass(shapes):
    """Maximum modeled rope length and unmeasured termination/washers/nuts allowance in kg."""
    additions = {}
    for k, shape in shapes.items():
        frame, _, rho = metadata(k)
        if frame == "base" or k == "lift-crossbar-drilled":
            continue
        additions[k]=volume(shape)[0]*rho*1e-9
    return additions


def catch(mass, slack_mm, stiffness_n_mm, speed_mps):
    # Conservative full loaded mass on ONE replacement tail. Shaft held at its current height.
    # Spring starts unstrained. Rope/termination/jig compliance is an expressly unmeasured range.
    k=1000*stiffness_n_mm
    energy=.5*mass*speed_mps**2+mass*G*slack_mm/1000
    x=(mass*G+math.sqrt((mass*G)**2+2*k*energy))/k
    drop=x*1000+slack_mm
    return {"slackMm":slack_mm,"effectiveStiffnessNPerMm":stiffness_n_mm,"dropMm":drop,
            "peakLineN":k*x,"shoeOffsetMm":58*drop/504,
            "withinNominal1mmShoeGap":58*drop/504 < 1, "withinReserved0p8mm":58*drop/504 < .8}


def common_shaft(mass,v):
    # All drums running free under a broken shaft. The backup legs can stay taut throughout.
    trip=.35
    threshold=(trip**2-v**2)/(2*G)
    trials=[]
    for lag_ms in (10,20,50):
        t=lag_ms/1000
        engage_v=trip+G*t
        fall=threshold+trip*t+.5*G*t*t
        for net_decel_g in (1,2):
            braking=engage_v**2/(2*G*net_decel_g)
            trials.append({"tripSpeedMps":trip,"lagMs":lag_ms,"netDecelG":net_decel_g,
                           "minimumStopMm":1000*(fall+braking),"meanTotalRailReactionN":mass*G*(1+net_decel_g)})
    stroke=.380
    energy=mass*G*stroke+.5*mass*v*v
    return {"unprotected380mmEnergyJ":energy,"unprotectedBottomSpeedMps":math.sqrt(v*v+2*G*stroke),
            "linear20mmBufferPeakN":2*(energy/.020+mass*G), "independentRailBrakeHypotheses":trials,
            "installedIndependentOverspeedDevice":None}


def evaluate():
    installed=audit()
    shapes=native(560)
    additions=suspended_mass(shapes)
    # Increment relative to the 401-body geometry. Primary masts/crossbar were drilled, the shaft extended.
    fixed={}
    for name,shape in shapes.items():
        if name in additions or name in ("long-common-shaft","lift-crossbar-drilled") or name.endswith("drilled-mast"):
            continue
        _, _, rho=metadata(name)
        fixed[name]=volume(shape)[0]*rho*1e-9
    fixed["longer-steel-hex-shaft"]=(640-540)*(math.sqrt(3)/2)*12.7**2*7850e-9
    # Retain full old crossbar mass (four holes removed); retaining hardware and eye splices omitted by CAD.
    extra=sum(additions.values())+.25
    dry=installed["dryKg"][1]+extra
    mass=installed["loadedKg"][1]+extra
    minimum_empty=installed["dryKg"][0]
    radii=[(v+.125)*INCH/2 for v in (.750,.787)]
    travel=380
    common_radius_mm=.0254*(.750+.125)/2*1000
    mismatch=lambda diff: travel*abs(diff)/common_radius_mm
    # Setup length + paired effective pitch-radius tolerance + bedding/stretch/fleet/wear terms.
    slack=.30+mismatch(.012)+.45+.30
    speed=30*.270*.25**2/2
    checks=[catch(mass,slack,k,speed) for k in (90,120)]
    failure=catch(mass,slack+mismatch((.787-.750)*25.4/2),40,speed)
    slow=catch(mass,slack,40,speed)
    peak=max(r["peakLineN"] for r in checks)
    shaft_moment=peak*0.022*0.072/(0.022+0.072)
    # Circular 12.7 mm diameter inscribed inside the across-flats hex, conservative section.
    shaft_bend=32*shaft_moment/(math.pi*.0127**3)*1e-6
    # Loading from the outboard clevis to the near mounting bolt, ignoring load sharing.
    # Use narrow 23 x 12 mm inboard aluminum neck and the most distant bolt at |y|=214.
    plate_bend=6*peak*(292-214)/(23*12**2)
    screw_couple=peak*(292-232)/(232-214)
    static_pin=2*mass*G
    service_pin_stress=32*static_pin*40/(math.pi*8**3)
    # Reflect the extra shaft length, two complete drums, and both upper sheaves to the motor.
    # Outer-radius ring bounds include an extra 50 g rotating tail/hub allowance on each drum.
    drum_kg=sum(m for k,m in fixed.items() if "-drum-" in k)+.10
    sheave_kg=sum(m for k,m in fixed.items() if "sheave" in k)
    added_inertia=(drum_kg*.0207**2+fixed["longer-steel-hex-shaft"]*.00734**2+
                   sheave_kg*.022**2*(max(radii)/.0205)**2)/16**2
    schedule={}
    for name,dist,t,m in (("loaded-receive-carry",.270,2,mass),("loaded-carry-place",.110,1.5,mass),
                          ("empty-place-carry",.110,1,dry),("empty-carry-receive",.270,2,dry),
                          ("too-fast-return",.270,1,dry)):
        schedule[name]=[profile(dist,t,m,r,16,.78,ROTOR_STACK_INERTIA+added_inertia) for r in radii]
    return {"schema":"receiver-independent-retention-screen/1","registeredMoving":installed,
            "candidateMovingAddedKg":additions,"candidateFixedAddedKg":fixed,
            "fixedAdditionUpperEstimateKg":sum(fixed.values())+.35, "additionalFixedHardwareAllowanceKg":.35,"additionalMovingTerminationAllowanceKg":.25,
            "candidateDryUpperKg":dry,"candidateLoadedUpperKg":mass,"candidateMeasuredKg":None,
            "perSideSlackBudgetMm":slack,"effectiveRadiusMismatchLimitMm":.012,
            "historicalMixedCoreDriftMm":mismatch((.787-.750)*25.4/2),"oneSideCatch":checks,
            "mixedCoreControl":failure,"compliantLineControl":slow,
            "componentScreen":{"peakOneSideLineN":peak,"topSheaveReactionN":2*peak,
                "outboardShaftBendingMPa":shaft_bend,"plateCantileverStressMPa":plate_bend,
                "carriageBoltAdditionalTensionN":screw_couple,"servicePinOneSide2xCantileverMPa":service_pin_stress,
                "fullWeightTorqueNm":mass*G*max(radii),"singleFailureDrumTorqueNm":peak*max(radii),
                "minimumEmptyLineNIfTwoPrimaryShare":(minimum_empty*G-20)/2,
                "commonShaftFreeRotationRemainingDropBoundMm":None},
            "commonShaftLoss":common_shaft(mass,speed),
            "additionalMotorReflectedInertiaKgM2":added_inertia,"schedule":schedule,"physicalQualification":False}


if __name__=="__main__":
    print(json.dumps(evaluate(),indent=2,allow_nan=False))
