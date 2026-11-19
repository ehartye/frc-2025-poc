"""Static strip model for the deck and one-sided permanent-lock demand."""
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
E = 68300.0
G = 9.80665
POCKET_CENTRES = (-168,-68,-25,25,68,168)


def strip(force, width, at, pocketed):
    # Simple compression-bearing lines at the two inner rails. Unit-load integration
    # gives deflection at the loading point for the measured variable-thickness strip.
    span = 464.
    a = at + span/2
    left = (span-a)/span
    compliance = 0.
    peak = 0.
    for i in range(928):
        x = (i+.5)*.5
        y = x-span/2
        h = 2.25 if pocketed and any(abs(y-c)<16 for c in POCKET_CENTRES) else 4.
        unit_moment = left*x-max(0,x-a)
        compliance += unit_moment**2 * .5/(E*width*h**3/12)
        peak = max(peak,6*force*unit_moment/(width*h*h))
    return {"forceN":force,"stripWidthMm":width,"yMm":at,"deflectionMm":force*compliance,"stressMPa":peak}


def screen():
    fit=json.loads((HERE/"fit.json").read_bytes())
    lock=json.loads((ROOT/"design/fixed-height/review.json").read_bytes())["mountDemand"]
    added=json.loads((ROOT/"integrated-slide/mass.json").read_bytes())["moving"]
    # #53 predates the moving camera. Full new allocations on one pin; motor stays fixed.
    upper=lock["movingMassUpperKgWithAllLockMass"]+.440+added["extraToIssuedPlantKg"]
    demand={}
    for gs in (1,3,10):
        f=upper*G*gs
        demand[str(gs)]={"forceN":f,"pinBendingMPa":32*f*18/(math.pi*8**3),
                         "pinShearMPa":f/(math.pi*4**2),"mastBearingMPa":f/(2*2*12)}
    # The two longitudinal rail contact lines lie 464 mm apart. Ignore load spreading
    # along X and end-rail assistance. Explicit assumed effective strip widths.
    plate={}
    for name,mass,w,y,in_trial in (("battery",6.17,76,-14.5,False),
                                   ("controllerAndWiring",.40,69,117.25,True),
                                   ("platformPost",2.,8,205,False),
                                   ("mastSingleSide",upper,50,222.5,False)):
        plate[name]={shape:strip(3*G*mass,w,y,shape=="sixPockets" and in_trial)
                     for shape in ("original","sixPockets")}
    plate["servicePoint"]={shape:strip(100,30,25,shape=="sixPockets") for shape in ("original","sixPockets")}
    # Show the additional full-retention reaction needed if one end of a locked shelf
    # unloads the chassis deck. Bearing contact alone cannot carry the uplift.
    paper=1000. # project shop rail 1 kN comparison only
    force=upper*G*3
    stock_volume=6*((252-32)*32+math.pi*16**2)*1.75
    width=516
    bands=[(width-6*32,4.,2.),(6*32,2.25,4-2.25/2)]
    centroid=sum(w*t*z for w,t,z in bands)/sum(w*t for w,t,z in bands)
    inertia=sum(w*t**3/12+w*t*(z-centroid)**2 for w,t,z in bands)
    report={"sourceDeckBoundsMm":fit["deckLoadPaths"]["deck-main"],"releasedDeckRemovalCreditKg":0,
        "pocketVolumeMm3":stock_volume,"nominalPocketChipsG":stock_volume*.00270,
        "sectionRatio":inertia/(width*4**3/12),"localRigidityRatio":(2.25/4)**3,
        "deckTouchingSupports":["cross-front","cross-rear","drive-rail-inner-L","drive-rail-inner-R"],
        "deckToFrameTensionFastenerCountInSource":0,"oneDirectionBearingOnly":True,
        "stripAt3g":plate,"oneSideLockLoadKg":upper,"oneSideLock":demand,
        "shelfOverturnStudy":{"verticalN":force,"assumedCgBeyondSupportMm":200,"endRailsApartMm":680,
                              "additionalEndCoupleN":force*200/680,
                              "horizontalFixtureForceN":paper,"assumedApplicationAboveDeckMm":350,
                              "horizontalEndCoupleN":paper*350/680},
        "physicalAcceptance":None}
    (HERE/"mechanics.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    screen()
