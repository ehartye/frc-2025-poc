"""Resolve signed shelf reactions at the fixed low locks before evaluating hoist removal."""
import argparse
from pathlib import Path

import numpy as np
from fit import HERE, ROOT, j


def run(traces, registry):
    support = ROOT/"shelf-structure/coupled-mounts.json"
    static = j.read(ROOT/"fixed-lift-omission/mechanics.json")
    gravity = static["oneSideLock"]["1"]["forceN"]
    fit = j.read(HERE/"fit.json")
    if j.digest(registry)!=fit["sourceSha256"]["registeredScene"]:
        raise ValueError("Wrong registered robot")
    bounds = j.read(registry)["stowed"]
    centers = []
    for side in ("L","R"):
        b = np.array(bounds["height-"+side+"-pin"])
        centers.append((b[:3]+b[3:])/2)
    separation = (centers[0][1]-centers[1][1])/1000
    if abs(separation-.445)>.00001 or abs(centers[0][2]-422.5)>.001:
        raise ValueError("Lock centerlines changed")
    b = bounds["lift-shoe-210"]
    shoe_height = (b[5]-b[2])/1000
    per_newton = static["oneSideLock"]["1"]["pinBendingMPa"]/gravity
    cases = {}
    for case in j.read(support):
        path = traces/(case["id"]+"-support.csv")
        if j.digest(path)!=case["supportHistorySha256"]:
            raise ValueError("Wrong signed load history")
        d = np.loadtxt(path,delimiter=",",skiprows=1)
        t,fx,fy,fz,mx,my = d[:,:6].T
        # Transport the foot moment (-45,0,154) to the crossbar (-87,0,422.5).
        rise = (centers[0][2]-154)/1000
        roll = mx + rise*fy
        pitch = my - rise*fx - .042*fz
        vertical = fz - gravity  # centered parked-mass comparison; no extra damper force
        sides = {}
        for side, sign in (("left",1),("right",-1)):
            load = vertical/2 + sign*roll/separation
            i = int(np.argmax(abs(load)))
            sides[side] = {"timeS":float(t[i]),"verticalN":float(load[i]),
                "nominalPinBendingMPa":float(abs(load[i])*per_newton),
                "coincidentPitchNm":float(pitch[i])}
        i = int(np.argmax(abs(pitch)))
        cases[case["case"]] = {"supportSha256":case["supportHistorySha256"],"sampleCount":len(t),
            "bilateralVerticalCouple":sides,"equalSharePeakPinBendingMPa":float(abs(vertical).max()*per_newton/2),
            "singleSidePeakPinBendingMPa":float(abs(vertical).max()*per_newton),
            "peakAbsolutePitchNm":float(abs(pitch[i])),"pitchPeakTimeS":float(t[i]),
            "totalForeAftCoupleAtFullShoeHeightN":float(abs(pitch[i])/shoe_height)}
    j.write(HERE/"lock-transfer.json",{"sourceSha256":j.digest(support),
        "sourcePinScreenSha256":j.digest(ROOT/"fixed-lift-omission/mechanics.json"),
        "lockSpacingM":separation,"shoeFullHeightM":shoe_height,
        "centeredReferenceDeadLoadN":gravity,"cases":cases,
        "actualContactShare":None,"pinMaterialAcceptance":None})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces",type=Path)
    parser.add_argument("registry",type=Path)
    args=parser.parse_args()
    run(args.traces,args.registry)
