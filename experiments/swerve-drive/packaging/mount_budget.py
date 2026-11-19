"""Calculate single-corner load and dimensional/mass sensitivities for the proposed joint."""
import math
from module_placement import HERE,read,write

def run():
    frame=read(HERE/"frame.json")
    changes=frame["stockChange"]
    parts=frame["parts"]
    classes={}
    for name,v in parts.items():
        if not name.startswith(("FL-","FR-","RL-","RR-")):continue
        kind=next(k for k in ("saddle","locknut","tab-screw","rail-screw","boss") if k in name)
        density=2700 if kind in ("saddle","boss") else 7850
        row=classes.setdefault(kind,{"count":0,"volumeMm3":0,"kg":0,"densityKgM3":density})
        row["count"]+=1;row["volumeMm3"]+=v["volumeMm3"];row["kg"]+=v["volumeMm3"]*density/1e9
    # Same original 6061 skins, finished bar, and cast/tab envelopes; positive allowances
    # cover 8 boss pairs, new four-web fabrication, finishes and standalone mount fittings.
    matched=sum(v if k.endswith("RemovedMm3") else -v for k,v in changes.items())*2700/1e9
    nominal=sum(v["kg"] for v in classes.values())
    budget={"modeledNetNominalKg":nominal-matched,"newKitOnlyKg":nominal,
            "sameStockSubtractionKg":matched,"zeroChipCreditUpperKg":nominal*1.08+.16,
            "matchedStockUpperKg":nominal*1.08+.16-matched*.94,
            "fabricationAndWeldAllowanceKg":.16,"finishedHardwareAndDensityAllowanceFraction":.08,
            "stockCreditDerateFraction":.06,"physicalScaleKg":None}
    total_mass=67; g=9.80665
    # Screen two vertical g and one lateral g at a single jammed/loaded corner; this
    # is an explicitly declared prebuild limit, not a competition collision spectrum.
    vertical=2*total_mass*g
    side=total_mass*g
    arm=math.hypot(69.85,31.75)
    lever_load=vertical*arm+side*95.25
    # Any line across the two perpendicular mounting groups must be resolved with
    # its own direction. One-saddle / one-bolt cases do not inherit four-bolt sharing.
    return {"kit":classes,"mass":budget,"sourceFrameSha256":frame["stepSha256"],
            "jointScreen":{"vehicleKg":total_mass,"singleCornerVerticalN":vertical,
            "singleCornerLateralN":side,"momentIfOneSaddleNmm":lever_load,
            "oneRemainingSaddleTwoTabScrewsCoupleN":lever_load/25.4,
            "oneRemainingSaddleTwoFrameScrewsCoupleN":lever_load/21,
            "fourTabFastenersSharedDirectShearN":side/4,
            "fourFrameFastenersSharedDirectShearN":math.hypot(side,vertical)/4,
            "twoStageBossFit":"Welded 13.00 mm outside diameter boss in a 13.10 mm bore, 13 mm deep. Thread and head flats are reserved at major diameter.",
            "requiredEvidence":"Relocated rail bending/torsion, joint prying, heat affected section, weld throat, screw grip, tapped stock, installation preload, end fatigue and one-side loss require separate qualification."}}
if __name__=="__main__":write(HERE/"mount-budget.json",run())
