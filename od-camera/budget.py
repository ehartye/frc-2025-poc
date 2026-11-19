"""Compare installed IDs, material and branch voltage for the replacement receiver."""
import json
import math
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"fixed-low-power"));sys.path.insert(0,str(ROOT/"design/receiver-drives"))
import power
import dual_camera

def read(p):return json.loads((ROOT/p).read_bytes())

def mass():
    old=read("moving-observer/mount.json");new=read("od-camera/native.json")
    od=read("od-gate/engineering.json")["mass"]
    air=read("design/fixed-no-air/inspection.json")["cases"]["low-camera-red"]
    same=set(new["relocated"])
    if same!={"roof-camera-envelope","roof-camera-M4--37.5","roof-camera-M4-37.5"}:raise ValueError("Moved roof hardware mismatch")
    removed={k:v["kg"] for k,v in old["parts"].items() if "roof-"+k not in same}
    added={k:v*(7.95e-6 if "anchor" in k else 2.82e-6) for k,v in new["nativeVolumesMm3"].items() if k not in same}
    min_retired=sum(removed.values())*.95
    # Already-credited original PE shelf contained four 4.5 mm bores in this camera package.
    drilled=old["removed_shelf_mm3"]*.99e-6
    oldanchors=sum(read("od-gate/package.json")["volumeMm3"][k] for k in new["gateExcluded"])*7.70e-6
    # Retain the original protected branches and full moving cable budget. This allowance is incremental for the changed bracket.
    extra=.025
    delta=od["conservativeNetAdditionKg"]+drilled+sum(added.values())-min_retired-oldanchors+extra
    L=.4315;outer=.015;inner=.0134;I=(outer**4-inner**4)/12
    moving=sum(added.values())+.061+.10
    lateral=2*9.80665*moving/2
    return {"originalConditionalR103Kg":air["checker_upper_headroom_kg"],"originalCorrelatedHeadroomKg":air["paired_upper_headroom_kg"],
            "odNetUpperKg":od["conservativeNetAdditionKg"],"removedShelfBoreCreditReversalKg":drilled,
            "removedRoofKgLow":min_retired,"replacedODAnchorsLowKg":oldanchors,"newStockKgUpper":sum(added.values()),
            "addedStockKgUpperById":added,"removedRoofKgNominalById":removed,"additionalWeldAndLoopKg":extra,
            "combinedUpperDeltaKg":delta,"remainingConservativeR103Kg":air["checker_upper_headroom_kg"]-delta,
            "remainingCorrelatedR103Kg":air["paired_upper_headroom_kg"]-delta,
            "uninstalledThreeConverterStudy":{"fixedBaseConverterCount":3,"eachEnvelopeMm":[35,90,54.5],"grossKg":.75,
                "displacedCreditKg":0.,"ODRemainingConservativeR103Kg":air["checker_upper_headroom_kg"]-delta-.75,
                "endBeltRemainingConservativeR103Kg":air["checker_upper_headroom_kg"]-.75,
                "ODEvenWithoutAnyMovingCameraKg":od["cases"]["red-low-light-dedicated-masts-physically-deleted-pneumatics"]["afterUpperReplacementKg"]-.75},"finishedMarginKg":None,
            "symmetricCameraPostTipAt2gMm":1000*lateral*L**3/(3*69e9*I),
            "singlePostTipAt2gMm":2000*lateral*L**3/(3*69e9*I),"postDesignMassKg":moving}

def electrical(split_efficiency=None):
    od=read("od-gate/engineering.json")
    # Use simultaneous actual drive and slide samples from the issued 67 kg route solely as background loads.
    source=list(power.make_samples({"ticks":{}},False,25))
    cases={}
    starts=[18+s["drive"]+power.match.corrected().routes(s["sector"])[1].duration+1.5 for s in power.plans]
    # Charge the whole upper replacement on the outer stage, even though portions of the loom sit on the base.
    carrying=2*(power.rack["loaded_outer_mass_kg"]+.440+mass()["combinedUpperDeltaKg"])+.5*power.rack["moving_middle_mass_kg"]
    accel=.370/9*10/math.sqrt(3);speed=.370/3*1.875
    extension=power.motor(carrying*accel+54,speed/2,.010,power.UP5*power.UP4,.70,accel/2,3e-6)
    for label,act,park in (("modeled-parked-transfer",[(8,od["commonDrive"]["simulatedPeakTerminalV"]),(od["opening"]["phaseA"],od["opening"]["terminalV"]),(od["hood"]["phaseA"],od["hood"]["terminalV"])],True),
            ("carry-hood-hold",[(0,0),(0,0),(od["hood"]["phaseA"],.12*od["hood"]["phaseA"])],False),
            ("coincident-drive-and-transfer",[(8,od["commonDrive"]["simulatedPeakTerminalV"]),(od["opening"]["phaseA"],od["opening"]["terminalV"]),(od["hood"]["phaseA"],od["hood"]["terminalV"])],False),
            ("simultaneous-allocation",[(8,9.5),(15,9.5),(20,9.5)],False)):
        patterns={}
        for t,stage,ph,watts,needs in source:
            watt=watts.copy();needed=needs.copy()
            if any(t<=a+3 and a<t+power.DT for a in starts):
                phase=max(ph["Slide"][0],extension["phase_A"])
                vmotor=max(ph["Slide"][1],extension["terminal_V"])
                needed["Slide"]=vmotor+phase*power.electric.PHASE_R["Slide"]
                watt["Slide"]=.7+phase*needed["Slide"]/.90
            if park:
                for motor in ("DriveLeft","DriveRight"):watt[motor]=.7;needed[motor]=0.
            # Second camera allocation 6 W, both 3.3 V boards 40 mA each plus 0.5 W IO.
            watt["Observer"]+=(6+.08*3.3+.5)/.85
            if split_efficiency:
                del watt["Observer"]
                for name,(_,_,_,_,_,amps) in dual_camera.SUPPLIES.items():watt[name]=5.21*amps/split_efficiency
                watt["RIO"]+=(.08*3.3+.5)/.85
            for branch,(a,v) in zip(("LeftGrip","RightGrip","Lift"),act):
                v+=a*power.electric.PHASE_R[branch];watt[branch]=.7+a*v/.90;needed[branch]=v
            key=tuple(watt.values())+tuple(needed.values())
            patterns.setdefault(key,(t,stage,watt,needed))
        configurations={}
        for voc,res in ((12.4,.025),(11.8,.027),(11.8,.032)):
            worst=math.inf;peak=0;critical=math.inf;under=0;missing=0;branch_max={};limiting=None
            for t,stage,watt,required in patterns.values():
                limit,_=power.critical(watt,required,0,voc);critical=min(critical,limit)
                try:
                    if split_efficiency:term,current=dual_camera.solve(voc,res,watt)
                    else:_,term,current=power.electric.rail(voc,res,watt,{"PH":0})
                except ValueError:missing+=1;continue
                caps={k:term[k]-max(9.5,required.get(k,0)) for k in watt}
                key=min(caps,key=caps.get)
                if caps[key]<worst:worst=caps[key];limiting={"branch":key,"wallS":t,"background":stage,"terminalV":term[key]}
                under+=int(caps[key]<0);peak=max(peak,sum(current.values()))
                for k,v in current.items():branch_max[k]=max(branch_max.get(k,0),v)
            configurations[f"{voc}V-{res*1000:g}mOhm"]={"distinctLoadPatterns":len(patterns),"unresolved":missing,"belowFloorOrCommand":under,
                          "worstMarginV":worst,"limiting":limiting,"peakBatteryA":peak,"peakBranchA":branch_max,"maximumResistanceOhm":critical}
        cases[label]=configurations
    return cases
def three_converter_screen():
    from unittest.mock import patch
    with patch.dict(power.electric.RESIST,dual_camera.RESISTANCE):
        return {"inputBranchOhm":{k:dual_camera.RESISTANCE[k] for k in dual_camera.SUPPLIES},
                "channelAndOutputAmps":{k:[row[0],row[-1]] for k,row in dual_camera.SUPPLIES.items()},
                "inputSourceSha256":dual_camera.sha(ROOT/"fixed-low-power/dual-camera.json"),
                "efficiency85":electrical(.85),"efficiency75":electrical(.75)}
if __name__=="__main__":
    (HERE/"budget.json").write_text(json.dumps({"mass":mass(),"electrical":electrical(),"threeIndependentConverters":three_converter_screen()},indent=2)+"\n",encoding="utf8",newline="\n")
