"""Apply the installed assembly checker and rerun the straight-drive mass and stock case."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
from importlib.util import spec_from_file_location,module_from_spec
loader=spec_from_file_location("accounting_source",HERE.parent/"assembly-accounting/build.py")
accounting=module_from_spec(loader)
loader.loader.exec_module(accounting)
baseline,evidence,write_map=accounting.build,accounting.evidence,accounting.write_map
sys.path.insert(0,str(ROOT/"leave-park"))
from rehearse import corrected
from straight_role import path as direct_path, assess
from installed_stock import check as stock_clearance


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mass(row):
    vs=row["addedVolumesMm3"]
    steel=sum(v for k,v in vs.items() if "fillet" not in k)*7.85e-6
    al_weld=sum(v for k,v in vs.items() if "fillet" in k)*2.70e-6
    removed=sum(v*(.95e-6 if "lift-shoe" in k else 2.7e-6) for k,v in row["editedVolumesMm3"].items())
    return {"installedSteelKg":steel,"weldVolumeNominalKg":al_weld,"machiningCreditKg":removed,
            "netDeltaKg":[steel*.9+al_weld*.8+removed*1.05,steel*1.1+al_weld*1.2+removed*.95]}


def review_map(plugin,mode,color,row,output):
    path=(output/(mode+"-"+color+".json")).resolve()
    data=baseline(color,path)
    def pin(id_,p):
        data["sources"].append({"id":id_,"file":Path(os.path.relpath(p,path.parent)).as_posix(),"sha256":sha(p)})
    pin("lock-mass",HERE/"mass.json")
    pin("lock-native",HERE/"native.json")
    pin("locked-step",HERE/("locks.step" if mode=="high" else "low-locks.step"))
    ids=[]
    for k in list(row["editedVolumesMm3"])+list(row["addedVolumesMm3"]):
        identity=mode+"-lock/"+k
        ids.append(identity)
        disposition="buy" if k.endswith(("washer","locking-nut","screw")) else "make"
        item={"id":identity,"presence":"candidate","disposition":disposition}
        if disposition=="buy":
            dim="M8" if k.startswith("height-") else "M6"
            if k.startswith("stroke-"):
                dim="6 mm clevis"
            family=k.split("-")[-1]
            article="fixed-height "+dim+" "+family
            item["article"]=article
            if not any(a["id"]==article for a in data["articles"]):
                data["articles"].append({"id":article,"sku":None,"onHand":None,"unitsPerPack":None})
        data["instances"].append(item)
    data["changes"].append({"id":mode+"-lock","exclusiveGroup":"height-lock",
        "configuration":data["configuration"],"revision":data["revision"],
        "remove":list(row["editedVolumesMm3"]),"add":ids,
        "delta":evidence("lock-mass","/"+mode+"/netDeltaKg",mass(row)["netDeltaKg"])})
    data["selected"]=[mode+"-lock"]
    data["mass"]["omissions"]=["Lot, finished fabrication, locknut and weld mass remain unweighed; installed robot scale pending."]
    write_map(path,data)
    done=subprocess.run(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(path)],check=True,capture_output=True,encoding="utf-8")
    result=json.loads(done.stdout)
    return {"installedCoverage":result["coverage"]["covered"],"unrepresented":result["coverage"]["missing"],
            "orderQuantitiesKnown":result["orderQuantitiesKnown"],**{k:result["mass"][k] for k in
            ("representedRangeKg","representedMarginKg","configurationMarginKg","limitBasis")}}


def route_cases(native):
    f=corrected()
    direct=direct_path(f)
    stock=read(ROOT/"full-match/installed-stock.json")["stock"]
    original=read(ROOT/"leave-park/study.json")["preassigned_straight"]
    base=read(ROOT/"design/competition-package/shop-duplex/mass.json")["poses"]["stowed"]["empty"]
    cases={}
    for mode,row in native.items():
        if mode not in ("red","low-red-receive"):
            continue
        delta=mass(row)["netDeltaKg"]
        lo,hi=(n+d for n,d in zip(base["mass_kg"],delta))
        gross=mass(row)["installedSteelKg"]*1.1+mass(row)["weldVolumeNominalKg"]*1.2
        removals=abs(mass(row)["machiningCreditKg"])*1.05
        r=math.hypot(.425,.425)
        # Absolute perturbation bound relative to an arbitrary baseline CG in its registered disk.
        cg=base["radial_cg_upper_m"]+(gross+removals)*(r+base["radial_cg_upper_m"])/base["mass_kg"][0]
        minimum=base["load_transfer"]["turn-side"]["opposite_edge_equivalent_load_fraction"]-(gross+removals)/lo
        args={"inertiaKgM2":base["yaw_inertia_upper_kg_m2"]+gross*(r+cg)**2,
             "cgOffsetM":cg,"minSideWeightFraction":minimum,"outerAxleWeightFraction":1.0}
        results=[assess(dict(args,massKg=m),{"segments":direct.segments}) for m in (lo,hi)]
        # Covers the entire body, including added hardware, across the straight motion.
        stocks=stock_clearance(f,stock,{"complete-rectangular-envelope":row["overallBoxMm"]},direct.segments)
        cases[mode]={"loadedMassAssumption":"empty, retracted","massKg":[lo,hi],"parameters":args,
            "fieldGapM":min(x["gap_m"] for x in f.audit(direct,row["overallBoxMm"])),
            "heightClearanceToR104Mm":1066.8-row["overallBoxMm"][5],"stockGaps":stocks,
            "peakA":max(x["extrema"]["phaseA"] for x in results),"minimumBusV":min(x["extrema"]["busV"] for x in results),
            "requiredMu":max(x["extrema"]["frictionRequired"] for x in results),
            "passesDriveCases":all(x["parameterCaseClears"] for x in results),"travelM":direct.segments[0]["demand"],
            "durationS":direct.duration,"bumperMaterialPreserved":True,"issuedStartTapeMm3":original["start_tape_mm3"],
            "issuedFinalZoneMm3":original["park_zone_mm3"]}
    return cases


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--plugin-root",type=Path,required=True)
    p.add_argument("--output",type=Path,default=ROOT/"artifacts/fixed-height/maps")
    p.add_argument("--native-folder",type=Path)
    a=p.parse_args()
    if a.native_folder:
        labels=("red","blue","low-red-receive","low-red-place","low-blue-receive","low-blue-place","original-collision","synchronizer-negative")
        records={k:read(a.native_folder/(k+".json")) for k in labels}
        (HERE/"native.json").write_text(json.dumps(records,indent=2)+"\n",encoding="utf-8",newline="\n")
    records=read(HERE/"native.json")
    ledger={"high":mass(records["red"]),"low":mass(records["low-red-place"])}
    (HERE/"mass.json").write_text(json.dumps(ledger,indent=2)+"\n",encoding="utf-8",newline="\n")
    a.output.mkdir(parents=True,exist_ok=True)
    summary={"checkerSha256":sha(a.plugin_root/"scripts/assembly-accounting.mjs"),"mass":{},
             "straightDrive":route_cases(records)}
    for mode in ("high","low"):
        for color in ("red","blue"):
            row=records[color if mode=="high" else "low-"+color+"-place"]
            # Source mass is computed once from the red geometry after configured-volume equivalence.
            if any(abs(mass(row)["netDeltaKg"][i]-ledger[mode]["netDeltaKg"][i])>1e-9 for i in (0,1)):
                raise ValueError("Alliance mass differs")
            summary["mass"][mode+"-"+color]=review_map(a.plugin_root,mode,color,records["red" if mode=="high" else "low-red-place"],a.output)
    moving=read(ROOT/"design/lift-arrest/study.json")["size"]["massRangeKg"][1]
    bound=moving+max(ledger["high"]["netDeltaKg"][1],ledger["low"]["netDeltaKg"][1])
    summary["mountDemand"]={"movingMassUpperKgWithAllLockMass":bound,"caseAssumptions":{},
        "assumedPinDiameterMm":8,"singleSideBearingSpanMm":18,"mastWallMm":2,"bearingSleeveOdMm":12,
        "clearanceTravelCenteredMm":.25,"clearanceTravelAcrossOppositeExtremesMm":.5,
        "controlledSeriesGapsMm":[.1,.1,.05],"risk":"Rigidity, tolerance, joining, single-side torsion and impact loads require representative bench evidence."}
    for multiplier in (1,3,10):
        force=bound*9.80665*multiplier
        summary["mountDemand"]["caseAssumptions"][str(multiplier)+"g"]={
            "entireVerticalLoadOnOneLockN":force,"singleShearMPa":force/(math.pi*8**2/4),
            "cantileverBendingMPa":32*force*18/(math.pi*8**3),"mastTwoWallAverageBearingMPa":force/(2*2*12),
            "weldAverageShearMPaIfFourFullSeams":force/(4*100*3/math.sqrt(2))}
    (HERE/"review.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    main()
