"""Reconcile the CB-1, light carriers and a completely populated terminal-stop study."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE.parent/"assembly-accounting"))
from build import build, evidence, write_map
sys.path.insert(0,str(HERE.parent/"lift-arrest"))
from sizing import buffer, ENGAGE, TOP


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mass_and_loads(native):
    export=native["red-place"]["mountedExport"]["volumeMm3"]
    supplier,fastener=read(HERE/"sources.json")
    bracket=sum(v for key,v in export.items() if key.endswith("bracket-reserve"))*2.70e-6
    removed=sum(native["red-place"]["materialRemovedMm3"].values())*2.70e-6
    steel=sum(v for key,v in export.items() if key.endswith(("sleeve","washer","screw","locknut")))*7.85e-6
    nut=sum(v for key,v in export.items() if key.endswith("KM20"))*7.85e-6
    # Catalog mass scope is ambiguous. Credit two included nuts only at the low endpoint.
    catalog=2*supplier["catalogMassKg"]
    low=catalog-.010+.95*bracket+.90*(steel+.5*nut)-1.05*removed
    high=catalog+.010+1.05*bracket+1.10*(steel+nut)-.95*removed
    bridge=read(HERE.parent/"carriage-bridge/review.json")
    moving=bridge["liftMassRangeKg"][1]
    cases={key:buffer(moving,share,start) for key,share,start in (
        ("balanced",.5,TOP),("uneven",.7,TOP),("single",1,TOP),("slowBalanced",.5,ENGAGE))}
    force=cases["uneven"]["idealMeanForceN"]
    couple=force*63/29
    return {"basis":"Signed estimates vs issued 545 assembly before stops. Catalog and material allowances are project scenarios.",
            "groupKg":{"twoBrackets":bracket,"fourScrewSleeveNutSetsWithEightWashers":steel,
                       "fourKM20Nominal":nut,"twoCatalogDampers":catalog,"removedMastAluminum":-removed},
            "nominalDeltaKgIfOneKM20IncludedPerDamper":catalog+bracket+steel+nut*.5-removed,
            "addedKg":[low,high],"movingUpperKg":moving,"terminalCases":cases,
            "attachmentScreen":{"meanPadN":force,"meanCoupleNm":force*.063,"axialCoupleN":couple,
                "directShearPerScrewN":force/2,"M6class8p8ProofN":fastener["proofN"],
                "proofToMeanAxialRatio":fastener["proofN"]/couple,"doubleWallProjectedBearingMPa":force/2/(4*10),
                "meanBendingMPaAtGross32x16Arm":6*force*63/(32*16**2),
                "projectedBoreToExistingCrossSleeveGapMm":10-5.05-4,
                "requiredSupport":"No peak force, preload, joint stiffness, sleeve pull-through, local mast fatigue or test evidence."}}


def configured(color,path):
    data=build(color,path,["light","bridge","mounted-stop"])
    def pin(id,file):
        data["sources"].append({"id":id,"file":Path(os.path.relpath(file,path.parent)).as_posix(),"sha256":sha(file)})
    pin("cb1",HERE.parent/"carriage-bridge/review.json")
    pin("mounted-study",HERE/"study.json")
    pin("mounted-step",HERE/"mounted-place.step")
    pin("mount-native",HERE/"native.json")
    pin("mount-authority",HERE/"sources.json")
    registration={"revision":data["revision"],"configuration":data["configuration"]}
    def instance(name,disposition,article=None):
        key="terminal-detail/"+name
        part={"id":key,"presence":"candidate","disposition":disposition}
        if article:
            part["article"]=article
            if not any(a["id"]==article for a in data["articles"]):
                data["articles"].append({"id":article,"sku":None,"onHand":None,"unitsPerPack":None})
        data["instances"].append(part)
        return key
    bridge=read(HERE.parent/"carriage-bridge/review.json")
    native=read(HERE/"native.json")["red-place"]["mountedExport"]["volumeMm3"]
    parts=[]
    for name in native:
        if name.endswith("-rod"):
            continue  # The piston rod and body identify a single supplied damper.
        if name.endswith("body-reserve"):
            parts.append(instance(name,"buy","ACE MC225EUM"))
        elif name.endswith("KM20"):
            parts.append(instance(name,"undecided") if "upper" in name else instance(name,"buy","Additional ACE KM20 locknut"))
        elif name.endswith(("screw","washer","locknut")):
            kind="M6 x 45 socket screw 8.8" if name.endswith("screw") else ("M6 washer" if name.endswith("washer") else "M6 prevailing-torque nut")
            parts.append(instance(name,"buy",kind))
        else:
            parts.append(instance(name,"make"))
    for item in data["changes"]:
        if item["id"]=="stop":
            item["exclusiveGroup"]="terminal-mount"
    study=read(HERE/"study.json")
    data["changes"].extend([
        {"id":"bridge",**registration,"remove":[bridge["retiredId"]],"add":[instance("CB-1","make")],
         "delta":evidence("cb1","/changeKg",bridge["changeKg"])},
        {"id":"mounted-stop",**registration,"exclusiveGroup":"terminal-mount","remove":[f"backup-{tag}-drilled-mast" for tag in "LR"],"add":parts,
         "delta":evidence("mounted-study","/addedKg",study["addedKg"])}])
    data["mass"]["omissions"]=[
        "Catalog weight coverage, manufactured mounts, finished robot weights and stock lots require measurement.",
        "Midtravel arrest, qualified peak reaction and outstanding parts remain open."]
    return data


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin-root",required=True,type=Path)
    parser.add_argument("--native-folder",type=Path,help="Refresh recorded native results from newly generated pose files.")
    parser.add_argument("--output",type=Path,default=ROOT/"artifacts/terminal-mounts")
    args=parser.parse_args()
    if args.native_folder:
        inputs=[f"{color}-{pose}" for color in ("red","blue") for pose in ("place","receive","transport","stowed")]
        native={name:read(args.native_folder/(name+".json")) for name in inputs+["original"]}
        (HERE/"native.json").write_text(json.dumps(native,indent=2)+"\n",encoding="utf-8",newline="\n")
    native=read(HERE/"native.json")
    if sha(HERE/"mounted-place.step")!=native["red-place"]["mountedExport"]["sha256"]:
        raise ValueError("Placement export differs from its reopen record")
    if any(any(row[k] for k in ("installedIntersectionsMm3","kitInternalIntersectionsMm3","toolApproachIntersectionsMm3"))
           for name,row in native.items() if name!="original"):
        raise ValueError("Revised candidate has a registered interference")
    study=mass_and_loads(native)
    (HERE/"study.json").write_text(json.dumps(study,indent=2)+"\n",encoding="utf-8",newline="\n")
    args.output.mkdir(parents=True,exist_ok=True)
    cli=args.plugin_root/"scripts/assembly-accounting.mjs"
    review={"checkerSha256":sha(cli),"configurations":{}}
    for color in ("red","blue"):
        path=(args.output/(color+".json")).resolve()
        data=configured(color,path)
        write_map(path,data)
        result=subprocess.run(["node",str(cli),str(path)],capture_output=True,check=True,encoding="utf-8")
        report=json.loads(result.stdout)
        mass=report["mass"]
        review["configurations"][color]={"configuration":report["configuration"],"selected":report["selected"],
            "representedInstances":report["representedInstances"],"covered":report["coverage"]["covered"],
            "missing":report["coverage"]["missing"],"orderQuantitiesKnown":report["orderQuantitiesKnown"],
            **{k:mass[k] for k in ("representedRangeKg","representedMarginKg","configurationMarginKg","limitBasis")}}
        data["selected"].append("stop")
        negative=path.with_name("duplicate-"+color+".json")
        write_map(negative,data)
        bad=subprocess.run(["node",str(cli),str(negative)],capture_output=True,encoding="utf-8")
        if bad.returncode==0 or "Incompatible alternatives" not in bad.stderr:
            raise ValueError("Original and mounted stop double-credit control failed")
        review["configurations"][color]["overlappingStopRejected"]=True
    (args.output/"accounting.json").write_text(json.dumps(review,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    main()
