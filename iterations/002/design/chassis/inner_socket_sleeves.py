"""Fit captured load-spreading sleeves to the eight inner factory plate screws on the cold four-corner chassis."""
import hashlib
import json
import math
from pathlib import Path
import sys
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut

from crossmember_envelope import cylinder

HERE = Path(__file__).resolve().parent


def cut(stock, tool):
    op = BRepAlgoAPI_Cut(stock,tool)
    if not op.IsDone() or op.Shape().IsNull():
        raise ValueError("Invalid stock machining")
    return op.Shape()


def run(package):
    sys.path.insert(0,str(package/"scripts"))
    from fusion_authority import verify, register_path
    from mechanical_motion import bounds, common, distance
    from showcase_contacts import load_solids, box_gap
    from component_shape import volume
    root=HERE.parents[3]
    reg=register_path(root)
    lineage=reg.parent/"issue-005/lineage.json"
    auth=verify(root,reg,lineage)
    parts,sheets=load_solids(lineage.with_name("assembly.step"))
    if len(parts)!=552 or sheets:
        raise ValueError("Expected all 552 source solids")
    load_input=HERE/"diagonal-lower-elastic.json"
    load=json.loads(load_input.read_bytes())
    if load["authority"]!=auth:
        raise ValueError("Load study is for a different installed revision")
    sleeve_od,inner,clearance=9.975,5.25,.075
    counterbore=10.000
    up=(0,0,1)
    stock=json.loads((HERE/"socket-holes.json").read_bytes())["rails"]
    extents={key:bounds(body) for key,body in parts.items()}
    cases=[]
    for rail,definition in stock.items():
        matches=[k for k,bb in extents.items() if "socket rail [" in k and
                 all(abs(v-t)<.001 for v,t in zip(bb,definition["minimumMm"]+definition["maximumMm"]))]
        if len(matches)!=1:
            raise ValueError(f"Unresolved installed rail at measured world bounds: {rail} {matches}")
        host_name=matches[0]
        size=definition["lengthAxis"]
        for sign in (-1,1):
            xy=min((p for p in definition["boltCentersMm"] if sign*p[size]>0),key=lambda p:abs(p[size]))
            lo,hi=definition["minimumMm"][2],definition["maximumMm"][2]
            length=hi-lo-clearance
            bore=lambda z,r,h:cylinder((*xy,z),up,r,h)
            sleeve=cut(bore(lo+clearance/2,sleeve_od/2,length),bore(lo-1,inner/2,hi-lo+2))
            host=parts[host_name]
            drilled=cut(host,bore(lo-1,counterbore/2,hi-lo+2))
            removed=volume(host)-volume(drilled)
            analytic=(counterbore**2-definition["throughDiameterMm"]**2)*math.pi/4*2*definition["wallMm"]
            if abs(removed-analytic)>.01:
                raise ValueError(f"{rail} has an unexpected wall profile {removed} vs {analytic}")
            bb=bounds(sleeve)
            near={}
            for name,body in parts.items():
                if box_gap(bb,extents[name])>1:
                    continue
                candidate=drilled if name==host_name else body
                d=distance(sleeve,candidate)
                near[name]={"surfaceGapMm":d,"commonMm3":common(sleeve,candidate) if d<.0001 else 0}
            fast=[k for k in near if "91251A355" in k]
            if len(fast)!=1:
                raise ValueError(f"Unresolved actual OEM screw: {fast}")
            cases.append(dict(host=host_name,xyMm=xy,sourceScrew=fast[0],sleeveVolumeMm3=volume(sleeve),
                              netTubeCutMm3=removed,preMachiningAdverseOverlapMm3=common(host,sleeve),
                              afterMachining=near))
    before=sum(i["sleeveVolumeMm3"] for i in cases)*2700e-9
    after=before-sum(i["netTubeCutMm3"] for i in cases)*2700e-9
    by_rank={key:max(s["plateBounds"][key]["ratio"] for s in load["states"]) for key in
             ("innerScrewN","middleScrewN","outerScrewN","axialTwoWallTearout")}
    f=by_rank["innerScrewN"]
    # Per wall 60% governing signed pin load, with 3.00 mm minimum stock, minimum
    # bushing OD and maximum ID. Both walls must actually engage. Independent
    # uncontrolled one-wall load is preserved as an adverse case.
    def screen(frac,od,id_):
        area=math.pi/4*(od**2-id_**2)
        return dict(wallBearingMPa=frac*f/(od*3),sleeveShearMPa=frac*f/area,
                    shearWorkingMPa=120/math.sqrt(3),areaMm2=area)
    return dict(authority=auth,loadInputSha256=hashlib.sha256(load_input.read_bytes()).hexdigest(),cases=cases,
        stock="8 identical shop-turned 6061-T6 full-height sleeves captured between the original factory plates. Retain OEM screw and screw axes. 10.000 mm line-reamed bore through both socket walls, 9.975 mm OD, 5.25 mm ID. Face 0.075 mm shorter than measured rail height so both ends are captured by the factory plates after socket clamp. Assembly requires removal of one module. 0.025 mm diametral fit and 60/40 load sharing are design premises pending tolerance, pin slope and plate deflection.",
        innerWorstN=f,byRank=by_rank,adverseOriginalTwoWallBearingMPa=f/(4.826*6),
        strongestInnerAt60Percent=screen(.6,9.95,5.30),oneWallAdverse=screen(1,9.95,5.30),
        middleAndOuterWorstAt60PercentMPa=max(by_rank["middleScrewN"],by_rank["outerScrewN"])*.6/(4.826*3),
        directSleeveKg=before,netInstalledAdditionKg=after,weakWallLoadFractionAt120MPa=3*9.95*120/f,
        shaftWorking="Original steel screw bending, underhead/plate clamp, actual shank vs threaded section, manufacturing alignment and fatigue still need qualification. The positive shear scheme uses no friction credit. Form and access are not an approval for the OEM assembly internals.")


if __name__=="__main__":
    result=run(Path(sys.argv[1]))
    (HERE/"inner-socket-sleeves.json").write_text(json.dumps(result,indent=2)+"\n")
    print(result["netInstalledAdditionKg"],result["strongestInnerAt60Percent"],result["oneWallAdverse"])
    print([(i["xyMm"],i["afterMachining"][i["sourceScrew"]],max(j["commonMm3"] for j in i["afterMachining"].values())) for i in result["cases"]])
