"""Register ball-compensated shop measurements and compare the two signed liner surfaces."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

HERE=Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_bytes())


def save(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")


def point(value):
    if not isinstance(value,list) or len(value)!=3 or not all(number(v) for v in value):
        raise ValueError("Each compensated point must have three finite millimetre coordinates")
    return np.array(value,dtype=float)


def number(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def interval(value):
    if not isinstance(value,list) or len(value)!=2 or not all(number(v) for v in value) or value[0]>value[1]:
        raise ValueError("Supply an ordered pair of finite interval endpoints")
    return value


def normal(rows):
    _,singular,vt=np.linalg.svd(rows,full_matrices=False)
    if singular[1]<=max(singular[0],1)*1e-10:
        raise ValueError("Datum contacts cannot establish a plane")
    return vt[-1]


def register(raw, layout):
    if raw.keys()!=layout.keys():
        raise ValueError("Survey needs exactly the registered mast contact IDs")
    points={k:point(v) for k,v in raw.items()}
    fore=np.array([v for k,v in points.items() if "-fore-" in k])
    inside=[np.array([v for k,v in points.items() if k.startswith(side+"-inner-")]) for side in ("L","R")]
    top=np.array([v for k,v in points.items() if k.endswith("-top")])
    means=np.array([side.mean(axis=0) for side in inside])
    fore_center=fore.mean(axis=0)
    x=normal(fore-fore_center)
    if x@(fore_center-means.mean(axis=0))<0:
        x=-x
    y=normal(np.vstack([rows-middle for rows,middle in zip(inside,means)]))
    y-=x*(x@y)
    if np.linalg.norm(y)<.1:
        raise ValueError("Mast side and fore planes are nearly parallel")
    y/=np.linalg.norm(y)
    if y@(means[0]-means[1])<0:
        y=-y
    z=np.cross(x,y)
    if z@(top.mean(axis=0)-fore_center)<=0:
        raise ValueError("Datum sides or vertical sense reversed")
    axes=np.array([x,y,z])
    offsets=np.array([fore_center@x+20.,means.mean(axis=0)@y,top.mean(axis=0)@z-1020.])
    projected={k:axes@v-offsets for k,v in points.items()}
    residuals={}
    for k,xyz in projected.items():
        idx=0 if "-fore-" in k else 1 if "-inner-" in k else 2
        residuals[k]=float(xyz[idx]-layout[k]["pointMm"][idx])
    return axes, offsets, {"axesInInstrumentFrame":axes.tolist(),"originProjectionMm":offsets.tolist(),
        "datumPlaneResidualsMm":residuals,"maxPlaneResidualMm":max(abs(v) for v in residuals.values()),
        "innerMastSpacingMm":float((means[0]-means[1])@y)}


def load_layout():
    data=read(HERE/"probe.json")
    for k,path in (("settingStepSha256",HERE/"setting.step"),("guideStepSha256",HERE.parent/"guide-mounts/guide-mounts.step")):
        if data[k]!=hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError("Survey probe layout does not match the exchanged geometry")
    contacts=[v for row in data["stations"].values() for v in row["sides"].values()]
    contacts.extend(data["mastDatums"].values())
    if any(row["materialOverlapMm3"] or row["cableIntrusionMm"] for row in contacts):
        raise ValueError("The survey includes an obstructed approach")
    return data


def form(probe):
    return {"schema":"guide-survey-1","kind":"shop","readingId":"","datumDescription":"",
        "guideStepSha256":probe["guideStepSha256"],"settingStepSha256":probe["settingStepSha256"],
        "pointErrorMm":None,"frameProjectionErrorMm":None,"pipeLengthMm":None,"pipeCenterYMm":None,
        "mastPointsMm":{k:None for k in probe["mastDatums"]},
        "linerPointsMm":{k:{s:None for s in ("L","R")} for k in probe["stations"]}}


def assess(data, probe):
    if (data.get("schema")!="guide-survey-1" or data.get("kind") not in ("shop","example")
            or any(not isinstance(data.get(k),str) or not data[k].strip() for k in ("readingId","datumDescription"))):
        raise ValueError("Identify the survey, its kind and its physical reference")
    for key in ("guideStepSha256","settingStepSha256"):
        if data.get(key)!=probe[key]:
            raise ValueError("Survey belongs to another guide revision")
    axes,offsets,frame=register(data["mastPointsMm"],probe["mastDatums"])
    length=interval(data["pipeLengthMm"])
    center=interval(data["pipeCenterYMm"])
    if length[0]<=0:
        raise ValueError("Pipe length must be positive")
    err=data["pointErrorMm"]
    bound=data["frameProjectionErrorMm"]
    if not number(err) or err<0 or (bound is not None and (not number(bound) or bound<0)):
        raise ValueError("Supply nonnegative uncertainties in mm or leave the frame allowance unknown")
    if data["linerPointsMm"].keys()!=probe["stations"].keys():
        raise ValueError("Survey needs the ten registered liner stations")
    exceptions=[]
    if bound is None:
        exceptions.append("Frame uncertainty at the liners is unknown")
    elif frame["maxPlaneResidualMm"]>bound+err+1e-8:
        exceptions.append("Mast residual exceeds the declared metrology allowance")
    rows={}
    for key,target in probe["stations"].items():
        pair=data["linerPointsMm"][key]
        if set(pair)!={"L","R"}:
            raise ValueError("Each station needs both signed faces")
        coords={s:axes@point(p)-offsets for s,p in pair.items()}
        l,r=(float(coords[s][1]) for s in ("L","R"))
        if not r<0<l:
            raise ValueError("Liner measurement signs disagree with the robot frame")
        displacement=max(max(abs(p[0]-target["xMm"]),abs(p[2]-target["zMm"])) for p in coords.values())
        if displacement>.25+1e-8:
            exceptions.append(key+" measured outside the ±0.25 mm X/Z station window")
        row={"zMm":target["zMm"],"xMm":target["xMm"],"faceYMm":{"L":l,"R":r},
             "maxStationOffsetMm":float(displacement),"widthMm":l-r,"centerYMm":(l+r)/2,
             "overlapMm":None}
        if bound is not None:
            u=err+bound
            row["overlapMm"]={"L":[center[0]+length[0]/2-l-u,center[1]+length[1]/2-l+u],
                              "R":[-center[1]+length[0]/2+r-u,-center[0]+length[1]/2+r+u]}
        rows[key]=row
    lower=[row for row in rows.values() if row["zMm"]<770]
    worst=None if exceptions else min(row["overlapMm"][s][0] for row in lower for s in ("L","R"))
    return {"schema":"guide-survey-report-1","kind":data["kind"],"readingId":data["readingId"],
        "datumDescription":data["datumDescription"],"guideStepSha256":probe["guideStepSha256"],
        "settingStepSha256":probe["settingStepSha256"],"pointErrorMm":err,"frameProjectionErrorMm":bound,
        "frame":frame,"stations":rows,"unresolved":exceptions,"minimumLowerOverlapMm":worst,
        "bothEndsContactAtAllLowerStations":None if worst is None else bool(worst>0),
        "scoringAccepted":None}


def compare(before,after):
    for k in ("guideStepSha256","settingStepSha256","kind","datumDescription"):
        if before[k]!=after[k]:
            raise ValueError("The two readings use different configurations, sources or reference datums")
    if before["readingId"]==after["readingId"]:
        raise ValueError("Use separate readings taken before and after assembly")
    missing=before["unresolved"]+after["unresolved"]
    total=None if missing else sum(r["pointErrorMm"]+r["frameProjectionErrorMm"] for r in (before,after))
    delta={}
    for key in before["stations"]:
        d={s:after["stations"][key]["faceYMm"][s]-before["stations"][key]["faceYMm"][s] for s in ("L","R")}
        delta[key]={"centerShiftMm":sum(d.values())/2,"widthChangeMm":d["L"]-d["R"],
                   "faceChangeMm":{s:None if total is None else [v-total,v+total] for s,v in d.items()}}
    return {"before":before["readingId"],"after":after["readingId"],"kind":before["kind"],
            "stationChanges":delta,"unresolved":missing,"physicalRepeatabilityQualified":None}


def example(probe,name):
    data=form(probe)
    rotation=math.radians(17)
    rz=np.array([[math.cos(rotation),-math.sin(rotation),0],[math.sin(rotation),math.cos(rotation),0],[0,0,1]])
    theta=math.radians(-23)
    rx=np.array([[1,0,0],[0,math.cos(theta),-math.sin(theta)],[0,math.sin(theta),math.cos(theta)]])
    def raw(xyz):
        return (rx@rz@np.array(xyz)+[125.,-76.,52.]).tolist()
    data.update(kind="example",readingId=name,datumDescription="same hypothetical chassis inspection fixture",
                pointErrorMm=.02,frameProjectionErrorMm=.03,pipeLengthMm=[301.625]*2,pipeCenterYMm=[-.03,.03])
    data["mastPointsMm"]={k:raw(v["pointMm"]) for k,v in probe["mastDatums"].items()}
    for k,t in probe["stations"].items():
        for s,sign in (("L",1),("R",-1)):
            move=.315 if name=="sideways" else sign*.315 if name=="both-outboard" else 0.
            data["linerPointsMm"][k][s]=raw([t["xMm"],sign*t["nominalHalfWidthMm"]+move,t["zMm"]])
    return data


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest="action",required=True)
    sub.add_parser("form").add_argument("output",type=Path)
    evaluate=sub.add_parser("check")
    evaluate.add_argument("input",type=Path);evaluate.add_argument("output",type=Path)
    pair=sub.add_parser("compare")
    pair.add_argument("before",type=Path);pair.add_argument("after",type=Path);pair.add_argument("output",type=Path)
    sample=sub.add_parser("example")
    sample.add_argument("name",choices=("nominal","sideways","both-outboard"));sample.add_argument("output",type=Path)
    args=p.parse_args()
    probe=load_layout()
    if args.action=="form":
        result=form(probe)
    elif args.action=="check":
        result=assess(read(args.input),probe)
        result["inputSha256"]=hashlib.sha256(args.input.read_bytes()).hexdigest()
    elif args.action=="compare":
        result=compare(assess(read(args.before),probe),assess(read(args.after),probe))
        result["inputSha256"]=[hashlib.sha256(path.read_bytes()).hexdigest() for path in (args.before,args.after)]
    else:
        result=example(probe,args.name)
    save(args.output,result)
