"""Park four unspliced diagonal ties at the actual four removable terminal interfaces."""
from pathlib import Path
import argparse
import json
import math
import sys
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec
from four_corner_packaging import polygon

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]


def strut(sx,sy,offset,width,height,z,wall):
    # Both ends are mitered to the planes of the two actual 1×3 terminal tubes.
    start=np.array([sx*319.2,sy*offset]);end=np.array([sx*offset,sy*314.2])
    vec=(end-start)/np.linalg.norm(end-start);side=np.array([-vec[1],vec[0]])
    def prism(w,h,z0):
        corners=[]
        for tip,sign,n in ((start,1,0),(end,1,1),(end,-1,1),(start,-1,0)):
            q=tip+sign*w/2*side
            q+=vec*((tip[n]-q[n])/vec[n])
            corners.append([q[0],q[1],z0])
        return BRepPrimAPI_MakePrism(polygon(corners),gp_Vec(0,0,h)).Shape()
    outer=prism(width,height,z-height/2)
    inner=prism(width-2*wall,height-2*wall,z-height/2+wall)
    return BRepAlgoAPI_Cut(outer,inner).Shape(), float(np.linalg.norm(end-start)), vec


def run(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify,digest
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import common,distance,bounds,bearing_area
    register=register_path(ROOT);line=register.parent/"issue-005/lineage.json"
    auth=verify(ROOT,register,line)
    parts,sheets=load_solids(line.with_name("assembly.step"))
    if len(parts)!=552 or sheets:raise ValueError("Unissued lower")
    mass=json.loads((HERE/"inspection-mass-552.json").read_bytes())
    if mass["authority"]!=auth:raise ValueError("Changed installed lower")
    heavy=mass["cases"]["heavy-offset-control"]["fieldKg"]
    impact=heavy*1.8**2/.018
    bbs={k:bounds(v) for k,v in parts.items()}
    rows={};stock_mass=0
    for sx,sy in ((1,1),(-1,1),(-1,-1),(1,-1)):
        tube,L,vec=strut(sx,sy,65,25.4,50.8,132.15,3.175)
        b=bounds(tube);close=[]
        for k,v in parts.items():
            if box_gap(b,bbs[k])>2:continue
            cross=np.array([-vec[1],vec[0]]);bb=bbs[k]
            xy=np.array([(x,y) for x in (bb[0],bb[3]) for y in (bb[1],bb[4])])
            normal=(xy-np.array([sx*319.2,sy*65]))@cross
            if normal.min()>14.7 or normal.max()<-14.7:continue
            dist=distance(tube,v)
            if dist<2:
                commonV=common(tube,v) if dist<.001 else 0
                item=dict(placed=k,gapMm=dist,commonMm3=commonV)
                if "Removable port terminal" in k and dist<.001:
                    item["twoPlaneCandidateAreasMm2"]=[bearing_area(tube,v,axis) for axis in ((1,0,0),(0,1,0))]
                close.append(item)
        row=dict(lengthMm=L,articleMassKg=L/1000*.809*.45359237/.3048,bodyBoundsMm=b,neighborsWithin2Mm=close)
        rows[f"{sx},{sy}"]=row;stock_mass+=row["articleMassKg"]
    width,height,wall=25.4,50.8,3
    area=width*height-(width-2*wall)*(height-2*wall)
    Iz=(width*height**3-(width-2*wall)*(height-2*wall)**3)/12
    Iy=(height*width**3-(height-2*wall)*(width-2*wall)**3)/12
    axial=impact/min(abs(vec))
    moment=axial*max(abs(132.15-h) for h in (114.3,150))
    stress=axial/area+moment*(height/2)/Iz
    current=json.loads((HERE/"low-line-frame-clock.json").read_bytes())["remodeledStructuralItems"]
    original=sum(x["kg"] for x in current[:4])
    return dict(authority=auth,massRegisterSha256=digest(HERE/"inspection-mass-552.json"),case=rows,
                upperCaseN=impact, conditionalOneStrutWholeForceN=axial,
                tubeCombinedMPaUnderFullEccentricForce=stress,
                weakAxisEulerBucklingN=math.pi**2*69000*Iy/L**2,
                diagonalStockKg=stock_mass, outgoingPrincipalStockKg=original,
                eightEndAttachmentsMassAvailableForHeavyR103Kg=original-stock_mass+mass["cases"]["heavy-offset-control"]["R103MarginKg"],
                scope="Possible new load path only. All four vertical 2×1 tubes miter into terminal sidewalls on their modeled faces; any uncommanded overlap disqualifies that pose. One member conservatively resolves an entire field impulse in either Cartesian direction for this section screen. End loads, positive retention, assembly stiffness, fatigue, tire steering sweep and service remain separate. No weight credit before accepted physical connections.")


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("plugin",type=Path)
    result=run(p.parse_args().plugin);(HERE/"corner-tie-trial.json").write_text(json.dumps(result,indent=2)+"\n")
    print(result["diagonalStockKg"],result["tubeCombinedMPaUnderFullEccentricForce"],result["eightEndAttachmentsMassAvailableForHeavyR103Kg"])
