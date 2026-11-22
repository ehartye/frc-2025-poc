"""Compare two spaced pairs of continuous tubes with the high-mass crossframe at the existing 552 interfaces."""
import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np

from perimeter_footprint import structure, cases_and_forces
from terminal_net_sections import section
from crossmember_envelope import box
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def max_case(dst, label, values, info, cases):
    i = int(np.argmax(values))
    if label not in dst or values[i] > dst[label]["value"]:
        dst[label] = dict(value=float(values[i]), case=cases[i], **info)


def run(package):
    sys.path.insert(0, str(package / "scripts"))
    from fusion_authority import register_path, verify, digest
    from showcase_contacts import load_solids, box_gap
    from mechanical_motion import bounds, common, distance
    reg = register_path(ROOT)
    issue = reg.parent / "issue-005/lineage.json"
    auth = verify(ROOT, reg, issue)
    solids, sheets = load_solids(issue.with_name("assembly.step"))
    if sheets or len(solids) != 552:
        raise ValueError("Wrong installed chassis")
    mass = json.loads((HERE / "inspection-mass-552.json").read_bytes())
    if auth != mass["authority"]:
        raise ValueError("Mass bound belongs to another assembly")
    m = mass["cases"]["heavy-offset-control"]["fieldKg"]
    peak = m * 1.8**2 / .018
    p = 85.0
    width, height, wall = 50.8, 25.4, 3.0
    area = width*height-(width-2*wall)*(height-2*wall)
    inertia = (width*height**3-(width-2*wall)*(height-2*wall)**3)/12
    lengths = [638.4,628.4]
    middle = (114.3+150)/2
    z = [middle+(height+2)/2,middle-(height+2)/2]
    placements = {}
    clearances = {}
    original_bounds = {k: bounds(s) for k,s in solids.items()}
    for axis in (0,1):
        other = 1-axis
        for offset in (-p,p):
            key = f"{'XY'[axis]} at {offset:g}"
            lo,hi = [0.,0.,z[axis]-height/2], [0.,0.,z[axis]+height/2]
            lo[axis],hi[axis]=-lengths[axis]/2,lengths[axis]/2
            lo[other],hi[other]=offset-width/2,offset+width/2
            inner_lo,inner_hi=lo.copy(),hi.copy()
            for n in (other,2):
                inner_lo[n]+=3.175;inner_hi[n]-=3.175
            inner_lo[axis]-=1;inner_hi[axis]+=1
            tube=BRepAlgoAPI_Cut(box(lo,hi),box(inner_lo,inner_hi)).Shape()
            tb=bounds(tube)
            near={}
            for k,s in solids.items():
                if box_gap(tb, original_bounds[k])>.001:
                    continue
                near[k]=dict(gapMm=distance(tube,s),commonMm3=common(tube,s))
            placements[key]=dict(boundsMm=tb, adjacent=near)
            for prev,shape in clearances.items():
                gap=distance(tube,shape)
                if gap<3:
                    placements[key].setdefault("crossings",{})[prev]=gap
            clearances[key]=tube
    features=tuple(x+d for x in (-9,9) for d in (-7.15,-6.55,0,6.55,7.15))
    nodes,faces,supports,K,_,_,_,_=structure(10,section,features)
    loads,cases=cases_and_forces(nodes,faces)
    sdof=np.array([3*n+i for n in supports for i in (0,1)])
    evidence={}
    for stiffness in (3000,30000):
        matrix=K.copy()
        matrix[sdof,sdof]+=stiffness
        u=np.linalg.solve(matrix,np.column_stack([peak*loads,np.eye(len(K))[:,sdof]]))
        u0=u[:,:loads.shape[1]]
        influence=u[:,loads.shape[1]:]
        for absent in (None,*supports):
            state="intact" if absent is None else "one perimeter mount absent"
            if absent is None:
                v=u0
            else:
                rm=[3*absent,3*absent+1]
                g=influence[:,np.isin(sdof,rm)]
                v=u0+g@np.linalg.solve(np.eye(2)/stiffness-g[rm],u0[rm])
            for face,(ids,_,direction) in enumerate(faces):
                stations=[n for n,(f,q) in supports.items() if f==face and n!=absent]
                if not stations:
                    continue
                axis=int(np.argmax(abs(direction))); tangent=1-axis
                force=np.array([-stiffness*v[3*n:3*n+2] for n in stations])
                normal=force[:,axis,:]
                lateral=force[:,tangent,:]
                along=nodes[stations,tangent]
                # End beam sees all three external normal reactions on this side and the moment
                # of their tangential component about its radial attachment datum.
                radial_offset=nodes[stations,axis] - np.sign(nodes[stations,axis])*lengths[axis]/2
                moment=(along[:,None]*normal-radial_offset[:,None]*lateral).sum(axis=0)
                total=normal.sum(axis=0)
                for side in (-1,1):
                    axial=total/2+side*moment/(2*p)
                    for h in (114.3,150.):
                        applied=abs(axial*(h-z[axis]))
                        stress=abs(axial)/area+applied*(height/2)/inertia
                        bow=applied*lengths[axis]**2/(8*69000*inertia)
                        tag=dict(face=face,heightMm=h,springNPerMm=stiffness,missed=absent,memberOffsetMm=side*p)
                        target=evidence.setdefault(state,{})
                        max_case(target,"sectionStressAtServiceMPa",stress,tag,cases)
                        max_case(target,"constantEndMomentBowMm",bow,tag,cases)
                        max_case(target,"tubeAxialN",abs(axial),tag,cases)
                        max_case(target,"terminalTangentialReactionN",abs(lateral.sum(axis=0)),tag,cases)
    current=json.loads((HERE/"low-line-frame-clock.json").read_bytes())["remodeledStructuralItems"]
    original=sum(r["kg"] for r in current[:4])
    stock=(2*sum(lengths)/1000)*.809*.45359237/.3048
    return dict(authority=auth, measuredMassSha256=digest(HERE/"inspection-mass-552.json"),
                heavyFieldMassKg=m, upperDesignPeakN=peak,planningTubeWidthHeightWallMm=[width,height,wall],
                profileAreaMm2=area,verticalBendingInertiaMm4=inertia, sourceStockKgPerFt=.809*.45359237,
                principalStockKg=stock, replacedCentralSheetWebAndBeamKg=original,
                grossBeforeAnyNewEndAttachmentsSavedKg=original-stock, installedGeometry=placements, cases=evidence,
                scope="Two pairs of straight 6061-T6 rectangular tubes, spanning the four actual cartridges in two layers. Each member captures the most unfavorable simultaneous side reaction and eccentric height for the stated three-mount ring screen. The pure moment bow is a bounding challenge for end fixity, not a solved assembled contact. Plane-frame shear routing, end moment attachments, modified group stiffness, stock radius, service and welds remain to be established. No additional inspection weight credit for an unqualified connection.")


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("plugin",type=Path)
    result=run(p.parse_args().plugin)
    (HERE/"opposed-tube-trial.json").write_text(json.dumps(result,indent=2)+"\n")
    print(result["grossBeforeAnyNewEndAttachmentsSavedKg"], result["cases"]["intact"])
