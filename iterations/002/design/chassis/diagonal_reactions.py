"""Give each cartridge three planar freedoms and resolve its four lower anchors and two axial diagonal ties."""
import json
import math
from pathlib import Path

import numpy as np
from perimeter_footprint import structure, cases_and_forces
from terminal_net_sections import section

HERE=Path(__file__).resolve().parent


def lower_rail_challenge(reactions, rail, face):
    length_axis=rail["lengthAxis"]; radial=1-length_axis
    interior=min(abs(x[length_axis]) for x in rail["boltCentersMm"])
    x=np.array([-interior,-65,-35,35,65,interior])
    # No module fastener lies in this span. Arbitrary outer support end moments and reactions
    # can only add an affine function to M(x). Each three-section chord cancels that function.
    # Half its residual is a lower bound on the largest absolute bending moment in the free span.
    moment=np.maximum(0,x[:,None]-x[None,1:-1])@reactions[4*face:4*face+4,radial,:]
    admissible=np.zeros(moment.shape[1]);pinned=moment-(x-x[0])[:,None]/(x[-1]-x[0])*moment[-1]
    for i in range(len(x)-2):
        for j in range(i+1,len(x)-1):
            for k in range(j+1,len(x)):
                residual=moment[j]-((x[k]-x[j])*moment[i]+(x[j]-x[i])*moment[k])/(x[k]-x[i])
                admissible=np.maximum(admissible,abs(residual)/2)
    # Use *gross nominal* 1/8 wall, a favorable bending-only test before drill holes or torsion.
    inertia=(50.8*25.4**3-44.45*19.05**3)/12
    return admissible*12.7/inertia, np.max(abs(pinned),axis=0)*12.7/inertia


def run():
    trial=json.loads((HERE/"corner-tie-trial.json").read_bytes())
    feature=tuple(x+d for x in (-9,9) for d in (-7.15,-6.55,0,6.55,7.15))
    nodes,faces,supports,ring,_,_,_,_=structure(14,section,feature)
    incoming,cases=cases_and_forces(nodes,faces)
    sidecentres=np.array([[333.9375,0],[0,328.9375],[-333.9375,0],[0,-328.9375]])
    labels=["Front transverse socket rail","Port longitudinal socket rail",
            "Rear transverse socket rail","Starboard longitudinal socket rail"]
    stock=json.loads((HERE/"socket-holes.json").read_bytes())["rails"]
    n=len(ring);size=n+12
    load=np.zeros((size,len(cases)));load[:n,:]=incoming*trial["upperCaseN"]
    def side_jac(i,xy):
        x,y=xy-sidecentres[i];b=np.zeros((2,size))
        b[:,n+3*i:n+3*i+3]=[[1,0,-y],[0,1,x]]
        return b
    ring_jac=[]
    for node,(f,xy) in supports.items():
        side=int(np.argmin(np.linalg.norm(sidecentres-xy,axis=1)))
        b=-side_jac(side,xy)
        b[:,3*node:3*node+2]=np.eye(2)
        ring_jac.append(b)
    foundation=[]
    for i,center in enumerate(sidecentres):
        tangent=1 if i%2==0 else 0
        for off in (-65,-35,35,65):
            end=center.copy();end[tangent]+=off
            foundation.append(side_jac(i,end))
    braces=[];props=[]
    for sx,sy in ((1,1),(-1,1),(-1,-1),(1,-1)):
        a=np.array([sx*319.2,sy*65]);b=np.array([sx*65,sy*314.2])
        u=b-a;L=np.linalg.norm(u);u/=L
        diff=side_jac(1 if sy==1 else 3,b)-side_jac(0 if sx==1 else 2,a)
        braces.append(u@diff)
        props.append(69000*(25.4*50.8-19.4*44.8)/L)
    braces=np.array(braces);props=np.array(props)
    grouped=[(name,np.array(jac)) for name,jac in (("ring",ring_jac),("pilot",foundation))]
    selected={}
    for braced in (False,True):
        for ka in (3000.,30000.):
            for kb in (3000.,30000.):
                M=np.zeros((size,size));M[:n,:n]=ring
                for stiffness,(_,jac) in zip((ka,kb),grouped):
                    for j in jac:M+=stiffness*j.T@j
                if braced:M+=braces.T@(props[:,None]*braces)
                every_joint=np.concatenate((ring_jac,foundation),axis=0).reshape(-1,size)
                response=np.linalg.solve(M,np.column_stack((load,every_joint.T)))
                initial=response[:,:load.shape[1]]
                columns=iter(response[:,i:i+2] for i in range(load.shape[1],response.shape[1],2))
                removals=[("intact",None,None,None)]
                for stiffness,(group,jacs) in zip((ka,kb),grouped):
                    removals.extend((group+" one lost",f"{group}{i}",j,stiffness) for i,j in enumerate(jacs))
                for group,obsolete,j,stiffness in removals:
                    if j is None:
                        disp=initial
                    else:
                        g=next(columns)
                        disp=initial+g@np.linalg.solve(np.eye(2)/stiffness-j@g,j@initial)
                    # Recover side reactions on the installed lower and verify the full two-component force residual.
                    lower=np.stack([-kb*(q@disp) for q in foundation])
                    if obsolete and obsolete.startswith("pilot"):lower[int(obsolete[5:])]=0
                    residual=np.max(abs(lower.sum(axis=0)+load[:n].reshape((-1,3,len(cases))).sum(axis=0)[:2]))
                    if residual>.005:raise ValueError(f"Unbalanced chassis supports {residual}")
                    tagged=("braced" if braced else "without diagonals")+"/"+group
                    slot=selected.setdefault(tagged,{})
                    braceforce=(props[:,None]*(braces@disp)) if braced else np.zeros((4,len(cases)))
                    radial=np.stack([abs(lower[i,(i//4)%2]) for i in range(16)])
                    tangential=np.stack([abs(lower[i,1-(i//4)%2]) for i in range(16)])
                    # A 22 x 12.65 mm rounded key running along the rail. The second wall must engage
                    # the full specified minimum depth; both trials use the same conservative contact stress.
                    key_usage=radial/(22*2.8*120)+tangential/(12.65*2.8*120)
                    values={"pilotShearN":np.linalg.norm(lower,axis=1),"braceAxialN":abs(braceforce),
                            "pilotRadialN":radial,"pilotTangentialN":tangential,
                            "oval22x12Minimum28BearingFraction":key_usage}
                    checks=[lower_rail_challenge(lower,stock[label],idx) for idx,label in enumerate(labels)]
                    values["lowerGrossBestPossibleMomentStressMPa"]=np.stack([r[0] for r in checks])
                    values["lowerGrossSimplySupportedStressMPa"]=np.stack([r[1] for r in checks])
                    for label,mat in values.items():
                        idx,col=np.unravel_index(np.argmax(mat),mat.shape)
                        if label not in slot or mat[idx,col]>slot[label]["value"]:
                            slot[label]=dict(value=float(mat[idx,col]),row=int(idx),applied=cases[col],
                                            lost=obsolete,ringJointNPerMm=ka,lowerPilotNPerMm=kb,
                                            simultaneousBraceN=braceforce[:,col].tolist(),
                                            simultaneousLowerXYN=lower[:,:,col].tolist(),maxForceResidualN=float(residual))
    return dict(source=trial["authority"], heavyPeakN=trial["upperCaseN"],states=selected,
                scope="Planar ring, simultaneously reacting rigid terminal cartridges, 16 in-plane lower mounting springs and four axial steel-free aluminum members. True mounting axes set the moment arms. Compliance ranges are sensitivity screens. Effective lower-rail support, out-of-plane contact and twist, material behavior, impact duration, fatigue and end connections still require separate qualification.")


if __name__=="__main__":
    result=run();(HERE/"diagonal-reactions.json").write_text(json.dumps(result,indent=2)+"\n")
    for tag,row in result["states"].items():print(tag,{k:round(v["value"],2) for k,v in row.items()})
