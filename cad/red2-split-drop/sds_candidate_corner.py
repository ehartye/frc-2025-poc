"""Rehearse a detachable SDS L2 front-left cartridge on the existing 2025 tire and bumper datums."""
from pathlib import Path
import json
import math
import sys
import numpy as np
from output_mount import ROOT,read,digest,place
from pan_counterbrace import box
from pan_thrust_layout import cylinder,subtract,translate


def run(plugin,source):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import common,bounds,distance
    from showcase_contacts import load_solids,box_gap
    p=ROOT/'artifacts/red2-integrated';folder=p/'module-option'
    fit=read(folder/'sds-2023-fl-fit.json');datums=read(folder/'sds-six-installed-datums.json');matrix=np.array(fit['cases']['90']['physicalPlacementMm'])
    assert digest(source)==fit['vendorStepSha256']==datums['sourceSha256']
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive'];assert digest(ROOT/authority['file'])==authority['sha256']
    step=p/'pan-thrust/moving-stack-issued/checked-301.step';assert digest(step)==fit['currentStep']
    labels={};root,_=load_solids(step,labels);assert len(root)==301
    # Replace only the FL-specific REV hardware and the terminal portions of
    # two existing rails. The original forward inclined tower ends at x163.
    body={n:s for n,s in root.items() if n not in set(fit['removedPriorHardware'])}
    for n,cut in [('front frame tube',box((150,163.5,-10),(450,560,500))),('left frame tube',box((168.5,160,-10),(510,450,500)))]:
        body[n]=subtract(body[n],cut)
    rim={}
    for n,limits in [('left SDS socket',(169.5,240.0,318.275,343.675)),('front SDS socket',(323.275,348.675,164.5,235.0))]:
        x0,x1,y0,y1=limits;z0,z1=43.8333629,94.6333629;t=3.175
        tube=box((x0,y0,z0),(x1,y1,z1))
        if n.startswith('left'):cavity=box((x0-.1,y0+t,z0+t),(x1+.1,y1-t,z1-t))
        else:cavity=box((x0+t,y0-.1,z0+t),(x1-t,y1+.1,z1-t))
        tube=subtract(tube,cavity)
        for q in datums['rows'].values():
            x,y=q['XYmm']
            if x0<x<x1 and y0<y<y1:tube=subtract(tube,cylinder((x,y,z0-.2),(0,0,1),2.65,z1-z0+.4))
        rim[n]=tube
    all_source={};original,_=load_solids(source,all_source)
    off={'Double Gear, 20DP, 28T, 50T','Double Gear, 20DP, 25T, 50T','Gear, 20DP, 16T, .375Hex, 217-5451','Gear, 19T, 20DP, .375Hex'}
    assert off<=set(original)
    installed={n:place(s,matrix) for n,s in original.items() if n not in off}
    assert len(installed)==121
    retained=body.copy();installed.update(rim)
    bboxes={k:bounds(s) for k,s in retained.items()};ib={k:bounds(s) for k,s in installed.items()}
    progress=dict(authority=authority,issuedStep=fit['currentStep'],source=fit['vendorStepSha256'],sixAxisBasis=digest(folder/'sds-six-installed-datums.json'),
        option={'gearRatio':(50/14)*(17/27)*(45/15),'excludedAlternateGearIds':sorted(off),'genuineIncluded':121},sourceMatrixMm=matrix.tolist(),parked={},
        externalTubeFits={},structuralScope='The two replacement socket cartridges include genuine six screw passage bores. Connection to retained parent rails and bumper backing supports are not yet incorporated; no structural pass.')
    sink=folder/'sds-l2-fl-cartridge.json'
    def save():sink.write_text(json.dumps(progress,indent=2)+'\n')
    for k,s in installed.items():
        for v,b in retained.items():
            if box_gap(ib[k],bboxes[v])>.01:continue
            a=common(s,b)
            if a>.01:progress['parked'][k+' vs '+v]=a
        if k in rim:
            for v,b in installed.items():
                if v==k or box_gap(ib[k],ib[v])>.01:continue
                a=common(s,b)
                if a>.01:progress['externalTubeFits'][k+' vs '+v]=a
    save();print('parked',progress['parked'],'tube',progress['externalTubeFits'],flush=True)
    # Vertical drop is bounded on each entire interval, not sampled just at
    # its endpoints. Exact midpoint BRep gap exceeds the maximum displacement
    # from that pose for a separating certificate; retain unresolved intervals.
    def swept_one(part,other,s0,s1,depth=0):
        b=ib[part].copy();b[2]-=s1;b[5]-=s0
        if box_gap(b,bboxes[other])>.10:return True
        mid=(s0+s1)/2;shifted=translate(installed[part],[0,0,-mid]);gap=distance(shifted,retained[other])
        if gap>(s1-s0)/2+.05:return True
        if depth==8:
            return dict(intervalMm=[s0,s1],midpointGapMm=gap,midpointCommonMm3=common(shifted,retained[other]))
        for a,b in [(s0,mid),(mid,s1)]:
            v=swept_one(part,other,a,b,depth+1)
            if v is not True:return v
        return True
    pot=[]
    for n,b in ib.items():
        wide=b.copy();wide[2]-=180
        for k,bb in bboxes.items():
            if box_gap(wide,bb)<.10:pot.append((n,k))
    progress['dropout']={'travelMm':180,'candidatePairs':len(pot),'tested':0,'open':{}}
    save()
    for a,b in pot:
        answer=swept_one(a,b,0,180)
        if answer is not True:progress['dropout']['open'][a+' vs '+b]=answer
        progress['dropout']['tested']+=1
    save();print('drop',progress['dropout'],flush=True)
    # The unmodified supplier turret occurrence owns the wheel, two wheel
    # bearings, axle, pulleys and forks. Additional shaft/encoder behaviors
    # need verification against the manufacturer's selected option.
    rotor=[n for n in installed if n in all_source and 'MK4 Turret Assembly_Billet Wheel' in all_source[n]['occurrencePath']]
    if not rotor:raise ValueError('Lost the complete steering subassembly identity')
    tc=progress['steering']={'movingSourceBodies':rotor,'outsideScope':list(rim),'degreesPerCell':5,'testedBodies':0,'potential':[],'open':{}}
    ext=retained|rim;eb=bboxes|{k:ib[k] for k in rim}
    # A conservative full-turn swept cylinder first rejects most installed
    # bodies. For remaining near neighbors, check angle midpoint with a
    # Lipschitz bound on rigid motion of every material point in the interval.
    for n in rotor:
        nbox=ib[n];xc,yc=282.,277.
        radius=max(math.hypot(a-xc,b-yc) for a in (nbox[0],nbox[3]) for b in (nbox[1],nbox[4]))
        polar=[xc-radius,yc-radius,nbox[2],xc+radius,yc+radius,nbox[5]]
        candidates=[k for k,b in eb.items() if box_gap(polar,b)<.15]
        for k in candidates:
            tc['potential'].append([n,k]);minimum=1e9
            for j in range(72):
                mid=math.radians(5*j+2.5);c,s=math.cos(mid),math.sin(mid)
                R=np.array([[c,-s,0],[s,c,0],[0,0,1]]);center=np.array([xc,yc,0.])
                obj=place(installed[n],np.c_[R,center-R@center]);bound=2*radius*math.sin(math.radians(1.25))
                if box_gap(bounds(obj),eb[k])>bound+.05:continue
                gap=distance(obj,ext[k]);minimum=min(minimum,gap)
                if gap<=bound+.05:
                    tc['open'][n+' vs '+k]=dict(angleDeg=5*j+2.5,actualGapMm=gap,possibleDisplacementMm=bound,commonMm3=common(obj,ext[k]));break
        tc['testedBodies']+=1;save()
    print('steer',len(rotor),len(tc['potential']),tc['open'],flush=True)


if __name__=='__main__':run(Path(sys.argv[1]),Path(sys.argv[2]))
