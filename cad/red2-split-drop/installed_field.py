"""Bound the entire cold-reopened upper against the finite signed field obstacles."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/split-drop-native'
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main(rev):
    native=read(OUT/(rev+'-load-01.json'))
    archive=OUT/('red2-split-drop-fixture-'+rev+'.f3d')
    assert native['sha256']==sha(archive) and native['restored']
    reg=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert reg['sha256']==sha(ROOT/reg['file'])==native['authority']['archiveSha256']
    target=read(ROOT/'experiments/six-wheel/mission/target.json')
    cases=read(ROOT/'experiments/swerve-drive/low-placement/clearance.json')['cases']
    records=[]
    targetX=np.array([15.6,245.9]);targetY=np.array([-469.2,469.2])
    for st in native['native']['states']:
        names=list(st['bodies'])
        boxes=np.array([st['bodies'][k] for k in names])
        low=boxes[:,:3]-1.;hi=boxes[:,3:]+1.
        local=np.stack([np.where(corner,hi,low) for corner in itertools.product([0,1],repeat=3)],axis=1)
        worst={}
        for sign,case in cases.items():
            for dx,dy,yaw in itertools.product(*zip(*case['enclosure'])):
                for p,dz,sector in itertools.product((-.04,.04),(-5.,5.),(0.,360.)):
                    co,si=math.cos(yaw),math.sin(yaw)
                    cp,sp=math.cos(p),math.sin(p)
                    rot=np.array([[co*cp,-si,-co*sp],[si*cp,co,-si*sp],[sp,0,cp]])
                    pt=local@rot.T+np.array([dx-490,dy+sector,dz])
                    a,b=pt.min(1),pt.max(1)
                    floor=np.maximum.reduce([a[:,0]-targetX[1],targetX[0]-b[:,0],a[:,1]-targetY[1],targetY[0]-b[:,1],
                         (pt[:,:,2]-target['floor_slope']*pt[:,:,0]-1000*target['floor_intercept_m']).min(1)/math.hypot(1,target['floor_slope'])])
                    def box_separation(ob):
                        return np.maximum((a-ob[3:]).max(1),(ob[:3]-b).max(1))
                    hazards={'floor':floor,'lip':box_separation(np.array([0,-469.2,3.175,12.7,469.2,457.2])),
                             **{f'riser-{n}':box_separation(1000*np.array(bnd)) for n,bnd in enumerate(target['riser_bounds_m'])}}
                    for hazard,vals in hazards.items():
                        for i,d in enumerate(vals):
                            key=(names[i],hazard)
                            if key not in worst or d<worst[key]['separationMm']:
                                worst[key]=dict(body=names[i],hazard=hazard,separationMm=float(d),dock=[sign,dx,dy,yaw,p,dz,sector])
        records.append(dict(pose=st['pose'],bodies=len(names),perHazardMinimum={h:min((x for x in worst.values() if x['hazard']==h),key=lambda x:x['separationMm']) for h in hazards},
                            ambiguous=[v for v in worst.values() if v['separationMm']<=0]))
    (OUT/(rev+'-all-geometry-field.json')).write_text(json.dumps(dict(authority=reg,native=sha(archive),states=records,
        basis='Finite extracted two-panel floor, full lip and two riser AABBs from January field. Complete cold native robot-frame bounds enlarged 1 mm, signed dock pitch/yaw/translation and both sectors. Negative values require a narrow-phase test. The complete robot, motion between sampled poses and physical wall impact remain open.'),indent=2)+'\n')
    print([(s['pose'],len(s['ambiguous'])) for s in records])
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('revision');main(p.parse_args().revision)
