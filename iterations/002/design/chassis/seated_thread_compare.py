"""Check a commissioned screw clock at fixed head seat and retain raw and solid-validity evidence."""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.gp import gp_Ax1,gp_Dir,gp_Pnt,gp_Trsf
from OCP.TopLoc import TopLoc_Location
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

HERE=Path(__file__).parent;ROOT=HERE.parents[3]


def run(pkg,issue):
    sys.path.insert(0,str(pkg/'scripts'))
    from fusion_authority import register_path,verify,digest
    from component_shape import volume
    from showcase_contacts import load_solids
    from mechanical_motion import bounds,bearing_area
    reg=register_path(ROOT);auth=verify(ROOT,reg,reg.parent/'issue-003/lineage.json')
    study=json.loads((HERE/'port-504-seated-phase-inputs.json').read_bytes())
    cold=json.loads((issue/'report.json').read_bytes())
    step=issue/'port-504-negative.step'
    if cold['authority']!=auth or digest(step)!=study['source']:raise ValueError('Scope mismatch')
    solids,rest=load_solids(step);bb={k:np.array(bounds(v)) for k,v in solids.items()}
    if len(solids)!=504 or rest:raise ValueError('Incomplete derivative')
    female=solids[study['female']]
    def identity(row):
        pool=sorted((np.max(abs(bounds_-row['boundsMm'])),k) for k,bounds_ in bb.items())
        if pool[0][0]>.1 or pool[1][0]<.5:raise ValueError('Ambiguous physical identity')
        return pool[0][1]
    result=dict(authority=auth,archiveSha256=cold['archiveSha256'],stepSha256=digest(step),
                phaseInputSha256=digest(HERE/'port-504-seated-phase-inputs.json'),cases={})
    for label,station in study['cases'].items():
        sku,x=label.split('/');x=float(x);orig=station['headXYZmm'];axis=station['intoScrewAxis']
        row=next(r for r in cold['anchor'] if sku in r['path'] and abs((r['boundsMm'][3]+r['boundsMm'][0])/2-x)<.01)
        key=identity(row);shape=solids[key]
        if sku=='91251A542':
            seatrow=next(r for r in cold['anchor'] if 'Head load washer' in r['path'] and abs((r['boundsMm'][0]+r['boundsMm'][3])/2-x)<.01)
        else:
            seatrow=next(r for r in cold['worldBodies'] if r['path']=='Port longitudinal socket rail:1')
        seat=identity(seatrow);angles=[station['farFlankAngleDeg']]
        if x>0:angles.append(0.)
        frames=[]
        for angle in angles:
            m=gp_Trsf();m.SetRotation(gp_Ax1(gp_Pnt(*orig),gp_Dir(*axis)),math.radians(angle))
            rotated=shape.Moved(TopLoc_Location(m))
            op=BRepAlgoAPI_Common(rotated,female);body=op.Shape()
            if not op.IsDone() or body.IsNull():raise ValueError('Unresolved real screw common')
            census=[];it=TopExp_Explorer(body,TopAbs_SOLID)
            while it.More():
                s=it.Current();census.append(dict(volumeMm3=volume(s),valid=BRepCheck_Analyzer(s).IsValid(),boundMm=bounds(s)));it.Next()
            datum=bearing_area(rotated,solids[seat],axis)
            frames.append(dict(phaseDeg=angle,rawCommonMm3=volume(body),valid=BRepCheck_Analyzer(body).IsValid(),
                               solids=census,underheadAreaMm2=datum))
            print(label,angle,frames[-1]['rawCommonMm3'],frames[-1]['valid'],datum,flush=True)
            result['cases'][label]=dict(nativeOccurrence=row['path'],placedSolid=key,bearingHost=seat,measured=frames)
            (HERE/'port-504-seated-comparison.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('pkg',type=Path);p.add_argument('issue',type=Path)
    a=p.parse_args();run(a.pkg,a.issue)
