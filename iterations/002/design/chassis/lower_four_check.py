"""Check every current placed SDS leaf and every rail seat directly from the canonical 488-body exchange."""
import argparse
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer

HERE=Path(__file__).parent


def run(project, candidate, plugin, manufacturer, option):
    sys.path.insert(0,str(plugin/'scripts'))
    from fusion_authority import register_path, verify, load, resolve, digest
    from mechanical_motion import bounds, distance, common, bearing_area
    from component_shape import volume
    from showcase_contacts import load_solids
    reg=register_path(candidate)
    lineage=reg.parent/'issue-003/lineage.json'
    issue=verify(candidate,reg,lineage)
    srcplan=load(HERE/'front-corner.json')
    legacy=load(HERE/'independent-123.json')
    if not(digest(manufacturer)==srcplan['sourceSha256']==legacy['supplierSha256']
           and digest(option)==srcplan['optionSha256']):
        raise ValueError('Uncommissioned manufacturer or selectable configuration')
    rawlabels={};labels={}
    raw,sheets=load_solids(manufacturer,rawlabels)
    placed,sheets2=load_solids(resolve(candidate,load(lineage)['step']),labels)
    selector=load(option); chosen=selector['commonSolids']+selector['selectableFamilies']['L2']['selected']
    if sheets or sheets2 or len(raw)!=125 or len(placed)!=488 or len(chosen)!=121:raise ValueError('Incomplete source or installed exchange')
    wanted={}; used=set(); bb={k:bounds(v) for k,v in placed.items()}
    def match(wanted_bounds, predicate):
        scores=sorted((float(np.max(abs(wanted_bounds-box))),k) for k,box in bb.items() if predicate(k))
        if not scores or scores[0][0]>.15 or len(scores)>1 and scores[1][0]-scores[0][0]<.1:raise ValueError('Ambiguous source registration '+str(scores[:3]))
        err,k=scores[0]
        if k in used:raise ValueError('Duplicated installed identification '+k)
        used.add(k);return k,err
    rotations={'FL':(0.,(0,0)),'FR':(-math.pi/2,(5,5)),'RR':(math.pi,(0,0)),'RL':(math.pi/2,(-5,-5))}
    for station,(a,shift) in rotations.items():
        rot=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
        offset=np.array([*shift,0.]);cx=np.array(srcplan['worldMatrixMm'])
        at_station=rot@cx[:3,:3];trans=rot@cx[:3,3]+offset
        data={}
        for key in chosen:
            b=bounds(raw[key])
            corners=np.array([at_station@np.array(v)+trans for v in itertools.product(*zip(b[:3],b[3:]))])
            n,error=match(np.r_[corners.min(axis=0),corners.max(axis=0)],lambda k:labels[k]['definition']==rawlabels[key]['definition'])
            explorer=TopExp_Explorer(placed[n],TopAbs_SHELL);shells=[]
            while explorer.More():
                shells.append(explorer.Current());explorer.Next()
            if not shells:raise ValueError('Missing placed boundary '+n)
            actual=[]
            for point in legacy['sourceChecks'][key]['sourceWorldVerticesMm']:
                v=rot@point+offset
                probe=BRepBuilderAPI_MakeVertex(gp_Pnt(*v)).Vertex()
                actual.append(min(distance(probe,s) for s in shells))
            data[key]=dict(placed=n,boundsErrorMm=error,faceWitnessGapsMm=actual,rawAndPlacedVolumeDifferenceMm3=volume(placed[n])-volume(raw[key]))
        wanted[station]=data
    tubes=load(HERE/'socket-holes.json')['rails']; seats=[]
    for tube in tubes.values():
        name,err=match(np.r_[tube['minimumMm'],tube['maximumMm']],lambda k:'socket rail' in k)
        for station in sorted(set(s['station'] for s in tube['sources'])):
            for position,index,sign in (('upper',17,1),('lower',16,-1)):
                other=wanted[station][f'MK4i Master Part V2_NEO Config [1.43.{index}]']['placed']
                shape=placed[name];plate=placed[other]
                seats.append(dict(rail=tube['name'],installed=name,station=station,plate=other,face=position,
                    gapMm=distance(shape,plate),overlapMm3=common(shape,plate),areaMm2=bearing_area(shape,plate,[0,0,sign])))
    gaps=[v for d in wanted.values() for r in d.values() for v in r['faceWitnessGapsMm']]
    if len(used)!=488 or len(seats)!=16 or len(gaps)!=1452 or max(gaps)>.001 or any(s['areaMm2']<1000 or s['gapMm']>.001 or s['overlapMm3']>.01 for s in seats):
        raise ValueError('Nonconforming installed landmark/seat cohort: '+str((len(used),len(seats),max(gaps),seats)))
    return dict(authority=issue,supplierSha256=digest(manufacturer),optionSha256=digest(option),sourceLandmarkReceiptSha256=digest(HERE/'independent-123.json'),
        measured=wanted,maximumSurfaceGapMm=max(gaps),seats=seats,scope='Three material-surface landmarks per actual supplier article, four real named stations, all 488 uniquely mapped and 16 fixed plate/rail seats. Geometric form discrepancies in certain imported supplier leaves remain present; motor screw bottoming, bolt clamp, complete interference, wheel motion, loads, wiring and upper interfaces need distinct evidence.')

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('project','candidate','plugin','manufacturer','option'):p.add_argument(name,type=Path)
    a=p.parse_args();result=run(a.project.resolve(),a.candidate.resolve(),a.plugin.resolve(),a.manufacturer.resolve(),a.option.resolve())
    (HERE/'lower-488-independent.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result['maximumSurfaceGapMm'],[(r['station'],r['face'],round(r['areaMm2'],2)) for r in result['seats']])
