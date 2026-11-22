"""Resolve ring reactions with the proposed access and shoulder holes at their actual positions."""
import json
import math
from pathlib import Path
import numpy as np
from perimeter_footprint import structure, cases_and_forces

HERE = Path(__file__).parent
CENTERS = tuple(s + dx for s in (-100, 0, 100) for dx in (-9, 9))


def section(point, direction):
    width, height, wall = 25.4, 101.6, 3.
    gross = width * height - (width-2*wall) * (height-2*wall)
    inertia = (height*width**3 - (height-2*wall)*(width-2*wall)**3)/12
    area, moment, first = gross, inertia, 0.
    if min(abs(direction)) < .00001:
        along = point[np.argmax(abs(direction))]
        dx = min(abs(along-center) for center in CENTERS)
        for sign, radius in ((1.,7.15),(-1.,6.55)):
            # Upper/lower two bores at each X pierce the corresponding radial wall.
            height_removed = 4*math.sqrt(max(0,radius**2-dx**2))
            cut = wall*height_removed
            radial = sign*(width-wall)/2
            area -= cut
            first -= cut*radial
            moment -= cut*(wall**2/12 + radial**2)
    neutral = first/area
    return area, moment-area*neutral**2, width/2+abs(neutral)


def run(mesh, peak):
    # Include each bore center as an actual node and both diameter bounds on both walls.
    feature=tuple(x+d for x in (-9,9) for d in (-7.15,-6.55,0,6.55,7.15))
    nodes,faces,supports,K,ends,indices,_,_=structure(mesh,section,feature)
    loads,cases=cases_and_forces(nodes,faces)
    ids={k:np.array([i for i,row in enumerate(cases) if row['profile']==k]) for k in ('point','flat','triangular')}
    all_dofs=np.array([3*n+i for n in supports for i in (0,1)])
    evidence={}
    for k in (3000.,30000.):
        matrix=K.copy()
        matrix[all_dofs,all_dofs]+=k
        u=np.linalg.solve(matrix,np.column_stack([peak*loads,np.eye(len(K))[:,all_dofs]]))
        sample=u[:,:loads.shape[1]]
        influence=u[:,loads.shape[1]:]
        for absent in (None,*supports):
            state='all anchors' if absent is None else 'one anchor absent'
            solution=sample
            if absent is not None:
                rm=[3*absent,3*absent+1]
                response=influence[:,np.isin(all_dofs,rm)]
                solution=sample+response@np.linalg.solve(np.eye(2)/k-response[rm],sample[rm])
            active=np.array([3*n+i for n in supports if n!=absent for i in (0,1)])
            error=np.max(abs((k*solution[active]).reshape(-1,2,len(cases)).sum(axis=0)
                         -peak*loads.reshape(-1,3,len(cases)).sum(axis=0)[:2]))
            if error>.01:raise ValueError(f'Unbalanced frame {error}')
            for i,(forces,dofs) in enumerate(zip(ends,indices)):
                q=forces@solution[dofs]
                j=(i+1)%len(nodes)
                for end,axial,moment in ((i,q[0],q[2]),(j,q[3],q[5])):
                    A,I,c=section(nodes[end],nodes[j]-nodes[i])
                    stress=abs(axial)/A+abs(moment)*c/I
                    for profile,indices_ in ids.items():
                        at=indices_[np.argmax(stress[indices_])]
                        label=f'{state}/{profile}'
                        if stress[at]>evidence.get(label,{}).get('MPa',0):
                            evidence[label]=dict(MPa=float(stress[at]),forceN=float(axial[at]),
                                momentNm=float(moment[at]/1000),Amm2=A,Imm4=I,extremeFiberMm=c,
                                locationMm=nodes[end].tolist(),supportNPerMm=k,contact=cases[at],
                                missingMm=None if absent is None else nodes[absent].tolist())
    return dict(meshMm=mesh,loadCases=len(cases),meshNodes=len(nodes),results=evidence)


if __name__=='__main__':
    import argparse,sys
    parser=argparse.ArgumentParser();parser.add_argument('plugin',type=Path)
    pkg=parser.parse_args().plugin
    sys.path.insert(0,str(pkg/'scripts'))
    from fusion_authority import register_path,verify,digest
    root=HERE.parents[3];reg=register_path(root)
    auth=verify(root,reg,reg.parent/'issue-003/lineage.json')
    src=HERE/'lower-terminal-anchor-loads.json'
    cases=json.loads(src.read_bytes())['settings']
    max_load=cases['heavy-offset-control']['upperN']
    results=[run(step,max_load) for step in (12.,6.)]
    for r in results:print(r['meshMm'],[(k,round(v['MPa'],3)) for k,v in r['results'].items()],flush=True)
    report=dict(authority=auth,loadInputSha256=digest(src),maximumCaseN=max_load,otherLoadFactors={
        k:{kind:case[kind]/max_load for kind in ('routineN','upperN')} for k,case in cases.items()},
        workingLimitMPa=120,minimumWallMm=3,outerHoleDiameterMm=14.30,innerHoleDiameterMm=13.10,runs=results,
        scope='Actual four screw positions per station and upper/lower rows. Geometric net-section zero-radius extreme fibers; shared nominal miter rigidity and 3/30 kN/mm support values; closed total force on all solved cases. The local hole stress concentration, connection rotation, neutral-axis offset coupling, deflection and contact width of the real assembled backing/foam, and reinforcement from end boxes are not resolved. No section strength credit from adjoining materials. Rare upper pulse is conditional on assumed stopping stroke; a failed point case stays recorded separately from broad contact and approach.')
    (HERE/'terminal-net-sections.json').write_text(json.dumps(report,indent=2)+'\n')

