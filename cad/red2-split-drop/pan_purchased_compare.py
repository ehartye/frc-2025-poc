"""Compare every individually placed pan purchase to its registered source and rigid pose."""
import json
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from output_mount import ROOT, digest, place, read
from pan_mechanics import rotation


def matrix(raw):
    result=np.eye(4)
    result[:3]=raw
    return result


def fingerprint(shape,bounds):
    props=GProp_GProps()
    BRepGProp.VolumeProperties_s(shape,props)
    # Coordinates in mm. A distinct volume also identifies concentric constituents.
    return np.array([*bounds(shape),*props.CentreOfMass().Coord(), abs(props.Mass())**(1/3)])


def assign(cost):
    # Clocks, planets and repeated screws must retain one source per occurrence.
    # Minimize the aggregate distance across this vendor subassembly only.
    @lru_cache(None)
    def best(i,used):
        if i==len(cost):return 0,()
        options=[]
        for j in range(len(cost)):
            if used & (1<<j):continue
            remainder,indices=best(i+1,used | (1<<j))
            options.append((cost[i][j]+remainder,(j,)+indices))
        return min(options)
    return best(0,0)[1]


def run(plugin,revision):
    sys.path.insert(0,str(plugin/'scripts'))
    import component_shape
    from mechanical_motion import bounds
    from showcase_contacts import load_solids
    directory=ROOT/'artifacts/red2-integrated'
    station=directory/'pan-native'
    installation=read(station/('pan-installation-01.json' if revision=='r1' else 'pan-r2-foundation-install-02.json'))
    archive=station/f'integrated-340mm-pan-{revision}.f3d'
    step=archive.with_suffix('.step')
    proof={'archiveSha256':digest(archive),'stepSha256':digest(step),'pluginSha256':digest(plugin/'scripts/component_shape.py')}
    if installation.get('status')!='pass' or proof['stepSha256']!=installation.get('derivativeSha256',installation.get('stepSha256')) or proof['archiveSha256']!=installation.get(archive.stem,installation.get('archiveSha256')):
        raise ValueError('Requires a registered reopened-archive STEP')
    authority=read(directory/'pan-shop-plan.json')['authority']
    if digest(ROOT/authority['file'])!=authority['sha256']:raise ValueError('Changed canonical source')
    manifest=read(directory/'pan-drive-registration.json')
    components=manifest['components']
    if revision=='r2':
        components+=read(directory/'pan-assembly-fasteners.json')['preparedForNextNativeIssue']
    expected=70 if revision=='r1' else 78
    metadata={}
    placed,extras=load_solids(step,metadata)
    if extras:raise ValueError('Unaccounted non-solid geometry')
    found={k for k,v in metadata.items() if any(p.endswith(' pan purchase') for p in v['occurrencePath'])}
    if len(found)!=expected:raise ValueError('Incorrect installed purchase count '+str(len(found)))
    registration=matrix([[1,0,0,205],[0,0,1,216],[0,-1,0,580]])
    spin=matrix(rotation(70))
    bearing=read(directory/'pan-bearing-placement.json')
    jobs=[]
    roster={}
    for row in components:
        part=row['id']
        moving=(part=='REV-41-1620' or part.startswith('M3x8 adapter') or row.get('moving',False))
        installed=registration @ matrix(row['frameMm'])
        if revision=='r2' and part in ('REV-21-1915 drive','REV-21-1915 idler'):
            installed=matrix(next(r['installedMatrixMm'] for r in bearing['parts'][part.split()[-1]] if r['case']=='seated-clocked' and r['panAngleDeg']==0))
            moving=True
        if moving:installed=spin @ installed
        source=ROOT/row['source']
        if digest(source)!=row['sourceSha256']:raise ValueError('Changed supplier source '+part)
        original=load_solids(source)[0]
        keys=list(original)
        candidates=[k for k in found if part+' pan purchase' in metadata[k]['occurrencePath']]
        if not candidates or len(keys)!=len(candidates):raise ValueError('Incomplete source mapping '+part)
        source_points=[fingerprint(place(original[k],installed[:3]),bounds) for k in keys]
        target_points=[fingerprint(placed[k],bounds) for k in candidates]
        cost=[[np.linalg.norm(a-b) for b in target_points] for a in source_points]
        selected=assign(cost)
        for i,j in enumerate(selected):
            k=candidates[j]
            jobs.append((k,source,keys[i],installed.ravel().tolist()))
            if k in roster:raise ValueError('Duplicate installed body mapping')
            roster[k]=dict(part=part,path=metadata[k]['occurrencePath'],supplierSolid=keys[i],sourceSha256=row['sourceSha256'],
                transform=installed.ravel().tolist(),selectionDistance=float(cost[i][j]))
    if set(roster)!=found:raise ValueError('Missing registered body')
    rec=station/f'first-party-purchases-{revision}.json'
    proof.update(authority=authority,expectedBodies=expected,roster=roster,status='awaiting directed Boolean comparisons')
    rec.write_text(json.dumps(proof,indent=2))
    # Keep an inconclusive boolean tied to its original constituent. Cache
    # the unmodified released STEP loader so one fault cannot discard all
    # other directed differences or require 78 parses of the same assembly.
    loader=component_shape.load_solids
    component_shape.load_solids=lru_cache(None)(loader)
    try:
        for index,job in enumerate(jobs,1):
            name=job[0]
            try:
                roster[name].update(component_shape.compare_components(step,[job])[name])
            except ValueError as exc:
                roster[name]['unresolved']=str(exc)
            proof['completed']=index
            if index%5==0:
                rec.write_text(json.dumps(proof,indent=2))
    finally:
        component_shape.load_solids=loader
    if digest(step)!=proof['stepSha256']:raise ValueError('Input STEP changed during comparison')
    proof['unresolved']=[name for name,row in roster.items() if 'unresolved' in row]
    proof['failed']=[name for name,row in roster.items() if 'unresolved' not in row and max(row['absentMm3'],row['extraMm3'])>.025]
    proof['status']='failed' if proof['failed'] else 'open' if proof['unresolved'] else 'pass'
    rec.write_text(json.dumps(proof,indent=2))
    print(revision,len(found),proof['status'],proof['failed'],proof['unresolved'])


if __name__=='__main__':
    if len(sys.argv)!=3 or sys.argv[2] not in ('r1','r2'):raise ValueError('Usage: PLUGIN_ROOT r1|r2')
    run(Path(sys.argv[1]),sys.argv[2])
