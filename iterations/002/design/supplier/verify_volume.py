"""Resolve nonadaptive volume differences by independent adaptive integration."""
import json
import sys
from pathlib import Path
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from select_l2 import digest


def integrate(shape, relative):
    props = GProp_GProps()
    error = BRepGProp.VolumeProperties_s(shape, props, relative, True, False)
    return dict(volumeMm3=props.Mass(), relativeErrorEstimate=error)


def run(plugin, source, issue):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    readback=json.loads((issue/'readback.json').read_bytes())
    selected=issue/'sds-mk4i-neo-l2.step'
    if [digest(source),digest(selected)] != [readback['selection'][k] for k in ('sourceSha256','derivedStepSha256')]:
        raise ValueError('Changed supplier issue')
    org,_=load_solids(source)
    new,_=load_solids(selected)
    results={}
    for k,raw in readback['occupiedVolumeExceptions'].items():
        n=readback['roster'][k]['configuredOccurrence']
        measures=[]
        for e in (1e-6,1e-9):
            old=integrate(org[k],e); now=integrate(new[n],e)
            measures.append(dict(relative=e,source=old,configured=now,differenceMm3=abs(old['volumeMm3']-now['volumeMm3'])))
        results[k]=dict(originalNonadaptiveDifferenceMm3=raw,adaptive=measures)
    return dict(inputSha256=digest(issue/'readback.json'),results=results)


if __name__=='__main__':
    plugin,source,issue=map(Path,sys.argv[1:])
    result=run(plugin,source,issue)
    (issue/'adaptive-volumes.json').write_text(json.dumps(result,indent=2)+'\n')
    print({k:v['adaptive'][-1]['differenceMm3'] for k,v in result['results'].items()})
