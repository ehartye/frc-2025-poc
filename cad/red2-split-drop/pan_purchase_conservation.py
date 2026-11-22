"""Audit stock-volume conservation for inconclusive supplier-shape Booleans in an issued robot."""
import json
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from output_mount import ROOT, digest, place, read


def volume(shape):
    props=GProp_GProps()
    BRepGProp.VolumeProperties_s(shape,props,True)
    return props.Mass()


def test(plugin, revision):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    d=ROOT/'artifacts/red2-integrated'
    station=d/'pan-native'
    study=read(station/f'first-party-purchases-{revision}.json')
    assert digest(ROOT/study['authority']['file'])==study['authority']['sha256']
    master=station/f'integrated-340mm-pan-{revision}.step'
    assert digest(master)==study['stepSha256']
    buys=read(d/'pan-drive-registration.json')['components']
    if revision=='r2':buys+=read(d/'pan-assembly-fasteners.json')['preparedForNextNativeIssue']
    components={p['id']:p for p in buys}
    resolved=load_solids(master)[0]
    rows={}
    for item in study['failed']+study['unresolved']:
        row=study['roster'][item]
        source=ROOT/components[row['part']]['source']
        assert digest(source)==row['sourceSha256']
        placed=resolved[item]
        supplier=place(load_solids(source)[0][row['supplierSolid']],np.array(row['transform']).reshape(4,4)[:3])
        official,current=volume(supplier),volume(placed)
        def op(tool,a,b):
            q=tool(a,b)
            if not q.IsDone() or q.Shape().IsNull():return None
            return volume(q.Shape())
        missing=op(BRepAlgoAPI_Cut,supplier,placed)
        added=op(BRepAlgoAPI_Cut,placed,supplier)
        shared=op(BRepAlgoAPI_Common,supplier,placed)
        r=dict(sourcePath=source.relative_to(ROOT).as_posix(),sourceSha256=digest(source),nativeOccurrence=row['path'],sourceConstituent=row['supplierSolid'],frozenMatrix=row['transform'],
               sourceVolumeMm3=official,receivedVolumeMm3=current,sourceTopologyValid=BRepCheck_Analyzer(supplier).IsValid(),receivedTopologyValid=BRepCheck_Analyzer(placed).IsValid(),
               commonMm3=shared,absentMm3=missing,extraMm3=added,conservationDifferenceMm3=added-missing-(current-official) if None not in (added,missing) else None,
               sourceLessCommonAndAbsentMm3=official-shared-missing if None not in (shared,missing) else None,
               receivedLessCommonAndExtraMm3=current-shared-added if None not in (shared,added) else None)
        rows[item]=r
        print(row['part'],official,current,missing,added,shared,r['conservationDifferenceMm3'],r['sourceTopologyValid'],r['receivedTopologyValid'],flush=True)
    result=dict(authority=study['authority'],archiveSha256=study['archiveSha256'],stepSha256=study['stepSha256'],driverSha256=study['pluginSha256'],cases=rows,
                note='A directed difference and common must conserve each input solid. Contradictory or negative Boolean volumes remain unresolved, including when reported as zero or positive. No tolerance override has been applied.')
    (station/f'purchase-boolean-conservation-{revision}.json').write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':
    test(Path(sys.argv[1]), sys.argv[2])
