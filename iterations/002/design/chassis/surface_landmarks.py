"""Reconcile Fusion seam changes against the real exported boundary, retaining adverse translated points."""
from pathlib import Path
import argparse,json,sys
import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.gp import gp_Pnt
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SHELL


def run(project,plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from fusion_authority import register_path,verify,load,resolve,digest
    from showcase_contacts import load_solids
    from mechanical_motion import distance
    register=register_path(project);lineage=register.parent/'issue-001/lineage.json';native=verify(project,register,lineage)
    f=register.parent.parent/'chassis/independent-123.json';first=load(f)
    issued=resolve(project,load(lineage)['step'])
    previous=resolve(project,load(register.parent/'issue-123.json')['issuedStep'])
    if issued.read_bytes().split(b'DATA;',1)[1]!=previous.read_bytes().split(b'DATA;',1)[1]:raise ValueError('Diagnostic and canonical body records differ')
    bodies,_=load_solids(issued);rows={};control={}
    for name,r in first['sourceChecks'].items():
        k=r['placed'];it=TopExp_Explorer(bodies[k],TopAbs_SHELL);surfaces=[]
        while it.More():surfaces.append(it.Current());it.Next()
        if not surfaces:raise ValueError('Missing closed boundary: '+k)
        p=r['sourceWorldVerticesMm']
        errors=[min(distance(BRepBuilderAPI_MakeVertex(gp_Pnt(*v)).Vertex(),s) for s in surfaces) for v in p]
        rows[name]=dict(occupiedVolumeDifferenceMm3=r['placedVolumeMm3']-r['sourceVolumeMm3'],
                        maxSourceLandmarkToTrueBoundaryMm=max(errors),gapsMm=errors,maxNearestExportedVertexMm=max(r['nearestExportedVertexErrorsMm']))
        if '91251A355' in name:
            b=np.array(p[0]);displaced=b+np.array([12,0,0])
            control[name]=min(distance(BRepBuilderAPI_MakeVertex(gp_Pnt(*displaced)).Vertex(),s) for s in surfaces)
    return dict(authority=native,inspectionSha256=digest(f),deltaControlMm=12,displacedScrewWitnesses=control,
                maximumGapMm=max(r['maxSourceLandmarkToTrueBoundaryMm'] for r in rows.values()),rows=rows,
                scope='Three independently located finite surface landmarks per purchased item in the actual F3D-derived STEP. A close surface observation and matching bounds do not establish equality of the complete occupied sets. Supplier re-import volume discrepancies remain separate.')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for n in ('root','plugin','out'):p.add_argument(n,type=Path)
    a=p.parse_args();d=run(a.root.resolve(),a.plugin);a.out.write_text(json.dumps(d,indent=2)+'\n');print(d['maximumGapMm'],d['displacedScrewWitnesses'])
