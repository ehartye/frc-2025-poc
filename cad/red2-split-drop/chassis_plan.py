"""Bind the native recipe to exact source placements, geometry and open materials."""
import argparse
import json
from pathlib import Path
import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from chassis_mounts import roster
from chassis_interfaces import blank
from chassis_layout import ROOT,OUT,read,digest
from output_mount import place


def main(plugin):
    base,rail,hardware,data=roster(plugin)
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    fit=read(OUT/'module-mounts.json')
    assert not fit['unexpected'] and fit['threadPairs']==48
    items=[]
    def add(s,row):
        props=GProp_GProps();BRepGProp.VolumeProperties_s(s,props)
        row.update(expectedVolumeMm3=abs(props.Mass()),expectedBoundsMm=bounds(s))
        if not row.get('grade') and not row.get('materialBasis'):raise ValueError('Missing source material disposition')
        items.append(row)
    for group in ('frame','bumper','upperSupport'):
        for rec in base[group]:
            name=rec['name'];basis=rec['material']
            grade='6061-T6' if '6061-T6' in basis else '5052-H32 aluminum' if '5052-H32' in basis else 'Sealed Air Ethafoam 220, 37 kg/m3' if 'Ethafoam 220' in basis else None
            add(rail[name] if name in rail else blank(rec),dict(id=name,group=group,kind='stock',grade=grade,
                materialBasis=basis,stock=rec,ports=data['frameBores'].get(name,[])))
    for name,r in data['installed'].items():
        sku=r.get('sku')
        row=dict(id=name,group=name[:2]+' removable mounts',kind=r['kind'],matrixMm=r['matrixMm'],
                 materialBasis=r['material'],grade='6061-T6' if r['kind'] in ('boss','saddle') else
                 'stainless shim' if r['kind']=='shim' else 'REV alloy steel, black oxide, grade unspecified')
        if sku:row.update(article=r['article'],articleSha256=r['sha256'],sku=sku,engagementMm=r['engagementMm'])
        if r['kind']=='saddle':row.update(stockBoundsMm=r['stockBoundsMm'],holes=r['drilling'])
        if r['kind']=='shim':
            row.update(stockBoundsMm=r['stockBoundsMm'],holes=[dict(startMm=[-31.75,82.45,z],axis=[0,1,0],
                radiusMm=2.65,depthMm=.1,process='Cut clearance for selected tab screws') for z in (57.15,82.55)])
        add(hardware[name],row)
    plates=read(OUT/'registered-interfaces.json')['installedPurchasedPlates']
    for name,r in plates.items():
        path=ROOT/r['article'];assert digest(path)==r['articleSha256']
        s=next(iter(load_solids(path)[0].values()))
        add(place(s,np.array(r['matrixMm'])),dict(id=name,group=name[:2]+' module plate registration',
            kind='plate',matrixMm=r['matrixMm'],article=r['article'],articleSha256=r['articleSha256'],
            nativeSupplierOccurrence=r['sourcePath'],materialBasis='Supplier plate material unconfirmed. Exclude default STEP material from mass and structural allowables.'))
    assert len(items)==148 and len({r['id'] for r in items})==148
    p=dict(authority=base['authority'],scope='Private editable foundation issue, pending complete installed drivetrain, superstructure and native robot acceptance.',
           r5Archive=read(OUT/'registered-interfaces.json')['fixture'],
           registeredInterfacesSha256=digest(OUT/'registered-interfaces.json'),registeredMountsSha256=digest(OUT/'module-mounts.json'),
           items=items)
    (OUT/'native-plan.json').write_text(json.dumps(p,indent=2)+'\n')
    print(len(items),'roster',sum(bool(r.get('grade')) for r in items),'assigned design grades','plan',digest(OUT/'native-plan.json'))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('plugin',type=Path);main(ap.parse_args().plugin)
