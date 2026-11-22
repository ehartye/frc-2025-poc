"""Register original module plates and R5 rail seats against the new chassis blanks."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
from OCP.gp import gp_Pnt,gp_Vec
from chassis_layout import ROOT,OUT,digest,read,layout
from output_mount import place


def blank(record):
    def prism(poly,z,travel):
        polygon=BRepBuilderAPI_MakePolygon()
        for x,y in poly:polygon.Add(gp_Pnt(x,y,z))
        polygon.Close()
        return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()).Face(),gp_Vec(*travel)).Shape()
    z=record['zMinMm'];h=record['heightMm']
    travel=record.get('sectionTranslationMm',[0,0,h])
    result=prism(record['planMm'],z,travel)
    if 'openCavityPlanMm' in record:
        bottom,top=record['cavityZMm']
        cutter=prism(record['openCavityPlanMm'],bottom,[travel[0],travel[1],top-bottom])
        result=BRepAlgoAPI_Cut(result,cutter).Shape()
    return result


def prepare(plugin,source):
    sys.path[:0]=[str(plugin/'scripts'),str(ROOT/'experiments/swerve-drive/packaging')]
    from mechanical_motion import bounds,common,distance
    from showcase_contacts import load_solids,box_gap
    from native_module import read as original_shapes
    design=layout();OUT.mkdir(parents=True,exist_ok=True)
    drive=read(ROOT/'experiments/swerve-drive/packaging/module.json')
    assert digest(source)==drive['sourceStepSha256']
    source_shapes=original_shapes(source)
    stock={r['name']:blank(r) for group in ('frame','bumper','upperSupport') for r in design[group]}
    assert len(stock)==60
    plates={};register={}
    for sku in ['REV-21-3005-P01','REV-21-3005-P02']:
        (path,part),=[(k,s) for k,s in source_shapes.items() if k.endswith(':'+sku)]
        # Export just the selected original purchased part with the actual
        # placement it had in the official vendor assembly, still in its frame.
        step=OUT/(sku+'.step')
        writer=STEPControl_Writer();writer.Transfer(part,STEPControl_AsIs)
        assert writer.Write(str(step))==IFSelect_RetDone
        saved,bad=load_solids(step);assert len(saved)==1 and not bad
        saved=next(iter(saved.values()))
        assert np.max(np.abs(np.array(bounds(saved))-bounds(part)))<.0001
        from OCP.GProp import GProp_GProps
        from OCP.BRepGProp import BRepGProp
        def measure(sh):
            p=GProp_GProps();BRepGProp.VolumeProperties_s(sh,p);return p.Mass()
        assert abs(measure(saved)-measure(part))<.001
        for corner,m in drive['modules'].items():
            matrix=np.column_stack((m['sourceXyzToRobot'],[m['xM']*1000,m['yM']*1000,38.1]))
            key=corner+' '+sku
            plates[key]=place(saved,matrix)
            register[key]=dict(sourcePath=path,sourceSha256=digest(source),article=step.relative_to(ROOT).as_posix(),
                               articleSha256=digest(step),matrixMm=matrix.tolist(),sourceVolumeMm3=measure(part))
    upper=ROOT/'artifacts/split-drop-native';native=read(upper/'r5-load-01.json')
    assert native['authority']['archiveSha256']==design['authority']['sha256']
    assert digest(upper/'red2-split-drop-fixture-r5.f3d')==native['sha256']
    assert digest(upper/'r5-export.step')==native['step']['sha256']
    shapes,bad=load_solids(upper/'r5-export.step');assert len(shapes)==93 and not bad
    inv=read(upper/'r5-home-native.json')
    assert inv['archiveSha256']==native['sha256']
    known={r['path']+'/'+r['name']:r['boundsMm'] for r in inv['bodies']}
    cassette={}
    for s in shapes.values():
        name,=[k for k,box in known.items() if np.max(abs(np.array(box)-bounds(s)))<.04]
        cassette[name]=s
    all_shapes=stock|plates|cassette;boxes={k:bounds(s) for k,s in all_shapes.items()}
    def check(a,b):
        gap=box_gap(boxes[a],boxes[b])
        if gap>8:return
        gap=distance(all_shapes[a],all_shapes[b])
        if gap>8:return
        return dict(a=a,b=b,gapMm=gap,positiveMm3=common(all_shapes[a],all_shapes[b]) if gap<1e-6 else 0.)
    results=[]
    def compare(first,second):
        for a in first:
            for b in second:
                row=check(a,b)
                if row:results.append(row)
    compare(stock,plates)
    compare(stock,cassette)
    for i,a in enumerate(stock):compare([a],list(stock)[i+1:])
    seat=[]
    for b in design['upperSupport']:
        owner=b['seatAgainst'];names=owner if isinstance(owner,list) else ['Fixture bench rails:1/'+owner]
        for contact in names:
            row=check(b['name'],contact)
            assert row and row['gapMm']<1e-5 and row['positiveMm3']<.001,(contact,row)
            seat.append(row)
    result=dict(authority=design['authority'],fixture=native['sha256'],exchange=native['step']['sha256'],
                purchasedConfiguration='Original supplier upper and lower plates at four sites. The selected V1.1 wheel and fasteners remain outside this partial roster.',
                manufacturerSourceSha256=digest(source),installedPurchasedPlates=register,
                frameAndBumperStockCount=len(stock),expectedSourceBodyCounts=[8,93],nearby=results,
                nominalSupportContacts=seat,unintendedPositive=[r for r in results if r['positiveMm3']>.001])
    (OUT/'registered-interfaces.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('plugin',type=Path);ap.add_argument('source',type=Path)
    a=ap.parse_args();r=prepare(a.plugin,a.source)
    print('stock',r['frameAndBumperStockCount'],'plates',len(r['installedPurchasedPlates']),'seats',len(r['nominalSupportContacts']),'unexpected',r['unintendedPositive'])
