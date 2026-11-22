"""Check an unreleased frame screw, and quantify loads on the installed upper sills."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from chassis_mounts import roster
from chassis_layout import ROOT,OUT,read,digest
from output_mount import place


def main(plugin):
    base,rail,parts,register=roster(plugin)
    from mechanical_motion import common,distance,bounds
    from showcase_contacts import load_solids
    control={}
    for k,r in register['installed'].items():
        if r.get('sku')!='REV-29-2951':continue
        owner=k[:4]+' saddle';s=parts[owner]
        lowered=place(s,np.column_stack((np.eye(3),[0.,0.,-1.])))
        volume=common(lowered,parts[k])
        assert volume>.001
        control[k]=dict(saddle=owner,parkedCommonMm3=common(s,parts[k]),wrongOrderDropMm=1.,wrongOrderCommonMm3=volume)
    cassette=ROOT/'artifacts/split-drop-native'
    cold=read(cassette/'r5-load-01.json')
    assert cold['authority']['archiveSha256']==base['authority']['sha256']
    assert digest(cassette/'r5-export.step')==cold['step']['sha256']
    assert digest(cassette/'red2-split-drop-fixture-r5.f3d')==cold['sha256']
    manifest=read(cassette/'r5-home-native.json')
    assert manifest['archiveSha256']==cold['sha256']
    lookup={b['path']+'/'+b['name']:b['boundsMm'] for b in manifest['bodies']}
    mats=cold['native']['materialReadback']
    densities={'Supplier alloy steel, black oxide, grade unspecified':7850,'Supplier nylon insert, grade unspecified':1140,
        'Supplier zinc plated steel shell, grade unspecified':7850,'Aluminum 6061 T6 0 Cold Formed':2700,'UHMW, White':940,
        'Supplier 6061-T6 aluminum':2700,'Supplier 4140 steel, temper unspecified':7850,
        'Supplier 7075 aluminum, temper unspecified':2810}
    # Nominal densities with open alloy grades are an estimate. The screen
    # adds 25% mass and tests the isolated upper, before the unmodeled drives.
    solids,bad=load_solids(cassette/'r5-export.step');assert len(solids)==93 and not bad
    rows=[]
    for s in solids.values():
        b=bounds(s);name,=[k for k,v in lookup.items() if max(abs(np.array(v)-b))<.04]
        props=GProp_GProps();BRepGProp.VolumeProperties_s(s,props)
        rows.append(dict(name=name,kg=props.Mass()*densities[mats[name]]/1e9,centerMm=props.CentreOfMass().Coord(),
                         material=mats[name],densityKgM3=densities[mats[name]],moving=not name.startswith('Fixture bench rails:')))
    result={}
    for xshift in (0.,404.):
        items=[(b['kg'],np.array(b['centerMm'])+[xshift if b['moving'] else 0,0,0]) for b in rows]
        items.append((.816466266,np.array([196.046336+xshift,0,609.924660])))
        total=sum(w for w,_ in items);cg=sum(w*p for w,p in items)/total
        a,b=-144.,148.
        # Two sills share a symmetric static vertical load. Keep discrete
        # load centers to infer the worst positive/negative span reactions.
        # A prescribed 3g vertical load multiplied by 1.25 provisional mass
        # is a design case. No measurement of hardware shock is implied.
        wts=[(3*9.80665*1.25*w/2,p[0]) for w,p in items]
        front=sum(w*(x-a)/(b-a) for w,x in wts);rear=sum(w for w,x in wts)-front
        def bend(x):
            return rear*max(0,x-a)+front*max(0,x-b)-sum(w*max(0,x-pos) for w,pos in wts)
        # Load centers on the moving carriage cannot directly load an unbounded
        # beam. The actual captured rails transmit the net at their interface.
        # For statics only the external resultant and tower reactions survive.
        # Sill end moment at its forward tower is still conservative if all
        # forward material is transferred at its own center.
        xs=sorted({a,b,*[x for _,x in wts]})
        moments={x:bend(x) for x in xs}
        I=(18*30**3-14*26**3)/12
        result[str(xshift)]=dict(cassettePlusPipeKg=total,centerMm=cg.tolist(),
            loadMultiplier=3.75,perSideAftSupportN=rear,perSideForeSupportN=front,
            upliftAtAftN=max(0,-rear),sillGrossInertiaMm4=I,distributedLoadMomentEstimateNmm=max(abs(m) for m in moments.values()),
            discreteModelBendingStressEstimateMPa=max(abs(m) for m in moments.values())*15/I,
            limitations='A screen of the 93-body cassette plus admitted pipe; selected motion drives, transient load path at actual keeper shoes, heat-affected weld strength, off-center impact, bonded foam and installed battery are outside this calculation.')
    path=OUT/'tower-load-and-stuck-attachment.json'
    path.write_text(json.dumps(dict(authority=base['authority'],fixture=cold['sha256'],stock=rows,heldHardware=control,loads=result),indent=2)+'\n')
    print('stuck',min(r['wrongOrderCommonMm3'] for r in control.values()),'loads',result)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('plugin',type=Path);main(p.parse_args().plugin)
