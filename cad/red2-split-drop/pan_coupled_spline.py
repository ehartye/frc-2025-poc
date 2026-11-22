"""Measure a physically coupled cartridge carrier and output at the native seated depth."""
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from output_mount import ROOT, digest, read, place
from pan_mechanics import rotation


def run(plugin, refine):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    from mechanical_motion import bounds, common
    d=ROOT/'artifacts/red2-integrated';station=d/'pan-native'
    record=read(station/'critical-interfaces-r2.json')
    if digest(ROOT/record['authority']['file'])!=record['authority']['sha256']:
        raise ValueError('Unregistered robot authority')
    native=station/'integrated-340mm-pan-r2.step'
    assert digest(native)==record['stepSha256']
    labels={}
    allparts,extras=load_solids(native,labels)
    assert len(allparts)==294 and not extras
    identify=lambda path:next(s for k,s in allparts.items() if labels[k]['occurrencePath']==path)
    male=identify(record['spline']['male']);female=identify(record['spline']['female'])
    flat=place(female,rotation(-70))
    # The quarter-turn coordinate map of the donor aligns positive supplier
    # Z with positive robot Y. Pan pitch is right-handed about negative robot Y.
    result=dict(authority=record['authority'],parentNativeSha256=record['archiveSha256'],stepSha256=record['stepSha256'],
        male=record['spline']['male'],female=record['spline']['female'],exactArticles=['REV-41-1601','REV-41-1615'],seatedPlanesSourceZmm=[11.25,11.25],
        basis='Hold the six-bolt housing registration; set only the independent carrier tooth index at zero output pitch. Drive the carrier and splined output together at 1:1, the physical coupling of two directly engaged members. This does not solve internal planetary phase or load.',
        trials={})
    path=station/('r2-coupled-spline-refined.json' if refine else 'r2-coupled-spline-fit.json')
    def save():path.write_text(json.dumps(result,indent=2)+'\n')
    samples=[-11.8,-11.75,-11.725,-11.7,-11.675,-11.65,-11.6]
    if refine:
        # Fit the two opposite load flanks independently from the preserved
        # whole-solid values on each side. Their zero crossings must agree.
        coarse=read(station/'r2-coupled-spline-fit.json')
        t=coarse['trials']
        if coarse['stepSha256']!=record['stepSha256']:raise ValueError('Different spline issue')
        crossings=[]
        for a,b in [(-11.8,-11.75),(-11.7,-11.675)]:
            va,vb=t[str(a)],t[str(b)]
            crossings.append(a-va*(b-a)/(vb-va))
        if abs(crossings[1]-crossings[0])>.00005:
            raise ValueError('Opposite supplier flank calibrations disagree')
        center=sum(crossings)/2
        samples=[center-.002,center-.0002,center,center+.0002,center+.002]
        result['coarseFileSha256']=digest(station/'r2-coupled-spline-fit.json')
        result['oppositeFlankExtrapolatedZerosDeg']=crossings
    for phase in samples:
        b=place(male,rotation(phase))
        vol=common(b,flat)
        result['trials'][str(phase)]=vol
        save()
        print('phase',phase,vol,flush=True)
    best=min(samples,key=lambda k:result['trials'][str(k)])
    result['bestSamplePhaseDeg']=best
    # Compute all connected penetration components in the actual native flat frame.
    raised=place(male,rotation(best))
    op=BRepAlgoAPI_Common(raised,flat)
    if not op.IsDone():raise ValueError('Unresolved connected penetration components')
    sub=TopExp_Explorer(op.Shape(),TopAbs_SOLID);regions=[]
    while sub.More():
        obj=TopoDS.Solid(sub.Current());sub.Next();q=GProp_GProps()
        BRepGProp.VolumeProperties_s(obj,q)
        p=q.CentreOfMass().Coord()
        regions.append(dict(volumeMm3=q.Mass(),centerMm=p,boundsMm=bounds(obj),radialCenterMm=math.hypot(p[0]-205,p[2]-580),
                            azimuthDeg=math.degrees(math.atan2(p[2]-580,p[0]-205))))
    result['residualRegions']=regions
    result['poses']={}
    for name,angle in [('stowed',70),('mid',34),('discharge',-2),('stowed-reset',70)]:
        if name=='stowed-reset':
            result['poses'][name]=result['poses']['stowed']
            continue
        rotor=place(male,rotation(best+angle))
        output=place(female,rotation(angle-70))
        result['poses'][name]=dict(panAngleDeg=angle,carrierAngleDeg=angle+best,
            coupledCommonMm3=common(rotor,output),frozenCarrierCommonMm3=record['spline']['poses'][name]['commonMm3'])
    result['scope']=('Rigid-phase invariant for the two directly mating real BReps. Zero measured overlap in all four named poses at the calibrated index. The coupled native train, backlash, manufactured fit and duty still need proof.' if all(row['coupledCommonMm3']==0 for row in result['poses'].values()) else 'Rigid-phase invariant for the two directly mating real BReps. Residual overlap remains at this sampled index; the fixed-carrier source poses remain a separate negative.')
    save()
    print('registered',best,len(regions),result['poses'],flush=True)

if __name__=='__main__':run(Path(sys.argv[1]), '--refine' in sys.argv)
