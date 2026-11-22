"""Reconcile actual captive-receiver load direction, tube corner relief and threaded solids."""
import argparse
import json
from pathlib import Path
import sys
import math

from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec

from crossmember_envelope import box,cylinder
from four_corner_packaging import polygon
from port_anchor_fit import ROOT,HERE,UP,SIDE,Y,Z,STATIONS,oriented,cut,pieces


def corrected_bar():
    # The datum side of the blank bears directly on the rail inside surface.
    # Five-millimeter stock-corner relief is separate from the commissioned pilot diameters.
    l,r,top=321.45,339.25,Z-3.175
    section=[(l,top-10),(r,top-10),(r,top-5),(r-5,top),(l+5,top),(l,top-5)]
    bar=BRepPrimAPI_MakePrism(polygon([(-77,y,z) for y,z in section]),gp_Vec(154,0,0)).Shape()
    for x in STATIONS:
        pilot=cylinder((x,Y,top-5),UP,6.325,17.50)
        bar=BRepAlgoAPI_Fuse(bar,pilot).Shape()
        bar=cut(bar,cylinder((x,Y,top-11),UP,2.5527,25))
    for x in (-50.,50.):
        bar=cut(bar,cylinder((x,321.3,top-7.1),SIDE,2.0574,16))
    return bar


def fillet(rad,xlo=-78,xhi=78):
    y,z=321.45,Z-3.175; ymax=340.5
    rows=[]
    for sign,yedge in ((1,y),(-1,ymax)):
        rcenter=yedge+sign*rad
        a,b=sorted((yedge,rcenter))
        material=box((xlo,a,z-rad),(xhi,b,z))
        bore=cylinder((xlo-.1,rcenter,z-rad),(1,0,0),rad,xhi-xlo+.2)
        rows.append(cut(material,bore))
    return rows


def run(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from component_shape import volume
    from fusion_authority import verify,load,resolve,register_path,digest
    from mechanical_motion import distance,common,bearing_area,bounds
    from interface_consistency import inscribed_balls
    from showcase_contacts import load_solids
    reg=register_path(ROOT);line=reg.parent/'issue-003/lineage.json';auth=verify(ROOT,reg,line)
    raw,sheets=load_solids(resolve(ROOT,load(line)['step']))
    prior=load(HERE/'port-anchor-fit.json')
    if prior['authority']['stepSha256']!=auth['stepSha256'] or len(raw)!=488 or sheets:raise ValueError('Incomplete parent')
    sid=prior['shopScrews'];host='Port longitudinal socket rail [1.2.1]'
    stock=pieces(raw[host],raw[sid['main']],raw[sid['keeper']])
    before=stock['nutbar'];bar=corrected_bar()
    side=raw[host];top=Z-3.175
    for x in STATIONS:side=cut(side,cylinder((x,Y,Z-3.5),UP,6.4,4))
    for x in (-50.,50.):side=cut(side,cylinder((x,318.1,top-7.1),SIDE,2.60,3.5))
    def at(s,v):return oriented(s,(0,0,0),UP,v,UP)
    # The previous keeper drew a free 0.625 mm side gap closed. Measure the resulting four-pilot obstruction.
    bad=common(at(before,(0,-.625,0)),stock['machined rail'])
    ligaments=[]
    for R in (3.,4.,5.,6.):
        intruders=fillet(R)
        machined=[]
        for solid in intruders:
            for x in STATIONS:
                solid=cut(solid,cylinder((x,Y,Z-12),UP,6.4,13))
            machined.append(solid)
        ligaments.append(dict(radiusMm=R,corrected=[common(bar,s) for s in intruders],
                              correctedAfterThroughReam=[common(bar,s) for s in machined],
                              former=[common(before,s) for s in intruders]))
    # Independently select material strictly inside the true purchased screw AND tap-drill stock.
    # Direct common on this fine thread can be empty. A positive interior witness is decisive.
    thread={}
    for name,bolt,probes in [
        ('vertical 1/4-20',stock['bolt 65.0'],
         [[65+2.8,Y,z/10] for z in range(857,928,1)]),
        ('side 10-32',at(stock['retainer 50.0'],(0,0,(top-7.1)-(Z-7.175))),
         [[50+2.2,y/10,top-7.1] for y in range(3241,3282,1)])]:
        balls,v=inscribed_balls(bolt,bar,probes,1.e-5)
        thread[name]=dict(directCommonMm3=common(bolt,bar),sampleSize=len(probes),
                          disjointInteriorMm3=v,interiorWitness=balls[:3],allSuccessfulProbes=len(balls))
    reg_area=bearing_area(bar,side,(0,-1,0))
    # Lower receiver slides with the datum side tangent. The swept positive material remains within the
    # raw tube. A continuous tube's interior fillets occur along X, so testing 154 mm of unchanged section suffices.
    prism=box((-77,321.45,top-24),(405,339.25,top-1.5))
    lowered=at(bar,(0,0,-14))
    controls=dict(previousSideDrawOverlapMm3=bad,currentSideDatumAreaMm2=reg_area,
        currentTopDatumAreaMm2=bearing_area(bar,side,UP),installedOverlapMm3=common(bar,side),
        axialPrismBareTubeCommonMm3=common(prism,side),loweredFourMmFilletsMm3=[common(lowered,s) for s in fillet(4)],
        maxWorkingInnerFilletMm=4.0)
    for x in STATIONS:
        # X lip range excludes the raw machining burr or residual blend.
        controls[f'pilot {x} OD witness overlap with true bore at side draw 0.625 mm']=common(at(cylinder((x,Y,top),UP,6.325,12.5),(0,-.625,0)),side)
    return dict(authority=auth,previousTrialSha256=digest(HERE/'port-anchor-fit.json'),
        previousAndRevisedVolumesMm3=[volume(before),volume(bar)],stockBoundsMm=bounds(bar),
        controls=controls,fillets=ligaments,thread=thread,limitation='As-fabricated geometry needs a positive datum spotface or equivalent fixture. The receiver touches the near wall and is drilled through its 10-32 tap axis at installed Z, then all four pilot holes must be matched in one setup. Conservative inside-radius gauge is project receiving requirement. Thread-interior evidence is against minor-diameter tap-drill stock only; cosmetic tap drill does not represent a ready physical female thread. Seat registration, actual assembled tightening scatter and alloy remain independent checks.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('plugin',type=Path)
    d=run(p.parse_args().plugin)
    (HERE/'port-anchor-locators.json').write_text(json.dumps(d,indent=2)+'\n')
    print(d['controls'],d['fillets'],[(k,r['disjointInteriorMm3']) for k,r in d['thread'].items()])

