"""Measure a proposed port terminal attachment against the active four-module Fusion export."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.gp import gp_Trsf
from OCP.TopLoc import TopLoc_Location

from crossmember_envelope import box, cylinder

HERE = Path(__file__).parent
ROOT = HERE.parents[3]
STATIONS = (-65., -35., 35., 65.)
Y = 328.9375
Z = 94.63336287672979
UP = (0, 0, 1)
SIDE = (0, 1, 0)


def oriented(shape, point, direction, target, axis):
    """Register an actual head bearing plane and thread axis, preserving the complete supplier shape."""
    a, b = np.array(direction), np.array(axis)
    a = a / np.linalg.norm(a)
    cr = np.cross(a, b)
    if np.dot(a, b) < -.999999999:
        matrix = np.diag([1., -1., -1.])
    else:
        k = np.array([[0, -cr[2], cr[1]], [cr[2], 0, -cr[0]], [-cr[1], cr[0], 0]])
        matrix = np.eye(3) + k + k @ k / (1 + np.dot(a, b))
    off = np.array(target) - matrix @ np.array(point)
    m = gp_Trsf()
    m.SetValues(*np.column_stack([matrix, off]).ravel())
    return shape.Moved(TopLoc_Location(m))


def cut(stock, tool):
    op = BRepAlgoAPI_Cut(stock, tool)
    if not op.IsDone() or op.Shape().IsNull():
        raise ValueError('Incomplete machining')
    return op.Shape()


def pieces(host, supplier, keep):
    bore = lambda x, y, z, r, l: cylinder((x, y, z), UP, r, l)
    port = host
    terminal = cut(box((-127,314.2,101.6),(127,339.6,177.8)),
                   box((-128,317.375,104.775),(128,336.425,174.625)))
    bar = box((-77,322.075,Z-11.175),(77,339.875,Z-3.175))
    shapes = {}
    for x in STATIONS:
        port = cut(port, bore(x,Y,Z-3.5,6.4,4))
        terminal = cut(terminal, bore(x,Y,101.5,6.4,3.5))
        terminal = cut(terminal, bore(x,Y,174.5,7,3.5))
        bar = BRepAlgoAPI_Fuse(bar, bore(x,Y,Z-3.175,6.325,12.50)).Shape()
        # Nominal tap-drill representation keeps the interference with the supplier's modeled threads measurable.
        bar = cut(bar, bore(x,Y,Z-12,2.5527,23))
        shapes[f'spacer {x}'] = cut(box((x-14,Y-9.65,Z),(x+14,Y+9.65,101.6)),bore(x,Y,Z-.1,6.4,8))
        shapes[f'washer {x}'] = cut(box((x-11,318.0,104.775),(x+11,335.8,107.95)),bore(x,Y,104.7,3.35,3.4))
        shapes[f'bolt {x}'] = oriented(supplier,(274.815795103,241.078975516,75.583362877),UP,(x,Y,107.95),(0,0,-1))
    for x in (-50.,50.):
        port = cut(port, cylinder((x,318.1,Z-7.175),SIDE,2.60,3.5))
        bar = cut(bar, cylinder((x,321.9,Z-7.175),SIDE,2.0574,15))
        shapes[f'retainer {x}'] = oriented(keep,(312.002520539,262.623125112,84.244609809),
                    (-.478690491070075,.478690491070072,-.736010072973314),
                    (x,318.275,Z-7.175),SIDE)
    return {'machined rail':port, 'terminal':terminal, 'nutbar':bar, **shapes}


def run(plugin):
    sys.path.insert(0, str(plugin/'scripts'))
    from component_shape import volume
    from fusion_authority import register_path, verify, resolve, load, digest
    from mechanical_motion import bounds, common, distance, bearing_area
    from showcase_contacts import load_solids, box_gap
    reg = register_path(ROOT)
    line = reg.parent/'issue-003/lineage.json'
    auth = verify(ROOT,reg,line)
    installed, sheets = load_solids(resolve(ROOT,load(line)['step']))
    if sheets or len(installed)!=488:
        raise ValueError('Missing installed bodies')
    prior = load(HERE/'lower-488-independent.json')
    if prior['authority']['stepSha256'] != auth['stepSha256']:
        raise ValueError('Source selection came from a different issue')
    donor = prior['measured']['FL']
    select = lambda text: next(v['placed'] for k,v in donor.items() if text in k)
    host = 'Port longitudinal socket rail [1.2.1]'
    bolt, keep = select('91251A542'), select('97763A232')
    stock = pieces(installed[host], installed[bolt], installed[keep])
    identities = {k:dict(volumeMm3=volume(s),boundsMm=bounds(s)) for k,s in stock.items()}
    if any(r['volumeMm3']<=0 for r in identities.values()):
        raise ValueError('Empty article')
    bb = {k:bounds(v) for k,v in installed.items() if k!=host}
    census, mutual, seats = {}, [], []
    for k,a in stock.items():
        b = bounds(a)
        near = sorted((box_gap(b,v),other) for other,v in bb.items())
        census[k]=[dict(other=n,gapMm=distance(a,installed[n]),overlapMm3=common(a,installed[n])) for lb,n in near if lb<.001]
    for i,(name,a) in enumerate(stock.items()):
        for other,b in list(stock.items())[i+1:]:
            lb=box_gap(bounds(a),bounds(b))
            if lb>.001:continue
            measured=common(a,b)
            gap=distance(a,b)
            if measured>.005 or gap<.001:
                mutual.append(dict(a=name,b=other,overlapMm3=measured,gapMm=gap))
    def seat(a,b,axis):
        seats.append(dict(a=a,b=b,gapMm=distance(stock[a],stock[b]),
            commonMm3=common(stock[a],stock[b]),areaMm2=bearing_area(stock[a],stock[b],axis)))
    for x in STATIONS:
        seat(f'bolt {x}',f'washer {x}',(0,0,-1))
        seat(f'washer {x}','terminal',(0,0,-1))
        seat('terminal',f'spacer {x}',(0,0,-1))
        seat(f'spacer {x}','machined rail',(0,0,-1))
    for x in (-50.,50.):seat(f'retainer {x}','machined rail',SIDE)
    seat('nutbar','machined rail',UP)
    # Continuous occupied prism bounds a bar with its pilots, lowered 14 mm while sliding.
    # Parked full chassis has genuine fasteners crossing that occupied corridor. Show a material hit with the actual bar.
    bar=stock['nutbar']
    def shift(a,xyz):return oriented(a,(0,0,0),UP,xyz,UP)
    envelope = box((-77,322.075,Z-25.175),(405,339.875,Z-4.675))
    travel={}
    for name,s in [('nominal bare rail lowered axial prism',stock['machined rail']),
                   ('same rail with pilot still at final height',stock['machined rail'])]:
        trial=envelope if 'lowered' in name else shift(bar,(95,0,0))
        travel[name]={'volumeMm3':common(trial,s),'gapMm':distance(trial,s)}
    # Exact vertical swept union for monotonic straight body+pilot upward placement, including the annuli.
    raising = box((-77,322.075,Z-25.175),(77,339.875,Z-3.175))
    for x in STATIONS:
        raising=BRepAlgoAPI_Fuse(raising,cylinder((x,Y,Z-17.175),UP,6.325,26.5)).Shape()
    travel['vertical lift after reaching final X']={'overlapMm3':common(raising,stock['machined rail'])}
    late={}
    for k,r in donor.items():
        ident=r['placed']
        if '91251A355' in ident and box_gap(bounds(envelope),bb[ident])<.001:
            q=shift(bar,(185,0,-14))
            late[ident]=dict(gapMm=distance(q,installed[ident]),commonMm3=common(q,installed[ident]))
    for x in STATIONS:
        entry=cylinder((x,Y,114.31),UP,6.5,110)
        travel[f'{x} socket straight entry']={k:common(entry,s) for k,s in {**stock,**{i:installed[i] for i,b in bb.items() if box_gap(bounds(entry),b)<.001}}.items() if box_gap(bounds(entry),bounds(s))<.001}
    bad=box((-10,Y-10,104.775),(10,Y+10,107.95))
    return dict(authority=auth,sourceRegistrationSha256=digest(HERE/'lower-488-independent.json'),
        shopScrews={'main':bolt,'keeper':keep},articles=identities,placedScope=len(stock)*487,
        installedNear=census,proposedPairs=mutual,seats=seats,pathBounds=travel,
        closedFrameInsert=late,wideWasherAdverseMm3=common(bad,stock['terminal']),
        limits='Port-only machined proposal. Raw open rail receives a lowered nutbar before either corner. Lift it at final X before fitting keepers. Shaft thread vs minor-diameter tap cylinders is a measured CAD representation; true tolerance class, finish and running thread fit are separate. Surface assignments and source article signatures are unchanged. 13 mm cylindrical socket envelope checks straight insertion only. Outer bumper ring, horizontal attachments, internal tube fillets, retention/preload, tool drive reach/torque, and assembled floor/upper duty are open.')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('plugin',type=Path)
    data=run(parser.parse_args().plugin.resolve())
    (HERE/'port-anchor-fit.json').write_text(json.dumps(data,indent=2)+'\n')
    print('scope',data['placedScope'],'seats',[(x['a'],round(x['areaMm2'],3)) for x in data['seats']])
    print('path',data['pathBounds'],'closed',data['closedFrameInsert'],'wide',data['wideWasherAdverseMm3'])

