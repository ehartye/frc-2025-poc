"""Bound full azimuth rotation using the swept annular envelope of a finite frame socket."""
import argparse
import json
import math
import sys
from pathlib import Path
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from select_l2 import digest


def envelope(center, radius, height, lower, outer):
    def cylinder(r, y, h):
        line = gp_Ax2(gp_Pnt(center[0],y,center[1]),gp_Dir(0,1,0))
        return BRepPrimAPI_MakeCylinder(line,r,h).Shape()
    return BRepAlgoAPI_Cut(cylinder(outer,lower,height),
                           cylinder(radius,lower-1,height+2)).Shape()


def run(plugin, issue, original=None):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds,distance,common
    from showcase_contacts import load_solids
    selector = json.loads((issue/'selection.json').read_bytes())
    source = original or issue/'sds-mk4i-neo-l2.step'
    expected = selector['sourceSha256'] if original else selector['derivedStepSha256']
    if digest(source) != expected:
        raise ValueError('Changed configured exchange')
    labels = {}
    solids,sheets = load_solids(source,labels)
    if len(solids)!=(125 if original else 121) or sheets:
        raise ValueError('Unexpected configured module')
    rotation_axis = (-66.675,-66.675)
    # Express the new straight front and side socket zones in SDS source X/Z.
    # Symmetry about X=Z makes the two annuli identical. The nearest end
    # extends 5.625 mm past the innermost of the three 25.4 mm-spaced bolts.
    nearest_face = 41.275
    nearest_end = 42.0
    farthest_face = nearest_face+25.4
    farthest_end = nearest_end+70.5
    inner = math.hypot(nearest_face,nearest_end)
    outer = math.hypot(farthest_face,farthest_end)
    zone = envelope(rotation_axis,inner,50.8,44.45,outer)
    rotor = [k for k,v in labels.items()
             if 'MK4 Turret Assembly_Billet Wheel' in v['occurrencePath'] and
                v['definition'] not in selector['configuration']['excluded']]
    if len(rotor)!=43:
        raise ValueError('Rotating article roster changed: '+str(len(rotor)))
    evaluated = {}
    for key in rotor:
        gap = distance(solids[key],zone)
        evaluated[key] = dict(minimumSeparationToConservativeSocketEnvelopeMm=gap)
        if gap<.001:
            evaluated[key]['annularIntersectionMm3']=common(solids[key],zone)
    tread, = [k for k in rotor if k.startswith('Tread,')]
    # Move the socket radially inward to the steering axis at its original
    # elevation to require an actual swept tread intersection.
    adverse = envelope(rotation_axis,10.,50.8,44.45,outer)
    negative = common(solids[tread],adverse)
    if negative < 1.:
        raise ValueError('The intersecting adverse control did not trigger')
    return dict(sourceSelectorSha256=selector['selectorSha256'],actualInputStepSha256=digest(source),untouchedManufacturerExchange=bool(original),
        shaftAxisXZmm=rotation_axis,rotationAxis='supplier +Y',rangeDeg=[0,360],frameSectionMm=[50.8,25.4,3.175],
        sourceYWindowMm=[44.45,95.25],nearestSocketCornerRelativeXZmm=[nearest_face,nearest_end],
        innerOuterSweptRadiiMm=[inner,outer],roster=evaluated,
        minimumGapMm=min(v['minimumSeparationToConservativeSocketEnvelopeMm'] for v in evaluated.values()),
        adverseTread={'name':tread,'innerRadiusMm':10,'commonMm3':negative},
        scope='Full-turn lower bound for every source turret member against both proposed closed rectangular socket envelopes. Excludes retained azimuth pulley and stationary/sensor groups, wire motion, tread compression and housing/rail deflection. An envelope touch requires independent contact-phase analysis.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plugin',type=Path)
    parser.add_argument('issue',type=Path)
    parser.add_argument('--original',type=Path)
    args=parser.parse_args()
    result=run(args.plugin,args.issue,args.original)
    name='socket-steering-original.json' if args.original else 'socket-steering.json'
    (args.issue/name).write_text(json.dumps(result,indent=2)+'\n')
    print(len(result['roster']),result['minimumGapMm'],result['adverseTread'])
