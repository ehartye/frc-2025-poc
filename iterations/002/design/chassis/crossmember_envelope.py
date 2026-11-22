"""Fit a through member, its removable end block and the four external service axes in the active 244."""
import argparse
import json
import math
import sys
from pathlib import Path

from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from four_corner_packaging import rotated


def box(lo, hi):
    return BRepPrimAPI_MakeBox(gp_Pnt(*lo), gp_Pnt(*hi)).Shape()


def cylinder(start, direction, radius, length):
    axis = gp_Ax2(gp_Pnt(*start), gp_Dir(*direction))
    return BRepPrimAPI_MakeCylinder(axis, radius, length).Shape()


def run(root, plugin):
    sys.path.insert(0, str(plugin / 'scripts'))
    from fusion_authority import register_path, verify, load, resolve
    from mechanical_motion import bounds, distance, common
    from showcase_contacts import load_solids, box_gap
    register = register_path(root)
    lineage = register.parent / 'issue-002/lineage.json'
    issue = verify(root, register, lineage)
    installed, sheets = load_solids(resolve(root, load(lineage)['step']))
    if len(installed) != 244 or sheets:
        raise ValueError('Expected both configured fronts and the two installed rails')
    obstacles = {('installed', k): v for k,v in installed.items()}
    for k, v in installed.items():
        obstacles[('rear projection', k)] = rotated(v, math.pi)
    boxes = {k: bounds(v) for k, v in obstacles.items()}
    forms = {}
    for x in (0, 100, -100, 150):
        forms[f'{x}/through-member'] = box((x-25.4,-339.6,101.6), (x+25.4,339.6,177.8))
        for side in (1,-1):
            # The two backing plates and tools live on the outer tube side. The solid
            # stock is a conservative obstruction envelope before any access holes.
            cy = side*345
            forms[f'{x}/{side}/backing'] = box((x-30,cy-2.2,105), (x+30,cy+2.2,175))
            for dx in (-12.7,12.7):
                for z in (114.3,165.1):
                    forms[f'{x}/{side}/outside-access/{dx}/{z}'] = cylinder((x+dx,side*343,z), (0,side,0), 7, 110)
            for offset in (28,47,66):
                for i, z in enumerate((119,160)):
                    # Stagger left/right screws so the blind threads never meet.
                    sign = (1 if i else -1) * (1 if offset==47 else -1)
                    forms[f'{x}/{side}/side-access/{offset}/{z}'] = cylinder((x+sign*25.4,side*(339.6-offset),z),(sign,0,0),6.5,63)
    data={}
    for name, form in forms.items():
        own=bounds(form); candidates=sorted((box_gap(own,b),k) for k,b in boxes.items()); exact=[]
        for lower,k in candidates:
            if lower>3:
                break
            gap=distance(form,obstacles[k])
            vol=common(form,obstacles[k]) if gap<.0001 else 0
            exact.append(dict(group=k[0],id=k[1],gapMm=gap,commonMm3=vol))
        data[name]=dict(checked=exact,remainingGapBoundMm=next((a for a,b in candidates if a>3),None))
    return dict(authority=issue, scope='244 native/STEP with rear two modules, remaining sockets at the exact central half turn. Member is a solid bounding envelope. End blocks lie within its envelope. Tool cylinders only bound straight entry/removal after the external bumper is removed; commissioned grip, ring/backing holes, side approach conflicts with other proposed through members, fastener and internal assembly are separate.', records=data)


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('project',type=Path);p.add_argument('plugin',type=Path)
    args=p.parse_args(); data=run(args.project.resolve(),args.plugin.resolve())
    out=Path(__file__).parent/'crossmember-envelope.json'
    out.write_text(json.dumps(data,indent=2)+'\n')
    for x in (0,100,-100,150):
        rows={k:v for k,v in data['records'].items() if k.split('/')[0]==str(x)}
        coll={k:[(r['id'],round(r['commonMm3'],1)) for r in v['checked'] if r['commonMm3']>.01] for k,v in rows.items()}
        print(x,[(k,v[:1]) for k,v in coll.items() if v])
