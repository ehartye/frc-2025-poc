"""Fit the finite 4×1 structural perimeter to two issued modules and both registered rear poses."""
import sys,json,math
from pathlib import Path
from four_corner_packaging import band,rotated
root=Path.cwd();d=root/'iterations/002/design';sys.path.insert(0,str(Path(sys.argv[1])/'scripts'))
from fusion_authority import register_path,verify,load,resolve
from showcase_contacts import load_solids,box_gap
from mechanical_motion import bounds,distance,common
reg=register_path(root);lin=reg.parent/'issue-002/lineage.json';authority=verify(root,reg,lin)
parts,_=load_solids(resolve(root,load(lin)['step']));parts={k:v for k,v in parts.items() if 'socket rail' not in k}
assert len(parts)==242
scenes={'front':parts,'rear':{k:rotated(v,math.pi) for k,v in parts.items()}}
outline,ss=band(370,365,42,25.4,101.6,101.6);checks=[];minBroad=1e6
for side,solids in scenes.items():
 for body,part in solids.items():
  b=bounds(part)
  for i,s in enumerate(ss):
   lower=box_gap(b,bounds(s))
   if lower>5:
    minBroad=min(lower,minBroad);continue
   dist=distance(part,s)
   checks.append(dict(group=side,id=body,segment=i,gapMm=dist,overlapMm3=common(part,s) if dist<.0001 else 0.))
result=dict(authority=authority,outline=outline,zMm=[101.6,203.2],stockNominalMm=[25.4,101.6,3.175],worstTestedGapMm=min(v['gapMm'] for v in checks),unexaminedPairsLowerBoundMm=minBroad,checked=checks,allPairCount=484*6,scope='Parked finite proposed 4 by 1 outer stock envelopes; rear corner placements projected. Inner fasteners, welds/splices, moving tool paths, bumper backing and swerve motion remain separate.')
(d/'chassis/full-height-perimeter.json').write_text(json.dumps(result,indent=2)+'\n')
print(result['worstTestedGapMm'],minBroad,len(checks),[(x['id'],x['overlapMm3']) for x in checks if x['overlapMm3']>.01])
