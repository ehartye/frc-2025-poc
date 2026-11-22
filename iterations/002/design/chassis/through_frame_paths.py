"""Screen two directly opposed load members against the canonical front pair."""
import sys,json
from pathlib import Path
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt
root=Path.cwd();d=root/'iterations/002/design'
sys.path.insert(0,str(Path(sys.argv[1])/'scripts'))
from fusion_authority import register_path,verify,load,resolve
from showcase_contacts import load_solids,box_gap
from mechanical_motion import bounds,distance,common
reg=register_path(root);lin=reg.parent/'issue-002/lineage.json';issue=verify(root,reg,lin)
parts,sheets=load_solids(resolve(root,load(lin)['step']))
if len(parts)!=244 or sheets:raise ValueError('Unexpected authoritative issue')
boxes={k:bounds(v) for k,v in parts.items()};result={}
for datum in (150.,100.,0.):
 for label,axis,half in [('cross',0,339.6),('sill',1,344.6)]:
  for sign in [1,-1] if datum else [1]:
   center=sign*datum
   lo=[-half,-half,101.6];hi=[half,half,152.4];lo[axis]=center-12.7;hi[axis]=center+12.7
   shape=BRepPrimAPI_MakeBox(gp_Pnt(*lo),gp_Pnt(*hi)).Shape();bb=bounds(shape)
   tested={};hits={}
   for k,v in parts.items():
    gap=box_gap(boxes[k],bb)
    if gap>2:continue
    # Interior solids are simple uncut candidates; any penetration demands a further study.
    before=distance(v,shape);tested[k]=before
    if before<.0001:
     overlap=common(shape,v)
     if overlap>.01:hits[k]=overlap
   result[f'{label}-{center}']=dict(boxMm=[lo,hi],overlapsMm3=hits,testedDistancesMm=tested)
(d/'chassis/through-frame-paths.json').write_text(json.dumps(dict(authority=issue,proposed=result),indent=2)+'\n')
for k,r in result.items():print(k,len(r['overlapsMm3']),max(r['overlapsMm3'].values(),default=0))
