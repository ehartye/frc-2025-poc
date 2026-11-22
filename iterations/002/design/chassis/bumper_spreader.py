"""Dry-fit plates that route bumper corner load to the two lower rail webs."""
from pathlib import Path
import argparse,json,sys
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec
from four_corner_packaging import polygon

def intersection(project,plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from fusion_authority import register_path,verify,resolve,load
    from mechanical_motion import common,bounds,distance
    from showcase_contacts import load_solids,box_gap
    authority=register_path(project);record=authority.parent/'issue-002/lineage.json'
    root=verify(project,authority,record)
    parts,sheets=load_solids(resolve(project,load(record)['step']))
    if len(parts)!=244 or sheets:raise ValueError('Expected the complete current two-module export')
    boxes={k:bounds(o) for k,o in parts.items()}
    sections={
       # The corner cap is a continuous planar shear panel, not a floating side bolt.
       # Bottom webs and upper plywood connection have independent interfaces.
       'full-gusset':[[370,365],[135,365],[135,318.275],[173,318.275],[323.275,168],[323.275,130],[370,130]],
       'outside-L':[[370,365],[135,365],[135,339.6],[344.6,339.6],[344.6,130],[370,130]],
       'separate-front-web':[[323.275,145],[370,145],[370,115],[323.275,115]],
       'separate-side-web':[[145,318.275],[145,365],[115,365],[115,318.275]]}
    result={}
    for label,outline in sections.items():
        for z in ([94.6333628767298,101.6,146.05] if label in ('full-gusset','outside-L') else [94.6333628767298]):
            height=3.175
            test=BRepPrimAPI_MakePrism(polygon([[*v,z] for v in outline]),gp_Vec(0,0,height)).Shape()
            bb=bounds(test);near={};overlap={}
            others=[(box_gap(bb,val),k) for k,val in boxes.items()];tested=0
            for lower,k in sorted(others):
                if lower>5:break
                vol=common(parts[k],test);tested+=1
                if vol>.005:overlap[k]=vol
                else:near[k]=distance(parts[k],test)
            result[f'{label}/{z:.3f}']=dict(xyOutlineMm=outline,zMm=z,thicknessMm=height,possible=tested,commonMm3=overlap,nearGapMm=near)
    return dict(authority=root,scope='Proposed front-left plates in native 244 frame. Includes actual two front module issues and the attached front and port stock. Second longitudinal tube, permanent attachment hardware and plywood mounting are absent from this trial.',runs=result)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('project',type=Path);p.add_argument('plugin',type=Path);a=p.parse_args()
    d=intersection(a.project.resolve(),a.plugin);(Path(__file__).parent/'bumper-spreader-clearance.json').write_text(json.dumps(d,indent=2)+'\n')
    for k,v in d['runs'].items():print(k,len(v['commonMm3']),sum(v['commonMm3'].values()))
