"""Original unrounded trough-floor patch contact fixtures, NOT installed REEF geometry."""
import pathlib,json,hashlib,sys,OCP
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2,gp_Vec
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism,BRepPrimAPI_MakeCylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Common
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
ROOT=pathlib.Path(__file__).resolve().parents[1]
PINS=['scripts/coral-trough-part-study.py','case-studies/target-frame-evidence.json','case-studies/2025/coral-fixture-nominal.json']
def report():
 e=json.loads((ROOT/PINS[1]).read_text())['coralFloor'];polygon=BRepBuilderAPI_MakePolygon()
 for x,y in e['verticesInch']:polygon.Add(gp_Pnt(x*.0254,y*.0254,0))
 polygon.Close();face=BRepBuilderAPI_MakeFace(polygon.Wire()).Face();floor=BRepPrimAPI_MakePrism(face,gp_Vec(0,0,-e['thicknessInch']*.0254)).Shape();rows=[]
 geom=json.loads((ROOT/PINS[2]).read_text())['pieceSource']['geometryMm'];length=geom['length']/1000;ro=geom['outerDiameter']/2000;ri=geom['innerDiameter']/2000
 for u in [.2,.45,.6]:
  for gap in [.01,0,-.001]:
   axis=gp_Ax2(gp_Pnt(u-length/2,.08,ro+gap),gp_Dir(1,0,0));tube=BRepAlgoAPI_Cut(BRepPrimAPI_MakeCylinder(axis,ro,length).Shape(),BRepPrimAPI_MakeCylinder(axis,ri,length).Shape()).Shape();d=BRepExtrema_DistShapeShape(tube,floor);d.Perform()
   if not d.IsDone():raise ValueError('native distance failed')
   op=BRepAlgoAPI_Common(tube,floor);op.Build()
   if not op.IsDone():raise ValueError('native common failed')
   props=GProp_GProps();BRepGProp.VolumeProperties_s(op.Shape(),props)
   rows.append(dict(season=2025,frame='TROUGH_FLOOR_PART_LOCAL_NOT_INSTALLED',centerU_M=u,centerV_M=.08,assumedGapM=gap,nativeDistanceM=d.Value(),nativeIntersectionM3=props.Mass(),robotContact=None,otherLevelScored=None,installedTroughContact=None,nominalComponentPoints=None))
 return dict(schema='frc-coral-trough-part-study/v1',cases=rows,sourceCodeSha256={p:hashlib.sha256((ROOT/p).read_text(encoding='utf-8-sig').encode()).hexdigest() for p in PINS},runtime=dict(python=sys.version,ocp=OCP.__version__),limitations=['Original nominal unrounded part patch, not vendor native CAD','Bolt holes and corner radii omitted; positive overlap is obstruction/compression witness, not load','No assembled floor/bracket/back/lip transform established','Static piece placement, no dynamics or verified robot separation','No conversion from tangent/overlap fixture to scored L1 contact'],engineeringAcceptance='blocked')
if __name__=='__main__':
 import argparse
 a=argparse.ArgumentParser();a.add_argument('--output',required=True);p=pathlib.Path(a.parse_args().output)
 if p.exists():raise ValueError('fresh output required')
 r=report();p.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n',encoding='utf-8');print(r['cases'])
