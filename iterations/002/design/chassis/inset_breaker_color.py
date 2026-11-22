from pathlib import Path
from OCP.XCAFDoc import XCAFDoc_DocumentTool,XCAFDoc_ColorType
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_SHELL
from OCP.Quantity import Quantity_Color
from OCP.collections import Sequence_TDF_Label as Seq
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
import sys,json
from inset_electrical_package import CAD,bb
sys.path.insert(0,sys.argv[1]+"/scripts")
from mechanical_motion import bounds
f=CAD/"am-0282-CB185-120.step";d=TDocStd_Document(TCollection_ExtendedString("BinXCAF"));r=STEPCAFControl_Reader();r.SetColorMode(True);r.ReadFile(str(f));r.Transfer(d)
table=XCAFDoc_DocumentTool.ShapeTool_s(d.Main());color=XCAFDoc_DocumentTool.ColorTool_s(d.Main());seq=Seq();table.GetFreeShapes(seq)
a=table.GetShape_s(seq.Value(1));it=TopExp_Explorer(a,TopAbs_SHELL);out=[]
while it.More():
    shell=it.Current();it.Next();faces=TopExp_Explorer(shell,TopAbs_FACE);colors={};red=[]
    while faces.More():
        face=faces.Current();faces.Next(); c=Quantity_Color()
        if not color.GetColor(face,XCAFDoc_ColorType.XCAFDoc_ColorSurf,c):continue
        rgb=(c.Red(),c.Green(),c.Blue());k=str(tuple(round(x,3) for x in rgb));colors[k]=colors.get(k,0)+1
        if rgb[0]>3*max(rgb[1],rgb[2]):red.append(bounds(face))
    out.append(dict(bounds=bounds(shell),colors=colors,redFaceBounds=red))
p=Path(__file__).resolve().parent/"inset-breaker-color.json";p.write_text(json.dumps(out,indent=2)+"\n")
for i,v in enumerate(out):
    if v["redFaceBounds"]:print(i+1,v["colors"],bb([p for a in v['redFaceBounds'] for p in [a[:3],a[3:]]]))

