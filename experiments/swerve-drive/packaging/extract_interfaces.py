"""Identify the tab bores and classify the installed module's wheel and azimuth axes."""
from pathlib import Path
import json,sys
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRepTools import BRepTools
from native_module import cad,read,sha

def bores(shape):
    out=[]
    faces=TopExp_Explorer(shape,TopAbs_FACE)
    while faces.More():
        s=BRepAdaptor_Surface(TopoDS.Face(faces.Current()));faces.Next()
        if s.GetType()!=GeomAbs_Cylinder:continue
        c=s.Cylinder();ax=c.Axis();u=ax.Direction().Coord()
        if abs(u[1])<.999999:continue
        p=ax.Location().Coord()
        out.append([round(p[0],4),round(p[2],4),round(c.Radius(),4)])
    return sorted(set(map(tuple,out)))
def build(path,out):
    shapes=read(path)
    out.mkdir(parents=True,exist_ok=True)
    for key,s in shapes.items():
        num=key.split("/")[1].split(":")[0]
        if num in ["9","10","28"]:
            print(num,bores(s))
        # The owner supplies these commercial geometries separately; cache locally only.
        BRepTools.Write_s(s,str(out/(key.split("/",1)[1].replace("/","_").replace(":","-")+".brep")))
    (out/"source.sha256").write_text(sha(path))
if __name__=="__main__":build(Path(sys.argv[1]),Path(sys.argv[2]))
