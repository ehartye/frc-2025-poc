"""Bound the full azimuth revolution, including rotating wheel hardware and V1.1 changes."""
import itertools,math,sys
from pathlib import Path
import numpy as np
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape
from frame import HERE,SITES,register,cylinder,j
from registration import read,write,rotated_box
from native_module import PIN

ROTATING=set([11,12,13,14,15,16,17,18,23,*range(32,47),*range(49,55),*range(58,63)])
def load_raw(path):
    data=read(path/"original.json")
    cache=path/"vendor-native"
    if (cache/"source.sha256").read_text().strip()!=PIN:raise ValueError("Wrong supplier reference")
    result=[]
    for key,meta in data.items():
        name=key.split("/",1)[1].replace("/","_").replace(":","-")
        s=TopoDS_Shape()
        if not BRepTools.Read_s(s,str(cache/(name+".brep")),BRep_Builder()):raise ValueError(name)
        result.append((name,s,np.array(meta["boundsMm"])))
    if len(result)!=72:raise ValueError("Supplier occurrence count changed")
    return result

def corner_data(path):
    raw=load_raw(path);native={};bounds={};rad={}
    for i,(name,s,b) in enumerate(raw):
        if i not in ROTATING:
            native[name]=s
            continue
        rx=max(abs(b[0]),abs(b[3]));rz=max(abs(b[2]),abs(b[5]))
        full=math.hypot(rx,rz)+.01
        if full>47.4:
            # The analytic trimmed solid must fit wholly inside the vertical OCCT cylinder.
            trial=cylinder([0,b[1]-1,0],[0,1,0],47.4,b[4]-b[1]+2)
            excess=j.cad.volume(j.cut(s,trial))[0]
            if excess>.001:raise ValueError("Azimuth radius too small: "+name+" "+str(excess))
            full=47.4
        rad[name]=float(full)
        native[name]=cylinder([0,b[1],0],[0,1,0],full,b[4]-b[1])
    # The newer source bridge fits its original shaft/hole pattern. Reserve its entire bounding blank.
    native["V1.1-P26-bridge-blank"]=j.box(-15.875,15.875,40.95,71.05,-28.58,28.58)
    native["V1.1-P24-P25-shaft-key-reserve"]=cylinder([0,41,0],[0,1,0],9.6,29.8)
    native["V2-wheel-full-azimuth"]=cylinder([0,-38.1,0],[0,1,0],math.hypot(38.1,12.8),76.2)
    for site in SITES:
        for name,s in native.items():
            bounds[site+"/"+name]=[round(float(a),5) for a in rotated_box(j.cad.bounds(s),site)]
    return native,bounds,rad
def run(path):
    native,boxes,verified=corner_data(path)
    frame=read(HERE/"frame.json")
    write(HERE/"envelopes.json",{"sourceStepSha256":PIN,"frameStepSha256":frame["stepSha256"],
          "cornerBoundsMm":boxes,"frameBoundsMm":{k:v["boundsMm"] for k,v in frame["parts"].items()},
          "movingRadiusMm":verified,"sweep":"Continuous full rotation and conservative wheel spin. Exact BRep clipping caps the largest turning pieces."})
if __name__=="__main__":run(Path(sys.argv[1]))
