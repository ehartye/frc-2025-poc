"""Reopen the two affected back plates and the actual four host posts at the motor corners."""
from pathlib import Path
import sys
import numpy as np
from frame import j,ROOT,register,HERE
from module_placement import read,write
from envelopes import load_raw
from fit import indexed
def run(art):
    raw=load_raw(art)[1][1]
    base=indexed(art)
    guides_meta=read(ROOT/"fixed-camera-route/guides.json")
    guides=j.cad.read_named(ROOT/"fixed-camera-route/harness-guides.step",[{"id":k} for k in guides_meta["volumeMm3"]])
    results={}
    for tag,panel in (("RL","electrical-backplate"),("RR","observer-pi-and-converter-backplate")):
        motor=register(raw,tag);br=j.cad.bounds(motor);p=base[panel]
        hits={}
        for k,s in guides.items():
            if j.lower_bound(br,j.cad.bounds(s))<.1:
                hits[k]=[j.cad.intersection(motor,s),j.distance(motor,s)]
        results[tag]={"backplate":panel,"motorBoundsMm":br,"backplateBoundsMm":j.cad.bounds(p),
                      "plateOverlapMm3":j.cad.intersection(motor,p),"hardwareNative":hits}
    write(HERE/"backplate-control.json",results)
if __name__=="__main__":run(Path(sys.argv[1]))
