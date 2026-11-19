"""Compute the elastic stress demand at one idealized rail-end weld group."""
import json
from itertools import product
from pathlib import Path

import numpy as np


def screen(force_n, throat_mm):
    # Centroid at (340, 325, 100). Local x/z span the weld plane.
    width, height = 20., 100.
    area = 2*(width+height)*throat_mm
    ix = throat_mm*(width*height**2/2+height**3/6)
    iz = throat_mm*(height*width**2/2+width**3/6)
    polar = ix+iz
    best = {"equivalentMPa":0}
    for px,py,pz in product((350,425),(350,425),(42.5,181.5)):
        rx,ry,rz = px-340,py-325,pz-100
        for x,z in product((-width/2,width/2),(-height/2,height/2)):
            # Columns give the normal and shear response to a unit X or Y force.
            normal = np.array([-x*ry/iz, 1/area+z*rz/ix+x*rx/iz])
            taux = np.array([1/area+z*rz/polar,0])
            tauz = np.array([-x*rz/polar,0])
            matrix = np.stack([normal,3**.5*taux,3**.5*tauz])
            _,s,v = np.linalg.svd(matrix,full_matrices=False)
            if s[0]*force_n > best["equivalentMPa"]:
                best = {"equivalentMPa":s[0]*force_n,"referencePointMm":[px,py,pz],
                        "groupCornerMm":[340+x,325,100+z],"unitHeadingXY":v[0].tolist(),
                        "normalAndTwoShearsMPa":(np.stack([normal,taux,tauz])@v[0]*force_n).tolist()}
    return {"forceN":force_n,"assumedEffectiveThroatMm":throat_mm,"areaMm2":area,
            "IxMm4":ix,"IzMm4":iz,"worst":best}


if __name__=="__main__":
    out=Path(__file__).with_name("load-demand.json")
    cases=[screen(force,throat) for force in (1000,2000) for throat in (1.4,1.)]
    out.write_text(json.dumps(cases,indent=2)+"\n",encoding="utf-8",newline="\n")
