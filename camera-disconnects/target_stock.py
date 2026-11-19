"""Re-evaluate the matched reef-ball constraint with connector bodies and the real cable curves."""
import json
import sys
import numpy as np
from connector_fit import HERE,ROOT
from routes import read,values,derivative_bound

sys.path.insert(0,str(ROOT/"bridge-tolerance"))
from field import robust

def report():
    bodies={n:row["box_mm"] for n,row in read(HERE/"support.json")["parts"].items()}
    bodies["registered-camera-case"]=[29.5,-45.5,842.05,57.5,45.5,859.5]
    bodies["Y-case-and-output"]=[19.5,-28.5,841.3,39,-2.6,854.25]
    bodies["data-plug-and-backed-out-studs"]=[-13,-27.1,837.4,25.1,-20.0,858.2]
    bodies["power-plug-and-selected-tail"]=[-53,-11.4,841.35,26,-3.7,854.2]
    p=read(HERE/"route-controls.json")
    for key,rad in (("data",3.15),("power",4.30)):
        N=256;centers=values(p[key],(np.arange(N)+.5)/N)
        err=rad+derivative_bound(p[key],1)/(2*N)
        for i,center in enumerate(centers):bodies[f"{key}-{i}"]=np.r_[center-err,center+err]
    xyz=read(ROOT/"full-match/installed-stock.json")["stock"]["NAUO201"]["native_center_mm"]
    prev=read(ROOT/"camera-datum-shop/fit.json")["ball_tolerance"]
    flex=read(ROOT/"camera-datum-shop/mechanics.json")["derived_2g"]["summed_tip_mm"]
    out={}
    for sector in (-180,220):
        point=(xyz[0]-(3453.050980729168-490),-xyz[2]-sector,xyz[1])
        gap,owner=min((robust(b,point,flex),k) for k,b in bodies.items())
        out[sector]={"new_hardware_lower_mm":gap,"limiting_new_body":owner,
                     "retained_lower_mm":prev[str(sector)]["combined_lower_mm"],
                     "combined_lower_mm":min(gap,prev[str(sector)]["combined_lower_mm"])}
    return out

if __name__=="__main__":
    (HERE/"target-stock.json").write_text(json.dumps(report(),indent=2)+"\n",encoding="utf8",newline="\n")
