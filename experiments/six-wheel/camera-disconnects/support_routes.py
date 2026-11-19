"""Check cable jacket sweep against its own guide and the opposite line's support hardware."""
import json
from pathlib import Path
import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.gp import gp_Pnt
from connector_fit import bounds, distance, HERE
from routes import read, values, derivative_bound
from support import design

def audit():
    kit,_,_=design()
    controls=read(HERE/"route-controls.json")
    proof={}
    for key,r in (("data",3.15),("power",4.30)):
        p=controls[key]; speed=derivative_bound(p,1)
        # The last section after the stripped jacket on the heavy cable contains two separate insulated leads.
        end=1.0 if key=="data" else .978
        low=10.;fail=[];unresolved=[]
        for name,part in kit.items():
            box=np.array(bounds(part));stack=[(0.,end)]
            while stack:
                t0,t1=stack.pop();mid=(t0+t1)/2;pos=values(p,mid)[0];travel=(t1-t0)*speed/2
                box_gap=np.linalg.norm(np.maximum(0,np.maximum(box[:3]-pos,pos-box[3:])))
                if box_gap-r-travel>2:continue
                dist=distance(BRepBuilderAPI_MakeVertex(gp_Pnt(*pos)).Vertex(),part)
                if dist<r-1e-4:
                    fail.append([name,mid,dist-r]);break
                lower=dist-r-travel
                if lower>.01:
                    low=min(low,lower);continue
                if t1-t0<.00002:
                    unresolved.append([name,mid,lower]);break
                stack.extend([(t0,mid),(mid,t1)])
        proof[key]={"radius_mm":r,"jacket_last_t":end,"penetrations_mm":fail,
                    "uncertain_intervals":unresolved,"minimum_interval_clearance_mm":low}
    return proof

if __name__=="__main__":
    (HERE/"support-routes.json").write_text(json.dumps(audit(),indent=2)+"\n",encoding="utf8",newline="\n")
