"""Screen both relocated quarter-twenty pairs with their simultaneous upper reaction history."""
import json, math, sys
from pathlib import Path
import numpy as np
from inset_pan_truss import run as truss,HERE

def run(pkg):
    plan=json.loads((HERE/"inset-corrected-brace-foot.json").read_bytes())
    cases={-1:[],1:[]}
    def record(event,node,force):
        x,y,z=node
        if abs(x-250)>.001 or abs(abs(y)-145)>.001:return
        sign=1 if y>0 else -1
        # The modeled axial truss joints land centrally. Bound the distance to the
        # bottom of the fitted boss to include the 6.35 mm foot and full boss.
        offset=6.35+2.95
        fx,fy,fz=force
        moment=np.cross([0,0,offset], force)
        # Conservative single-bolt tension envelope incl. an opposite compressive seat,
        # taking only 13 mm of available edge support in either planar direction.
        tension=max(0,fz)+sum(abs(moment[:2]))/13.
        # No friction credit. Independent planar load components bear on finite boss flats.
        # Corner radii 1.5 mm each end removed from either loaded face.
        foot=plan["stockChanges"][f"Truss landing 250 {int(y)}"]
        boss=foot["steps"][1]["profiles"][0];a,b,A,B=boss["bounds"];r=boss["radius"]
        stress=[abs(fx)/((B-b-2*r)*2.95),abs(fy)/((A-a-2*r)*2.95)]
        cases[sign].append(dict(event=event,station=node.tolist(),reactionN=force.tolist(),baseMomentNmm=moment.tolist(),
                 upperTensionOneBoltN=tension,bossBearingMPa=stress,fullAxialAreaAtBoltMPa=tension/20.48))
    review=truss(pkg,record)
    if plan["authority"]!=review["authority"] or any(len(rows)!=review["events"] for rows in cases.values()):raise ValueError("Incomplete signed reactions")
    return dict(authority=review["authority"],source=review["inputs"],eventsPerStation=review["events"],
                stations={str(sign):{key:max(rows,key=val) for key,val in {
                     "combined":lambda row:row["upperTensionOneBoltN"],"xBearing":lambda row:row["bossBearingMPa"][0],
                     "yBearing":lambda row:row["bossBearingMPa"][1],"uplift":lambda row:row["reactionN"][2],
                     "compression":lambda row:-row["reactionN"][2]}.items()} for sign,rows in cases.items()},
                limits="Bounds are conditional on holding a compression patch at least 13 mm onto the opposite side of the tension line and direct contact on the fitted key, plus the published twofold upper reaction assumption. Nominal 6.35 mm female engagement precedes runout allowance. Full thread stripping, cyclic preload, seat stiffness, beam roof deflection, welded attachment and asymmetric contact still need separate checks.")
if __name__=="__main__":
    d=run(Path(sys.argv[1]));(HERE/"inset-corrected-foot-loads.json").write_text(json.dumps(d,indent=1)+"\n")
