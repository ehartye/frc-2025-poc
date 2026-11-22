"""Compare service positions for the two obstructed pan brace screws on actual shop geometry."""
import json, sys
from pathlib import Path
import numpy as np
from inset_chassis_shop import HERE
from inset_stock_check import run as form
from port_anchor_fit import oriented
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt


def run(plugin):
    stock, base = form(plugin, True)
    from component_shape import volume
    from mechanical_motion import common, distance, bounds
    from showcase_contacts import load_solids, box_gap
    from fusion_authority import register_path
    register = register_path(HERE.parents[3])
    originals, _ = load_solids(register.parent/"issue-005/assembly.step")
    original = json.loads((HERE/"inset-mount-purchases.json").read_bytes())["originals"]["91251A539"]
    screw = originals[original["source"]]
    rows = []
    for sign in (-1,1):
        for new_x, inset in ((234,0),(234,8),(234,10),(234,12),(232,8),(232,10)):
            x,y,z=new_x,(145-inset)*sign,193.675
            placed=oriented(screw,original["underHeadCenterMm"],original["intoThreadUnit"],[x,y,z],[0,0,-1])
            # Includes the screw head and the entire space to lower a standard short straight bit.
            # The closed envelope deliberately overbounds a 1/4-inch hex shank and 3/16-inch driver.
            bit=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z+3.05),gp_Dir(0,0,1)),3.68,47).Shape()
            # Whole screw plus entry path along +Z. Its head's radius bounds the smaller thread.
            withdrawal=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z),gp_Dir(0,0,1)),4.7625,24).Shape()
            vals={}
            for label,shape in (("screw",placed),("bit",bit),("withdrawal",withdrawal)):
                b=bounds(shape);hits=[]
                for name,body in stock.items():
                    if "Pan truss" not in name and "Upper pan stringer" not in name:continue
                    g=box_gap(b,base["parts"][name]["bounds"])
                    if g>5:continue
                    hits.append(dict(name=name,commonMm3=common(body,shape),gapMm=distance(body,shape)))
                vals[label]=hits
            rows.append(dict(sign=sign,x=x,y=y,outerSeatEdgeReserveMm=min(x-226,274-x,16-abs(inset))-4.7625,
                 backupTapMinorEdgeLigamentMm=min(x-(250-21.8),250+21.8-x,15-abs(inset))-2.5527,tests=vals))
    return dict(authority=base["authority"],source=original,stockPlanSHA256=base["shopPlanSHA256"],rows=rows,
                scope="Initial local two-foot design screen. Three finite envelopes vs actual strut stock. The paired partner screw, current sheet access opening, final handheld torque head and other vehicle articles require comparison before issuing new holes.")

if __name__=="__main__":
    result=run(Path(sys.argv[1]));(HERE/"inset-truss-screw-options.json").write_text(json.dumps(result,indent=1)+"\n")
