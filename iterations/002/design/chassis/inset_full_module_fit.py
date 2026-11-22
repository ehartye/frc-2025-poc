"""Find installed-geometry conflicts between the inset commercial corner projections and proposed chassis stock."""
from pathlib import Path
import json,sys,re
import numpy as np
from inset_stock_check import HERE,run as stock
from four_corner_packaging import rotated


def evaluate(pkg):
    parts,report=stock(pkg,True)
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import bounds,common
    from fusion_authority import register_path
    master=register_path(HERE.parents[3]).parent/"issue-005/assembly.step"
    installed,sheets=load_solids(master)
    if len(installed)!=552 or sheets:raise ValueError("Wrong canonical input")
    groups={1:(1,1),4:(1,-1),6:(-1,-1),8:(-1,1)}
    selected={}
    for name,shape in installed.items():
        m=re.search(r"\[1\.(\d+)\.1\.",name)
        if m and int(m[1]) in groups:
            sx,sy=groups[int(m[1])];p=rotated(shape,0,(-12.7*sx,-12.7*sy,0))
            selected[name]=(p,bounds(p))
    if len(selected)!=484:raise ValueError("Incomplete genuine donor set")
    checks={};ledger=HERE/"inset-stock-module-inventory.json"
    for name,row in report["parts"].items():
        exact=[]
        for subject,(s,b) in selected.items():
            if box_gap(b,row["bounds"])<=.01:
                exact.append(dict(subject=subject,commonMm3=common(s,parts[name])))
        checks[name]=dict(boundsSeparated=len(selected)-len(exact),tested=exact)
        ledger.write_text(json.dumps(dict(authority=report["authority"],stockSha256=report["shopPlanSHA256"],expectedStocks=len(parts),finishedStocks=len(checks),installedModuleBodies=484,checks=checks),indent=1)+"\n")
    print([(n,q["subject"],round(q["commonMm3"],3)) for n,r in checks.items() for q in r["tested"] if q["commonMm3"]>.01])
if __name__=="__main__":evaluate(Path(sys.argv[1]))
