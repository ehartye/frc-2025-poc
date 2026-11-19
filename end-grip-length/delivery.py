"""Resume finite-mass indexed jaws at the actual indicated low receiver seat."""
from dataclasses import replace
import argparse
import json
from pathlib import Path
import sys
from unittest.mock import patch
from grip_study import HERE,ROOT,Trial,contact_law,work,study
sys.path.insert(0,str(ROOT/"design/fixed-height"))
import handoff
import pipe_axial_contact as axial

def build(jdk):
    force=(1.97+.32)*4.4482216152605
    selected=[]
    for cut,piece in zip((-6.35,6.35),handoff.spin.DATA["identified_transfers"]):
        case=Trial(piece["id"],cut=cut,spring=force,cassette=True,open_until_ms=0,indexed_center=cut/2,pad=.5)
        row=work(jdk,[case])[0]
        initial=row["mechanical"].get("firstQualifiedJaws")
        for profile in ("HISTORICAL","RAMP_090"):
            history={"travel":[6.,-6.],"peakNormal":0.,"outerStop":False,"samples":[]}
            if not initial:raise ValueError("The new jaw never reached the indicated state")
            law=contact_law(replace(case,side=0,yaw=0),history,initial)
            with patch.object(axial.AxialContact,"normals",law):
                result=handoff.one(piece,profile,row["optical"])
            history.pop("samples")
            selected.append({"cutMm":cut,"jawState":initial,"history":history,"result":result})
    return selected
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--jdk",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps(build(a.jdk),indent=2)+"\n",encoding="utf-8",newline="\n")
