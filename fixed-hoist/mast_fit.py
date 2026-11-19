"""Check the short shaft against the direct mast joints and their complete removal sweep."""
import argparse
from pathlib import Path
from fit import ENDS, HERE, X, Z, box_from_bounds, cross, j


def run(folder):
    study = j.read(folder/"fit.json")
    file = folder/"mast-joints.step"
    if study["stepSha256"]!=j.digest(file) or study["extractionJogXmm"]!={"L":0.,"R":0.}:
        raise ValueError("Expected the direct two-stage mast-splice removal")
    source = j.cad.read_named(file,[{"id":k} for k in study["volumeMm3"]])
    ref = j.read(HERE/"fit.json")
    shaft_file = HERE/"stationary-shaft.step"
    if ref["stepSha256"]!=j.digest(shaft_file):
        raise ValueError("Shaft exchange changed")
    kit = j.cad.read_named(shaft_file,[{"id":k} for k in ref["volumeMm3"]])
    kit_clearances = {k:j.prove(j.clearance(v,source)) for k,v in kit.items()}
    service = {}
    for side,sign in ENDS:
        b = j.cad.bounds(source["mast-base-"+side+"-splice"])
        x,y,z,xx,yy,zz = b
        # One enclosure covers the entire 35 mm lift plus the 220 mm outboard pull.
        envelope = box_from_bounds([x,y+min(0,sign*220),z,xx,yy+max(0,sign*220),zz+35])
        service[side+"-splice"] = j.prove(j.clearance(envelope,kit))
        sweep = j.cad.brep(["hex",[X,-280+min(0,sign*750),Z,12.7,1310]])
        service[side+"-shaftPull"] = j.prove(j.clearance(sweep,source))
        for key,tool in {"driver":cross(sign*274,X-65,X-14,4.5),
                         "nutSocket":cross(sign*274,X+10,X+36,5.5),
                         "screwWithdrawal":cross(sign*274,X-48,X+20,3.5)}.items():
            service[side+"-"+key] = j.prove(j.clearance(tool,source))
    j.write(HERE/"mast-fit.json",{"mastStepSha256":study["stepSha256"],"shaftStepSha256":ref["stepSha256"],
        "mastPartsInExchange":len(source),"installed":kit_clearances,"service":service})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mast",type=Path)
    run(parser.parse_args().mast)
