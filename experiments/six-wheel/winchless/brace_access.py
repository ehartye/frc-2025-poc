"""Check the two direct carriage brackets and service tools with all pit stands installed."""
import argparse
import importlib.util
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent


def run(carriage,fixture):
    sys.path.insert(0,str(carriage.resolve()))
    spec=importlib.util.spec_from_file_location("carriage_bracket_fit",carriage/"fit.py")
    braces=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(braces)
    j=braces.j
    data=j.read(fixture/"fit.json")
    if (j.digest(fixture/"service-stands.step")!=data["stepSha256"] or
            j.digest(carriage/"carriage-support.step")!=data["carriageSupportSha256"]):
        raise ValueError("The fixture and bracket exchanges do not match")
    stands=j.cad.read_named(fixture/"service-stands.step",[{"id":k} for k in data["parts"]])
    results={}
    for sign,side in ((1,"L"),(-1,"R")):
        tools=braces.extraction_envelopes(sign)
        for i,(x,z) in enumerate(braces.BOLTS):
            tools[f"mast-fastener-{i}"]=j.rod(x,sign,151,266,z,7)
        for i,y in enumerate(braces.Y_FRONT):
            tools[f"front-fastener-{i}"]=braces.zcyl(braces.FRONT,sign*y,397,6.4,68)
            tools[f"front-key-{i}"]=braces.xcyl(braces.FRONT,sign*y,460,3.2,50)
        tools["rear-fastener"]=braces.zcyl(braces.REAR,sign*braces.Y_REAR,366,7,63)
        results[side]={key:j.prove(j.clearance(gauge,stands)) for key,gauge in tools.items()}
    j.write(fixture/"brace-access.json",{"standSha256":data["stepSha256"],
        "bracketSha256":data["carriageSupportSha256"],"bracketMethodSha256":j.digest(carriage/"fit.py"),
        "sweptMm":535,"clearances":results})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("carriage",type=Path)
    parser.add_argument("--fixture",type=Path,default=HERE)
    args=parser.parse_args()
    run(args.carriage,args.fixture)
