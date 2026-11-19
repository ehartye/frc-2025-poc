"""Classify the selected physical BOM, keeping reused components distinct."""
import json
from pathlib import Path
import sys
from module_placement import HERE,read,write

RETIRE=("axle-","bearing-","carrier-","center-","chain-","clip-","front-","gearbox-",
        "hub-","jack-","keeper-M3-","light/inner-","rear-","repacked-light/",
        "retaining-screw-","select-shim-","shop-L-REV-21-21","shop-R-REV-21-21",
        "spacer-front-","spacer-rear-","spacer-center-","sprocket-","supported-covers/",
        "takeup-","unmodeled-wheel-to-hub-screw-","wheel-")
def run(account):
    inp=read(account)
    ids=sorted(inp["make"]+inp["undecided"]+[v for d in inp["buy"] for v in d["instances"]])
    if len(ids)!=len(set(ids)) or len(ids)!=1199:raise ValueError("Expected captured-rail source register")
    retired=[k for k in ids if k.startswith(RETIRE)]
    reused=[k for k in ids if k in {"motor-L","motor-R","drive-controller-L","drive-controller-R"}]
    moved=[k for k in ids if k.startswith("inner-rail-joint/")]
    return {"sourceCount":len(ids),"retired":retired,"sameDriveMotorsAndControllers":reused,
            "relocatedInnerEarSet":moved,"retainedOtherCount":len(ids)-len(retired)-len(reused)-len(moved),
            "scope":"Physical identities. Booked dollars or mass are separate from this list."}
if __name__=="__main__":write(HERE/"retirement.json",run(Path(sys.argv[1])))
