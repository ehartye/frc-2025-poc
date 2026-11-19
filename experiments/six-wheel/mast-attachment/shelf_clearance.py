"""Reopen the selected receiver and use a common vertical separating plane through the service sweep."""
import argparse
from pathlib import Path
from mast import HERE,j

def run(source):
    manufactured=j.read(source/"installed.json")
    step=source/"stow-receiver.step"
    if j.digest(step)!=manufactured["manufacturingSha256"]:raise ValueError("Different receiver revision")
    stock=j.cad.read_named(step,[{"id":k} for k in manufactured["volumeMm3"]])
    floor={k:j.cad.bounds(v)[2] for k,v in stock.items()}
    # Published 61.4 mm full supplier length; the selected tips are 0.10 below Z485.
    # Both KM8 collars flank the Z433..438 bridge and lie above the body bottom.
    for y in (-66,42):floor["MA30EUM-"+str(y)]=484.9-61.4
    # Highest shop service pose is the shortened splice Z229 plus its 35 mm lift.
    ceiling=264.
    j.write(HERE/"damper-clearance.json",{"sourceStepSha256":j.digest(step),"sourceFitSha256":j.digest(source/"damper-fit.json"),
            "minimumZmm":floor,"maximumToolOrMovingSpliceZmm":ceiling,
            "minimumSeparationMmThroughSlideStroke":min(floor.values())-ceiling,
            "slideMotion":"horizontal at the permanently locked receiver height","sourceSiteMeasured":False})
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("source",type=Path)
    run(p.parse_args().source)
