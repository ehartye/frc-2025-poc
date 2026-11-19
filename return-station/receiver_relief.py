"""Bring the earlier physically drawn relief kit into the actual zero-stroke 560 mm pose."""
import json
from registration import ROOT,mount,sha,model


def low_relief(lamp,stroke=0):
    shapes=model(lamp,stroke)
    folder=ROOT/"station-relief"
    cheeks=json.loads((folder/"geometry.json").read_bytes());shims=json.loads((folder/"shims.json").read_bytes())
    if sha(folder/"cheeks.step")!=cheeks["reopenedStepSha256"] or sha(folder/"shims.step")!=shims["nativeSha256"]:raise ValueError("Changed relief hardware")
    names=list(shims["parts"])+list(shims["stopChecks"])+[k+"-seat-shim" for k in shims["parts"]]
    for p,ids in ((folder/"cheeks.step",list(cheeks["stock"])),(folder/"shims.step",names)):
        for k,s in mount.cad.read_named(p,[{"id":i} for i in ids]).items():
            # These receiver/cartridge parts all travel with the full inner slide.
            shapes[k]=mount.cad.translated(s,(stroke-270,0,-380))
    if len(shapes)!=597:raise ValueError("Unexpected relief inventory")
    return shapes
