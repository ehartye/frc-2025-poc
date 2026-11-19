"""Rerun the registered corrected-wheel station delivery with the candidate section."""
import argparse
import importlib.util
import inspect
import json
from pathlib import Path
import sys

from cam_model import activate
from loaded_rack import HERE, ROOT, digest


def run(project,jdk,drive,raw,sign,step,gap):
    receipt=activate()
    motion=project/"experiments/swerve-drive/motion"
    spec=importlib.util.spec_from_file_location("registered_swerve_feed",motion/"feed.py")
    feed=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(feed)
    cls=feed.engine.coupled.CoupledPipe
    original=cls.__init__
    signature=inspect.signature(original)
    def initializer(*args,**kwargs):
        parameters=signature.bind(*args,**kwargs)
        parameters.arguments["running_gap"]=gap/1000
        original(*parameters.args,**parameters.kwargs)
    cls.__init__=initializer
    target=raw/f"cam-station-{sign}-{step:g}us-{gap:g}mm"
    out=feed.run(jdk,drive,target,sign,step,False,motion/"energy.json")
    full,=target.glob(f"{sign}/*/result.json")
    detail=json.loads(full.read_bytes())
    out.update(candidateStepSha256=receipt["candidateStepSha256"],capGapMm=gap,
               sectionScriptSha256=digest(HERE/"cam_model.py"),sourceControllerSha256=digest(motion/"feed.py"),
               registeredPlateTable=detail["shelfDisplacementSlopeTable"],controlTransitions=detail["portTransitions"],
               loss=detail["loss"],hardStop=detail["hardStop"],reactionPeakN=detail["peakShelfNormalN"])
    (raw/(target.name+".json")).write_text(json.dumps(out,indent=2,allow_nan=False)+"\n",encoding="utf8")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("jdk","drive","raw"):
        parser.add_argument(name,type=Path)
    parser.add_argument("sign",choices=("plus","minus"))
    parser.add_argument("step",type=float)
    parser.add_argument("gap",type=float)
    parser.add_argument("--project",type=Path,default=ROOT)
    args=parser.parse_args()
    run(args.project,args.jdk,args.drive,args.raw,args.sign,args.step,args.gap)
