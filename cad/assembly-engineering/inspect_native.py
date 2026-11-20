"""Join independent Fusion world bounds / volume to STEP identities; expose each disagreement."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
from installed_audit import HERE, ROOT, sha, reopen
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
def bounds(shape):
    b=Bnd_Box();BRepBndLib.AddOptimal_s(shape,b,False,False);return (*b.CornerMin().Coord(),*b.CornerMax().Coord())
from clearance import base
def evaluate(report,output):
    obs=json.loads(report.read_bytes());step=HERE/"station-and-jack-interfaces.step"
    kit=base.named(step)
    manifest=json.loads((HERE/"interfaces.json").read_bytes())
    planned={r["id"]:r for r in csv.DictReader((HERE/"installed-bodies.csv").open(encoding="utf8"))}
    for removed in manifest["fixedHeight"]["removed"]:planned.pop(removed)
    expected={k:([float(r[a]) for a in ("xMin","yMin","zMin","xMax","yMax","zMax")],float(r["volumeMm3"])) for k,r in planned.items()}
    for k,_,_,_,_,shape in reopen():
        if k in expected:expected[k]=(list(bounds(shape)),expected[k][1])
    for k,shape in kit.items():expected[k]=(list(bounds(shape)),base.cad.volume(shape)[0])
    ids={hashlib.sha256(("team-1780-engineering-v1/"+k).encode()).hexdigest():k for k in expected}
    changes=[]
    all_errors=[]
    for r in obs["bodies"]:
        if r["instance"] is None:continue
        k=ids[r["identity"]]
        bb,v=expected[k]
        diffs=[a-b for a,b in zip(r["boundsMm"],bb,strict=True)]
        volume_error=r["volumeCm3"]*1000-v
        row=dict(id=k,axisDifferencesMm=[round(q,4) for q in diffs],volumeErrorMm3=round(volume_error,4),expectedVolumeMm3=round(v,3),nativeVolumeMm3=round(r["volumeCm3"]*1000,3))
        all_errors.append(row)
        if max(abs(q) for q in diffs)>.02 or abs(volume_error)>max(.05,.000005*v):changes.append(row)
    return dict(archiveSha256=obs["archiveSha256"],targetInterfaceSha256=sha(step),
                nativeTotal=obs["nativeBodyCount"],compared=len(all_errors),largestBoundErrorMm=max(max(abs(x) for x in r["axisDifferencesMm"]) for r in all_errors),
                mismatches=changes,rawKg=obs["rawBodyTotalKg"],rootKg=obs["rootMassKg"])
if __name__=="__main__":
    cli=argparse.ArgumentParser();cli.add_argument("observations",type=Path);cli.add_argument("output",type=Path)
    args=cli.parse_args();args.output.write_bytes((json.dumps(evaluate(args.observations,args.output),indent=2)+"\n").encode())

