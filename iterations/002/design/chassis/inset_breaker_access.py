"""Locate the supplier's red CB185 push-to-trip surface and physically approach it in the 002 frame."""
from pathlib import Path
import json,sys,hashlib
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from crossmember_envelope import cylinder
from inset_electrical_package import CAD,gap,box_corners,bb
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,sys.argv[1]+"/scripts")
from mechanical_motion import distance
e=json.loads((HERE/"inset-electrical-package.json").read_bytes());c=json.loads((HERE/"inset-breaker-color.json").read_bytes())
red=[i for i,row in enumerate(c) if row["redFaceBounds"]]
if red!=[0]:raise ValueError("Expected exactly one source red switch surface")
src=CAD/"am-0282-CB185-120.step";reader=STEPControl_Reader();reader.ReadFile(str(src));reader.TransferRoots()
sh=[];it=TopExp_Explorer(reader.OneShape(),TopAbs_SHELL)
while it.More():sh.append(it.Current());it.Next()
if len(sh)!=16:raise ValueError("Incomplete actual switch source")
rb=c[0]["bounds"];target=[(rb[0]+rb[3])/2,rb[4],(rb[2]+rb[5])/2]
m=np.array(e["candidatePlacements"]["Main 120 A breaker"]["worldMatrixMm"]);world=m[:,:3]@target+m[:,3]
out={}
for trial in (4.,0.,-1.,-3.):
    start=[target[0],target[1]+trial,target[2]]
    probe=cylinder(start,(0,1,0),6.,65.)
    d=[distance(probe,s) for s in sh]
    if trial==0 and d[0]>.0001:raise ValueError("Finger did not touch the red switch")
    out[str(trial)]=dict(redSurfaceGapMm=d[0],other15NearestGapMm=min(d[1:]),otherNearestShell=2+int(np.argmin(d[1:])))
# Narrow downward passage is contained within the already tested complete overhead hand corridor
# except for the final contact location; check its exterior clearance independently.
envelope=[world[0]-6,world[1]-6,world[2]-3,world[0]+6,world[1]+6,720.]
sources={k:r["boundsMm"] for k,r in e["candidatePlacements"].items() if k!="Main 120 A breaker"}
near=min((gap(envelope,v),k) for k,v in sources.items())
d=dict(authority=e["authority"],supplierShellSha256=hashlib.sha256(src.read_bytes()).hexdigest(),
    sourceColorReadingSha256=hashlib.sha256((HERE/"inset-breaker-color.json").read_bytes()).hexdigest(),redShell=1,colorSource="Ten red supplier surface faces on one body; Eaton 5084 on the CB185 flush-mounted push-to-trip function.",
    targetLocalMm=target,targetWorldMm=world.tolist(),fingerRadiusMm=6,signedRelativeToFirstContact=out,
    handRoom=e["service"]["full breaker above-hand corridor"],finalFingerVsOtherDevicesMinMm=near[0],nearestDevice=near[1],
    scope="Demonstrates finger access to the actual red supplier surface from above the selected side bay, with a 3 mm inward geometric allowance before checking the remaining source shells. Supplier publication does not establish trip travel, required force or electrical function; neither installed sense nor a complete rotating pan/bumper/wired/fused assembly is claimed.")
(HERE/"inset-breaker-access.json").write_text(json.dumps(d,indent=2)+"\n")
print(world,out,near)

