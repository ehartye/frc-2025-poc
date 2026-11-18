"""Verify the reopened exported design and the two independent CAD kernels' shelf witnesses."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
from build_assembly import PARTS,HERE,MODEL,read_named,bounds,volume


def budget(e):
    result={}
    for name,currents in e["busCurrentA"].items():
        if len(currents)!=len(e["columns"]) or min(currents)<0:
            raise ValueError("branch current inventory")
        amps=sum(currents)
        supply=e["openCircuitV"]-amps*(e["batteryOhm"]+e["trunkOhm"])
        result[name]={"busCurrentA":amps,"busVoltageV":round(supply,3),"aboveStudyFloor":supply>=e["studyFloorV"]}
    return result


def identity(plugin, archive_sha, observed, excluded=False):
    nodes=[dict(id=k,parentId=None,sourceId=k) for k in observed]
    bodies=[dict(id=k,occurrenceId=k) for k in observed]
    if excluded:
        bodies.pop()
    expected={"schema":"agent-frc/native-identity-manifest@1","archiveSha256":archive_sha,
              "allowedEvidenceBases":["native-observed"],
              "bodies":[dict(id=p["id"],occurrenceId=p["id"],sourceId=p["id"],sourceOwnerId=p["id"]) for p in PARTS]}
    actual={"schema":"agent-frc/native-identity-receipt@1","archiveSha256":archive_sha,
            "evidenceBasis":"native-observed","occurrences":nodes,"bodies":bodies}
    with tempfile.TemporaryDirectory() as temp:
        root=Path(temp)
        for key,data in (("planned",expected),("reopened",actual)):
            (root/(key+".json")).write_text(json.dumps(data))
        r=subprocess.run(["node",str(plugin/"scripts/check-native-identity.mjs"),str(root/"planned.json"),str(root/"reopened.json")],
                         capture_output=True,text=True)
        if r.returncode!=int(excluded):
            raise ValueError(r.stderr or r.stdout)
        return json.loads(r.stdout)["status"]


def audit(review_dir,plugin=None,openscad=None):
    review=json.loads((review_dir/"review.json").read_text())
    result={"identity":{},"powerStudy":budget(json.loads((HERE/"electrical.json").read_text()))}
    for name,obs in review["poses"].items():
        path=review_dir/(name+".step")
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if digest!=obs["stepSha256"] or obs["interferencesOver0.001mm3"]:
            raise ValueError("Unmatched STEP or material overlap at "+name)
        seen=read_named(path)
        shelf=seen["receiver-R6-ported"]
        pose=MODEL["poses"][name]
        expect=[-100+pose["stroke"],-210,pose["height"]-75,165+pose["stroke"],210,pose["height"]-60]
        if any(abs(x-y)>1.e-5 for x,y in zip(bounds(shelf),expect)):
            raise ValueError("shelf world pose")
        if plugin:
            result["identity"][name]=identity(plugin,digest,seen)
            if name=="stowed":
                result["missingBodyControl"]=identity(plugin,digest,seen,True)
        if name=="stowed":
            shelf_volume=volume(shelf)[0]
    if not openscad:
        return result
    with tempfile.TemporaryDirectory() as temp:
        out=Path(temp)
        def run(args):
            subprocess.run([str(openscad),*args,str((review_dir/"station-l1.scad").resolve())],capture_output=True,check=True,timeout=120)
        pose_hash={}
        for name in MODEL["poses"]:
            csg=out/(name+".csg")
            run(["-o",str(csg),"-D",f'pose="{name}"'])
            pose_hash[name]=hashlib.sha256(csg.read_bytes()).hexdigest()
        if len(set(pose_hash.values()))!=len(MODEL["poses"]):
            raise ValueError("Two CAD pose selections emitted identical CSG")
        stl=out/"shelf.stl"
        run(["-o",str(stl),"-D",'selected_part="receiver-R6-ported"'])
        vertices=[tuple(map(float,line.split()[1:])) for line in stl.read_text().splitlines() if line.split()[:1]==["vertex"]]
        signed_sum=0
        for a,b,c in zip(vertices[::3],vertices[1::3],vertices[2::3]):
            signed_sum += (a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6
        extent=[f(p[i] for p in vertices) for f in (min,max) for i in range(3)]
        if abs(signed_sum-shelf_volume)/shelf_volume>2.e-5 or extent!=[-100,-210,865,165,210,880]:
            raise ValueError("Editable CAD shelf registration/volume differs from the reopened STEP")
        result["editableCad"]={"fourDistinctCSG":True,"stlTriangles":len(vertices)//3,"shelfStlBoundsMm":extent,
                               "shelfFacetedMm3":round(signed_sum,4),"shelfStepMm3":round(shelf_volume,4)}
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("review_dir",type=Path)
    p.add_argument("--plugin",type=Path)
    p.add_argument("--openscad",type=Path)
    args=p.parse_args()
    print(json.dumps(audit(args.review_dir,args.plugin,args.openscad),indent=2))
