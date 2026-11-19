"""Render the exact no-air assembly and check changes along the moving camera's pipe rays."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"moving-observer"))
import optics
from optics import camera
from observe import read_pair
sys.path.insert(0,str(ROOT/"design/fixed-no-air"))
import whole
sys.path.insert(0,str(ROOT/"low-observation"))
from geometry import pose_layers


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(lamp_path,scratch,cache,only):
    cache.mkdir(parents=True,exist_ok=True)
    register=json.loads((ROOT/"design/fixed-no-air/whole-robot.json").read_bytes())
    if digest(lamp_path)!=register["supplied_lamp_sha256"]:
        raise ValueError("Different lamp")
    kit=ROOT/"moving-observer"
    names=[*whole.read(kit/"mount.json")["parts"],"receiver-R6-drilled-for-camera"]
    roof=whole.cad.read_named(kit/"moving-bridge.step",[{"id":s} for s in names])
    deck=whole.cad.read_named(whole.LIGHT/"deck-mount.step",[{"id":s} for s in ["front-electronics-platform","light-spacer"]])
    lamp=whole.mounted(whole.product(lamp_path),whole.X,whole.Y,whole.PANEL)
    out={"lampSha256":digest(lamp_path),"registerSha256":digest(ROOT/"design/fixed-no-air/whole-robot.json"),"poses":{}}
    profiles=json.loads((scratch/"source.json").read_bytes())["pieces"]
    for stroke,mode in ((0,"receive"),(370,"place")):
        if stroke!=only: continue
        old,new=whole.model(stroke,roof,deck,lamp)
        _,group,colors,frames=pose_layers(560,stroke,"red")
        if len(old)!=598 or len(new)!=593:raise ValueError("Changed native inventory")
        receipt=register["cases"][mode]
        for name,sha in receipt["identical_brep_sha256"].items():
            if whole.fingerprint(new[name])!=sha:raise ValueError("Unregistered common body "+name)
        for name,sha in receipt["new_or_changed"].items():
            if whole.fingerprint(new[name])!=sha:raise ValueError("Unregistered replacement "+name)
        mesh=[]
        for name,shape in sorted(new.items()):
            v,t=whole.original.triangulate(shape)
            ref="receiver-R6-ported" if name=="receiver-R6-drilled-for-camera" else name
            mesh.append([name,group.get(ref,"Additional"),colors.get(ref,"#737d85"),v,t])
        images=camera.images_for_pose({"parts":mesh})
        file=cache/f"fixed-{stroke}.npz"
        np.savez_compressed(file,left=images[0][0],left_z=images[0][1],right=images[1][0],right_z=images[1][1])
        stats={}
        gray=[a for a,b in images]
        for k,rec in profiles.items():
            tr=np.load(scratch/(k+"-trajectory.npy"))
            times=(list(range(0,int(rec["first_in_band_ms"])+230,17)) if not stroke else
                   list(range(int(np.ceil(rec["truth"]["place"])),int(rec["truth"]["paired-trough-first-contact"])+1)))
            rows=[]
            for tm in times:
                coords=[np.interp(tm,tr[:,0],s) for s in tr[:,1:5].T] if tm>=tr[0,0] else None
                pixels=camera.exposure(images,None if coords is None else coords[:3],0 if coords is None else coords[3])
                obs=read_pair(pixels,gray,560,stroke)
                rows.append([tm,obs])
            stats[k]=rows
        (cache/f"fixed-{stroke}.json").write_text(json.dumps(stats,allow_nan=False),encoding="utf-8")
        out["poses"][mode]={"nativeBodies":len(new),"fixedFrames":sum(map(len,stats.values())),
                            "frameFileSha256":digest(cache/f"fixed-{stroke}.json")}
        if stroke==0:
            # Bound the sight prism from either moving-head eye to the full finite pipe during
            # custody AND both fixed/moving handoff phases before the nose clears. Roof and
            # slide move together. All changed bodies are base-fixed throughout the 370 mm.
            # The nearer visible side of any pipe is above this conservative lower height.
            zmin=430.; xmax=720.; xmin=-40.; yhalf=195.
            # Enclose both complete finite transverse pipes on every registered 0.5 ms
            # trajectory sample, allowing 1.5 deg yaw, +1 mm radius and surveyed error.
            extent_x=59+152*np.sin(np.radians(1.5))+2
            for key,record in profiles.items():
                tr=np.load(scratch/(key+"-trajectory.npy"))
                part=tr[(tr[:,0]>=record["first_in_band_ms"]-34)&(tr[:,0]<=record["truth"]["paired-trough-first-contact"])]
                if (part[:,1].min()-extent_x<xmin or part[:,1].max()+extent_x>xmax or
                    np.abs(part[:,2]).max()+155>yhalf or part[:,3].min()-60<zmin or
                    np.abs(part[:,4]).max()>1.5 or part[:,3].max()+60>842):
                    raise ValueError("Pipe or calibrated ray falls outside sight prism")
            rows=[]
            for label,shapes in (("598",old),("593",new)):
                for name in sorted(set(shapes)&(set(receipt["removed"])|set(receipt["new_or_changed"])|{"front-electronics-platform"})):
                    b=whole.cad.bounds(shapes[name])
                    gaps=(xmin-b[3],b[0]-xmax,-yhalf-b[4],b[1]-yhalf,zmin-b[5],b[2]-842)
                    rows.append({"revision":label,"part":name,"aabb":[round(x,3) for x in b],"separationMm":round(max(gaps),3)})
            out["movingRayPrism"]={"xyzMinMm":[xmin,-yhalf,zmin],"xyzMaxMm":[xmax,yhalf,842],"baseChanges":rows,
                                   "allChangedBodiesOutside":min(r["separationMm"] for r in rows)>0}
    (cache/f"optics-{only}.json").write_text(json.dumps(out,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("lamp",type=Path);p.add_argument("scratch",type=Path);p.add_argument("output",type=Path)
    p.add_argument("--stroke",type=int,choices=(0,370)); a=p.parse_args()
    if a.stroke is not None:
        run(a.lamp,a.scratch,a.output,a.stroke)
    else:
        import subprocess
        for stroke in (0,370):
            subprocess.run([sys.executable,__file__,str(a.lamp),str(a.scratch),str(a.output),"--stroke",str(stroke)],check=True)
        receipt=json.loads((a.output/"optics-0.json").read_bytes())
        receipt["poses"].update(json.loads((a.output/"optics-370.json").read_bytes())["poses"])
        (HERE/"optics.json").write_text(json.dumps(receipt,indent=2)+"\n",encoding="utf-8",newline="\n")
