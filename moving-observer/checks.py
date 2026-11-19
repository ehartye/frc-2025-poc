"""Separate plant speed, installation and image-check measurements from the optical estimator."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import optics
from replay import load_scene
from optics import HERE,ROOT
from PIL import Image

def run(scratch):
    primary=json.loads((ROOT/"low-observation/results.json").read_bytes())
    result={"cases":{}}
    for key,p in primary["pieces"].items():
        tr=np.load(scratch/f"{key}-trajectory.npy")
        tr=tr[(tr[:,0]>=p["truth_ms"]["resume-from-modeled-java-in-band"]) & (tr[:,0]<=p["truth_ms"]["place"])].copy()
        tr[:,1]-=tr[:,5]
        delta=np.diff(tr,axis=0)
        delta=delta[delta[:,0]>0]
        speeds=np.max(1000*abs(delta[:,1:4])/delta[:,0,None],axis=0)
        result["cases"][key]={"maximum_modeled_shelf_relative_speed_mm_s":speeds.tolist(),"bounds_used_mm_s":[120,10,20]}
    lamp=json.loads((ROOT/"design/signal-light/review.json").read_bytes())["cases"]["low-place"]
    result["minimum_new_hardware_z_above_deck_light_mm"]=min(v["bounds_mm"][2] for v in
        json.loads((HERE/"mount.json").read_bytes())["parts"].values())-lamp["mount_mm"][5]
    # A simply supported static check of a symmetric bridge under transverse acceleration:
    # each welded post is modeled as a cantilever with half the total inertial load at its tip.
    length=.3595
    inertia=(.015**4-.013**4)/12
    force=.4396671*2*9.80665/2
    result["two_g_ideal_post_tip_mm"]=1000*force*length**3/(3*69e9*inertia)
    result["post_second_moment_m4"]=inertia
    result["end_to_end_calibration"]=False
    (HERE/"checks.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")
    optics.configure()
    signature=hashlib.sha256((HERE/"moving-bridge.step").read_bytes()).hexdigest()[:16]
    bg=load_scene(scratch/("moving-custody-"+signature+".npz"))
    cams=optics.camera.exposure(bg,[91.419,0,557.15],1.15)
    titles=["Registered empty shelf","Left eye at transit extreme","Right eye at transit extreme"]
    svg=['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 980 274"><title>Registered moving-shelf stereo views</title><rect width="980" height="274" fill="#18232c"/>']
    for i,im in enumerate([bg[0][0],*cams]):
        x=10+i*325
        svg.append(f'<text x="{x}" y="22" fill="white" font-family="sans-serif" font-size="14">{titles[i]}</text>')
        raster=np.asarray(Image.fromarray(im).resize((317,237),Image.Resampling.BOX))//16
        values={}
        for y,row in enumerate(raster):
            borders=np.r_[0,np.flatnonzero(np.diff(row))+1,len(row)]
            for first,last in zip(borders,borders[1:]):
                values.setdefault(int(row[first]),[]).append(f"M{first},{y}h{last-first}")
        for value,path in values.items():
            gray=value*16+8
            svg.append(f'<path transform="translate({x} 30)" d="{"".join(path)}" stroke="rgb({gray},{gray},{gray})" stroke-width="1.04"/>')
    (HERE/"views.svg").write_text("".join(svg)+"</svg>\n",encoding="utf-8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scratch",type=Path,required=True)
    run(p.parse_args().scratch)
