"""Probe the measured 597-solid camera views at the attained station and release states."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

from cameras import HERE,ROOT,bridge,camera,optics,sha
from images import silhouette
import bridge_fit


def background(path):
    with np.load(path) as f:
        return [(f[n].copy(),f[n+"_z"].copy()) for n in ("left","right")]


def run(cache):
    refs=json.loads((HERE/"camera-registration.json").read_bytes())
    nominal=background(cache/"relieved-roof-nominal.npz")
    guarded=background(cache/"relieved-roof-guaranteed.npz")
    backed=background(cache/"relieved-roof-backed.npz")
    possible=[(g,d) for (g,_),(_,d) in zip(nominal,backed)]
    optics.ORIGIN=np.array([48.,0.,842.])
    optics.configure()
    records=[]
    for name,piece in (("first","coral-A"),("return","coral-B")):
        data=json.loads((HERE/(name+"-feed.json")).read_bytes())["cases"][0]
        trace=np.load(cache/(name+"-nominal.npy"))
        for tick in (17*((data["firstPortBandMs"]+16)//17),17*((int(data["scheduledDriveCameraMs"])-18)//17)):
            obs=np.array([np.interp(tick,trace[:,0],col) for col in trace[:,1:7].T])
            scene=silhouette(nominal,obs[:3],obs[3:]/np.linalg.norm(obs[3:]))
            for kind,bg in (("nominal",nominal),("registered-stable",guarded),("fixed-material-behind",possible)):
                fit=bridge_fit.fit(scene,bg)
                reg=bridge_fit.registration_scores(scene,bg)
                print(piece,tick,kind,fit["status"],reg,flush=True)
                records.append({"pipe":piece,"tickMs":tick,"background":kind,"fit":fit,"registrationFractions":reg})
    (HERE/"station-roof.json").write_text(json.dumps({"register":refs["roofGuaranteedSha256"],"cases":records},indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("cache",type=Path)
    run(p.parse_args().cache)
