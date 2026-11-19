"""Student diagram of the native image measurements and the attained end poses."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from panel import pixels,referenced
import numpy as np


def main(directory):
    here=Path(__file__).resolve().parent
    routes=json.loads((here/"routes.json").read_bytes())["cases"]
    feeds=json.loads((here/"feed.json").read_bytes())["outcomes"]
    _,bias,_,_=referenced(directory)
    data=pixels([0,0,0])+bias
    plt.rcParams.update({"svg.hashsalt":"lower-station-approach","svg.fonttype":"none","font.size":10})
    fig,(im,ax)=plt.subplots(1,2,figsize=(10.8,4.25),layout="constrained")
    im.set(xlim=(0,640),ylim=(480,0),xlabel="Fixed-camera image column (px)",ylabel="Row (px)")
    im.set_title("Four lower-panel holes appear in each eye",fontsize=12)
    for points,lab,col,marker in zip(data,("Left eye","Right eye"),("#aa3f2e","#2665a2"),("o","s")):
        im.scatter(*points.T,label=lab,color=col,marker=marker,s=37)
        im.plot(*points[[0,1,3,2,0]].T,color=col,alpha=.42,lw=1)
    im.legend(loc="lower left");im.grid(alpha=.16)
    names=("nominal","leftSmall","rightSmall","trackOnly","bothLarge","bothSmall","crossedDrift","badRegistration")
    labels=("Nominal","Left small","Right small","Track +2 mm","Both large","Both small","Crossed drift","Mount shifted")
    for i,(name,label) in enumerate(zip(names,labels)):
        f=feeds[name]
        y=routes[name]["observer"]["finalStationMmDeg"][1]
        works=f["lastPortState"]=="IN_BAND" and f["hardStop"] is None and not f["loss"]
        ax.scatter(y,i,marker="o" if works else "X",color="#267252" if works else "#a73633",s=65)
    ax.axvline(0,color="gray",lw=1)
    ax.set_yticks(range(len(labels)),labels=labels);ax.invert_yaxis()
    ax.set_xlim(-3.45,1.1);ax.grid(axis="x",alpha=.18)
    ax.set_xlabel("Attained chassis lateral offset (mm)")
    ax.set_title("Port confirmation after the finite feed",fontsize=12)
    ax.text(.03,.95,"● Both ports   × Mechanical stop",transform=ax.transAxes,va="top",fontsize=9)
    fig.savefig(here/"station-evidence.svg",metadata={"Date":None})
    plt.close(fig)
    svg=here/"station-evidence.svg"
    clean="\n".join(line.rstrip() for line in svg.read_text(encoding="utf8").splitlines())
    svg.write_text(clean+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("native_views",type=Path)
    main(parser.parse_args().native_views)
