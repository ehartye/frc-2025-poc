"""Plot time-resolved shelf load and cartridge travel for the two opposing wheel calibrations."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE=Path(__file__).resolve().parent


def plot(cache):
    plt.rcParams.update({"svg.fonttype":"none","svg.hashsalt":"frc-shelf-impact","font.size":10})
    fig,(a,b)=plt.subplots(1,2,figsize=(10,4.0),layout="constrained")
    reviews=json.loads((HERE/"fine-review.json").read_bytes())["cases"]
    for key,color,style,label in (("leftSmall","#136f72","-","Left tire small"),("rightSmall","#914125","--","Right tire small")):
        rows=np.genfromtxt(cache/(key+"-shelf.csv"),delimiter=",",names=True)
        mask=(rows["lip_s"]>=.2165)&(rows["lip_s"]<=.221)
        a.plot(rows["lip_s"][mask]*1000,-rows["force_z_n"][mask]/1000,style,color=color,label=label)
        vals=[reviews[key]["runs"][f"dt-{i}"]["cartridgeMm"] for i in range(1,5)]
        b.plot(range(4),vals,style,color=color,marker="o",label=label)
    a.set(title="Shelf reaction during first contact",xlabel="Time from the chute lip (ms)",ylabel="Downward load on shelf (kN)")
    b.set(title="Cartridge travel under refinement",xticks=range(4),xticklabels=["62.5","31.25","15.625","7.8125"],
          xlabel="Contact timestep (µs)",ylabel="Peak outward travel (mm)",ylim=(0,1.55))
    b.axhline(1.4,ls=":",color="#975b19",label="Cartridge stop")
    for ax in (a,b):
        ax.grid(axis="y",alpha=.20)
        ax.spines[["right","top"]].set_visible(False)
        ax.legend(fontsize=8,loc="best")
    path=HERE/"impact.svg";fig.savefig(path,metadata={"Date":None});plt.close(fig)
    path.write_text("\n".join(line.rstrip() for line in path.read_text(encoding="utf8").splitlines())+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);a=p.parse_args();plot(a.cache)
