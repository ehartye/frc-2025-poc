"""Plot the native guide dimensions and the two hardware-limited numerical trials."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

HERE=Path(__file__).resolve().parent


def draw():
    review=json.loads((HERE/"review.json").read_bytes())["cases"]
    plt.rcParams.update({"font.size":10,"svg.fonttype":"none","svg.hashsalt":"frc-station-passive-guide"})
    fig,(ax,bx)=plt.subplots(1,2,figsize=(10,4.1),layout="constrained")
    slate="#253643"; teal="#176d70"; red="#a23c2c"
    ax.axhspan(862.5,877.5,color="#d0d6d9",label="Moving camera cross tube")
    for width,shift,top,color,lw,label in ((80,0,900,red,1.6,"Original 80 mm face"),
                 (40,20,850,teal,2.5,"Short 40 mm face")):
        lo=625.
        x=lambda z:145+.46*(z-625)+shift
        vertices=[(x(lo)-width/2,lo),(x(lo)+width/2,lo),(x(top)+width/2,top),(x(top)-width/2,top)]
        ax.add_patch(Polygon(vertices,closed=True,facecolor="none",edgecolor=color,linewidth=lw,label=label))
    ax.set(xlim=(70,340),ylim=(600,925),xlabel="Robot forward X (mm)",ylabel="Height above carpet (mm)",title="Guide and moving tube")
    ax.legend(loc="upper left",fontsize=8)
    for name,color,marker,label in (("leftSmall",teal,"o","Left tire small"),("rightSmall",slate,"s","Right tire small")):
        values=[r["observedPeakMm"] for r in review[name]["runs"] if r["run"].startswith("dt-")]
        bx.plot([0,1,2,3],values,marker=marker,color=color,label=label)
    bx.axhline(1.4,color=red,ls="--",label="Cartridge stop")
    bx.set_xticks(range(4),["500","250","125","62.5"])
    bx.set(xlabel="Contact timestep (µs)",ylabel="Largest observed outward travel (mm)",ylim=(0,1.65),title="Same arrivals, refined contact")
    bx.legend(loc="upper left",fontsize=8)
    for a in (ax,bx):
        a.spines[["top","right"]].set_visible(False)
        a.grid(axis="y",alpha=.16)
    fig.savefig(HERE/"guide-study.svg",metadata={"Date":None})
    plt.close(fig)
    path=HERE/"guide-study.svg"
    path.write_text("\n".join(line.rstrip() for line in path.read_text(encoding="utf8").splitlines())+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    draw()
