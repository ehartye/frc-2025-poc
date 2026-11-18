"""Plot measured-in-simulation angular speed on the two time scales that set the outcome."""
from pathlib import Path
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent


def draw():
    d=json.loads((HERE/"study.json").read_bytes())["cases"]["nominal"]["coral-A"]
    capture=d["sampled_receiver_spin"]
    release=d["sampled_release_spin"]
    plt.rcParams.update({"font.family":"DejaVu Sans","font.size":9,"svg.fonttype":"none","svg.hashsalt":"coral-spin"})
    fig,axes=plt.subplots(1,2,figsize=(9,3.5),gridspec_kw={"width_ratios":[1.25,1]})
    fig.subplots_adjust(left=.09,right=.97,bottom=.21,top=.78,wspace=.40)
    fig.patch.set_facecolor("#faf8f1")
    for ax in axes:
        ax.set_facecolor("#faf8f1")
        ax.spines[["top","right"]].set_visible(False)
        ax.grid(axis="y",alpha=.2)
        ax.axhline(0,color="#435b53",lw=.85)
        ax.set_ylabel("Axial spin (rad/s)")
    a,b=axes
    a.plot([p[0] for p in capture],[p[1] for p in capture],color="#225d50",lw=2.4)
    a.axvline(d["pickup"]["impact"]["time_s"],ls="--",color="#b05032",lw=1)
    a.set(xlabel="Seconds after chute exit",title="Station capture")
    a.annotate("Shelf impact",(.053,24),(.23,25),arrowprops={"arrowstyle":"-","color":"#b05032"})
    b.plot([p[0]*1000 for p in release],[p[1] for p in release],color="#225d50",lw=2.4)
    b.set(xlabel="Milliseconds after placement command",title="Release toward the reef")
    b.annotate(f"Leaves at {d['release_spin_rad_s']:.2f} rad/s",(release[-1][0]*1000,release[-1][1]),
               xytext=(.28,.10),textcoords="axes fraction",arrowprops={"arrowstyle":"-","color":"#b05032"})
    fig.suptitle("One pipe from rolling entry to controlled release",fontweight="bold",fontsize=14,y=.97)
    fig.savefig(HERE/"spin-trace.svg",metadata={"Date":None})
    plt.close(fig)
    svg=HERE/"spin-trace.svg"
    svg.write_text("\n".join(line.rstrip() for line in svg.read_text(encoding="utf8").splitlines())+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":draw()
