"""Draw the supply panel and its fixed-camera lead from the checked geometry."""
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent


def draw():
    data = json.loads((HERE / "converter-panel.json").read_bytes())
    pose = data["poses"]["fixed-no-air/pickup"]
    fig, (plan, elevation) = plt.subplots(1, 2, figsize=(9, 5), width_ratios=(1.3, 1),
                                         layout="constrained")
    fig.suptitle("Power panel beside the Pi", fontsize=17, weight="bold")
    x,y,_,w,h,_ = data["reservedEnvelopeOriginAndSizeMm"]
    plan.add_patch(Rectangle((x,y),w,h, fill=False, linestyle="--", linewidth=1.5))
    for label, bounds in zip(("Pi", "Fixed", "Moving"),data["converterBoundsOriginAndSizeMm"]):
        x,y,_,w,h,_ = bounds
        plan.add_patch(Rectangle((x,y),w,h,facecolor="#d9e8ed",edgecolor="#245864"))
        plan.text(x+w/2,y+h/2,label,ha="center",va="center",fontsize=10)
    for foot in pose["feet"].values():
        x,y,_=foot["baseMm"]
        plan.plot(x,y,"o",color="#bf5d31",markersize=6)
    plan.set(xlim=(-204,-43),ylim=(-215,-105),title="Plan · four support feet",
             xlabel="Chassis X (mm)",ylabel="Chassis Y (mm)",aspect="equal")
    segments = pose["fixedCameraLeadSegments"]
    pts = [segments[0]["from"],*[s["to"] for s in segments]]
    # Cumulative horizontal travel keeps the sideward trunk approach visible.
    along = [0.]
    for a,b in zip(pts,pts[1:]):
        along.append(along[-1]+math.dist(a[:2],b[:2]))
    elevation.plot(along,[p[2] for p in pts],"-o",color="#245864",linewidth=2.5,markersize=4)
    elevation.annotate("USB service space",(along[-1],pts[-1][2]),xytext=(-10,6),
                       textcoords="offset points",ha="right")
    elevation.annotate("Fixed supply",(0,pts[0][2]),xytext=(5,7),textcoords="offset points")
    elevation.set(xlim=(-20,along[-1]+20),ylim=(210,1045),title="Fixed camera lead · elevation",
                  xlabel="Travel in chassis X/Y plane (mm)",ylabel="Height (mm)")
    for ax in (plan,elevation):
        ax.grid(alpha=.16)
        ax.spines[["top","right"]].set_visible(False)
    plt.rcParams["svg.hashsalt"]="fixed-low-panel"
    plt.rcParams["svg.fonttype"]="none"
    output = HERE / "converter-panel.svg"
    fig.savefig(output,metadata={"Date":None})
    lines = output.read_text(encoding="utf-8").splitlines()
    output.write_text("\n".join(line.rstrip() for line in lines)+"\n",encoding="utf-8",newline="\n")


if __name__ == "__main__":
    draw()
