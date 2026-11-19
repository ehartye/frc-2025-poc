"""Plot the actual centered simulation and each sampled custody/placement outcome."""
from pathlib import Path
import json
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from motion import Case,run,HERE

def draw():
    data=run(Case(withdraw=0),True)
    tr=data["trace"];fig,(a,b)=plt.subplots(1,2,figsize=(10.8,4.7),gridspec_kw={"width_ratios":[1.5,1]})
    for label,lo,hi,color in (("Station feed",0,1.1,"#167477"),("Target discharge",3.48,5.6,"#b85f26")):
        pts=[r for r in tr if lo<=r[0]<=hi];a.plot([1000*r[1] for r in pts],[1000*r[2] for r in pts],lw=2.8,color=color,label=label)
        ix=min(pts,key=lambda x:abs(x[0]-(.195 if lo==0 else 3.86)))
        a.add_patch(Circle((1000*ix[1],1000*ix[2]),57.15,fill=False,color=color,ls="--"))
    a.plot([-70,152.3],[566,589.4],lw=6,color="#283943",label="Closed lower OD belts")
    a.add_patch(Circle((0,714),38.1,facecolor="#cdd5d9",edgecolor="#283943"));a.set_aspect("equal");a.set(xlabel="Forward from outer slide origin (mm)",ylabel="Height above carpet (mm)",title="Pickup and placement on the same slide")
    a.legend(loc="upper left",frameon=False,fontsize=8);a.grid(alpha=.14)
    rows=[r for r in json.loads((HERE/"study.json").read_bytes())["rows"] if r["id"].startswith("grid")]
    for j,r in enumerate(rows):
        b.scatter(j%8,7-j//8,s=180,marker="o" if r["conditionalSequence"] else "x",c="#167477" if r["conditionalSequence"] else "#ae472f",linewidth=2)
    b.set_aspect("equal");b.set_xlim(-1,8);b.set_ylim(-1,8);b.set_axis_off();b.set_title("64 paired disturbance cases")
    b.text(3.5,-1.4,f"{sum(r['conditionalSequence'] for r in rows)} conditional sequences  /  {sum(not r['conditionalSequence'] for r in rows)} holds or misses",ha="center",fontsize=10)
    fig.tight_layout();plt.rcParams["svg.hashsalt"]="od-gate"
    fig.savefig(HERE/"motion.svg",metadata={"Date":None},bbox_inches="tight");plt.close(fig)
if __name__=="__main__":draw()
