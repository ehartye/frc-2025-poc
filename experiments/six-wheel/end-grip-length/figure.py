"""Draw the retained two-pulley geometry and the source-registered receiver boundary."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.patches import Circle,Rectangle
from shoe import HALF,RADIUS,DRIVE,IDLER
HERE=Path(__file__).resolve().parent

def draw():
    plt.rcParams.update({"svg.hashsalt":"coral-end-grip","svg.fonttype":"none"})
    fig,(left,right)=plt.subplots(2,1,figsize=(8,5.6),layout="constrained",height_ratios=[1,1.25])
    y=HALF+RADIUS+1.2
    for x in (DRIVE,IDLER):
        left.add_patch(Circle((x,y),RADIUS,fill=False,edgecolor="#376572",lw=2))
    for offset in (-RADIUS,RADIUS):
        left.plot([DRIVE,IDLER],[y+offset]*2,color="#376572")
    left.add_patch(Rectangle((-38,HALF-14),110,14,facecolor="#eac99f",edgecolor="#8d4f31"))
    left.annotate("Outside of pipe end",(-24,HALF-7),fontsize=9)
    left.annotate("Longitudinal belt working face",(0,y-RADIUS),xytext=(10,y+21),arrowprops={"arrowstyle":"->"},fontsize=9)
    left.annotate("Cassette moves laterally",(-87,y+18),fontsize=9)
    left.set(xlim=(-110,118),ylim=(132,187),aspect="equal",xlabel="Robot +X (mm)",ylabel="Robot +Y (mm)",title="Left fixed-center cassette from above")
    rows=json.loads((HERE/"replay.json").read_bytes())
    outcomes=[r for r in rows if r["case"]["name"].startswith("preload-")]
    for row in outcomes:
        force=row["case"]["spring"]
        position=row["optical"]["minimum_u_mm"]
        success=row["optical"]["outcome"]=="IN_BAND"
        right.scatter(force,position,marker="o" if success else "x",color="#2a6871" if success else "#9b431e")
    right.axhline(-26.263,color="#6e7278",ls="--",lw=1,label="Earlier centered 0.65 m/s minimum")
    right.set(xlabel="Ideal quasi-static normal per cassette (N)",ylabel="Minimum pipe X (mm)",title="Where the centered pipe reverses at the fixed low receiver")
    right.legend(fontsize=8)
    for ax in (left,right):ax.spines[["top","right"]].set_visible(False)
    path=HERE/"capture.svg"
    fig.savefig(path,metadata={"Date":None})
    plt.close(fig)
    path.write_text("\n".join(line.rstrip() for line in path.read_text(encoding="utf-8").splitlines())+"\n",encoding="utf-8",newline="\n")
if __name__=="__main__":draw()
