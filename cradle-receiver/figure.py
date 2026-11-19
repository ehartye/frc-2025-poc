"""Draw the working section and registration failures from committed simulation evidence."""
import json
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from model import BACK_PROFILE, HERE, PIVOT

def draw(output):
    plt.rcParams.update({"svg.hashsalt":"coral-cradle-1","svg.fonttype":"none"})
    fig, (left,right) = plt.subplots(1,2,figsize=(11,4),layout="constrained")
    rows=json.loads((HERE/"study.json").read_bytes())["rows"]
    nominal=next(r for r in rows if r["id"]=="backstop-A")
    xs,zs=zip(*BACK_PROFILE)
    left.plot([1000*x for x in xs],[1000*z for z in zs],color="#2d646e",linewidth=4,label="Cradle surface")
    left.plot([-100,165],[0,0],color="#60707a",label="Retained shelf")
    left.scatter(1000*PIVOT[0],1000*PIVOT[1],marker="x",s=55,color="#463c30",label="Pivot")
    x,z=nominal["received"][:2]
    for radius in (57.15,50.8):
        left.add_patch(Circle((1000*x,1000*z),radius,fill=False,color="#963b24"))
    left.set(xlim=(-110,175),ylim=(-10,150),aspect="equal",xlabel="Forward position (mm)",ylabel="Height above shelf (mm)",title="Tube at rest on the two rails")
    left.legend(loc="upper right",fontsize=8)
    cases=[r for r in rows if r["id"].startswith("factor-")]
    counts=[len(cases),sum(r["seatAtExistingTime"] is True for r in cases),sum(r["conditionalSequence"] for r in cases)]
    bars=right.barh(["Rows tested","Seated by deadline","Seated and supported"],counts,color=["#6c7983","#42727a","#2a5e69"])
    right.bar_label(bars,padding=5)
    right.invert_yaxis()
    right.set(xlim=(0,72),xlabel="Pairwise disturbance rows",title="Fixed-angle dump at the target")
    for ax in (left,right):ax.spines[["top","right"]].set_visible(False)
    fig.savefig(output,metadata={"Date":None})
    plt.close(fig)

if __name__=="__main__":
    draw(HERE/"cradle.svg")
