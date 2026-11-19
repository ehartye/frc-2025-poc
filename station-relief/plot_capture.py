"""Create the student comparison figure from the coupled source and native witness."""
import json
from dataclasses import replace
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams.update({"svg.hashsalt":"station-capture","svg.fonttype":"none"})
import matplotlib.pyplot as plt
from capture import CANDIDATE,HERE,simulate

def plot(path):
    figure,axis=plt.subplots(figsize=(8.4,3.6),layout="constrained")
    witness=json.loads((HERE/"witness.json").read_bytes())
    case=next(x for x in witness["cases"] if x["dxMm"]==-5 and x["yawDeg"]==-.25 and x["preloadN"]==10)
    for preload,color in ((10.,"#af4c21"),(11.5,"#136a74")):
        samples=[]
        simulate(-5,.8,-.25,cfg=replace(CANDIDATE,preload=preload),path=samples)
        rows=[(t*1000,x-270) for t,x,*_ in samples if t<=.65]
        axis.plot(*zip(*rows),color=color,lw=2.4,label=f"{preload:g} N per grip")
    crosses={(r["timeS"]*1000,r["poseMmDeg"][0]-270) for r in case["nativeHits"]}
    axis.scatter(*zip(*sorted(crosses)),marker="x",color="#4d3326",s=35,label="Sampled pipe/hardware overlap",zorder=3)
    axis.axhline(80,lw=.7,ls="--",color="#81949c")
    axis.set(xlabel="Milliseconds since the pipe left the chute",ylabel="Pipe center on shelf (mm)",xlim=(0,650),
             title="Spring preload changes the rearward excursion")
    axis.grid(axis="y",color="#d5dee1",lw=.55)
    axis.spines["right"].set_visible(False);axis.spines["top"].set_visible(False)
    axis.legend(loc="lower right",frameon=False,fontsize=8.5)
    figure.savefig(path,dpi=145,metadata={"Date":None})
    plt.close(figure)
    lines=path.read_text(encoding="utf-8").splitlines()
    path.write_text("\n".join(line.rstrip() for line in lines)+"\n", encoding="utf-8",newline="\n")

if __name__=="__main__":
    plot(HERE/"capture-path.svg")
