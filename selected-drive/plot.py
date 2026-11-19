"""Plot the original two-pipe reference demand and native station counterexamples."""
from pathlib import Path
import argparse
import matplotlib.pyplot as plt
import numpy as np

HERE=Path(__file__).resolve().parent

def run(cache):
    fig,(top,bottom)=plt.subplots(2,1,figsize=(10,5.7),sharex=True,layout="constrained")
    axes=[top,bottom]
    samples=None
    for name,color,label in (("screened","#16755f","27 mÃŽÂ© screened pack"),("weak","#a03f35","32 mÃŽÂ© weak pack")):
        samples=np.genfromtxt(cache/("nominal-"+name+"-bus.csv"),delimiter=",",names=True)
        top.plot(samples["wall_s"],samples["right_terminal_v"],color=color,linewidth=1.1,label=label)
    top.axhline(9.5,color="#333333",linestyle="--",linewidth=.9,label="9.5 V design floor")
    top.set(ylabel="Right drive inlet (V)",ylim=(8.7,12.05))
    top.legend(loc="upper center",ncol=3,fontsize=8)
    bottom.plot(samples["wall_s"],samples["drive_phase_a"],color="#205b89",linewidth=1,label="Maximum of both drive motors")
    bottom.axhline(38,color="#333333",linestyle="--",linewidth=.9,label="38 A phase cap")
    bottom.set(xlabel="Match wall time (s)",ylabel="Drive phase current (A)",ylim=(-1,44),xlim=(0,113))
    bottom.legend(loc="upper center",ncol=2,fontsize=8)
    for ax in axes:
        ax.grid(axis="y",alpha=.2)
        for x in (15,18,27.032,50.765,77.435,100.845):ax.axvline(x,color="#909ba1",linewidth=.5,alpha=.4)
    fig.suptitle("Selected chassis on the two-pipe reference clock",fontsize=15)
    plt.rcParams["svg.hashsalt"]="selected-drive"
    fig.savefig(HERE/"match-demand.svg",metadata={"Date":None})
    path=HERE/"match-demand.svg"
    path.write_text("\n".join(line.rstrip() for line in path.read_text(encoding="utf8").splitlines())+"\n",encoding="utf8",newline="\n")
    plt.close(fig)

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path)
    run(p.parse_args().cache)
