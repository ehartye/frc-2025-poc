"""Show full-match terminal minima and supply current from the retained load traces."""
import argparse
import csv
from pathlib import Path
import matplotlib.pyplot as plt


def plot(folder,out):
    plt.rcParams.update({"svg.hashsalt": "fixed-low-power", "svg.fonttype": "none"})
    fig,axes=plt.subplots(2,1,sharex=True,figsize=(10,5.8),layout="constrained")
    cases=(("fixed","12.4 V / 25 mΩ, compressor off","#166b57"),
           ("weak","11.8 V / 32 mΩ, compressor on","#a33d2d"),
           ("screenedPack","11.8 V / 27 mΩ, compressor off","#335ca4"))
    for key,label,color in cases:
        with (folder/("park372-"+key+".csv")).open(encoding="utf-8") as f:
            rows=list(csv.DictReader(f))
        t=[float(r["wall_s"]) for r in rows]
        axes[0].plot(t,[float(r["minimum_terminal_V"]) for r in rows],color=color,lw=1.1,label=label)
        axes[1].plot(t,[float(r["pack_A"]) for r in rows],color=color,lw=1.)
    for ax in axes:
        ax.grid(axis="y",color="#dce1df")
        ax.spines[["top","right"]].set_visible(False)
        ax.set_xlim(0,156)
        ax.axvspan(15,18,alpha=.11,color="#566163")
        ax.axvspan(153,156,alpha=.11,color="#566163")
    axes[0].axhline(9.5,color="#414748",ls="--",lw=1)
    axes[0].text(155,9.39,"9.5 V floor",ha="right",va="top",fontsize=8)
    axes[0].set_ylabel("Lowest device terminal (V)")
    axes[1].set_ylabel("Battery current (A)")
    axes[1].set_xlabel("Wall clock (s)   ·   AUTO 0–15   ·   TELEOP 18–153   ·   final assessment 153–156")
    axes[0].legend(loc="upper right",fontsize=8)
    axes[0].set_title("Fixed-low two-pipe design  |  shared supply during the 372 in parking trial",loc="left")
    fig.savefig(out, metadata={"Date": None})
    plt.close(fig)
    lines = out.read_text(encoding="utf-8").splitlines()
    out.write_text("\n".join(line.rstrip() for line in lines) + "\n", encoding="utf-8", newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("traces",type=Path)
    args=p.parse_args()
    plot(args.traces,Path(__file__).with_name("match-power.svg"))
