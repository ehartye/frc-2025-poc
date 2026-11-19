"""Plot centered docking coverage for the rounded entrance using the registered receiver."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt

if __package__:
    from . import receiver_dock
else:
    import receiver_dock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    args = parser.parse_args()
    design = json.loads(receiver_dock.receiver.DESIGN.read_text())
    angles = [i/20 for i in range(201)]
    plt.rcParams["svg.hashsalt"] = "station-dock-field-study"
    fig,axes = plt.subplots(1,2,figsize=(8.8,3.6),sharex=True,sharey=True,layout="constrained")
    for extension,ax in zip((220,240),axes):
        for speed,color in ((.5,"#b2482c"),(1,"#276582"),(2,"#388058")):
            fractions = [receiver_dock.dock(design,speed,a,0,extension,6)["flat_fraction"]
                         for a in angles]
            ax.plot(angles,[100*f for f in fractions],color=color,linewidth=2,label=f"{speed:g} m/s entry")
        ax.set_title(f"{extension} mm extension")
        ax.set_xlabel("Heading error (degrees)")
        ax.set_ylim(-2,103)
        ax.set_xlim(0,10)
        ax.grid(alpha=.2)
        ax.spines[["top","right"]].set_visible(False)
    axes[0].set_ylabel("Tube bottom line over flat shelf (%)")
    axes[1].legend(frameon=False,loc="lower left",fontsize=9)
    fig.savefig(args.output,metadata={"Date":None})
    plt.close(fig)
    if args.output.suffix.lower() == ".svg":
        lines = args.output.read_text(encoding="utf-8").splitlines()
        args.output.write_text("\n".join(line.rstrip() for line in lines) + "\n", encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
