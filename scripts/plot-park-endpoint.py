"""Draw the R4 terminal-pose tradeoff from the native park study."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "experiments/swerve-drive/motion/park-endpoint.json"
OUTPUT = ROOT / "experiments/swerve-drive/motion/park-endpoint.svg"


def draw():
    data = json.loads(SOURCE.read_bytes())
    keys = ["scoring-return", "inboard", "offset-for-cage-swing", "outside-control"]
    labels = ["Scoring return", "Inboard", "Offset target", "Outside control"]
    rows = [data["cases"][key] for key in keys]
    data_sets = [[sum(row["guaranteedClothVolumeMm3"].values()) / 1000 for row in rows],
                 [row["clearanceOutsideSixDegreeCageConeMm"] for row in rows]]
    base = "#10273b"
    plt.rcParams.update({"svg.fonttype":"none", "svg.hashsalt":"r4-park-v1", "font.family":"DejaVu Sans"})
    fig, axes = plt.subplots(1, 2, figsize=(10.6, 3.5), gridspec_kw={"width_ratios":[1, 1.27]})
    fig.patch.set_facecolor("#faf8f1")
    fig.subplots_adjust(left=.16, right=.95, bottom=.18, top=.71, wspace=.25)
    fig.text(.03,.88,"Where can the stowed robot park?",fontsize=20,weight="bold",color=base)
    fig.text(.03,.78,"Terminal-pose allowance: ±25 mm on each field axis and ±3° heading",fontsize=10,color=base)
    for axis, values, title, limits in zip(axes, data_sets, ["Cloth inside zone (cm³)", "Margin to 6° cage cone (mm)"], [(-8,410),(-320,710)]):
        axis.set_facecolor("#faf8f1")
        axis.set_xlim(*limits)
        axis.set_ylim(3.6,-.6)
        axis.spines[:].set_visible(False)
        axis.axvline(0,color="#526777",lw=1)
        axis.set_title(title,loc="left",fontsize=11,weight="bold",color=base)
        axis.set_yticks(range(4), labels if axis is axes[0] else [""] * 4)
        axis.tick_params(length=0,labelsize=9,colors=base)
        for i, value in enumerate(values):
            color = "#17765d" if i == 2 else "#8b433d" if value < 0 else "#386585"
            axis.barh(i,value,height=.42,color=color)
            axis.text(value+(8 if value >= 0 else -8),i,f"{value:.0f}",va="center",
                      ha="left" if value >= 0 else "right",fontsize=10,color=base)
    fig.text(.03,.035,"A positive margin clears the modeled cage envelope. Parking also requires bumper cloth in the zone.",fontsize=9,color=base)
    fig.savefig(OUTPUT,format="svg",metadata={"Date":None,"Title":"R4 parking target comparison",
                "Description":"The offset target retains bumper cloth in the zone and clearance to a six-degree cage swing."})
    plt.close(fig)
    lines = OUTPUT.read_text(encoding="utf-8").splitlines()
    OUTPUT.write_bytes(("\n".join(line.rstrip() for line in lines) + "\n").encode("utf-8"))


if __name__ == "__main__":
    draw()
