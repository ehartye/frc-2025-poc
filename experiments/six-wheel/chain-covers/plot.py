"""Plot the tightest lateral interfaces and the budget returned by the installed plugin."""
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
mpl.rcParams.update({"font.family": "DejaVu Sans", "svg.fonttype": "none", "svg.hashsalt": "drive-cover-fit"})
cases = json.loads((HERE / "study.json").read_bytes())["cases"]
mass = json.loads((HERE / "accounting.json").read_bytes())
fig, (fit, budget) = plt.subplots(1, 2, figsize=(11.4, 4.2), gridspec_kw={"width_ratios": [1.3, 1]})
fig.subplots_adjust(left=.075, right=.97, bottom=.22, top=.76, wspace=.42)
fig.suptitle("Where can the front chain cover fit?", x=.075, y=.97, ha="left", fontsize=19, weight="bold")
cool, hot, dark = "#207978", "#ac523a", "#233541"
for y, name, sprocket in ((1, "original", 272.7), (0, "repacked-light", 269.7)):
    case = cases[name]
    face = case["faceCenterAbsYmm"]
    fit.barh(y, sprocket-267.5, left=267.5, height=.27, color=cool)
    fit.barh(y, .5, left=face-.25, height=.49, color=dark)
    fit.barh(y, 278-275.6752, left=275.6752, height=.27, color=hot)
    fit.text(267.6, y+.34, ("Original" if y else "Repacked") +
             f"   ·   nearest face {case['installed']['upper-face-L']['nearestMm']:.2f} mm", fontsize=10)
fit.set(xlim=(267.5,278), ylim=(-.5,1.75), yticks=[], xlabel="Distance from chassis centerline (mm)")
fit.set_title("Sprocket, cover and wheel-bolt tip", loc="left", fontsize=11, weight="bold")
from matplotlib.patches import Patch
fit.legend(handles=[Patch(color=cool,label="Sprocket"),Patch(color=dark,label="Cover"),Patch(color=hot,label="Bolt")],
           loc="lower left", bbox_to_anchor=(-.03,-.4), ncol=3, frameon=False, fontsize=9)
for y, (name, label) in enumerate((("repacked-thick","0.75 mm"),("repacked-light","0.50 mm"))):
    grams = 1000 * mass[name]["conditionalDeltaKg"][1]
    budget.barh(y, grams, height=.27, color=cool if y else hot)
    budget.text(4, y+.26, f"{label} sheet   ·   {grams:.1f} g", fontsize=10)
budget.axvline(160.9836, color=dark, linestyle="--", linewidth=1.3)
budget.text(156, 1.72, "161 g available", ha="right", fontsize=9)
budget.set(xlim=(0,215), ylim=(-.5,1.95), yticks=[], xlabel="Combined sheet mass at proposed upper allowance (g)")
budget.set_title("Both wheel bays, before attachments", loc="left", fontsize=11, weight="bold")
for ax in (fit, budget):
    ax.spines[["top","left","right"]].set_visible(False)
    ax.spines["bottom"].set_color("#abb9bc")
    ax.tick_params(axis="x", labelsize=9, length=3)
target = HERE / "cover-fit.svg"
fig.savefig(target, metadata={"Date": None, "Title": "Chain cover fit and mass"})
lines = target.read_text(encoding="utf-8").splitlines()
target.write_text("\n".join(line.rstrip() for line in lines)+"\n", encoding="utf-8", newline="\n")
