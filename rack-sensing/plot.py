"""Plot the first withdrawal in the issued HAL traces."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


def figure(raw, destination):
    plt.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "receiver-onset"})
    with (raw / "results.json").open() as stream:
        cases = json.load(stream)["cases"]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.8), sharey=True)
    traces = (("legacy-slip94", "Former detector", "#a34f31"),
              ("ramp94", "Revised drive profile", "#086765"),
              ("ramp-drag-adverse94", "High drag + adverse reading", "#74559f"))
    for ax in axes:
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=.22)
        ax.set(xlim=(-1.5, 44), ylim=(-.12, 4.6), xlabel="Time since command (ms)")
        ax.axvline(40, linestyle=":", color="#797979", linewidth=1)
    for mode, label, color in traces:
        case = cases[mode]["coral-A"]
        with (raw / f"path-{mode}-coral-A.csv").open() as stream:
            rows = [r for r in csv.DictReader(stream) if int(r["ms"]) <= 40]
        x = [int(r["ms"]) for r in rows]
        y = [1000 * float(r["withdrawn_m"]) for r in rows]
        ax = axes[0 if mode == "legacy-slip94" else 1]
        ax.plot(x, y, color=color, label=label, linewidth=2.5)
        event = "motion" if "motion" in case else "hold"
        tick = case[event] - case["withdraw"]
        # A fault can return before the time bin is logged. The last point is the
        # final integrated sample before the fault, not an estimate beyond it.
        mark = x.index(tick) if tick in x else -1
        ax.scatter([x[mark]], [y[mark]], s=48, color=color, zorder=3)
    axes[0].set_ylabel("Modeled rack withdrawal (mm)")
    axes[0].set_title("Stationary rack, motor encoder moving", fontsize=11)
    axes[1].set_title("Loaded rack, calibrated sensor", fontsize=11)
    axes[0].annotate("Old detector acknowledged at 5 ms\nwith 0 mm of rack motion.",
                     xy=(5, 0), xytext=(5, 2.4), arrowprops={"arrowstyle": "->", "color": "#a34f31"}, fontsize=9)
    axes[0].text(1, 3.75, "Revised detector withheld through 40 ms.", fontsize=9)
    axes[1].legend(loc="upper left", frameon=False, fontsize=9)
    fig.suptitle("Did the shelf actually move?", fontsize=17, weight="bold")
    fig.tight_layout()
    fig.savefig(destination, format="svg", metadata={"Date": None})
    plt.close(fig)
    lines = destination.read_text(encoding="utf8").splitlines()
    destination.write_text("\n".join(line.rstrip() for line in lines) + "\n", encoding="utf8", newline="\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    figure(args.raw, args.destination)
