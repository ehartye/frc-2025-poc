"""Draw the unslipped forecast and the independently simulated finite stops."""

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent


def plot(folder, out):
    drives = json.loads((HERE / "drive.json").read_bytes())
    stops = json.loads((HERE / "stop.json").read_bytes())
    mpl.rcParams["svg.hashsalt"] = "elastic-receiver-carry"
    mpl.rcParams["svg.fonttype"] = "none"
    fig, (ax, bars) = plt.subplots(
        2,
        1,
        figsize=(9.3, 5.2),
        sharex=True,
        gridspec_kw={"height_ratios": [1.5, 1.0]},
        layout="constrained",
    )
    for row in drives:
        if row["stepS"] != 0.0000625:
            continue
        side = row["case"]
        file = folder / "carry" / f"{side}-62500ns.npy"
        if hashlib.sha256(file.read_bytes()).hexdigest() != row["traceSha256"]:
            raise ValueError("Unknown carry")
        series = np.load(file, mmap_mode="r")
        ax.plot(
            row["startWallS"] + series[::12, 0],
            1000 * series[::12, 1],
            color="#245a7a" if side == "leftSmall" else "#cf711e",
            linewidth=2.3,
            linestyle="-" if side == "leftSmall" else (0, (3, 3)),
            label="Left wheel small" if side == "leftSmall" else "Right wheel small",
        )
    ax.axhline(165, color="#9a3431", linewidth=1.0)
    ax.text(
        27.2, 160, "Forward shelf edge · X165 mm", color="#8b302d", va="top", fontsize=9
    )
    ax.set(
        ylabel="Pipe center X (mm)",
        ylim=(65, 174),
        title="Pipe position if the wheels continue the programmed route",
    )
    ax.legend(loc="lower right", frameon=False, fontsize=9)
    chosen = [
        r
        for r in stops
        if r["stepS"] == 0.0000625
        and r["assumedActuationLatencyMs"] == 20
        and (r["monitor"] == "roof" or r["case"] == "leftSmall")
    ]
    chosen.sort(key=lambda r: (r["monitor"] == "ports", r["case"] == "leftSmall"))
    for i, r in enumerate(chosen):
        alarm, act, end = r["alarmWallS"], r["actuationWallS"], r["wheelStopWallS"]
        bars.plot(
            [alarm, act], [i, i], color="#5b6570", linewidth=8, solid_capstyle="butt"
        )
        bars.scatter(alarm, i, marker="|", s=115, color="#283442", zorder=5)
        bars.plot(
            [act, end], [i, i], color="#b85e1a", linewidth=8, solid_capstyle="butt"
        )
        if r["loss"]:
            bars.scatter(
                r["endWallS"],
                i,
                color="#b62527",
                marker="x",
                s=60,
                linewidth=2,
                zorder=5,
            )
        else:
            bars.plot(
                [end, r["endWallS"]],
                [i, i],
                color="#247966",
                linewidth=8,
                solid_capstyle="butt",
            )
    bars.set_yticks(
        range(3),
        [
            "Roof · right wheel small",
            "Roof · left wheel small",
            "Ports · left wheel small",
        ],
        fontsize=8,
    )
    bars.set(
        ylim=(2.75, -0.65),
        xlim=(26.8, 37.4),
        xlabel="Match wall time (s)",
        title="Wheel braking after the alarm and a 20 ms actuation delay",
    )
    bars.plot([], [], color="#b85e1a", linewidth=4, label="Braking")
    bars.plot([], [], color="#247966", linewidth=4, label="One-second retained hold")
    bars.scatter([], [], color="#b62527", marker="x", label="Pipe lost")
    bars.legend(
        loc="upper center",
        bbox_to_anchor=(0.48, -0.24),
        ncol=3,
        fontsize=8,
        frameon=False,
    )
    for axis in (ax, bars):
        axis.spines[["top", "right"]].set_visible(False)
        axis.grid(axis="x", color="#dce1e5", linewidth=0.7)
        axis.set_axisbelow(True)
    fig.savefig(out, format="svg", metadata={"Date": None})
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    plot(a.folder, a.out)
