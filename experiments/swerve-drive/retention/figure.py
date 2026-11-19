"""Draw the positive-Y pipe end at the actual minus stop, plus both independent probe histories."""

import argparse
from pathlib import Path
import json
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from fingers import PINS_X, PIN_Z, PROBE_XZ, HALF, INNER, OUTER


def draw(artifacts, output):
    fig, (face, readings) = plt.subplots(
        1,
        2,
        figsize=(10, 4.2),
        constrained_layout=True,
        gridspec_kw={"width_ratios": [1, 1.35]},
    )
    raw = artifacts / "retained-coral"
    data = json.loads((raw / "minus/capture-1/receipt.json").read_bytes())
    center = np.array(data["finalCenterMm"]) + HALF * np.array(data["finalAxis"])
    x, z = center[[0, 2]]
    face.add_patch(
        Circle((x, z), OUTER, facecolor="#f6c760", edgecolor="#5b430c", lw=1.2)
    )
    face.add_patch(
        Circle((x, z), INNER, facecolor="white", edgecolor="#5b430c", lw=0.8)
    )
    for pin in PINS_X:
        face.add_patch(Circle((pin, PIN_Z), 4, color="#007c79"))
    face.scatter(*PROBE_XZ, marker="D", s=32, color="#75345e", zorder=3)
    face.annotate(
        "Rim probe",
        PROBE_XZ,
        (113, 624),
        ha="center",
        arrowprops=dict(arrowstyle="-", color="#75345e"),
    )
    face.annotate(
        "Bore pins",
        (PINS_X[0], PIN_Z),
        (55, 536),
        arrowprops=dict(arrowstyle="-", color="#007c79"),
    )
    face.set(
        xlim=(12, 151),
        ylim=(489, 636),
        aspect="equal",
        xlabel="Chassis X (mm)",
        ylabel="Height (mm)",
        title="Where each end is sensed",
    )
    for side, style in (("minus", "-"), ("plus", "--")):
        trace = np.load(raw / side / "capture-1/trace.npy")
        for col, name, color in ((14, "−Y", "#11647b"), (15, "+Y", "#b04d23")):
            readings.plot(
                trace[:, 0] * 1000,
                trace[:, col],
                style,
                color=color,
                label=f"{name} probe · {side} calibration",
            )
    readings.axhspan(0.65, 4.5, color="#88c7ad", alpha=0.17)
    readings.axvline(255, color="#474f55", ls=":", lw=1.5)
    readings.text(264, 0.16, "5 fresh samples", fontsize=9)
    readings.set(
        xlim=(0, 500),
        ylim=(0, 4.8),
        xlabel="Time after confirmed stop (ms)",
        ylabel="Probe compression (mm)",
        title="Both ends must report contact",
    )
    readings.grid(axis="y", alpha=0.2)
    readings.legend(frameon=False, fontsize=8, loc="upper left", ncol=2)
    for ax in (face, readings):
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_title(ax.get_title(), loc="left", fontweight="bold")
    fig.savefig(output, dpi=170, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    draw(args.artifacts, args.output)
