"""Plot the registered contact event and the largest displacement on either guide face."""

import argparse
import json
from pathlib import Path
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from beam import digest


def plot(results, raw, out):
    r = json.loads(results.read_bytes())
    choose = [
        x
        for x in r["cases"]
        if x["case"] == "rightSmall"
        and x["stepUs"] == 31.25
        and not x["carriageSupport"]
        and x["guideModel"][1:] == [1800, 0.236]
        and (x["layout"] == "folded" or x["layout"] == "flat")
    ]
    choose.sort(key=lambda x: (x["layout"] == "flat", bool(x["outwardPerSideMm"])))
    if len(choose) != 3:
        raise ValueError("Need folded, flat and outboard catches")
    mpl.rcParams.update(
        {
            "svg.hashsalt": "registered-guide-impact",
            "svg.fonttype": "none",
            "font.size": 9,
        }
    )
    fig, (a, b) = plt.subplots(2, 1, sharex=True, figsize=(8.5, 4.9))
    fig.subplots_adjust(left=0.1, right=0.98, top=0.93, bottom=0.14, hspace=0.36)
    for x in choose:
        p = raw / x["id"] / "guide.npy"
        if digest(p) != x["guideHistorySha256"]:
            raise ValueError("Unknown guide history")
        h = np.load(p, mmap_mode="r")
        label, color, style = (
            ("Folded, nominal", "#126782", "-")
            if x["layout"] == "folded" and not x["outwardPerSideMm"]
            else ("Folded, outboard", "#ac4420", "--")
            if x["outwardPerSideMm"]
            else ("Flat backing", "#684393", "-.")
        )
        # Preserve both maxima in each 0.5 ms interval rather than decimating spikes.
        bins = np.floor(h[:, 0] * 2000).astype(int)
        starts = np.r_[0, 1 + np.flatnonzero(np.diff(bins))]
        times = h[starts, 0] * 1000
        a.plot(
            times,
            np.maximum.reduceat(h[:, 1:3].max(axis=1), starts),
            color=color,
            linestyle=style,
            label=label,
            lw=1.5,
        )
        b.plot(
            times,
            1000 * np.maximum.reduceat(h[:, 9:11].max(axis=1), starts),
            color=color,
            linestyle=style,
            lw=1.5,
        )
    a.set(
        title="Guide force during the approach with the smaller right wheel",
        ylabel="Normal force (N)",
    )
    a.legend(loc="upper right", frameon=False)
    b.set(
        title="Largest lateral travel anywhere on either backing",
        ylabel="Displacement (mm)",
        xlabel="Time after chute lip (ms)",
        xlim=(90, 240),
    )
    for ax in (a, b):
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_ylim(bottom=0)
        ax.grid(axis="x", color="#e0e3e4", lw=0.7)
    fig.savefig(out, format="svg", metadata={"Date": None})
    out.write_text(
        "\n".join(line.rstrip() for line in out.read_text(encoding="utf8").splitlines())
        + "\n",
        encoding="utf8",
        newline="\n",
    )
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("results", type=Path)
    p.add_argument("raw", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    plot(a.results, a.raw, a.out)
