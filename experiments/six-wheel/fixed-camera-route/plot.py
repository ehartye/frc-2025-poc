"""Draw the issued centerlines in three orthographic views."""

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import numpy as np
from pathlib import Path
from paths import routes, points, DATA, POWER, PI, T, B, host

ROOT = Path(__file__).resolve().parent


def draw():
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.spines.right": False,
            "axes.spines.top": False,
            "axes.titlelocation": "left",
        }
    )
    fig, axes = plt.subplots(
        1, 3, figsize=(13, 5.1), width_ratios=[1.05, 1, 1.1], layout="constrained"
    )
    colors = {"data": "#176886", "pigtail": "#b44222", "power": "#b44222", "roof-host": "#368149"}
    labels = {
        "data": "Fixed data",
        "pigtail": "Fixed power pigtail",
        "power": "Fixed power pair",
        "roof-host": "Roof data to carrier",
    }
    rr, _ = routes()

    def plot(ax, k, i, j):
        ps = points(rr[k])
        return ax.plot(ps[:, i], ps[:, j], color=colors[k], lw=2, label=labels[k])[0]

    ax = axes[0]
    for k in rr:
        plot(ax, k, 0, 2)
    ax.add_patch(Rectangle((-337, 158), 20, 798, fc="#e1e7ea", ec="#87959e", zorder=0))
    ax.axhline(1059, color="#60666c", ls="--", lw=1)
    ax.text(-360, 1080, "1,059 mm stowed hull", fontsize=8)
    ax.set(
        title="Installed routes", xlabel="Chassis X (mm)", ylabel="Height Z (mm)", ylim=(126, 1125)
    )
    ax.set_aspect("equal")
    ax = axes[1]
    for k in ("data", "pigtail", "power"):
        plot(ax, k, 1, 2)
    ax.plot(
        [POWER[1], POWER[1]],
        [POWER[2], 1064.735],
        color="#978980",
        ls=":",
        label="Earlier straight exit",
    )
    for p, t in ((DATA, "J1"), (POWER, "J3")):
        ax.scatter(p[1], p[2], s=22, color="#25333b", zorder=4)
        ax.text(p[1] + 2, p[2] - 2, t, fontsize=8)
    ax.axhline(1059, color="#60666c", ls="--", lw=1)
    ax.annotate(
        "R21 power turn",
        xy=(-14, 1040),
        xytext=(-110, 1050),
        arrowprops={"arrowstyle": "-", "color": "#b44222"},
    )
    ax.set(
        title="Fixed camera, viewed from the front",
        xlabel="Chassis Y (mm)",
        ylabel="Height Z (mm)",
        xlim=(-132, 36),
        ylim=(930, 1075),
    )
    ax.set_aspect("equal")
    ax = axes[2]
    outline = np.array(
        [PI + x * T + y * B for x, y in ((-42.5, -28), (42.5, -28), (42.5, 28), (-42.5, 28))]
    )
    ax.add_patch(Polygon(outline[:, :2], closed=True, fc="#dbe7df", ec="#69856e", lw=1))
    ax.add_patch(Rectangle((-190, -202), 135, 85, ec="#9b8162", fc="#f2ebdf", lw=1))
    ax.text(-173, -163, "Converter\npanel", ha="left", fontsize=8)
    ax.text(-252, -193, "Pi 4B", ha="center")
    for k in ("data", "roof-host"):
        plot(ax, k, 0, 1)
    for k, h, style in (("data", 35, "-"), ("roof-host", 38, "--")):
        a = host(-202, -200, 0)
        b = a + h * T
        ax.plot([a[0], b[0]], [a[1], b[1]], c=colors[k], lw=5, ls=style)
    ax.set(
        title="Stacked host plugs, viewed from above",
        xlabel="Chassis X (mm)",
        ylabel="Chassis Y (mm)",
        xlim=(-313, 17),
        ylim=(-304, -145),
    )
    ax.set_aspect("equal")
    for ax in axes:
        ax.grid(alpha=0.16)
    handles = [
        plt.Line2D([], [], c=colors[k], lw=2, label=labels[k])
        for k in ("data", "power", "roof-host")
    ]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, frameon=False)
    plt.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "fixed-camera-route-2025"})
    fig.savefig(ROOT / "routing.svg", metadata={"Creator": "Matplotlib", "Date": None})
    output = ROOT / "routing.svg"
    lines = output.read_text(encoding="utf8").splitlines()
    output.write_text(
        "\n".join(line.rstrip() for line in lines) + "\n", encoding="utf8", newline="\n"
    )
    plt.close(fig)


if __name__ == "__main__":
    draw()
