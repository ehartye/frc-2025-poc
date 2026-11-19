"""Draw the experimental shelf and the effect of plate support on two signed arrivals."""

from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
from installed import SIDE_POCKETS, STATIONS
from tray import LANDS, OUTLINE, RIB_X, RIB_Y

HERE = Path(__file__).resolve().parent


def rounded(ax, x, y, u, v, r, **kwargs):
    ax.add_patch(
        FancyBboxPatch(
            (x, y), u - x, v - y, boxstyle=f"round,pad=0,rounding_size={r}", **kwargs
        )
    )


def main():
    plt.rcParams.update(
        {
            "svg.fonttype": "none",
            "svg.hashsalt": "receiving-shelf-trial",
            "font.family": "DejaVu Sans",
            "font.size": 9,
        }
    )
    fig, (a, b) = plt.subplots(
        1, 2, figsize=(10.6, 6.4), gridspec_kw={"width_ratios": [1.35, 1]}
    )
    fig.patch.set_facecolor("#ffffff")
    a.set_aspect("equal")
    rounded(a, -100, -210, 165, 210, 6, color="#dde4df")
    for x, y, u, v in SIDE_POCKETS:
        rounded(a, x, y, u, v, 3, color="#c4d0c5")
    for x, y, u, v in OUTLINE:
        rounded(a, x, y, u, v, 3, color="#a3c4d2")
    for x in RIB_X:
        a.plot([x, x], [-165, 165], color="#396176", linewidth=1.15)
    for y in RIB_Y:
        a.plot([-92, 153], [y, y], color="#396176", linewidth=1.0)
    for x, y, u, v in LANDS:
        a.add_patch(Rectangle((x, y), u - x, v - y, color="#799cae"))
    for x in (-80, 140):
        for y in (-180, 180):
            a.add_patch(Circle((x, y), 2.65, facecolor="white", edgecolor="#263940"))
    for x in (40, 100):
        a.add_patch(Circle((x, 0), 6, color="white", ec="#263940"))
    for x, y in STATIONS:
        a.plot([-99, x - 3], [y, y], color="#f9f5e8", linewidth=2.0)
        a.add_patch(Circle((x, y), 4, color="#a54231"))
    a.set(
        xlim=(-113, 177),
        ylim=(-221, 221),
        xlabel="Chassis X (mm, forward →)",
        ylabel="Chassis Y (mm)",
    )
    a.set_title("Receiver underside", loc="left", fontweight="bold")
    b.set_title("Through-bore mounting at stow", loc="left", fontweight="bold")
    b.set_aspect("equal")
    b.add_patch(Rectangle((-21, 496), 42, 4, color="#dde4df"))
    b.add_patch(Rectangle((-21, 494.4), 42, 1.6, color="#7396a8"))
    b.add_patch(Rectangle((-5, 485), 10, 9.4, color="#7396a8"))
    b.add_patch(Rectangle((-4, 423.5), 8, 48.3, color="#6e7880"))
    b.add_patch(Rectangle((-1.25, 471.8), 2.5, 11, color="#6e7880"))
    b.add_patch(Rectangle((-3.2, 482.8), 6.4, 2.1, color="#a54231"))
    for x in (-21, 4.15):
        b.add_patch(Rectangle((x, 433), 16.85, 5, color="#7396a8"))
    for z in (430, 438):
        b.add_patch(Rectangle((-5.75, z), 11.5, 3, color="#465057"))
    for label, x, z, tx, tz in [
        ("Original PE face", 14, 498, 26, 503),
        ("Milled landing", 4, 490, 26, 494),
        ("0.10 mm trial gap", 2, 484.95, 26, 483),
        ("MA30EUM", 4, 456, 26, 460),
        ("Two KM8 nuts", 5.5, 439, 26, 444),
        ("5 mm bridge", 13, 435, 26, 434),
        ("Adjust from below", 0, 423.5, 26, 420),
    ]:
        b.annotate(
            label,
            xy=(x, z),
            xytext=(tx, tz),
            va="center",
            color="#243943",
            arrowprops={"arrowstyle": "-", "color": "#667982", "lw": 0.9},
        )
    b.set(xlim=(-23, 75), ylim=(415, 510), ylabel="Installed Z (mm)")
    b.set_xticks([])
    for ax in (a, b):
        ax.spines[["right", "top"]].set_visible(False)
        ax.tick_params(color="#84939a", labelsize=8)
    fig.subplots_adjust(left=0.08, right=0.96, top=0.91, bottom=0.12, wspace=0.24)
    path = HERE / "installation.svg"
    fig.savefig(path, format="svg", metadata={"Date": None})
    text = path.read_text(encoding="utf8")
    path.write_text(
        "\n".join(l.rstrip() for l in text.splitlines()) + "\n",
        encoding="utf8",
        newline="\n",
    )


if __name__ == "__main__":
    main()
