"""Regenerate the side elevation using the same states as the receiver screen."""
import argparse
import io
import math
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

from receiver_contact import (
    DEFAULT_SPEEDS_MPS, EXTENDED_FRONT, LIP_X, LIP_Z, PANEL_RIGHT, PANEL_TOP,
    RADIUS_M, SHELF_LENGTH, SHELF_TOP, SLOPE_RAD, path_to_shelf, stand_off_screen,
)


def draw(output):
    mpl.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "receiver-contact"})
    fig, ax = plt.subplots(figsize=(8.5, 5.5), layout="constrained")
    mm = 1000
    ax.set_aspect("equal")
    ax.set_axisbelow(True)
    ax.grid(color="#dee4e8", linewidth=0.6)
    ax.plot([-60, LIP_X * mm], [LIP_Z * mm + (60 + LIP_X * mm) * math.tan(SLOPE_RAD),
                                LIP_Z * mm], color="#263844", linewidth=3)
    ax.text(-59, 1006, "Ideal chute tangent", fontsize=10)
    ax.add_patch(Rectangle((-6.35, 845), 6.35 + PANEL_RIGHT * mm, PANEL_TOP * mm - 845,
                           color="#73848f"))
    ax.text(-56, 898, "Lower panel", fontsize=10)
    ax.add_patch(Rectangle((EXTENDED_FRONT * mm, SHELF_TOP * mm - 15), SHELF_LENGTH * mm, 15,
                           color="#354a50"))
    ax.text(76, 868, "Extended shelf", color="white", fontsize=10)
    colors = ("#984522", "#23775f", "#334f9b")
    for speed, color in zip(DEFAULT_SPEEDS_MPS, colors):
        _, _, path = path_to_shelf(speed)
        ax.plot([x * mm for x, _ in path], [z * mm for _, z in path],
                color=color, lw=2, label=f"{speed:g} m/s entry")
        x, z = path[-1]
        ax.plot([x * mm], [z * mm], "o", color=color, markersize=4)
        ax.plot([x * mm], [SHELF_TOP * mm], "o", color=color, markersize=4)
        if speed == 0.5:
            ax.add_patch(Circle((x * mm, z * mm), RADIUS_M * mm, fill=False,
                                linestyle="--", linewidth=1.3, color=color))
    ax.set(xlim=(-62, 170), ylim=(853, 1020), xlabel="Robotward distance from station datum (mm)",
           ylabel="Height (mm)", title="First shelf contact for a centered CORAL")
    ax.legend(loc="upper right", frameon=False, fontsize=9)
    save_svg(fig, output)


def draw_stand_off(output):
    mpl.rcParams.update({"svg.fonttype": "none", "svg.hashsalt": "receiver-contact"})
    fig, ax = plt.subplots(figsize=(9, 3.2), layout="constrained")
    for row, speed in enumerate(DEFAULT_SPEEDS_MPS):
        state = stand_off_screen(speed, 0)
        top = state["last_top_contact_stand_off_mm"]
        corner = state["last_upper_corner_contact_stand_off_mm"]
        for start, end, label, color in (
            (0, top, "Top", "#28624e"), (top, corner, "Corner", "#875031"),
            (corner, 220, "Clear*", "#445366"),
        ):
            ax.barh(row, end - start, left=start, height=0.58, color=color)
            ax.text((start + end) / 2, row, label, ha="center", va="center", color="white")
    ax.set_yticks(range(len(DEFAULT_SPEEDS_MPS)), [f"{v:g} m/s" for v in DEFAULT_SPEEDS_MPS])
    ax.invert_yaxis()
    ax.set(xlim=(0, 220), xlabel="Additional robot stand-off (mm)",
           title="First contact as the receiver moves away from the station")
    ax.set_xticks(range(0, 221, 20))
    ax.spines[["top", "left", "right"]].set_visible(False)
    fig.supxlabel("*Clear through the instant the pipe center reaches shelf-top height.", fontsize=9)
    save_svg(fig, output)


def save_svg(fig, output):
    svg = io.StringIO()
    fig.savefig(svg, format="svg", metadata={"Date": None})
    lines = (line.rstrip() for line in svg.getvalue().splitlines())
    Path(output).write_bytes(("\n".join(lines) + "\n").encode("utf-8"))
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output")
    parser.add_argument("--stand-off", action="store_true", help="plot first contact versus stand-off")
    args = parser.parse_args()
    if args.stand_off:
        draw_stand_off(args.output)
    else:
        draw(args.output)
