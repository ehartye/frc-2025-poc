"""Draw the public timing chart from the signed receiver study. No field or supplier imagery is used."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "experiments/swerve-drive/motion/receiver-witness.json"
OUTPUT = ROOT / "docs/robot/station-witness.png"


def read_rows(source):
    receipt = json.loads(source.read_bytes())
    if receipt["schema"] != "reefscape-r4-station-custody/1":
        raise ValueError("Select the registered R4 receiver study")
    cases = {(r["case"], r["stepUs"], r["finish"]): r for r in receipt["cases"]}
    layouts = []
    for sign in ("plus", "minus"):
        for step in (31.25, 15.625):
            clear = cases[sign, step, "guarded"]
            matched = cases[sign, step, "matched"]
            required = clear["gate"]["eligibleCapturesJavaMs"]
            if len(required) != 5 or len(set(required)) != 5:
                raise ValueError("Expected five distinct timely stereo observations")
            rows = []
            for case, gate_id in ((clear, "gate"), (matched, "gate"), (clear, "late")):
                gate = case[gate_id]
                frames = {f["captureJavaMs"]: f for f in case["frames"]}
                points = []
                for stamp in required:
                    frame = frames[stamp]
                    dt = round(1000*(gate["serviceWallS"][str(stamp)] - gate["firstEligibleWheelServiceWallS"]), 6)
                    if frame["fixed"] != "position-witness":
                        raise ValueError("Fixed camera no longer confirms position")
                    if frame["roof"] == "missing-rear-edge":
                        state = "edge"
                    elif frame["roof"] == "inside":
                        state = "late" if dt > 0 else "ready"
                    else:
                        raise ValueError("Unknown roof observation")
                    if (state == "ready") != (stamp in gate["eligibleCapturesJavaMs"]):
                        raise ValueError("Camera evidence and controller decision disagree")
                    points.append((dt, state))
                ready = sum(s == "ready" for _,s in points)
                if (ready == len(required)) != gate["accepted"]:
                    raise ValueError("Unexpected controller decision")
                rows.append((points, ready))
            layouts.append((clear["gate"]["firstEligibleWheelServiceWallS"], rows))
    if not all(layout == layouts[0] for layout in layouts):
        raise ValueError("Signed placements or refinements differ; draw them separately")
    return layouts[0]


def draw(source, output):
    tick, rows = read_rows(source)
    ink, muted, paper = "#17334c", "#425d71", "#f5f8f8"
    fig = plt.figure(figsize=(11, 5.3), facecolor=paper)
    fig.text(.042,.895,"Can the robot confirm its grip?", size=27, weight="bold", color=ink)
    fig.text(.044,.81,"Both stereo heads must agree on five captures before the next wheel update.",
             size=13, color=muted)
    ax = fig.add_axes([.32, .26, .50, .46], facecolor=paper)
    ax.set_xlim(-83, 31)
    ax.set_ylim(-.6, 2.6)
    ax.axvspan(0, 31, color="#e3eaf0", zorder=0)
    ax.axvline(0, color=ink, linewidth=1.3)
    ax.text(0, 2.63, f"{tick:.3f} s", ha="center", color=ink, fontsize=11, weight="bold")
    marks = {"ready": ("o","#0a776a"), "edge": ("X","#a34632"), "late": ("D","#8a6108")}
    titles = [("Visible rear edge","15 ms image delay"),("Matching pipe and shelf","15 ms image delay"),
              ("Visible rear edge","45 ms image delay")]
    for index, (points, ready) in enumerate(rows):
        y = 2-index
        baseline = ax.get_yaxis_transform()
        ax.text(-.55, y+.03, titles[index][0], transform=baseline, color=ink, size=13, weight="bold")
        ax.text(-.55, y-.27, titles[index][1], transform=baseline, color=muted, size=10.5)
        ax.hlines(y, -83, 31, color="#c7d4d9", linewidth=.8)
        for time, state in points:
            shape, color = marks[state]
            ax.scatter(time, y, s=100, marker=shape, color=color, zorder=3)
        ax.text(1.06,y+.03,f"{ready} of 5",transform=baseline,size=15,weight="bold",color=ink)
        ax.text(1.06,y-.27,"Grip confirmed" if ready == 5 else "Hold",transform=baseline,size=10.5,color=muted)
    ax.set_yticks([])
    ax.set_xticks([-75,-50,-25,0,25])
    ax.tick_params(axis="x",length=0,labelcolor=muted)
    ax.set_xlabel("Milliseconds relative to the wheel update",color=muted,labelpad=7)
    ax.spines[:].set_visible(False)
    keys = [Line2D([],[],marker=m,color=c,linestyle="",markersize=8) for m,c in marks.values()]
    fig.legend(keys, ["Both heads agree","Rear edge missing","Arrived after the update"],
               loc="lower center",bbox_to_anchor=(.5,.043),ncol=3,frameon=False,labelcolor=ink)
    fig.savefig(output,dpi=150,facecolor=paper)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,default=SOURCE)
    parser.add_argument("--output",type=Path,default=OUTPUT)
    opts = parser.parse_args()
    draw(opts.source,opts.output)
