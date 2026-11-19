"""Plot the long-belt counterexample and the short-belt whole-robot observation clock."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("svg")
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent


def render():
    plt.rcParams.update({"font.family":"DejaVu Sans","svg.fonttype":"none","svg.hashsalt":"registered-receiver"})
    long=json.loads((HERE/"combined-departure-long.json").read_text())
    short=json.loads((HERE/"combined-departure-short.json").read_text())
    clock=json.loads((HERE/"short-vision-clock.json").read_text())["runs"][0]
    fig,(overlap,timing)=plt.subplots(1,2,figsize=(11.5,4.5),gridspec_kw={"width_ratios":[1,1.2]})
    fig.patch.set_facecolor("#fffdf7")
    for axis in (overlap,timing):
        axis.set_facecolor("#fffdf7")
        axis.spines[["top","right","left"]].set_visible(False)
        axis.grid(axis="x",alpha=.2)
        axis.tick_params(length=0)
        axis.set_xlabel("Milliseconds after the modeled shelf event",fontsize=9,labelpad=8)
    for data,color,label in [(long,"#b45336","537 mm belt"),(short,"#007568","420 mm belt")]:
        rows=data["departures"][0]["early_native_pipe_wall_snapshots"]
        overlap.plot([r["after_shelf_ms"] for r in rows],
                     [max(r["forward_endpoint_inward_grip_overlap_mm3"].values(),default=0) for r in rows],
                     marker="o",markersize=4,linewidth=2,color=color,label=label)
    overlap.set_title("Pipe overlap with either belt",loc="left",fontweight="bold",fontsize=13)
    overlap.set_ylabel("Material envelope overlap (mm³)",fontsize=9)
    overlap.legend(frameon=False,loc="upper right",fontsize=9)
    clear=clock["all_tight_zero_wait_onsets_combined_geometry"]["last_possible_contact_after_shelf_ms"]
    records=clock["budgets"]["0.1"]["0"]["exposures"]
    exposure=[min(r["exposure_after_shelf_ms"] for r in records),max(r["exposure_after_shelf_ms"] for r in records)]
    labels=["Possible body contact","Qualified exposures","Motion · zero wait","Motion · 50 ms wait"]
    ranges=[[0,clear],exposure,clock["budgets"]["0.1"]["0"]["onset_after_shelf_truth_ms"],
            clock["budgets"]["0.1"]["50"]["onset_after_shelf_truth_ms"]]
    for i,((start,end),color) in enumerate(zip(ranges,["#9b8458","#4689a2","#007568","#b45336"],strict=True)):
        timing.barh(i,end-start,left=start,color=color,height=.4)
        timing.text((start+end)/2,i-.29,f"{start:g}–{end:g} ms",ha="center",va="bottom",fontsize=8)
    timing.set_yticks(range(4),labels,fontsize=9)
    timing.set_ylim(3.6,-.85)
    timing.set_xlim(-2,168)
    timing.set_title("Short receiver · projected stereo at ±0.10 pixel",loc="left",fontweight="bold",fontsize=12)
    fig.suptitle("When can the receiver withdraw?",x=.055,ha="left",fontsize=21,fontweight="bold",color="#183935")
    fig.subplots_adjust(left=.08,right=.975,bottom=.19,top=.77,wspace=.42)
    fig.savefig(HERE/"release-window.svg",metadata={"Date":None,"Title":"Modeled receiver release window"})
    plt.close(fig)


if __name__=="__main__":
    render()
