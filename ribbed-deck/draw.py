"""Draw the underside and the transverse profile using the reopened panel dimensions."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

HERE=Path(__file__).resolve().parent
read=lambda n:json.loads((HERE/n).read_bytes())
def draw():
    plate=read("geometry.json")
    with plt.rc_context({"font.family":"DejaVu Sans","svg.fonttype":"none","svg.hashsalt":"integral-deck",
                         "axes.spines.right":False,"axes.spines.top":False,"font.size":10}):
        fig=plt.figure(figsize=(12,7.2),facecolor="#faf9f2")
        fig.text(.055,.93,"The deck from underneath",fontsize=25,fontweight="bold",color="#183e49")
        fig.text(.057,.875,"One machined panel carries the battery, mast feet and electronics on their original mounting plane.",fontsize=12)
        plan=fig.add_axes([.04,.19,.53,.61],facecolor="#faf9f2")
        plan.set_aspect("equal")
        def box(x,xx,y,yy,color):
            plan.add_patch(Rectangle((x,y),xx-x,yy-y,facecolor=color,edgecolor="none"))
        skin="#e4ebdc"; land="#9bc6b4"; rib="#286e70"; tie="#993b25"
        box(-346,346,-258,258,skin)
        for x,xx,y,yy in [*plate["upperBearingLands"].values(),(-346,-321,-258,258),(321,346,-258,258),
                            (-346,346,-258,-212),(-346,346,212,258)]:
            box(x,xx,y,yy,land)
        for x in plate["stationsMm"]:
            box(x-1.5,x+1.5,-226,226,rib)
        for x,ys in ((-339,(-225,72,225)),(339,(-225,-72,225))):
            plan.scatter([x]*3,ys,s=25,color=tie,zorder=3)
        plan.set(xlim=(-355,355),ylim=(-268,268),xlabel="Fore and aft (mm)",ylabel="Across the robot (mm)")
        plan.text(-207,-23,"battery tray",ha="center",bbox={"facecolor":"#f5fbef","edgecolor":"none","pad":2})
        fig.text(.59,.77,"Section through one rib",fontsize=15,fontweight="bold",color="#183e49")
        sec=fig.add_axes([.595,.56,.35,.14],facecolor="#faf9f2")
        y,d=np.array(plate["sectionDepthFromTopMm"]).T
        yy=np.r_[-232,-y[:0:-1],y,232]; dd=np.r_[4,d[:0:-1],d,4]
        sec.fill_between(yy,-dd,np.zeros(len(dd)),color=rib)
        sec.set(xlabel="Distance from centerline (mm)",ylabel="Depth (mm)",ylim=(-20,2))
        fig.text(.59,.43,"Why the ribs taper",fontsize=15,fontweight="bold",color="#183e49")
        fig.text(.59,.388,"Each 3 mm web is deepest near the middle of the span.",fontsize=10)
        fig.text(.59,.35,"At each rail the web rises into the 4 mm bearing land.",fontsize=10)
        fig.text(.59,.285,"The center of the plate is 18 mm deep at a rib.",fontsize=10)
        fig.text(.59,.247,f"The skin between ribs is {plate['skinMm']:g} mm thick.",fontsize=10)
        fig.text(.059,.075,"Light green   4 mm mounting and rail lands",color="#376d62",fontsize=11)
        fig.text(.41,.075,"Dark green   13 integral ribs",color=rib,fontsize=11)
        fig.text(.70,.075,"Red   six deck tie holes",color=tie,fontsize=11)
        fig.savefig(HERE/"underside.svg",metadata={"Date":None})
        svg=HERE/"underside.svg"
        svg.write_text("\n".join(line.rstrip() for line in svg.read_text(encoding="utf8").splitlines())+"\n",
                       encoding="utf8",newline="\n")
        plt.close(fig)
if __name__=="__main__":
    draw()
