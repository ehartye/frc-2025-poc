"""Render original engineering sections from the project measurements and candidate trajectory."""
from pathlib import Path
import json
import math
import matplotlib as mpl
mpl.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon
from native_station import ROOT
from scripts import receiver_contact as st

HERE=Path(__file__).resolve().parent
r=json.loads((HERE/"transfer.json").read_bytes())
opening=r["opening_sections"];feed=r["feed"]
x0=opening["center_x_mm"];z0=feed["first_ramp_center_xz_m"][1]*1000
fig,(plan,side)=plt.subplots(1,2,figsize=(10.8,5.5),gridspec_kw={"width_ratios":[1,1.65]})
for ax in (plan,side):
    ax.set_facecolor("#f7f4eb");ax.set_aspect("equal");ax.grid(alpha=.14);ax.tick_params(labelsize=8)
fig.patch.set_facecolor("#f7f4eb")
for ax in (plan,side):ax.spines[["right","top"]].set_visible(False)
low,high=opening["human_side_face_x_mm"],opening["field_side_face_x_mm"]
for xx in (low-9,high):plan.add_patch(Rectangle((xx,-175),9,350,color="#607473"))
plan.add_patch(Rectangle((x0-57.15,-150.8125),114.3,301.625,color="#76aaaa",alpha=.45,label="Aligned pipe projection"))
# A yawed finite cylinder projects as a rotated rectangle in plan.
ang=math.radians(3)
coords=[(x0+x*math.cos(ang)+y*math.sin(ang),-x*math.sin(ang)+y*math.cos(ang)) for x,y in ((-57.15,-150.8125),(57.15,-150.8125),(57.15,150.8125),(-57.15,150.8125))]
plan.add_patch(Polygon(coords,fill=False,linestyle="--",linewidth=2,edgecolor="#a34635",label="3° yaw"))
plan.set_title("Entry from above",loc="left",fontweight="bold");plan.set_xlabel("Robotward X (mm)");plan.set_ylabel("Across chute (mm)");plan.set_xlim(low-25,high+25);plan.set_ylim(-190,190)
plan.legend(loc="lower center",bbox_to_anchor=(.5,-.36),frameon=False,fontsize=9)
# Centered path in the station XZ frame. The gray segments show registered contact surfaces.
lx,lz=st.LIP_X*1000,st.LIP_Z*1000
side.plot([lx-.4191*1000*math.cos(math.radians(35)),lx],[lz+.4191*1000*math.sin(math.radians(35)),lz],lw=4,color="#607473")
side.plot([low,low],[1191,1394],lw=3,color="#607473");side.plot([high,high],[1259,1532],lw=3,color="#607473")
side.plot([39.525,304.525],[880,880],lw=4,color="#607473")
side.plot([x0,x0,29.29358],[1550,z0,997.795],color="#12817f",lw=2,label="Pipe center")
state,tf,_=st.path_to_shelf(feed['lip_mps'])
arc=[st.flight_center(state,tf*i/60) for i in range(61)]
side.plot([v[0]*1000 for v in arc],[v[1]*1000 for v in arc],color="#12817f",lw=2)
for x,z in ((x0,1450),(x0,z0),(arc[-1][0]*1000,arc[-1][1]*1000)):
    side.add_patch(Circle((x,z),57.15,facecolor="#76aaaa",alpha=.4,edgecolor="#136663"))
side.annotate("1.697 m/s at lip",(29.3,997.8),xytext=(-170,895),arrowprops=dict(arrowstyle="->",color="#345b5b"),fontsize=9)
side.annotate("Receiver shelf",(180,880),xytext=(90,808),arrowprops=dict(arrowstyle="->",color="#345b5b"),fontsize=9)
side.set_title("Centered drop and first contact",loc="left",fontweight="bold");side.set_xlabel("Robotward X (mm)");side.set_ylabel("Above field floor (mm)");side.set_xlim(-405,325);side.set_ylim(770,1605)
fig.suptitle("Station supply | A 122 mm opening for a 114 mm pipe",fontsize=16,fontweight="bold",x=.06,ha="left");fig.subplots_adjust(left=.07,right=.99,top=.86,bottom=.23,wspace=.19)
mpl.rcParams["svg.hashsalt"]="station-supply-2025"
mpl.rcParams["svg.fonttype"]="none"
output=HERE/"handoff.svg"
fig.savefig(output,metadata={"Date":None})
output.write_text("\n".join(line.rstrip() for line in output.read_text(encoding="utf8").splitlines())+"\n",encoding="utf8",newline="\n")
