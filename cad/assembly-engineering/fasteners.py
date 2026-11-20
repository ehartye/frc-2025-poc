"""Separate make-stock from bought threaded hardware in a cold shop pick list."""
import csv
import re
from pathlib import Path
from installed_audit import HERE, ROOT
from materials import rows, write_csv

def picklist():
    mast=ROOT/"experiments/six-wheel/mast-attachment"
    doc={
      "chassis":"experiments/swerve-drive/packaging/README.md",
      "carriage":"experiments/six-wheel/carriage-support/README.md",
      "guide":"experiments/six-wheel/guide-mounts/README.md",
      "bumper":"experiments/six-wheel/design/bumper-kit/README.md",
      "mast":"experiments/six-wheel/mast-attachment/account.py",
      "deck":"experiments/six-wheel/deck-joints/README.md",
      "rear":"experiments/swerve-drive/packaging/README.md",
      "jack":"cad/assembly-engineering/shop.md"}
    out=[]
    for row in rows("cad/assembly-engineering/material-register.csv"):
        k=row["id"];s=row["source"]
        if not re.search(r"screw|locknut|(?<![a-z])nut|washer|bolt|M3-platform|M3-post-base",k,re.I):continue
        what,ref,status,access="","", "drawing selection",""
        if re.search(r"-tab-(screw|locknut)",k):
            what="Bolt Depot 8232, #10-32 × 2-3/4 in socket screw, ASTM A574" if "screw" in k else "Bolt Depot 2623, regular #10-32 UNF zinc nylon-insert locknut"
            ref=doc["chassis"];status="16 measured tab stations; 58.74875 mm tight grip, 4.77665 mm beyond maximum nut";access="Before closing deck, support module at its two tab datums; outboard and inboard saddles. Both bumpers and corner tie come off for extraction."
        elif re.search(r"^(FL|FR|RL|RR)-[AB]-rail-screw",k):
            what="M6 x 1 socket head, ISO 4762, class 12.9";ref=doc["chassis"];status="choose length after gauging 13 mm boss thread";access="Inboard 6 mm shaft gauge/62 mm insert and 40 mm withdrawal screened; select actual hex key on received head."
        elif "bumper-" in k:
            what="ISO 7089 M8, 16 x 8.4 x 1.6 mm washer" if "washer" in k else ("ISO 4762 M8 x 35 mm" if "-end-" in k else "ISO 4762 M8 x 40 mm")
            ref=doc["bumper"];access="Install bumper backing before foam. Turn from inside the chassis rail, measure boss engagement and sleeve compression."
        elif re.match(r"^[LR]-[0-9]+-(screw|washer|nut)$",k):
            what={"screw":"ISO 10642 M6 x 120 mm, class 10.9, actual 90-degree head gauge","washer":"ISO 7089 M6, OD12 x ID6.4 x 1.6","nut":"ISO 10511 M6 low-profile prevailing-torque nut, 10 mm across flats, 6 mm received height cap"}[k.split("-")[-1]]
            ref=doc["jack"];access="4 mm hex / Ø4.8 shank top; 10 mm thin socket / Ø15 OD underneath."
        elif k.startswith("M3-platform") or k.startswith("M3-post-base"):
            what="ISO 10642 M3 x 10 mm, 90-degree flat head";ref=doc["jack"];access="Tap posts M3 x 0.5 both ends, lower screws installed on bench before structural deck. Match actual head to seat."
        elif k.startswith("battery-"):
            if "-foot-" in k:what="M4 x 16 ISO 4762 screw" if "screw" in k else "ISO 7089 M4 steel washer" if "washer" in k else "M4 low-profile prevailing locknut, 7 mm AF"
            else:what="M5 steel washer ID5.4 OD12 x 1" if "washer" in k else "M5 prevailing locknut 8 AF, 5 mm height"
            ref=doc["jack"];access="Bolt angles and tray through structural deck before setting deck on chassis. Disconnect and cover terminals before battery removal."
        elif "mast-attachment/" in s:
            ref=doc["mast"];what="M6 DIN 985 steel class 8 prevailing nut" if "nut" in k else "ISO 7089 M6 flat washer BLOHM B0112.06" if "washer" in k else "ISO-class 8.8 M6 fastener, FABORY 01000060040 or 01210060020 by long/short position"
            if k.endswith("-screw"):
                index=int(k.split("-")[-2])
                what="M6 x 40, class 8.8, FABORY 01000060040, finish at 37.5 mm" if index>=4 else "M6 x 20, class 8.8, FABORY 01210060020, finish at 18.5 mm"
            status="order size selected; check finished grip";access="Follow drilled mast joint socket-and-washer stack from the indicated side."
        elif "carriage-support" in s:
            if "-mast-" in k and k.endswith("-screw"):what="M6 x 45 class 8.8 hex head; trim to 43 mm"
            elif k.endswith("rear-screw"):what="M6 x 25 class 8.8 hex head"
            elif k.endswith("flush-screw"):what="ISO 10642 M5 x 25; trim to 23 mm"
            else:what=("M5" if "-front-" in k else "M6")+(" plain steel washer" if k.endswith("washer") else " steel prevailing locknut")
            ref=doc["carriage"];status="order size selected; gauge assembled fit";access="At X167 gauge flat-head seat. At mast gauge sleeve/end clamping faces first."
        elif "rear-electronics" in s:
            ref=doc["rear"];what="M4 x 40 through screw, ISO 7089 washer or M4 prevailing nut by ID";access="From above the PCB carrier, remove board before bridge or rear module."
        elif "guide-mounts" in s:
            if "face-screw-" in k:what="ISO 10642 M3 x 16 countersunk steel screw"
            else:what="M4 x 65 steel through screw" if k.endswith("-screw") else "M4 steel washer" if k.endswith("-washer") else "M4 steel prevailing locknut"
            ref=doc["guide"];status="order size selected; gauge actual lot";access="Remove the guide module on its outboard side. Hold the root and preserve the two fitted 50 mm crush sleeves."
        elif "deck-joints" in s:
            what="M5 DIN 982 high steel locknut, Wuerth 03715, 8 AF 6.3 mm high" if "locknut" in k else "M5 ISO 7089 flat washer"
            ref=doc["deck"];status="order size selected";access="Fit 8 AF socket from above and 16 mm wide holding fork from below; gauge 4 mm minimum load-bearing metal thread."
        else:
            ref=s;what=row["item"];status="article/size/finish or access unresolved";access="Treat as a separate purchased stock item pending exact drawing; mass based on CAD is provisional."
        out.append(dict(id=k,quantity=1,article=what,selection=status,assembly=access,source=ref))
    write_csv("fastener-pick-list.csv",out)
if __name__=="__main__":picklist()

