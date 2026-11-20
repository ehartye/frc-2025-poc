"""Build the inspection bill from installed bodies, make-stock recipes and sealed articles.

A supplier's solid envelope can have a very different density and volume from its contents.
Substitute complete-unit weights once; retain the native material property separately.
"""

import collections
import csv
import json
import re
from pathlib import Path

from installed_audit import HERE, ROOT, sha
from clearance import base

SIX="experiments/six-wheel/"
SW="experiments/swerve-drive/"
MOUNT=SW+"packaging/mount-budget.json"
BUMPER=SIX+"design/bumper-kit/mass.csv"
DRIVE=SIX+"design/receiver-drives/bom.csv"
BUY=SIX+"design/installed-procurement/installed.csv"
INSPECT=SIX+"design/electrical-inspection/inspection.py"
HARNESS=SIX+"fixed-camera-route/mass-input.json"
AL=("Aluminum 6061",2700,2850)
STEEL=("Steel, Alloy",7730,8050)
AL7075=("Aluminum 7075",2810,2920)
NYLON=("Nylon 6/6",1140,1250)
PE=("Polyethylene, High Density",950,990)
BRONZE=("Bronze",8800,9000)
ACETAL=("Acetal Resin, Black",1410,1460)
ABS=("ABS Plastic",1060,1100)
FR4=("FR4",1850,2000)
RUBBER=("Rubber, Black",1150,1300)
FABRIC=("PE coated cloth, areal equivalent",700,800)
FOAM=("Expanded polyethylene, 37 kg per cubic metre",37,45)
MARKING=("Bumper marking, thickness-equivalent",900,1300)
PVC=("PVC, Unplasticized",1400,1460)


def rows(file):
    return list(csv.DictReader((ROOT/file).open(encoding="utf-8-sig")))


def load_json(file):
    return json.loads((ROOT/file).read_bytes())


def write_csv(name,items):
    with (HERE/name).open("w", encoding="utf8", newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(items[0]),lineterminator="\n")
        writer.writeheader()
        writer.writerows(items)


def assemble():
    source=rows("cad/assembly-engineering/installed-bodies.csv")
    bumper={r["solid"].replace("-blue","-red"):r for r in rows(BUMPER)}
    receiver={r["id"]:r for r in rows(DRIVE)}
    procurement={r["redId"]:r for r in rows(BUY)}
    guide=load_json(SIX+"fixed-camera-route/guides.json")
    changes=load_json("cad/assembly-engineering/interfaces.json")
    step=HERE/"station-and-jack-interfaces.step"
    if sha(step)!=changes["outputSha256"]: raise ValueError("Interface register differs")
    altered=base.named(step)
    base_rows={r["id"]:r for r in source}
    for k in changes["fixedHeight"]["removed"]: del base_rows[k]
    for k,s in altered.items():
        if k not in base_rows:
            base_rows[k]=dict(id=k,representation="machined",appearanceRole="",cadSource="cad/assembly-engineering/attachments.py")
        base_rows[k]["volumeMm3"]=base.cad.volume(s)[0]
    base_rows["Practice CORAL tube"]=dict(id="Practice CORAL tube",volumeMm3=649551.4844,appearanceRole="game-piece",cadSource="cad/swerve-captive/exchange.py",representation="demonstration")
    ledger=[]
    purchased={}
    def group(key,description,kg,basis):
        record=dict(instance=key,description=description,upperKg=kg,source=basis)
        if key in purchased and purchased[key]!=record:raise ValueError(key)
        purchased[key]=record
        return key
    for k,r in sorted(base_rows.items()):
        source=r["cadSource"]
        volume=float(r["volumeMm3"])
        region="inspection"
        identity=procurement.get(k,{}).get("item",k)
        disposition="made-stock"
        stock=AL
        article=""
        basis=source
        # Classify named physical stock independently of public render color.
        if k in bumper:
            b=bumper[k]
            stock={"alloy":AL,"steel":STEEL,"foam":FOAM,"blue":FABRIC,"red":FABRIC}[b["material"]]
            region="removable bumper" if b["assignment"]=="removable" else region
            basis=BUMPER
        elif r["appearanceRole"]=="bumper-lettering":
            stock=MARKING
            region="removable bumper"
        elif k=="Practice CORAL tube":
            stock=PVC
            region="demonstration"
            disposition="non-installed"
        elif r["representation"]=="appearance-overlay":
            stock=MARKING
            disposition="surface treatment"
        elif "-display-" in k:
            corner=k[:2]
            article=group(corner+"-module","REV-21-3005 v1.1, 12T, REV-21-3004 wheel, NEO v1.1 and 550",1.806,SW+"power/installed.json")
            stock=RUBBER if "tread" in k else AL
        elif k in receiver:
            a=receiver[k]
            text=a["description"].lower()
            identity=a["description"]
            basis=DRIVE
            if a["representation"] in {"selected-article-envelope","bearing-envelope","spring-envelope","belt-envelope","sensor-reserve","measurement-line"}:
                article=group(k,a["description"],float(a["highMassKg"]),basis)
                stock=RUBBER if "belt" in a["representation"] else STEEL if a["representation"] in {"bearing-envelope","spring-envelope"} else FR4 if "sensor" in a["representation"] else AL
            elif "steel" in text:stock=STEEL
            elif "uhmw" in text:stock=PE
        elif k=="np18-battery":
            region="removable battery"
            stock=ABS
            article=group(k,"EnerSys Genesis NP18-12",6.17,INSPECT)
        elif k=="rev-pdh":
            stock=ABS
            article=group(k,"REV-11-1850 power distribution hub, catalog 517 g plus 3% receiving margin",.533,"https://www.revrobotics.com/rev-11-1850/")
        elif k in {"rio2-envelope","vh109-reserve","signal-lamp-public-envelope","main-breaker"}:
            stock=ABS
            # Preserve the original 1.5 kg total as separate explicitly bounded
            # receivers; catalog numbers are identified separately from these caps.
            label,cap,reference={
                "rio2-envelope":("NI roboRIO 2.0, am-3000a; project receiving allowance",1.07,"https://andymark.com/products/ni-roborio-2-0"),
                "vh109-reserve":("VH-109 version 1; 0.4 lb catalog plus receiving reserve",.210,"https://wcproducts.info/files/frc/drawings/VH-109%20Public%202024-01-02.pdf"),
                "signal-lamp-public-envelope":("am-3583 indicator assembly; nominal 0.1085 lb without wiring",.055,"https://andymark.com/products/robot-signal-light"),
                "main-breaker":("120 A breaker with fastening and terminal allowance; exact stud-series drawing required",.165,INSPECT)}[k]
            article=group(k,label,cap,reference)
        elif k.startswith("spark-") or re.fullmatch(r"(FL|FR|RL|RR)-(drive|steer)-controller",k):
            stock=ABS
            article=group(k,"REV-11-2158 SPARK MAX with included leads; 113.3 g catalog with receiving allowance",.120,"https://www.revrobotics.com/rev-11-2158/")
        elif k in {"fixed-camera-public-envelope","moving-camera-public-envelope"}:
            stock=AL
            article=group(k,"Luxonis OAK-D Lite fixed-focus stereo camera, complete unit receiving allowance",.15,SIX+"moving-observer/README.md")
        elif k in {"sp1-supplier-reservation","sp1-eye-reservation","sp1-live-cable"}:
            stock=AL
            article=group("SP1","SP1-4-A sensor including eye/line and factory cable",.190,SIX+"integrated-slide/README.md")
        elif k in {"receiver-absorber-0","receiver-absorber-1"} or k.startswith(("receiver-absorber-piston-","receiver-absorber-cap-")):
            stock=STEEL
            article=group("plunger-"+k[-1],"One complete shock absorber with lock rings, receiving allowance",.065,SIX+"shelf-structure/damper_fit.py")
        elif k.endswith("drive and pawl envelope"):
            stock=AL
            article=group(k,"Reserved complete screw drive and independent catch package",.8,"cad/shallow-cage/installed.py")
        elif k.endswith("-servo-envelope"):
            stock=ABS
            article=group(k,"Capture servo, complete assembly receiving allowance",.080,SW+"retention/README.md")
        elif re.search(r"wire--converter",k):
            stock=FR4
            article=group(k,"DC converter and terminations (isolated enclosure)",.1,SIX+"fixed-camera-route/README.md")
        elif k.startswith("wire--") or k.startswith("usb-"):
            stock=ABS
            article=group("camera-leads","Fixed adapter and leads, roof factory lead, fitted signal and power tails",.275,HARNESS)
        elif k=="host-board-and-components":
            stock=FR4
            article=group(k,"Raspberry Pi 4 and integral sockets",.075,SIX+"fixed-camera-route/README.md")
        elif k.startswith("observer-moving-pad") or k.startswith("qtrx-") or "angle-board" in k or k in {"slide-extended-switch","slide-retracted-switch"}:
            stock=FR4
            article=group(k,identity,.015,SIX+"design/installed-procurement/installed.csv")
        elif r["appearanceRole"]=="jack":
            stock=BRONZE if "travelling nut" in k else STEEL if any(v in k for v in ("screw core","shoe","dovetail","Bearing pad","Latch stem")) else AL
        elif k in {"support-L-ribbed-bracket","support-R-ribbed-bracket","mast-base-L-splice","mast-base-R-splice"}:
            stock=AL7075
        elif k.startswith("battery-") and k.endswith("tie-rod"):stock=STEEL
        elif k=="r6-polymer-facing" or k.endswith("-liner"):
            stock=PE
        elif ("welded-thread-boss" in k or re.search(r"(-sleeve|-boss|M[34]-platform)",k) and "screw" not in k):
            stock=AL
        elif k.startswith("M3-") or re.search(r"(-screw|-nut|-washer|-locknut|-bolt|-dowel|-crosspin|threaded-pin|M[234568](?:x|[- ]|$)|-shaft|-magnet|-bushing|-bearing|height-[LR]-pin|^pin-[LR]|-clevis-pin|service-pin(?!-rail|-bushing))",k,re.I) or r["appearanceRole"] in {"steel","fastener","shaft"}:
            stock=STEEL
        elif k in guide["supportBlanks"]:stock=AL
        elif k in guide["removableTroughs"]:stock=NYLON
        elif k.startswith(("mast-shoe-","data-trough-","pigtail-trough-","power-trough-","roof-host-trough-")):
            stock=NYLON
        elif k.startswith("support--transition-board"):stock=FR4
        elif k.startswith("retainer-") and any(v in k for v in ("tube","rim-probe","pinion","rack-and-bar")):stock=STEEL
        elif k.startswith("head-") or k.startswith("transition-support"):stock=NYLON
        if article:
            disposition="sealed article or routed assembly"
        nominal=volume*stock[1]*1e-9
        upper=0 if article or region=="demonstration" else volume*stock[2]*1e-9
        ledger.append(dict(id=k,instance=article or (k if region!="demonstration" else ""),kind=disposition,
            scope=region,physicalDisplayMaterial=stock[0],nativeNominalDensityKgM3=stock[1],designStockMaxDensityKgM3=stock[2],
            volumeMm3=round(volume,4),displayStockKg=round(nominal,7),inspectionStockUpperKg=round(upper,7),
            item=identity,source=basis))
    # Explicit residual stock; retain the same unitemized allowance as the inspected project.
    group("unitemized-receiving","All unresolved mounting, manufacturing and small-parts reserve",5.0,INSPECT)
    group("drive-harness","Four issued corner-specific finished branch cuts including connectors and protections",1.492097135,SW+"mass-closure/recipe.json")
    group("base-harness","Other fixed wiring, moving service loops and terminations",1.65,INSPECT)
    group("finish","Weld and finish portion of supplier package allowance",.160,MOUNT)
    group("jack-controls","Jack wiring, switches, preload actuator, mounting remainder",.750,"cad/shallow-cage/installed.py")
    group("camera-closures","Board, connector, guide closures, ties and finishing",.053,HARNESS)
    region=collections.defaultdict(float)
    for r in ledger:region[r["scope"]]+=r["inspectionStockUpperKg"]
    for a in purchased.values():region["removable battery" if a["instance"]=="np18-battery" else "inspection"]+=a["upperKg"]
    write_csv("material-register.csv",ledger)
    write_csv("article-register.csv",sorted(purchased.values(),key=lambda x:x["instance"]))
    out={"basis":"Proposed make-stock allocation and stated complete-unit receiving maxima. As-built weights and commercial mixed-body densities are unmeasured.", "groupsKg":dict(region),
        "inspectionLimitKg":52.16312255,"R103MarginKg":52.16312255-region["inspection"],"R408Kg":region["inspection"]+region["removable bumper"],
        "chargedEmptyFieldKg":region["inspection"]+region["removable bumper"]+region["removable battery"],
        "sourcePins":{f:sha(ROOT/f) for f in (BUMPER,DRIVE,BUY,INSPECT,HARNESS,MOUNT,SW+"power/installed.json",SW+"power/harness.json")},
        "nativeRows":len(ledger),"articleRows":len(purchased),"sumDisplayBulkBeforeSubstitutionsKg":sum(r["displayStockKg"] for r in ledger)}
    (HERE/"mass-comparison.json").write_bytes((json.dumps(out,indent=2)+"\n").encode())


if __name__=="__main__":
    assemble()
