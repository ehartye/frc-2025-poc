"""Create a per-alliance installed-to-shop crosswalk and count unique represented articles."""
from collections import defaultdict
import csv
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DESIGN = ROOT/"design"
PACKAGE = DESIGN/"competition-package/shop-duplex"
OLD = DESIGN/"shop-package"


def table(path,key):
    with path.open(encoding="utf-8",newline="") as source:
        return {r[key]:r for r in csv.DictReader(source)}


def write(path, header, rows):
    with path.open("w",encoding="utf-8",newline="") as target:
        writer=csv.writer(target,lineterminator="\n")
        writer.writerow(header)
        writer.writerows(rows)


def intake():
    historical=table(OLD/"inventory.csv","installedId")
    grip=table(DESIGN/"receiver-drives/bom.csv","id")
    duplex=json.loads((DESIGN/"receiver-lift/duplex-inventory.json").read_bytes())["poses"]["stowed"]
    def resolve(k):
        instance=k
        drawing=""
        if k in historical:
            row=historical[k]
            source=row["owningSource"]
            drawing=row["drawingFamily"]
            owner=row["owner"]
            basis=row["dimensionOrProcurementBasis"]
            if owner=="Receiver transmission":
                rep=grip[k]["representation"]
                if rep=="measurement-line":return "included","SP1-25 draw wire","slide-position-transducer",source,"Cable included with string pot",drawing
                if rep=="belt-envelope" or "REV UP 550" in basis or "REV-" in basis or "SP1-25" in basis or "AS5600" in basis:
                    route="buy candidate"
                elif rep in ("machined","rack-section","involute-section","shaft-envelope"):
                    route="make"
                else:route="selection open"
                return route,basis if route=="buy candidate" else k,instance,source,basis,drawing
            if owner=="Drive and retained chassis":
                if k.startswith(("motor-","bearing-","sprocket-","hub-","wheel-","chain-","gearbox-L","gearbox-R","drive-controller-")):
                    name=basis.split(";")[0]
                    if k.startswith("motor-"):name="REV-21-1650 NEO V1.1"
                    if k.startswith("drive-controller-"):name="REV-11-2158 SPARK MAX"
                    return "buy candidate",name,instance,source,basis,drawing
                if k.startswith(("gearbox-face-screw","jack-")):
                    return "commodity",basis.split(";")[0],instance,source,basis,drawing
                return "make",drawing or k,instance,source,basis,drawing
            if owner=="Bumpers":
                item=re.sub(r"-(front|rear)","",k).removesuffix("-L").removesuffix("-R")
                if k.startswith("bumper-screw"):
                    label="M8 x 35 end socket screw" if "-end-" in k else "M8 x 40 side socket screw"
                    return "commodity",label,instance,source,basis,drawing
                if k.startswith("bumper-washer"):return "commodity","16 OD x 1.6 washer",instance,source,basis,drawing
                return "make",item,instance,"design/bumper-kit/README.md",basis+"; use per-color cut schedule",drawing
            if owner=="Electrical/pneumatic":
                names={"np18-battery":"EnerSys Genesis NP18-12","rev-pdh":"REV-11-1850 PDH","main-breaker":"120 A main breaker",
                       "viair-90c":"Viair 90C","tank-avt-pp-35":"AndyMark am-2649 tank","rio2-envelope":"roboRIO 2.0",
                       "vh109-reserve":"VH-109 radio","rev-ph":"REV-11-1852 PH","rsl-reserve":"RSL"}
                item=names.get(k)
                if k.startswith("qtrx-"):item="Pololu 4501 QTRX"
                if k.startswith("spark-"):item="REV-11-2158 SPARK MAX"
                if item:return "buy candidate",item,instance,source,"Envelope; verify exact enclosure, terminals, wiring and mounting",drawing
                if k in ("gauge-regulator-dump-reserve","regulated-pawl-valve"):
                    return "selection open",k,instance,source,"Grouped pneumatics reservation; fittings and line quantities open",drawing
                return "make",k,instance,source,"Original mount or support; cut, fastener and access detail required",drawing
            if owner=="Observer":
                catalog={"observer-camera-proxy":"OAK-D Lite fixed focus","observer-pi4-service-envelope":"Raspberry Pi 4B",
                         "observer-camera-5v-reserve":"Pololu D24V50F5","observer-host-5v-reserve":"Pololu D24V50F5"}
                if k in catalog or k.startswith("observer-moving-pad"):
                    return "buy candidate",catalog.get(k,"Interlink FSR 400 Short"),instance,source,"Approximate installation and cable or laminate envelope",drawing
                if k in ("observer-usb-service-reserve","observer-cable-trunk-reserve"):
                    return "space",k,"",source,"Connector/service routing allowance",drawing
                return "make",k,instance,source,basis,drawing
            if owner=="Receiver lift":
                src="design/receiver-lift/README.md"
                if k=="motor-body":return "buy candidate","REV-21-1650 NEO V1.1",instance,src,"Lift drive",drawing
                if k.startswith(("drum-","running-line-","return-line-","sheave-")) and not k.startswith("sheave-support-"):
                    side=k[-1]
                    if k.startswith("drum-"):kind,name="buy candidate","am-4647_2000 primary drum"
                    elif k.startswith("sheave-"):kind,name="selection open","primary guide sheave"
                    else:kind,name="buy candidate","am-4559 primary line"
                    return kind,name,name+"-"+side,src,"Several modeled segments constitute one article per side",drawing
                if "reserve" in k or "reach-" in k or "bearing-envelope" in k:
                    return "selection open",k,instance,src,"Member of primary drive/transfer selection; see component schedule",drawing
                return "make",k,instance,src,basis,drawing
            raise ValueError("Unassigned historical subsystem "+owner)
        if k in duplex:
            src="design/receiver-lift/RETENTION.md"
            if k in ("lift-crossbar-drilled","long-common-shaft"):return "make",k,instance,src,"Machine or cut registered parent stock",drawing
            m=re.fullmatch(r"backup-([LR])-(.+)",k)
            if not m:raise ValueError(k)
            side,part=m.groups()
            if part in ("running-line","return-line","spliced-eye-envelope"):
                return "selection open","terminated secondary line","secondary-line-"+side,src,"One length and qualified termination per side",drawing
            if part.startswith(("drum-core","drum-flange")):
                return "selection open","secondary drum","secondary-drum-"+side,src,"Core and flanges form one drum; winding and shaft key open",drawing
            if part=="upper-sheave" or part.startswith("sheave-flange"):
                return "selection open","secondary guide sheave","secondary-sheave-"+side,src,"Groove, shaft, keepers and load rating open",drawing
            if part.startswith(("mast-bolt","carriage-bolt")):
                return "commodity",part.split("-bolt")[0]+" mounting bolt",instance,src,"Schematic fastener; select grade, length and locking",drawing
            if part in ("thimble","outboard-bearing","clevis-pin") or "bushing" in part:
                return "selection open",re.sub(r"-\d+$","",part),instance,src,"Unpurchased interface envelope; tolerance and load rating open",drawing
            return "make",part,instance,src,"Dimensioned in editable layer; process, jig and first-article inspection open",drawing
        src="design/shop-package/JOINTS.md"
        if k.endswith("nut-saddle"):return "make","outer threaded U saddle",instance,src,"8 mm steel saddle, tapped M5",drawing
        if re.match(r"^(front|rear|center)-[LR]-",k):
            if "-screw-" in k:return "commodity","M5 x 16 outer cap screw",instance,src,"Property class 8.8 proposal",drawing
            if "-washer-" in k:return "commodity","outer 10 x 5.3 x 1 washer",instance,src,"Measure thickness",drawing
        if k.startswith("pin-"):return "commodity","3 x 20 receiver crosspin",instance,src,"Choose certified pin and actual fit",drawing
        if k.startswith("drilled-slide-"):return "make","cross-drilled inner slide",instance,src,"Match-drill eight pin axes at installed blocks",drawing
        if k.startswith("tapped-anchor-"):return "make","receiver tapped block",instance,src,"Steel M5 tapped and cross-drilled",drawing
        raise ValueError("Missing installed ID "+k)
    blue=json.loads((PACKAGE/"blue/part-frames.json").read_bytes())
    red=json.loads((PACKAGE/"red/part-frames.json").read_bytes())
    rows=[]
    for k,frame in sorted(blue.items()):
        redid=k.replace("-blue","-red")
        if red.get(redid)!=frame:raise ValueError("Alliance correspondence "+k)
        rows.append([k,redid,frame,*resolve(k)])
    if len(rows)!=545 or len(red)!=len(rows):raise ValueError("Unexpected assembly revision")
    return rows


def generate():
    rows=intake()
    write(HERE/"installed.csv",["blueId","redId","frame","route","item","instance","source","basis","drawingFamily"],rows)
    groups=defaultdict(list)
    for row in rows:groups[row[3],row[4]].append(row)
    result=[]
    for (route,item),group in sorted(groups.items()):
        source=" | ".join(sorted({r[6] for r in group}))
        quantity=len({r[5] for r in group if r[5]})
        if route in ("included","space"):quantity=0
        result.append([route,item,quantity,len(group),source])
    write(HERE/"item-rollup.csv",["route","item","representedUnitsPerAlliance","nativeBodies","source"],result)


if __name__=="__main__":
    generate()
