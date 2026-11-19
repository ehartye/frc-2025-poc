"""Rebuild a selected red shop list and estimated inspection weight from the joined-frame map."""
import argparse
import csv
import hashlib
import json
import math
import os
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
EXPECTED = {"low-lock", "deck-lamp", "moving-camera", "no-match-air", "four-welded-corners"}


def read(path):
    return json.loads(path.read_bytes())


def write(path, value):
    path.write_bytes((json.dumps(value,indent=2)+"\n").encode())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    with path.open(encoding="utf8",newline="") as stream:
        return list(csv.DictReader(stream))


def stage(plugin, baseline, out):
    data=read(baseline)
    if set(data["selected"])!=EXPECTED or data["configuration"]!="shop-duplex-545-red":
        raise ValueError("Use the issued #82 red welded configuration exactly once")
    out.mkdir(parents=True,exist_ok=True)
    for s in data["sources"]:
        # Locate the issued project file under THIS checkout, even if the old generated
        # map is in an adjacent worktree. Refuse stale bytes in either source.
        original=(baseline.parent/s["file"]).resolve()
        if sha(original)!=s["sha256"]:
            raise ValueError("Input pin changed: "+s["id"])
        relative=next((original.relative_to(p) for p in original.parents if (p/".git").exists()),None)
        if relative is None:
            raise ValueError("Cannot locate project-relative source "+s["id"])
        current=ROOT/relative
        if sha(current)!=s["sha256"]:
            raise ValueError("Current checkout differs: "+s["id"])
        s["file"]=os.path.relpath(current,out).replace(os.sep,"/")
    def pin(name,path):
        data["sources"].append({"id":name,"file":os.path.relpath(path,out).replace(os.sep,"/"),"sha256":sha(path)})

    stock=read(HERE/"articles.json")
    caps=stock["wheels"]["accept_bare_kg"]
    fastener=stock["hub_screws"]["accept_each_kg"]
    bom=ROOT/"design/drivetrain/review/bom.csv"
    wheels=[r for r in rows(bom) if r["id"].startswith("wheel-")]
    old=[sum(float(w[k]) for w in wheels) for k in ("minMassKg","maxMassKg")]
    if len(wheels)!=6 or any("am-0940b" not in w["qualification"] for w in wheels):
        raise ValueError("Wheel mass requires the six selected bare wheels")
    units={u["id"]:u for u in data["instances"]}
    if not all(units[w["id"]]["article"].startswith("am-0940b ") for w in wheels):
        raise ValueError("Wheel identity changed")
    screws=[k for k in units if k.startswith("unmodeled-wheel-to-hub-screw-")]
    if len(screws)!=36 or len(screws)!=len(wheels)*stock["hub_screws"]["quantity_each_wheel"]:
        raise ValueError("Wheel screw count changed")
    base=data["mass"]["baseline"]["rangeKg"]
    # Correlated replacement. Each endpoint contains these same six explicit BOM rows.
    # Subtract the old endpoint from itself, then add the measured-lot acceptance bound.
    revised=[base[i]-old[i]+len(wheels)*caps[i] for i in (0,1)]
    mass={"old_baseline_kg":base,"booked_wheel_group_kg":old,"conditional_bare_wheel_group_kg":[6*v for v in caps],
          "wheel_replaced_baseline_kg":revised,"hub_screw_group_kg":[36*v for v in fastener],
          "fixed_adapter_and_short_leads_kg":[0.005,0.105],
          "shop_additions_kg":[36*fastener[0]+0.005,36*fastener[1]+0.105],
          "source_sha256":{p.relative_to(ROOT).as_posix():sha(p) for p in (
              bom,HERE/"articles.json",HERE/"wheel-fit.json",ROOT/"design/competition-package/shop-duplex/mass.json",
              ROOT/"fixed-low-power/dual-camera.md",ROOT/"design/electrical-inspection/circuits.json",
              ROOT/"camera-disconnects/mass-conditions.json")}}
    evidence=out/"mass-input.json"
    write(evidence,mass)
    pin("selected-shop-mass",evidence)
    def interval(key):
        return {"source":"selected-shop-mass","pointer":"/"+key,"rangeKg":mass[key],"basis":"estimated"}
    data["mass"]["baseline"]=interval("wheel_replaced_baseline_kg")
    data["selected"].append("light")
    catalog={a["id"]:a for a in data["articles"]}
    def supply(label,sku=None,per=None):
        if label not in catalog:
            catalog[label]={"id":label,"sku":sku,"unitsPerPack":per,"onHand":None}
        elif sku:
            old_sku=catalog[label]["sku"]
            if old_sku not in (None,sku):
                raise ValueError("Two SKUs for "+label)
            catalog[label].update(sku=sku,unitsPerPack=per)
        return label

    def buy(existing,label,sku=None,per=None):
        units[existing].update(disposition="buy",article=supply(label,sku,per))

    fixed="OAK-D Lite fixed focus"
    for which in ("observer-camera","roof-camera-envelope"):
        if which not in units:
            matches=[i for i,r in units.items() if r.get("article")==fixed]
            if which!="observer-camera" or len(matches)!=1:
                raise ValueError("Camera identity changed")
            which=matches[0]
        buy(which,fixed,"A00532",1)
    for w in wheels:
        buy(w["id"],"Six-inch white HiGrip wheel","am-0940b",1)
    for screw in screws:
        buy(screw,"Wheel to hub thread-forming screw","am-1592",1)
    base_article="MAXPlanetary base; socket included"
    for lr in "LR":
        buy("gearbox-"+lr,base_article,"REV-25-2109",1)
    for i,r in units.items():
        if r.get("article","").startswith("am-4124 "):
            buy(i,r["article"],"am-4124",1)
    extra=[]
    def add(identity,label,sku=None,per=None):
        if identity in units:
            raise ValueError("Duplicate installed identity "+identity)
        units[identity]={"id":identity,"presence":"candidate","disposition":"buy","article":supply(label,sku,per)}
        extra.append(identity)

    for lr in "LR":
        for sku in stock["gearbox_base_kit"]["separate_selected_ratios"]:
            add("shop-"+lr+"-"+sku, "MAXPlanetary cartridge "+sku,sku,1)
    for load in ("host","fixed","moving"):
        add("shop-ddr-"+load,"Independent 5 V converter","DDR-30G-5",1)
        add("shop-output-fuse-"+load,"5 A MINI output fuse" if load=="host" else "3 A MINI output fuse")
        add("shop-holder-"+load,"MINI output fuse holder")
    add("shop-fixed-camera-adapter","dual USB-C power and data adapter","DM6010",1)
    add("shop-fixed-camera-usb-lead","Fixed camera data lead; measure mating length")
    add("shop-fixed-camera-power-lead","Fixed camera power plug/tail; verify hot resistance")
    # Channel 6 becomes the host supply. Add 8 and 9 for the two new camera branches.
    # The two ATM fuses on low-current channels come in the PDH kit, independent of ATO packs.
    circuits=read(ROOT/"design/electrical-inspection/circuits.json")["branches"]
    breakers={10:"REV-11-1860-PK4",20:"REV-11-1861-PK4",30:"REV-11-1862-PK4",40:"REV-11-1863-PK4"}
    slots={r["channel"]:r["ratingA"] for r in circuits if r["protection"]=="REV ATO breaker"}
    if slots.get(6)!=10 or 8 in slots or 9 in slots:
        raise ValueError("Camera power channels changed")
    slots.update({8:10,9:10})
    for channel,amperes in slots.items():
        add(f"shop-PDH-breaker-{channel}",f"{amperes} A ATO auto-resetting breaker",breakers[amperes],4)
    for adc in ("48","49"):
        add("shop-ads1015-"+adc,"ADS1015 acquisition board; select breakout and protection")
    # The camera disconnect input represents the chain as one assembly. A service buyer
    # needs 23 installed links. Use one retained first link and add precisely 22 more.
    chain="10.025.100.0"
    label="igus Series 10 chain link"
    buy("service-"+chain,label,chain)
    for link in range(2,24):
        add(f"shop-chain-link-{link:02}",label,chain)
    # 12 original bearings are installed in chassis plates. Nothing is pressed into the hubbed wheels.
    # The old regulator blocks remain in native geometry; do not order or credit them.
    removed=[k for k,v in units.items() if v.get("article")=="Pololu D24V50F5"]
    if len(removed)!=2:
        raise ValueError("Both original regulator reserve IDs are required")
    data["instances"]=list(units.values())
    linked={part["article"] for part in units.values() if part["disposition"]=="buy"}
    data["articles"]=[v for k,v in catalog.items() if k in linked]
    data["changes"].append({"id":"shop-supplies","revision":data["revision"],"configuration":data["configuration"],
            "add":extra,"remove":removed,"delta":interval("shop_additions_kg")})
    data["selected"].append("shop-supplies")
    data["mass"]["omissions"].append("Receiving and assembled scale data pending; remaining loose hardware, sensors, keepers and harness allocations are not item-level weights.")
    data["omissions"].append("Legacy Pololu box geometry retained as clearance reserves. Split drive article and chain supplies; measured physical stock remains unknown.")
    path=out/"selected.json"
    write(path,data)
    result=json.loads(subprocess.check_output(["node",str(plugin/"scripts/assembly-accounting.mjs"),str(path)]))
    if any(result["coverage"][k] for k in ("missing","duplicates","orphanIncluded")):
        raise ValueError("Incomplete baseline coverage")
    buy_list=result["buy"]
    with (out/"supply.csv").open("w",encoding="utf8",newline="") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow(["article","sku","installed","unitsPerPack","packsIfZeroStock","actualOrder","installedIdentity"])
        for r in buy_list:
            size=r["unitsPerPack"]
            qty=r["requiredUnits"]
            writer.writerow([r["article"],r["sku"],qty,size,math.ceil(qty/size) if size else None,None," | ".join(r["instances"])])
    new=result["mass"]["representedMarginKg"][0]
    light=next(c["delta"]["rangeKg"][1] for c in data["changes"] if c["id"]=="light")
    steel=next(c["delta"]["rangeKg"][1] for c in data["changes"] if c["id"]=="steel")
    weight={"accepted_receiving_caps":new,"old_550g_wheels_with_loose_items":new-(old[1]-6*caps[1]),
            "steel_carriers_same_wheel_caps":new-(steel-light),
            "additional_1kg_loose_items":new-1.0,"unwelded_control_only":new+0.020}
    return {"installed_version":read(plugin/"package.json")["version"],"input_selection":sorted(EXPECTED),
            "estimated_mass_kg":result["mass"]["representedRangeKg"],"margin_at_upper_kg":weight,
            "physical_margin_kg":result["mass"]["configurationMarginKg"],"buy_instances":sum(r["requiredUnits"] for r in buy_list),
            "supply_rows":len(buy_list),"make":len(result["make"]),"open_selection":len(result["undecided"]),
            "retired_purchase_reserves":removed,"component_bounds":mass,
            "supplies_sha256":sha(out/"supply.csv")}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("plugin","baseline","output"):
        p.add_argument(k,type=Path)
    a=p.parse_args()
    write(a.output/"review.json",stage(a.plugin.resolve(),a.baseline.resolve(),a.output.resolve()))
