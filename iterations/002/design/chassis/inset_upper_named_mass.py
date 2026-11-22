from pathlib import Path
import json,hashlib
here=Path(__file__).resolve().parent
root=here.parents[3]
base=root/"artifacts/red2-integrated"
ref=base/"pan-mechanics.json";purch=base/"pan-drive-registration.json"
metrics=json.loads(ref.read_bytes());drive=json.loads(purch.read_bytes())
catalog={"REV-21-1651":("NEO 550 with leads",.142),"REV-41-1601":("3:1 stage",.0355),
         "REV-41-1602":("4:1 stage",.034),"REV-41-1603":("5:1 stage",.036),
         "REV-41-1607":("550 input plate",.0335),"REV-41-1615":("complete V2 output cartridge",.0665)}
items=[dict(id=sku,name=name,quantity=1,kg=mass,source="https://www.revrobotics.com/"+sku.lower()+"/") for sku,(name,mass) in catalog.items()]
open_items=["REV-41-1608 13-tooth motor pinion","REV-41-1620 1/2 inch hex adapter","nine previously registered M3x8 adapter/housing screws and the full-length cartridge stack fasteners",
"two REV-21-1915 radial support bearings","6655K17 opposed thrust bearings, both sets of races, balls and cages","machined thrust diaphragm, axial shim, end cap, cartridge screws and shaft retention",
"new two-sill upper supports and their fasteners, replacing the independent tower feet and knees",
"sensor, limits, fixture attachments and dressed motor cable; the ninth SPARK MAX is separately budgeted to electronics",
"mouth/guide wear surfaces, article containment and any production adjustment required by the registered 30 A duty"]
stock=[dict(name=k,kg=v["massKg"],moves=v["moving"],disposition="retired support; charged in comparator until the new connection is weighed" if any(z in k.lower() for z in ["tower foot","tower knee"]) else "historical fabricated article, mass reference only; revalidate in 002")
       for k,v in metrics["masses"].items()]
total_stock=sum(a["kg"] for a in stock)
if abs(total_stock-metrics["fixedShopStockKg"]-metrics["movingShopStockKg"])>1e-8:raise ValueError("Wrong reference stock roster")
known=sum(i["kg"] for i in items);allow=8.2
d=dict(sourceStockSha256=hashlib.sha256(ref.read_bytes()).hexdigest(),sourceDriveSha256=hashlib.sha256(purch.read_bytes()).hexdigest(),
    allocationKg=allow,historicStock=stock,movingStockKg=metrics["movingShopStockKg"],fixedStockKg=metrics["fixedShopStockKg"],
    originalRegisteredDriveIds=[v["id"] for v in drive["components"]],currentCatalogArticles=items,
    catalogSubtotalKg=known,stockPlusNamedPurchasedKg=total_stock+known,remainingForUnweighedArticlesKg=allow-total_stock-known,
    openArticles=open_items,scope="Shared robot mass planning uses only a single 8.2 kg upper. This comparison includes 68 historical physical stock weights and six independent declared purchased-article catalog masses. It does not import the retired tower geometry or imply a fully mated 002 pan. The documented gearbox output is one assembly split into two bodies in CAD and counted once by procurement.")
(here/"inset-upper-named-mass.json").write_text(json.dumps(d,indent=2)+"\n")
print(total_stock,known,d["remainingForUnweighedArticlesKg"])

