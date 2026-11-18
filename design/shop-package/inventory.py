"""Cross-reference every current native ID against a single owning design layer."""
import csv
import json
from pathlib import Path
import sys

from profiles import profiles

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/"design/electrical-inspection"))
import package as electrical


def csv_by(path,key):
    with path.open(encoding="utf-8",newline="") as f:return {r[key]:r for r in csv.DictReader(f)}


def inventory():
    frames=json.loads((ROOT/"design/competition-package/part-frames.json").read_text())
    drive=csv_by(ROOT/"design/drivetrain/review/bom.csv","id")
    grip=csv_by(ROOT/"design/receiver-drives/bom.csv","id")
    bumpers=csv_by(ROOT/"design/bumper-kit/mass.csv","solid")
    electric=electrical.layout({"height":940,"stroke":0})
    sheet={k:p.key for p in profiles() for k in p.ids}
    rows=[]
    for key,frame in frames.items():
        if key in bumpers:owner,record,basis="Bumpers","design/bumper-kit/mass.csv",bumpers[key]["material"]
        elif key in grip:owner,record,basis="Receiver transmission","design/receiver-drives/bom.csv",grip[key]["description"]
        elif key in electric:owner,record,basis="Electrical/pneumatic","design/electrical-inspection/README.md","Equipment or service envelope; identify installed hardware separately"
        elif key.startswith("observer-"):owner,record,basis="Observer","release-observer/README.md","Original mount, physical candidate or reserved clearance; use observer assembly table"
        elif key in drive:owner,record,basis="Drive and retained chassis","design/drivetrain/review/bom.csv",drive[key]["qualification"]
        else:owner,record,basis="Receiver lift","design/receiver-lift/README.md","Registered route or shaft/actuator element; see lift BOM and qualification"
        rows.append([key,frame,owner,record,basis,sheet.get(key,"")])
    expected={"Bumpers":44,"Receiver transmission":144,"Electrical/pneumatic":33,"Observer":14,"Drive and retained chassis":132,"Receiver lift":34}
    observed={k:sum(row[2]==k for row in rows) for k in expected}
    if observed!=expected:raise ValueError((observed,expected))
    return rows


if __name__=="__main__":
    with (HERE/"inventory.csv").open("w",encoding="utf-8",newline="") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["installedId","frame","owner","owningSource","dimensionOrProcurementBasis","drawingFamily"])
        w.writerows(inventory())
