"""Reopen the changed solids and bound added mass without taking unknown removal credit."""
import csv
import hashlib
import json
from cassette import ROOT, HERE, built, originals, cad, touch, finalize_step, mech

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def run():
    source=originals()
    fixed,moving=built(0,0)
    parts=fixed|moving
    selected={k:v for k,v in parts.items() if k.startswith("cassette-") or
              k.startswith("grip-") and k.endswith(("cheek","slide-guide","hardstop","riser","support-foot"))}
    tags=[{"id":k,"group":"new"} for k in selected]
    path=HERE/"fixed-center-cassette.step"
    cad.write_named(path,list(selected.values()),tags,{"new":"#668d9d"})
    finalize_step(path)
    opened=cad.read_named(path,tags)
    counts={k:cad.solid_count(v) for k,v in opened.items()}
    overlap={}
    for k,part in opened.items():
        if not k.startswith("cassette-"):continue
        for a,v in moving.items():
            if a==k or a.startswith("cassette-") and a<k:continue
            volume=touch(part,v)
            if volume>.05:overlap[k+" / "+a]=volume
    positive_stock=0.
    for k,v in opened.items():
        if k.startswith("cassette-") and k.endswith(("coil-reserve","ribbon")):continue
        density=7.85e-6 if k.endswith(("axle","slide-guide")) else 2.70e-6
        volume=cad.volume(v)[0]
        if k in source:volume-=cad.volume(source[k])[0]
        positive_stock+=max(0,volume)*density
    spring_kg_per_jaw=(533*12.70*.15+381*6.35*.10)*8.0e-6
    # One full source strip per spring. The visible ribbon and coiled reserve describe the same article.
    additions=positive_stock+2*spring_kg_per_jaw
    cap=additions+.12
    initial=json.loads((ROOT/"design/fixed-height/review.json").read_bytes())["mass"]["low-red"]["representedMarginKg"][0]
    bom=list(csv.DictReader((ROOT/"design/receiver-drives/bom.csv").open()))
    carried=[r for r in bom if r["id"].startswith("grip-L-") and r["id"] in moving]
    mass=[sum(float(r[k]) for r in carried) for k in ("lowMassKg","highMassKg")]
    return {"registeredCount":len(parts),"writtenCount":len(opened),"solidCounts":counts,"newRigidOverlapMm3":overlap,
            "nativeSha256":digest(path),"originalCarriedLeftKg":mass,"positiveStockDeltaKg":positive_stock,
            "springStockPerJawKg":spring_kg_per_jaw,"stockAndFullStripsKg":additions,
            "additionalHardwareAllowanceKg":.12,"conditionalAddedUpperKg":cap,"conditionalMassHeadroomKg":initial-cap,
            "unknownOpenerAndLatchKg":None,"completeInspectionMarginKg":None,
            "sources":{str(p.relative_to(ROOT).as_posix()):digest(p) for p in (
            ROOT/"design/fixed-height/ports-065.json",ROOT/"design/fixed-height/review.json",ROOT/"station-relief/cheeks.step",
            ROOT/"station-relief/shims.step",ROOT/"design/receiver-drives/bom.csv",ROOT/"cradle-receiver/study.json")}}
if __name__=="__main__":
    (HERE/"package.json").write_text(json.dumps(run(),indent=2)+"\n",encoding="utf-8",newline="\n")
