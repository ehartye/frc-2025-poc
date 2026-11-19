"""Check the low robot's registered footprint and each native component against initial stock."""
import json
import sys
from native import ROOT,HERE,hardware,cad
sys.path.insert(0,str(ROOT/"design/competition-package/shop-duplex"))
from assembly import pose_shapes
sys.path.insert(0,str(ROOT/"full-match"))
from match import corrected
from installed_stock import check


def study():
    model=pose_shapes(560,0,"red")
    modified,add=hardware(model,height=560)
    boxes={k:cad.bounds(v) for k,v in {**model,**modified,**add}.items()}
    hull=[(min if i<3 else max)(v[i] for v in boxes.values()) for i in range(6)]
    stock=json.loads((ROOT/"full-match/installed-stock.json").read_bytes())["stock"]
    f=corrected()
    rows={}
    for sector in (-.18,.18):
        r=f.RegisteredMission(sector)
        stages={}
        for name,leg in (("approach",r.approach),("carry",r.carry),("return",r.return_leg)):
            stages[name]={"durationS":leg.duration,"staticField":min(f.audit(leg,hull),key=lambda x:x["gap_m"]),
                          "stock":check(f,stock,boxes,leg.segments)}
        rows[str(sector)]=stages
    return {"footprintMm":hull,"bodies":len(boxes),"scope":"Red route and retracted rigid robot; initial stock. Carried pipe, docking extension and traffic are separate.","stages":rows}


if __name__=="__main__":
    (HERE/"transit.json").write_text(json.dumps(study(),indent=2)+"\n",encoding="utf-8",newline="\n")
