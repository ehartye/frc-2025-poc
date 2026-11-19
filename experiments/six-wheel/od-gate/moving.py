"""Interior checks between unlike moving frames during the station hood and target gate strokes."""
import json
from gate_cad import local,shapes,base,cad,interior,HERE,sha

def review():
    layers=local();gates=list(range(0,113,7))
    cases=[("open",370,q,0) for q in gates]+[("hood",30,0,q) for q in range(0,41,4)]
    out=[]
    for kind,stroke,opening,rise in cases:
        changing={"L","R"} if kind=="open" else {"hood"}
        add=shapes(stroke,opening,rise);allparts=base(stroke)|add
        boxes={k:cad.bounds(v) for k,v in allparts.items()}
        hits={}
        for k,shape in add.items():
            fk=layers[k][0]
            if fk not in changing:continue
            for j,test in allparts.items():
                fj=layers[j][0] if j in layers else "robot"
                if fj==fk or (fj in changing and j<=k) or not interior(boxes[k],boxes[j]):continue
                v=cad.intersection(shape,test)
                if v>.05:hits[k+" / "+j]=v
        out.append({"state":kind,"strokeMm":stroke,"openingMm":opening,"hoodRiseMm":rise,"interiorMm3":hits})
    return {"nativeSha256":sha(HERE/"split-gate.step"),"registration":"Rigid envelopes on the original slide and stationary fixture. Gate samples 7 mm; hood lift samples 4 mm. Mating faces and contacts below 0.05 cubic mm excluded. Intermediate extrema and shop tolerance remain open.","samples":out}
if __name__=="__main__":
    (HERE/"moving.json").write_text(json.dumps(review(),indent=2)+"\n",encoding="utf8",newline="\n")
