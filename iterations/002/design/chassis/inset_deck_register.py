"""Bound two-direction sheet bearing, edge ligaments and the one-count equipment budget."""
from pathlib import Path
import json,math,hashlib
import numpy as np
from inset_electronics_deck import definition

HERE=Path(__file__).resolve().parent


def run(contact=False,truss=False):
    file=HERE/("inset-truss-key-contact.json" if truss else "inset-key-contact.json" if contact else "inset-cassette-decks.json");frame=json.loads(file.read_bytes());geom=definition(True,truss)
    def edges(panel,x,y,fx,fy):
        x0,y0,x1,y1=panel["limits"]
        # Closest free edge or other service opening across the projected load
        # band. The actual 36.2 × 14.2 slot retains a 0.1 mm assembly clearance.
        hx,hy=18.1,7.1
        e=[x1-(x+hx),x-hx-x0,y1-y-hy,y-hy-y0]
        openings=panel["windows"]+[(tag,xx-r,yy-r,xx+r,yy+r) for tag,xx,yy,r in panel["holes"]]
        for tag,xa,ya,xb,yb in openings:
            if max(ya,y-hy)<min(yb,y+hy):
                if xa>x+hx:e[0]=min(e[0],xa-x-hx)
                elif xb<x-hx:e[1]=min(e[1],x-hx-xb)
                else:raise ValueError("Hardware and opening overlap "+str((tag,x,y)))
            if max(xa,x-hx)<min(xb,x+hx):
                if ya>y+hy:e[2]=min(e[2],ya-y-hy)
                elif yb<y-hy:e[3]=min(e[3],y-hy-yb)
                else:raise ValueError("Hardware and opening overlap "+str((tag,x,y)))
        # Preserve BOTH directions regardless of which sign the solver's pin loads
        # are reported in. The least one is a conservative fully reversed bound.
        return e,min(e[:2]),min(e[2:])
    t=1.8796;condition=120.;shear=condition/math.sqrt(3)
    physical=[];control=[]
    for name,r in frame["outcomes"].items():
        if "keyed" not in name:continue
        for case,v in r["cases"].items():
            for row in v["deckLoads"]:
                x,y=row["xyMm"];sign=1 if y>0 else -1;fx,fy=row["sheetForceN"]
                panel=geom[sign];e,lx,ly=edges(panel,x,abs(y),fx,fy*sign)
                # Separate flat tangential/radial faces. Summation conservatively charges
                # the simultaneous projected bearing loads to one allowable. This does
                # not grant a fatigue or edge-contact/clearance distribution assumption.
                bearing=abs(fx)/(10*t)+abs(fy)/(32*t)
                ratio=abs(fx)/(2*lx*t*shear)+abs(fy)/(2*ly*t*shear)
                kind="perimeter" if abs(y)>300 else "upper crossmember" if x in (110,250) else "rear sill"
                physical.append(dict(variant=name,case=case,host=kind,xyMm=[x,y],loadsN=[fx,fy],
                    flatBearingMPa=bearing,reversedTearoutRatio=ratio,ligamentsMm=e))
                control.append(dict(variant=name,case=case,xyMm=[x,y],round5mmBearingMPa=math.hypot(fx,fy)/(5*t)))
    reference=next(r for n,r in frame["outcomes"].items() if "keyed" in n)
    blank_min=sum(p["nominalDrilledBlankKg"] for p in reference["decks"])
    # This named bracket is from a real source. Material-density conversion is only
    # charged by the separate hardware BOM until the complete installed pack is weighed.
    return dict(authority=frame["authority"],responseSha256=hashlib.sha256(file.read_bytes()).hexdigest(),
        sheet=dict(alloy="Unwelded 6061-T6 sheet, ASTM B209",projectMinYieldMPa=240,T6source="Kaiser Aluminum sheet/plate Table 7.2 6061-T6, 0.021–0.249 inch, minimum 35 ksi.",nominalMm=2.032,minMm=t,maxMm=2.1844,workingMPa=condition,
                   nominalChargeKg=blank_min*2.032/t,maxChargeKg=blank_min*2.1844/t,whereCharged="Single line in 7 kg electronics assembly; zero additional chassis charge for the same two boards."),
        overall=dict(cases=len(physical),worstFlatBearing=max(physical,key=lambda q:q["flatBearingMPa"]),worstReversedTearout=max(physical,key=lambda q:q["reversedTearoutRatio"]),
                     adversePlain5mm=max(control,key=lambda q:q["round5mmBearingMPa"])),
        rows=physical,scope="Selected 36 × 14 mm key with R2 corners on each 36.2 × 14.2 R2.1 opening, 0.1 mm radial/lateral fit allowance. No support from component cases or friction. Checks use separate finite frame/sill mounting-stiffness scenarios at the prior governing strike, ordinary dock and single-wheel traction. The frame-side load path, local pressure peaks, actual slip, backing, vertical retention and the full simultaneous pose/hit set remain separate.")


if __name__=="__main__":
    import sys
    contact="--clearance" in sys.argv
    truss="--truss" in sys.argv
    v=run(contact,truss);(HERE/("inset-truss-key-register.json" if truss else "inset-key-contact-register.json" if contact else "inset-deck-register.json")).write_text(json.dumps(v,indent=2)+"\n")
    print(v["sheet"]);print(v["overall"])
