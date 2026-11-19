"""Check a legal pre-tightening placement of both guide kits inside their nominal bolt clearances."""
import json
from parts import HERE,ROOT,SIDES,STATIONS,j


def run():
    src=j.read(HERE/"fit.json")
    path=HERE/"guide-mounts.step"
    if j.digest(path)!=src["stepSha256"]:
        raise ValueError("Different exchange")
    kit=j.cad.read_named(path,[{"id":k} for k in src["volumesMm3"]])
    offset={"sleeve":.04,"bolt":.12,"guide":.315}
    cases={}
    for side,sign in SIDES:
        by_name={}
        for k,v in kit.items():
            if not k.startswith("guide-"+side+"-"):
                continue
            if k.endswith("machined-mast"):
                shift=0.
            elif k.endswith("-sleeve"):
                shift=offset["sleeve"]
            elif "-root-" in k:
                shift=offset["bolt"]
            else:
                shift=offset["guide"]
            by_name[k]=j.cad.translated(v,(0,sign*shift,0))
        hits={}
        for i,(a,first) in enumerate(by_name.items()):
            ba=j.cad.bounds(first)
            for b,second in list(by_name.items())[i+1:]:
                if j.lower_bound(ba,j.cad.bounds(second))<.001:
                    v=j.cad.intersection(first,second)
                    if v>.001:
                        hits[a+"/"+b]=round(v,5)
        if hits:
            raise ValueError(hits)
        lands=[]
        for z in STATIONS:
            base=by_name[f"guide-{side}-{z}-mount-arm"]
            mast=by_name[f"guide-{side}-machined-mast"]
            lands.append(j.cad.intersection(j.cad.translated(base,(-.1,0,0)),mast))
        if min(lands)<1:
            raise ValueError("Missing mast seat")
        cases[side]={"materialInterferencesMm3":hits,"mastSeatsAt0p1Mm3":lands}
    # Centered 301.625 mm cylinder, lower parallel region; width alone does not set its center.
    length=301.625
    nominal=2*150.5125
    result={"stepSha256":src["stepSha256"],"outboardOffsetsMm":offset,"checked":cases,
            "nominalThroatWidthMm":nominal,"alternateThroatWidthMm":nominal+2*offset["guide"],
            "centeredPipeLengthMm":length,"alternateEndGapMm":(nominal+2*offset["guide"]-length)/2,
            "actualMeasuredWidthMm":None}
    j.write(HERE/"datum.json",result)
    print(result)


if __name__=="__main__":
    run()
