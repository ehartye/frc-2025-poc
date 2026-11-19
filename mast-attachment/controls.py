"""Reconstruct two rejected details against the released cover mount and camera trough."""
from mast import HERE, ROOT, j, prism,plate


def run():
    source={}
    for folder,file,meta in (("cover-retention","supported-covers.step","fit.json"),
                              ("fixed-camera-route","harness-guides.step","guides.json")):
        r=j.read(ROOT/folder/meta)
        if j.digest(ROOT/folder/file)!=r["stepSha256"]:raise ValueError("Different mounted equipment")
        source.update(j.cad.read_named(ROOT/folder/file,[{"id":k} for k in r["volumeMm3"]]))
    s=-1
    leg=prism(s,[(-132,60),(-104,60),(-104,121),(-132,121)],235,238)
    leg=j.cut(leg,j.rod(-118,s,234,239,72,3.25))
    rib=prism(s,[(-107,158),(-95,158),(-75,182),(-63,249),(-70,252),(-82,185)],238,243)
    data={"deepLegFromZ60Mm3":{k:j.cad.intersection(leg,source[k]) for k in (
        "tray-upper-R","post--120_64-R","base-screw--120_64-R","post--120_88-R")},
        "unrelievedRibMm3":{k:j.cad.intersection(rib,source[k]) for k in ("roof-host-trough-01","roof-host-trough-02")}}
    if any(v<=0 for row in data.values() for v in row.values()):raise ValueError(data)
    ds=ROOT/"design/receiver-lift"
    ids=j.read(ds/"duplex-inventory.json")["poses"]["place"]
    stock=j.cad.read_named(ds/"duplex-place.step",[{"id":k} for k in ids])
    data["trueTallExtraction"]={}
    for side,sign in (("L",1),("R",-1)):
        tall=plate(sign,full=True)
        target=stock["backup-"+side+"-bearing-support"]
        trials={dy:round(j.cad.intersection(j.cad.translated(tall,[0,sign*dy,35]),target),5) for dy in (20.,40.,60.)}
        if max(trials.values())<=0:raise ValueError(trials)
        data["trueTallExtraction"][side]={"lateralOffsetMmAndCommonMm3":trials,"bearingSha256":j.digest(ds/"duplex-place.step")}
    j.write(HERE/"controls.json",data)


if __name__=="__main__":run()
