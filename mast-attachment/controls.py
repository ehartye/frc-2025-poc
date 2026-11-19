"""Reconstruct two rejected details against the released cover mount and camera trough."""
from mast import HERE, ROOT, j, prism


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
    j.write(HERE/"controls.json",data)


if __name__=="__main__":run()
