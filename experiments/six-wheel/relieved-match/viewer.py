"""Embed the measured paths and controller events in an offline field replay."""
import csv
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def run(cache):
    read=lambda path:json.loads(path.read_bytes())
    match=read(HERE/"match.json")
    stock=read(ROOT/"full-match/installed-stock.json")["stock"]
    profile=read(ROOT/"kickoff/season-profile.json")
    field=[profile["field"]["length"]*.0254,profile["field"]["width"]*.0254]
    points=[]
    for item in stock.values():
        x,_,z=item["native_center_mm"]
        points.append([field[0]/2+x/1000,field[1]/2-z/1000,item["group"]])
    with (HERE/"events.csv").open() as f:events=list(csv.DictReader(f))
    origin=read(ROOT/"mission/field-registration.json")["redReef"]["lipOriginNativeMm"]
    targets={k:[field[0]/2+origin[0]/1000+v["center"][0],field[1]/2-origin[2]/1000+v["center"][1]] for k,v in match["sourceToFinal"].items()}
    data={"field":field,"targets":targets,"reefs":[[s["x"]*.0254,s["y"]*.0254,s["acrossFlats"]*.0254] for s in profile["landmarks"]],
          "stock":points,"events":events,"cases":match["cases"],"tracks":{m:read(cache/(m+"-visual.json")) for m in match["cases"]}}
    text=(HERE/"template.html").read_text(encoding="utf8").replace("/*DATA*/",json.dumps(data,separators=(",",":")))
    (HERE/"replay.html").write_text(text,encoding="utf8",newline="\n")


if __name__=="__main__":run(Path(sys.argv[1]))
