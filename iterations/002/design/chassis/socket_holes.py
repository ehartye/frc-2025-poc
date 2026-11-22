"""Derive the remaining socket holes from actual cold front-left screw centers."""
import json
import math
from pathlib import Path

here = Path(__file__).resolve().parent
load = lambda name: json.loads((here / name).read_bytes())
single = load("front-corner.json")
observed = load("independent-123.json")
packaging = load("four-corner-packaging.json")
corners = packaging["kingpinsXYmmAndRelativeYawDeg"]
rails = {}
for old in single["rails"]:
    for original in (True, False):
        rail = dict(old)
        side = old["lengthAxis"]
        if not original:
            rail["name"] = ["Starboard longitudinal socket rail", "Rear transverse socket rail"][side]
            j = 1 - side
            low, high = list(old["minimumMm"]), list(old["maximumMm"])
            low[j], high[j] = -high[j], -low[j]
            rail.update(minimumMm=low,maximumMm=high)
        rail.update(original=original, boltCentersMm=[], sources=[])
        rails[rail["name"]] = rail
for station, (x, y, degrees) in corners.items():
    co = round(math.cos(math.radians(degrees)))
    si = round(math.sin(math.radians(degrees)))
    for screw, data in observed["withdrawal"].items():
        u,v = data["worldAxisMm"]
        u,v = u-282.,v-277.
        center = [round(x+co*u-si*v,4), round(y+si*u+co*v,4)]
        matches = [r for r in rails.values() if all(r["minimumMm"][i] < center[i] < r["maximumMm"][i] for i in (0,1))]
        if len(matches)!=1: raise ValueError((station,center,matches))
        rail=matches[0]
        rail["boltCentersMm"].append(center)
        rail["sources"].append({"station":station,"screw":screw})
if any(len(r["boltCentersMm"])!=6 for r in rails.values()):raise ValueError("Incomplete pattern")
for r in rails.values():
    if r["original"]:
        old,=[s for s in single["rails"] if s["name"]==r["name"]]
        observed_fl=[c for c,s in zip(r["boltCentersMm"],r["sources"]) if s["station"]=="FL"]
        if any(min(math.dist(c,p) for p in observed_fl)>.002 for c in old["boltCentersMm"]):raise ValueError("Source screw centers changed")
    r["minimumHoleWallToEndMm"]=min(min(point[r["lengthAxis"]]-r["minimumMm"][r["lengthAxis"]],r["maximumMm"][r["lengthAxis"]]-point[r["lengthAxis"]])-r["throughDiameterMm"]/2 for point in r["boltCentersMm"])
(here/"socket-holes.json").write_text(json.dumps({"nativeSha256":observed["nativeSha256"],"rails":rails,"discipline":"World coordinates; three supplier axes on each side of a rail. Source corner rotations are proper rigid transforms. Four modules in planned pattern; only the front left currently installed."},indent=2)+"\n")
