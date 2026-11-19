"""Check candidate carriers and bridge against the reopened installed terminal module."""
import json
import re
from native import HERE, ROOT, cad, contact, digest


def load(path, expected):
    if digest(path)!=expected:
        raise ValueError(f"Native identity mismatch for {path.name}")
    names={s.decode("ascii") for s in re.findall(rb"\bPRODUCT\(\s*'([^']*)'",path.read_bytes())}-{"SOLID","COMPOUND"}
    return cad.read_named(path,[{"id":n} for n in names])


def read(path):
    return json.loads(path.read_bytes())


def run():
    reg=read(HERE/"native.json")["red-place"]["mountedExport"]
    parts=load(HERE/"mounted-place.step",reg["sha256"])
    light=ROOT/"design/installed-procurement"
    carrier_sha=read(light/"clamp-review.json")["candidateStepSha256"]
    carriers=load(light/"inner-clamps.step",carrier_sha)
    bridge_folder=ROOT/"design/carriage-bridge"
    bridge_sha=read(bridge_folder/"native.json")["exportSha256"]
    bridge=load(bridge_folder/"bridge-place.step",bridge_sha)["slide-base-bridge-windowed"]
    bounds={k:cad.bounds(v) for k,v in parts.items()}
    carrier_hits={k:h for k,v in carriers.items() if (h:=contact(v,parts,bounds))}
    bridge_hits={}
    for pose,height in (("place",560),("transport",670),("receive",940),("stowed",940)):
        bridge_hits[pose]=contact(cad.translated(bridge,[0,0,height-560]),parts,bounds)
    # The rod extends another 10.2 mm above placement on the three higher poses.
    # Check the rod's entire extended cylindrical envelope, including between poses.
    extra_rods={tag:cad.brep(["cyl",[-60,y,392.7,0,0,1,3.15,17.5]]) for tag,y in (("L",298),("R",-298))}
    reserve_bounds={k:cad.bounds(v) for k,v in extra_rods.items()}
    extra={}
    for k,v in {**carriers,"bridge-low":bridge,"bridge-high":cad.translated(bridge,[0,0,380])}.items():
        hits=contact(v,extra_rods,reserve_bounds)
        if hits:
            extra[k]=hits
    if carrier_hits or any(bridge_hits.values()) or extra:
        raise ValueError("Selected subassemblies conflict")
    return {"mountedStep":reg["sha256"],"lightCarrierStep":carrier_sha,"windowedBridgeStep":bridge_sha,
            "mountedSolids":len(parts),"lightSolids":len(carriers),"carrierIntersectionsMm3":carrier_hits,
            "bridgeIntersectionsMm3":bridge_hits,"extendedRodIntersectionsMm3":extra,
            "minimumBridgeToRodLateralGapMm":min(abs(y)-3.15-max(abs(cad.bounds(bridge)[1]),abs(cad.bounds(bridge)[4])) for y in (-298,298))}


if __name__=="__main__":
    (HERE/"compatibility.json").write_text(json.dumps(run(),indent=2)+"\n",encoding="utf-8",newline="\n")
