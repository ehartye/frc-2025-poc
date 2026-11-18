"""Geometry, kinematic frames and mass hypotheses for integrating the isolated retention layer."""
import json

from duplex import REPLACES, native, volume


def metadata(name):
    if name.endswith("return-line"):
        return "variable-rope", "braid-envelope", 1100
    if name=="lift-crossbar-drilled" or any(t in name for t in ("carriage-", "clevis", "thimble", "spliced-eye")):
        material = "aluminum" if (name=="lift-crossbar-drilled" or "carriage-plate" in name) else (
            "braid-envelope" if "spliced-eye" in name else "steel")
        return "lift", material, {"aluminum":2700,"braid-envelope":1100,"steel":7850}[material]
    if name.endswith("running-line"):
        return "base", "braid-envelope", 1100
    steel = name=="long-common-shaft" or any(t in name for t in (
        "mast-bolt", "mast-spacer", "outboard-bearing", "upper-pin", "service-pin-bushing"))
    return ("base", "steel", 7850) if steel else ("base", "aluminum", 2700)


def inventory():
    poses={}
    for k,h in (("place",560),("transport",670),("stowed",940)):
        members={}
        for name,shape in native(h).items():
            frame,material,rho=metadata(name)
            v,c=volume(shape)
            members[name]={"frame":frame,"materialBasis":material,"massKg":rho*v*1e-9,"centroidMm":list(c)}
        poses[k]=members
    poses["receive"]=poses["stowed"]
    return {"schema":"receiver-retention-layer-inventory/1","replaces":sorted(REPLACES),"poses":poses,
            "additionalUnplacedAllowancesKg":{"moving":.25,"fixed":.35},
            "basis":"Uniform geometric density estimates. Four original bodies are replaced, not additional mass."}


if __name__=="__main__":
    print(json.dumps(inventory(),indent=2,allow_nan=False))
