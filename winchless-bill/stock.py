"""Reopen the old lift at H560 and charge a finite envelope for residual material."""

import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "drive-side-attachment"))
import joint as j

sys.path.insert(0, str(ROOT / "design/receiver-lift"))
from check_routing import routing

DENSITY = 8100  # receiving maximum for the specified steels, kg/m³
SIZE_ALLOWANCE = 1.0  # total oversize on each of the three full dimensions, mm
SITES = (
    "anchor-tab-L",
    "anchor-tab-R",
    "bearing-arm-L",
    "bearing-arm-R",
    "sheave-support-L",
    "sheave-support-R",
    "mount-riser",
)
NATIVE = "design/competition-package/poses/place.step"


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def read(rel):
    return json.loads((ROOT / rel).read_bytes())


def load_step(rel, names, pin):
    path = ROOT / rel
    if sha(path) != pin:
        raise ValueError(f"Native exchange changed: {rel}")
    return j.cad.read_named(path, [{"id": name} for name in names])


def source_bill():
    old_review = read("design/stability/review.json")
    frames = read("design/competition-package/part-frames.json")
    old = load_step(NATIVE, frames, old_review["poses"]["place"]["step_sha256"])
    original = routing(560)
    if not set(original) <= old.keys() or len(original) != 34:
        raise ValueError("Incomplete original lift")
    m = read("guide-setting/fit.json")
    setting = load_step("guide-setting/setting.step", m["volumesMm3"], m["stepSha256"])
    c = read("carriage-support/fit.json")
    carriage = load_step(
        "carriage-support/carriage-support.step", c["volumesMm3"], c["stepSha256"]
    )
    deck = "mast-slotted-ribbed-deck"
    current = {
        **load_step(
            "mast-attachment/slotted-ribbed-deck.step",
            [deck],
            read("mast-attachment/fit.json")["ribbedSlottedSha256"],
        ),
        "support-machined-crossbar": carriage["support-machined-crossbar"],
    }
    for side in ("L", "R"):
        name = f"guide-{side}-machined-mast"
        current["match-drilled/" + name] = setting[name]
    rows = {}
    for site in SITES:
        bounds = j.cad.bounds(old[site])
        native_volume = j.cad.volume(old[site])[0]
        if abs(native_volume - j.cad.volume(original[site])[0]) > 1e-3:
            raise ValueError(f"Original drawing and STEP differ: {site}")
        sizes = [bounds[k + 3] - bounds[k] for k in range(3)]
        if site.startswith("anchor-tab"):
            owner = "support-machined-crossbar"
        elif site == "mount-riser":
            owner = deck
        else:
            owner = "match-drilled/guide-" + site[-1] + "-machined-mast"
        overlap = j.cad.intersection(old[site], current[owner])
        rows[site] = dict(
            parent=owner,
            robotBoundsMm=[round(x, 5) for x in bounds],
            originalSolidMm3=round(native_volume, 5),
            materialSharedWithParentMm3=round(overlap, 5),
            maxSizeMm=[round(v + SIZE_ALLOWANCE, 5) for v in sizes],
            finishedStockUpperKg=math.prod(s + SIZE_ALLOWANCE for s in sizes)
            * DENSITY
            / 1e9,
        )
    # Pessimistic two M6 through attachments per original site. Enclose the real #113
    # screw, sleeve, washer and nut (including each void) in four solid cylinders.
    reference = {}
    for kind in ("screw", "sleeve", "washer", "locknut"):
        name = "support-L-mast-0-" + kind
        box = j.cad.bounds(carriage[name])
        x, y, z = [box[i + 3] - box[i] for i in range(3)]
        # A circle enclosing the entire end bounding rectangle supports hex orientations.
        diameter = math.hypot(x, z) + SIZE_ALLOWANCE
        length = y + SIZE_ALLOWANCE
        reference[kind] = dict(
            source=name,
            diameterMm=diameter,
            lengthMm=length,
            upperKg=math.pi / 4 * diameter**2 * length * DENSITY / 1e9,
            nativeNominalKg=j.cad.volume(carriage[name])[0] * 7850 / 1e9,
        )
    # The oversized screw head rectangle encompasses the whole shank here. Use two
    # separately derived coaxial cylinders to keep a shop-verifiable upper practical.
    screw = reference["screw"]
    screw["singleCylinderUpperKg"] = screw["upperKg"]
    screw["headDiameterMm"] = 13.0
    screw["headLengthMm"] = 5.0
    screw["shankDiameterMm"] = 6.5
    screw["shankLengthMm"] = 44.0
    screw["upperKg"] = math.pi / 4 * DENSITY / 1e9 * (13**2 * 5 + 6.5**2 * 44)
    raw = carriage[screw["source"]]
    # Reference source head at Y197..201 and shank Y201..244, axis along +Y.
    b = j.cad.bounds(raw)
    x = (b[0] + b[3]) / 2
    z = (b[2] + b[5]) / 2
    limit = j.fuse(
        j.rod(x, 1, 196.5, 201.5, z, 6.5), j.rod(x, 1, 200.5, 244.5, z, 3.25)
    )
    excess = j.cad.volume(raw)[0] - j.cad.intersection(raw, limit)
    if excess > 1e-4:
        raise ValueError("The source screw does not fit its declared gage")
    screw["sourceOutsideTwoCylinderGaugeMm3"] = round(max(0.0, excess), 6)
    mass_stock = sum(v["finishedStockUpperKg"] for v in rows.values())
    per_fastener = sum(v["upperKg"] for v in reference.values())
    opaque = original["sport-ratchet-and-mount-reserve"]
    opaque_mass = j.cad.volume(opaque)[0] * DENSITY / 1e9
    paths = [
        NATIVE,
        "design/stability/review.json",
        "design/competition-package/part-frames.json",
        "design/receiver-lift/check_routing.py",
        "design/receiver-lift/layer_inventory.py",
        "guide-setting/setting.step",
        "guide-setting/fit.json",
        "carriage-support/carriage-support.step",
        "carriage-support/fit.json",
        "mast-attachment/slotted-ribbed-deck.step",
        "mast-attachment/fit.json",
    ]
    return dict(
        sources={p: sha(ROOT / p) for p in paths},
        originalRoutingIds=sorted(original),
        originalBodies=len(old),
        machinedParentIds=sorted(current),
        retainedStock=rows,
        permittedMaxDensityKgM3=DENSITY,
        dimensionalOversizeMm=SIZE_ALLOWANCE,
        throughSet=reference,
        throughSetsPerSite=2,
        totalSets=2 * len(rows),
        stockUpperKg=mass_stock,
        throughSetUpperKg=per_fastener,
        fastenerContingencyUpperKg=14 * per_fastener,
        stockAndFastenerUpperKg=mass_stock + 14 * per_fastener,
        opaqueSportSolidBoxSensitivityKg=opaque_mass,
        installedStockMeasuredKg=None,
    )


if __name__ == "__main__":
    j.write(HERE / "stock.json", source_bill())
