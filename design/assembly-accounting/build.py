"""Build source-pinned Agent FRC inputs for a single alliance configuration."""
import argparse
import csv
import hashlib
import json
import os
import re
from collections import defaultdict
from decimal import Decimal
from functools import cache
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PROC = ROOT / "design/installed-procurement"
ROBOT = ROOT / "design/competition-package/shop-duplex"
STOP = ROOT / "design/lift-arrest"
POSES = ("stowed", "receive", "transport", "place")
REVISION = "ec188e69828e32c610de3e208378edb7f6e1566e"
DISPOSITION = {"make": "make", "commodity": "buy", "buy candidate": "buy", "selection open": "undecided"}
BEARING = "REV-21-1916-PK4 rounded-hex bearing"


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def rows(path):
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


@cache
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@cache
def step_names(path):
    return {v.decode("ascii") for v in re.findall(rb"\bPRODUCT\(\s*'([^']*)'", path.read_bytes())} - {"SOLID", "COMPOUND"}


def evidence(source, pointer, value):
    return {"source": source, "pointer": pointer, "rangeKg": value if isinstance(value, list) else [value, value], "basis": "estimated"}


def build(color, output, selected=()):
    configuration = f"shop-duplex-545-{color}"
    trio = {"revision": REVISION, "configuration": configuration}
    pins = []
    exports = []
    assignments = []
    instances = {}
    articles = {}

    def pin(label, filename):
        pins.append({"id": label, "file": Path(os.path.relpath(filename, output.parent)).as_posix(), "sha256": digest(filename)})

    def physical(identity, presence, disposition, article=None):
        record = {"id": identity, "presence": presence, "disposition": disposition}
        if disposition == "buy":
            record["article"] = article
            articles.setdefault(article, {"id": article, "sku": None, "unitsPerPack": None, "onHand": None})
        if identity in instances and record != instances[identity]:
            raise ValueError(f"Inconsistent physical instance {identity}")
        instances[identity] = record
        return identity

    for key, file in {"crosswalk": PROC/"installed.csv", "rollup": PROC/"item-rollup.csv", "open-items": PROC/"loose-items.csv",
                      "inspection-limit": HERE/"inspection-limit.json", "rule-sources": ROOT/"design/electrical-inspection/SOURCES.md",
                      "purchase": ROOT/"design/shop-package/purchasing.csv", "mass": ROBOT/"mass.json", "mass-model": ROBOT/"mass.py",
                      "frames": ROBOT/color/"part-frames.json", "reopen": ROBOT/color/"review.json",
                      "light": PROC/"clamp-review.json", "steel": PROC/"steel-review.json", "clamp-step": PROC/"inner-clamps.step",
                      "clamp-builder": PROC/"clamps.py", "stop": STOP/"study.json", "stop-step": STOP/"end-stop-place.step",
                      "stop-attachment": STOP/"README.md"}.items():
        pin(key, file)
    frames = read_json(ROBOT/color/"part-frames.json")
    opened = read_json(ROBOT/color/"review.json")
    crosswalk = rows(PROC/"installed.csv")
    names = {r[color+"Id"] for r in crosswalk}
    if set(frames) != names or len(crosswalk) != len(names):
        raise ValueError("Duplicate or unmatched installation crosswalk occurrence")
    for pose in POSES:
        source = ROBOT/color/(pose+".step")
        if (step_names(source) != names or opened[pose]["bodies"] != len(names)
                or {r[color+"Id"]: r["frame"] for r in crosswalk} != frames
                or opened[pose]["export_sha256"] != digest(source) or opened[pose]["alliance"] != color):
            raise ValueError("Pose native IDs or source no longer match: "+pose)
        pin(pose+"-step", source)
        exports.append({"id": pose, "source": "frames", "pointer": "", "variant": {"group": "bumper", "choice": color}})

    groups = defaultdict(set)
    for row in crosswalk:
        occurrence = row[color+"Id"]
        route = row["route"]
        assignment = {"occurrence": occurrence}
        if route == "space":
            assignment["role"] = "service"
        elif route == "included":
            if (occurrence, row["instance"]) != ("slide-position-line", "slide-position-transducer"):
                raise ValueError("Unknown included-part parent alias")
            assignment.update(role="included", instance="slide-position-SP1-25")
        else:
            identity = physical(row["instance"], "modeled", DISPOSITION[route], row["item"])
            groups[route, row["item"]].add(identity)
            assignment.update(role="installed", instance=identity)
        assignments.extend({"export": pose, **assignment} for pose in POSES)
    for row in rows(PROC/"item-rollup.csv"):
        if len(groups[row["route"],row["item"]]) != int(row["representedUnitsPerAlliance"]):
            raise ValueError("Purchased-article rollup changed: "+row["item"])

    purchasing = {row["item"]: row for row in rows(ROOT/"design/shop-package/purchasing.csv")}
    bearing = purchasing["REV-21-1916-PK4 rounded hex bearings"]
    if len(groups["buy candidate",BEARING]) != int(bearing["installedQuantity"]) or "three packs of four" not in bearing["orderUnitOrDecision"]:
        raise ValueError("Bearing pack evidence changed")
    for name, article in articles.items():
        # Only a single supplier identity from the grouped list identifies one SKU.
        # The 2109 assembly needs cartridges and a socket in addition to its base.
        if name.startswith("REV-25-2109 with "):
            continue
        match = re.match(r"(?:AndyMark )?((?:REV-|am-)[A-Za-z0-9_-]+)(?: |$)", name)
        if match:
            article["sku"] = match.group(1)
    articles[BEARING]["unitsPerPack"] = 4
    for i in range(int(purchasing["Wheel-to-hub fasteners"]["installedQuantity"])):
        physical(f"unmodeled-wheel-to-hub-screw-{i+1:02d}", "unmodeled", "buy", "Wheel-to-hub fasteners")

    changes = []
    light_parts = step_names(PROC/"inner-clamps.step")
    for material in ("light", "steel"):
        receipt = read_json(PROC/("clamp-review.json" if material == "light" else "steel-review.json"))
        parts = light_parts if material == "light" else {
            p.replace("pocket-carrier", "threaded-carrier") for p in light_parts if not p.endswith("square-nut")}
        if len(parts) != receipt["reopenedNamedBodies"] or not all(p.startswith("inner-") for p in parts):
            raise ValueError("Carrier inventory differs from the native review")
        added = []
        for p in sorted(parts):
            kind = next((k for k in ("washer","screw","square-nut") if p.endswith(k)), None)
            # A keyed bill item identifies geometry and drawing while exact vendor SKUs remain outstanding.
            station = "center" if "-center-" in p else "end"
            interface = "adapter" if "adapter" in p else "mount"
            article = f"{material} {station} {interface} {kind}" if kind else None
            disposition = "make" if kind in (None, "square-nut") else "buy"
            added.append(physical(material+"/"+p,"candidate",disposition,article))
        changes.append({"id": material,"exclusiveGroup": "inner-carriers", **trio,
                        "remove": receipt["replaces"], "add": added,"delta": evidence(material,"/deltaKg",receipt["deltaKg"])})
    native_stops = {side+"-"+item for side in "LR" for item in ("bracket-reserve","body-reserve","rod","nut-reserve")}
    if step_names(STOP/"end-stop-place.step") != native_stops:
        raise ValueError("Stop bodies changed")
    attached = []
    for side in "LR":
        attached.append(physical("stop-bracket-"+side,"candidate","make"))
        attached.append(physical("stop-absorber-"+side,"candidate","buy","ACE MC225EUM damper (body and rod)"))
        attached.append(physical("stop-nut-"+side,"candidate","undecided"))
        for index in (1,2):
            attached.append(physical(f"stop-mount-bolt-{side}{index}","candidate","buy","Provisional sleeved through-bolt"))
            attached.append(physical(f"stop-mount-sleeve-{side}{index}","candidate","undecided"))
    stop_mass = read_json(STOP/"study.json")["bracketFit"]["place"]["export"]["massWithoutFastenersKg"]
    changes.append({"id":"stop", **trio, "remove":[], "add":attached,
                    "delta":evidence("stop","/bracketFit/place/export/massWithoutFastenersKg",stop_mass)})

    mass_source = read_json(ROBOT/"mass.json")["poses"]["stowed"]
    mass = mass_source["R103_kg"]
    limit_source = read_json(HERE/"inspection-limit.json")
    limit = limit_source["R103LimitKg"]
    converted = Decimal(str(limit_source["kilogramsPerPound"]))*limit_source["R103LimitLb"]
    known_authority = (ROOT/"design/electrical-inspection/SOURCES.md").read_text(encoding="utf-8")
    if (Decimal(str(limit)) != converted or limit_source["sourcePdfSha256"] not in known_authority
            or limit_source["sourceUrl"] not in known_authority):
        raise ValueError("Inspection comparison source or unit conversion changed")
    if abs(limit - mass[1] - mass_source["R103_margin_at_high_kg"]) > 1e-12:
        raise ValueError("R103 limit differs from the project mass source")
    return {"schema":"agent-frc/assembly-accounting@1", **trio, "scope":"partial",
            "omissions":["Loose-items register has unresolved installation quantities and hardware selections.",
                         "Spare stock and the opposite-color kit are outside this installed configuration."],
            "sources":pins,"exports":exports,"assignments":assignments,"instances":list(instances.values()),
            "articles":list(articles.values()),"changes":changes,"selected":list(selected),
            "mass":{"unit":"kg",**trio,"limitKg":limit,"limitEvidence":{"source":"inspection-limit","pointer":"/R103LimitKg"},
                    "baseline":evidence("mass","/poses/stowed/R103_kg",mass),
                    "omissions":["No installed weigh-in. Baseline hardware and consumables remain estimates.",
                                 "Candidate stop margin precedes mounting bolts, sleeves and final attachment details.",
                                 "Additional full-travel carriage arrest and unfinished design changes have no credited mass."]}}


def write_map(path, data):
    # Keep each occurrence and physical article visible on its own line for Git review.
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for key, value in data.items():
        body = json.dumps(value, ensure_ascii=True, separators=(",", ":"))
        if isinstance(value,list) and value and isinstance(value[0],dict):
            body = "[\n    "+",\n    ".join(json.dumps(row,ensure_ascii=True,separators=(",",":")) for row in value)+"\n  ]"
        lines.append("  "+json.dumps(key)+": "+body)
    path.write_text("{\n"+",\n".join(lines)+"\n}\n",encoding="utf-8",newline="\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("color",choices=("red","blue"))
    parser.add_argument("output",type=Path)
    parser.add_argument("--select",nargs="*",choices=("light","steel","stop"),default=[])
    args = parser.parse_args()
    write_map(args.output.resolve(),build(args.color,args.output.resolve(),args.select))
