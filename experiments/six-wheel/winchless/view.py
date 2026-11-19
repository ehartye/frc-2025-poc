"""Build an offline viewer of the authored fixture and its carriage contact surfaces."""
import argparse
import base64
import gzip
import importlib.util
import json
from pathlib import Path

from fixture import HERE, ROOT, j, read_kit


def build(carriage):
    template = ROOT / "design/competition-package"
    spec = importlib.util.spec_from_file_location("robot_surface_builder", template / "build.py")
    renderer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(renderer)

    fit = j.read(HERE / "fit.json")
    if j.digest(HERE / "service-stands.step") != fit["stepSha256"]:
        raise ValueError("Fixture geometry changed")
    if j.digest(carriage / "carriage-support.step") != fit["carriageSupportSha256"]:
        raise ValueError("Support geometry changed")
    stands = j.cad.read_named(HERE / "service-stands.step", [{"id": k} for k in fit["parts"]])
    support = read_kit(carriage, "carriage-support.step", "fit.json", "stepSha256", "volumesMm3")
    lift = ROOT / "design/receiver-lift"
    carriage_parts = j.cad.read_named(lift / "duplex-place.step", [
        {"id": k} for k in j.read(lift / "duplex-inventory.json")["poses"]["place"]])
    robot = {k: v for k, v in support.items()
             if k in ("support-machined-bridge", "support-machined-crossbar") or k.endswith("ribbed-bracket")}
    for side in ("L", "R"):
        for part in ("carriage-plate", "service-pin-rail"):
            key = f"backup-{side}-{part}"
            robot[key] = carriage_parts[key]

    data = {}
    for pose in ("approach", "supported", "keeper"):
        parts = []
        for key, shape in robot.items():
            if pose == "keeper" and key not in ("backup-L-carriage-plate", "backup-L-service-pin-rail"):
                continue
            parts.append([key, "Robot", "#497873", *renderer.triangulate(shape)])
        for key, shape in stands.items():
            location, side, part = key.split("-", 2)
            if pose == "keeper" and (location, side) != ("rear", "L"):
                continue
            if pose == "approach":
                if part.startswith("keeper"):
                    continue
                dx = 115 if location == "front" else 0
                dy = (105 if side == "L" else -105) if location == "rear" else 0
                shape = j.cad.translated(shape, (dx, dy, -2 if part == "screw" else 0))
            color = "#7d8fa6" if part == "frame" else "#c86424"
            parts.append([key, "Front stands" if location == "front" else "Rear stands",
                          color, *renderer.triangulate(shape)])
        data[pose] = {"height": 400, "stroke": 433, "parts": parts}

    page = (template / "viewer.template.html").read_text(encoding="utf8")
    substitutions = {
        "Station receiver": "Supporting the fixed carriage",
        "../../mission/moving-field.html": "../conditioned-match/replay.html",
        "Follow the shelf from pickup to placement. Rotate the robot or isolate a subsystem to inspect how its parts fit.":
            "Bring the two front stands in from the nose and the rear pair in from the sides. Zoom in on the rear keeper to see what captures the plate.",
        "Assembly review": "Service fixture", "Carriage pose": "Fixture position",
        "3D robot assembly": "3D service stands and carriage", "Whole robot": "Full fixture",
        "Remove bumpers and deck": "Hide robot parts",
        "(!p.name.startsWith('bumper')&&p.name!=='deck-main')": "p.group!=='Robot'",
        "Carriage height": "Rear contact height", "Slide extension": "Front contact height",
        "sceneCenter=[0,0,480];sceneSpan=1260;":
            "sceneCenter=pose==='keeper'?[-77,294,411]:[125,0,240];sceneSpan=pose==='keeper'?130:1120;",
        "const names={stowed:'Stowed',receive:'Receive',transport:'Carry',place:'Place'};":
            "const names={approach:'Approach',supported:'Four pads seated',keeper:'Left rear keeper'};",
        "pose='place'": "pose='supported'", "model.place.surfaces": "model.supported.surfaces",
        "setPose('place')": "setPose('supported')",
        "The lift changes height while the slide moves the shelf toward the station or trough. Use the pose controls to compare their motions.":
            "Orange parts adjust or retain the load. The rear pins withdraw toward the front to clear the service rails.",
        "Nominal CAD surfaces share the chassis frame. Purchased-part envelopes, cable routes and sensing clearances are design assumptions.":
            "The colored surfaces show nominal fit. The fixture needs an anchored tooling table and physical load qualification.",
        "@@MESH@@": base64.b64encode(gzip.compress(json.dumps(data, separators=(",", ":")).encode(),
                                                    mtime=0)).decode(),
    }
    for source, replacement in substitutions.items():
        if source not in page:
            raise ValueError("Viewer template changed at " + source[:40])
        page = page.replace(source, replacement)
    (HERE / "service.html").write_bytes(page.encode())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--carriage", type=Path, default=ROOT / "carriage-support")
    build(parser.parse_args().carriage)
