"""Check the actual occupied reef balls against the OD assembly at both scoring sectors."""
from dataclasses import replace
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from vision import argparse, cad, cases, configuration, field_at, field_source, HERE, json, Path, sha, source
from dock_tolerance import separation

def closest(robot, stock):
    robot_boxes = {k: cad.bounds(s) for k, s in robot.items()}
    stock_boxes = {k: cad.bounds(s) for k, s in stock.items()}
    possible = sorted((separation(b, v), k, n) for k, b in robot_boxes.items() for n, v in stock_boxes.items())
    closest_pair = None
    hits = {}
    for lower, part, ball in possible:
        if closest_pair and lower > closest_pair[0] + .001:
            break
        measure = BRepExtrema_DistShapeShape(robot[part], stock[ball])
        if not measure.IsDone():
            raise ValueError("Unresolved native stock distance")
        row = (measure.Value(), part, ball)
        closest_pair = min(row, closest_pair) if closest_pair else row
        if row[0] < .01:
            overlap = cad.intersection(robot[part], stock[ball])
            if overlap > .05:
                hits[part + " / " + ball] = overlap
    return {"minimumMm": closest_pair[0], "nearest": closest_pair[1:], "intersectionsMm3": hits}

def run(kit, field):
    data = {}
    reef = {k: shape for k, shape in field[1].items() if k.startswith("installed-ball-")}
    for label, case in cases().items():
        for sector in (-.18, .22):
            for state, opening, slide, approach in (("closed", 0, 370, 0), ("open", 112, 370, 0),
                                                   ("forward-error", 112, 370, .025)):
                displaced = replace(case, sector=sector, target_error=case.target_error+approach)
                stock = field_at(({}, reef), displaced, False)
                robot = configuration(slide, opening, 0, kit)
                data[f"{label}/{sector}/{state}"] = closest(robot, stock)
    return {"bridgeSha": sha(HERE/"revised-bridge.step"), "reefBalls": len(reef), "samples": data,
            "scope": "Exact native installed-ball checks at enumerated endpoints and 25 mm additional forward chassis error with the slide held at its physical limit. Continuous field collision and physical measurements remain open."}

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--lamp", type=Path, required=True)
    parser.add_argument("--field", type=Path, required=True)
    args = parser.parse_args()
    result = run(source(args.lamp), field_source(args.field))
    (HERE/"occupied.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf8", newline="\n")
