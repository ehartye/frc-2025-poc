"""Verify station approach clearance after replacing the receiver and camera supports."""
from bridge import argparse, cad, configuration, HERE, json, Path, sha, source
from installed import bounded_dock, native_dock, parts as station_parts, FIRST_SHA

def run(kit, field):
    if sha(field) != FIRST_SHA:
        raise ValueError("Field source digest differs from the registered January CAD")
    station = station_parts(field)
    robot = configuration(30, 0, 40, kit)
    field_boxes = {key: cad.bounds(shape) for key, shape in station.items()}
    boxes = {key: cad.bounds(shape) for key, shape in robot.items()}
    return {
        "fieldSha": FIRST_SHA,
        "bridgeSha": sha(HERE / "revised-bridge.step"),
        "robotBodies": len(robot),
        "stationOccurrences": len(station),
        "standoffMm": 40,
        "normalAndLateralErrorMm": 10,
        "chassisYawErrorDegrees": 3,
        "approach": bounded_dock(boxes, field_boxes, 40),
        "nativeAdverseControl": native_dock(robot, station, field_boxes, 0, 3)
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--field", type=Path, required=True)
    parser.add_argument("--lamp", type=Path, required=True)
    args = parser.parse_args()
    result = run(source(args.lamp), args.field)
    (HERE / "station.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n")
