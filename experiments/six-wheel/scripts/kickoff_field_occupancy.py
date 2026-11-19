"""Screen a fixed-heading trial box against an archived corner-plane proxy and a timed choke."""
import json
import math

if __package__:
    from .kickoff_route_clearance import MODEL, check_route
else:
    from kickoff_route_clearance import MODEL, check_route


def boundary_gap(model, constraints, center, half_size):
    x, y = center
    limits = model["field"]
    gaps = {"Blue wall": x-half_size, "Red wall": limits["length"]-x-half_size,
            "Lower rail": y-half_size, "Upper rail": limits["width"]-y-half_size}
    for face in constraints["stationSurvey"]["faces"]:
        angle = math.radians(face["inwardDeg"])
        nx, ny = math.cos(angle), math.sin(angle)
        fx, fy = face["xyIn"]
        # Support of an axis-aligned square projected onto the station normal.
        gaps["Station "+str(face["id"])] = (x-fx)*nx+(y-fy)*ny-half_size*(abs(nx)+abs(ny))
    closest = min(gaps, key=gaps.get)
    return {"surface": closest, "gapIn": gaps[closest]}


def choke_screen(model, constraints, phase, requests):
    trial, barge = constraints["queueTrial"], constraints["barge"]
    c1, c2 = barge["cageCentersFromMidfieldIn"][:2]
    opening = c2-c1-barge["cageWidthIn"]
    duration = (barge["depthIn"]+trial["boxLengthIn"])/trial["speedInPerSecond"]
    deadline = next(p["seconds"] for p in model["scoring"]["phases"] if p["name"] == phase)
    available = 0
    crossings = []
    for request in sorted(requests):
        enter = max(available, request)
        exit_time = enter+duration
        crossings.append({"requestSeconds": request, "enterSeconds": enter, "exitSeconds": exit_time,
                          "waitSeconds": enter-request, "completesWithinActivePhase": exit_time <= deadline})
        available = exit_time
    return {"openingIn": opening, "trialWidthMarginIn": opening-trial["boxWidthIn"],
            "comparisonWidthMarginIn": opening-trial["comparisonWidthIn"],
            "twoTrialWidthsFit": 2*trial["boxWidthIn"] <= opening,
            "durationSeconds": duration, "phase": phase, "phaseLengthSeconds": deadline,
            "reservationHypotheses": crossings}


def cage_height_screen(barge, height):
    # A solid vertical column from the carpet up to this height at a cage's footprint.
    return {kind: height >= barge[kind+"BottomIn"] for kind in ("deep", "shallow")}


def main():
    model = json.loads(MODEL.read_text(encoding="utf-8"))
    c = json.loads((MODEL.parent / "field-constraints.json").read_text(encoding="utf-8"))
    coral = next(r for r in model["routeHypotheses"] if r["phase"] == "TELEOP" and r["piece"] == "CORAL")
    net = next(r for r in model["routeHypotheses"] if r["phase"] == "TELEOP" and r["destination"] == "net")
    reroute = {**net, "routeIn": [net["routeIn"][0], [228, 101], net["routeIn"][-1]]}
    reroute_length = sum(math.dist(a,b) for a,b in zip(reroute["routeIn"],reroute["routeIn"][1:]))
    result = {
        "stationPickupAtModelPose": boundary_gap(model, c, coral["routeIn"][0], 15),
        "stationPickupAtTrialPose48_45": boundary_gap(model, c, [48,45], 15),
        "netDetourIn": reroute["routeIn"], "netDetourLengthIn": reroute_length,
        "netDetourExtraSecondsAt60InPerSecond": (reroute_length-net["lengthIn"])/60,
        "netDetourReefScreen": check_route(model, reroute, 0),
        "cageVerticalOverlapFor24InBox": cage_height_screen(c["barge"], 24),
        "cageVerticalOverlapFor30InBox": cage_height_screen(c["barge"], 30),
        "choke": choke_screen(model, c, "AUTO", c["queueTrial"]["arrivalTimesAutoSeconds"])
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
