"""Continuous plan-view sweep of a fixed-heading box along an exported field-lab route."""
import json
import math
from pathlib import Path

MODEL = Path(__file__).resolve().parents[1] / "kickoff" / "rules-v0.json"
EPS = 1e-9


def projected(points, axis):
    values = [x*axis[0] + y*axis[1] for x, y in points]
    return min(values), max(values)


def edges(points):
    return zip(points, points[1:] + points[:1])


def hull(points):
    """Monotone-chain convex hull of the translated start and end footprints."""
    points = sorted(set(points))

    def half(seq):
        chain = []
        for x, y in seq:
            while len(chain) >= 2:
                a, b = chain[-2:]
                if (b[0]-a[0])*(y-a[1]) - (b[1]-a[1])*(x-a[0]) > EPS:
                    break
                chain.pop()
            chain.append((x, y))
        return chain

    return half(points)[:-1] + half(reversed(points))[:-1]


def edge_distance(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    u = max(0.0, min(1.0, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.dist(p, (a[0]+u*dx, a[1]+u*dy))


def sweep(a, b, footprint, obstacle):
    """Return the earliest contact fraction and minimum unsigned boundary gap in inches."""
    start = [(x+a[0], y+a[1]) for x, y in footprint]
    delta = (b[0]-a[0], b[1]-a[1])
    entry, leave = 0.0, 1.0
    for p, q in [*edges(start), *edges(obstacle)]:
        edge_length = math.dist(p, q)
        axis = ((q[1]-p[1])/edge_length, (p[0]-q[0])/edge_length)
        amin, amax = projected(start, axis)
        bmin, bmax = projected(obstacle, axis)
        speed = delta[0]*axis[0] + delta[1]*axis[1]
        if abs(speed) < EPS:
            if amax < bmin-EPS or bmax < amin-EPS:
                entry = math.inf
                break
            continue
        u, v = (bmin-amax)/speed, (bmax-amin)/speed
        entry, leave = max(entry, min(u, v)), min(leave, max(u, v))
    if entry <= leave + EPS:
        return max(0, min(entry, 1)), 0.0
    swept = hull(start + [(x+delta[0], y+delta[1]) for x, y in start])
    gap = min(edge_distance(p, u, v) for lhs, rhs in [(swept, obstacle), (obstacle, swept)]
              for p in lhs for u, v in edges(rhs))
    return None, gap


def regular_landmark(m):
    radius = m["acrossFlats"] / (2*math.cos(math.pi/m["sides"]))
    return [(m["x"]+radius*math.cos((2*i+1)*math.pi/m["sides"]),
             m["y"]+radius*math.sin((2*i+1)*math.pi/m["sides"])) for i in range(m["sides"])]


def check_route(model, route, projection):
    # Neutral 30-inch square from the field exercise. Offset only the forward (+X) face.
    shape = [(-15, -15), (15+projection, -15), (15+projection, 15), (-15, 15)]
    along = 0
    closest = math.inf
    first = []
    for a, b in zip(route["routeIn"], route["routeIn"][1:]):
        length = math.dist(a, b)
        for mark in model["landmarks"]:
            fraction, gap = sweep(a, b, shape, regular_landmark(mark))
            closest = min(closest, gap)
            if fraction is not None:
                first.append((along+length*fraction, mark["name"]))
        along += length
    hit = min(first) if first else None
    return {"piece": route["piece"], "goal": route["destination"], "projectionIn": projection,
            "minimumPlanarGapIn": closest, "firstContactDistanceIn": hit[0] if hit else None,
            "firstLandmark": hit[1] if hit else None}


def main():
    model = json.loads(MODEL.read_text(encoding="utf-8"))
    rows = [check_route(model, route, extension) for route in model["routeHypotheses"]
            if route["phase"] == "TELEOP" for extension in [0, 8]]
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
