"""Use one station receiving lane to compare equal-duration partner reservations."""
import json
import math

if __package__:
    from .kickoff_shuttle import read
else:
    from kickoff_shuttle import read


def next_pickup(arrival, duration, reservations):
    """Exclusive, non-preemptive service. Adjacent half-open intervals can touch."""
    start = arrival
    for begin, end in sorted(reservations):
        if start + duration <= begin:
            break
        if start < end:
            start = end
    return start


def schedule(trial, session, route, speed, reservations):
    distance = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
    travel = distance / speed
    now = session["startingSetupSeconds"]
    pieces = []
    for n in range(1, trial["allocatedAtSource"]+1):
        at_source = now if n == 1 else now + travel
        pickup = next_pickup(at_source, trial["pickupSeconds"], reservations)
        now = pickup + trial["pickupSeconds"] + travel + trial["releaseSeconds"]
        pieces.append({"piece": n, "requestedPickupS": at_source, "pickupStartS": pickup,
                       "waitS": pickup-at_source, "scoreCompleteS": now})
    achieved = sum(p["scoreCompleteS"] <= session["windowSeconds"] for p in pieces)
    goal_finish = pieces[trial["goal"]-1]["scoreCompleteS"]
    return {"routeLengthIn": distance, "oneWayTravelS": travel, "pieces": pieces,
            "piecesByDeadline": achieved, "goalMarginS": session["windowSeconds"]-goal_finish}


def main():
    model, session = read("rules-v0.json"), read("trials.json")
    path = next(r["routeIn"] for r in model["routeHypotheses"]
                if r["phase"] == "TELEOP" and r["piece"] == "CORAL")
    route = [[48,45], *path[1:]]
    # A 2.5-second partner request shifts in time between otherwise identical trials.
    cases = {"uncontended": [], "partner_at_opening": [(0.5,3)], "partner_during_transit": [(2.1,4.6)]}
    results = {case: schedule(session["trials"][0], session, route, 80, busy)
               for case, busy in cases.items()}
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
