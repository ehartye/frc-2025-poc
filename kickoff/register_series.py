"""Register 2025 finite inventory and student match programs in the canonical profile."""

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
BASE = PROJECT / "experiments/six-wheel/kickoff/season-profile.json"
STOCK = PROJECT / "experiments/six-wheel/full-match/installed-stock.json"
SIDES = ("blue", "red")


def build():
    profile = json.loads(BASE.read_bytes())
    game = profile["rehearsal"]
    length, width = (profile["field"][k] for k in ("length", "width"))
    stations = {}

    def place(p, side):
        return list(p) if side == "blue" else [length - p[0], width - p[1]]

    def add(side, key, label, piece, kind, at, display, **kwargs):
        sid = f"{side}-{key}"
        stations[sid] = dict(
            id=sid,
            label=side.title() + " " + label,
            piece=piece,
            side=side,
            kind=kind,
            at=place(at, side),
            displayAt=place(display, side),
            radiusIn=6,
            **kwargs,
        )

    def pair(key, label, piece, kind, at, display, **kwargs):
        for side in SIDES:
            add(side, key, label, piece, kind, at, display, **kwargs)

    # Sixty supplied CORAL per alliance before three robot preloads are drawn.
    for sign, wall, dock in (
        ("low", (33.51, 25.8), (64, 53)),
        ("high", (33.51, 291.2), (64, 264)),
    ):
        pair(
            "chute-" + sign,
            sign + " corner chute",
            "CORAL",
            "retrieval",
            dock,
            wall,
            stock=30,
        )
    stock = json.loads(STOCK.read_bytes())
    floor_lanes = ("low", "mid", "high")
    blue_center = (176.75, 158.5)
    for native_id, obj in stock["stock"].items():
        x, _, z = obj["native_center_mm"]
        field = [length / 2 + x / 25.4, width / 2 - z / 25.4]
        side = "blue" if x < 0 else "red"
        normalized = place(field, side)
        group = obj["group"]
        if group == "reef-ball":
            dx, dy = normalized[0] - blue_center[0], normalized[1] - blue_center[1]
            theta = math.atan2(dy, dx)
            face = int(round(theta / (math.pi / 3))) % 6
            key = "reef-algae-" + str(face)
            robot = [
                blue_center[0] + 60 * math.cos(theta),
                blue_center[1] + 60 * math.sin(theta),
            ]
        else:
            lane = min(range(3), key=lambda i: abs(normalized[1] - (86.5 + i * 72)))
            key = (
                "floor-coral-" if group == "floor-pipe" else "floor-algae-"
            ) + floor_lanes[lane]
            robot = [normalized[0] + 22, normalized[1]]
        piece = "CORAL" if group == "floor-pipe" else "ALGAE"
        add(
            side,
            key,
            key.replace("-", " "),
            piece,
            "retrieval",
            robot,
            normalized,
            stock=1,
            nativeStockId=native_id,
        )
    targets = profile["scoring"]["targets"]
    # Four separately marked scoring approaches, with one representative branch per upper level.
    for index, (at, mark) in enumerate(
        (
            ((235, 158.5), (209.5, 158.5)),
            ((205.5, 108), (193.125, 130.14)),
            ((147.5, 209), (160.375, 186.86)),
            ((119, 158.5), (144, 158.5)),
            ((236, 24), (236, 0)),
            ((306, 213), (length / 2, 213)),
        )
    ):
        source = targets[index]
        level = index + 1
        key = f"l{level}" if level <= 4 else source["goal"]
        scoring = dict(
            points=dict(zip(("AUTO", "TELEOP"), source["points"])),
            criterion=(
                [
                    "CORAL contacts the trough and clears robot contact.",
                    "CORAL is fully supported on the selected L2 branch.",
                    "CORAL is fully supported on the selected L3 branch.",
                    "CORAL is fully supported on the selected L4 branch.",
                    "ALGAE passes through the processor opening.",
                    "ALGAE rests in the alliance net.",
                ][index]
            ),
        )
        if level in (2, 3, 4):
            scoring["capacity"] = 1
        pair(key, source["name"], source["piece"], "scoring", at, mark, **scoring)

    def step(key, operation, dwell, outcome=None):
        s = stations["blue-" + key]
        result = dict(
            at=s["at"],
            label=("Collect " if operation == "acquire" else "Attempt ") + key,
            dwell=dwell,
            station={side: side + "-" + key for side in SIDES},
            operation=operation,
        )
        if outcome:
            result["outcome"] = outcome
        return result

    acquire = lambda key: step(key, "acquire", 0.75)
    attempt = lambda key, outcome: step(key, "attempt", 0.9, outcome)

    def program(key, label, steps, loop=False):
        game["programs"][key] = dict(label=label, steps=steps, loop=loop)

    program(
        "preload-l2", "Place one preloaded CORAL on L2", [attempt("l2", "credited")]
    )
    program(
        "preload-l3", "Place one preloaded CORAL on L3", [attempt("l3", "credited")]
    )
    program(
        "preload-l4", "Place one preloaded CORAL on L4", [attempt("l4", "credited")]
    )
    program(
        "preload-l1",
        "Place one preloaded CORAL in the trough",
        [attempt("l1", "credited")],
    )
    program(
        "clear-low",
        "Clear south floor ALGAE for the processor",
        [acquire("floor-algae-low"), attempt("processor", "credited")],
    )
    program(
        "coral-low",
        "Cycle lower chute CORAL to the trough",
        [acquire("chute-low"), attempt("l1", "credited")],
        True,
    )
    program(
        "coral-high-miss",
        "Try upper chute CORAL and miss L1",
        [acquire("chute-high"), attempt("l1", "missed")],
    )
    program(
        "reef-net",
        "Remove one reef ALGAE and score in the net",
        [acquire("reef-algae-1"), attempt("net", "credited")],
    )
    program(
        "stow-at-station", "R4 rehearsal: take CORAL and hold", [acquire("chute-low")]
    )
    program(
        "keep-lane",
        "Clear the east reef face and hold a traffic lane",
        [dict(at=[290, 158.5], label="Back out of the trough approach", dwell=0.5)],
    )
    program(
        "approach-chute",
        "R4 rehearsal: stage outside the lower station",
        [dict(at=[94, 67], label="Stage outside the supply chute", dwell=0.5)],
    )
    game["stations"] = list(stations.values())
    game["defaults"]["capacity"] = 1
    # Retain the standard single-file queue program in slot one for the student clearance controls.
    game["programs"]["station"] = game["programs"].pop("coral-low")
    game["programs"]["processor"] = game["programs"].pop("clear-low")
    game["programs"]["net"] = game["programs"].pop("reef-net")

    def roster(blue, red):
        out = []
        for side, orders in zip(SIDES, (blue, red)):
            for slot, (auto, teleop) in enumerate(orders, 1):
                row = dict(side=side, slot=slot, plans=dict(AUTO=auto, TELEOP=teleop))
                if auto.startswith("preload-"):
                    row["preload"] = dict(
                        station=side + ("-chute-high" if slot == 3 else "-chute-low"),
                        quantity=1,
                    )
                out.append(row)
        return out

    feeder = [
        ("preload-l2", "station"),
        ("preload-l1", "processor"),
        ("preload-l3", "net"),
    ]
    contrast = [
        ("preload-l2", "coral-high-miss"),
        ("preload-l1", "processor"),
        ("preload-l4", "net"),
    ]
    clear_lane = [
        ("preload-l2", "station"),
        ("preload-l1", "keep-lane"),
        ("preload-l3", "net"),
    ]
    studied = [
        ("approach-chute", "stow-at-station"),
        ("preload-l1", "processor"),
        ("preload-l3", "net"),
    ]
    game["series"] = {
        "matches": [
            dict(
                id="Qual 1",
                basis="Station cycling and mixed ALGAE duties; prescribed peer scoring outcomes.",
                setup="mixed",
                roster=roster(feeder, contrast),
            ),
            dict(
                id="Qual 2",
                basis="Deeper cages. The red partner clears the trough lane for repeated CORAL cycles; blue misses a trough attempt.",
                setup="deep",
                roster=roster(contrast, clear_lane),
            ),
            dict(
                id="Qual 3",
                basis="Red slot 1 holds its first pipe; other robots are rehearsal surrogates.",
                setup="shallow",
                roster=roster(feeder, studied),
            ),
        ]
    }
    game["sources"].append(
        "V0 game rules and Table 6-2 for inventory and points. Floor/reef setup uses project FE-2025 native inventory. Approach centers, chute split, dwell and peer action outcomes are teaching assumptions."
    )
    from register_endgame import register
    register(profile)
    return profile


if __name__ == "__main__":
    (HERE / "season-profile.json").write_bytes(
        (json.dumps(build(), indent=2, ensure_ascii=False) + "\n").encode()
    )
