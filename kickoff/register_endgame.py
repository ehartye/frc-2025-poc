"""Register alliance BARGE choices in the same period and roster as the piece programs."""

import copy
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
CAGE = ["cage-contact", "clear-carpet", "only-permitted-contacts"]
PARK = ["bumper-in-own-zone", "no-cage-credit"]


def register(profile):
    rehearsal = profile["rehearsal"]
    score = json.loads((HERE / "rules-v0.json").read_bytes())["scoring"]["barge"]
    profile["scoring"]["barge"] = score
    length, width = (profile["field"][k] for k in ("length", "width"))
    targets = []
    inventory = json.loads(
        (PROJECT / "experiments/six-wheel/leave-park/cage-motion.json").read_bytes()
    )
    for station, row in inventory["stations"].items():
        x, z = row["axis_mm"]
        center = [length / 2 + x / 25.4, width / 2 - z / 25.4]
        side = "red" if z > 0 else "blue"
        index = {
            "NAUO259": 3,
            "NAUO260": 2,
            "NAUO261": 1,
            "NAUO262": 1,
            "NAUO263": 2,
            "NAUO264": 3,
        }[station]
        # A 32.5-inch surrogate holds 29 inches from the cage axis. A 3.6875-inch
        # near face is 9.06 inches from its bumper. Service assumes a working arm.
        at = [center[0] + (29 if side == "red" else -29), center[1]]
        for level in ("shallow", "deep"):
            targets.append(
                dict(
                    id=f"{side}-{index}-{level}",
                    side=side,
                    label=f"{side.title()} cage {index} {level}",
                    at=at,
                    displayAt=center,
                    radiusIn=1.5,
                    resource=station,
                    capacity=1,
                    actions=[level],
                    setups=[level] + (["mixed"] if level == row["setup"] else []),
                )
            )
        # The older field lab extended each cage through the whole 44-inch BARGE.
        # Keep other BARGE elements; cage X and Y now follow the January field.
        obs = next(
            r
            for r in rehearsal["obstacles"]
            if r["name"] == f"{'South' if side == 'red' else 'North'} cage {index}"
        )
        cx, cy = center
        half = 7.375 / 2
        obs["poly"] = [
            [cx - half, cy - half],
            [cx + half, cy - half],
            [cx + half, cy + half],
            [cx - half, cy + half],
        ]
        obs["lower"]["mixed"] = obs["lower"][row["setup"]]
    red_park = [9600 / 25.4, 2427.2875 / 25.4]
    for side in ("red", "blue"):
        at = red_park if side == "red" else [length - red_park[0], width - red_park[1]]
        targets.append(
            dict(
                id=side + "-park",
                side=side,
                label=side.title() + " park lane",
                at=at,
                radiusIn=1,
                resource="reserved-park",
                capacity=1,
                actions=["park"],
            )
        )
    rehearsal["endgame"] = dict(
        period="TELEOP",
        assessment=dict(maxDelayS=3),
        targets=targets,
        actions=[
            dict(
                id="park",
                label="Park in own BARGE ZONE",
                points=score["parkPoints"],
                serviceS=1,
                requires=PARK,
            ),
            dict(
                id="shallow",
                label="Hang on shallow cage",
                points=score["shallowPoints"],
                serviceS=8,
                requires=CAGE,
            ),
            dict(
                id="deep",
                label="Hang on deep cage",
                points=score["deepPoints"],
                serviceS=12,
                requires=CAGE,
            ),
        ],
        basis="V0 §§5.4.1, 6.3.5 and 6.5. Match points awarded once per robot. Service and contact witnesses for peer robots are scenario inputs.",
        sources=[
            "Kickoff V0 §§6.5, 6.5.2, Table 6-2",
            "January FE-2025 hanging inventory",
            "Robot-specific empty south parking route",
        ],
    )
    matches = rehearsal["series"]["matches"]
    # Pre-endgame profiles are archived in git. Preserve three piece-focused teaching games.
    for i, setup in enumerate(("mixed", "deep", "shallow")):
        m = copy.deepcopy(matches[i])
        m.update(id="BARGE " + str(i + 1), setup=setup)
        m["basis"] = [
            "Compare a mixed-cage alliance with an incomplete support on the other side.",
            "Commit a cycling partner to deep climb; a late partner loses its service window.",
            "Shallow service on both sides. The team's loaded robot holds its actual unresolved endgame.",
        ][i]
        m["endgameAtRestAfterS"] = [3, 1.5, 0.5][i]

        def assign(side, slot, kind, station, start, **states):
            r = next(r for r in m["roster"] if r["side"] == side and r["slot"] == slot)
            needed = PARK if kind == "park" else CAGE
            witnesses = dict.fromkeys(needed, "met")
            witnesses.update(states)
            target = side + "-park" if kind == "park" else f"{side}-{station}-{kind}"
            r["endgame"] = dict(
                action=kind, target=target, startS=start, witnesses=witnesses
            )

        if i == 0:
            assign("red", 3, "deep", 3, 85, **{"clear-carpet": "failed"})
            assign("blue", 2, "deep", 3, 110)
            assign("blue", 3, "shallow", 2, 120)
        elif i == 1:
            assign("red", 1, "deep", 1, 95)
            assign("red", 3, "deep", 2, 85)
            assign("blue", 3, "deep", 2, 130)
        else:
            assign("blue", 3, "park", 0, 85)
            assign("red", 2, "shallow", 3, 85)
            assign("red", 3, "shallow", 2, 85)
        matches.append(m)
