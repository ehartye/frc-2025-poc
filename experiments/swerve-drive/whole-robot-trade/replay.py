"""Replay field traffic and finite stock, then build project-owned study inputs."""

import argparse
import copy
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

from assess_robot import HERE, ROOT, report, write
from custody_review import run as compare_custody


def ref(file, pointer):
    p = HERE / file
    return dict(
        path=p.relative_to(ROOT).as_posix(),
        sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
        pointer=pointer,
    )


def spatial_study(hardening):
    """Test four real CG corners about both axes of the front jack."""
    model = copy.deepcopy(hardening["elevation"])
    model["geometry"] = "spatial-vertical-supports"
    model["massBasis"] += (
        " Both screw axes lie at X305 mm and are 642 mm apart across the chassis."
        " Each vertical-only case reports the reaction moment still required from the structure."
    )
    for support in model["supports"]:
        support["xyM"] = [0.305, support.pop("xM")]
    existing = {case["id"]: case for case in model["cases"]}
    model["cases"] = []
    for cg_x in (-0.060, 0.060):
        for cg_y in (-0.040, 0.040):
            corner = "".join("minus" if value < 0 else "plus" for value in (cg_x, cg_y))
            for condition in ("full-travel-upward", "disabled-at-cage", "stationary-bottom-catch", "one-side-missing"):
                case = copy.deepcopy(existing[condition])
                del case["cgXM"], case["pitchMomentNm"]
                case.update(
                    id=f"{condition}-{corner}",
                    cgXYM=[cg_x, cg_y],
                    appliedMomentXYNm=[0, 0],
                    basis="Chassis-frame vertical reactions at 319.4 mm rise with empty, stowed rack. The balance supplies the unresolved roll and pitch couples.",
                )
                model["cases"].append(case)
    write(HERE / "spatial-inputs.json", {
        "inventory": hardening["inventory"],
        "jack": model,
        "origin": "Operating mass and screw axes from robot-cases.json. The crosshead, mount and cage require separate capacity evidence.",
    })
    write(HERE / "spatial-study.json", {
        "schema": "agent-frc/elevation-study@1",
        "inventory": ref("spatial-inputs.json", "/inventory"),
        "mechanisms": [{"id": hardening["inventory"][0], "source": ref("spatial-inputs.json", "/jack")}],
    })


CRITERION = dict(
    criterionId="red-l1-coral",
    criterion="One distinct pipe acquired at the assigned chute and credited at red L1 during TELEOP.",
    focalRobot="R1",
    windowScope="Complete TELEOP from its first active sample through cutoff",
)


def field_episode(scripts, source, name, hold, stock):
    profile = copy.deepcopy(source)
    r = profile["rehearsal"]
    match = next(m for m in r["series"]["matches"] if m["id"] == "Qual 2")
    r["series"]["matches"] = [match]
    r["programs"]["keep-lane"]["steps"][0]["at"][0] = hold
    next(s for s in r["stations"] if s["id"] == "red-chute-low")["stock"] = stock
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "profile.json"
        write(p, profile)
        trial = Path(d) / "trial.json"
        subprocess.run(
            ["node", str(scripts / "rehearsal-series.mjs"), str(p), str(trial)],
            check=True,
        )
        result = json.loads(trial.read_bytes())
    game = result["matches"][0]["game"]
    preload = sum(
        v.get("preload", {}).get("quantity", 0)
        for v in match["roster"]
        if v.get("preload", {}).get("station") == "red-chute-low"
    )
    assert preload == 2
    initial = [f"red-chute-low/{n}" for n in range(preload + 1, stock + 1)]
    events = []
    for row in game["events"]:
        if row["phase"] != "TELEOP" or row["robot"] != "R1":
            continue
        for piece in row["pieces"]:
            if row["outcome"] == "acquired":
                assert row["station"] == "red-chute-low"
                events.append(dict(timeS=row["timeS"], kind="acquire", piece=piece))
            elif row["outcome"] == "credited":
                assert row["station"] == "red-l1"
                events.append(
                    dict(
                        timeS=row["timeS"],
                        kind="accept",
                        piece=piece,
                        witness="Prescribed L1 attempt accepted at the red trough by the field engine.",
                    )
                )
            elif row["operation"] == "attempt":
                events.append(
                    dict(timeS=row["timeS"], kind="miss", piece=piece, release=True)
                )
    robot = next(
        p for p in result["matches"][0]["periods"][-1]["robots"] if p["id"] == "R1"
    )
    return dict(
        runId=name,
        revision="Published Qual 2 roster and behavior logic",
        condition=f"Red 2 hold X {hold} inches, red low chute {stock} before its two preloads.",
        **CRITERION,
        variation=dict(holdXIn=hold, chuteInitialStock=stock),
        evidence="field-rehearsal",
        windowS=[0, 135],
        initialPieces=initial,
        events=events,
        score=game["score"],
        waitSeconds=robot["waitSeconds"],
        blocked=robot["blocked"],
        profileSha256=result["profileSha256"],
    )


def refine_route(scripts, issued):
    """Use the other alliance CORAL STATION without moving peers or changing stock."""
    profile = copy.deepcopy(issued)
    field = profile["rehearsal"]
    initial = field["programs"]["station"]["steps"]
    opposite = copy.deepcopy(initial[0])
    opposite.update(
        at=next(s["at"] for s in field["stations"] if s["id"] == "blue-chute-high"),
        station={"red": "red-chute-high", "blue": "blue-chute-high"},
        label="Acquire at the other CORAL STATION",
    )
    study = json.loads((HERE / "alliance-study.json").read_bytes())
    study["roles"] = [study["roles"][0]]
    for role, pickup, waypoints in (
        ("outside-rail", initial[0], [[223, 36]]),
        ("inside-control", initial[0], [[160, 96]]),
        ("upper-chute", opposite, []),
        ("behind-reef", opposite, [[152, 250]]),
    ):
        legs = [{"at": at, "label": "Stage clear of the crossing"} for at in waypoints]
        field["programs"][role] = {
            "label": role,
            "loop": True,
            "steps": [pickup, *legs, initial[1], *reversed(legs)],
        }
        study["roles"].append(
            {
                "id": role,
                "basis": "Change only the focal TELEOP supply or approach. All stations, other five players, AUTO start and total match time keep their issued values.",
                "plans": {"AUTO": "approach-chute", "TELEOP": role},
                "endgame": None,
            }
        )
    limited = []
    with tempfile.TemporaryDirectory() as d:
        file = Path(d) / "profile.json"
        study_file = HERE / "refinement-study.json"
        write(study_file, study)
        write(file, profile)
        write(HERE / "refinement-profile.json", profile)
        command = [
            "node",
            str(scripts / "strategy-lab.mjs"),
            str(file),
            str(study_file),
        ]
        result = json.loads(subprocess.check_output(command))
        subprocess.run(
            [*command, "--html", str(Path(d) / "role-lab.html")],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        (HERE / "role-lab.html").write_bytes((Path(d) / "role-lab.html").read_bytes())
        supply = next(s for s in field["stations"] if s["id"] == "red-chute-high")
        for available in (1, 0):
            # Qual 3 and BARGE 3 each preload one CORAL from this feeder.
            for c in study["contexts"]:
                roster = next(
                    m["roster"]
                    for m in field["series"]["matches"]
                    if m["id"] == c["match"]
                )
                assert (
                    sum(
                        r.get("preload", {}).get("quantity", 0)
                        for r in roster
                        if r.get("preload", {}).get("station") == supply["id"]
                    )
                    == 1
                )
            supply["stock"] = available + 1
            write(file, profile)
            limited_run = json.loads(subprocess.check_output(command))
            for cell in limited_run["cells"]:
                if cell["role"] == "upper-chute":
                    robot = next(
                        v
                        for v in cell["byRobot"]
                        if v["id"] == "R1" and v["phase"] == "TELEOP"
                    )
                    limited.append(
                        dict(
                            availableAfterPreload=available,
                            context=cell["context"],
                            focal=robot,
                            score=cell["score"],
                            profileSha256=limited_run["profileSha256"],
                            events=[
                                e
                                for e in cell["events"]
                                if e["robot"] == "R1" and e["phase"] == "TELEOP"
                            ],
                        )
                    )
    write(HERE / "refinement-result.json", result)
    write(HERE / "refinement-stock.json", limited)


def load_model(robot):
    mass = robot["currentFrontHoist"]["operating-split"]["operatingMassKg"]
    full = robot["currentFrontHoist"]["full-extra-reserve"]["operatingMassKg"]
    rate = 8451.6210685
    path = [
        {
            "id": "neo550-drive",
            "kind": "actuator",
            "advantage": 1,
            "efficiency": 1,
            "modes": ["raise"],
            "ratings": {},
        },
        {
            "id": "90234-bronze-nut",
            "kind": "transmission",
            "advantage": 1,
            "efficiency": 1,
            "ratings": {
                mode: {
                    "unit": "N",
                    "value": rate,
                    "basis": "Roton 90234 working axial load, 1900 lbf. The nut is one article in the series load path.",
                }
                for mode in ("raise", "hold", "power-loss")
            },
        },
        {
            "id": "screw-shaft-and-thrust-stack",
            "kind": "member",
            "advantage": 1,
            "efficiency": 1,
            "ratings": {},
        },
        {
            "id": "stationary-mast-saddle-through-chassis",
            "kind": "anchor",
            "advantage": 1,
            "efficiency": 1,
            "ratings": {},
        },
        {
            "id": "spring-pawl-and-casing-stop",
            "kind": "lock",
            "advantage": 1,
            "efficiency": 1,
            "modes": ["hold", "power-loss"],
            "ratings": {},
        },
    ]
    cases = []
    for name, mode, which, cg, acc, missing in [
        ("full-travel-upward", "raise", mass, 0.040, 4.903325, None),
        ("opposed-cg", "raise", mass, -0.040, 4.903325, None),
        ("extra-reserve", "raise", full, 0.040, 4.903325, None),
        ("disabled-at-cage", "power-loss", mass, 0.040, 0, None),
        ("stationary-bottom-catch", "hold", mass, -0.040, 0, None),
        ("one-side-missing", "power-loss", mass, 0.040, 0, "negative-Y"),
    ]:
        c = dict(
            id=name,
            mode=mode,
            massKg=which,
            cgXM=cg,
            accelZMps2=acc,
            pitchMomentNm=0,
            basis="Registered transverse Y-axis pair at ±321 mm. Charge 0.5g additional during raising and ±40 mm assumed transverse CG. Full extension; independent longitudinal clamp reactions and pawl capacity in robot-cases.json.",
            asymmetric=True,
            worstExtension=True,
        )
        if missing:
            c["unavailableSupport"] = missing
        cases.append(c)
    return dict(
        geometry="two-simple-supports",
        massEvidence="conservative-upper-bound",
        massBasis="Current full inspection receiving cap plus battery and 52 detachable bumper rows. The rack must be empty to enter the climb.",
        factor=1,
        factorBasis="Explicit upward acceleration already charges 1.5 times gravity on the raising cases. Unspecified strength ratings remain open.",
        supports=[
            dict(id=n, xM=y, path=path)
            for n, y in [("negative-Y", -0.321), ("positive-Y", 0.321)]
        ],
        cases=cases,
    )


def compile_studies(scripts):
    p = ROOT / "kickoff/season-profile.json"
    source = json.loads(p.read_bytes())
    refine_route(scripts, source)
    demand = dict(
        **CRITERION,
        requiredPieces=2,
        windowS=135,
        basis="Chosen role needs two TELEOP L1 deliveries before time expires, in addition to its independent AUTO leave and final park.",
    )
    episodes = [
        field_episode(scripts, source, n, x, s)
        for n, x, s in [
            ("clear-ally", 290, 30),
            ("blocked-ally", 264, 30),
            ("one-pipe-left", 290, 3),
            ("empty-after-preloads", 290, 2),
        ]
    ]
    robot = report()
    write(HERE / "robot-cases.json", robot)
    hard = dict(
        profileSha256=hashlib.sha256(p.read_bytes()).hexdigest(),
        demand=demand,
        episodes=episodes,
        inventory=["front-twin-screw-shallow-jack"],
        elevation=load_model(robot),
        scope="Only the front jack lifts the robot. The rack and grip carry CORAL; shoulder/shallow cage load paths and full shutdown are independent requirements.",
    )
    write(HERE / "hardening-inputs.json", hard)
    write(
        HERE / "throughput-study.json",
        dict(
            schema="agent-frc/throughput-study@1",
            demand=ref("hardening-inputs.json", "/demand"),
            episodes=[
                dict(
                    id=e["runId"], source=ref("hardening-inputs.json", f"/episodes/{i}")
                )
                for i, e in enumerate(episodes)
            ],
        ),
    )
    write(
        HERE / "robot-throughput-study.json",
        dict(
            schema="agent-frc/throughput-study@1",
            demand=ref("hardening-inputs.json", "/demand"),
            episodes=[
                dict(id="corrected-positive-start"),
                dict(id="corrected-negative-start"),
            ],
        ),
    )
    write(
        HERE / "elevation-study.json",
        dict(
            schema="agent-frc/elevation-study@1",
            inventory=ref("hardening-inputs.json", "/inventory"),
            mechanisms=[
                dict(
                    id=hard["inventory"][0],
                    source=ref("hardening-inputs.json", "/elevation"),
                )
            ],
        ),
    )
    cases = [
        dict(
            id=s + "-" + c,
            condition=f"{s} cage, "
            + (
                "candidate intended approach followed by suspended travel"
                if c == "design-entry"
                else "opposite lateral approach with suspended travel"
            ),
        )
        for s in ("red", "blue")
        for c in ("design-entry", "opposite-side-entry")
    ]
    keys = [
        "cagePoseObstacles",
        "suspendedTravelObstacles",
        "internalSweepInterferences",
    ]
    designs = []
    for layout, location in [("front", "front face"), ("rear", "rear face")]:
        runs = []
        for c in cases:
            sr = ref("robot-cases.json", "")
            del sr["pointer"]
            runs.append(
                dict(
                    case=c["id"],
                    source=sr,
                    execution="/layoutCases/" + layout + "/" + c["id"],
                    values={
                        k: "/layoutCases/" + layout + "/" + c["id"] + "/" + k
                        for k in keys
                    },
                )
            )
        designs.append(
            dict(
                id=layout + "-jack-swerve",
                architecture="Twin cage jack at the "
                + location
                + " of a four-module, station-fed receiver robot. The rear candidate crosses the rear-left controller tray. The front arrangement uses a different approach and gearbox location.",
                interfaces=[
                    {
                        "from": "swerve chassis",
                        "to": "shallow cage",
                        "exchange": "Approach clearance at the registered cage",
                    },
                    {
                        "from": location + " lift columns",
                        "to": "receiver and electronics",
                        "exchange": "Room for the installed hoist to traverse its complete stroke",
                    },
                ],
                runs=runs,
            )
        )
    write(
        HERE / "design-study.json",
        dict(
            schema="agent-frc/design-review@1",
            objectives=[
                dict(id=k, unit="interfering bodies or pairs", max=0) for k in keys
            ],
            cases=cases,
            concepts=designs,
        ),
    )
    commands = [
        ("strategy-lab", ["strategy-lab.mjs", p, HERE / "alliance-study.json"]),
        ("design-review", ["design-review.mjs", HERE / "design-study.json", ROOT]),
    ]
    for name, mode in [
        ("elevation", "elevation"),
        ("spatial", "elevation"),
        ("throughput", "throughput"),
        ("robot-throughput", "throughput"),
    ]:
        commands.append(
            (name, ["hardening.mjs", mode, HERE / (name + "-study.json"), ROOT])
        )
    spatial_study(json.loads((HERE / "hardening-inputs.json").read_bytes()))
    for name, args in commands:
        data = subprocess.check_output(
            ["node", str(scripts / args[0]), *map(str, args[1:])]
        )
        write(HERE / (name + "-result.json"), json.loads(data))
    compare_custody(scripts)
    write(HERE / "role-publication.json", {
        "htmlSha256": ref("role-lab.html", "")["sha256"],
        "demandSha256": ref("role-demand.json", "")["sha256"],
        "profileSha256": ref("refinement-profile.json", "")["sha256"],
        "studySha256": ref("refinement-study.json", "")["sha256"],
    })


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("plugin_scripts", type=Path)
    compile_studies(p.parse_args().plugin_scripts.resolve())
