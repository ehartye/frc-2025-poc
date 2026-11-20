"""Build the two live REEFSCAPE labs from the immutable field and role studies."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
KICKOFF = ROOT / "kickoff"
ROLE = ROOT / "experiments/swerve-drive/whole-robot-trade"


def read(path):
    return json.loads(path.read_bytes())


def write(path, value):
    path.write_bytes((json.dumps(value, indent=2, ensure_ascii=False) + "\n").encode())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def live_profile(source):
    profile = read(source)
    game = profile["rehearsal"]
    stations = {s["id"]: s for s in game["stations"]}
    for s in game["stations"]:
        if "-floor-" in s["id"]:
            pipe = s["piece"] == "CORAL"
            s["floor"] = dict(radiusIn=2.25 if pipe else 8.125, positions=[s["displayAt"]],
                              heightIn=11.875 if pipe else 16.25, captureReachIn=9 if pipe else 14)
            if not pipe:
                # Leave bumper clearance for the ball at its projected floor mark.
                s["at"][0] = s["displayAt"][0] + (32 if s["side"] == "blue" else -32)
        if "-chute-" in s["id"]:
            # Stage the pipe 23 inches toward the end wall, clear of the arrival footprint.
            outlet = [s["at"][0] + (-23 if s["side"] == "blue" else 23), s["at"][1]]
            s["service"] = dict(operator=s["id"] + "-attendant", secondsPerPiece=3,
                                capacity=1, phases=["TELEOP"], outlet=outlet,
                                radiusIn=2.25, heightIn=4.5, captureReachIn=9)
    for program in game["programs"].values():
        for steps in [program.get("steps", []), *program.get("byPeriod", {}).values()]:
            for step in steps:
                station = step.get("station")
                if isinstance(station, dict) and "-floor-algae-" in station["blue"]:
                    step["at"] = list(stations[station["blue"]]["at"])
    for side, other in (("blue", "red"), ("red", "blue")):
        source = stations[other + "-processor"]
        key = side + "-processor-exit"
        source["returns"] = {"credited": dict(station=key, delayS=1)}
        game["stations"].append(dict(
            id=key, label=side.title() + " human player at the processor exit",
            piece="ALGAE", side=side, kind="retrieval", stock=0, radiusIn=6,
            at=source["at"], displayAt=source["displayAt"],
            service=dict(operator=side + "-processor-attendant", secondsPerPiece=5,
                         phases=["TELEOP"], outlet=source["displayAt"], radiusIn=8.125,
                         capacity=1, delivery=dict(station=side + "-net", outcome="unresolved",
                         basis="The human player attempts the net. Throw accuracy is unmeasured."))))
    game["sources"].append(
        "Live rehearsal assumptions: one operator per CORAL STATION and one at each processor exit. "
        "Each chute stages one pipe in three TELEOP seconds. Opposing processor ALGAE transfer in "
        "one second and require five seconds per unmeasured human throw. Floor stock uses circular "
        "top-down envelopes. The starting ALGAE and upright CORAL share a floor mark; vertical "
        "stacking, reef extraction, chute flight and net trajectories are outside this motion model.")
    return profile


def build(plugin):
    with tempfile.TemporaryDirectory() as temp:
        out = Path(temp)
        field = KICKOFF / "live-profile.json"
        role = ROLE / "live-profile.json"
        write(field, live_profile(KICKOFF / "season-profile.json"))
        write(role, live_profile(ROLE / "refinement-profile.json"))
        def run(script, *args):
            return subprocess.check_output(["node", str(plugin / "scripts" / script), *args], cwd=ROOT)
        run("kickoff.mjs", "--profile", field, "--output", out / "field.html")
        series_file = out / "series.json"
        run("rehearsal-series.mjs", field, series_file)
        scores = read(series_file)["matches"]
        run("strategy-lab.mjs", role, ROLE / "refinement-study.json", "--html", out / "role.html")
        for html in (out / "field.html", out / "role.html"):
            html.write_bytes(html.read_bytes().replace(b"\r\n", b"\n"))
        for dest in (KICKOFF / "field-lab.html", ROOT / "experiments/six-wheel/kickoff/field-lab.html"):
            shutil.copyfile(out / "field.html", dest)
        shutil.copyfile(out / "role.html", ROLE / "role-lab.html")
        manifest = read(ROOT / "highlights.json")
        manifest["fieldLab"]["sha256"] = digest(out / "field.html")
        write(ROOT / "highlights.json", manifest)
        write(KICKOFF / "live-publication.json", dict(
            fieldHtmlSha256=digest(out / "field.html"), roleHtmlSha256=digest(out / "role.html"),
            fieldProfileSha256=digest(field), roleProfileSha256=digest(role),
            roleStudySha256=digest(ROLE / "refinement-study.json"), seriesSha256=digest(series_file),
            matches=[dict(id=m["id"], score=m["game"]["score"]) for m in scores]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin", type=Path, help="Checkout of the live rehearsal engine")
    build(parser.parse_args().plugin.resolve())
