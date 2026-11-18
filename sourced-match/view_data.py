"""Write the small, offline browser dataset from the issued numerical receipts."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def dataset():
    source = json.loads((HERE / "shop-545.json").read_bytes())
    spin = json.loads((HERE / "spin-screen.json").read_bytes())
    pipes = []
    for case in source["case_studies"]:
        state = case["state"]
        events = case["events_teleop_s"]
        pipes.append({
            "id": state["piece_id"],
            "clock": [
                ["Human releases", case["station_source"]["human_release_teleop_s"]],
                ["Pipe leaves ramp", state["lip_teleop_s"]],
                ["Seat attained", state["seat_teleop_s"]],
                ["Chassis in transit", events["drive-and-turn"]],
                ["Placement starts", events["place"]],
                ["First target contact", events["paired-trough-first-contact"]],
                ["Sustained support", case["zero_spin"]["support_teleop_s"]],
            ],
            "spin": {speed: row["outcome"] for speed, row in spin["runs"][state["piece_id"]].items()},
        })
    return {"pipes": pipes, "scenarios": source["scenarios"],
            "rebounds": [[row["assumed_normal_restitution"], row["first_material_witness"]["elapsed_after_impact_s"]]
                         for row in source["rejected_upstream_bounce"]]}


def render():
    return "window.sourceMatch = " + json.dumps(dataset(), indent=2, allow_nan=False) + ";\n"


if __name__ == "__main__":
    (HERE / "timeline-data.js").write_text(render(), encoding="utf-8", newline="\n")
