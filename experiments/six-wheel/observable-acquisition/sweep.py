"""Sweep the entire centered, shelf-supported search travel through the issued receiving assembly."""
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT/"station-supply"))
from transfer import cad, corridor, hits


def run():
    poses = ROOT/"design/competition-package/shop-duplex/red"
    native = json.loads((HERE/"ports.json").read_bytes())
    step = poses/"receive.step"
    if hashlib.sha256(step.read_bytes()).hexdigest() != native["receive_sha256"]:
        raise ValueError("receiver changed")
    frames = json.loads((poses/"part-frames.json").read_bytes())
    parts = cad.read_named(step, [{"id": key} for key in frames])
    boxes = {k: cad.bounds(v) for k,v in parts.items()}
    study = json.loads((HERE/"results.json").read_bytes())
    accepted = [row for name,levels in study["cases"].items() for row in levels.values()
                if study["conditions"][name]["counterfeed"] and row["outcome"] == "IN_BAND"]
    extrema = [min(row["minimum_u_mm"] for row in accepted), max(row["first_contact_ms_mm"][1] for row in accepted)]
    if extrema[0] < -27 or extrema[1] > 110:
        raise ValueError("supported paths extend outside native sweep")
    # Distances in robot mm: 240 mm slide, 880 mm shelf, 57.15 mm pipe radius.
    overlaps = corridor(parts, boxes, (240-35,937.15), (240+110,937.15))
    candidates = {k: parts[k] for k in overlaps}
    candidate_boxes = {k: boxes[k] for k in overlaps}
    selected = corridor(parts, boxes, (240-27,937.15), (240+110,937.15))
    probes = {str(round(u,5)): hits(candidates,candidate_boxes,240+u,937.15) for u in (
              -35, study["cases"]["stationary--5"]["0.0005"]["minimum_u_mm"], -32, -31, -30, -27, -20, 0, 75, 110)}
    return {"pose_sha256": native["receive_sha256"], "bodies": len(parts),
            "selected_enclosure_mm": [-27,110], "selected_continuous_overlap_mm3": selected,
            "annular_material_at_selected_u_mm": probes,
            "accepted_min_and_max_touchdown_u_mm": extrema, "stationary_brake_control_enclosure_mm": [-35,110],
            "continuous_outer_enclosure_over_0_05_mm3": overlaps,
            "results_sha256": hashlib.sha256((HERE/"results.json").read_bytes()).hexdigest()}


if __name__ == "__main__":
    (HERE/"sweep.json").write_text(json.dumps(run(),indent=2)+"\n", encoding="utf-8", newline="\n")
