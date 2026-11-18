"""Build the offline student docking lab from the measured cases in this directory."""
import json
import hashlib
from pathlib import Path
from transfer import ST

HERE = Path(__file__).resolve().parent


def build():
    dock = json.loads((HERE/"dock-tolerance.json").read_bytes())
    comp = json.loads((HERE/"compensated-dock.json").read_bytes())
    entry = json.loads((HERE/"entry-window.json").read_bytes())
    for name,field in (("compensated-dock.json","compensated_pose"),("receiver-stops.step","reopened_stop_step")):
        if hashlib.sha256((HERE/name).read_bytes()).hexdigest() != entry[field]:
            raise ValueError("Changed receiver-entry input: "+name)
    source = json.loads((HERE/"transfer.json").read_bytes())
    if comp["reference_step_sha256"]!=dock["receive_step_sha256"] or comp["transfer_sha256"]!=dock["source_transfer_sha256"]:
        raise ValueError("The two studies use different robot or feeder inputs")
    velocity = source["feed"]["lip_mps"]
    state,flight,_ = ST.path_to_shelf(velocity)
    rows = []
    for off,stroke,label in ((10,240,"Issued approach"),(40,240,"Move chassis back"),(40,270,"Move chassis and extend shelf")):
        if stroke==270:
            robust = comp["structural_error_box"]
            nominal = comp["finite_pipe_samples"][0]
        else:
            robust = dock["continuous_dock_bounds"][str(off)]
            nominal = next(r for r in dock["finite_pipe_flight"] if r["station_standoff_mm"]==off and r["yaw_deg"]==0)
        hit = nominal["first_positive_material_sample"]
        rows.append({"label":label,"standoff":off,"stroke":stroke,"bound":robust["gap_lower_bound_mm"],
                     "pads":hit is not None and all(k.startswith("observer-moving-pad-") for k in hit["overlaps_mm3"]),
                     "firstMs":None if hit is None else hit["seconds_from_lip"]*1000})
    limits = [{"normal":r["normal_mm"],"lateral":r["lateral_mm"],"yaw":r["yaw_deg"],
               "gap":r["hardware_lower_mm"]} for r in entry["revised"] if r["hardware_lower_mm"]>0
              and not r["unresolved_material"] and r["max_belt_excursion_mm"]<=entry["outward_travel_nominal_mm"]-entry["stop_position_allowance_mm"]]
    payload = {"cases":rows,"entryLimits":limits,"flight":[[round(v*1000,3) for v in ST.flight_center(state,flight*i/40)] for i in range(41)],
               "radius":ST.RADIUS_M*1000,"speed":velocity,"shelfTop":ST.SHELF_TOP*1000,
               "lipX":ST.LIP_X*1000,"lipZ":ST.LIP_Z*1000,
               "rigidGap":min(row["hardware_gap_mm"] for row in comp["free_flight"]["nominal_chords"]),
               "shelfLength":ST.SHELF_LENGTH*1000,"frontAtIssued":(ST.EXTENDED_FRONT-.010)*1000}
    template=(HERE/"dock-lab.template.html").read_text(encoding="utf8")
    (HERE/"dock-lab.html").write_text(template.replace("__DATA__",json.dumps(payload,separators=(",",":"))),
                                      encoding="utf8",newline="\n")


if __name__=="__main__":
    build()
