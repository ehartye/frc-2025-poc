"""Render a portable TELEOP replay directly from project programs and the screened matrix."""
import json

from moving_traffic import ROOT, SCENARIOS, f, programs


def track(program):
    rows=[]
    previous=None
    for i in range(676):
        t=i*.2
        pose,_,phase=program.at(t)
        row=[round(t,1),*[round(v,4) for v in pose],phase]
        if previous is None or previous[1:]!=row[1:]:
            if previous is not None and previous[0]>rows[-1][0]:
                rows.append(previous)
            rows.append(row)
        previous=row
    if previous[0]>rows[-1][0]:
        rows.append(previous)
    return rows


def render():
    receipts=json.loads((ROOT/"mission/moving-traffic.json").read_text())["scenarios"]
    data={"field":[f.PROFILE["field"][k]*f.IN for k in ("length","width")],"lip":f.LIP,
          "obstacles":f.obstacles(),"tracks":{},"cases":{}}
    for key,(name,hold,stock) in SCENARIOS.items():
        team,_,_=programs(hold,stock)
        ids={}
        for robot,program in team.items():
            tid=key+"-"+robot if robot=="red2" else robot
            if tid not in data["tracks"]:
                data["tracks"][tid]=track(program)
            ids[robot]=tid
        receipt=receipts[key]
        data["cases"][key]={"name":name,"tracks":ids,"stop":receipt["first_unresolved"],
                            "contact":receipt["first_sampled_overlap"],"stock":stock,
                            "gap":receipt["min_separating_bound_m"],"feeds":receipt["feeds"],
                            "pieces":receipt["support_observations"],"reset":receipt["planned_empty_finish_s"]}
    template=(ROOT/"mission/moving-field-template.html").read_text(encoding="utf-8")
    if template.count("REPLAY_SOURCE")!=1:
        raise ValueError("Missing replay source marker")
    (ROOT/"mission/moving-field.html").write_text(
        template.replace("REPLAY_SOURCE",json.dumps(data,separators=(",",":"),allow_nan=False)),
        encoding="utf-8",newline="\n")


if __name__=="__main__":
    render()
