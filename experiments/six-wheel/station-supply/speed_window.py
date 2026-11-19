"""Find the sampled upper speed boundary of the idealized dry, centered seat check."""
import json
from pathlib import Path
from transfer import seat


def screen():
    result=[]
    for dt in (.002,.001,.0005):
        cases=[(i/100,seat(i/100,dt)) for i in range(160,251)]
        passed=[(speed,data) for speed,data in cases if data["result"]=="within-modeled-seat-window"]
        failed=[(speed,data) for speed,data in cases if data["result"]=="recover"]
        result.append({"integration_dt_s":dt,"trials":len(cases),"passed":len(passed),"failed":len(failed),
                       "greatest_passing_speed_mps":passed[-1][0],"lowest_failed_speed_mps":failed[0][0],
                       "minimum_x_at_maximum_pass_m":passed[-1][1]["minimum_local_x_m"],
                       "minimum_x_at_first_failure_m":failed[0][1]["minimum_local_x_m"]})
    return {"basis":"Centered no-rebound shelf impact, nominal dry grip and nominal flat geometry. Speed grid only.",
            "range_mps":[1.6,2.5],"step_mps":.01,"runs":result}


if __name__=="__main__":
    p=Path(__file__).with_name("speed-window.json")
    p.write_text(json.dumps(screen(),indent=2)+"\n",encoding="utf8",newline="\n")
