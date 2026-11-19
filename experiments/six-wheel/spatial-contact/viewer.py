"""Build a standalone student contact replay from the same rigid-body samples as the receipts."""
import csv
import json
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent


def build(trace):
    traces={}
    with trace.open() as f:
        for r in csv.DictReader(f):
            k=r["case"]
            if not k.endswith(("-scheduled","-held","-late")):continue
            traces.setdefault(k,[]).append([round(float(r[n])*s,4) for n,s in
                (("t_s",1),("x_m",1000),("y_m",1000),("z_m",1000),("axis_x",1),("axis_z",1),("front_gap_m",1000))])
    data={"traces":traces,"cases":json.loads((HERE/"physics.json").read_bytes())["cases"],
          "match":json.loads((HERE/"match.json").read_bytes())["cases"]}
    html=(HERE/"template.html").read_text(encoding="utf8").replace("/*DATA*/",json.dumps(data,separators=(",",":")))
    (HERE/"replay.html").write_text(html,encoding="utf8",newline="\n")


if __name__=="__main__":build(Path(sys.argv[1]))
