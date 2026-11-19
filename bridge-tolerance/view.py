"""Build the standalone student tolerance explorer from the two native component inventories."""
from pathlib import Path
import json
import math
from design import HERE,ROOT
from replay_candidate import mod
field_bounds=mod("bridge_clearance",HERE/"field.py")

def build():
    field=json.loads((HERE/"field.json").read_bytes())
    cases={}
    for name,folder in (("Issued",ROOT/"moving-observer"),("Aft",HERE)):
        bodies=json.loads((folder/"mount.json").read_bytes())["parts"]
        cases[name]={}
        for key,part in bodies.items():
            b=part["bounds_mm"]
            cases[name][key]=field_bounds.heading_sweep([b[0]-field_bounds.STROKE,b[1],b[2],b[3]+370+field_bounds.STROKE,b[4],b[5]],field_bounds.ANGLE)
    ball=json.loads((ROOT/"full-match/installed-stock.json").read_bytes())["stock"]["NAUO201"]["native_center_mm"]
    data={"bounds":cases,"radius":field_bounds.BALL_RADIUS,"reserve":field_bounds.MOUNT+field["ideal_flex_2g_mm"],
          "ball":[ball[0]-(3453.050980729168-490),-ball[2],ball[1]]}
    template=(HERE/"viewer.template").read_text(encoding="utf8")
    (HERE/"explore.html").write_text(template.replace("/*registered-data*/",json.dumps(data,separators=(",",":"))),
                                  encoding="utf8",newline="\n")
if __name__=="__main__":build()
