"""Build a portable view of the identified pipe, both optical ports and electronic gate."""
import json
import math
from pathlib import Path
import sys
import numpy as np
HERE=Path(__file__).resolve().parent


def run(cache):
    records={}
    for prefix,file in (("issued","feed.json"),("probe","probe-feed.json")):
        observations=json.loads((HERE/"custody.json").read_bytes())[prefix]["run"]
        for row in json.loads((HERE/file).read_bytes()):
            name=row["name"]
            if not name.endswith("-250") or name=="center-250":continue
            tr=np.load(cache/(name+".npy"));frames=[]
            for ms in np.arange(tr[0,0],tr[-1,0]+.01,5):
                p=[round(float(np.interp(ms,tr[:,0],v)),3) for v in tr[:,1:4].T]
                tick=next((q for q in reversed(row["portTicks"]) if q[0]<=ms),None)
                frames.append([round(ms),*p,tick[1],int(tick[2]),int(tick[3])])
            gate=next(r for r in observations if r["case"]==name and r["challenge"]=="nominal")
            records[name]={"frames":frames,"hard":row["hardStop"],"band":row["firstPortBandMs"],"gate":gate,"delta":row["actualReturn"]}
    (HERE/"replay.html").write_text((HERE/"template.html").read_text(encoding="utf8").replace("/*INPUT*/",json.dumps(records,separators=(",",":"))),encoding="utf8",newline="\n")


if __name__=="__main__":run(Path(sys.argv[1]))
