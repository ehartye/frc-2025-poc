"""Build a standalone student replay from the measured robot poses and docking decisions."""
import json
from pathlib import Path

D=Path(__file__).resolve().parent
data=json.loads((D/"results.json").read_bytes());clock=json.loads((D/"clock.json").read_bytes())
cases={}
for row,case in zip(data,clock["cases"],strict=True):
    cases[row["pipe"]+"/"+row["mode"]]={**row,**{k:case[k] for k in ("startWallS","extensionDeadlineWallS","extensionAgainstOccupiedBallMm")}}
out=(D/"view.template.html").read_text(encoding="utf8").replace("__DATA__",json.dumps(cases,separators=(",",":")))
(D/"replay.html").write_text(out,encoding="utf8",newline="\n")
