"""Publish a standalone view of the two actual fastener-detector streams."""
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent

if __name__=="__main__":
    rows=json.loads((HERE/"field-trials.json").read_bytes())
    payload=[{k:r[k] for k in ("wheelCase","mode","accepted","reason","terminalCheckPixelError","latestQualifiedInterval","studentFrames")}
             for r in rows if r.get("studentFrames")]
    html=(HERE/"viewer.template.html").read_text(encoding="utf8").replace("SENSOR_SAMPLES",json.dumps(payload,separators=(",",":")))
    (HERE/"viewer.html").write_text(html,encoding="utf8",newline="\n")
