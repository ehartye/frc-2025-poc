"""Re-run the registered elastic shelf alternatives at the powered, attained opening pose."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import subprocess

HERE=Path(__file__).resolve().parent

def run(structure,jdk,assets):
    poses={side:r["sourceRobotMmDeg"] for side,r in json.loads((HERE/"opening.json").read_bytes()).items()}
    cache=assets/"conditioned-match/coupled-corrected";cache.mkdir(parents=True,exist_ok=True)
    pose_file=cache/"opening-poses.json"
    pose_file.write_text(json.dumps(poses,indent=2)+"\n",encoding="utf8")
    pins={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in structure.glob("*.py")}
    rows=[]
    for group in ("main","installed"):
        dest=cache/(group+".json")
        subprocess.run([sys.executable,str(structure/"coupled.py"),str(jdk),str(cache),str(dest),
                        "--group",group,"--workers","2","--poses",str(pose_file)],check=True)
        rows.extend(json.loads(dest.read_bytes()))
    out={"modelSourceSha256":pins,"openingPoseByCase":poses,"cases":rows}
    (HERE/"elastic.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("structure","jdk","assets"):p.add_argument(key,type=Path)
    a=p.parse_args();run(a.structure,a.jdk,a.assets)
