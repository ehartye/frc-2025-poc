"""Re-run the registered elastic shelf alternatives at the powered, attained opening pose."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import subprocess

HERE=Path(__file__).resolve().parent

def run(structure,jdk,assets,collect=False):
    poses={side:r["sourceRobotMmDeg"] for side,r in json.loads((HERE/"opening.json").read_bytes()).items()}
    cache=assets/"conditioned-match/coupled-corrected";cache.mkdir(parents=True,exist_ok=True)
    pose_file=cache/"opening-poses.json"
    pose_file.write_text(json.dumps(poses,indent=2)+"\n",encoding="utf8")
    names=("coupled.py","response.py","tray.py","installed.py","damper_fit.py")
    pins={name:hashlib.sha256((structure/name).read_bytes()).hexdigest() for name in names}
    rows=[]
    for group in ("main","installed","machined"):
        dest=cache/(group+".json")
        if not collect:
            subprocess.run([sys.executable,str(structure/"coupled.py"),str(jdk),str(cache),str(dest),
                        "--group",group,"--workers","2","--poses",str(pose_file)],check=True)
        observed=json.loads(dest.read_bytes())
        for row in observed:
            if row["sourceRobotMmDeg"]!=poses[row["case"]]:raise ValueError("Elastic result came from a different approach")
            raw=cache/(row["id"]+".npy")
            with raw.open("rb") as fh:
                if hashlib.file_digest(fh,"sha256").hexdigest()!=row["trajectorySha256"]:raise ValueError("Changed elastic trajectory")
        rows.extend(observed)
    out={"reproductionSourceSha256":pins,"groupHashes":{name:hashlib.sha256((cache/(name+".json")).read_bytes()).hexdigest() for name in ("main","installed","machined")},"openingPoseByCase":poses,"cases":rows}
    (HERE/"elastic.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("structure","jdk","assets"):p.add_argument(key,type=Path)
    p.add_argument("--collect",action="store_true",help="Use separately produced raw group receipts from this exact pose.")
    a=p.parse_args();run(a.structure,a.jdk,a.assets,a.collect)
