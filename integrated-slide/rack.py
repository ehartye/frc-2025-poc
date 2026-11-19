"""Run the released calibrated scheduler and the selected physical mass on the new image register."""
import argparse
from pathlib import Path
import sys
import json

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"rack-sensing"))
import subprocess


def run(args):
    mass=HERE/"mass.json"
    if args.stage=="scheduler":
        import rack_trial
        inputs=args.study/"observer-hal/registered-inputs"
        selected=json.loads(mass.read_bytes())["moving"]
        rack_trial.run(inputs/"raw",args.output,args.jdk,args.gradle,inputs/"receipts",
                       args.study/"controller-register.json",selected["extraToIssuedPlantKg"])
    else:
        subprocess.run([sys.executable,str(ROOT/"rack-sensing/qualify.py"),str(args.trajectories),
                        str(args.output),str(args.jdk),"--mass-source",str(mass)],check=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("stage",choices=["scheduler","contacts"])
    for name in ("study","trajectories","output","jdk","gradle"):
        p.add_argument(name,type=Path)
    a=p.parse_args()
    run(a)
