"""Replay the combined SP1 installation, optical handoff, rack plant and shared supply."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def run(a):
    raw=a.artifacts/"relieved-match"
    issued=a.artifacts/"selected-optics/issued"
    previous=a.artifacts/"sp1-mount/optics-maps/sp1-install.json"
    if (a.output/"rack/results.json").exists():
        raise ValueError("Choose a fresh output directory for the scheduler evidence")
    def call(program,*args):
        env=os.environ.copy()
        cached=a.artifacts/"sector-alignment/packages"
        if cached.is_dir():
            env["PYTHONPATH"]=os.pathsep.join(filter(None,[str(cached),env.get("PYTHONPATH")]))
        subprocess.run([sys.executable,str(program),*map(str,args)],check=True,env=env)
    call(HERE/"inertia.py",a.plugin,previous,a.output/"maps")
    for label,offset in (("",0.),("fixed-front",.75),("fixed-rear",-.75)):
        call(HERE/"render.py","endpoints",a.artifacts,a.lamp,a.field,a.output/label,"--fixed-depth",offset)
    call(HERE/"render.py","sweep",a.artifacts,a.lamp,a.field,a.output)
    call(ROOT/"selected-optics/shelf_marks.py",a.output,a.output/"marked")
    for group in ("central","depths"):
        call(HERE/"expose.py",a.output,raw,group)
    call(HERE/"visibility.py",a.artifacts,a.lamp,a.output,issued)
    for stage in ("field","packets"):
        call(HERE/"observations.py",stage,a.output,raw,a.field,a.jdk,a.gradle)
    for stage in ("scheduler","contacts"):
        call(HERE/"rack.py",stage,a.output,raw,a.output/"rack",a.jdk,a.gradle)
    call(HERE/"receipt.py",a.output,issued)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("artifacts","lamp","field","output","jdk","gradle","plugin"):
        p.add_argument(name,type=lambda s:Path(s).resolve())
    run(p.parse_args())
