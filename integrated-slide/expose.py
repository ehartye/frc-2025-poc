"""Recompute the two stereo policies from the combined native sensor backgrounds."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def run(output, trajectories, group):
    jobs=[]
    def task(folder,piece,head,*options):
        jobs.append([sys.executable,str(ROOT/"selected-optics/exposures_check.py"),
                     str(folder),str(trajectories),str(folder),piece,head,*options])
    marked=output/"marked"
    if group=="central":
        for piece in ("coral-A","coral-B"):
            task(marked,piece,"moving","--occlusion")
            task(marked,piece,"fixed","--critical")
            task(marked,piece,"seat")
        for control in ("lost-eye","swapped-eyes","stale-roof"):
            task(marked,"coral-B","moving","--control",control,"--critical")
        for position in ("front","rear"):
            task(marked,"coral-B","moving","--origin",position,"--critical")
    else:
        for position in ("front","rear"):
            folder=output/("fixed-"+position)
            for piece in ("coral-A","coral-B"):
                for head in ("fixed","seat"):
                    task(folder,piece,head,"--critical")
    with ThreadPoolExecutor(max_workers=2) as pool:
        for result in pool.map(subprocess.call,jobs):
            if result:
                raise SystemExit(result)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("output",type=Path)
    p.add_argument("trajectories",type=Path)
    p.add_argument("group",choices=["central","depths"])
    a=p.parse_args()
    run(a.output,a.trajectories,a.group)
