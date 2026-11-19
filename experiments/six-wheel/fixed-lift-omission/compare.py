"""Compare the base SP1 installation with an independently selected skin and harness."""
import argparse
from pathlib import Path
from account import HERE,read,write,digest,run


def compare(plugin,issued,combined,out):
    result={}
    for key,source in (("issued",issued),("jointHarnessAndLightSkin",combined)):
        data=read(source)
        if key=="jointHarnessAndLightSkin" and not {"fixed-lead-install","repacked-light"}<=set(data["selected"]):
            raise ValueError("Missing fixed lead or repacked cover")
        folder=out/key
        run(plugin,source,folder)
        result[key]={**read(folder/"summary.json"),"inputSourceHashes":{s["id"]:s["sha256"] for s in data["sources"]},
                     "selectedInputMapSha256":digest(source)}
    write(HERE/"comparison.json",result)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("plugin","issued","combined","out"):
        p.add_argument(key,type=lambda s:Path(s).resolve())
    a=p.parse_args()
    compare(a.plugin,a.issued,a.combined,a.out)
