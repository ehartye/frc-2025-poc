"""Check alternate stage-B counterfeed settings on the changed mechanical receiver."""
import argparse
import json
import subprocess
import tempfile
from pathlib import Path
from feed import HERE,ROOT,trial


def run(cache,jdk):
    base=ROOT/"observable-acquisition"
    centers=json.loads((HERE/"return.json").read_bytes())["cases"]
    summary={}
    for speed in (.45,.55,.60):
        with tempfile.TemporaryDirectory() as folder:
            tmp=Path(folder)
            text=(base/"PortAcquisition.java").read_text().replace("speed = 0.45;",f"speed = {speed:.2f};")
            (tmp/"PortAcquisition.java").write_text(text,encoding="utf8",newline="\n")
            subprocess.run([str(jdk/"bin/javac"),"-d",str(tmp),str(tmp/"PortAcquisition.java"),str(base/"PortConsole.java")],check=True)
            with subprocess.Popen([str(jdk/"bin/java"),"-cp",str(tmp),"org.reefscape.controls.PortConsole"],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) as proc:
                for mode,row in centers.items():
                    key=f"{speed:.2f}-{mode}"
                    r=trial(proc,row["stationRobotDxDyYawMmDeg"],key,.0005,19,cache,relief=True)
                    summary[key]={"firstPortBandMs":r["firstPortBandMs"],"minCenterMm":r["minimumUMm"],"reason":r["reason"],"hardStop":r["hardStop"],"sourceBeltPeakTravelMm":r["sourceBeltPeakTravelMm"],"trace":r["trajectorySHA256"]}
                    print(key,summary[key],flush=True)
                proc.stdin.close();proc.wait()
    (HERE/"tuning.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);p.add_argument("jdk",type=Path)
    a=p.parse_args();run(a.cache,a.jdk)
