"""Drive the guide-contact candidate through the original Java intake ticks and the scheduled departure."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

from guide import HERE, ROOT, Guide, GuidedPipe, feed


def test_pose(job):
    name, dt, params, cls, jdk, cache, observed = job
    data = json.loads((ROOT/"station-landmark/feed.json").read_bytes())
    case = data["outcomes"][name]
    delta = case["actualRobotMmDeg"]
    if observed:
        route=observed["trials"]["nominal" if name=="shiftedHumanSource" else name]
        if route["status"]!="observed stationary" or route["wallS"] >= data["humanFeedWallS"] or min(route["fieldMinimumMm"].values())<=0:
            return {"case":name,"dtSeconds":dt,"lastState":"INHIBITED_BEFORE_FEED"}
        n,y,h=route["finalStationMmDeg"];delta=[-n,-y,h]
    original = feed.Pipe
    actual = []
    def plant(delta, feed_side, relief):
        if not relief:
            raise ValueError("Relieved jaw required")
        p = GuidedPipe(delta,feed_side,Guide(**params))
        actual.append(p)
        return p
    feed.Pipe = plant
    duration = round(1000*(data["throughScheduledDriveWallS"]-data["epochWallUs"]/1e6))
    try:
        with subprocess.Popen([str(jdk/"bin/java"),"-cp",str(cls),"org.reefscape.controls.PortConsole"],
                              stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) as proc:
            label = f"{name}-{round(dt*1e6)}-{params['sectors']}-{params.get('upper_mm',900)}"
            run = feed.trial(proc, delta,label,dt,0,cache,
                             feed_side=case["humanFeedDisplacementMm"],relief=True,until_ms=duration)
            proc.stdin.close()
            proc.wait()
    finally:
        feed.Pipe = original
    p, = actual
    return {"case":name, "attainedRobotMmDeg":delta, "dtSeconds":dt, "guide":params, "shaftPortEpochUs":data["epochWallUs"],
            "lastTickMs":run["portTicks"][-1][0], "stop":run["hardStop"], "loss":run["loss"],
            "firstInBandMs":run["firstPortBandMs"], "lastState":run["lastPortState"], "reason":run["reason"],
            "finalCenterMm":run["finalPositionMm"], "peakCartridgeTravelMm":run["sourceBeltPeakTravelMm"],
            "firstGuideSec":p.first_guide, "lastGuideSec":p.last_guide, "peakOneFaceForceN":p.peak_guiding_n,
            "peakLocalDeflectionMm":p.max_local_compression_mm, "trajectorySha256":run["trajectorySHA256"]}


if __name__ == "__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("jdk",type=Path)
    cli.add_argument("cache",type=Path)
    cli.add_argument("--short", action="store_true")
    cli.add_argument("--narrow",action="store_true")
    a=cli.parse_args()
    a.cache.mkdir(parents=True,exist_ok=True)
    cases=("leftSmall","rightSmall","nominal","badRegistration","shiftedHumanSource")
    observed=json.loads((HERE/"optics-narrow.json").read_bytes()) if a.narrow else None
    if a.narrow: cases=(*cases,"trackOnly","bothLarge","bothSmall","crossedDrift")
    with tempfile.TemporaryDirectory() as tmp:
        cls=Path(tmp);source=cls/"PortAcquisition.java"
        source.write_text(feed.port_program(True),encoding="utf8",newline="\n")
        expected=json.loads((ROOT/"station-landmark/feed.json").read_bytes())["programSha256"]
        if hashlib.sha256(source.read_bytes()).hexdigest()!=expected:raise ValueError("Different controller")
        subprocess.run([str(a.jdk/"bin/javac"),"-d",str(cls),str(source),str(ROOT/"observable-acquisition/PortConsole.java")],check=True)
        selected=({"stiffness_n_mm":5,"throat_mm":-.3,"damping_n_s_m":100,"upper_mm":850} if a.short else {"stiffness_n_mm":5,"throat_mm":.3})
        if a.narrow:selected={"stiffness_n_mm":10,"throat_mm":-.3,"damping_n_s_m":200,"upper_mm":850,"x_width_mm":40,"x_offset_mm":20}
        jobs=[(case,dt,{**selected,"sectors":sectors},cls,a.jdk,a.cache,observed)
              for case in cases for dt,sectors in ((.0005,32),(.00025,32),(.000125,32),(.00025,64))
              if (a.narrow and case in cases[:5]) or a.short or case in cases[:2] or (dt==.00025 and sectors==32)]
        with ProcessPoolExecutor(max_workers=5) as pool:rows=list(pool.map(test_pose,jobs))
    (HERE/("seat-narrow.json" if a.narrow else "seat-short.json" if a.short else "seat.json")).write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
    for r in rows:print(r["case"],r["dtSeconds"],r["lastState"],r.get("firstInBandMs"))
