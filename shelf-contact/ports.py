"""Feed the attained approach and finite compliant shelf into the existing desktop Java intake."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict,replace
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

from experiment import CompliantPipe,Shelf,GUIDE,source,ROOT,HERE,feed


def replay(job, *, attained_pose=None):
    name,cfg,dt,cls,jdk,cache,high=job
    delta,shift=source(name) if attained_pose is None else (attained_pose,0.)
    g=replace(GUIDE,sectors=128,radial_bins=4) if high else GUIDE
    old_law=feed.cfg
    if high:feed.cfg=replace(feed.cfg,slices=10)
    pipe=CompliantPipe(delta,shift,g,cfg)
    original=feed.Pipe
    feed.Pipe=lambda *_:pipe
    try:
        with subprocess.Popen([str(jdk/"bin/java"),"-cp",str(cls),"org.reefscape.controls.PortConsole"],
                              stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1) as proc:
            label=f"{name}-{cfg.stiffness_n_m:.0f}-{cfg.dissipation_s_m:g}-{dt*1.e6:g}"
            result=feed.trial(proc,delta,label,dt,0,cache,feed_side=shift,relief=True,until_ms=5460)
            proc.stdin.close();proc.wait()
    finally:
        feed.Pipe=original
        feed.cfg=old_law
    transitions=[]
    for tick,state,entry,seat,speed in result["portTicks"]:
        row=[state,entry,seat,speed]
        if not transitions or transitions[-1][1:]!=row:transitions.append([tick,*row])
    return {"case":name,"sourceRobotMmDeg":delta,"sourceShiftMm":shift,"shelf":asdict(cfg),"guideAngleSectors":g.sectors,"guideRadialBands":g.radial_bins,"beltStrips":10 if high else old_law.slices,"dtS":dt,
            "lastTickMs":result["portTicks"][-1][0],"firstBandMs":result["firstPortBandMs"],"state":result["lastPortState"],
            "fault":result["reason"],"stop":pipe.hard,"loss":pipe.loss,"peakCartridgeMm":pipe.peak,
            "peakShelfCompressionMm":1000*pipe.peak_shelf_compression,"impactSpeedMps":pipe.first_impact_speed,
            "peakShelfN":pipe.peak_shelf_n,"centerMm":(pipe.p*1000).tolist(),"axis":pipe.a.tolist(),
            "trajectorySha256":result["trajectorySHA256"],"portTransitions":transitions}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("jdk",type=Path);p.add_argument("cache",type=Path)
    p.add_argument("--fine",action="store_true")
    p.add_argument("--soft",action="store_true")
    a=p.parse_args();a.cache.mkdir(parents=True,exist_ok=True)
    docs=json.loads((ROOT/"station-landmark/feed.json").read_bytes())
    names=("nominal","leftSmall","rightSmall","shiftedHumanSource","badRegistration")
    with tempfile.TemporaryDirectory() as tmp:
        cls=Path(tmp);fp=cls/"PortAcquisition.java"
        fp.write_text(feed.port_program(True),encoding="utf8",newline="\n")
        if hashlib.sha256(fp.read_bytes()).hexdigest()!=docs["programSha256"]:raise ValueError("Different controller")
        subprocess.run([str(a.jdk/"bin/javac"),"-d",str(cls),str(fp),str(ROOT/"observable-acquisition/PortConsole.java")],check=True)
        jobs=[(n,Shelf(),dt,cls,a.jdk,a.cache) for n in names for dt in (.00025,.000125,.0000625)]
        jobs.extend((n,Shelf(dissipation_s_m=a0),.000125,cls,a.jdk,a.cache) for n in names[:3] for a0 in (0.,1.,12.))
        jobs.extend((n,Shelf(stiffness_n_m=k),.000125,cls,a.jdk,a.cache) for n in ("leftSmall","rightSmall") for k in (1.e5,1.6e6))
        jobs=[(*job,False) for job in jobs]
        if a.fine:
            jobs=[(n,Shelf(stations=16),dt,cls,a.jdk,a.cache,True) for n in ("leftSmall","rightSmall")
                  for dt in (.0000625,.00003125,.000015625)]
        if a.soft:
            jobs=[(n,Shelf(stiffness_n_m=k,stations=16),.0000625,cls,a.jdk,a.cache,True)
                  for k in (20000.,40000.,80000.) for n in ("leftSmall","rightSmall")]
        with ProcessPoolExecutor(max_workers=6) as pool:rows=list(pool.map(replay,jobs))
    (HERE/("soft-ports.json" if a.soft else "fine-ports.json" if a.fine else "ports.json")).write_text(json.dumps({"javaSha256":docs["programSha256"],"results":rows},indent=2)+"\n",encoding="utf8",newline="\n")
    for r in rows:print(r["case"],r["dtS"],r["shelf"]["dissipation_s_m"],r["state"],r["firstBandMs"],r["loss"])
