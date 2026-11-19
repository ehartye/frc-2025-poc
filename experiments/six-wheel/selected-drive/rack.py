"""Exercise the original closed-loop control with additional shelf/loom inertia and drag."""
import argparse
import csv
import json
from pathlib import Path
import sys
import importlib.util

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"relieved-match"))
import control
spec=importlib.util.spec_from_file_location("packet_recovery",ROOT/"packet-recovery/replay.py")
recovery=importlib.util.module_from_spec(spec);spec.loader.exec_module(recovery)

def configure(folder):
    recovery.configure(folder)
    recovery.rewrite(folder,"SlideFeedbackTest",[
        ("final double drag;","final double drag; double extraMass,extraDrag;"),
        ("Math.min(Math.abs(effort),drag),effort) : Math.copySign(drag,v)","Math.min(Math.abs(effort),drag+extraDrag),effort) : Math.copySign(drag+extraDrag,v)"),
        ("(effort-friction)/Retreat.MASS","(effort-friction)/(Retreat.MASS+extraMass)"),
        ("Math.abs(effort)<drag","Math.abs(effort)<drag+extraDrag")])
    recovery.rewrite(folder,"IntegratedTest",[
        ('for(String mode:List.of('+','.join('"'+m+'"' for m in recovery.MODES)+'))',
         'for(String mode:List.of("nominal","bridge26","heavyDrag","base","extra50g"))'),
        ('var contract=new PacketClockTest.Contract','motor.extraMass=scenario.equals("base")?0:scenario.equals("extra50g")?1.25:1.20; motor.extraDrag=scenario.equals("heavyDrag")?15:0; var contract=new PacketClockTest.Contract'),
        ('assertTrue(a.stowed);','if(mode.equals("base"))assertTrue(a.stowed);'),
        ('assertEquals(Set.of('+','.join('"'+m+'"' for m in ("nominal",*recovery.RECOVERED))+').contains(mode),b.stowed,mode);',
         'if(mode.equals("base"))assertTrue(b.stowed);')])

def run(images,out,jdk,gradle):
    out.mkdir(parents=True,exist_ok=True)
    control.run(out,jdk,gradle,output=out,source_cache=images,configure=configure)
    with (out/"events.csv").open() as fh:events=list(csv.DictReader(fh))
    results={}
    for mode in ("base","nominal","bridge26","heavyDrag","extra50g"):
        rows={}
        for piece in ("coral-A","coral-B"):
            selected=[r for r in events if r["scenario"]==mode and r["piece"]==piece]
            rows[piece]={r["event"]:int(r["wall_ms"]) for r in selected}
            motion=out/("path-"+mode+"-"+piece+".csv")
            if motion.exists():
                with motion.open() as fh:raw=list(csv.DictReader(fh))
                rows[piece].update(phasePeakA=max(abs(float(r["phase_A"])) for r in raw),
                                   motorPeakV=max(abs(float(r["motor_V"])) for r in raw),rawSHA256=control.sha(motion))
        results[mode]=rows
    print(results,flush=True)
    record={"additionalCarriedKg":1.20,"runningDragN":{"base":27,"nominal":27,"bridge26":27,"heavyDrag":42},
            "extra50gSensitivityKg":1.25,"actualMassAndDragMeasured":False,"cases":results,"inputs":control.read(out/"hal.json")}
    (HERE/"rack.json").write_text(json.dumps(record,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ("images","out","jdk","gradle"):p.add_argument(arg,type=Path)
    a=p.parse_args();run(a.images,a.out,a.jdk,a.gradle)
