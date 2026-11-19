"""Join a single fixed-camera delay to its independently observed pipe and rerun desktop HAL."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"relieved-match"))
import control

SWEEP=tuple("delay"+str(d) for d in range(23,35))
RECOVERED=("bridge26","bridge30",*SWEEP[:10])
MODES=("nominal","single","lateB",*RECOVERED,*SWEEP[10:],"bothLate","secondGap","bridgeLateSector","roofMismatch","fixedDuplicate",
       "fixedSilent","wrongEpoch","wrongPiece","roofSilent","rackJam","expiredPrior")

def rewrite(folder,name,edits):
    path=folder/(name+".java")
    source=path.read_text()
    for old,new in edits:source=control.replace(source,old,new)
    path.write_text(source,encoding="utf8",newline="\n")

def configure(folder):
    shutil.copyfile(HERE/"GapWitness.java",folder/"GapWitness.java")
    rewrite(folder,"PacketClockTest",[
        ('case "one-late" ->', '''case "bridge26", "bridge30", "bothLate", "secondGap", "roofMismatch", "roofSilent", "fixedDuplicate", "fixedSilent", "wrongEpoch", "wrongPiece", "rackJam", "expiredPrior" -> {
                    if (!moving && n==5) arrival=capture+(r.scenario().equals("bridge30")?30:26);
                    if (moving && n==5 && r.scenario().equals("bothLate")) arrival=capture+26;
                    if (!moving && n==6 && r.scenario().equals("secondGap")) arrival=capture+26;
                    if (moving && n==5 && r.scenario().equals("roofMismatch")) b=new double[]{800,805,-2,2,-2,2};
                    if (moving && n==5 && r.scenario().equals("roofSilent")) continue;
                    if (!moving && n==6 && r.scenario().equals("fixedDuplicate")) seq--;
                    if (!moving && n>=6 && r.scenario().equals("fixedSilent")) continue;
                    if (!moving && n==6 && r.scenario().equals("wrongEpoch")) epoch="other-attempt";
                    if (!moving && n==6 && r.scenario().equals("wrongPiece")) key="coral-A";
                }
                case "bridgeLateSector" -> {if (!moving && n==6)arrival=capture+26;}
                case "one-late" ->''')])
    rewrite(folder,"PacketClockTest",[
        ('default -> {}','default -> {if(r.scenario().startsWith("delay") && !moving && n==5)arrival=capture+Integer.parseInt(r.scenario().substring(5));}')])
    rewrite(folder,"IntegratedTest",[
        ("OpticalWindow gate;","OpticalWindow gate; GapWitness standby; int reported; boolean suspended;"),
        ('? "one-late":"constant";','? "one-late":piece.equals("coral-B")?scenario:"constant";'),
        ('gate=new OpticalWindow(new OpticalWindow.Context(', 'var ctx=new OpticalWindow.Context('),
        ('release,prior,first,first,40,180));','release,prior,first,first,40,180); gate=new OpticalWindow(ctx); standby=new GapWitness(ctx);'),
        ('if(d!=null && !d.accepted()){failed=true;event(log,mode,w,"hold",d.reason());return;}', '''boolean recovery=piece.equals("coral-B") && !Set.of("nominal","single","lateB").contains(mode);
                    var stand=recovery?standby.update(t,batch,true):null;
                    for(var lines=standby.history();reported<lines.size();reported++)event(log,mode,w,"observe",lines.get(reported));
                    if(d!=null && !d.accepted()){
                        if(recovery && (d.reason().equals("camera-silent") || d.reason().equals("unobserved-age-gap"))){
                            if(!suspended){event(log,mode,w,"inhibit",d.reason());suspended=true;}
                            d=stand;
                            if(d==null && standby.reason().equals("waiting"))return;
                            if(d==null){failed=true;event(log,mode,w,"hold",standby.reason());return;}
                        } else {failed=true;event(log,mode,w,"hold",d.reason());return;}
                    }'''),
        ('config,stroke,prior,capture,capture+15','config,stroke,mode.equals("expiredPrior") && piece.equals("coral-B")?prior-100:prior,capture,capture+15'),
        ('motor.step(volts,false,true);','motor.step(volts,mode.equals("rackJam") && piece.equals("coral-B"),true);'),
        ('List.of("nominal","lateB","single")','List.of('+','.join('"'+n+'"' for n in MODES)+')'),
        ('assertFalse(b.issued && !a.stowed);','''assertTrue(a.stowed);
                assertEquals(Set.of('''+','.join('"'+m+'"' for m in ("nominal",*RECOVERED))+''').contains(mode),b.stowed,mode);
                assertFalse(b.issued && !a.stowed);''')])

def run(raw,out,jdk,gradle):
    out.mkdir(exist_ok=True,parents=True)
    control.run(out,jdk,gradle,output=HERE,source_cache=raw,configure=configure)
    with (HERE/"events.csv").open() as f:events=list(csv.DictReader(f))
    with (ROOT/"relieved-match/events.csv").open() as f:base=list(csv.DictReader(f))
    for mode in ("nominal","single","lateB"):
        assert [r for r in events if r["scenario"]==mode]==[r for r in base if r["scenario"]==mode]
    reg=control.read(ROOT/"relieved-match/hal.json")
    assert control.sha(HERE/"controller-register.json")==reg["registration"][-1]
    cases={}
    for mode in MODES:
        selected=[r for r in events if r["scenario"]==mode and r["piece"]=="coral-B"]
        final={r["event"]:int(r["wall_ms"]) for r in selected}
        identical=False
        if mode in RECOVERED:
            normal=[r for r in base if r["scenario"]=="nominal" and r["piece"]=="coral-B"]
            required=("feed","seat","carry","release","withdraw","motion","stow")
            assert all(final[n]==int(next(r["wall_ms"] for r in normal if r["event"]==n)) for n in required)
            name="path-"+mode+"-coral-B.csv"
            identical=control.sha(out/name)==reg["rackPaths"]["path-nominal-coral-B.csv"]
            assert identical
        cases[mode]={"BEventsWallMs":final,"lastBEvent":selected[-1]["witness"] if selected else None,
                     "bridge":[r["witness"] for r in selected if r["event"]=="observe"],
                     "matchesNominalRack":identical}
    result={"sourceMatch":control.sha(ROOT/"relieved-match/match.json"),"sourceEvents":control.sha(ROOT/"relieved-match/events.csv"),
            "sourceHal":control.sha(ROOT/"relieved-match/hal.json"),"hal":control.sha(HERE/"hal.json"),"cases":cases}
    (HERE/"results.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("raw","out","jdk","gradle"):p.add_argument(key,type=Path)
    a=p.parse_args()
    run(a.raw,a.out,a.jdk,a.gradle)
