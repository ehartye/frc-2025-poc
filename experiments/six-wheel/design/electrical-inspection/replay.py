"""Phase-resolved electrical design allowances on the project's complete registered two-trip clock."""
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/"design/drivetrain")]
from route_check import demand,DEFAULT
from scripts.registered_transit import routes,wheel_profile
J=lambda p: json.loads(p.read_text())
CIRCUITS=J(HERE/"circuits.json")["branches"]
MISSION=J(ROOT/"mission/registered-field-receipt.json")
LIFT=J(ROOT/"design/receiver-lift/sizing.json")["winch"]
AWG={10:.003277,12:.005211,14:.008286,16:.01317,18:.02095}  # approximate copper DC ohms per metre at 20 C
RESIST={b["load"]:1.25*2*b["leadM"]*AWG[b["awg"]]+.003 for b in CIRCUITS}
# Last 0.25 m on each drive supply stays 12 AWG at the factory controller connection.
for drive in ("DriveLeft","DriveRight"):
    RESIST[drive]+=2*1.25*.25*(AWG[12]-AWG[10])
PHASE_LEADS={"DriveLeft":(12,.70),"DriveRight":(12,.70),"Lift":(12,.75),
             "Slide":(14,2.10),"LeftGrip":(16,2.65),"RightGrip":(16,2.85)}
PHASE_R={k:2*1.25*AWG[g]*length+.003 for k,(g,length) in PHASE_LEADS.items()}
NEO_R=12/105
NEO550_R=12/100


def validate(branches):
    if len({b["channel"] for b in branches})!=len(branches):
        raise ValueError("duplicate protected channel")
    can=[b["can"] for b in branches if b["can"] is not None]+[1]
    if len(set(can))!=len(can):
        raise ValueError("duplicate CAN identity")
    for b in branches:
        if b["awg"]>(12 if b["ratingA"]>30 else 14 if b["ratingA"]>20 else 18):
            raise ValueError("undersized supply conductor")
        if b["load"] in ("RIO","Radio") and (b["channel"] not in (20,21,22) or b["ratingA"]!=10):
            raise ValueError("protected core supply mismatch")
        if b["channel"]>=20 and b["ratingA"]>15 and (b["load"]!="PH" or b["ratingA"]!=20):
            raise ValueError("invalid fused-channel exception")


def rail(voc,common,watts,constant):
    # Solve the stable high-voltage root for each branch and shared supply.
    def branch(v):
        amps={}
        terminal={}
        for key,power in watts.items():
            r=RESIST[key]
            avail=v-r*constant.get(key,0)
            disc=avail*avail-4*r*power
            if disc<=0 or avail<=0:
                raise ValueError("branch voltage collapse")
            terminal[key]=(avail+math.sqrt(disc))/2
            amps[key]=constant.get(key,0)+power/terminal[key]
        return terminal,amps
    v=voc
    for _ in range(100):
        try:
            terminal,amps=branch(v)
        except ValueError:
            break
        next_v=voc-common*sum(amps.values())
        if next_v<=0:
            raise ValueError("common supply collapse")
        if abs(v-next_v)<1e-8:
            return next_v,terminal,amps
        v=next_v
    # Near collapse the iteration can converge slowly. Find the convex residual's
    # minimum over the physical branch domain, then bracket the upper root.
    lo=max(RESIST[k]*constant.get(k,0)+2*math.sqrt(RESIST[k]*p) for k,p in watts.items())+1e-9
    hi=voc
    if lo>=hi: raise ValueError("branch voltage collapse")
    def residual(v):
        return v+common*sum(branch(v)[1].values())-voc
    for _ in range(70):
        a=lo+(hi-lo)/3; b=hi-(hi-lo)/3
        if residual(a)<residual(b): hi=b
        else: lo=a
    lo=(hi+lo)/2; hi=voc
    if residual(lo)>1e-8: raise ValueError("common voltage collapse")
    for _ in range(65):
        mid=(lo+hi)/2
        if residual(mid)>0: hi=mid
        else: lo=mid
    return hi,*branch(hi)


def schedule(turn_accel=None):
    a=routes(-.18); b=routes(.22)
    moving={"empty-first-approach":a[0],"loaded-coral-A":a[1],"empty-return-coral-A":a[2],
            "loaded-coral-B":b[1],"empty-return-coral-B":b[2]}
    for e in MISSION["events"]:
        name=e["event"]
        dt=e.get("elapsed_s",0)
        if not dt:
            continue
        if name in moving:
            r=moving[name]
            if abs(r.duration-dt)>1e-5: raise ValueError("route duration mismatch: "+name)
            for s in r.segments:
                acc=s["a"]
                peak,rise,coast=s["peak"],s["rise"],s["coast"]
                if turn_accel and s["kind"]=="turn":
                    acc=turn_accel
                    peak,rise,coast,_=wheel_profile(s["wheel"],1.25*.580/2,acc)
                for duration,initial,acc in [(rise,0,acc),(coast,peak,0),(rise,peak,-acc)]:
                    if duration: yield name,duration,(s["kind"],initial,acc)
        else:
            yield name,dt,None


def loading(name,wheel,t,heavy=False,mass=62):
    phase={key:0.0 for key in ["DriveLeft","DriveRight","Lift","Slide","LeftGrip","RightGrip"]}
    watts={b["load"]:0.0 for b in CIRCUITS}
    needed={k:0.0 for k in phase}
    watts.update(RIO=15,Radio=12,Observer=25,PH=1.5)  # control, radio and converter design provisions
    def motor(k,phase_a,volts):
        phase[k]=phase_a
        needed[k]=volts+phase_a*PHASE_R[k]
        watts[k]=abs(phase_a*needed[k])/.90+.7
    for k in phase:
        motor(k,0,0)
    if wheel:
        kind,initial,acc=wheel
        params={**DEFAULT,"massKg":mass,"inertiaKgM2":mass*2*.425**2}
        d=demand(params,kind,max(0,initial+acc*t),acc)
        for k in ("DriveLeft","DriveRight"): motor(k,d["phaseA"],d["motorV"])
    lift=("loaded_receive_to_carry" if name=="acquire-retract-lower" else
          "loaded_carry_to_place" if name.startswith("lower-extend") else
          "empty_place_to_carry" if name=="raise-empty-to-travel" else
          "empty_carry_to_receive" if name=="raise-station-receiver" else None)
    if lift:
        rows=LIFT[lift]
        motor("Lift",max(r["peak_motor_phase_A"] for r in rows),max(r["peak_needed_V"] for r in rows))
        watts["PH"]+=3/.85
    grips=(name.startswith(("loaded-","acquire-","lower-extend","observe-floor")))
    if grips:
        for k in ("LeftGrip","RightGrip"): motor(k,6.81,6.724)
    if "extend" in name or "retract" in name:
        motor("Slide",14.4 if heavy else 5.96,3.92 if heavy else 2.90)
    return watts,phase,needed


def replay(voc=12.4,common=.025,heavy=False,mass=62,turn_accel=None):
    validate(CIRCUITS)
    lowest=1e9; headroom=1e9; total=ah=pack_j=trunk_j=0
    stats={b["load"]:dict(peakA=0.,minV=99.,wireAndContactJ=0.,phaseLeadHeatJ=0.,windingCopperJ=0.,maxPhaseA=0.) for b in CIRCUITS}
    events={}; elapsed=0.
    for name,dt,wheel in schedule(turn_accel):
        ev=events.setdefault(name,{"elapsedS":0.,"minBusV":99.,"maxBusA":0.})
        ev["elapsedS"]+=dt
        n=max(1,math.ceil(dt/.020))
        h=dt/n
        for i in range(n+1):
            weight=h*(.5 if i in (0,n) else 1)
            watts,phase,needed=loading(name,wheel,i*h,heavy,mass)
            # Treat compressor as continuously on at its PH ceiling plus hub/valve power.
            try:
                v,terminal,amps=rail(voc,common,watts,{"PH":15})
            except ValueError as error:
                raise ValueError(f"{name} at {elapsed+i*h:.3f} s: {error}") from error
            current=sum(amps.values()); total=max(total,current);lowest=min(lowest,v)
            headroom=min(headroom,min(terminal[k]-req for k,req in needed.items()))
            ev["minBusV"]=min(ev["minBusV"],v);ev["maxBusA"]=max(ev["maxBusA"],current)
            ah+=current*weight/3600
            pack_j+=current**2*(common*.68)*weight;trunk_j+=current**2*(common*.32)*weight
            for k,a in amps.items():
                st=stats[k]
                st["peakA"]=max(st["peakA"],a);st["minV"]=min(st["minV"],terminal[k])
                st["wireAndContactJ"]+=a*a*RESIST[k]*weight
                if k in phase:
                    st["windingCopperJ"]+=phase[k]**2*(NEO550_R if k in ("Slide","LeftGrip","RightGrip") else NEO_R)*weight
                    st["phaseLeadHeatJ"]+=phase[k]**2*PHASE_R[k]*weight
                    st["maxPhaseA"]=max(st["maxPhaseA"],phase[k])
        elapsed+=dt
    return dict(durationS=elapsed,peakBusA=total,minimumBusV=lowest,minimumMotorHeadroomV=headroom,
                minimumDeviceV=min(s["minV"] for s in stats.values()),AH=ah,packHeatJ=pack_j,trunkHeatJ=trunk_j,
                loadEstimateByBranch=stats,events=events)


def round_values(obj):
    if isinstance(obj,dict):return {k:round_values(v) for k,v in obj.items()}
    return round(obj,5) if isinstance(obj,float) else obj


if __name__=="__main__":
    base=replay()
    original_upper=replay(mass=67)
    revised_upper=replay(mass=67,turn_accel=.64)
    weak=replay(voc=11.8,common=.032,heavy=True)
    try:
        replay(voc=10.9,common=.040,heavy=True)
        collapse=None
    except ValueError as error:
        collapse=str(error)
    a={"nominal":base,"upperAtRegisteredTiming":{"massKg":67,"spinWheelAccel":.70,
       "drivePeakPhaseA":original_upper["loadEstimateByBranch"]["DriveRight"]["maxPhaseA"]},
       "upperWithSlowerTurns":revised_upper,"weakenedSource":{"openCircuitV":11.8,"commonOhm":.032,
       "minBusV":weak["minimumBusV"],"minDeviceV":weak["minimumDeviceV"],
       "motorHeadroomV":weak["minimumMotorHeadroomV"]},"collapsedCase":{"openCircuitV":10.9,"commonOhm":.040,
       "observation":collapse},"floorV":9.5,
       "inputSha256":{s:hashlib.sha256((ROOT/s).read_bytes().replace(b"\r\n",b"\n")).hexdigest() for s in
         ("mission/registered-field-receipt.json","design/receiver-lift/sizing.json","design/drivetrain/route_check.py",
          "design/electrical-inspection/circuits.json")}}
    (HERE/"power.json").write_text(json.dumps(round_values(a),indent=2)+"\n")
    print(base["durationS"],base["peakBusA"],base["minimumBusV"],base["minimumDeviceV"],base["minimumMotorHeadroomV"],a["weakenedSource"])
