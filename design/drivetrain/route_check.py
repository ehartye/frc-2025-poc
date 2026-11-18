"""Inverse dynamics of each rest-to-rest wheel segment, with conditional current and traction limits."""
import argparse
import json
import math
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROUTE=json.loads((HERE/"route.json").read_text())
DEFAULT=dict(massKg=62.0,trackM=.580,radiusM=.0762,inertiaKgM2=62*2*.425**2,
             cgOffsetM=.05,minSideWeightFraction=.44,outerAxleWeightFraction=.75,
             scrubCoefficient=.38,longitudinalFriction=.90,rollingCoefficient=.020,
             rotatingInertiaPerSideKgM2=.040,ratio=20,gearEfficiency=.82,controllerEfficiency=.92,
             openCircuitV=12.4,wiringAndBatteryOhm=.025,auxiliaryA=11,batteryFloorV=9.5,
             phaseLimitA=38.0,branchLimitA=40)
R=12/105
K_T=2.6/(105-1.8)
K_E=(12-1.8*R)/(5676*2*math.pi/60)
CHAIN_LIMIT_N=170*4.4482216152605  # Tsubaki catalog p A-46, endless riveted BF25H
SPROCKET_R=.00635/(2*math.sin(math.pi/32))


def demand(p,kind,v,acc):
    # Symmetric sides under a centered design. CG offset adds a conservative force magnitude.
    # v and acc are expressed in direction of the segment; braking acc is negative.
    m=p["massKg"]; w=m*9.80665; b=p["trackM"]; r=p["radiusM"]
    rolling=w*p["rollingCoefficient"]/2
    if kind=="turn":
        yaw=2*v/b
        alpha=2*acc/b
        scrub=w*p["outerAxleWeightFraction"]*.24765*p["scrubCoefficient"]
        force=(p["inertiaKgM2"]*alpha+scrub)/b+rolling
        uncertainty=m*p["cgOffsetM"]*(abs(alpha)+yaw**2)/2
        lateral=w*p["scrubCoefficient"]/2+uncertainty
    else:
        force=m*acc/2+rolling
        uncertainty=0
        lateral=0
    traction=math.hypot(abs(force)+uncertainty,lateral)/(w*p["minSideWeightFraction"])
    wheel_torque=force*r+p["rotatingInertiaPerSideKgM2"]*acc/r
    # Both extrema are needed: braking and motoring losses have different signs.
    results=[]
    for torque in [wheel_torque-uncertainty*r,wheel_torque+uncertainty*r]:
        out=torque/p["ratio"]
        motor=out/p["gearEfficiency"] if out>=0 else out*p["gearEfficiency"]
        amps=motor/K_T+1.8
        rpm=v/r*p["ratio"]
        volts=K_E*rpm+amps*R
        watts=volts*amps
        load=2*max(0,watts)/p["controllerEfficiency"]
        avail=p["openCircuitV"]-p["wiringAndBatteryOhm"]*p["auxiliaryA"]
        rad=avail**2-4*p["wiringAndBatteryOhm"]*load
        bus=(avail+math.sqrt(rad))/2 if rad>=0 else 0
        branch=load/(2*bus) if bus else math.inf
        results.append({"phaseA":abs(amps),"motorV":abs(volts),"busV":bus,"voltageMarginV":bus-abs(volts),
                        "branchA":branch,"regenerativeW":2*max(0,-watts),"wheelTorqueNm":abs(torque)})
    row={key:(min if key in ("busV","voltageMarginV") else max)(case[key] for case in results) for key in results[0]}
    row["frictionRequired"]=traction
    return row


def assess(params=None,route=ROUTE):
    p={**DEFAULT,**(params or {})}
    rows=[]
    length=time=regen=0
    for s in route["segments"]:
        checks=[]
        # Piecewise acceleration is constant. Examine both endpoints of each interval
        # and interior quadrature for braking generation and integrated electrical energy.
        for phase,duration,start,acc in [("accelerate",s["rise"],0,s["a"]),("cruise",s["coast"],s["peak"],0),
                                         ("brake",s["rise"],s["peak"],-s["a"])]:
            if not duration:
                continue
            n=max(2,math.ceil(duration/.001))
            last=None
            for i in range(n+1):
                obs=demand(p,s["kind"],max(0,start+acc*duration*i/n),acc)
                checks.append(obs)
                if last is not None:
                    regen+=(last+obs["regenerativeW"])*duration/(2*n)
                last=obs["regenerativeW"]
        area=s["peak"]*(s["rise"]+s["coast"])
        expected=abs(s["demand"])*(p["trackM"]/2 if s["kind"]=="turn" else 1)
        if not math.isclose(area,expected,rel_tol=1e-10):
            raise ValueError("Registered profile area differs from commanded motion")
        if s["kind"]=="drive":
            length+=area
        time+=2*s["rise"]+s["coast"]
        rows.append({"kind":s["kind"],"signedDemand":s["demand"],**summarize(checks)})
    stats=summarize(rows)
    # Assume one loop receives all gearbox torque, no loss reduction and 25 N slack-side allowance.
    chain=25+p["ratio"]*K_T*p["phaseLimitA"]/SPROCKET_R
    radial=support_screen(p,chain)
    margin={"phaseA":p["phaseLimitA"]-stats["phaseA"],"branchA":p["branchLimitA"]-stats["branchA"],
            "batteryV":stats["busV"]-p["batteryFloorV"],"motorV":stats["voltageMarginV"],
            "friction":p["longitudinalFriction"]-stats["frictionRequired"],"chainN":CHAIN_LIMIT_N-chain}
    return {"routeSeconds":time,"straightTravelM":length,"parameters":p,"segments":rows,"extrema":stats,
            "maximumBrakingGenerationJ":regen,"chainAtCurrentLimitN":chain,"chainRivetedCatalogN":CHAIN_LIMIT_N,
            "supportScreen":radial,"margins":margin,"parameterCaseClears":all(v>=0 for v in margin.values())}


def support_screen(p,tight):
    # Conservative force magnitudes. Allocate an entire side and the full side torque to
    # one wheel. Both chain pulls align in the bound. Moments are summed at each station.
    left,right=238.62,321.38
    normal=p["massKg"]*9.80665*(1-p["minSideWeightFraction"])
    wheel=normal*math.sqrt(1+p["longitudinalFriction"]**2)
    rows=[]
    for high,low in [(251.5,268),(268,251.5)]:
        loads=[(high,tight+25),(low,50),(290,wheel)]
        inner=sum(force*(right-y)/(right-left) for y,force in loads)
        outer=sum(force for _,force in loads)-inner
        m=max(inner*(y-left)-sum(f*max(0,y-x) for x,f in loads) for y,_ in loads)
        rows.append((inner,outer,m))
    moment=max(x[2] for x in rows)
    torque=p["ratio"]*K_T*p["phaseLimitA"]*1000
    d=12.7
    bending=32*moment/(math.pi*d**3)
    torsion=16*torque/(math.pi*d**3)
    return {"innerBearingRadialN":max(x[0] for x in rows),"outerBearingRadialN":max(x[1] for x in rows),
            "shaftMomentNm":moment/1000,"shaftTorqueNm":torque/1000,
            "inscribedRoundNominalVonMisesMPa":math.sqrt(bending**2+3*torsion**2)}


def summarize(rows):
    return {key:(min if key in ("busV","voltageMarginV") else max)(r[key] for r in rows)
            for key in ("phaseA","branchA","motorV","busV","voltageMarginV","frictionRequired","regenerativeW","wheelTorqueNm")}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("output",type=Path)
    opts=p.parse_args()
    nominal=assess()
    cases={k:assess(v)["margins"] for k,v in {
        "reducedTraction":{"longitudinalFriction":.45},"highScrub":{"scrubCoefficient":.8},
        "weakSupply":{"openCircuitV":10.9,"wiringAndBatteryOhm":.040},
        "fastTwelveToOne":{"ratio":12}}.items()}
    opts.output.write_text(json.dumps({"candidate":nominal,"counterexamples":cases},indent=2,allow_nan=False)+"\n")
    print(nominal["margins"],nominal["extrema"],cases)
