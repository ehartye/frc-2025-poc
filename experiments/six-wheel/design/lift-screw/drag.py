"""Calculate the admissible passive shaft drag interval for a replacement drive."""
import math


def interval(thread, heights, masses, ratio, mu_low, mu_high, *, downstream_inertia=0):
    from screen import G,HALF_ANGLE,SCHEDULE,profile,size_lift
    k, pitch=thread["radiusM"],thread["pitchRadiusM"]
    lead_tan=k/pitch
    low,high=mu_low/math.cos(HALF_ANGLE),mu_high/math.cos(HALF_ANGLE)
    upper_eff=lead_tan*(1-lead_tan*high)/(lead_tan+high)
    require=0
    available=math.inf
    for m in masses:
        inertia=2*thread["singleInertiaUpperKgM2"]+size_lift.ROTOR_STACK_INERTIA*ratio**2+downstream_inertia
        net_thread=m*G*pitch*(low-lead_tan)/(1+low*lead_tan)
        # Positive margin at rest even with the rotor electrically disconnected.
        require=max(require,-net_thread)
        for _,a,b,t in SCHEDULE:
            for i in range(1001):
                h,v,accel=profile(i/1000,a,b,t)
                if v<0:
                    energy=.5*(inertia+m*k*k)*(v/k)**2
                    require=max(require,energy*k/(h-heights)-net_thread)
                omega=abs(v)/k
                volts_available=9.5-omega*ratio/size_lift.OMEGA_FREE*(12-(12/105)*1.8)
                amps_available=min(35,volts_available/(12/105))
                permitted_motor=(amps_available-1.8)*2.6/(105-1.8)
                extra_torque=(permitted_motor-size_lift.ROTOR_STACK_INERTIA*abs(accel)*ratio/k)*ratio*.95
                load=(m*(G+abs(accel))+20)*k/upper_eff
                extra_torque-=(2*thread["singleInertiaUpperKgM2"]+downstream_inertia)*abs(accel)/k
                available=min(available,extra_torque-load)
    return {"ratio":ratio,"minActualPairNm":require,"maxActualPairNm":available,
            "minSettingWith20pctNm":require/.8,"maxSettingWith20pctNm":available/1.2,
            "toleranceWindowNm":available/1.2-require/.8}


def evaluate(th,masses):
    results={}
    for low,high in ((0,.20),(.05,.20),(.10,.15)):
        grid=[interval(th,.5575,masses,i/100,low,high,downstream_inertia=.000020) for i in range(60,106)]
        coarse=max(grid,key=lambda v:v["toleranceWindowNm"])
        refined=[interval(th,.5575,masses,coarse["ratio"]+j/10000,low,high,downstream_inertia=.000020) for j in range(-100,101,2)]
        best=max(refined,key=lambda v:v["toleranceWindowNm"])
        ratios={key:interval(th,.5575,masses,g,low,high,downstream_inertia=.000020) for key,g in
                (("18to21",18/21),("20to23",20/23),("24to28",24/28),("19to22",19/22))}
        results[f"{low:.2f}..{high:.2f}"]={"best":best,"coarseBest":coarse,
            "positiveGridChoices":sum(r["toleranceWindowNm"]>0 for r in grid),
            "ratiosTested":[.60,1.05,.01],"refinement":[coarse["ratio"]-.01,coarse["ratio"]+.01,.0002],
            "discreteRatioExamples":ratios}
    return results
