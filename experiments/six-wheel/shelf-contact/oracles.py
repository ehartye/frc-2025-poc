"""Compare finite-shelf dynamics with closed-form energy, rebound, stiffness and moment checks."""
from dataclasses import replace
import json
import math
import numpy as np
from contact import CompliantPipe,Shelf,feed,analytic_impact,HERE


def initial(cfg):
    pipe=CompliantPipe((0,0,0),0,None,cfg)
    pipe.p=np.array([0.,0.,feed.TOP+feed.R])
    pipe.v[:]=0.;pipe.l[:]=0.;pipe.a=np.array([0.,1.,0.])
    pipe.time=0.
    return pipe


def impact(speed,cfg,dt):
    p=initial(cfg);p.v[2]=-speed
    energy0=.5*feed.M*speed**2
    peak_error=0.;release=None
    for _ in range(math.ceil(.1/dt)):
        p.floor(dt);p.p+=p.v*dt;p.time+=dt
        d=max(0.,feed.TOP+feed.R-p.p[2])
        balance=.5*feed.M*p.v[2]**2+.5*cfg.stiffness_n_m*d*d+p.normal_dissipation_j
        peak_error=max(peak_error,abs(balance-energy0))
        if p.touch and d==0 and p.v[2]>0:release=p.time;break
    d_ref,v_ref=analytic_impact(speed,cfg)
    return {"dt":dt,"incomingMps":speed,"stiffnessNm":cfg.stiffness_n_m,"dissipationSm":cfg.dissipation_s_m,
            "referenceMaxCompressionMm":1000*d_ref,"modelMaxCompressionMm":1000*p.peak_shelf_compression,
            "referenceReboundMps":v_ref,"modelReboundMps":float(p.v[2]),"releaseSeconds":release,
            "relativePeakEnergyResidual":peak_error/energy0,"angularImpulseNms":float(np.linalg.norm(p.l))}


def static(label,center,tube_axis,cfg):
    p=initial(cfg);p.a=np.array(tube_axis,float);p.a/=np.linalg.norm(p.a);p.p=np.array(center,float)
    dt=.00001;p.floor(dt)
    return {"case":label,"reactionN":p.shelf_impulse_ns/dt,"torqueNm":(p.l/dt).tolist()}


def run():
    base=Shelf(friction=0.)
    drops=[impact(2.95,replace(base,dissipation_s_m=d),dt) for d in (0.,1.,4.)
           for dt in (.000125,.0000625,.00003125,.000015625)]
    z=feed.TOP+feed.R-.001
    controls=[static("gap above surface",[0,0,z+.002],[0,1,0],base),
              static("full generator, 1 mm",[0,0,z],[0,1,0],base),
              static("half generator, 1 mm",[0,.175,z],[0,1,0],base),
              static("past front edge",[feed.EDGE+.001,0,z],[0,1,0],base)]
    expected=base.stiffness_n_m*.001
    if abs(controls[1]["reactionN"]-expected)>1.e-8 or abs(controls[2]["reactionN"]-.5*expected)>1.e-8:
        raise ValueError("Finite load extent failed")
    if abs(controls[2]["torqueNm"][0]+expected*feed.HALF/4)>1.e-8:
        raise ValueError("Half line of support has the wrong moment")
    record={"symmetricGravityFreeDrops":drops,"static":controls,"elasticHalfPeriodS":math.pi*math.sqrt(feed.M/base.stiffness_n_m)}
    (HERE/"analytical.json").write_text(json.dumps(record,indent=2)+"\n",encoding="utf8",newline="\n")
    for r in drops[3::4]:print(r)


if __name__=="__main__":run()
