"""Advance the free receiver load and six unilateral moving rail witnesses."""
import argparse
import json
from pathlib import Path
import numpy as np
from free_shell import Tube, feed
from running_sections import HERE, ROOT, build, gaps, previous, sample, shape_table
from dynamics_run import equilibrium

def bounded_reactions(R, fm, rhs, coef, active, limits):
    f=np.zeros(len(rhs))
    for _ in range(len(rhs)+1):
        if not len(active): break
        A=R[active]
        mat=np.eye(len(active))+(coef[active,None]*A)@fm@A.T
        val=np.linalg.solve(mat,rhs[active]-coef[active]*(A@(fm@(R.T@f))))
        clipped=np.clip(val,0.,limits[active])
        fixed=val!=clipped
        if not fixed.any():
            f[active]=val
            break
        f[active[fixed]]=clipped[fixed]
        active=active[~fixed]
    return f

def run(case,out,variant,dt,nm=13,hold=1.5,duration=3.,stop=.20,grips=True,tolerance=0.,commands=None):
    out.mkdir(parents=True,exist_ok=True)
    state=previous.read(HERE/"starts.json")["cases"][case]
    if nm<13 or dt<=0 or tolerance<0:raise ValueError("Invalid resolution or tolerance")
    relief=variant in ("relief","combined")
    keep=variant in ("keepers","combined")
    M,K,C,g,pitch,section=build(state,relief,keep,nm)
    n=len(g); shape=shape_table(nm); shell=Tube(state,shape)
    t=np.arange(round((hold+duration+stop)/dt)+1)*dt
    s,sd,sa=previous.profile(t-hold,duration)
    if commands is None:
        commands=np.zeros((len(t),2))
    geo=previous.read(ROOT/"extension-support/geometry.json")
    keeper=previous.read(ROOT/"extension-support/keepers.json")
    rows,dr=previous.contact_rows(s,geo,keeper)
    clear=np.array([.04,.04,.06]*2 if keep else [1.,1.,.35]*2)*.001+tolerance
    ks=np.full(6,1e7)
    J0=np.pad(rows[0],((0,0),(0,n-6)))
    nodes,weights=np.polynomial.legendre.leggauss(16)
    point=shell.p+np.outer(nodes*feed.HALF,shell.a)
    load=sample(shape,point[:,0],point[:,1])[:,0,:].T@(weights*.5)
    q=equilibrium(M,K,g-feed.G*feed.M*load,J0,clear,ks)
    z,v,roll,rollspeed=previous.project_legacy(state)
    q[:len(z)]+=z
    v=np.pad(v,(0,n-len(v)))
    damping=2*.06*np.sqrt(ks/np.einsum("ij,jk,ik->i",J0,np.linalg.inv(M),J0))
    inverse=np.linalg.inv(M+.5*dt*C+.25*dt*dt*K)
    fm=dt*inverse
    V=inverse@(M-.5*dt*C-.25*dt*dt*K)
    D=-fm@K
    grid,gap=gaps(relief)
    local=np.maximum(-.100,.118-grid)
    btn=np.stack([sample(shape,local,np.full(len(grid),cy))[:,0,:] for cy in (-.066,.042)],axis=1)
    btn[:,:,0]-=1;btn[:,:,1]-=.118
    index=np.clip(np.rint(s/.00025).astype(int),0,len(grid)-1)
    ang=np.gradient(btn,.00025,axis=0)
    slope=np.zeros_like(gap)
    for j in range(1,len(grid)-1):
        valid=np.isfinite(gap[:,j+1])&np.isfinite(gap[:,j-1])
        delta=gap[valid,j+1]-gap[valid,j-1]
        slope[valid,j]=np.where(abs(delta)<.001,delta/.0005,0.)
    plungers=np.array(state["plunger"])
    init_excess=np.maximum(0,-gap[:,0]-btn[0]@q-plungers)
    prevgap=gap[:,0]
    rhs=np.zeros(8);R=np.zeros((8,n))
    coef=np.r_[.25*dt*ks+.5*damping,[750.,750.]]
    limits=np.r_[np.full(6,np.inf),[750.,750.]]
    maxrail=np.zeros(6);maxbuttons=np.zeros(2);maxpress=plungers.copy()
    maxintr=np.zeros(2);firststrike=None;maxgap=0.;minguide=np.inf
    longest=0.;off=0.;firstfree=None;firstreturn=None;previous_off=False
    maximum_compression=0.;max_x=-np.inf;min_x=np.inf;negative=None
    hist=[]; terminal=None;first_depart=None;pressure_ports=[]
    stride=max(1,round(.001/dt));end_reason="scheduled stop"
    for i,clock in enumerate(t):
        J=rows[i];j=index[i];B=btn[j]
        gaprow=gap[:,j] if s[i]<=grid[-1] else np.full(2,np.inf)
        depth=-gaprow-B@q
        strike=np.isfinite(gaprow)&(prevgap>gaprow+.001)
        intr=np.where(strike,np.maximum(0,depth-plungers),0.)
        if np.any(intr>0) and firststrike is None:firststrike=[float(clock),float(s[i]),intr.tolist()]
        maxintr=np.maximum(maxintr,intr)
        prevgap=gaprow
        shell.z=-q;shell.zdot=-v
        # Save synchronized absolute local state, independent of commanded rack position.
        top=shell.p[2]+feed.HALF*abs(shell.a[2])+feed.R*np.sqrt(max(0,1-shell.a[2]**2))
        minguide=min(minguide,.625-top)
        extent=feed.HALF*abs(shell.a[0])+feed.R*np.sqrt(max(0,1-shell.a[0]**2))
        max_x=max(max_x,shell.p[0]+extent);min_x=min(min_x,shell.p[0]-extent)
        if i%stride==0 or i==len(t)-1:
            hist.append(np.r_[clock,s[i],sd[i],q,v,shell.p,shell.a,shell.v,shell.l,shell.angular(shell.l),plungers,getattr(shell,"last_normal_n",0.)])
        if i%max(1,round(.020/dt))==0:
            pressure_ports.append([float(clock),*[bool(x) for x in shell.ports()]])
        if shell.p[0]-extent>feed.EDGE and first_depart is None:
            first_depart=dict(timeS=float(clock),wallS=float(state["wallS"]+clock),strokeM=float(s[i]),
                 localCenterM=shell.p.tolist(),axis=shell.a.tolist(),localVelocityMps=shell.v.tolist(),
                 chassisRelativeVelocityMps=(shell.v+[sd[i],0,0]).tolist(),omegaRadS=shell.angular(shell.l).tolist(),
                 angularMomentum=shell.l.tolist(),rearOfPipePastShelfLipM=float(shell.p[0]-extent-feed.EDGE),
                 achievedChassisFieldXYHeading=state.get("chassisFieldXYHeading"))
        if i==len(t)-1: break
        if not np.isfinite(q).all() or max(abs(q[[1,3,5]]))>.1 or top>=.625 or shell.p[2]<.39:
            end_reason="left represented shelf/guide domain";break
        R[:6,:6]=J;R[6:]=B
        shell.commands=commands[i]
        load=shell.advance_tube(dt,q,v,sa[i],grips)
        vfree=V@v+D@q+fm@(g+pitch*sa[i]+load)
        rates=R@(.5*(v+vfree))+np.r_[(dr[i]@q[:6])*sd[i],(ang[j]@q+slope[:,j])*sd[i]]
        gaps_now=np.r_[clear+J@q[:6],-depth]
        comp=-gaps_now-.5*dt*rates
        plungers=np.maximum(0,plungers-dt*.008/.3)
        active=np.flatnonzero(np.r_[comp[:6]>0,comp[6:]>=plungers])
        rhs[:6]=-ks*gaps_now[:6]-(.5*dt*ks+damping)*rates[:6]
        rhs[6:]=-1500*rates[6:]
        f=bounded_reactions(R,fm,rhs,coef,active,limits)
        vnew=vfree+fm@(R.T@f)
        q+=.5*dt*(v+vnew);v=vnew
        plungers=np.maximum(plungers,depth)
        maxrail=np.maximum(maxrail,f[:6]);maxbuttons=np.maximum(maxbuttons,f[6:])
        maxpress=np.maximum(maxpress,plungers)
        off=off+dt if shell.last_normal_n<1e-5 else 0.
        if off>0 and firstfree is None:firstfree=float(clock)
        if previous_off and off==0 and firstreturn is None:firstreturn=float(clock)
        previous_off=off>0;longest=max(longest,off)
        if shell.last_normal_n==0 or i%stride==0:
            station=shell.p+np.outer(nodes*feed.HALF,shell.a)
            bottom=station[:,2]-feed.R*np.sqrt(max(0,1-shell.a[2]**2))
            valid=(station[:,0]>=-.1)&(station[:,0]<=feed.EDGE)&(abs(station[:,1])<=.175)
            if valid.any():
                field=sample(shape,station[:,0],station[:,1])[:,0,:]@q
                maxgap=max(maxgap,float((bottom-.5-field)[valid].min()))
    key=f"{case}-{variant}-{dt*1e6:g}us-{nm}m-{tolerance*1e6:g}um"+("" if grips else "-no-grip")
    dest=out/(key+".npz");h=np.array(hist)
    np.savez_compressed(dest,timeS=h[:,0],strokeM=h[:,1],speedMps=h[:,2],q=h[:,3:3+n],v=h[:,3+n:3+2*n],pipe=h[:,3+2*n:],ports=np.array(pressure_ports))
    r=dict(case=case,variant=variant,dtUs=dt*1e6,modes=nm,addedClearanceUm=tolerance*1e6,gripsEnabled=grips,
           motionSourceSha256=previous.digest(ROOT/"extension-support/dynamics-motion.json"),geometrySha256=previous.digest(HERE/"geometry.json"),
           terminalSourceSha256=state["rawSha256"],traceSha256=previous.digest(dest),tubeMassKg=feed.M,
           holdS=hold,extensionS=duration,lastTimeS=float(clock),lastStrokeM=float(s[i]),termination=end_reason,
           peakRailN=maxrail.tolist(),peakButtonN=maxbuttons.tolist(),buttonTravelMm=(1000*maxpress).tolist(),
           initialPlungerOverlapMm=(1000*init_excess).tolist(),projectedRollResidualMm=1000*roll,
           residualRollSpeedMmS=1000*rollspeed,sharpSideEntryMm=(1000*maxintr).tolist(),firstSharpEntry=firststrike,
           longestZeroNormalMs=1000*longest,firstZeroNormalS=firstfree,firstRecontactS=firstreturn,
           maxApproxLowestStationSeparationMm=1000*maxgap,minimumUpperGuideClearanceMm=1000*minguide,
           peakNormalN=shell.peak_shelf_n,peakCompressionMm=1000*shell.peak_shelf_compression,
           maximumEndCartridgeMm=shell.max_cartridge_mm,firstEndHardStopS=shell.first_hard,
           tubeOccupiedXM=[min_x,max_x],terminalLocalCenterM=shell.p.tolist(),terminalLocalVelocityMps=shell.v.tolist(),
           terminalAxis=shell.a.tolist(),terminalOmegaRadS=shell.angular(shell.l).tolist(),
           terminalChassisFieldXYHeading=state.get("chassisFieldXYHeading"),firstFullyDischarged=first_depart,
           fittedPlateMassKg=section["massKg"],frequenciesHz=section["frequenciesHz"][:4].tolist(),
           physicalQualification=None,placementCredit=None)
    (out/(key+".json")).write_text(json.dumps(r,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(key,r["termination"],r["lastStrokeM"],r["sharpSideEntryMm"],r["longestZeroNormalMs"],flush=True)
    return r

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("out",type=Path);p.add_argument("case");p.add_argument("variant",choices=["baseline","relief","keepers","combined"])
    p.add_argument("--dt-us",type=float,default=62.5);p.add_argument("--modes",type=int,default=13)
    p.add_argument("--extra-gap-um",type=float,default=0.);p.add_argument("--no-grip",action="store_true")
    a=p.parse_args();run(a.case,a.out,a.variant,a.dt_us*1e-6,a.modes,grips=not a.no_grip,tolerance=a.extra_gap_um*1e-6)
