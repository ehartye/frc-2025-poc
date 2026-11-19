"""Augment the issued plate Ritz basis with four bracket lands and two guide freedoms."""
import hashlib
import json
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS","1")
from pathlib import Path
import sys
from functools import lru_cache
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/'shelf-structure'))
import response as plate
import coupled
BASE=coupled.CoupledPipe
def read(p):return json.loads(p.read_bytes())

def axes(spacing):
    def offsets(values,width):return [v+s for v in values for s in (-width/2,width/2)]
    ax=offsets([-93,*plate.RIB_X,153],plate.RIB)
    ay=offsets([-168,*plate.RIB_Y,168],plate.RIB)
    for a,b,c,d in plate.OUTLINE+plate.LANDS:
        ax.extend([a-plate.FIT,a,c,c+plate.FIT]);ay.extend([b-plate.FIT,b,d,d+plate.FIT])
    ax.extend([69.,113.,115.,123.,124.85,155.15]);ay.extend(v+p for v in (-66.,42.) for p in (-5.,5.))
    x,wx=plate.axis_cells(-100,165,ax,spacing);y,wy=plate.axis_cells(-180,180,ay,spacing)
    gx,gy=np.meshgrid(x,y)
    return gx.ravel(),gy.ravel(),np.outer(wy,wx).ravel()

def bilinear(x,y,dx=0,dy=0):
    one=np.ones_like(x);zero=np.zeros_like(x)
    if (dx,dy)==(0,0):return np.column_stack((one,x,y,x*y))
    if (dx,dy)==(1,0):return np.column_stack((zero,one,zero,y))
    if (dx,dy)==(0,1):return np.column_stack((zero,zero,one,x))
    if (dx,dy)==(1,1):return np.column_stack((zero,zero,zero,one))
    return np.zeros((len(x),4))

def guide(y,dx=0,dy=0):
    if dx:return np.zeros((len(y),2))
    if dy:return np.tile([1/.360,-1/.360],(len(y),1))
    return np.column_stack((.5+y/.36,.5-y/.36))

def modes(M,K):
    l=np.linalg.cholesky(M)
    v=np.linalg.solve(l,K);v=np.linalg.solve(l,v.T).T
    w,u=np.linalg.eigh((v+v.T)/2)
    if w[0]<=0:raise ValueError("Nonpositive support modes")
    return np.sqrt(w),np.linalg.solve(l.T,u)

@lru_cache(maxsize=4)
def build(stiffness_scale,bearing_stiffness,participation,layers=1):
    file=HERE/'boundary.json';b=read(file)
    if hashlib.sha256((ROOT/'carriage-support/carriage-support.step').read_bytes()).hexdigest()!=b['sourceStepSha256']:raise ValueError('Different bracket stock')
    pts=np.array(b['positionsMm'])/1000
    nodal=np.linalg.inv(bilinear(pts[:,0],pts[:,1]))
    def H(x,y,dx=0,dy=0):return bilinear(x,y,dx,dy)@nodal
    x,y,area=axes(2.2)
    t,h=plate.sections(x*1000,y*1000,True,True);mp=(plate.RHO_PE*t+plate.RHO_AL*h)*area
    bend_pe=plate.E_PE*t**3/(12*(1-plate.NU_PE**2))*area
    bend_al=plate.E_AL*h**3/(12*(1-plate.NU_AL**2))*area
    twist=2*(bend_pe*(1-plate.NU_PE)+bend_al*(1-plate.NU_AL))
    shapes,old_freq,_=plate.plate(True,13,20,2.2,0.,True);use=old_freq<=2300*2*np.pi
    shapes=shapes[:,use];old_freq=old_freq[use];nm=len(old_freq)
    def B(x,y,dx=0,dy=0):return np.column_stack((plate.basis(x,y,13,20,dx,dy)@shapes,H(x,y,dx,dy),np.tile(guide(y,dx,dy),layers)))
    b0=B(x,y);bxy=B(x,y,1,1)
    ng=2*layers; n=nm+4+ng;M=b0.T@(mp[:,None]*b0)
    if np.max(abs(M[:nm,:nm]-np.eye(nm)))>5e-6:raise ValueError('Integration differs from original mass-normalized plate')
    K=np.zeros((n,n));K[:nm,:nm]=np.diag(old_freq**2)
    # Only mixed curvature of the bilinear bridge contributes to plate strain energy.
    kc=bxy.T@(twist[:,None]*bxy);kc[:nm,:nm]=0
    K+=kc
    kb=stiffness_scale*np.asarray(b['stiffnessNPerM'])
    K[nm:nm+4,nm:nm+4]+=kb
    kg=bearing_stiffness*layers
    K[-ng:,-ng:]+=np.eye(ng)*kg
    mov=read(ROOT/'integrated-slide/mass.json')['moving']
    total=read(ROOT/'fixed-lift-omission/mechanics.json')['oneSideLockLoadKg']-coupled.feed.M
    guide_mass=(mov['outerWithAllowanceKg']+mov['middleMassKg']-coupled.feed.M-float(mp.sum()))*participation
    bridge_mass=total-float(mp.sum())-guide_mass
    if min(guide_mass,bridge_mass)<0:raise ValueError('Invalid participating mass')
    # Distribute moving non-plate mass uniformly to four measured support stations.
    # It follows the guide plane (L or R) as well as its own bracket land.
    middle=mov['middleMassKg']*participation if layers==2 else 0.
    guide_static=[(float(mp.sum())+guide_mass)*9.80665/2]*2
    if layers==2:guide_static.extend([(float(mp.sum())+guide_mass-middle)*9.80665/2]*2)
    def mass_vectors(i):
        v=np.zeros(n);v[nm+i]=1
        yield bridge_mass/4,v.copy()
        v[nm+4+i%2]=1
        if layers==2:
            yield middle/4,v.copy()
            v[nm+6+i%2]=1
        yield (guide_mass-middle)/4,v.copy()
    for i in range(4):
        for mass,v in mass_vectors(i):M+=mass*np.outer(v,v)
    omega,Q=modes(M,K)
    # Table includes local plate slopes, four moving support stations, two relative
    # guide lines. All transformed modes are integrated even above the prior plate cutoff.
    tx,ty=np.meshgrid(np.linspace(-.100,.165,107),np.linspace(-.180,.180,145))
    grid=np.stack([(B(tx.ravel(),ty.ravel(),a,c)@Q).reshape(145,107,n) for a,c in [(0,0),(1,0),(0,1)]],axis=2)
    xy=np.array([[.118,-.066],[.118,.042]])
    damper_abs=B(xy[:,0],xy[:,1])@Q
    base_damper=H(xy[:,0],xy[:,1])@Q[nm:nm+4]
    lock=H(np.array([-.087,-.087]),np.array([.2225,-.2225]))@Q[nm:nm+4]
    impulse=np.column_stack((np.ones_like(y),(y+.18)/.36,x+.087)).T@(mp[:,None]*b0)
    # Additional equipment contributes inertial force and first moment on the frame.
    for i,p in enumerate(pts):
        coef=np.array([1,(p[1]+.18)/.36,p[0]+.087])
        for mass,v in mass_vectors(i):impulse+=mass*np.outer(coef,v)
    return dict(table=grid,omega=omega,inertia=impulse@Q,site=Q[nm:nm+4],relative=Q[-ng:],lock=lock,
        Kbase=kb,Kguide=kg,damperRel=damper_abs-base_damper,damperBase=base_damper,
        Cplate=Q[:nm].T@(np.diag(2*.02*old_freq)@Q[:nm]),
        staticGuideN=np.asarray(guide_static),stages=layers,plateKg=float(mp.sum()),frameKg=bridge_mass,
        guideKg=guide_mass,basis=Q,linearMass=M,linearStiffness=K,nm=nm,nodal=nodal,
        modalClosure=float(np.max(abs(Q.T@M@Q-np.eye(n)))), cardinalityError=float(np.max(abs(H(pts[:,0],pts[:,1])-np.eye(4)))))

class FlexiblePipe(BASE):
    settings=(1.,5e6,1.,.002,.0001,1.)
    last=None
    def __init__(self,*args):
        super().__init__(*args)
        st,guide_k,active_mass,self.lash,self.pin_gap,self.pin_factor,*extra=self.settings
        self.layers=extra[0] if extra else 1
        self.lash/=self.layers
        self.model=build(st,guide_k,active_mass,self.layers)
        q=self.model;self.table=q['table'];self.freq=q['omega'];self.inertia=q['inertia']
        self.z=np.zeros_like(self.freq);self.zdot=self.z.copy();self.clock_cache={}
        # Apply the predecessor plate damping on plate strain alone. An open guide
        # gap must not transmit an artificial dashpot force to the frame.
        self.modal_damping=0.
        self.damper_points=q['damperRel']
        self.datum=self.sample(np.array([.1164,.1,.040]),np.zeros(3))[:,0]
        self.pad_table=self.sample(np.full(3,.1251),np.array([-.070,0.,.070]))
        self.site_bounds=np.zeros((2,4));self.guide_bounds=np.zeros((2,2));self.each_guide_bounds=np.zeros((2,2*self.layers));self.pin_peak=np.zeros(2)
        self.base_peak=np.zeros(2);self.pin_clocks=np.zeros(2);self.guide_upper_touches=np.zeros(2*self.layers,dtype=int)
        self.samples=[];FlexiblePipe.last=self
    def advance_modes(self,dt,q):
        m=self.model;relative=m['relative']@self.z
        k=m['Kguide'];c0=m['staticGuideN']/k
        bottom=np.maximum(c0+relative,0)
        top=np.maximum(-c0-self.lash-relative,0)
        self.guide_upper_touches+=(top>0)
        # Zero disturbance at the installed gravity-loaded datum. Lash=0 recovers
        # the linear bilateral line exactly. A floating strip exerts no fictitious tension.
        speed=m['relative']@self.zdot
        cguide=2*.02*np.sqrt(k*m['staticGuideN']/9.80665)
        bottom_force=np.where(bottom>0,np.maximum(0,k*bottom+cguide*speed),0.)
        top_force=np.where(top>0,np.maximum(0,k*top-cguide*speed),0.)
        physical=bottom_force-top_force-m['staticGuideN']
        correction=k*relative-physical
        pin_displacement=m['lock']@self.z
        kpin=self.pin_factor*3*200e9*(np.pi*.008**4/64)/.018**3
        pin=kpin*np.sign(pin_displacement)*np.maximum(abs(pin_displacement)-self.pin_gap,0)
        changed=abs(pin)>self.pin_peak
        self.pin_peak=np.maximum(self.pin_peak,abs(pin));self.pin_clocks=np.where(changed,self.time,self.pin_clocks)
        q=q-m['Cplate']@self.zdot+m['relative'].T@correction-m['lock'].T@pin
        super().advance_modes(dt,q)
        sites=m['site']@self.z;each=m['relative']@self.z;g=each.reshape(self.layers,2).sum(axis=0)
        self.each_guide_bounds[0]=np.minimum(self.each_guide_bounds[0],each);self.each_guide_bounds[1]=np.maximum(self.each_guide_bounds[1],each)
        self.site_bounds[0]=np.minimum(self.site_bounds[0],sites);self.site_bounds[1]=np.maximum(self.site_bounds[1],sites)
        self.guide_bounds[0]=np.minimum(self.guide_bounds[0],g);self.guide_bounds[1]=np.maximum(self.guide_bounds[1],g)
        self.base_peak=np.maximum(self.base_peak,abs(m['damperBase']@self.z))
        if self.touch and self.time-self.touch[0]<.15:
            self.samples.append([self.time,*sites,*g,*pin,*(m['Kbase']@sites),*self.applied_dampers])
