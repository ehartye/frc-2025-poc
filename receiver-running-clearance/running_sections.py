"""Project the revised native section into the plate space and remove the carried mass."""
from functools import lru_cache
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS","1")
from pathlib import Path
import sys
import numpy as np
from running_geometry import CUT_MM, FINISH_X, START_X, LANE_HALF, STATIONS, bottom_mm, read

HERE=Path(__file__).resolve().parent; ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"extension-support"))
import dynamics as previous
from dynamics import carriage, plate


def cut(x,y,pe,al):
    in_lane=(abs(y-STATIONS[0])<=LANE_HALF)|(abs(y-STATIONS[1])<=LANE_HALF)
    loss=np.maximum(0,bottom_mm(x)/1000-(.5-pe-al))*in_lane*(x<=START_X)
    return np.maximum(0,pe-np.maximum(0,loss-al)),np.maximum(0,al-loss)


@lru_cache(maxsize=6)
def fields(relief,nm):
    geo=read(HERE/"geometry.json")
    mode,omega,_=plate.plate(True,13,20,2.2,0.,True)
    phi=mode[:,:nm]; w=omega[:nm]
    x,y,area=carriage.axes(2.2)
    pe,al=plate.sections(x*1000,y*1000,True,True)
    p,a=cut(x*1000,y*1000,pe,al) if relief else (pe,al)
    exact=sum((geo["parts"][k]["revisedMm3" if relief else "originalMm3"]*rho*1e-9)
              for k,rho in [("r6-polymer-facing",plate.RHO_PE),("r6-receiver-insert",plate.RHO_AL)])
    m=(p*plate.RHO_PE+a*plate.RHO_AL)*area
    scale=exact/m.sum();m*=scale
    b=plate.basis(x,y,13,20)@phi
    mass=b.T@(m[:,None]*b)
    K=np.diag(w*w)
    if relief:
        # Project the differential material bending and twist on the existing
        # Ritz space. The retained top face and stiff rib strip remain distinct.
        dp=area*plate.E_PE*(p**3-pe**3)/(12*(1-plate.NU_PE**2))
        da=area*plate.E_AL*(a**3-al**3)/(12*(1-plate.NU_AL**2))
        bend=dp+da; nu=dp*plate.NU_PE+da*plate.NU_AL
        xx=plate.basis(x,y,13,20,2,0)@phi
        yy=plate.basis(x,y,13,20,0,2)@phi
        xy=plate.basis(x,y,13,20,1,1)@phi
        K+=xx.T@(bend[:,None]*xx+nu[:,None]*yy)+yy.T@(bend[:,None]*yy+nu[:,None]*xx)
        K+=xy.T@((2*(bend-nu))[:,None]*xy)
    freq,T=carriage.modes(mass,K)
    C=mass@T@np.diag(.04*freq)@T.T@mass
    rigid=np.stack((np.ones_like(x),x))
    return dict(phi=phi,mass=mass,k=K,c=C,cross=(rigid*m)@b,
                rigid=(rigid*m)@rigid.T,gravity=-previous.G*m@b,
                frequenciesHz=freq/(2*np.pi),massKg=exact,quadratureScale=scale)


def build(state,relief,keepers,nm=13,cg=.04):
    original=read(HERE/"geometry.json")
    M0,K0,C0,g0,pitch,pipes,weights,stage=previous.matrices(state,True,cg)
    tube=read(ROOT/"extension-support/balance.json")["pipeMassKg"]
    # The pipe is now a separate rigid body. Subtract both its vertical mass
    # and its full gravity/offset acceleration contribution to the fixture.
    pipe_mass=tube*(pipes.T*weights)@pipes
    M0-=pipe_mass
    g0+=previous.G*tube*(weights@pipes)
    pitch[5]-=tube*(state["p"][2]-.467)
    f=fields(relief,nm)
    n=6+nm
    M=np.zeros((n,n));K=np.zeros_like(M);C=np.zeros_like(M)
    M[:6,:6]=M0[:6,:6];K[:6,:6]=K0[:6,:6];g=np.r_[g0[:6],f["gravity"]]
    # Replace the original fitted plate rigid block with the actual CAD mass
    # and consistent quadrature from this variant.
    old=f["rigid"] if not relief else fields(False,nm)["rigid"]
    M[4:6,4:6]+=f["rigid"]-old
    g[4:6]-=previous.G*(f["rigid"][0]-old[0])
    p=np.r_[pitch[:6],np.zeros(nm)]
    p[5]+=(f["rigid"][0,0]-old[0,0])*.035
    M[6:,6:]=f["mass"];M[4:6,6:]=f["cross"];M[6:,4:6]=f["cross"].T
    K[6:,6:]=f["k"];C[6:,6:]=f["c"]
    if keepers:
        for name,part in original["parts"].items():
            if not name.startswith("slide-"): continue
            i=2 if name.endswith("-middle") else 0
            for key,sign in (("revised",1),("original",-1)):
                dm=sign*part[key+"Mm3"]*plate.RHO_AL*1e-9
                x=part[key+"CenterMm"][0]/1000
                M[i:i+2,i:i+2]+=dm*np.array([[1,x],[x,x*x+.32**2/12]])
                g[i:i+2]-=previous.G*dm*np.array([1,x])
    if np.linalg.eigvalsh(M)[0]<=0: raise ValueError("Nonpositive receiver mass")
    return M,K,C,g,p,f


@lru_cache(maxsize=4)
def shape_table(nm):
    # All variants use the same reference trial functions. Machining alters
    # the coupled mass, stiffness, damping and therefore the evolved amplitudes.
    phi=fields(False,nm)["phi"]
    x,y=np.meshgrid(np.linspace(-.100,.165,107),np.linspace(-.18,.18,145))
    n=6+nm
    t=np.zeros((len(x.ravel()),3,n))
    t[:,0,4]=1;t[:,0,5]=x.ravel();t[:,1,5]=1
    for i,(a,b) in enumerate(((0,0),(1,0),(0,1))):
        t[:,i,6:]=plate.basis(x.ravel(),y.ravel(),13,20,a,b)@phi
    return t.reshape(145,107,3,n)


def sample(t,x,y):
    xx=np.clip((x+.100)/.0025,0,106-1e-10)
    yy=np.clip((y+.180)/.0025,0,144-1e-10)
    i=xx.astype(int);j=yy.astype(int)
    fx=(xx-i)[:,None,None];fy=(yy-j)[:,None,None]
    return (t[j,i]*(1-fx)+t[j,i+1]*fx)*(1-fy)+(t[j+1,i]*(1-fx)+t[j+1,i+1]*fx)*fy


@lru_cache(maxsize=2)
def gaps(relief):
    grid,base=previous.button_table()
    if not relief: return grid,base
    d=np.linspace(-3.2,3.2,33);xx,yy=np.meshgrid(d,d);use=xx*xx+yy*yy<=3.2**2+1e-12
    out=[]
    for cy in STATIONS:
        h=[]
        for chunk in np.array_split(grid*1000,30):
            x=118-chunk[:,None]+xx[use];y=np.broadcast_to(cy+yy[use],x.shape)
            p,a=plate.sections(x.ravel(),y.ravel(),True,True)
            p,a=cut(x.ravel(),y.ravel(),p,a)
            bottom=.5-(p+a).reshape(x.shape);bottom[(x< -100)|(x>165)]=np.inf
            h.extend(bottom.min(axis=1)-.4849)
        out.append(h)
    return grid,np.array(out)
