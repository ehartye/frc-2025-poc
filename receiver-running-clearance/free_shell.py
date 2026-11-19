"""Free finite cylinder using the project's issued compliant shelf contact and belt cells."""
import math
from types import SimpleNamespace
import numpy as np
from running_sections import carriage, sample

coupled=carriage.coupled
feed=coupled.feed


class Tube(coupled.CoupledPipe):
    def __init__(self,state,table):
        # Reuse the issued pressure/damping/friction and annular belt dimensions.
        feed.Pipe.__init__(self,(0,0,0),relief=True)
        self.shelf=coupled.Shelf(stations=16)
        self.p,self.a,self.v=(np.array(state[k],dtype=float) for k in ("p","a","v"))
        omega=np.array(state["omega"])
        self.l=feed.IP*omega+(feed.I-feed.IP)*self.a*(self.a@omega)
        self.table=table
        self.idlers=np.array(state["idlers"])
        self.touch=[0,self.p.tolist(),self.v.tolist(),omega.tolist()]
        self.record_mount_load=False
        self.input_wrench=np.zeros(7)
        self.peak_shelf_n=self.peak_shelf_compression=self.shelf_impulse_ns=0.
        self.max_deformed_mm=self.max_contact_slope=0.
        self.reaction=np.zeros(table.shape[-1])
        self.max_cartridge_mm=0.
        self.first_hard=None

    def sample(self,x,y):
        return sample(self.table,x,y)

    def advance_modes(self,dt,q):
        # Original CoupledPipe uses positive downward deflection; structural
        # coordinates in this study are positive upward. Equal/opposite Fz
        # acts on the same interpolation used to calculate the contact impulse.
        self.reaction[:]=-q

    def belt_load(self,dt,q,v,enabled):
        if not enabled:return np.zeros(3),np.zeros(3),np.zeros(2)
        angle=q[5]; c,s=math.cos(angle),math.sin(angle)
        rot=np.array([[c,0,-s],[0,1,0],[s,0,c]])
        origin=np.array([0.,0.,.5+q[4]])
        center=rot.T@(self.p-origin)
        proxy=SimpleNamespace(p=center+np.array([0,0,.5]),a=rot.T@self.a)
        self.idlers-=np.clip(self.idlers,-2.4*dt,2.4*dt)
        force=np.zeros(3); moment=np.zeros(3); rail=np.zeros(2)
        omega=self.angular(self.l)
        for i,side in enumerate((-1,1)):
            cells=feed.Pipe.footprint(proxy,side)
            area=sum(w for _,w in cells)
            if area<=0: continue
            arms=np.array([r for r,_ in cells]);weights=np.array([w for _,w in cells])
            travel=1000*(side*(proxy.p[1]+arms[:,1])-feed.HALF)
            mask=travel>=-self.inward
            if not mask.any():continue
            arms=arms[mask];travel=travel[mask];weights=weights[mask]
            self.max_cartridge_mm=max(self.max_cartridge_mm,float(travel.max()))
            if travel.max()>self.outward and self.first_hard is None:self.first_hard=self.time
            r=arms@rot.T
            xyz=(self.p+r)-origin
            frame_v=np.column_stack((-v[5]*xyz[:,2],np.zeros(len(r)),v[4]+v[5]*xyz[:,0]))
            relative=(self.v+np.cross(omega,r)-frame_v)@rot
            normal=np.maximum(0,self.preload+2*travel+37.52*side*relative[:,1])*weights/max(feed.cfg.area_for_preload,area)
            slip=relative[:,[0,2]].copy();slip[:,0]-=self.idlers[i]
            shear=-feed.cfg.belt_mu*normal[:,None]*slip/np.maximum(feed.cfg.slip_scale,np.linalg.norm(slip,axis=1))[:,None]
            f=np.column_stack((shear[:,0],-side*normal,shear[:,1]))@rot.T
            force+=f.sum(axis=0);moment+=np.cross(r,f).sum(axis=0)
            rail+=np.array([-f[:,2].sum(),np.sum(-f[:,2]*xyz[:,0]+f[:,0]*xyz[:,2])])
        return force,moment,rail

    def advance_tube(self,dt,q,v,rack_accel,grips):
        self.z=-q;self.zdot=-v
        force,moment,rail=self.belt_load(dt,q,v,grips)
        self.v+=dt*(force/feed.M-np.array([rack_accel,0.,feed.G]))
        self.l+=dt*moment
        self.floor(dt)
        self.reaction[4:6]+=rail
        self.p+=dt*self.v
        self.a+=dt*np.cross(self.angular(self.l),self.a)
        self.a/=np.linalg.norm(self.a)
        self.time+=dt
        return self.reaction
