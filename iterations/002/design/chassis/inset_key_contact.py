"""Bilateral metal-key contact with a measured rectangular clearance and stepped crossbar ends."""
from pathlib import Path
import json
import sys
import numpy as np
from diagonal_lower_elastic import beam

HERE=Path(__file__).resolve().parent


def section(wall, end):
    width,height=50.8,25.4
    if end:
        h=13.325
        rect=[(width,wall,h-wall/2),(wall,h-wall,(h-wall)/2),(wall,h-wall,(h-wall)/2)]
        iy=wall*width**3/12+2*((h-wall)*wall**3/12+wall*(h-wall)*(width/2-wall/2)**2)
    else:
        h=height
        rect=[(width,h,h/2),(-(width-2*wall),h-2*wall,h/2)]
        iy=(h*width**3-(h-2*wall)*(width-2*wall)**3)/12
    area=sum(w*a for w,a,z in rect)
    centroid=sum(w*a*z for w,a,z in rect)/area
    ix=sum(w*a**3/12+w*a*(z-centroid)**2 for w,a,z in rect)
    return dict(area=area,Ihorizontal=iy,Ivertical=ix,Zhorizontal=iy/(width/2),
                Zvertical=ix/max(centroid,h-centroid),centroidAboveBottomMm=centroid)


def member(p,q,properties):
    k,g,standard_area,standard_i=beam(p,q,3.175)
    tangent=(q-p)/np.linalg.norm(q-p)
    transform=np.r_[tangent,0.,-tangent,0.]
    axial=68900*standard_area/np.linalg.norm(q-p)*np.outer(transform,transform)
    bend=properties["Ihorizontal"]/standard_i
    extension=properties["area"]/standard_area
    g[[1,2,4,5]]*=bend;g[[0,3]]*=extension
    return axial*extension+(k-axial)*bend,g


def relieve_end(K,xy):
    # Replace each corresponding original member. Introduce the real transition
    # at Y231.5 and eliminate only its internal, unloaded cross-section DOFs.
    results=[]
    for sign in (-1,1):
        p=np.array([250.,sign*115.3]);q=np.array([250.,sign*231.5]);r=np.array([250.,sign*242.])
        get=lambda pt:next(d for d,v in xy.items() if np.linalg.norm(v-pt)<1e-6)
        a,b=get(p),get(r);d=np.r_[a:a+3,b:b+3]
        kold,_=member(p,r,section(3.175,False))
        k1,_=member(p,q,section(3.,False));k2,recover=member(q,r,section(3.,True))
        block=np.zeros((9,9));block[:6,:6]+=k1;block[3:,3:]+=k2
        j=np.r_[0:3,6:9];i=np.r_[3:6]
        T=np.zeros((9,6));T[j]=np.eye(6);T[i]=-np.linalg.solve(block[np.ix_(i,i)],block[np.ix_(i,j)])
        K[np.ix_(d,d)]+=T.T@block@T-kold
        results.append((float(r[1]),d,recover@T[3:]))
    return results


def bounded_slip(Q,linear,half_gap):
    # Convex box QP. The three free plate motions make Q semidefinite.
    # Coordinate minimization permits nonunique zero-force motion; forces and
    # frame displacement are unique. Enforce the actual KKT, not solver status.
    x=np.zeros(len(linear));res=linear.copy();h=half_gap
    for iteration in range(30000):
        for i,qii in enumerate(np.diag(Q)):
            value=np.clip(x[i]-res[i]/qii,-h,h)
            diff=value-x[i];x[i]=value;res+=Q[:,i]*diff
        grad=Q@x+linear
        violation=np.where(x<=-h+1e-12,np.minimum(grad,0),np.where(x>=h-1e-12,np.maximum(grad,0),grad))
        if np.max(abs(violation))<2e-6:return x,iteration+1
    raise ValueError("Gap equilibrium did not converge")


def solve(K,free,rhs,panels,k,gap):
    H=np.vstack([p["hostMap"] for p in panels])
    C=np.zeros((len(H),len(H)))
    n=len(panels[0]["supports"])*2
    for i,p in enumerate(panels):
        C[i*n:(i+1)*n,i*n:(i+1)*n]=p["contactStiffness"]
    C=(C+C.T)/2
    coupling=H.T@C
    reduced=K[np.ix_(free,free)]
    X=np.linalg.solve(reduced,np.column_stack([rhs[free],coupling[free]]))
    base=X[:,:rhs.shape[1]]; influence=X[:,rhs.shape[1]:]
    Q=C-coupling[free].T@influence;Q=(Q+Q.T)/2
    u=np.zeros_like(rhs);out=[]
    for j,trial in enumerate(base.T):
        slip,iterations=bounded_slip(Q,-coupling[free].T@trial,gap)
        u[free,j]=trial+influence@slip
        force=C@(H@u[:,j]-slip)
        delta=slip+force/k
        predicted=k*(np.maximum(delta-gap,0)+np.minimum(delta+gap,0))
        error=np.max(abs(predicted-force))
        residual=K@u[:,j]-coupling@slip-rhs[:,j]
        bal=np.max(abs(residual));free_residual=np.max(abs(residual[free]))
        if error>5e-6 or free_residual>.001 or bal>1.:raise ValueError(("Contact/force imbalance",error,bal))
        out.append(dict(sheetLocalN=force.reshape((2,n)).tolist(),hostMinusSheetLocalMm=delta.reshape((2,n)).tolist(),
                        activeFaces=np.sign(np.where(abs(force)>1e-5,force,0)).reshape((2,n)).astype(int).tolist(),
                        faceLawErrorN=error,assemblyResidualN_or_Nmm=bal,freeDofResidualN_or_Nmm=free_residual,iterations=iterations))
    return u,out


if __name__=="__main__":
    from inset_corner_bypass import run
    truss="brace-access" if "--brace-access" in sys.argv else "--truss" in sys.argv
    result=run(Path(sys.argv[1]),True,True,truss)
    result["scope"]="Private 552-derived frame, freely floating sheets and two opposed faces per Cartesian axis at 0.10 mm, zero friction and no clamp preload. Four stiffness/mesh models under the signed rare stop, routine strike and traction. Includes the relieved transverse end. Quasistatic signed endpoints. Retains earlier imposed total mass and does not claim transient impact sequencing."
    result["relievedSectionAt3mm"]=section(3.,True)
    (HERE/("inset-brace-access-key-contact.json" if truss=="brace-access" else "inset-truss-key-contact.json" if truss else "inset-key-contact.json")).write_text(json.dumps(result,indent=1)+"\n")
    for name,variant in result["outcomes"].items():
        row=variant["cases"]["governing rare wall"]
        print(name,row["sameCaseRingMPa"],row["maxTransverseMountTravelMm"],row["maxDeckBoltN"])
