"""Sweep the graduated bumper contact locations on the free inset wheelbase and resolve ranked module screw reactions."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from direct_saddle_inertia import construct

HERE=Path(__file__).resolve().parent


def sweep(pkg,stagger=False):
    model=construct(pkg,12.7,stagger)
    ledger_file=HERE/model['ledgerFile']
    ledger=json.loads(ledger_file.read_bytes())
    if ledger["authority"]!=model["authority"]:raise ValueError("Authority changed")
    mass,cg,yaw=ledger["massKg"],np.array(ledger["cgMm"]),ledger["yawKgM2"]
    base=model["base"];n=len(base);module0=model["moduleStart"]
    coords={int(k):np.array(x) for k,x in ledger["fixedSupportDOFs"].items()}
    shapes=np.zeros((n,3))
    for d,(x,y) in coords.items():shapes[d:d+3]=[[1,0,-y],[0,1,x],[0,0,1]]
    # Assemble the dissimilar receivers' inertial load responses once, in N per m/s²
    # (first two columns) and N or N·mm per rad/s² (third). Check the three exact rigid
    # resultant wrenches at this issued mass, centroid and yaw inertia.
    inertia=[]
    for route in (0,1):
        response=np.zeros((n,3))
        for _,kg,xy,J,groups in ledger["items"]:
            xy=np.array(xy);rel=xy-cg;ids=groups[route]
            forces=-kg*np.array([[1.,0.,-rel[1]/1000],[0.,1.,rel[0]/1000]])
            ac=np.mean([coords[d] for d in ids],axis=0);delta=xy-ac
            moment=np.array([0.,0.,-1000*J])+delta[0]*forces[1]-delta[1]*forces[0]
            if len(ids)==1:
                d=ids[0];response[d:d+2]+=forces;response[d+2]+=moment
            else:
                offsets=np.array([coords[d]-ac for d in ids]);polar=(offsets**2).sum()
                for d,(x,y) in zip(ids,offsets):
                    response[d:d+2]+=forces/len(ids)+np.outer([-y,x],moment)/polar
        inertia.append(response)
    loads=model["forces"];cases=model["loadCases"];nc=len(cases)
    contact=shapes.T@loads
    normals=contact[:2]/np.linalg.norm(contact[:2],axis=0)
    # Linear basis for two independent traction directions on each of the four
    # genuine free modules. A single fully loaded wheel bounds the admissible
    # distribution for friction parallel to the local face normal; no yaw clamp.
    tire=np.zeros((n,8))
    for j in range(4):tire[module0+3*j:module0+3*j+2,2*j:2*j+2]=np.eye(2)*(.8*mass*9.80665)
    ext=np.column_stack((loads,tire));wrench=shapes.T@ext
    accel=np.vstack((wrench[:2]/mass,(wrench[2]-cg[0]*wrench[1]+cg[1]*wrench[0])/(1000*yaw)))
    freed=[ext+response@accel for response in inertia]
    max_rigid=max(abs(shapes.T@f).max() for f in freed)
    if max_rigid>1e-3:raise ValueError(f"Unbalanced free load cases {max_rigid}")
    gauge=[module0,module0+1,module0+2];dofs=np.setdiff1d(np.arange(n),gauge)
    stored={};profiles={}
    screw_ranks=[];ar=np.array(model["anchorXY"])
    for side in range(4):
        axis=1 if side%2==0 else 0
        for j in range(6):
            group=ar[side*6:side*6+6,axis]
            rank=sorted(set(np.round(abs(group),3))).index(round(abs(group[j]),3))
            screw_ranks.append(("inner","middle","outer")[rank])
    def keep(k,mat,site,stiff,route):
        # The first N columns are the contact-only cases. For each physical wheel
        # add its two directional influences along the incident face normal. abs(A+B)
        # is compared for both signs; do not combine independent force extrema.
        baseline=mat[:,:nc]
        for swivel in [-1,0,1,2,3]:
            correction=0 if swivel<0 else mat[:,nc+2*swivel,None]*normals[0]+mat[:,nc+2*swivel+1,None]*normals[1]
            for sign in ((0,) if swivel<0 else (-1,1)):
                quantity=abs(baseline+sign*correction)
                if k=="ringNetMPa" and swivel<0:
                    # Normal energy can turn the free robot around its released centroid.
                    # Preserve fixed full-speed force, fixed runout, and fixed stiffness.
                    # Effective mass models below are sensitivity comparisons only at zero external tire force.
                    point=np.array([c["atMm"] for c in cases])
                    lever=((point[:,0]-cg[0])*normals[1]-(point[:,1]-cg[1])*normals[0])/1000
                    ratio=1/(1+mass*lever**2/yaw)
                    for measure,factor in (("fullMassZeroTire",np.ones(nc)),("fixed18mmFreeEnergy",ratio),
                                           ("fixedStiffnessFreeEnergy",np.sqrt(ratio))):
                        scaled=quantity*factor
                        row,col=np.unravel_index(np.argmax(scaled),scaled.shape)
                        if scaled[row,col]>profiles.get(measure,{}).get("MPa",-1):
                            profiles[measure]=dict(MPa=float(scaled[row,col]),retainedLoadFactor=float(factor[col]),
                                site=site[row],contact=cases[col],stiffness=stiff,payload=route)
                ids=np.argmax(quantity,axis=1)
                i=int(np.argmax(quantity[np.arange(len(ids)),ids]));col=int(ids[i]);v=float(quantity[i,col])
                if v>stored.get(k,{}).get("value",-1):
                    stored[k]=dict(value=v,site=site[i],contact=cases[col],stiffnessNPerMm=stiff,
                                    payloadCase=route,tireWheel=swivel,tireForceSign=sign)
                    if k=="ringNetMPa":
                        member,axial_index,bend_index,fibre,area,I,c = sections[i]
                        q=model["ringMembers"][member]@u[model["ringDofs"][member],col]
                        if swivel>=0:
                            q=q+sign*model["ringMembers"][member]@(u[model["ringDofs"][member],nc+2*swivel:nc+2*swivel+2]@normals[:,col])
                        stress=q[axial_index]/area+fibre*q[bend_index]*c/I
                        if abs(abs(stress)-v)>1e-6:raise ValueError("Signed section reconstruction differs")
                        stored[k]["signedSection"]=dict(member=int(member),endAxialN=float(q[axial_index]),endMomentAboutZ_Nmm=float(q[bend_index]),
                            fibreSign=fibre,signedNetStressMPa=float(stress),Amm2=float(area),IzzMm4=float(I),extremeMm=float(c))

    print("contact cases",nc,flush=True)
    for kr in (3000.,30000.):
        for kp in (3000.,30000.):
            for km in (3000.,30000.):
                ks=(kr,kp,km);K=base+sum(k*p for k,p in zip(ks,model["stiffnessParts"]))
                for route,force in enumerate(freed):
                    u=np.zeros_like(force);u[dofs]=np.linalg.solve(K[np.ix_(dofs,dofs)],force[dofs])
                    gauge_res=K[gauge]@u-force[gauge]
                    if abs(gauge_res[:2]).max()>.04 or abs(gauge_res[2]).max()>25:raise ValueError("Gauge reaction")
                    curves=[];locations=[];sections=[]
                    for i,(rr,d) in enumerate(zip(model["ringMembers"],model["ringDofs"])):
                        nxt=(i+1)%len(model["nodes"]);q=rr@u[d]
                        for ix,ax,bend in ((i,0,2),(nxt,3,5)):
                            xy=model["nodes"][ix];a,I,c=model["ringSection"](xy,model["nodes"][nxt]-model["nodes"][i])
                            for sign in (-1,1):
                                curves.append(q[ax]/a+sign*q[bend]*c/I);locations.append(xy.tolist())
                                sections.append((i,ax,bend,sign,a,I,c))
                    keep("ringNetMPa",np.stack(curves),locations,ks,route)
                    curves=[];locations=[]
                    for label,span,rec,d,a,I in model["lowerMembers"]:
                        q=rec@u[d]
                        for m in (2,5):
                            for s in (-1,1):curves.append(q[0]/a+s*q[m]*12.7/I);locations.append((label,span))
                    keep("lowerGrossMPa",np.stack(curves),locations,ks,route)
                    # A projected direction grid gives a rigorous *upper* bound on
                    # the maximum vector norm by sec(2.5°), on the same clock/case.
                    dirs=np.column_stack((np.cos(np.radians(np.arange(0,180,5))),np.sin(np.radians(np.arange(0,180,5)))))/np.cos(np.radians(2.5))
                    screw=np.array([km*j@u for j in model["anchorJac"]])
                    for rank in ("inner","middle","outer"):
                        ii=[j for j,v in enumerate(screw_ranks) if v==rank]
                        p=np.einsum('ai,bij->baj',dirs,screw[ii]).reshape((-1,u.shape[1]))
                        keep(rank+"ScrewForceUpperN",p,[ar[j].tolist() for j in ii for ang in dirs],ks,route)
                    for rank in ("outer","middle","inner"):
                        ids=[j for j,r in enumerate(screw_ranks) if r==rank]
                        # First side is a transverse tube. Resolve the same saved reaction
                        # along and across the tube at its own hole, preserving case/side.
                        for direction in ("along","across"):
                            axes=[(1 if j//6%2==0 else 0)^(direction=="across") for j in ids]
                            keep(rank+"Screw"+direction.title()+"AbsN",np.stack([screw[j,a] for j,a in zip(ids,axes)]),
                                 [ar[j].tolist() for j in ids],ks,route)
                    pilot=np.array([kp*j@u for j in model["pilots"]])
                    p=np.einsum('ai,bij->baj',dirs,pilot).reshape((-1,u.shape[1]))
                    keep("lowerSaddleForceUpperN",p,[j for j in range(len(pilot)) for ang in dirs],ks,route)
                print(ks,round(stored["ringNetMPa"]["value"],2),flush=True)
    return dict(authority=model["authority"],ledgerSha256=hashlib.sha256(ledger_file.read_bytes()).hexdigest(),
        forceLocationsAndPatches=nc,matchedPhysicalRoutes=2,stiffnessTriples=8,tireCases=9,massKg=mass,
        loadCase="12.33891 kN contact on one finite face at a time, no ground reaction at the modules, including point and finite uniformly/triangularly loaded 100 mm patches. Up to 0.8mg at a single physical wheel, both signs parallel to the incident face, bounds intermediate co-directional tire-load distributions. Two independently supported payload rosters. Ideal section and quasistatic impact are conditional. Screw scalar peak is bounded by sec(2.5 deg) after actual force vector projection.",
        maximumRigidWrenchResidual=max_rigid,freeEnergyControl=profiles,peaks=stored)


if __name__=="__main__":
    stagger="--stagger" in sys.argv;d=sweep(Path(sys.argv[1]),stagger);(HERE/("inset-stagger-all-contacts.json" if stagger else "inset-saddle-all-contacts.json")).write_text(json.dumps(d,indent=2)+"\n")
    for k,r in d["peaks"].items():print(k,r["value"])
