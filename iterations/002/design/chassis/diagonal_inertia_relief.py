"""Compare a freely decelerating Red 2 and an opposed bumper pinch using the same actual attachment stations."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

from diagonal_lower_elastic import run as frame
from terminal_net_sections import section

HERE = Path(__file__).resolve().parent


def rows(name):
    return json.loads((HERE/name).read_bytes())


def perpendicular(vector):
    return np.array([-vector[1],vector[0]])


def cross(a,b):
    return a[0]*b[1]-a[1]*b[0]


def run(package, alternative=None):
    model=frame(package,free_model=True) if alternative is None else alternative(package)
    mass=rows("inspection-mass-552.json")
    clock=rows("terminal-mass-clock.json")["clock"]["cases"]["heavy-offset-control"]
    required_mass=mass["cases"]["heavy-offset-control"]["fieldKg"]
    required_cg=np.array(clock["cgMm"][:2])
    required_yaw=clock["estimatedYawInertiaKgM2"]
    budget=mass["currentCohortsKg"]
    nodes=model["nodes"]
    n=model["ringSize"]
    module0=model["moduleStart"]
    modules=model["moduleCenters"]
    centers=model["sideCenters"]
    count=len(model["base"])
    rigid=np.zeros((count,3))
    node_xy={}
    def add_node(first,xy):
        x,y=xy
        node_xy[first]=np.array([x,y])
        rigid[first:first+3]=[[1,0,-y],[0,1,x],[0,0,1]]
    for i,xy in enumerate(nodes):add_node(3*i,xy)
    for first,xy in model.get('additionalNodes',[]):add_node(first,xy)
    if alternative is None:
        for i,xy in enumerate(centers):add_node(n+3*i,xy)
    for first,side,xy in model["lowerNodes"]:add_node(first,xy)
    for i,xy in enumerate(modules):add_node(module0+3*i,xy)

    # Every independently commissioned body sends one force and one centroid couple to
    # its support group. Two or more supports carry that couple as distributed forces.
    # One rigid module receives its actual wrench, permitting free motion.
    def deposit(vector,group,xy,f,moment):
        xy=np.array(xy)
        center=np.mean([node_xy[i] for i in group],axis=0)
        residual=moment+cross(xy-center,f)
        if len(group)==1:
            a=group[0]
            vector[a:a+2]+=f
            vector[a+2]+=residual
            return
        arms=np.array([node_xy[i]-center for i in group])
        lever=(arms*arms).sum()
        forces=f[None,:]/len(group)+residual*np.column_stack((-arms[:,1],arms[:,0]))/lever
        for idx,v in zip(group,forces):
            vector[idx:idx+2]+=v

    # A discrete physical inventory is brought to the exact previously registered adverse
    # mass properties. The unissued bracket/attachment allowance carries the explicit closure;
    # no motor, pipe, battery, or measured purchased article moves to force a match.
    parts=[]
    terminal=model.get('terminalNodes',[n+3*i for i in range(4)])
    group_nodes={s:[first for first,side,_ in model["lowerNodes"] if side==s] for s in range(4)}
    def part(name,kg,cg,j,groups):
        parts.append(dict(name=name,kg=kg,cg=np.array(cg,dtype=float),j=float(j),groups=groups))
    def groups(same):return [same,same]
    for i,xy in enumerate(modules):
        part("SDS module "+str(i),budget["catalogFourSDSModules"]/4,xy,.009,groups([module0+3*i]))
    for i,xy in enumerate(centers):
        k=budget["measuredFourDrilledRails"]
        actuals=mass["fourActualDrilledRailsKg"]
        matched=[v for key,v in actuals.items() if ("transverse" if i%2==0 else "longitudinal") in key]
        kg=sum(matched)/2
        length=(470 if i%2==0 else 480)*.001
        part("Lower rail "+str(i),kg,np.mean([node_xy[d] for d in group_nodes[i]],axis=0),kg*length**2/12,groups(group_nodes[i]))
        terminal_kg=budget["measuredFourCartridgeVolumes"]/4
        part("Cartridge "+str(i),terminal_kg,xy,terminal_kg*.254**2/12,groups([terminal[i]]))
    chord=np.linalg.norm(nodes-np.roll(nodes,-1,axis=0),axis=1)
    weights=.5*(chord+np.roll(chord,1))/chord.sum()
    for i,weight in enumerate(weights):
        radial=nodes[i]/np.linalg.norm(nodes[i])
        part("Ring "+str(i),budget["plannedSixMiteredGrossTubes"]*weight,nodes[i],0.,groups([3*i]))
        part("Removable bumper "+str(i),budget["removableBumpersOneColor"]*weight,nodes[i]+47*radial,0.,groups([3*i]))
    diag=rows("corner-tie-trial.json")
    for sx,sy in ((1,1),(-1,1),(-1,-1),(1,-1)):
        a=np.array([sx*319.2,sy*65]);b=np.array([sx*65,sy*314.2])
        ij=[0 if sx>0 else 2,1 if sy>0 else 3]
        kg=diag["diagonalStockKg"]/4
        part("Diagonal "+str(ij),kg,(a+b)/2,kg*(np.linalg.norm(b-a)*.001)**2/12,groups([terminal[i] for i in ij]))
    # The detailed support saddles have not been built. Distribute their wrenches to
    # the physical carrier faces, with a second case excluding the remote terminal for
    # each off-center article. This tests a genuinely different load distribution.
    rear=[terminal[i] for i in (1,2,3)]
    front=[terminal[i] for i in (0,1,3)]
    part("Battery",budget["batteryAndBatterySideConnector"],[-100,35],.02185,[terminal,rear])
    part("Panel and electronics",budget["plannedElectricalAssembly"],[35,45],.12396,[terminal,front])
    part("Pan drive and upper",budget["plannedUpperAndDrive"],[300,25],.27991,[terminal,front])
    part("Held CORAL",budget["admittedGamePiece"],[240,0],.00708,[terminal,front])
    part("General hardware",budget["unspecifiedInstalledHardwareAndFinish"],[0,0],.04,groups(terminal))
    part("Joint contingency",1.25,[285,170],0,[terminal,front])
    part("Upper contingency",.5,[300,100],0,[terminal,front])
    part("Harness contingency",.25,[200,200],0,[terminal,front])
    # Keep the unspent gross weight available to the lower or off-center payload. This
    # reproduces the adverse design issue and does not count an unbuilt joint as mass saved.
    left=budget["remainingPlannedCrossframeAndInterfaceAllowances"]-diag["diagonalStockKg"]
    part("Unissued frame interfaces and reserve",left,[0,0],0,groups(terminal))
    if alternative is not None:
        for r in parts:
            matches=[groups for key,groups in model["payloadNodes"].items() if r["name"].startswith(key)]
            if len(matches)>1:raise ValueError(r["name"])
            if matches:r["groups"]=matches[0]
    measured_mass=sum(r["kg"] for r in parts)
    if abs(measured_mass-required_mass)>1e-6:raise ValueError(f"Mass ledger mismatch {measured_mass} {required_mass}")
    reserve=parts[-1]
    reserve["cg"]+=(required_mass*required_cg-sum(r["kg"]*r["cg"] for r in parts))/reserve["kg"]
    fixed_i=sum(r["j"]+r["kg"]*np.dot(r["cg"]-required_cg,r["cg"]-required_cg)*1e-6 for r in parts)
    reserve["j"]=required_yaw-fixed_i
    reserve_radius=np.sqrt(reserve["j"]/reserve["kg"])*1000 if reserve["j"]>=0 else -1
    if not (0<reserve_radius<465 and np.linalg.norm(reserve["cg"])<250):raise ValueError(f"Unphysical residual {reserve_radius} {reserve['cg']}")

    ledger={"authority":model["authority"],"units":{"xy":"mm","mass":"kg","polar":"kg m2 about own centroid"},
            "cgMm":required_cg.tolist(),"massKg":required_mass,"yawKgM2":required_yaw,
            "fixedSupportDOFs":{str(i):pt.tolist() for i,pt in node_xy.items()},
            "items":[[r["name"],r["kg"],r["cg"].tolist(),r["j"],r["groups"]] for r in parts],
            "columns":["article","mass","centroid","centroidPolar","supportNodeDofsByScenario"],
            "inputs":{name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in
                      ["inspection-mass-552.json","terminal-mass-clock.json","corner-tie-trial.json","socket-holes.json","diagonal_lower_elastic.py","diagonal_inertia_relief.py"]}}
    (HERE/model.get("ledgerFile","diagonal-free-mass-ledger.json")).write_text(json.dumps(ledger,separators=(",",":"))+"\n")
    case_ids={}
    impact_y=model.get("impactY",352.3)
    for label,xy in {"port":[109,impact_y],"starboard":[109,-impact_y]}.items():
        case_ids[label]=next(i for i,c in enumerate(model["loadCases"]) if c["widthMm"]==0 and np.linalg.norm(np.array(c["atMm"])-xy)<.001)
    peak=diag["upperCaseN"]
    cases=[];forces=[]
    for label,opposed in (("free strike",False),("opposed aligned same-strength pinch",True)):
        contact=model["forces"][:,case_ids["port"]].copy()
        if opposed:contact+=model["forces"][:,case_ids["starboard"]]
        # Extreme feasible gross floor traction, plus zero traction. Concentrating the
        # entire normal reaction on each wheel is a conservative support-location envelope.
        for mu,dist,sgn in ([(0,"none",0)] if opposed else
            [(0,"none",0)]+[(.8,dist,sgn) for dist in ["equal",0,1,2,3] for sgn in (-1,1)]):
            ext=contact.copy()
            floor=sgn*mu*required_mass*9.80665
            for i in range(4):
                share=.25 if dist=="equal" else float(i==dist)
                ext[module0+3*i+1]+=floor*share
            total=rigid.T@ext
            acc=total[:2]/required_mass
            alpha=(total[2]-cross(required_cg,total[:2]))*.001/required_yaw
            for route in (0,1):
                vector=ext.copy()
                for r in parts:
                    local_a=acc+alpha*perpendicular(r["cg"]-required_cg)*.001
                    deposit(vector,r["groups"][route],r["cg"],-r["kg"]*local_a,-1000*r["j"]*alpha)
                equilibrium=rigid.T@vector
                if abs(equilibrium[:2]).max()>1e-6 or abs(equilibrium[2])>1e-3:
                    raise ValueError(f"Unbalanced inertia relief {equilibrium}")
                cases.append(dict(condition=label,payloadSupport="four terminals" if route==0 else "nearest three terminals",
                                  frictionCoefficient=mu,floorYN=floor,distribution=dist,linearAccelerationMps2=acc.tolist(),
                                  yawAccelerationRadS2=alpha,freeWrenchResidual=equilibrium.tolist()))
                forces.append(vector)
    loads=np.column_stack(forces)
    # Gauge freedoms pin an unloaded free-body coordinate frame only, after full applied
    # force/moment equilibrium. Solving must recover zero force at every gauge constraint.
    gauge=np.array([module0,module0+1,module0+2])
    free=np.setdiff1d(np.arange(count),gauge)
    study=[]
    for kr in (3000.,30000.):
        for kp in (3000.,30000.):
            for km in (3000.,30000.):
                ks=(kr,kp,km);K=model["base"]+sum(k*p for k,p in zip(ks,model["stiffnessParts"]))
                u=np.zeros_like(loads)
                u[free]=np.linalg.solve(K[np.ix_(free,free)],loads[free])
                remainder=(K@u-loads)[gauge]
                if abs(remainder[:2]).max()>.025 or abs(remainder[2]).max()>12:
                    raise ValueError(f"Spurious gauge force/couple {abs(remainder).max(axis=1)}")
                ring=[]
                for i,(rec,dofs) in enumerate(zip(model["ringMembers"],model["ringDofs"])):
                    nxt=(i+1)%len(nodes);q=rec@u[dofs]
                    for end,axial,moment in ((i,0,2),(nxt,3,5)):
                        area,inertia,c=model.get("ringSection",section)(nodes[end],nodes[nxt]-nodes[i])
                        ring.append((abs(q[axial])/area+abs(q[moment])*c/inertia,nodes[end].tolist()))
                lower=[]
                for label,stations,rec,dofs,a,inertia in model["lowerMembers"]:
                    q=rec@u[dofs]
                    lower.append((abs(q[0])/a+np.maximum(abs(q[2]),abs(q[5]))*12.7/inertia,[label,stations]))
                metrics={"ringNetMPa":ring,"lowerGrossMPa":lower,
                         "pilotN":[(np.linalg.norm(kp*(j@u),axis=0),i) for i,j in enumerate(model["pilots"])],
                         "moduleScrewN":[(np.linalg.norm(km*(j@u),axis=0),model["anchorXY"][i].tolist()) for i,j in enumerate(model["anchorJac"])],
                         "braceN":[(abs(k*(j@u)),i) for i,(k,j) in enumerate(model["diagonals"])]}
                metrics={k:v for k,v in metrics.items() if v}
                scan={}
                for condition in ("free strike","free strike without floor traction","opposed aligned same-strength pinch"):
                    if condition=="free strike without floor traction":
                        ids=[i for i,c in enumerate(cases) if c["condition"]=="free strike" and c["frictionCoefficient"]==0]
                    else:
                        ids=[i for i,c in enumerate(cases) if c["condition"]==condition]
                    sample={}
                    for name,values in metrics.items():
                        mat=np.array([v[0] for v in values])[:,ids]
                        i,c=np.unravel_index(np.argmax(mat),mat.shape)
                        sample[name]=dict(value=float(mat[i,c]),physicalSite=values[i][1],case=cases[ids[c]])
                    scan[condition]=sample
                study.append(dict(ringPilotModuleStiffnessNPerMm=ks,peak=scan,
                                  maxGaugeReactionN=float(abs(remainder[:2]).max()),maxGaugeMomentNmm=float(abs(remainder[2]).max())))
    return dict(authority=model["authority"],massKg=required_mass,cgMm=required_cg.tolist(),yawAboutCGKgM2=required_yaw,
        unconstrainedEffectiveMassKg=1/(1/required_mass+((109-required_cg[0])*.001)**2/required_yaw),
        upperPeakN=peak,contactXYmm=[109.,impact_y],opposedXYmm=[109.,-impact_y],workingStressMPa=120,
        reserveClosure=dict(kg=reserve["kg"],xyMm=reserve["cg"].tolist(),radiusOfGyrationMm=reserve_radius),
        assumptions="Planar peak equivalent-static comparison. Rigid-body acceleration is solved independently for each actual external wrench. Both released translation and yaw and all finite mounted body offsets are included. Original 18 mm full-speed peak is held fixed even for the off-centre hit; angular energy absorption may lower it. Friction extremes are parallel and antiparallel to the collision direction. Independently applying the same external load at an opposite bumper models an aligned two-sided pinch; that scenario has not been established as routine field duty. The free-body jig is a gauge; its zero reactions are checked. Local finite elastic waves, backing contact dynamics, sprung mass pitch/roll, true partially assembled deck support and traction distribution need validation.",
        loadCount=len(cases),parts=[dict(name=r["name"],kg=r["kg"],xyMm=r["cg"].tolist(),cgYawKgM2=r["j"],
             candidateAttachmentDOFs=r["groups"]) for r in parts if not r["name"].startswith(("Ring ","Removable bumper "))],
        sourceHashes={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in
                    ["inspection-mass-552.json","terminal-mass-clock.json","corner-tie-trial.json","socket-holes.json","diagonal_lower_elastic.py"]},
        states=study)


if __name__=="__main__":
    data=run(Path(sys.argv[1]))
    (HERE/"diagonal-inertia-relief.json").write_text(json.dumps(data,indent=2)+"\n")
    for label in data["states"][0]["peak"]:
        print(label,{k:round(max(st["peak"][label][k]["value"] for st in data["states"]),2) for k in data["states"][0]["peak"][label]})
    print(data["reserveClosure"])
