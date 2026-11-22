"""Screen the four cartridge braces with elastic socket rails and actual commercial plate screw stations."""
import hashlib
import json
from pathlib import Path

import numpy as np

from perimeter_footprint import structure, cases_and_forces
from terminal_net_sections import section

HERE = Path(__file__).resolve().parent
SIDES = ("Front transverse", "Port longitudinal", "Rear transverse", "Starboard longitudinal")
CENTERS = np.array([[333.9375, 0], [0, 328.9375], [-333.9375, 0], [0, -328.9375]])
PILOTS = (-65., -35., 35., 65.)
LIMIT = 120.


def planar_point(size, first, offset):
    result = np.zeros((2, size))
    x, y = offset
    result[:, first:first+3] = [[1., 0., -y], [0., 1., x]]
    return result


def beam(start, end, wall):
    delta = end - start
    length = np.linalg.norm(delta)
    c, s = delta / length
    width, height = 25.4, 50.8
    area = width*height - (width-2*wall)*(height-2*wall)
    inertia = (height*width**3 - (height-2*wall)*(width-2*wall)**3)/12
    ext = 68900*area/length
    flex = 68900*inertia/length**3
    # Axial strain, common transverse translation/rigid rotation, and curvature
    # all enter in the beam-local frame; its world orientation follows the rail.
    k = np.array([[ext,0,0,-ext,0,0],
                  [0,12*flex,6*length*flex,0,-12*flex,6*length*flex],
                  [0,6*length*flex,4*length**2*flex,0,-6*length*flex,2*length**2*flex],
                  [-ext,0,0,ext,0,0],
                  [0,-12*flex,-6*length*flex,0,12*flex,-6*length*flex],
                  [0,6*length*flex,2*length**2*flex,0,-6*length*flex,4*length**2*flex]])
    tr = np.zeros((6,6))
    tr[:3,:3] = tr[3:,3:] = [[c,s,0],[-s,c,0],[0,0,1]]
    return tr.T@k@tr, k@tr, area, inertia


def run(package, free_model=False):
    import sys
    sys.path.insert(0, str(package / "scripts"))
    from fusion_authority import register_path, verify
    root = HERE.parents[3]
    reg = register_path(root)
    authority = verify(root, reg, reg.parent/"issue-005/lineage.json")
    trial = json.loads((HERE/"corner-tie-trial.json").read_bytes())
    if trial["authority"] != authority:
        raise ValueError("The source issue differs from the mass and diagonal study")

    rails = json.loads((HERE/"socket-holes.json").read_bytes())["rails"]
    feature = tuple(x+d for x in (-9,9) for d in (-7.15,-6.55,0,6.55,7.15))
    nodes, faces, mounts, ring, rings, ring_dofs, _, _ = structure(14, section, feature)
    applied, cases = cases_and_forces(nodes, faces)
    n = len(ring)
    count = 10  # four upper pilots plus six original two-wall mounting screws on each rail
    module_start = n+12+3*count*4
    size = module_start + (12 if free_model else 0)
    module_centres = np.array([[282.,277.],[-282.,277.],[-282.,-277.],[282.,-277.]])
    base = np.zeros((size,size))
    base[:n,:n] = ring
    load = np.zeros((size,len(cases)))
    load[:n,:] = applied*trial["upperCaseN"]

    ring_joints = []
    for node, (_, xy) in mounts.items():
        side = np.argmin(np.linalg.norm(CENTERS-xy, axis=1))
        relative = -planar_point(size,n+3*side,xy-CENTERS[side])
        relative[:,3*node:3*node+2] = np.eye(2)
        ring_joints.append(relative)

    pilots, anchors, anchor_xy, members = [], [], [], []
    lower_nodes = []
    for side, label in enumerate(SIDES):
        rail = rails[label+" socket rail"]
        axis = rail["lengthAxis"]
        screw_xy = np.array(rail["boltCentersMm"])
        stations = sorted(set(PILOTS) | set(screw_xy[:,axis]))
        if len(stations) != count:
            raise ValueError("Unexpected mounted stock stations")
        rail_center = screw_xy.mean(axis=0)
        ids = []
        coords = []
        for j, station in enumerate(stations):
            xy = rail_center.copy()
            xy[axis] = station
            first = n+12+3*(count*side+j)
            lower_nodes.append((first,side,xy.copy()))
            ids.append(first)
            coords.append(xy)
            if station in PILOTS:
                actual = CENTERS[side].copy()
                actual[axis] = station
                relative = planar_point(size,n+3*side,actual-CENTERS[side])
                relative -= planar_point(size,first,actual-xy)
                pilots.append(relative)
            else:
                relative = planar_point(size,first,[0,0])
                if free_model:
                    module_id = int(np.argmin(np.linalg.norm(module_centres-xy,axis=1)))
                    relative -= planar_point(size,module_start+3*module_id,xy-module_centres[module_id])
                anchors.append(relative)
                anchor_xy.append(xy)
        for a in range(count-1):
            # Favor the source with the nominal full-wall section. Holes and twist
            # only reduce the certainty of an otherwise acceptable gross screen.
            km, recovery, area, inertia = beam(coords[a],coords[a+1],rail["wallMm"])
            dofs = [ids[a]+j for j in range(3)] + [ids[a+1]+j for j in range(3)]
            base[np.ix_(dofs,dofs)] += km
            members.append((label,stations[a:a+2],recovery,dofs,area,inertia))

    diagonals = []
    for sx, sy in ((1,1),(-1,1),(-1,-1),(1,-1)):
        a = np.array([sx*319.2,sy*65])
        b = np.array([sx*65,sy*314.2])
        f = 0 if sx == 1 else 2
        g = 1 if sy == 1 else 3
        delta = b-a
        length = np.linalg.norm(delta)
        diff = planar_point(size,n+3*g,b-CENTERS[g])-planar_point(size,n+3*f,a-CENTERS[f])
        jac = (delta/length)@diff
        stiffness = 68900*(25.4*50.8-19.4*44.8)/length
        base += stiffness*np.outer(jac,jac)
        diagonals.append((stiffness,jac))
    stiffness_parts = [sum(j.T@j for j in group) for group in (ring_joints,pilots,anchors)]
    if free_model:
        return dict(authority=authority,nodes=nodes,ringSize=n,ringMembers=rings,ringDofs=ring_dofs,
                    base=base,stiffnessParts=stiffness_parts,loadCases=cases,forces=load,sideCenters=CENTERS,
                    moduleStart=module_start,moduleCenters=module_centres,mounts=mounts,lowerNodes=lower_nodes,
                    lowerMembers=members,anchorJac=anchors,anchorXY=anchor_xy,pilots=pilots,diagonals=diagonals)
    results = []
    for k_ring in (3000.,30000.):
        for k_pilot in (3000.,30000.):
            for k_plate in (3000.,30000.):
                ks = (k_ring,k_pilot,k_plate)
                matrix = base + sum(k*part for k,part in zip(ks,stiffness_parts))
                u = np.linalg.solve(matrix,load)
                reactions = np.stack([-k_plate*(j@u) for j in anchors])
                applied_forces = load[:n].reshape(-1,3,len(cases))
                residual = np.max(abs(reactions.sum(0)+applied_forces[:,:2].sum(0)))
                xy = np.array(anchor_xy)
                mz = (xy[:,0,None]*reactions[:,1]-xy[:,1,None]*reactions[:,0]).sum(0)
                mz += (nodes[:,0,None]*applied_forces[:,1]-nodes[:,1,None]*applied_forces[:,0]+applied_forces[:,2]).sum(0)
                if residual>.005 or np.max(abs(mz))>1:
                    raise ValueError("The load path did not equilibrate")
                # Positive module screw force on the rail is opposite to the internal beam's reaction
                # on the screw; the ground force applied to the rail is the value in reactions.
                plate_loads = []
                for rail_id, label in enumerate(SIDES):
                    rail = rails[label+" socket rail"]
                    axis = rail["lengthAxis"]
                    for sign in (-1,1):
                        selected = [i for i in range(6*rail_id,6*(rail_id+1)) if sign*xy[i,axis]>0]
                        vector = reactions[selected].sum(0)
                        for rank, i in enumerate(sorted(selected,key=lambda i:abs(xy[i,axis]))):
                            v = reactions[i]
                            resultant = np.linalg.norm(v,axis=0)
                            col = int(np.argmax(resultant))
                            plate_loads.append(dict(mode=("inner","middle","outer")[rank]+"ScrewN",ratio=float(resultant[col]),
                                rail=label,screwXY=xy[i].tolist(),forceXY=v[:,col].tolist(),case=cases[col]))
                            material_end = np.where(v[axis]>0,rail["maximumMm"][axis]-xy[i,axis],
                                                    xy[i,axis]-rail["minimumMm"][axis])
                            ligament = material_end-2.65-.25-.15-.05
                            util = abs(v[axis])/(2*6*ligament*120/np.sqrt(3))
                            col = int(np.argmax(util))
                            plate_loads.append(dict(mode="axialTwoWallTearout",ratio=float(util[col]),screwXY=xy[i].tolist(),
                                         forceXY=v[:,col].tolist(),rail=label,case=cases[col],ligamentMm=float(ligament[col])))
                        for key, row in (("axial",vector[axis]),("radial",vector[1-axis])):
                            col = int(np.argmax(abs(row)))
                            plate_loads.append(dict(mode=key+"EndGroupN",ratio=float(abs(row[col])),rail=label,endSign=sign,
                                groupXYN=vector[:,col].tolist(),screwsXYN=reactions[selected,:,col].tolist(),case=cases[col]))
                plate_records = {key:max((v for v in plate_loads if v["mode"]==key),key=lambda v:v["ratio"])
                                 for key in ("axialTwoWallTearout","axialEndGroupN","radialEndGroupN","innerScrewN","middleScrewN","outerScrewN")}
                ring_stresses = []
                for node,(recover,dofs) in enumerate(zip(rings,ring_dofs)):
                    nxt=(node+1)%len(nodes)
                    effort=recover@u[dofs]
                    for which,(idx,axial_i,bend_i) in enumerate(((node,0,2),(nxt,3,5))):
                        area,inertia,extreme=section(nodes[idx],nodes[nxt]-nodes[node])
                        stress=abs(effort[axial_i])/area+abs(effort[bend_i])*extreme/inertia
                        col=int(np.argmax(stress))
                        ring_stresses.append(dict(atMm=nodes[idx].tolist(),case=cases[col],stressMPa=float(stress[col])))
                stressed = []
                for label, stations, rec, dofs, area, inertia in members:
                    force = rec@u[dofs]
                    stress = abs(force[0])/area + np.maximum(abs(force[2]),abs(force[5]))*12.7/inertia
                    col = np.argmax(stress)
                    stressed.append(dict(rail=label,spanMm=stations,stressMPa=float(stress[col]),case=cases[col],
                                         endMomentsNm=(force[[2,5],col]/1000).tolist(),axialN=float(force[0,col])))
                pilot_force = np.stack([k_pilot*(j@u) for j in pilots])
                peak = np.linalg.norm(pilot_force,axis=1)
                i,col = np.unravel_index(np.argmax(peak),peak.shape)
                results.append(dict(kRingPilotPlateNPerMm=ks,lowerGross=max(stressed,key=lambda v:v["stressMPa"]),
                                    highestPrimaryPilotN=float(peak[i,col]),governingPilot=int(i),pilotCase=cases[col],
                                    ringNet=max(ring_stresses,key=lambda v:v["stressMPa"]),plateBounds=plate_records,highestModuleScrewN=float(np.linalg.norm(reactions,axis=1).max()),
                                    highestDiagonalN=max(float(abs(k*(j@u)).max()) for k,j in diagonals),
                                    maxForceResidualN=float(residual),maxMomentResidualNmm=float(np.max(abs(mz)))))
    return dict(authority=authority,upperCaseN=trial["upperCaseN"],workingStressMPa=LIMIT,distinctLoads=len(cases),
                signedInputFiles={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in
                    ["corner-tie-trial.json","socket-holes.json","terminal_net_sections.py","perimeter_footprint.py"]},
                corners="Two genuine three-screw socket connections per rail, both walls, ground in translation through independent stiffness. The module plate attachment is the assumed reaction for this conservative stopped-frame screen.",
                scope="Gross 6061-T6 linear planar lower rail strain, including finite in-plane cartridge rotation and actual axial locations with the upper pilots offset from the lower neutral axis. Working limit 120 MPa. Plate deformation, tire or inertial redistribution, holes, out-of-plane twist, finite upper stock bending, miter stiffness, tube-wall handling, preload and fatigue need independent qualification. Assumed reaction stiffness spans 3–30 kN/mm in each group. The analysis covers intact hardware only.",
                states=results)


if __name__ == "__main__":
    import sys
    result=run(Path(sys.argv[1]))
    (HERE/"diagonal-lower-elastic.json").write_text(json.dumps(result,indent=2)+"\n")
    for condition in result["states"]:
        print(condition["kRingPilotPlateNPerMm"],round(condition["lowerGross"]["stressMPa"],2),
              round(condition["highestPrimaryPilotN"]),round(condition["highestModuleScrewN"]))
