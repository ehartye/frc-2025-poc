"""Resolve the perimeter reactions and peak section stress over finite and concentrated bumper contacts."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from perimeter_elastic import corners

BASE = Path(__file__).parent


def structure(mesh_mm, section_at=None, feature_offsets=(), outline_xy=None):
    outline = np.array(corners() if outline_xy is None else outline_xy)
    nodes, faces, supports = [], [], {}
    for face, (start, end) in enumerate(zip(outline, np.roll(outline, -1, axis=0))):
        delta = end - start
        length = np.linalg.norm(delta)
        tangent = delta / length
        # Polygon traverses CCW. Inward left normal is positive for the first edge at -Y.
        normal = np.array([-tangent[1], tangent[0]])
        breaks = [0., length]
        support_locations = []
        axis = int(np.argmax(np.abs(delta)))
        if min(abs(delta)) < .001:
            for station in (-100., 0., 100.):
                s = (station - start[axis]) / tangent[axis]
                support_locations.append(s)
                breaks.extend(s+off for off in feature_offsets)
        breaks = sorted(set(breaks + support_locations))
        locations = []
        for a, b in zip(sorted(breaks), sorted(breaks)[1:]):
            count = math.ceil((b-a)/mesh_mm)
            locations.extend(np.linspace(a,b,count,endpoint=False))
        ids = []
        for s in locations:
            ids.append(len(nodes))
            point = start + s * tangent
            nodes.append(point)
            if any(abs(s-k) < 1e-8 for k in support_locations):
                supports[ids[-1]] = (face, point)
        faces.append((ids, locations+[length], normal))
    for i, (ids, locations, normal) in enumerate(faces):
        ids.append(faces[(i+1)%len(faces)][0][0])

    nodes = np.array(nodes)
    gross_area = 25.4*101.6 - 19.4*95.6
    gross_inertia = (101.6*25.4**3 - 95.6*19.4**3)/12
    E = 68900.
    indices, rows, cols, values, ends = [], [], [], [], []
    for i, (a,b) in enumerate(zip(nodes,np.roll(nodes,-1,axis=0))):
        L = np.linalg.norm(b-a)
        c,s = (b-a)/L
        area, inertia = (section_at((a+b)/2,b-a)[:2] if section_at else (gross_area,gross_inertia))
        K = E*np.array([[area/L,0,0,-area/L,0,0],[0,12*inertia/L**3,6*inertia/L**2,0,-12*inertia/L**3,6*inertia/L**2],[0,6*inertia/L**2,4*inertia/L,0,-6*inertia/L**2,2*inertia/L],[-area/L,0,0,area/L,0,0],[0,-12*inertia/L**3,-6*inertia/L**2,0,12*inertia/L**3,-6*inertia/L**2],[0,6*inertia/L**2,2*inertia/L,0,-6*inertia/L**2,4*inertia/L]])
        tr=np.zeros((6,6));tr[:3,:3]=tr[3:,3:]=[[c,s,0],[-s,c,0],[0,0,1]]
        dofs=list(range(3*i,3*i+3))+[3*((i+1)%len(nodes))+j for j in range(3)]
        rr,cc=np.meshgrid(dofs,dofs,indexing='ij')
        rows.extend(rr.ravel());cols.extend(cc.ravel());values.extend((tr.T@K@tr).ravel())
        ends.append(K@tr);indices.append(dofs)
    matrix=np.zeros((3*len(nodes),3*len(nodes)))
    np.add.at(matrix,(rows,cols),values)
    return nodes, faces, supports, matrix, ends, indices, gross_area, gross_inertia


def cases_and_forces(nodes, faces):
    cases=[];loads=[]
    for face,(ids,coords,normal) in enumerate(faces):
        coords=np.array(coords)
        boundaries=np.r_[0,.5*(coords[1:]+coords[:-1]),coords[-1]]
        for j,center in enumerate(coords):
            for width,shape in ((0,'point'),(100,'flat'),(100,'triangular')):
                if width and (center < width/2 or center > coords[-1]-width/2):
                    continue  # only include full 100 mm contact on ONE face
                if width:
                    x=np.clip((boundaries-center)/(width/2),-1,1)
                    cdf=(x+1)/2 if shape=='flat' else np.where(x<0,.5*(1+x)**2,1-.5*(1-x)**2)
                    fraction=np.diff(cdf)
                else:
                    fraction=np.zeros(len(ids));fraction[j]=1
                if abs(fraction.sum()-1)>1e-8:raise ValueError('Unbalanced patch')
                load=np.zeros(len(nodes)*3)
                load[3*np.array(ids)]=fraction*normal[0]
                load[3*np.array(ids)+1]=fraction*normal[1]
                loads.append(load)
                cases.append(dict(face=face,atMm=nodes[ids[j]].tolist(),widthMm=width,profile=shape))
    return np.array(loads).T,cases


def run(mesh_mm, peak):
    nodes,faces,anchors,stiff,ends,indices,area,inertia=structure(mesh_mm)
    F,cases=cases_and_forces(nodes,faces)
    groups={s:np.array([i for i,c in enumerate(cases) if c['profile']==s]) for s in ('point','flat','triangular')}
    worst={}
    for k in (3000.,30000.):
        all_dofs=np.array([3*i+j for i in anchors for j in (0,1)])
        K=stiff.copy(); K[all_dofs,all_dofs]+=k
        basis=np.eye(len(K))[:,all_dofs]
        solution=np.linalg.solve(K,np.column_stack((peak*F,basis)))
        intact=solution[:,:F.shape[1]]; influence=solution[:,F.shape[1]:]
        for absent in [None,*anchors]:
            active=[i for i in anchors if i!=absent]
            dofs=np.array([3*i+j for i in active for j in (0,1)])
            if absent is None:
                u=intact
            else:
                removed=np.array([3*absent,3*absent+1])
                g=influence[:,np.isin(all_dofs,removed)]
                correction=np.linalg.solve(np.eye(2)/k-g[removed],intact[removed])
                u=intact+g@correction
            summed=(k*u[dofs]).reshape(len(active),2,-1).sum(axis=0)
            resultant=peak*F.reshape(len(nodes),3,-1).sum(axis=0)[:2]
            if np.max(abs(resultant-summed))>.005:raise ValueError('Unbalanced forces')
            stress=np.zeros(len(cases))
            for matrix,d in zip(ends,indices):
                effort=matrix@u[d]
                stress=np.maximum(stress,abs(effort[0])/area+np.maximum(abs(effort[2]),abs(effort[5]))*12.7/inertia)
            for shape,idx in groups.items():
                highest=idx[np.argmax(stress[idx])]
                key=f"{shape}/{'intact' if absent is None else 'one-lost'}"
                if stress[highest] > worst.get(key,{}).get('stressMPa',0):
                    worst[key]=dict(stressMPa=float(stress[highest]),case=cases[highest],kNPerMm=k,absentMm=None if absent is None else nodes[absent].tolist())
    return dict(meshMm=mesh_mm,members=len(nodes),uniqueContacts=len(cases),supportCount=len(anchors),worst=worst)


if __name__=='__main__':
    duty=json.loads((BASE/'bumper-duty.json').read_bytes())
    peak=duty['upper']['peakN']
    results=[run(step,peak) for step in (12.,6.)]
    result=dict(authority=json.loads((BASE/'full-height-perimeter.json').read_bytes())['authority'],
        bumperDutySha256=hashlib.sha256((BASE/'bumper-duty.json').read_bytes()).hexdigest(),
        upperN=peak,routineN=duty['routine']['quasistaticInterpolatedPeakN'],runs=results,
        assumptions='Planar nominal miter rigidity; 3 mm minimum walls. Point load samples span each entire face, including exact support and corner positions. For each wide-face case the normal force occupies 100 mm on that face with either uniform or triangular pressure. Those pressure distributions are hypotheses for the broad 100 mm contact; a narrow field/robot corner can give a concentrated load. Combined two-face contact, net sections at holes, bracket rotation, splice rigidity, material damage and the physical bumper do not acquire a pass here. Routine force scales these linear static stresses by routineN / upperN.')
    (BASE/'perimeter-footprint.json').write_text(json.dumps(result,indent=2)+'\n')
    for r in results:print(r['meshMm'],[(key,round(row['stressMPa'],3)) for key,row in r['worst'].items()])
