"""Overhead moving stereo head with a pipe silhouette measured against robot-backed rays."""
import math
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"stereo-custody"))
import imaging as camera
F,W,H=camera.F,camera.W,camera.H
R,L=camera.R,2*camera.HALF
ORIGIN=np.array([98.,0.,842.])
EYES=camera.EYES

def configure():
    camera.ORIGIN=ORIGIN.copy()
    camera.S,camera.C=1.,0.
    camera.DX,camera.DY,camera.DZ=camera.Q,-(camera.U-W/2)/F,-np.ones_like(camera.Q)


def fit(images,background,error=1.0):
    # Image-plane samples compare only with known shelf-backed rays. Unknown field pixels cannot supply edges.
    cloud=[]
    masks=[]
    yaw_lo,yaw_hi=-math.inf,math.inf
    epsilon=.5+error
    for eye,(frame,(gray,depth)) in enumerate(zip(images,background)):
        known=depth<3490
        changed=np.abs(frame.astype(int)-gray.astype(int))>36
        masks.append((known,changed))
        column=W//2
        material=np.flatnonzero(known[:,column]&changed[:,column])
        if len(material)<35 or np.any(np.diff(material)!=1):return {"status":"missing-crown"}
        top,bottom=material[0],material[-1]+1
        if top<3 or bottom>H-3 or not np.all(known[top-2:bottom+2,column]):return {"status":"unbacked-crown"}
        # At a transverse pipe end, the crown is nearer than either flank. A flat rectangular
        # background patch with matching overall image extents cannot supply this curved contour.
        mid=(top+bottom-1)/2
        span=.65*(bottom-top-1)/2
        outline=[]
        for row in (round(mid-span),round(mid),round(mid+span)):
            cols=np.flatnonzero(known[row]&changed[row])
            near=cols[cols<W/2] if eye==0 else cols[cols>W/2]
            if len(near)<20 or np.any(np.diff(near)>2):return {"status":"missing-end-arc"}
            edge=int(near[0] if eye==0 else near[-1])
            if not np.all(known[row,edge-3:edge+4]):return {"status":"unbacked-end-arc"}
            outline.append(edge)
        curvature=(1 if eye==0 else -1)*((outline[0]+outline[2])/2-outline[1])
        if curvature<=2*epsilon+1:return {"status":"unresolved-end-curvature"}
        axis_pairs=0
        for half_span in (119,110,105):
            side_edges=[]
            for lateral_column in (column-half_span,column+half_span):
                hit=np.flatnonzero(known[:,lateral_column]&changed[:,lateral_column])
                if len(hit)<35 or np.any(np.diff(hit)!=1):break
                if not np.all(known[hit[0]-2:hit[-1]+3,lateral_column]):break
                side_edges.append([hit[0],hit[-1]+1])
            if len(side_edges)!=2:continue
            axis_pairs+=1
            # Tangent rulings of a horizontal cylinder have constant Z; their image slopes equal tan(yaw).
            for left,right in zip(*side_edges):
                yaw_lo=max(yaw_lo,math.degrees(math.atan((right-left-2*epsilon)/(2*half_span))))
                yaw_hi=min(yaw_hi,math.degrees(math.atan((right-left+2*epsilon)/(2*half_span))))
        if not axis_pairs:return {"status":"unbacked-axis"}
        if yaw_lo>yaw_hi:return {"status":"inconsistent-axis"}
        for a in (top-epsilon,top+epsilon):
            for b in (bottom-epsilon,bottom+epsilon):
                t,s=math.atan((H/2-a)/F),math.atan((H/2-b)/F)
                alpha,beta=(t+s)/2,(t-s)/2
                for radius in (R-.5,R+.5+.04):
                    dist=radius/math.sin(beta)
                    cloud.append([ORIGIN[0]+dist*math.sin(alpha),ORIGIN[2]-dist*math.cos(alpha)])
    if yaw_lo < -2 or yaw_hi > 2:return {"status":"unqualified-axis"}
    points=np.array(cloud)
    # ±2 deg transverse-axis yaw is independently checked below. Central columns sample each eye Y.
    dx=1+math.tan(math.radians(2))*(max(map(abs,EYES))+8+1)
    x0,z0=points.min(axis=0)-[dx,1]
    x1,z1=points.max(axis=0)+[dx,1]
    # Observe the two nearest cylinder ends in independently backed rows at the crown. Both inferred
    # lateral centres and their end positions must agree; a clipped end cannot substitute for an edge.
    x=(x0+x1)/2
    z=(z0+z1)/2
    row=round(H/2-(x-ORIGIN[0])*F/(ORIGIN[2]-(z+R))-.5)
    lateral=[]
    for eye,((known,changed),origin_y) in enumerate(zip(masks,EYES)):
        cols=np.flatnonzero(changed[row]&known[row])
        near=cols[cols<W/2] if eye==0 else cols[cols>W/2]
        if len(near)<20:return {"status":"missing-end"}
        col=int(near[0] if eye==0 else near[-1]+1)
        if not np.all(known[row,col-3:col+3]):return {"status":"unbacked-end"}
        if np.any(np.diff(near)>2):return {"status":"ambiguous-end"}
        endpoints=[]
        # Bound crown curvature over 12 mm longitudinal offset including yaw and edge uncertainty.
        rise=math.sqrt((R-.5)**2-12**2)
        for d in (ORIGIN[2]-z1-R-.5,ORIGIN[2]-z0-rise):
            for u in (col-epsilon,col+epsilon):
                for half in ((L-.5)/2*math.cos(math.radians(2)),(L+.5)/2):
                    endpoints.append(origin_y-(u-W/2)*d/F-(half if eye==0 else -half))
        lateral.append([min(endpoints)-1,max(endpoints)+1])
    y0=max(v[0] for v in lateral)
    y1=min(v[1] for v in lateral)
    if y0>y1:return {"status":"inconsistent-ends"}
    bottom=[z0-R-500,z1-R-500]
    status="inside" if 55<=x0<=x1<=100 and -8<=y0<=y1<=8 and -12<=bottom[0]<=bottom[1]<=12 else "outside"
    return {"status":status,"x_mm":[x0,x1],"y_mm":[y0,y1],"bottom_mm":bottom}


def moving_mesh():
    sys.path.insert(0,str(ROOT/"low-observation"))
    from geometry import pose_layers,original
    layers,groups,colors,frames=pose_layers(560,0,"red")
    parts=original.flatten(layers)
    from mount import parts as bridge_parts
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    kit,_,bores=bridge_parts()
    for bore in bores:
        parts["receiver-R6-ported"]=BRepAlgoAPI_Cut(parts["receiver-R6-ported"],original.native.brep(bore)).Shape()
    meshes=[]
    for k,shape in sorted(parts.items()):
        if frames[k]!="slide":continue
        v,tri=original.triangulate(shape)
        meshes.append([k,groups[k],colors[k],v,tri])
    for name,shape in sorted(kit.items()):
        if name=="camera-envelope":continue
        verts,tri=original.triangulate(shape)
        meshes.append([name,"Bridge","#737d85",verts,tri])
    return {"parts":meshes}


if __name__=="__main__":
    import argparse
    p=argparse.ArgumentParser()
    p.add_argument("--background",type=Path,required=True)
    args=p.parse_args()
    configure()
    if not args.background.exists():
        bg=camera.images_for_pose(moving_mesh())
        np.savez_compressed(args.background,left=bg[0][0],left_z=bg[0][1],right=bg[1][0],right_z=bg[1][1])
    file=np.load(args.background)
    bg=[(file[k],file[k+"_z"]) for k in ("left","right")]
    for x,yaw in [(68.58,-1.15),(75,0),(80,0),(91.42,1.15)]:
        pair=camera.exposure(bg,[x,0,557.15],yaw)
        print(x,yaw,fit(pair,bg))
