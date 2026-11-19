"""Rectified fixed-camera stereo pixels, AprilTag detection and planar pose from known feature heights."""
import math
import numpy as np
import cv2

F=320/math.tan(math.radians(36.5))
C,S=math.cos(math.radians(26)),math.sin(math.radians(26))
EYES=(37.5,-37.5)
SIDE=165.1
D=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
PARAM=cv2.aruco.DetectorParameters()
PARAM.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
PARAM.cornerRefinementWinSize=4
DETECT=cv2.aruco.ArucoDetector(D,PARAM)
def marker(tag):
    return np.pad(cv2.aruco.generateImageMarker(D,tag,96),12,constant_values=245)
MARKS={n:marker(n) for n in (10,11)}
# Pixel centers refer to the image samples. The four black-border edges cross half pixels.
SRC=np.array([[11.5,11.5],[107.5,11.5],[107.5,107.5],[11.5,107.5]],np.float32)
def corners(center):
    return np.array([center+[0,y*SIDE/2,z*SIDE/2] for y,z in [(1,1),(-1,1),(-1,-1),(1,-1)]])
def project(points,pose,eye):
    x,y,yaw=pose
    ct,st=math.cos(yaw),math.sin(yaw)
    dx,dy=points[:,0]-x,points[:,1]-y
    bx=ct*dx+st*dy+315
    by=-st*dx+ct*dy-eye
    bz=points[:,2]-970
    d=bx*C-bz*S
    return np.column_stack((320-F*by/d,240-F*(bx*S+bz*C)/d)),d
def unproject(pixels,z,eye):
    q=(240-pixels[:,1])/F
    depth=(z-970)/(-S+q*C)
    return np.column_stack((-315+depth*(C+q*S),eye-depth*(pixels[:,0]-320)/F))
def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s],[s,c]])
def render(points,pose,eye,bg,depth,tag,foreground):
    uv,d=project(points,pose,eye)
    h=cv2.getPerspectiveTransform(SRC,uv.astype(np.float32))
    roi=cv2.perspectiveTransform(np.array([[[-.5,-.5],[119.5,-.5],[119.5,119.5],[-.5,119.5]]],np.float32),h)[0]
    x0,y0=np.maximum(0,np.floor(roi.min(axis=0)).astype(int)-3)
    x1,y1=np.minimum([640,480],np.ceil(roi.max(axis=0)).astype(int)+4)
    out=bg.copy()
    # Red reef face and panel surround form the field backdrop. Preserve nearer CAD and pipe pixels.
    face=np.array([[points[0,0],y,z] for y,z in [(470,457),(-470,457),(-470,3),(470,3)]])
    fp,fd=project(face,pose,eye)
    matte=np.zeros((480,640),np.uint8)
    cv2.fillConvexPoly(matte,np.rint(fp).astype(np.int32),1)
    front=(matte!=0)&(depth>fd.max()+3)
    out[front]=128
    if x1<=x0 or y1<=y0:return out,uv
    m=h.copy();m[0]-=x0*h[2];m[1]-=y0*h[2]
    scale=4
    zoom=m*scale;zoom[2]=m[2]
    zoom[:2]+=(scale-1)/2*m[2]
    hi=cv2.warpPerspective(MARKS[tag],zoom,((x1-x0)*scale,(y1-y0)*scale),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=128)
    patch=cv2.resize(hi,(x1-x0,y1-y0),interpolation=cv2.INTER_AREA)
    # Front plane optical depth varies across this ROI. Intersect each pixel ray with the known wall.
    v,u=np.mgrid[y0:y1,x0:x1]
    q=(240-v)/F
    yaw=pose[2]
    direction=(C+q*S)*math.cos(yaw)+(u-320)/F*math.sin(yaw)
    eye_x=pose[0]-315*math.cos(yaw)-eye*math.sin(yaw)
    surface=(points[0,0]-eye_x)/direction
    view=(depth[y0:y1,x0:x1]>surface+1)
    dst=out[y0:y1,x0:x1]
    dst[view]=patch[view]
    for b in foreground:
        pts=np.array([[x,y,z] for x in (b[0],b[3]) for y in (b[1],b[4]) for z in (b[2],b[5])])
        pp,_=project(pts,pose,eye)
        cv2.fillConvexPoly(out,cv2.convexHull(np.rint(pp).astype(np.int32)),106)
    return out,uv
def detect(frame):
    quads,ids,_=DETECT.detectMarkers(frame)
    if ids is None:return None
    chosen=[q.reshape(4,2).astype(float) for q,i in zip(quads,ids.flatten()) if i==10]
    if len(chosen)!=1:return None
    # Fit the four white-to-black boundaries from grayscale transitions, excluding corner junctions.
    # The optical centerline of each edge lies halfway between its local inside/outside intensity.
    quad=chosen[0];lines=[]
    samples=np.arange(-2.5,2.51,.125,dtype=np.float32)
    for a,b in zip(quad,np.roll(quad,-1,axis=0)):
        tangent=(b-a)/np.linalg.norm(b-a);normal=np.array([-tangent[1],tangent[0]])
        mid=a+(b-a)*np.linspace(.10,.90,23)[:,None]
        xy=mid[:,None,:]+samples[None,:,None]*normal
        intensity=cv2.remap(frame,xy[:,:,0].astype(np.float32),xy[:,:,1].astype(np.float32),cv2.INTER_LINEAR).astype(float)
        level=(intensity[:,0]+intensity[:,-1])/2
        if np.min(intensity[:,0]-intensity[:,-1])<100:return None
        hits=[]
        for row,l in zip(intensity,level):
            i=np.flatnonzero((row[:-1]>=l)&(row[1:]<l))
            if len(i)!=1:return None
            i=i[0];hits.append(samples[i]+.125*(row[i]-l)/(row[i]-row[i+1]))
        edges=mid+np.array(hits)[:,None]*normal
        centroid=edges.mean(axis=0)
        _,_,vh=np.linalg.svd(edges-centroid,full_matrices=False)
        n=vh[-1];lines.append([*n,-n@centroid])
    result=[]
    for last,edge in zip(np.roll(lines,1,axis=0),lines):
        intersection=np.cross(last,edge);result.append(intersection[:2]/intersection[2])
    return np.array(result)
def estimate(pair,points,eyes=EYES):
    by_eye=np.array([unproject(q,points[:,2],eye) for q,eye in zip(pair,eyes)])
    b=by_eye.mean(axis=0);w=points[:,:2]
    a=b-b.mean(axis=0); target=w-w.mean(axis=0)
    yaw=math.atan2(np.sum(a[:,0]*target[:,1]-a[:,1]*target[:,0]),np.sum(a*target))
    xy=w.mean(axis=0)-rotation(yaw)@b.mean(axis=0)
    pose=np.array([*xy,yaw])
    def predicted(p):
        return np.concatenate([project(points,p,lens)[0].ravel() for lens in eyes])
    observed=np.asarray(pair).ravel()
    for _ in range(3):
        pred=predicted(pose)
        jac=np.stack([(predicted(pose+delta)-pred)/scale for delta,scale in zip(np.diag([.1,.1,.0001]),[.1,.1,.0001])],axis=1)
        pose+=np.linalg.lstsq(jac,observed-pred,rcond=None)[0]
    return pose
def station_frame(pose,ctx,tag=10,ambiguous=False,hide=False,save=None):
    projected=[]; measured=[]; tagquads=[]; detected_eyes=[]; diagnostics=[]; frames=[]
    for n,eye in zip(("left","right"),EYES):
        frame,tag_truth=render(ctx["points"],pose,eye,ctx[n],ctx[n+"_z"],tag,[])
        bolt_pixels,zs=project(ctx["bolts"],pose,eye)
        valid=np.ones(len(bolt_pixels),dtype=bool)
        scale=8
        for j,face in enumerate(ctx["faces"]):
            pp,d=project(face,pose,eye)
            x0,y0=np.floor(pp.min(axis=0)).astype(int)-3
            x1,y1=np.ceil(pp.max(axis=0)).astype(int)+4
            if x0<0 or y0<0 or x1>640 or y1>480:
                valid[j]=False;continue
            region=np.zeros(((y1-y0)*scale,(x1-x0)*scale),np.uint8)
            if not hide:
                pix=(pp-[x0,y0])*scale+(scale-1)/2
                cv2.fillConvexPoly(region,cv2.convexHull(np.rint(pix*16).astype(np.int32)),255,lineType=cv2.LINE_AA,shift=4)
            alpha=cv2.resize(region,(x1-x0,y1-y0),interpolation=cv2.INTER_AREA).astype(float)/255
            alpha*=ctx[n+"_z"][y0:y1,x0:x1]>zs[j]+1
            target=frame[y0:y1,x0:x1]
            target[:]=np.rint((1-alpha)*target+alpha*244).astype(np.uint8)
            if ambiguous and j==1 and x1+10<640:
                q=frame[y0:y1,x0+10:x1+10]
                q[:]=np.rint((1-alpha)*q+alpha*244).astype(np.uint8)
        if save is not None: frames.append(frame)
        obs=detect(frame)
        if obs is not None:
            tagquads.append(obs);detected_eyes.append(eye);diagnostics.append(float(np.max(abs(obs-tag_truth))))
        blobs=find_heads(frame)
        projected.append(bolt_pixels);measured.append(blobs)
    if save is not None:cv2.imwrite(str(save),np.column_stack(frames))
    minimum_eyes = 1 if ctx.get("allowSingleEyeId", False) else 2
    pose_tag=estimate(tagquads,ctx["points"],detected_eyes) if len(tagquads)>=minimum_eyes else None
    if pose_tag is not None:
        reprojection=max(np.max(abs(project(ctx["points"],pose_tag,e)[0]-q)) for e,q in zip(detected_eyes,tagquads))
        if reprojection>.35:pose_tag=None
    return pose_tag,measured,projected,diagnostics

def find_heads(image):
    # Native robot intensity is at most 167 in this lighting fixture. The white tag
    # surround can reach the upper threshold; candidates must also have a compact footprint.
    mask=np.uint8(image>183)
    count,labels,stats,centroids=cv2.connectedComponentsWithStats(mask,8)
    centers=[]
    for x,y,w,h,area in stats[1:]:
        if not (2<=w<=20 and 2<=h<=19 and 4<=area<=220):continue
        if x<=2 or y<=2 or x+w>=638 or y+h>=478:continue
        patch=image[y-2:y+h+2,x-2:x+w+2].astype(float)
        bg=np.median(np.r_[patch[0],patch[-1],patch[:,0],patch[:,-1]])
        weights=np.maximum(0,patch-bg)
        yy,xx=np.indices(patch.shape)
        if weights.sum()>0: centers.append([x-2+np.sum(weights*xx)/weights.sum(),y-2+np.sum(weights*yy)/weights.sum()])
    return centers

def associate(heads,points,prediction):
    pair=[]
    for eye,found in zip(EYES,heads):
        expected,_=project(points,prediction,eye)
        matches=[]
        for pixel in expected:
            nearby=[p for p in found if np.linalg.norm(pixel-p)<13]
            if len(nearby)!=1:return None
            matches.append(nearby[0])
        if np.linalg.norm(np.subtract(*matches))<5:return None
        pair.append(np.array(matches))
    return pair

def interval(pair,points,goal_xy,epsilon=.23):
    # Certified planar outer set for ALL headings. Each observed height plane gives
    # an interval of horizontal lens rays. For every angular bin, w - R*b must agree
    # at each corner and in both eyes. The angular-bin chord bounds the rotation
    # of every projected corner between its bin center and edge.
    centers=np.radians(np.arange(-179.95,180,.1))
    cs=np.cos(centers); ss=np.sin(centers)
    lows=np.full((len(centers),2),-np.inf); highs=-lows.copy()
    patches=[]
    for eye,obs in zip(EYES,pair):
        for i,uv in enumerate(obs):
            xy=unproject(uv+np.array([[-epsilon,-epsilon],[-epsilon,epsilon],[epsilon,-epsilon],[epsilon,epsilon]]),np.full(4,points[i,2]),eye)
            xx,yy=xy.T
            pad=2*np.hypot(xx,yy).max()*math.sin(math.radians(.025))
            patches.append((points[i,:2],xy,pad))
            vx=points[i,0]-cs[:,None]*xx+ss[:,None]*yy
            vy=points[i,1]-ss[:,None]*xx-cs[:,None]*yy
            lows=np.maximum(lows,np.column_stack((vx.min(axis=1),vy.min(axis=1)))-pad)
            highs=np.minimum(highs,np.column_stack((vx.max(axis=1),vy.max(axis=1)))+pad)
    valid=np.all(lows<=highs,axis=1)
    if not np.any(valid):return None
    vertices=[];angles=[]
    # Retain the XY correlations of every ray rather than combining independent extrema.
    # Each fixed-yaw quadrilateral is expanded by the bin's arc displacement in both axes.
    for angle in centers[valid]:
        region=None
        for point,xy,pad in patches:
            square=np.array([[-1,-1],[-1,1],[1,1],[1,-1]])*(pad+.001)
            candidates=(point-xy@rotation(angle).T)[:,None,:]+square
            poly=cv2.convexHull(candidates.reshape(-1,2).astype(np.float32))
            if region is None:region=poly
            else:
                area,region=cv2.intersectConvexConvex(region,poly)
                if area<=0:break
        if region is not None and area>0:
            angles.append(angle);vertices.extend(region[:,0,:])
    if not angles:return None
    vertices=np.asarray(vertices)
    return (np.r_[vertices.min(axis=0)-.003,min(angles)-math.radians(.05)],np.r_[vertices.max(axis=0)+.003,max(angles)+math.radians(.05)],float(np.linalg.norm(vertices-goal_xy,axis=1).max())+.005)
