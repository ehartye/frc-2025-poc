"""Infer a centered transverse pipe envelope solely from registered grayscale stereo frames."""
import math
import numpy as np
from imaging import W,H,F,C,S,R,HALF,ORIGIN,EYES


def read_pair(pair, empty, height, stroke, *, pixel_error=1.0):
    # All positions use the registered, stationary pose. Never feed port states or simulated pipe state here.
    mask=[np.abs(img.astype(int)-ref.astype(int))>36 for img,ref in zip(pair,empty)]
    rows=[]
    for v in range(2,H-2):
        edges=[]
        for m in mask:
            pixels=np.flatnonzero(m[v])
            if len(pixels)<35: break
            splits=np.flatnonzero(np.diff(pixels)>2)
            if len(splits): break
            edges.append((pixels[0]+.5,pixels[-1]+.5))
        if len(edges)==2: rows.append((v+.5,edges))
    if len(rows)<30: return {"status":"insufficient-stereo-contour","scanlines":len(rows)}
    # Use widely spaced rows; expect both corresponding outer edges in each eye.
    sampled=[rows[i] for i in np.linspace(3,len(rows)-4,12).astype(int)]
    if max(b[0]-a[0] for a,b in zip(sampled,sampled[1:]))>18:
        return {"status":"incomplete-contour","scanlines":len(rows)}
    # Obtain a finite search box from measured apparent width, pipe length and radius.
    # This includes objects outside the shelf and airborne pipes.
    edge_tol=pixel_error+.5
    lower=np.array([-np.inf,-np.inf])
    upper=np.array([np.inf,np.inf])
    for v,edges in sampled:
        left,right=edges[0]
        depths=[F*(2*HALF-.5)/(right-left+2*edge_tol),
                F*(2*HALF+.5)/(right-left-2*edge_tol)]
        surface=[]
        for d in depths:
            for row in (v-pixel_error,v+pixel_error):
                q=(H/2-row)/F
                surface.append([ORIGIN[0]+d*(C+q*S),ORIGIN[2]+d*(-S+q*C)])
        lower=np.maximum(lower,np.min(surface,axis=0)-R-2)
        upper=np.minimum(upper,np.max(surface,axis=0)+R+2)
    if np.any(lower>upper) or np.any(upper-lower>400):
        return {"status":"unbounded-contour"}
    x,cz=np.meshgrid(np.arange(math.floor(lower[0]), math.ceil(upper[0])+1.),
                     np.arange(math.floor(lower[1]), math.ceil(upper[1])+1.))
    u=x-stroke
    z=cz-height+60-R
    xx=x-ORIGIN[0]
    zz=cz-ORIGIN[2]
    valid=np.ones(u.shape,dtype=bool)
    y_low,y_high=-np.inf,np.inf
    # Bound half-pixel quantization, chosen extraction error and one-mm center grid cells.
    # For an external rim, the stripe width in a rectified row is f*L/depth.
    # Unknown width tolerance, grid error and radius bias are enclosed by an inflated/deflated near surface.
    depth_max=float((xx*C-zz*S).max()+R+2)
    margin=.5+math.sqrt(.5)+depth_max*pixel_error/F
    outer=R+margin
    inner=R-margin
    for v,edges in sampled:
        q=(H/2-v)/F
        dx,dz=C+q*S,-S+q*C
        norm=dx*dx+dz*dz
        dot=xx*dx+zz*dz
        perpendicular=(xx*dz-zz*dx)**2
        disc_outer=norm*outer**2-perpendicular
        disc_inner=norm*inner**2-perpendicular
        near=(dot-np.sqrt(np.maximum(disc_outer,0)))/norm
        far=(dot-np.sqrt(np.maximum(disc_inner,0)))/norm
        # If only the outer circle is intersected, the candidate can be tangent.
        far=np.where(disc_inner<0,dot/norm+.71,far)
        valid &= (disc_outer>=0)&(near>0)
        for eye,(left,right) in zip(EYES,edges):
            min_width=max(1,right-left-2*edge_tol)
            max_width=right-left+2*edge_tol
            valid &= (near<=F*(2*HALF+.5)/min_width)&(far>=F*(2*HALF-.5)/max_width)
            # End correspondence must agree with the centered lateral limit in both views.
            mid=(left+right)/2-W/2
            ys=np.stack([eye-p*d/F for p in (mid-edge_tol,mid+edge_tol) for d in (near,far)])
            y_low=np.maximum(y_low,ys.min(axis=0))
            y_high=np.minimum(y_high,ys.max(axis=0))
        disparity=(edges[0][0]+edges[0][1]-edges[1][0]-edges[1][1])/2
        if disparity<=2*edge_tol: return {"status":"ambiguous-disparity"}
        valid &= (near<=F*(EYES[0]-EYES[1])/(disparity-2*edge_tol))
        valid &= (far>=F*(EYES[0]-EYES[1])/(disparity+2*edge_tol))
    valid &= (y_low<=y_high)
    if not np.any(valid): return {"status":"inconsistent-stereo-shape","scanlines":len(rows)}
    # Independently surveyed extrinsics/slide pose ±1 mm and half of a 1 mm candidate cell.
    limits=lambda data:[float(data[valid].min()-1.5),float(data[valid].max()+1.5)]
    ub,zb=limits(u),limits(z)
    yb=[float(y_low[valid].min()-1),float(y_high[valid].max()+1)]
    status="position-witness" if (55<=ub[0] and ub[1]<=100 and -12<=zb[0] and zb[1]<=12
                                  and -8<=yb[0]<=yb[1]<=8) else "outside-custody-envelope"
    return {"status":status,"u_interval_mm":ub,"bottom_relative_to_shelf_mm":zb,"y_interval_mm":yb,
            "scanlines":len(rows),"cells":int(valid.sum())}
