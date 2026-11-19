"""Fit both transverse end extrema and the rear tangent where the front tangent lacks a known backdrop."""
import itertools
import math
import numpy as np
from design import optics as o

full_fit=o.fit


def separated_by_background(indices, known, max_gap, allow_unknown_gaps):
    # Unknown background portions between actually observed bright pixels cannot
    # establish a break in a convex, single-object silhouette. Keep every
    # certified outer-edge/pixel-neighborhood check in the geometric fit.
    for before, after in zip(indices[:-1], indices[1:]):
        if after-before > max_gap and (not allow_unknown_gaps or np.any(known[before+1:after])):
            return True
    return False


def partial_fit(images,background,error=1.0, *, allow_unknown_gaps=False):
    eps=error+.5
    peaks=[]
    tangents=[]
    yaw_lo,yaw_hi=-math.inf,math.inf
    for eye,(im,(gray,depth)) in enumerate(zip(images,background)):
        known=depth<3490
        signal=(np.abs(im.astype(int)-gray.astype(int))>36)&known
        # Trace one supported, continuous pipe end. The lateral extremum must be bracketed by
        # visible curvature in the same connected run; a straight-sided patch is insufficient.
        ends=[]
        for row in range(3,o.H-3):
            hit=np.flatnonzero(signal[row])
            near=hit[hit<o.W//2] if eye==0 else hit[hit>o.W//2]
            if len(near)<20 or separated_by_background(near, known[row], 2, allow_unknown_gaps):continue
            col=int(near[0] if eye==0 else near[-1]+1)
            if col<4 or col>o.W-4 or not np.all(known[row,col-3:col+3]):continue
            ends.append((row,abs(col-o.W/2)))
        if not ends:return {"status":"unbacked-end"}
        peak=max(v for _,v in ends)
        top=next(i for i,(_,v) in enumerate(ends) if v==peak)
        bottom=max(i for i,(_,v) in enumerate(ends) if v==peak)
        edge=dict(ends)
        row=(ends[top][0]+ends[bottom][0])//2
        left=[r for r,v in ends if r<row and peak-v>2*eps]
        right=[r for r,v in ends if r>row and peak-v>2*eps]
        if not left or not right:return {"status":"unresolved-end-curvature"}
        first,last=max(left),min(right)
        # Perspective preserves convexity. The outer edge of a solid cylinder's projected end
        # is concave as a function of image row. Each observed secant extended beyond either
        # endpoint bounds the unknown edge from above. Bound the continuous peak even across
        # a gap in the robot-backed pixels instead of treating unknown background as empty.
        q=np.arange(first,last+1)
        upper=np.full(len(q),np.inf)
        for stride in (12,24,36):
            for r0,v0 in ends:
                r1=r0+stride
                if r1 not in edge:continue
                v1=edge[r1]
                right_slope=(v1-v0+2*eps)/stride
                left_slope=(v1-v0-2*eps)/stride
                before=q+.5<=r0
                after=q-.5>=r1
                upper[before]=np.minimum(upper[before],v0+eps+(q[before]-r0)*left_slope+.5*abs(left_slope))
                upper[after]=np.minimum(upper[after],v1+eps+(q[after]-r1)*right_slope+.5*abs(right_slope))
        maximum=float(max(upper))
        if not np.isfinite(maximum) or maximum<peak-eps:return {"status":"unbounded-end"}
        peaks.append(((peak-eps)/o.F,maximum/o.F))
        rear={}
        center=o.W//2
        spans=(105,110,119,130,140)
        for col in [center,*[center+s*side for s in spans for side in (-1,1)]]:
            y=np.flatnonzero(signal[:,col])
            if len(y)<35 or separated_by_background(y, known[:, col], 1, allow_unknown_gaps):continue
            end=y[-1]+1
            if end>=o.H-3 or not np.all(known[end-3:end+3,col]):continue
            rear[col]=end
        valid=[s for s in spans if center+s in rear and center-s in rear]
        if not valid or center not in rear:return {"status":"missing-rear-edge"}
        for span in valid:
            delta=rear[center+span]-rear[center-span]
            yaw_lo=max(yaw_lo,math.degrees(math.atan((delta-2*eps)/(2*span))))
            yaw_hi=min(yaw_hi,math.degrees(math.atan((delta+2*eps)/(2*span))))
        tangents.append([(o.H/2-rear[o.W//2]-eps)/o.F,(o.H/2-rear[o.W//2]+eps)/o.F])
    if yaw_lo>yaw_hi or yaw_lo < -2 or yaw_hi > 2:return {"status":"unqualified-axis"}
    # Sum the two visible extreme-ray tangency equations to remove the unknown lateral center.
    # Independent interval endpoints intentionally overbound the small yaw and radius terms.
    sinmax=math.sin(math.radians(max(abs(yaw_lo),abs(yaw_hi))))
    cosmin=math.sqrt(1-sinmax*sinmax)
    baseline=o.EYES[0]-o.EYES[1]
    rlo,rhi=o.R-.5,o.R+.5
    ilo,ihi=peaks[0];jlo,jhi=peaks[1]
    # D = (L cos(yaw)-baseline)/(k1+k2) + R*sum(hypot(k,sin(yaw)))/sum(k).
    d0=((o.L-.5)*cosmin-baseline)/(ihi+jhi)+rlo
    d1=(o.L+.5-baseline)/(ilo+jlo)+rhi*max(math.hypot(1,sinmax/ilo),math.hypot(1,sinmax/jlo))
    ylim=[]
    for eye,((k0,k1),ey) in enumerate(zip(peaks,o.EYES)):
        # k(D-R) minus the small correction from the horizontal component of the end circle.
        low=k0*(d0-rhi)-rhi*(math.hypot(k0,sinmax)-k0)-(o.L+.5)/2
        high=k1*(d1-rlo)-(o.L-.5)*cosmin/2
        ylim.append((ey+low,ey+high) if eye==0 else (ey-high,ey-low))
    y0=max(v[0] for v in ylim)-1
    y1=min(v[1] for v in ylim)+1
    if y0>y1:return {"status":"inconsistent-ends"}
    xrange=[]
    for (q0,q1),ey in zip(tangents,o.EYES):
        qr=[q*d for q,d in itertools.product((q0,q1),(d0,d1))]
        lateral=sinmax/cosmin*max(abs(ey-y0),abs(ey-y1))
        off0=rlo*math.hypot(1,max(q0,-q1,0))
        off1=rhi*math.hypot(1/cosmin,max(abs(q0),abs(q1)))
        xrange.append((o.ORIGIN[0]+min(qr)+off0-lateral-1,o.ORIGIN[0]+max(qr)+off1+lateral+1))
    x0=max(v[0] for v in xrange)
    x1=min(v[1] for v in xrange)
    if x0>x1:return {"status":"inconsistent-rear"}
    bottom=[o.ORIGIN[2]-d1-rhi-501,o.ORIGIN[2]-d0-rlo-499]
    inside=55<=x0<=x1<=100 and -8<=y0<=y1<=8 and -12<=bottom[0]<=bottom[1]<=12
    return {"status":"inside" if inside else "outside","x_mm":[x0,x1],"y_mm":[y0,y1],"bottom_mm":bottom}


_registered={}

def registration_scores(images,background):
    cache_key=tuple(id(gray) for gray,depth in background)
    if cache_key not in _registered:
        witnesses=[]
        for eye,(gray,depth) in enumerate(background):
            y,x=np.indices(gray.shape)
            for roi in ((y>310)&(x<300),(y>310)&(x>340)):
                group=[]
                for axis in (0,1):
                    g=gray.astype(int)
                    delta=np.roll(g,-2,axis)-np.roll(g,2,axis)
                    use=roi & (x>8)&(x<o.W-8)&(y>8)&(y<o.H-8)&(depth<3490)
                    use&=(np.roll(depth,-2,axis)<3490)&(np.roll(depth,2,axis)<3490)&(abs(delta)>=8)
                    ys,xs=np.where(use)
                    # Uniform decimation preserves the two separated physical patches in each eye.
                    ys,xs=ys[::5],xs[::5]
                    group.append((axis,ys,xs,np.sign(delta[ys,xs])))
                witnesses.append((eye,group))
        _registered[cache_key]=(background,witnesses)
    scores=[]
    for eye,group in _registered[cache_key][1]:
        im=images[eye]
        matches=total=0
        for axis,ys,xs,sign in group:
            good=np.zeros(len(ys),dtype=bool)
            for dy in (-2,0,2):
                for dx in (-2,0,2):
                    y,x=ys+dy,xs+dx
                    a,b=(im[y+2,x],im[y-2,x]) if axis==0 else (im[y,x+2],im[y,x-2])
                    good|=((a.astype(int)-b.astype(int))*sign>=5)&(a<185)&(b<185)
            matches+=int(good.sum())
            total+=len(good)
        scores.append(matches/total if total>=25 else 0.)
    return scores


def fit(images,background,error=1.0):
    if min(registration_scores(images,background))<.90:
        return {"status":"robot-reference-lost"}
    # A shifted dark shelf edge alone can exceed the raw image-difference threshold.
    # Require the calibrated bright pipe against the darker registered shelf in both images.
    clean=[np.where((im>=185) & (im.astype(int)-gray.astype(int)>36),im,gray)
           for im,(gray,_) in zip(images,background)]
    result=full_fit(clean,background,error)
    if result["status"] in {"unbacked-crown","unbacked-end-arc","unbacked-end","missing-crown","unqualified-axis","unbacked-axis"}:
        return partial_fit(clean,background,error)
    return result
