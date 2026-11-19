"""Rectified stereo measurement study. Lengths are mm and pixels refer to the mono sensors."""
from itertools import product
from math import cos, sin, radians, hypot, isfinite, tan

WIDTH, HEIGHT = 640, 480
FOCAL = WIDTH / (2 * tan(radians(73 / 2)))
BASELINE = 75.0
ORIGIN = (-315.0, 0.0, 970.0)
TILT = radians(26)
C, S = cos(TILT), sin(TILT)
RADIUS, LENGTH = 57.15, 301.625


def project(point, eye):
    x, y, z = (p - o for p, o in zip(point, ORIGIN))
    depth = x*C-z*S
    if depth <= 0:
        raise ValueError("behind imaging plane")
    return (WIDTH/2 + FOCAL*(eye-y)/depth, HEIGHT/2 - FOCAL*(x*S+z*C)/depth)


def robot_point(u, v, disparity):
    depth = FOCAL*BASELINE/disparity
    vertical = (HEIGHT/2-v)*depth/FOCAL
    return (ORIGIN[0]+depth*C+vertical*S, BASELINE/2-(u-WIDTH/2)*depth/FOCAL,
            ORIGIN[2]-depth*S+vertical*C)


def bound_pair(mark, error):
    # The left lens has positive robot Y. Its horizontal coordinate is larger.
    (u,v),(w,k) = mark
    if not all(isfinite(a) for a in (u,v,w,k,error)) or error < 0:
        raise ValueError("invalid observation or tolerance")
    if any(not (error <= px < WIDTH-error and error <= py < HEIGHT-error)
           for px,py in mark) or abs(v-k) > 2*error+1e-8:
        raise ValueError("outside usable frame or epipolar interval")
    d = u-w
    if d <= 2*error:
        raise ValueError("unbounded disparity")
    return [robot_point(a,b,c) for a,b,c in product((u-error,u+error),
            (v-error,v+error),(d-2*error,d+2*error))]


def clearance(marks, front, top, pixel_error, registration_mm, size_margin_mm):
    """Minimum over both end-center intervals and the maximum pipe radius.

    Marks are two independently registered end-center image pairs, not edge or bore pixels.
    Caller must reject partial/ambiguous rim fits and supply an exterior shelf pose bound.
    """
    if len(marks)!=2 or not all(isfinite(q) for q in (front,top,registration_mm,size_margin_mm)):
        raise ValueError("missing landmarks or shelf registration")
    if registration_mm < 0 or size_margin_mm < 0:
        raise ValueError("negative bound")
    boxes=[bound_pair(mark,pixel_error) for mark in marks]
    # Known endpoint distance must be possible; both repeated endpoints fail.
    def distance_bounds(a,b):
        ranges=[(min(p[i] for p in a)-max(q[i] for q in b),
                 max(p[i] for p in a)-min(q[i] for q in b)) for i in range(3)]
        return (hypot(*(max(0,lo,-hi) for lo,hi in ranges)),
                hypot(*(max(abs(lo),abs(hi)) for lo,hi in ranges)))
    lo,hi=distance_bounds(*boxes)
    if lo > LENGTH+size_margin_mm+1e-7 or hi < LENGTH-size_margin_mm-1e-7:
        raise ValueError("inconsistent end identity or length")
    xyz=boxes[0]+boxes[1]
    ahead=max(0,min(p[0] for p in xyz)-front)
    above=max(0,min(p[2] for p in xyz)-top)
    return hypot(ahead,above)-RADIUS-size_margin_mm-registration_mm


def endpoints(row):
    wx,wy,wz=row["xyz_m"]
    cx,cy,heading=row["chassis_xy_heading"]
    x=1000*((wx-cx)*cos(heading)+(wy-cy)*sin(heading))
    y=1000*(-(wx-cx)*sin(heading)+(wy-cy)*cos(heading))
    yaw=row["yaw_rad"]-heading
    pitch=row["pipe_pitch_rad"]
    return [(x-side*LENGTH/2*sin(yaw)*cos(pitch),y+side*LENGTH/2*cos(yaw)*cos(pitch),
             1000*wz+side*LENGTH/2*sin(pitch)) for side in (-1,1)]


def image_pairs(points):
    return [[project(p,eye) for eye in (BASELINE/2,-BASELINE/2)] for p in points]
