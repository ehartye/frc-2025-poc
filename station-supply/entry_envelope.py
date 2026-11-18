"""Planar survey and flight geometry for a transverse cylindrical pipe."""
import math


def convex_hull(points):
    """Small monotone-chain hull of the horizontal and vertical segment endpoints."""
    points = sorted(set(points))
    def chain(seq):
        out = []
        for p in seq:
            while len(out) >= 2:
                a, b = out[-2:]
                if (b[0]-a[0])*(p[1]-a[1]) > (b[1]-a[1])*(p[0]-a[0]):
                    break
                out.pop()
            out.append(p)
        return out
    return chain(points)[:-1] + chain(reversed(points))[:-1]


def projected_section(p, q, landing_x, normal, lateral, yaw, travel, sag, radius_mm, half_length_mm):
    # The yaw pivot is the nominal touchdown center. Register normal and side
    # errors in station coordinates before inverse rotation into receiver axes.
    a = math.radians(yaw)
    arm = max(abs(p[0]-landing_x), abs(q[0]-landing_x))
    dx = normal + lateral*math.sin(a) + arm*(1-math.cos(a)) + half_length_mm*math.sin(a)
    dy = lateral + (normal+arm+radius_mm)*math.sin(a) + travel
    # For fixed tube-axis coordinate and radial height, yaw can only shorten
    # the horizontal radial projection. Inflate X separately to retain the
    # measured Z of the bottom and keep pressure-pad interactions observable.
    count = 64
    radius = radius_mm/math.cos(math.pi/count)
    circle = [(radius*math.cos((2*i+1)*math.pi/count), radius*math.sin((2*i+1)*math.pi/count))
              for i in range(count)]
    profile = convex_hull([(x+offset+cx, z+cz+rise) for x,z in (p,q) for offset in (-dx,dx)
                           for rise in (0,sag) for cx,cz in circle])
    return profile, half_length_mm+dy
