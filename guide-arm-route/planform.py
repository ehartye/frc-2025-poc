"""Machine a variable-width lower guide arm retaining the existing root and face fasteners."""
from arm_route import ARM, BACK, NOSE, Y, Z, center_x, clip, j, surface_y

# Each interior point gives X and the inner/outer Y edges. All profiles share
# the installed pin/screw root, the 12 mm stock height and the existing nose hardware.
PLANS = {
    "narrow-corridor": [(20,Y,Y+30),(80,150,158),(132,146,154),(155,148,155),
                        (173,154.5,161),(183,Y,Y+10.6)],
    "shallow-corridor": [(20,Y,Y+30),(80,151.8,159),(132,150.9,155.3),(155,151,156),
                         (173,155,161),(183,Y,Y+10.6)],
    "wide-corridor": [(20,Y,Y+30),(80,145,158),(132,139,153.5),(155,143,155),
                      (173,153,161),(183,Y,Y+10.6)],
    "straight-narrow": [(20,Y,Y+30),(82,Y,Y+10.6),(183,Y,Y+10.6)],
}


def build(kit,name):
    tail=center_x(Z)+17
    stations=[(4.,Y,Y+30),*PLANS[name],(tail,Y,Y+10.6)]
    pts=[(x,lo,Z-6) for x,lo,hi in stations]+[(x,hi,Z-6) for x,lo,hi in reversed(stations)]
    body=j.extrude(pts,(0,0,12))
    # Axial access for the original tapped bar. Retain 0.8 mm cheeks and a 0.2 mm vertical fit allowance.
    pocket=j.box(center_x(Z)-15,tail+1,Y+.8,Y+9.8,Z-5.2,Z+5.2)
    body=j.cut(body,pocket)
    for dx in (-8,8):
        body=j.cut(body,j.rod(center_x(Z)+dx,1,Y-1,Y+11,Z,1.65))
    return {ARM:j.fuse(clip(kit[ARM],-45,4),body),BACK:kit[BACK],**{k:kit[k] for k in NOSE}}

