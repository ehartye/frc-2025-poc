"""Bring the high lower-mast tube back down to the original guide face attachment."""
from arm_route import ARM, BACK, NOSE, Z, Y, center_x, clip, j, span

# X offsets are relative to the guide center at each elevation. The narrow
# terminal pocket accepts the original tapped bar and preserves its two holes.
RETURNS={
    "front-return": (10.,16.,10.6,.9),
    "central-return": (0.,22.,10.6,.9),
    "rear-return": (-10.,16.,10.6,.9),
    "deep-return": (8.,20.,20.,1.2),
}


def build(kit,name):
    offset,width,depth,wall=RETURNS[name]
    height=760.
    root=clip(kit[ARM],-45.,4.)
    high=j.fuse(span(4.,75.,Z,height-Z),span(75.,center_x(height)+17.,height,0))
    top=center_x(height)+offset
    foot=center_x(Z)+offset
    # Open-ended folded box with its inboard face just outside the backing lips.
    def sheet(inset):
        lo=Z+6.
        hi=height-6.
        xlow=foot+.46*(lo-Z)
        xhigh=top+.46*(hi-height)
        coords=[(xlow-width/2+inset,Y+inset,lo),(xlow+width/2-inset,Y+inset,lo),
                (xhigh+width/2-inset,Y+inset,hi),(xhigh-width/2+inset,Y+inset,hi)]
        return j.extrude(coords,(0,depth-2*inset,0))
    post=j.cut(sheet(0.),sheet(wall))
    # Join the original tapped bar to the returned section with a short box.
    x=center_x(Z)
    outer=j.box(x-17.,max(x+17.,foot+width/2+3.),Y,Y+depth,Z-6.,Z+6.)
    pocket=j.box(x-15.,x+32.,Y+.8,Y+9.8,Z-5.2,Z+5.2)
    lower=j.cut(outer,pocket)
    for dx in (-8.,8.):
        lower=j.cut(lower,j.rod(x+dx,1,Y-1.,Y+12.,Z,1.65))
    arm=j.fuse(j.fuse(j.fuse(root,high),post),lower)
    return {ARM:arm,BACK:kit[BACK],**{k:kit[k] for k in NOSE}}
