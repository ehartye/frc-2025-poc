"""Register the three scoped drive assemblies to the physical gate and hood."""
import math

def convex(points):
    p=sorted(set(points))
    def chain(seq):
        out=[]
        for a in seq:
            while len(out)>1:
                b,c=out[-2:]
                if (c[0]-b[0])*(a[1]-b[1])-(c[1]-b[1])*(a[0]-b[0])>0:break
                out.pop()
            out.append(a)
        return out
    return chain(p)[:-1]+chain(reversed(p))[:-1]

def circles(p,r,q,s):
    dx,dz=q[0]-p[0],q[1]-p[1];d=math.hypot(dx,dz)
    along=(r*r-s*s+d*d)/(2*d);side=math.sqrt(r*r-along*along)
    return (p[0]+(along*dx-side*dz)/d,p[1]+(along*dz+side*dx)/d)

def register(p,put,m):
    px=-m.ARM*1000;hz=714.;rear=(m.REAR,m.Z0);pivot=(px,hz)
    jack=circles(rear,165.,pivot,32.)
    def base(k,s,mat="Al"):put(k,"base",mat,s)
    def ringbelt(xz1,xz2,r1,r2,y,width):
        def solid(allow,lo,w):
            poly=convex([(x+(r+allow)*math.cos(2*i*math.pi/72),z+(r+allow)*math.sin(2*i*math.pi/72))
                         for (x,z),r in ((xz1,r1),(xz2,r2)) for i in range(72)])
            return m.rd.prism([(x,lo,z) for x,z in poly],(0,w,0))
        return m.subtract(solid(1.2,y,width),solid(-1.14,y-1,width+2))
    # NEO/gearboxes are procurement envelopes. Primary works at common fixed jackshaft.
    jx,jz=jack
    base("common-drive-NEO550",m.rod(jx,27,jz,m.AY,17.5,44.5),"motor")
    base("common-drive-UP3-UP5",m.box(jx-19,-8,jz-19,38,35,38),"gearbox")
    base("fixed-jackshaft",m.rod(jx,-53,jz,m.AY,4.0,45),"steel")
    base("common-drive-mount",m.subtract(m.box(jx-20,-12,jz-20,40,4,40),m.rod(jx,-13,jz,m.AY,8,6)))
    r=60/(2*math.pi)
    base("390-3M-9-fixed-drive",ringbelt(rear,jack,r,r,-44,9),"rubber")
    base("jack-drive-20T",m.ann(jx,-44,jz,"y",r-1.24,4.05,9))
    rear_pulley=m.subtract(m.rod(rear[0],-44,rear[1],m.AY,r-1.24,9),m.hex_shaft(rear[0],-45,rear[1],12.85,11))
    base("rear-drive-20T",rear_pulley)
    base("reverser-input-32T",m.ann(jx,-52,jz,"y",15.95,4.05,5))
    base("reverser-output-32T",m.ann(px,-52,hz,"y",15.95,5.05,5))
    # The hinge is the center of the small sprocket, so hood motion cannot alter belt length.
    hy=-188.; top=(0.,hz)
    put("315-3M-9-swing-drive","hood","rubber",ringbelt(pivot,top,108/(2*math.pi),150/(2*math.pi),hy,9))
    base("hood-input-36T",m.ann(px,hy,hz,"y",108/(2*math.pi)-1.24,5.05,9))
    put("hood-output-50T","hood","Al",m.subtract(m.rod(0,hy,hz,m.AY,150/(2*math.pi)-1.24,9),m.hex_shaft(0,hy-1,hz,12.85,11)))
    # One independent 82:1 hood positioner rotates the swing, around its powered input shaft.
    phx=px-36
    base("hood-position-NEO550",m.rod(phx,74,714,m.AY,17.5,44.5),"motor")
    base("hood-position-UP5-UP5",m.box(phx-19,118.5,695,38,35,38),"gearbox")
    base("hood-position-output-shaft",m.rod(phx,153.5,hz,m.AY,3,31),"steel")
    base("hood-position-18T",m.ann(phx,178,hz,"y",8.95,3.05,5))
    put("hood-sector-54T","hood","Al",m.ann(px,178,hz,"y",26.95,5.2,5))
    for zz in (-3,3):
        put("sector-link-"+str(zz),"hood","steel",m.rod(px+18,162,hz+zz,m.AY,1.5,16))
    # Proposed 900 mm fixed-center belt, 420 mm centers, 20-tooth ends.
    for sign in (-1,1):
        base(f"gate-pulley-{sign}",m.ann(38,210*sign+4,497,"z",r-1.24,4.05,9))
        base(f"gate-pulley-axle-{sign}",m.rod(38,210*sign+4,489,m.AZ,4,66 if sign==1 else 22),"steel")
    base("gate-drive-spur",m.ann(38,214,550,"z",17.95,4.05,5))
    base("gate-motor-spur",m.ann(38,250,550,"z",17.95,3.05,5))
    base("gate-output",m.rod(38,250,549,m.AZ,3,10),"steel")
    base("gate-UP5",m.box(19,231,559,38,38,33),"gearbox")
    base("gate-NEO550",m.rod(38,250,592,m.AZ,17.5,44.5),"motor")
    for lr,s in (("L",1),("R",-1)):
        y=s*85
        sx=38+s*r
        # Flat tooth region on the outside of each strand, plus welded connection to its carriage.
        arm=m.box(sx+(1.2 if s==1 else -4.2),y-5,497,3,10,9)
        put("gate-clamp-"+lr,lr,"Al",arm)
        if s==1:link=m.box(sx+4.2,y-3,500,68-8-(sx+4.2),6,4)
        else:link=m.box(-2,y-3,500,sx-4.2+2,6,4)
        put("gate-link-"+lr,lr,"Al",link)
        # Electrical sensors remain local and independent from steel channel/receiver hits.
        base("gate-open-switch-"+lr,m.box(48,s*210-3,483,7,6,6),"sensor")
        put("gate-travel-flag-"+lr,lr,"Al",m.box(50,min(s*85,s*98)-1,490,4,15,3))
        base("gate-closed-switch-"+lr,m.box(48,s*98-3,483,7,6,6),"sensor")
    return jack
