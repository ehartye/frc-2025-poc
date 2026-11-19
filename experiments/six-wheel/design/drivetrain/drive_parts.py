"""Supplier-informed original packaging proxies; robot X forward, dimensions millimetres."""
from pathlib import Path
import math
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"station-l1"))
from mechanical import assembly,box,cylinder,cut,COST

Z=76.2
PITCH=6.35
TEETH=32
LINKS=110
CENTERS=(LINKS-TEETH)*PITCH/2
PITCH_R=PITCH/(2*math.sin(math.pi/TEETH))
LANES=(251.5,268.0)


def parts():
    base=assembly()
    excluded={"frame","wheel","drive","shaft","chain","pulley","bearing"}
    base=[p for p in base if p["group"] not in excluded or p["id"].startswith(("cross-","mast-","lift-crossbar"))]
    base=[p for p in base if not p["id"].startswith("drive-foot")]
    def add(name,group,shape,basis,material=None,mass=None):
        base.append(dict(id=name,group=group,form=shape,frame="base",material=material,
                         mass=mass,basis=basis,costs=COST[group]))
    for s,side in [(-1,"R"),(1,"L")]:
        def pos(t,width):
            return t if s==1 else -(t+width)
        def bx(x,t,z,l,w,h):
            return box(x,pos(t,w),z,l,w,h)
        def cy(x,t,z,r,w):
            return cylinder(x,pos(t,w),z,[0,1,0],r,w)
        def hexbar(x,t,z,af,w):
            return ["hex",[x,pos(t,w),z,af,w]]
        def cap(a,b,t,z,r,w):
            return ["union",[cy(a,t,z,r,w),bx(a,t,z-r,b-a,w,2*r),cy(b,t,z,r,w)]]
        def axis_bore(nominal,x,t,z,w):
            return hexbar(x,t,z,nominal,w)
        def qty(id,group,shape,basis,material=None,mass=None):
            add(f"{id}-{side}",group,shape,basis,material,mass)
        for loc,t,w in [("inner",229,6),("outer",325,25)]:
            x0=-330 if loc=="inner" else -350
            tube=bx(x0,t,50,-2*x0,w,100)
            tools=[]
            if loc=="outer":
                tools.append(bx(x0-1,t+2,52,-2*x0+2,21,96))
            for idx,x in enumerate([-CENTERS,0,CENTERS]):
                for dx,dz,r in [(0,0,16.2),(-19,-17,2.7),(-19,17,2.7),(19,-17,2.7),(19,17,2.7)]:
                    if idx==1:
                        tools.append(cy(x+dx,t-1,Z+dz,r,w+2))
                    else:
                        tools.append(cap(x+dx-4,x+dx+4,t-1,Z+dz,r,w+2))
            if loc=="inner":
                tools.extend(cy(0,t-1,Z+dz,5.2,w+2) for dz in [-25.4,25.4])
                tools.extend(cy(dx,t-1,Z+dz,2.7,w+2) for dx in [-40,40] for dz in [-12,12])
            qty("drive-rail-"+loc,"frame",cut(tube,*tools),"Machined rail; end bearing slots have ±4 mm travel","Al")
        for xi,x in enumerate([-CENTERS,0,CENTERS]):
            tag=["rear","center","front"][xi]
            # Common x and z on both faces keep the axle parallel across the chassis.
            for loc,t in [("inner",235),("outer",319)]:
                cuts=[cy(x,t-1,Z,14.31,8)]
                if loc=="inner" and xi==1:
                    cuts.extend(cy(0,t-1,Z+dz,5.2,8) for dz in [-25.4,25.4])
                for dx in [-19,19]:
                    for dz in [-17,17]:
                        cuts.append(cy(x+dx,t-1,Z+dz,2.65,8))
                qty(f"carrier-{tag}-{loc}","plate",cut(bx(x-28,t,Z-26,56,6,52),*cuts),
                    "6 mm plate, bore and 4 clearance holes; original clamp carrier","Al")
                start=234.65 if loc=="inner" else 319
                # Flange faces the open bay. Rounded-hex bearing uses a circular bore.
                fl=241 if loc=="inner" else 317.41
                bearing=["union",[cy(x,start,Z,14.29,6.35),cy(x,fl,Z,15.56,1.59)]]
                qty(f"bearing-{tag}-{loc}","bearing",cut(bearing,cy(x,start-3,Z,6.875,13)),
                    "REV-21-1916-PK4 rounded-hex bearing; nominal stepped ring interface",mass=(0.023,0.023))
                if xi!=1:
                    # Jacks bear on the inboard X edge of each carrier in the tightening direction.
                    sign=1 if x>0 else -1
                    pad_x=x-sign*45
                    left=pad_x-4
                    line_y=pos(t,6)+3
                    pad=cut(bx(left,t,Z-5,8,6,10),cylinder(left-1,line_y,Z,[1,0,0],2.3,10))
                    qty(f"takeup-tab-{tag}-{loc}","plate",pad,"Original bonded/welded fixed jack tab, tapped M4","Al")
                    tip=x-sign*28
                    screw_start=min(pad_x-sign*8,tip)
                    screw=cylinder(screw_start,line_y,Z,[1,0,0],1.95,abs(pad_x-sign*8-tip))
                    qty(f"jack-{tag}-{loc}","fastener",screw,"M4 jack shank envelope; four carrier clamps carry operating loads","steel")
            # am-0940b nominal wheel counterbores and pilot; six #10 fastener interfaces.
            wheel=cy(x,277.3,Z,76.2,25.4)
            tools=[cy(x,276.3,Z,12.192,27.4),cy(x,277.2,Z,14.2875,6.45),cy(x,296.35,Z,14.2875,6.45)]
            wheel_holes=[]
            for i in range(6):
                a=math.radians(i*60)
                xx,zz=x+23.8125*math.cos(a),Z+23.8125*math.sin(a)
                wheel_holes.append((xx,zz))
                tools.append(cy(xx,276.3,zz,1.994,27.4))
            qty("wheel-"+tag,"wheel",cut(wheel,*tools),"am-0940b 77A wheel; outer contact and hub-interface proxy",mass=(0.35,0.55))
            # Orient long shoulder into the wheel. Drawing dimension chain referenced to short end.
            end=302.7+.313*25.4
            stations=[(0,.063,.75),(.063,.125,1.123),(.125,.313,2.31),(.313,.375,1.123),
                      (.375,.938,.88),(.938,1,.75)]
            hub=["union",[cy(x,end-b*25.4,Z,d*12.7,(b-a)*25.4) for a,b,d in stations]]
            cutters=[axis_bore(12.75,x,end-26,Z,27)]
            cutters.extend(cy(xx,end-26,zz,2.5527,27) for xx,zz in wheel_holes)
            qty("hub-"+tag,"pulley",cut(hub,*cutters),"am-4124 Rev 3 original radial envelope and bolt circle; long shoulder inward",
                mass=(0.042,0.052))
            begin=214 if xi==1 else 233.4
            bar=["intersect",[hexbar(x,begin,Z,12.7,327-begin),cy(x,begin,Z,6.875,327-begin)]]
            qty("axle-"+tag,"shaft",bar,"REV-21-1135 custom-cut rounded hex; ends tapped to retention drawing","Al")
            # Individual spacer ends land on bearing inner rings, sprocket hubs or the wheel hub.
            active=LANES if xi==1 else [LANES[0 if xi==0 else 1]]
            stops=[(242.59,242.59)]+[(t-4.8,t+4.7) for t in active]+[(end-25.4,end),(317.41,317.41)]
            for j,((_,start),(stop,_)) in enumerate(zip(stops,stops[1:])):
                width=stop-start-.10
                qty(f"spacer-{tag}-{j}","shaft",cut(cy(x,start+.05,Z,9,width),cy(x,start-.1,Z,7.1,width+.3)),
                    "Original 18 OD / 14.2 ID tube; selectively shim after measuring inner rings and wheel hub","Al")
        for i,(a,b) in enumerate([(-CENTERS,0),(0,CENTERS)]):
            t=LANES[i]
            # Side bands straddle the tooth faces. One representative spanning pin shows width.
            w=(9.1948-3.175)/2
            inner=PITCH_R-2.921
            outer=PITCH_R+2.921
            def band(face_t):
                return cut(cap(a,b,face_t,Z,outer,w),cap(a,b,face_t-.2,Z,inner,w+.4))
            chain=["union",[band(t-4.5974),band(t+1.5875),cy((a+b)/2,t-4.5974,Z+PITCH_R,1.15,9.1948)]]
            qty("chain-"+str(i),"chain",chain,"Tsubaki BF25H 110 pitches, condensed side-band and one-pin witness",mass=(0.07,0.11))
            for name,x in [("a",a),("b",b)]:
                sprocket=["union",[cy(x,t-1.5,Z,34.05,3),cy(x,t-4.8,Z,14.5,9.5)]]
                qty(f"sprocket-{i}-{name}","pulley",cut(sprocket,hexbar(x,t-6,Z,12.75,12)),
                    "REV-21-2018 32-tooth sprocket; tooth-tip disk and hub envelope",mass=(0.025,0.050))
        # Rotate the native output face 90 degrees about the axle. The two opposed face
        # holes on its 50.8 mm circle then straddle the bearing in Z.
        gear=cut(bx(-25.4,160.5,Z-30.95,50.8,63.5,61.9),hexbar(0,212.9,Z,12.76,13),
                 *(cy(0,217.5,Z+dz,2.5,7) for dz in [-25.4,25.4]))
        qty("gearbox","drive",gear,"REV-25-2109 with 4:1 and 5:1 cartridges; outer reservation, socket modeled",
            mass=(0.60,1.00))
        qty("motor","drive",cy(0,102.25,Z,30,58.25),"REV-21-1650 NEO V1.1 body envelope; sensor/phase tail separate",
            mass=(0.425,0.425))
        qty("drive-controller","power",bx(135,100,154,69.3,34.5,25),"REV-11-2158 SPARK MAX enclosure; cooling and plug/loom swing open",
            mass=(0.09,0.15))
        face_tools=[cy(0,223,Z,18,7)]
        face_tools.extend(cy(0,223,Z+dz,2.65,7) for dz in [-25.4,25.4])
        face_tools.extend(cy(dx,223,Z+dz,2.7,7) for dx in [-40,40] for dz in [-12,12])
        qty("gearbox-faceplate","plate",cut(bx(-48,224,Z-33,96,5,66),*face_tools),
            "5 mm adapter; two opposed 10-32 clearances, four M5 rail holes, 36 mm socket opening","Al")
        for dz,slug in [(-25.4,"low"),(25.4,"high")]:
            screw=["union",[cy(0,219.475,Z+dz,2.36,9.525),cy(0,229,Z+dz,4,4.7)]]
            qty("gearbox-face-screw-"+slug,"fastener",screw,"#10-32 x 3/8 inch socket screw; verify native thread depth","steel")
    return base
