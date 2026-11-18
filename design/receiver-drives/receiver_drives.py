"""Original actuator geometry. Coordinates mm, +X deploys, +Y left, +Z up."""
from functools import cache
from math import pi,cos,sin,tan,radians,acos
from pathlib import Path
import sys
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace,BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt,gp_Trsf,gp_Vec

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"station-l1"))
import build_assembly as cad
from mechanical import box,cylinder,cut

REPLACES={"slide-drive","belt-cartridge-L","belt-cartridge-R"}
REPLACES.update(f"belt-riser-{side}-{x}" for side in ("L","R") for x in (-90,136))
REPLACES.update(f"shelf-screw-{side}-{x}" for side in ("L","R") for x in (-80,140))
PITCH_RADIUS=60/(2*pi)
GRIP_Y=150.8125+PITCH_RADIUS+1.2
AXES={"z":(0,0,1),"y":(0,1,0),"x":(1,0,0)}
PINION_CENTERS={
    "slide-fixed-rack-pinion":(-105,74,-106,-pi/2-25/10),
    "slide-inner-rack-pinion":(105,52,-106,pi/2+230/10),
    "slide-power-pinion":(55,-120,-100,pi/2+192/10),
}


def prism(points,vec):
    wire=BRepBuilderAPI_MakePolygon()
    for p in points: wire.Add(gp_Pnt(*p))
    wire.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),gp_Vec(*vec)).Shape()


def hex_end(x,y,z,axis,length):
    r=5/3**.5
    uv=[(r*cos(i*pi/3),r*sin(i*pi/3)) for i in range(6)]
    pts=[(x+a,y+b,z) if axis=="z" else (x+a,y,z+b) for a,b in uv]
    return prism(pts,(0,0,length) if axis=="z" else (0,length,0))


def annulus(x,y,z,axis,ro,ri,length):
    v=AXES[axis]
    return cad.brep(cut(cylinder(x,y,z,v,ro,length),
                        cylinder(x-v[0],y-v[1],z-v[2],v,ri,length+2)))


def block(*a): return cad.brep(box(*a))


def loop(x1,x2,y,z,r,width,axis):
    # Smooth annular working envelope of the HTD belt; cord pitch radii are registered separately.
    def outline(rad):
        p=[(x2+rad*cos(-pi/2+i*pi/24),rad*sin(-pi/2+i*pi/24)) for i in range(25)]
        p.extend((x1+rad*cos(pi/2+i*pi/24),rad*sin(pi/2+i*pi/24)) for i in range(25))
        return p
    def shape(rad,offset,length):
        pts=[(x,y+s,z+offset) if axis=="z" else (x,y+offset,z+s) for x,s in outline(rad)]
        return prism(pts,(0,0,length) if axis=="z" else (0,length,0))
    return cad.BRepAlgoAPI_Cut(shape(r+1.2,0,width),shape(r-1.1,-1,width+2)).Shape()


def pinion(x,y,z,angle):
    n=20; base=10*cos(radians(20)); outer=11.; root=8.75
    def half(r):
        a=acos(base/max(base,r))
        return pi/(2*n)+tan(radians(20))-radians(20)-(tan(a)-a)-.0025
    radii=[root,base,9.65,10.,10.5,outer]
    pts=[]
    for tooth in range(n):
        c=angle+2*pi*tooth/n
        for side,rr in ((-1,radii),(1,reversed(radii))):
            for r in rr:
                t=c+side*half(r)
                pts.append((x+r*cos(t),y,z+r*sin(t)))
    gear=prism(pts,(0,8,0))
    return cad.BRepAlgoAPI_Cut(gear,cad.brep(cylinder(x,y-1,z,AXES["y"],2.5,10))).Shape()


def rack(start,y,pitch_z,cells,faces_up):
    # Module 1 straight rack, 20 degree PA, 0.10 mm total pitch backlash.
    sign=1 if faces_up else -1
    root=-1.25; tip=1.; flank=tan(radians(20)); half=pi/4-.025
    pts=[(start,pitch_z+sign*(root-3.75))]
    for i in range(cells):
        c=start+(i+.5)*pi
        pts.extend((c+d,pitch_z+sign*h) for d,h in ((-half+root*flank,root),
                   (-half+tip*flank,tip),(half-tip*flank,tip),(half-root*flank,root)))
    pts.extend([(start+cells*pi,pitch_z+sign*root),(start+cells*pi,pitch_z+sign*(root-3.75))])
    return prism([(x,y,z) for x,z in pts],(0,8,0))


@cache
def parts():
    items={}
    def put(name,frame,category,shape):items[name]=(frame,category,shape)
    def rod(x,y,z,axis,r,length):return cad.brep(cylinder(x,y,z,AXES[axis],r,length))
    def holes(spec,xs,y,z,depth,r):
        return cad.brep(cut(spec,*[cylinder(x,y,z,AXES["z"],r,depth) for x in xs]))
    x1,x2=-84.5,95.5
    for side in ("L","R"):
        def grip(name,category,body):
            if side=="R":
                tr=gp_Trsf();tr.SetMirror(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,1,0)))
                body=BRepBuilderAPI_Transform(body,tr,True).Shape()
            put("grip-"+side+"-"+name,"slide",category,body)
        y=GRIP_Y
        grip("420-3M-15","belt-envelope",loop(x1,x2,y,-10.35,PITCH_RADIUS,15,"z"))
        for label,x in (("drive",x1),("idler",x2)):
            grip(label+"-20T","pulley-root-envelope",annulus(x,y,-10.35,"z",PITCH_RADIUS-1.14,2.5,15))
            grip(label+"-shaft","shaft-envelope",rod(x,y,-21,"z",2.5,36))
            grip(label+"-lower-bearing","bearing-envelope",annulus(x,y,-19,"z",6.5,2.5,5))
            grip(label+"-upper-bearing","bearing-envelope",annulus(x,y,6,"z",6.5,2.5,5))
        for deck,z in (("lower",-19),("upper",6)):
            spec=cut(box(-94,y-9,z,200.5,19,5),cylinder(x1,y,z-1,AXES["z"],6.5,7),
                     box(x2-8,y-8,z-1,16,16,7))
            if deck=="lower":
                for x in (-73,82):spec=cut(spec,cylinder(x,y-10,-16.5,AXES["y"],1.6,22))
            grip(deck+"-cheek","machined",cad.brep(spec))
            grip(deck+"-replaceable-offset-insert","machined",
                 cad.brep(cut(box(x2-8,y-8,z,16,16,5),cylinder(x2,y,z-1,AXES["z"],6.5,7))))
        for label,x in (("rear",-73),("front",82)):
            # 3 mm rods in bushings through sprung cheek bosses. Outboard towers stay on the shelf.
            mountx=-80 if x<0 else 140
            blank=box(mountx-10,y-8,-42,22,40,4)
            if x>0:blank=["union",[blank,box(74,y+9,-42,58,23,4),box(74,y-8,-42,15,40,4)]]
            foot=cut(blank,cylinder(mountx,180,-43,AXES["z"],2.65,6),
                     cylinder(mountx,180,-41,AXES["z"],4.5,4))
            grip(label+"-tower","machined",cad.brep(cut(box(x-5,y+24,-38,10,8,24),
                            cylinder(x,y+23,-16.5,AXES["y"],1.6,10))))
            grip(label+"-support-foot","machined",cad.brep(foot))
            grip(label+"-riser","machined",cad.brep(cut(box(mountx-10,156 if x>0 else 150.8125,-60,22,34 if x>0 else 40,18),
                 cylinder(mountx,180,-61,AXES["z"],2.65,20))))
            grip(label+"-long-shelf-screw","fastener-envelope",cad.brep(["union",[
                 cylinder(mountx,180,-87,AXES["z"],2.5,46),cylinder(mountx,180,-41,AXES["z"],4.25,3)]]))
            grip(label+"-slide-guide","guide-envelope",rod(x,y-3,-16.5,"y",1.5,34))
            grip(label+"-spring","spring-envelope",annulus(x,y+10,-16.5,"y",3.2,1.7,14))
        grip("inboard-hardstop","machined",block(76,y+1,-38,4,2,16))
        grip("outboard-hardstop","machined",block(76,y+10,-38,4,2,16))
        grip("moving-stop-lug","machined",block(76,y+4.5,-29,4,4,10))
        grip("fixed-backbone","machined",block(-68,y+22,-42,142,6,4))
        chamber=cut(box(14,y+5,-38,26,25,7),box(16,y+3,-36.5,22,24,3))
        grip("dashpot-housing","machined",cad.brep(chamber))
        grip("dashpot-paddle","machined",block(17,y+8,-36,20,8,2))
        grip("dashpot-stem","machined",block(24,y-2,-36,5,10,2))
        grip("dashpot-link","machined",block(24,y-3,-36,5,1,17))
        grip("drive-22T","spur-pitch-envelope",annulus(x1,y,12,"z",11,2.5,5))
        grip("opposed-22T","spur-pitch-envelope",annulus(x1,y+22,12,"z",11,2.5,5))
        grip("gearbox-output","shaft-envelope",cad.BRepAlgoAPI_Fuse(
            rod(x1,y+22,12,"z",2.5,9),hex_end(x1,y+22,21,"z",8)).Shape())
        grip("cartridge-mount","machined",cad.brep(cut(box(x1-19,y+3,18,38,38,3),
                     cylinder(x1,y+22,17,AXES["z"],8.1,6))))
        grip("UP5","selected-article-envelope",cad.brep(cut(box(x1-19,y+3,21,38,38,33),
             cylinder(x1,y+22,20,AXES["z"],2.91,10))))
        grip("NEO550","selected-article-envelope",rod(x1,y+22,54,"z",17.5,44.5))
        grip("idler-magnet","selected-article-envelope",rod(x2,y,15,"z",2.5,1))
        grip("idler-angle-board","sensor-reserve",block(x2-6,y-6,17,12,12,4))
        grip("backing-strip","machined",block(-71,y-PITCH_RADIUS+1.1,-10,154.5,3,14))
        for x in (-25,65):
            grip("wear-strip-foot-"+str(x),"machined",block(x,y-PITCH_RADIUS+1.1,-14,5,3,4))
        grip("sensor-standoff","machined",block(x2+2,y+4,11,4,2,6))
        for x in (-68,78):
            grip("spacer-"+str(x),"machined",rod(x,y+5,-14,"z",2,20))
        grip("motor-backstay","machined",block(x1-16,y+6,11,6,5,7))
    # 210 mm between equal timing pulleys, pitch loop = 480 mm. Rack pitch radius = 10.
    put("slide-synchronizer-480-3M","middle","belt-envelope",loop(-105,105,87,-106,PITCH_RADIUS,9,"y"))
    for label,x in (("fixed-rack",-105),("inner-rack",105)):
        lane=74 if x<0 else 52
        angle=-pi/2-(x+130)/10 if x<0 else pi/2+(x+125)/10
        put("slide-"+label+"-pinion","middle","involute-section",pinion(x,lane,-106,angle))
        put("slide-"+label+"-pulley","middle","pulley-root-envelope",annulus(x,87,-106,"y",PITCH_RADIUS-1.14,2.5,9))
        put("slide-"+label+"-shaft","middle","shaft-envelope",rod(x,min(68,lane-1),-106,"y",2.5,103-min(68,lane-1)))
        for bearing_y in (68,97):
            put("slide-"+label+"-bearing-"+str(bearing_y),"middle","bearing-envelope",
                annulus(x,bearing_y,-106,"y",6.5,2.5,5))
    for by in (68,97):
        spec=box(-115,by,-116,229,5,20)
        spec=cut(spec,box(-113,by-1,-114,16,7,16),cylinder(105,by-1,-106,AXES["y"],6.5,7))
        put("slide-sync-offset-insert-"+str(by),"middle","machined",cad.brep(cut(
            box(-113,by,-114,16,5,16),cylinder(-105,by-1,-106,AXES["y"],6.5,7))))
        put("slide-sync-cheek-"+str(by),"middle","machined",cad.brep(spec))
    # Shallow transverse bracket terminates against the outside of the middle rail's near wall.
    for x in (-79,79):
        bridge=["union",[box(x-4,102,-109,8,4,30),box(x-4,102,-83,8,66.8,4),
                         box(x-4,166.2,-84,8,2.6,2)]]
        put("slide-sync-bracket-"+str(x),"middle","machined",cad.brep(bridge))
    put("slide-fixed-rack","lift","rack-section",rack(-130,74,-116,77,True))
    put("slide-fixed-rack-bed","lift","machined",block(-130,74,-122,77*pi,8,1))
    put("slide-inner-rack","slide","rack-section",rack(-125,52,-96,80,False))
    for x in (-96,86):
        put("slide-inner-rack-anchor-"+str(x),"slide","machined",block(x,52,-91,8,8,16))
    put("slide-driven-rack","middle","rack-section",rack(-137-3*pi,-120,-90,73,False))
    for x in (-118,60):
        put("slide-middle-rack-bracket-"+str(x),"middle","machined",cad.brep(["union",[
            box(x,-168.8,-83,7,56.8,3),box(x,-168.8,-84,7,2.6,2),box(x,-120,-85,7,8,3)]]))
    put("slide-power-pinion","lift","involute-section",pinion(55,-120,-100,pi/2+(55+137)/10))
    put("slide-power-output-shaft","lift","shaft-envelope",cad.BRepAlgoAPI_Fuse(
        rod(55,-126,-100,"y",2.5,18),hex_end(55,-108,-100,"y",8)).Shape())
    put("slide-power-output-support","lift","bearing-envelope",annulus(55,-126,-100,"y",6.5,2.5,5))
    put("slide-power-bearing-block","lift","machined",cad.brep(cut(box(46,-126,-122,18,5,30),
                    cylinder(55,-127,-100,AXES["y"],6.5,7))))
    put("slide-power-UP5-UP4","lift","selected-article-envelope",cad.brep(cut(box(36,-108,-119,38,44,38),
         cylinder(55,-109,-100,AXES["y"],2.91,10))))
    put("slide-power-NEO550","lift","selected-article-envelope",rod(55,-64,-100,"y",17.5,44.5))
    put("slide-power-bracket","lift","machined",cad.brep(cut(box(36,-112,-122,38,4,41),
                    cylinder(55,-113,-100,AXES["y"],8.1,6))))
    # Two independent angle channels on the middle: passive pitch pinions measure each rack.
    for sy,x in enumerate((-105,105)):
        put("slide-sync-angle-board-"+str(sy),"middle","sensor-reserve",block(x-6,105,-112,12,4,12))
        put("slide-sync-magnet-"+str(sy),"middle","selected-article-envelope",rod(x,103,-106,"y",2.5,1))
        put("slide-sync-sensor-standoff-"+str(sy),"middle","machined",block(x+4,102,-112,2,3,2))
    put("slide-position-SP1-25","lift","selected-article-envelope",block(1,-227,-189,67,61,60))
    put("slide-position-saddle","lift","machined",block(1,-205,-129,67,39,2))
    put("slide-position-target","slide","machined",block(117,-210,-148,6,4,73))
    put("slide-position-line","lift","measurement-line",rod(68,-208,-145,"x",.24,49))
    put("slide-middle-stop-dog","middle","machined",block(-100,-146,-104,5,10,8))
    put("slide-middle-stop-dog-leg","middle","machined",cad.brep(["union",[
        box(-100,-166.8,-83,5,26,3),box(-100,-146,-96,5,5,16),box(-100,-168.8,-84,5,2.6,2)]]))
    for label,x in (("retracted",-107),("extended",91)):
        put("slide-"+label+"-stop","lift","machined",block(x,-146,-122,6,10,28))
        put("slide-"+label+"-switch","lift","sensor-reserve",block(x-4,-157,-116,16,7,12))
    return items


def translates_with_grip(key):
    if not key.startswith("grip-"):
        return False
    fixed=("tower","support-foot","riser","long-shelf-screw","fixed-backbone","dashpot-housing","hardstop")
    return not key.endswith(fixed) and not key.endswith("-spring")


def added_shapes(pose,lateral_offset_mm=None):
    p=cad.MODEL["poses"][pose] if isinstance(pose,str) else pose
    output={}
    for name,(frame,kind,shape) in parts().items():
        if kind=="measurement-line":
            shape=cad.brep(cylinder(68,-208,-145,AXES["x"],.24,49+p["stroke"]))
        if kind=="involute-section":
            x,y,z,phase=PINION_CENTERS[name]
            shape=pinion(x,y,z,phase-p["stroke"]/20)
        dy=0.
        if lateral_offset_mm and name.startswith("grip-"):
            side=name.split("-")[1]
            outward=lateral_offset_mm[side]
            if abs(outward)>1.5:raise ValueError("beyond mechanical stop")
            sign=1 if side=="L" else -1
            if translates_with_grip(name):dy=sign*outward
            if name.endswith("-spring"):
                x=-73 if "-rear-" in name else 82
                ay=(0,sign,0);y=sign*(GRIP_Y+10+outward)
                shape=cad.brep(cut(cylinder(x,y,-16.5,ay,3.2,14-outward),
                                   cylinder(x,y-sign,-16.5,ay,1.7,16-outward)))
        output[name]=cad.translated(shape,(p["stroke"]*{"lift":0,"middle":.5,"slide":1}[frame],dy,p["height"]))
    return output
