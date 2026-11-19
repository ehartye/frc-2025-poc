"""Project-owned B4-derived packaging. Millimetres, robot X forward, Y left, Z up."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
MODEL = json.loads((HERE / "assembly.json").read_text())
B4 = json.loads((HERE / "b4-source.json").read_text())
COST = {  # USD each, design allowances only. No selected vendor or quotes.
    "frame": (8,30), "plate": (5,28), "bearing": (3,20), "shaft": (4,20),
    "wheel": (10,55), "drive": (130,380), "bumper": (20,85), "battery": (65,160),
    "power": (220,480), "compute": (400,950), "lift": (100,360), "slide": (60,240),
    "guide": (5,55), "belt": (60,220), "chain": (12,50), "pulley":(5,28), "sensor": (10,75), "shelf": (25,120),
    "fastener": (0.10,2.50), "reserve": (190,750)}
DENSITY = {"Al": 2.70e-6, "steel": 7.85e-6, "PE": 0.95e-6}  # kg/mm³
COLORS = {"frame":"#77899a","plate":"#9eafbb","bearing":"#d7b067","shaft":"#bcbdca",
          "wheel":"#28343b","drive":"#445971","bumper":"#344ba5","battery":"#2a323d",
          "power":"#cd745d","compute":"#51938d","lift":"#9272b1","slide":"#9e6ea7",
          "guide":"#b5c5d5","belt":"#277e77","sensor":"#edab58","shelf":"#e5ae4c",
          "fastener":"#a6acb0","chain":"#445366","pulley":"#cfae76","reserve":"#888888"}


def box(x,y,z,l,w,h):
    return ["box", [x,y,z,l,w,h]]


def cylinder(x,y,z,axis,r,length):
    return ["cyl", [x,y,z,*axis,r,length]]


def cut(shape,*tools):
    return ["cut", [shape,*tools]]


def offset(part, pose):
    frame = part["frame"]
    stroke = {"base":0,"lift":0,"middle":0.5,"slide":1}[frame] * pose["stroke"]
    return [stroke,0,0 if frame=="base" else pose["height"]]


def assembly():
    parts = []
    def put(key, group, form, frame="base", material=None, mass=None, note="Original machined part"):
        parts.append(dict(id=key, group=group, form=form, frame=frame, material=material,
                          mass=mass, basis=note, costs=COST[group]))
    g = B4["geometry"]
    hx,hy = 500*g["frameLength"],500*g["frameWidth"]
    zrail, rail_height = 50,100
    wheel_r, wheel_width = 76,40
    wheel_centers = (-250,0,250)
    for sign, side in ((-1,"R"),(1,"L")):
        out = -hy if sign<0 else hy-25
        inside = -266 if sign<0 else 260
        # Two hollow rails straddle each wheel. Coaxial bores carry original sleeve bearings.
        for name,y,w in (("outer",out,25),("inner",inside,6)):
            x0,xlen = (-hx,2*hx) if name=="outer" else (-hx+20,2*hx-40)
            outer = box(x0,y,zrail,xlen,w,rail_height)
            tools = [cylinder(x,y-1,76,[0,1,0],9.1,w+2) for x in wheel_centers]
            if name=="outer":
                tools.append(box(x0-1,y+2,zrail+2,xlen+2,w-4,rail_height-4))
            put(f"rail-{side}-{name}","frame",cut(outer,*tools),material="Al")
        for x in wheel_centers:
            instance=f"{side}-{x}"
            y=-310 if sign<0 else 270
            cyl=cylinder(x,y,76,[0,1,0],wheel_r,wheel_width)
            put("wheel-"+instance,"wheel",cut(cyl,cylinder(x,y-1,76,[0,1,0],6.1,42)),
                mass=(0.4,0.8),note="Tread and hub outline; actual wheel, key and hub open")
            shaft_y=(-349 if sign<0 else (249 if x==0 else 260))
            put("axle-"+instance,"shaft",cylinder(x,shaft_y,76,[0,1,0],6,100 if x==0 else 89),material="steel")
            for name,by in (("outer",out if sign<0 else out+19),("inner",inside)):
                sleeve=cut(cylinder(x,by,76,[0,1,0],9,6),cylinder(x,by-1,76,[0,1,0],6.1,8))
                put(f"sleeve-{instance}-{name}","bearing",sleeve,material="steel",
                    note="Original plain sleeve concept; load, product, fit and lubrication open")
        drive_y=-250 if sign<0 else 170
        drive=cut(box(-50,drive_y,33,100,80,86),cylinder(0,drive_y-1,76,[0,1,0],9,82))
        put("drive-"+side,"drive",drive,mass=(1.8,3.0),
            note="Coaxial central wheel gearbox allocation; motor, gearing, motor controller and mounting pattern open")
        mount_y=-260 if sign<0 else 250
        for z in (43,101):
            put(f"drive-foot-{side}-{z}","plate",box(-42,mount_y,z,84,10,9),material="Al")
        # Independent loops to front and rear wheels. Outer run has room for two narrow axial lanes.
        for index,(a,b) in enumerate(((-250,0),(0,250))):
            y=(311+index*6) if sign>0 else (-316-index*6)
            def capsule(radius,width,dy=0):
                return ["union",[cylinder(a,y+dy,76,[0,1,0],radius,width),
                                  box(a,y+dy,76-radius,b-a,width,2*radius),
                                  cylinder(b,y+dy,76,[0,1,0],radius,width)]]
            put(f"drive-loop-{side}-{index}","chain",cut(capsule(27,5),capsule(24,7,-1)),
                mass=(0.08,0.25),note="Continuous smooth pitch-band envelope; width, teeth, tensioning and torque open")
            for x in (a,b):
                hub=cut(cylinder(x,y,76,[0,1,0],24,5),cylinder(x,y-1,76,[0,1,0],6.1,7))
                put(f"drive-pulley-{side}-{index}-{x}","pulley",hub,material="Al",
                    note="Wheel transfer pitch envelope; actual hub/key and belt technology open")
        bumper_y = -425 if sign<0 else 350
        put("bumper-side-"+side,"bumper",box(-425,bumper_y,40,850,75,150),mass=(1.,2.),
            note="B4 full bumper envelope; construction and attachment hardware unresolved")
    for name,x in (("rear",-350),("front",330)):
        tube=cut(box(x,-325,50,20,650,100),box(x+2,-326,52,16,652,96))
        put("cross-"+name,"frame",tube,material="Al")
        bx=-425 if name=="rear" else 350
        put("bumper-end-"+name,"bumper",box(bx,-350,40,75,700,150),mass=(1.,2.),
            note="B4 full bumper envelope; construction and quick-removal open")
    # Structural lightweight bridge deck terminating on the end rails.
    put("deck-main","plate",box(-346,-258,150,692,516,4),material="Al")
    put("battery","battery",box(-300,-125,155,180,215,130),mass=(5.4,6.5),
        note="Removable battery reserve; exact SKU, terminal orientation and capacity open")
    put("battery-tray","plate",box(-306,-131,154,192,227,1),material="Al")
    put("battery-anchor-L","plate",box(-225,90,155,25,4,130),material="Al")
    put("battery-anchor-R","plate",box(-225,-129,155,25,4,130),material="Al")
    put("battery-restraint","plate",box(-225,-129,285,25,223,3),material="Al",
        note="Removable prototype restraint; detach before extraction. Latch/loads unresolved")
    put("electrical-backplate","plate",box(-294,96,154,181,131,4),material="Al")
    put("protected-distribution","power",box(-285,102,158,160,118,92),mass=(0.8,1.4),
        note="Main protection and distribution allocation; hardware and wiring approval open")
    put("controller-radio","compute",box(10,10,154,125,90,72),mass=(0.5,1.2),
        note="Robot controller/radio combined installation volume; exact articles and cooling open")
    for y in [-235,210]:
        mast=cut(box(-70,y,154,50,25,866),box(-68,y+2,153,46,21,868))
        put("mast-"+str(y),"frame",mast,material="Al")
    put("lift-drive","lift",box(-70,-140,154,95,120,125),mass=(1.1,2.4),
        note="Braked lift drive reserve; reeving, brake and hard stops require selection")
    # A U shoe surrounds each mast with a 1 mm running allowance. Back face sits against crossbar.
    for y in [-235,210]:
        shoe=cut(box(-77,y-5,-148,62,35,58),box(-71,y-1,-149,52,27,60))
        put("lift-shoe-"+str(y),"guide",shoe,"lift",material="PE")
    put("lift-crossbar","frame",box(-97,-240,-148,20,480,21),"lift",material="Al")
    put("slide-base-bridge","plate",box(-147,-205,-127,322,410,5),"lift",material="Al")
    put("slide-drive","slide",box(-58,-90,-168,118,180,41),"lift",mass=(1.2,2.4),
        note="Slide actuator/controller allocation; transmission & actual torque open")
    for side, cy in (("R",-180),("L",180)):
        # Nested U channels. Each rail is open above its floor and bears a separate stage.
        for label,frame,w,bottom,top,wall in (
                ("outer","lift",36,-122,-84,3),
                ("middle","middle",28,-118,-84,3),
                ("inner","slide",20,-114,-81,3)):
            rail=cut(box(-160,cy-w/2,bottom,320,w,top-bottom),
                     box(-161,cy-w/2+wall,bottom+wall,322,w-2*wall,top-bottom))
            put(f"slide-{side}-{label}","guide",rail,frame,material="Al")
        fixture_x=(-80,140)
        put("shelf-spacer-"+side,"plate",cut(box(-95,cy-10,-81,250,20,6),
            *[cylinder(x,cy,-82,[0,0,1],2.65,8) for x in fixture_x]),"slide",material="Al")
        for x in fixture_x:
            insert = cut(box(x-8,cy-7,-88,16,14,7), cylinder(x,cy,-89,[0,0,1],2.65,9))
            put(f"anchor-block-{side}-{x}","plate",insert,"slide",material="Al",
                note="Original crossblock; retained/threaded insert or captured nut and proof load required")
            screw=["union",[cylinder(x,cy,-87,[0,0,1],2.5,24),cylinder(x,cy,-63,[0,0,1],4.25,3)]]
            put(f"shelf-screw-{side}-{x}","fastener",screw,"slide",material="steel",
                note="Simplified Ø5 shank and recessed head. Thread engagement, selection and locking open")
    # Two optical ports in the registered R6 stock; no shelf hole is assumed in the earlier contact receipt.
    ports=[cylinder(x,0,-77,[0,0,1],6,20) for x in (40,100)]
    fasteners=[(x,y) for x in (-80,140) for y in (-180,180)]
    holes=[cylinder(x,y,-77,[0,0,1],2.65,20) for x,y in fasteners]
    holes.extend(cylinder(x,y,-63,[0,0,1],4.6,4) for x,y in fasteners)
    put("receiver-R6-ported","shelf",cut(["shelf",[]],*ports,*holes),"slide",material="PE",
        note="Reopened source shelf; optical/fastener bores are an original machined revision")
    for label,x in (("seat",40),("entry",100)):
        mount=cut(box(x-10,-11,-82,20,22,7),cylinder(x,0,-83,[0,0,1],6,10))
        put(f"optical-mount-{label}","plate",mount,"slide",material="Al")
        put(f"optical-{label}","sensor",box(x-5,-5,-94,10,10,12),"slide",mass=(0.01,0.035),
            note="Upward reflective sensing envelope; water/dirt, range and PVC response unmeasured")
    for side,y in (("R",-170.8125),("L",150.8125)):
        put("belt-cartridge-"+side,"belt",box(-95,y,-42,260,20,70),"slide",mass=(0.6,1.3),
            note="End-rim active belt contact envelope; belt/motor/bearings and preload unselected")
        for x in [-90,136]:
            put(f"belt-riser-{side}-{x}","plate",box(x,y,-60,18,20,18),"slide",material="Al")
    return parts
