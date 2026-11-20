"""Geometry of the 560-mm-only chassis with mechanically retained battery.

All coordinates are robot mm. Keep the entire upper contact face on the main deck.
Only equip this variant for its fixed-height receiver and electric shallow jack.
"""

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS


def apply(robot,repl,added,tools,x,full):
    from attachments import cut, fuse, cyl, tube, hexagon, base, bounds, volume
    def block(*b):
        return base.block(list(b))
    def stock(name):return repl.get(name,robot[name])
    details={}
    # Resolve both 600 mm AABBs into the actual discrete material intersections.
    # No 600 mm-wide cut is made; use the design cutter on each finite solid.
    # The upper deck skin stays intact between ribs and all electronics interfaces.
    deck=stock("host-drilled-swerve-deck")
    plate=added["front-electronics-platform"]
    # Leave a 1.4 mm outer skin and four full-depth cross bands. All original
    # mounting locations remain on the top face. The lamp/guide tab stays 3 mm.
    for x0,x1 in ((12,107),(115,221),(229,325)):
        for y0,y1 in ((-199,-108),(-100,-4),(4,100),(108,199)):
            plate=cut(plate,block(x0,y0,289,x1,y1,291.60))
    added["front-electronics-platform"]=plate
    # Dead service-height stock can be removed after choosing the 560-only machine.
    removed=["rev-ph"]
    for side in "LR":
        rail="backup-"+side+"-service-pin-rail"
        repl[rail]=cut(robot[rail],block(-126,-295,424,-110,295,1030))
        # Keep 34 mm of stock above the one retained pin at Z395.
        for tail in ("service-pin-bushing-670","service-pin-bushing-940","service-rail-arm-891",
                     "mast-bolt-886","mast-bolt-911","mast-spacer-886","mast-spacer-911"):
            removed.append("backup-"+side+"-"+tail)
        mast="guide-"+side+"-machined-mast"
        repl[mast]=cut(robot[mast],block(-75,-242,827,-16,242,1030))
    # Two captive tie rods outside the battery's end faces, on the cross bar
    # centerline. The steel rods also register the removable tray to the deck.
    bar="battery-hold-down"
    top=block(-224,-122,322,-200,92,327)
    tray=robot["battery-tray"]
    for suffix,y in (("rear",-116),("front",86)):
        name="battery-"+suffix
        x=-212
        top=cut(top,cyl(x,y,321,2.75,8))
        tray=cut(tray,cyl(x,y,153,2.75,4))
        bottom=round(bounds(BRepAlgoAPI_Common(deck,cyl(x,y,134,.25,22)).Shape())[2],4)
        deck=cut(deck,cyl(x,y,bottom-1,2.75,156-bottom))
        # 200mm stock rods trimmed after assembly, 2mm thread beyond the nylon.
        added[name+"-tie-rod"]=cyl(x,y,bottom-8,2.5,335.8-bottom+8)
        for end,z in (("lower",bottom-1),("upper",327)):
            added[name+"-"+end+"-washer"]=tube(x,y,z,12,5.4,1)
            nut_z=bottom-6 if end=="lower" else 328
            added[name+"-"+end+"-locknut"]=cut(hexagon(x,y,nut_z,8,5),cyl(x,y,nut_z-1,2.5,7))
        tools[name+"-top-socket"]=tube(x,y,328,13,10,50)
        tools[name+"-lower-socket"]=tube(x,y,bottom-45,13,10,44)
    repl[bar]=top
    repl["battery-tray"]=tray
    # The native chassis deck already has a ribbed 1.7 mm skin. Leave those
    # cells intact. Relieve the currently solid receiver bridge at four locations
    # between the crossbar, the fixed slide rails and the cartridge hardpoints.
    bridge_id="support-machined-bridge"
    bridge=robot[bridge_id]
    windows=[]
    for x0,x1 in [(-133,-103),(-69,-12),(2,78),(91,151)]:
        for y0,y1 in [(-159,-130),(-107,-80),(-50,-9),(9,29),(57,96),(112,157)]:
            # Keep local through-attachment material (including the two stowed
            # absorber interfaces) and a 12 mm perimeter or intermediate web.
            if x0==2 and -108<=y0<-70:continue
            if x0==91 and y0 in (-107,-50,9,57):continue
            if x0==-69 and y0 in (-159,-107):continue
            if x0==2 and y0==-159:continue
            if x0==91 and y0==112:continue
            # Vertical CNC pocket with R4 corners and a 1.8 mm upper skin.
            z=432.5
            depth=3.7
            tool=fuse(block(x0+4,y0,z,x1-4,y1,z+depth),block(x0,y0+4,z,x1,y1-4,z+depth))
            for x in (x0+4,x1-4):
                for y in (y0+4,y1-4):tool=fuse(tool,cyl(x,y,z,4,depth))
            before=volume(bridge)
            bridge=cut(bridge,tool)
            windows.append({"boundsMm":[x0,y0,x1,y1],"removedMm3":before-volume(bridge)})
    repl[bridge_id]=bridge
    # The lower corner stop blocks on the aluminum tray supply lateral restraint;
    # keep 45 mm high of each end cheek with a folded/coined upper edge. A positively
    # fastened top bar and end rods take uplift, independently of cheek height.
    for k in ("battery-cheek-left","battery-cheek-right"):
        repl[k]=cut(robot[k],block(-254,-110,200,-170,81,325))
    repl["host-drilled-swerve-deck"]=deck
    details.update(removed=removed,pocketsUndersideMm=[],bridgePockets=windows,batteryRodBottomDatum="native deck underside",
        fixedPriority="The selected receiver stays at H560 throughout this variant. Both lower lock arms and both Z395 backup bores remain. Raising to H670 or H940 requires the prior service rails and fixture; this build physically gives up that adjustment.")
    from battery_feet import apply as bolt_stops
    details["batteryFeet"]=bolt_stops(robot,repl,added,tools)
    return removed,details
