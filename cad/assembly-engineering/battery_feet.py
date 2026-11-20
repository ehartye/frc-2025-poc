"""Eight bolted feet connect all four positive side stops to the tray and structural deck."""
from attachments import base, bounds, cyl, cut, fuse, hexagon, tube
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
def apply(robot,repl,added,tools):
    deck=repl["host-drilled-swerve-deck"]
    tray=repl["battery-tray"]
    stations={}
    angle_sections={
       "battery-cheek-left":((-251,76,155,-173,90,158),[(-240,84),(-184,84)]),
       "battery-cheek-right":((-251,-119,155,-173,-105,158),[(-231,-113),(-184,-113)]),
       "battery-side-stop-in":((-264,-105,155,-250,76,158),[(-259,-82),(-259,53)]),
       "battery-side-stop-out":((-174,-105,155,-160,76,158),[(-165,-82),(-165,53)])}
    for name,(section,points) in angle_sections.items():
        leg=repl.get(name,robot[name])
        angle=fuse(leg,base.block(section))
        for index,(x,y) in enumerate(points):
            key=f"{name}-foot-{index}"
            lower=round(bounds(BRepAlgoAPI_Common(deck,cyl(x,y,131,.2,25)).Shape())[2],4)
            clearance=cyl(x,y,lower-1,2.25,162-lower)
            angle=cut(angle,clearance);tray=cut(tray,clearance);deck=cut(deck,clearance)
            # ISO 4762 socket cap screw and steel washer/locking nut. Stage entire
            # deck before lowering onto chassis. Gauge actual lot for thread runout.
            screw_length=16 if lower>=150 else 30
            added[key+"-screw"]=fuse(cyl(x,y,158-screw_length,2,screw_length),cyl(x,y,158,3.5,4))
            added[key+"-washer"]=tube(x,y,lower-.8,9,4.3,.8)
            added[key+"-locknut"]=cut(hexagon(x,y,lower-4.8,7,4),cyl(x,y,lower-5.1,2,5))
            tools[key+"-hex-key"]=cyl(x,y,162,1.80,30)
            tools[key+"-socket-before-deck"]=tube(x,y,lower-35,11.5,8.4,34.2)
            stations[key]=dict(x=x,y=y,deckBottomMm=lower,screwLengthMm=screw_length,
                protrusionMm=round(lower-4.8-(158-screw_length),3))
        repl[name]=angle
    repl["host-drilled-swerve-deck"]=deck;repl["battery-tray"]=tray
    return stations

