"""Machine the side-only wheel supports and the four separate rail-to-sill seats."""
import math
import numpy as np
from inset_chassis_shop import op,rect,circ,boxed,polygon


def add(article,hosts):
    purchases=[]
    faces=[("port",0,0.,[0.,365.]),("rear",2,90.,[-370.,0.]),("starboard",4,180.,[0.,-365.]),("front",5,-90.,[370.,0.])]
    for side,face,degrees,origin in faces:
        angle=math.radians(degrees);r=np.rint([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        wheel=side in ("port","starboard")
        for station in (-127.3,127.3):
            offset=np.array(origin)+r@[station,0.]
            matrix=np.eye(4);matrix[:2,:2]=r;matrix[:2,3]=offset
            def world(x,y,z):
                return [*(r@[x,y]+offset),z]
            def host_holes(jobs,name,points,diam,reach):
                a=int(np.argmax(abs(r[:,1])));direction=-r[a,1];start=points[0][a]
                jobs.append(op(name,"cut",a,start,reach*direction,*[circ(*np.delete(p,a),diam/2) for p in points]))
            tag=f"{side} {station:g}"
            if wheel:
                Z=94.63336287672979
                # A connected machining order makes each joined feature touch existing stock.
                ops=[boxed("Bridge between radial walls",[-25,-71.425,Z],[25,0,101.6]),
                     op("Inner lower jaw","join",2,Z,-(Z-55),rect(-25,-71.425,25,-59.425)),
                     op("Outer lower jaw","join",2,Z,-(Z-55),rect(-25,-34.025,25,-29.025)),
                     op("Upper fitted jaw","join",2,101.6,57.4,rect(-25,-58.45,25,-38.1))]
                for sg in (-1,1):
                    ops.append(op("Corner motor relief "+str(sg),"cut",2,54,48.,polygon([[sg*15,-72.425],[sg*26,-72.425],[sg*26,-61.425]])))
            else:
                ops=[op("Machine closed-section end seat","new",2,101.6,57.4,rect(-25,-58.45,25,-38.1))]
            for kind,z,span,start,far,depth,sku in ([("lower",65,13,-29.025,-59.425,25.4,"91251A351"),("lower",85,13,-29.025,-59.425,25.4,"91251A351")] if wheel else [])+[("upper",121,6,0.,-38.1,38.1,"91251A354"),("upper",149,13,0.,-38.1,38.1,"91251A354")]:
                if kind=="lower":
                    ops.append(op("Clear lower heads "+str(z),"cut",1,start+.2,-6.,circ(-span,z,2.65),circ(span,z,2.65)))
                    role=("Port" if side=="port" else "Starboard")+" longitudinal socket rail"
                    host=hosts[("socket",role,int(np.sign(offset[0])))]
                    sleeve_start=-34.025
                else:
                    host=hosts[("rim",face)];sleeve_start=0.
                ops.append(op("Tap drill "+str(z),"cut",1,far+.1,-(13. if kind=="lower" else 21.2),circ(-span,z,2.05),circ(span,z,2.05)))
                host_holes(host,tag+" sleeve seats "+str(z),[world(ds,sleeve_start+.1,z) for ds in (-span,span)],8.10,depth+.2)
                for ds in (-span,span):
                    sleeve=[op("Turn fitted anti-crush sleeve","new",1,sleeve_start-.04,-(depth-.08),circ(ds,z,4.)),
                            op("Bore clearance bore","cut",1,sleeve_start+.06,-(depth+.12),circ(ds,z,2.65))]
                    article(f"Sleeve {tag} {ds} {z}",sleeve,"wheel and sill sleeve",matrix.tolist())
                    purchases.append(dict(sku=sku,name=f"{tag} screw {ds} {z}",underHead=world(ds,start,z),direction=[*(-r[:,1]),0.],host=f"Frame support {tag}"))
            article("Frame support "+tag,ops,"wheel saddle" if wheel else "sill seat",matrix.tolist())
    return purchases
