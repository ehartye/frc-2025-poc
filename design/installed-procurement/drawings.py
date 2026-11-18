"""Planar hole schedules and 1:1 DXF shop patterns for new steel stock."""
import csv
from pathlib import Path

HERE=Path(__file__).resolve().parent
# U increases from rear edge toward robot front. V increases upward from bottom.
SHEETS={
    "end-inner-carrier":(56,52,6,4,[("bearing-seat",28,26,28.62)]
        +[("bolt-clearance",u,v,5.3) for u in (9,47) for v in (9,43)]),
    "center-faceplate":(96,66,5,2,[("drive-clearance",48,33,36),("window",16,33,14),("window",80,33,14)]
        +[("gear-screw-clearance",48,v,5.3) for v in (7.6,58.4)]
        +[("tap-M5-carrier",u,v,4.2) for u in (29,67) for v in (16,50)]
        +[("tap-M5-rail",u,v,4.2) for u in (8,88) for v in (21,45)]),
    "square-threaded-block":(10,10,6,16,[("tap-M5",5,5,4.2)])}


def pattern(name,width,height,holes):
    pairs=[]
    def add(*values):pairs.extend(str(v) for v in values)
    add(0,"SECTION",2,"HEADER",9,"$INSUNITS",70,4,0,"ENDSEC",0,"SECTION",2,"ENTITIES")
    def polygon(layer,points):
        for (x,y),(xx,yy) in zip(points,points[1:]+points[:1]):
            add(0,"LINE",8,layer,10,x,20,y,30,0,11,xx,21,yy,31,0)
    polygon("STOCK",[(20,0),(76,0),(96,20),(96,46),(76,66),(20,66),(0,46),(0,20)]
            if name=="center-faceplate" else [(0,0),(width,0),(width,height),(0,height)])
    if name=="end-inner-carrier":
        for u in (9,47):
            for v in (9,43):
                corners=[(u+x,v+y) for x,y in ((-5.1,-5.1),(5.1,-5.1),(5.1,5.1),(-5.1,5.1))]
                polygon("POCKET_3P75",corners)
                for a,b in corners:add(0,"CIRCLE",8,"RELIEF_3P75",10,a,20,b,30,0,40,1)
    for operation,u,v,diameter in holes:
        add(0,"CIRCLE",8,operation,10,u,20,v,30,0,40,diameter/2)
    add(0,"ENDSEC",0,"EOF")
    (HERE/(name+".dxf")).write_text("\n".join(pairs)+"\n",encoding="ascii",newline="\n")


def generate():
    with (HERE/"machining.csv").open("w",encoding="utf-8",newline="") as file:
        out=csv.writer(file,lineterminator="\n")
        out.writerow(["part","quantity","blankU","blankV","thickness","operation","centerU","centerV","diameter","depth"])
        for name,(w,h,t,quantity,holes) in SHEETS.items():
            for op,u,v,d in holes:out.writerow([name,quantity,w,h,t,op,u,v,d,"through"])
            if name=="end-inner-carrier":
                for u in (9,47):
                    for v in (9,43):
                        out.writerow([name,quantity,w,h,t,"square-pocket-10.20",u,v,"",3.75])
                        for du in (-5.1,5.1):
                            for dv in (-5.1,5.1):
                                out.writerow([name,quantity,w,h,t,"corner-relief",u+du,v+dv,2,3.75])
            pattern(name,w,h,holes)


if __name__=="__main__":generate()
