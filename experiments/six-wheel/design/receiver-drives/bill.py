"""Per-body estimated mass and procurement description. Original CAD proxies carry no vendor geometry."""
import csv
from receiver_drives import HERE,REPLACES,parts,cad

def item(key,kind,shape):
    if "NEO550" in key:return ("REV-21-1651 NEO 550 brushless motor",.142,.142)
    if "UP5" in key:return ("REV UP 550 plate, output, 68:13; slide also 76:21",.095,.22)
    if "SP1-25" in key:return ("TE/Celesco SP1-25 25-in analog string pot with bracket",.12,.19)
    if "420-3M" in key:return ("Gates PowerGrip HTD 420-3M-15 (9293-90008)",.017,.017)
    if "480-3M" in key:return ("Gates PowerGrip HTD 480-3M-9 (9293-40356)",.013,.013)
    if "spring" in key:return ("Matched compression spring, 1 N/mm; calibrate force at installed length",.002,.007)
    if "angle-board" in key:return ("ams AS5600 with on-axis magnet, original bracket/PCB allocation",.004,.020)
    if "switch" in key:return ("Independent normally-closed limit switch and positive-travel lever reserve",.005,.020)
    if kind=="measurement-line":return ("Measuring cable included in SP1 body mass",0,0)
    if "pulley" in key or "20T" in key:
        return ("Custom 20T genuine HTD 3M section pulley and split hub, pitch registered",.008,.028)
    if "bearing" in key and kind=="bearing-envelope":return ("Two-direction 5 mm bore × 13 × 5 mm bearing",.003,.007)
    material="6061-T6 Al"
    density=2.7
    if kind in ("shaft-envelope","involute-section","rack-section","fastener-envelope","spur-pitch-envelope","guide-envelope"):
        material="steel";density=7.85
    if "magnet" in key:material="diametric magnet";density=7.5
    if "backing-strip" in key:material="UHMWPE belt wear strip";density=.94
    mass=cad.volume(shape)[0]*density/1e6
    return ("Original "+material+" — "+kind,max(.00005,.85*mass),1.10*mass)


def rows():
    return [(key,frame,kind,*item(key,kind,shape)) for key,(frame,kind,shape) in parts().items()]


def write():
    with (HERE/"bom.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["id","frame","representation","description","lowMassKg","highMassKg"])
        for key,frame,kind,name,lo,hi in rows():w.writerow([key,frame,kind,name,round(lo,5),round(hi,5)])
    return {frame:round(sum(q[-1] for q in rows() if q[1]==frame),4) for frame in ("lift","middle","slide")}

if __name__=="__main__":print(write())
