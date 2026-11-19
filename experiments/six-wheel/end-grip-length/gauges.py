"""Three pairs of indexed stop dimensions, endpoint fits and conservative added mass."""
import json
from cassette import HERE, built, cad, finalize_step, mech
import hashlib
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

def build():
    rows={};exports={}
    for j,(label,cut) in enumerate((("short",-6.35),("nominal",0.),("long",6.35))):
        center=cut/2
        fixed,_=built(center,center,index=center)
        _,inside=built(center-.35,center-.35,index=center)
        _,outside=built(center+1.40,center+1.40,index=center)
        measures={}
        for side in "LR":
            lug="grip-"+side+"-moving-stop-lug"
            foot=fixed["grip-"+side+"-front-support-foot"]
            for face,at in (("inboard",inside),("outboard",outside)):
                k="grip-"+side+"-"+face+"-hardstop"
                part=fixed[k];p=cad.bounds(part)
                patch=mech.block(p[0],p[1],p[2]-.1,p[3]-p[0],p[4]-p[1],.1)
                pair=BRepExtrema_DistShapeShape(part,at[lug])
                measures[k]={"firstTouchGapMm":pair.Value(),"endpointOverlapMm3":cad.intersection(part,at[lug]),
                             "supportedFootprintMm2":cad.intersection(patch,foot)/.1}
                exports[label+"-"+k]=cad.translated(part,(400*j,0,0))
        rows[label]={"cutMm":cut,"spacerAddedInMm":center+5.65,"spacerAddedOutMm":4.60-center,"stopFaces":measures}
    path=HERE/"gauge-stops.step"
    tags=[{"id":k,"group":"gauge"} for k in exports]
    cad.write_named(path,list(exports.values()),tags,{"gauge":"#778d8e"})
    finalize_step(path)
    opened=cad.read_named(path,tags)
    return {"nativeSha256":hashlib.sha256(path.read_bytes()).hexdigest(),"reopenedSolidCount":sum(cad.solid_count(v) for v in opened.values()),
            "benchDisplayPitchXmm":400,"maximumFourSpacerMassKgSteel":2*4*16*(5.65+4.6)*7.85e-6,"sets":rows}
if __name__=="__main__":
    (HERE/"gauges.json").write_text(json.dumps(build(),indent=2)+"\n",encoding="utf-8",newline="\n")
