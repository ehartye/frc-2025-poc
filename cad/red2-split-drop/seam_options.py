"""Compare two supplier-supported tread stocks and two genuine fastener lengths."""
import argparse
import math
import sys
from pathlib import Path
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt,gp_Trsf
from tread_fit import OUT,ROOT,read,sha
import json

def main(assets,plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds,common
    from showcase_contacts import load_solids
    original=read(OUT/'tread-retention-fit.json')
    assert original['authority']['sha256']==sha(ROOT/original['authority']['file'])
    hubfile=assets/'REV-21-3002.STEP'
    assert sha(hubfile)==original['sourceWheel']
    hub=load_solids(hubfile)[0]['REV-21-3002'];rows=[]
    drive=read(ROOT/'experiments/swerve-drive/motion/model.json')['drive']
    torque=drive['stallTorqueNm']*drive['phaseLimitA']/drive['stallCurrentA']*drive['ratio']*drive['efficiency']
    for sku,sfile,length,digest in [('REV-29-2943',assets/'REV-29-2943.STEP',12.7,original['sourceScrew']),
                              ('REV-29-2941',OUT/'supplier/REV-29-2941.STEP',9.525,'079855e55b1e988fa405052d54693b370dc525974099f643a5b23d01f21ba58c')]:
        assert sha(sfile)==digest
        screw=next(iter(load_solids(sfile)[0].values()))
        bb=bounds(screw)
        seat=bb[2]+length
        for belt,gauge,rated in [('5994K9 blue Buna-N',7.874,150),('5994K1 black neoprene',7.62,225)]:
            for compression in (0,.8,1.5):
                free=30.480+gauge;loaded=free-compression
                recess=free+(bb[5]-seat)-math.sqrt((loaded-.25)**2-bb[3]**2)
                baseR=free-recess;tip=baseR-length
                pilot=[];total=[]
                for x,dy,dz in original['fourChosenSeamHoles']:
                    n=baseR-seat
                    mat=gp_Trsf();mat.SetValues(1,0,0,x,0,dz,dy,n*dy,0,-dy,dz,n*dz)
                    placed=BRepBuilderAPI_Transform(screw,mat,False).Shape()
                    probe=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,dy*tip,dz*tip),gp_Dir(0,dy,dz)),1.5,30.480-tip).Shape()
                    pilot.append(common(probe,hub));total.append(common(placed,hub))
                scale=4.4482216152605/25.4
                # The supplier rating is for whole cloth. Linear net-width scaling is
                # deliberately an optimistic screen and cannot qualify a drilled joint.
                rows.append(dict(fastener=sku,pinnedScrewSha256=digest,belt=belt,loadedRadiusMm=loaded,
                    estimatedCompressionMm=compression,headRecessMm=recess,actualMinorBoreBottomMm3=pilot,
                    requestedEngagementMm=30.480-tip,nominalPitchIntervalsEngaged=(30.480-tip)/(.79375),
                    remainingBeltThicknessMm=gauge-recess,headCommonIncludingExpectedThreadMm3=total,
                    motionModelTangentialDemandN=torque/(loaded/1000),singleEndTwoScrewTangentialDemandEachN=torque/(2*loaded/1000),
                    intactStripRatedTensionN=rated*21.59*scale,netWidthTwo5mmHolesScreenN=rated*(21.59-10)*scale,
                    netWidthTwoHeadCounterboresScreenN=rated*(21.59-4*bb[3])*scale,
                    minimumLigamentOutsideHeadsMm=21.59/2-4.7625-bb[3],ligamentBetweenHeadsMm=9.525-2*bb[3]))
    result=dict(authority=original['authority'],sourceWheel=sha(hubfile),stockSources=[
        'https://docs.revrobotics.com/ion-build/motion/maxswerve/aluminum-maxswerve-wheels',
        'https://www.mcmaster.com/products/slip-top-belts/','https://www.revrobotics.com/10-32-Button-Head-Hex-Drive-Screws/'],
        drive=drive,outputTorqueNm=torque,receipts=rows,
        scope='The belt fabric working-tension rating applies to intact stock. Individual cut-ply bearing, tension concentration, clamp friction, fatigue and true compression are unmeasured. All four actual hub bores and supplier STEP screws were checked. A positive bottom-probe means the screw cannot seat without crushing or modifying the hub.',
        selection='Keep the half-inch tread joint as a bottoming control. Prototype only a shorter 3/8-inch screw with the stronger manufacturer-listed black neoprene and a verified formed recess preserving reinforcement. A cut-through circular counterbore has no acceptable substantiated margin. Hold final drive selection pending actual ply depth, pull-through strength, compression and assembled running radius.')
    (OUT/'seam-options.json').write_text(json.dumps(result,indent=2)+'\n')
    for r in rows:
        if r['estimatedCompressionMm']==1.5:print(r['fastener'],r['belt'],round(max(r['actualMinorBoreBottomMm3']),4),round(r['requestedEngagementMm'],3),round(r['motionModelTangentialDemandN'],1),round(r['netWidthTwoHeadCounterboresScreenN'],1))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('assets',type=Path);parser.add_argument('plugin',type=Path)
    a=parser.parse_args();main(a.assets,a.plugin)
