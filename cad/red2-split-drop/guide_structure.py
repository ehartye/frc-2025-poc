"""Screen finite welded side-panel geometry at the attained lateral strike."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/split-drop-native'
def read(p): return json.loads(p.read_bytes())
def ref(p): return dict(file=p.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest())

def main():
    register=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert ref(ROOT/register['file'])==register
    receipt=OUT/'r4-spatial-moving-cheeks-challenge.json'
    source=read(receipt)
    assert source['source']==register['sha256'] and source['fixture']==ref(OUT/'red2-split-drop-fixture-r4.f3d')['sha256']
    impact=max(r['guideForceN'] for r in source['runs'].values())
    # Each panel is boxed on the OUTSIDE of the pipe path. Dimensioned ribs
    # span from the inner jaw's outboard face to the outer weld toe.
    ribs=[-61,4,70,136,202,253,286,309]
    span=max(b-a for a,b in zip(ribs,ribs[1:]))
    rootHeight,baseWidth,skinT,E=150.,42.,5.,69000.
    # A 150 mm longitudinal face intercepts an off-centre strike. Challenge small
    # load distribution widths; supplier PVC cut-edge geometry does not guarantee 10mm.
    results=[]
    for factor in (1.,1.5):
        force=impact*factor
        for strip in (3.,5.,10.,15.):
            inertia=strip*skinT**3/12
            delta=force*span**3/(48*E*inertia)
            stress=force*span*skinT/(8*inertia)
            results.append(dict(loadFactor=factor,forceN=force,loadedStripWidthMm=strip,
                        simplySupportedPanelDeflectionMm=delta,panelBendingMPa=stress,
                        postWeldYieldMarginMPa=105-stress))
    # Eight 6 mm-thick triangular transverse webs; assign ALL of the load to one.
    # A cantilever root of depth 42 mm transfers overturning to the machined rail.
    design=1.5*impact
    webWidth=6
    rootI=webWidth*baseWidth**3/12
    rootBending=design*rootHeight*(baseWidth/2)/rootI
    couple=design*rootHeight/baseWidth
    # The 38mm overhang connecting thin fence to jaw is a boxed shelf. A bare
    # 6mm ledge fastened to the 12mm-high receiving rail is a separate failure case.
    ledge=38.
    bareSection=webWidth*6**2/6
    bareStress=couple*ledge/bareSection
    # Pair of vertical fillets on the 12 mm-high web vs mounting on the tall
    # upright: approximate the weld group across two transverse root flanges.
    attachments=[]
    for depth in (12.,45.,60.):
        weld=3*.7071
        inertia=2*weld*depth**3/12
        bending=design*rootHeight*(depth/2)/inertia
        shear=design/(2*weld*depth)
        attachments.append(dict(verticalWeldLengthMm=depth,bendingAtThroatMPa=bending,directShearMPa=shear,
                                vonMisesApproxMPa=(bending*bending+3*shear*shear)**.5))
    path=OUT/'guide-structure.json'
    data=dict(authority=register,fixture=source['fixture'],dynamicTrace=ref(receipt),
        geometry=dict(candidate='Two removable native jaw weldments with a 5 mm laser-cut 6061-T6 panel, outside ribs, upper returns and a closed lower spine. Shop process and stock must be qualified.',
                      sheetMinToMaxXmm=[-65,315],sheetHeightMm=rootHeight,innerFaceAbsYmm=160,
                      sheetThicknessMm=skinT,ribCentersXmm=ribs,ribPitchMaxMm=span,ribThicknessMm=webWidth,
                      freeStraightLateralSlackMm=160-150.8125,closedJawOuterAbsYmm=122,
                      nominalFullWidthRailCoverageReserveAfterWallContactMm=150.8125-122-(160-150.8125),
                      sheetPairUntrimmedMassKg=2*.38*.15*.005*2700),
        load=dict(maximumSimulatedOneCheekN=impact,designScreenN=design,hazYieldMPa=105,
                  elasticModulusMPa=E,rootWebBendingMPa=rootBending,rootCoupleN=couple,
                  unbraced38mmShelfStressMPa=bareStress,skinStrips=results,weldGroup=attachments),
        duty='Candidate 6061-T6 with ER5356. Compare to 105 MPa as-welded yield for this material/filler condition. A 6061-T6 base-metal yield claim is insufficient after welding.',
        risks=['Load width at the first physical pipe contact is unverified. A thin contact strip can exceed post-weld yield. An actual rounded contact face, local structural solve and witness loading must close it.',
               'The original 12mm-high rail and single horizontal shelf cannot carry this couple. Introduce a deep rail spine or transfer to the two tall guide shoes with measurable welds.',
               'The free rectangular dimensions omit the tapered sections and ribs. Upper clearance, drive CG and serviced mass remain open.',
               'Socket access and guide edges, stowed field boundary, off-center exits, continuous complete installed clearances and material-specific weld fatigue still need measurement.'])
    path.write_text(json.dumps(data,indent=2)+'\n')
    print('One cheek',impact,'N, two bare sheets',data['geometry']['sheetPairUntrimmedMassKg'],'kg; shelf failure',bareStress,'MPa')
if __name__=='__main__': main()
