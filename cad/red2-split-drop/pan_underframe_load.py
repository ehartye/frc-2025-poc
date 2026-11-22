"""Size the lower-driven tower attachment over explicit torque scatter and geometry tolerance."""
import json
import math
from output_mount import ROOT,digest,read

MPA_PER_KSI=6.894757293168


def solve():
    root=ROOT/'artifacts/red2-integrated/pan-thrust'
    fit=read(root/'underframe-brace-fit.json')
    duty=read(root/'r3-brace-section-strength.json')
    current=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    if digest(ROOT/current['file'])!=current['sha256'] or fit['authority']!=current or duty['authority']!=current:
        raise ValueError('Changed engineering authority')
    d=.190*25.4;p=25.4/32.
    area=math.pi/4*(d-.9743*p)**2
    pitch_d=d-.64952*p
    min_root=3.72
    nominal=3600.;thread_mu=.18
    # Use full effective external tensile force, with no fraction assigned to
    # the member's stiffness; retain all thread torque and concurrent transverse load.
    lead=math.atan(p/(math.pi*pitch_d))
    cases={}
    for key,j in fit['cases'].items():
        loads=duty['stations'][key]['factoredN']
        up,shear,down=(loads[m] for m in ['upliftN','torsionInclusiveShearN','compressionN'])
        cross=key.endswith('cross')
        # Tube outside/stock/spotface tolerance affects only axial engagement;
        # (±1.0 tube depth, ±0.20 bridge thickness, ±0.15 nut seat, ±0.30 screw length + runout).
        uncertainty=1.0+.20+.15+.30
        engaged=j['screwPenetrationIntoNutMm']-uncertainty-2*p
        tip_gap=j['screwPointBelowNutTopMm']-uncertainty
        samples={}
        for variation in [.25,.35]:
            low=nominal*(1-variation)*.8
            high=nominal*(1+variation)
            pull=high+up
            torsion=high*pitch_d/2*math.tan(lead+math.atan(thread_mu/math.cos(math.pi/6)))
            tau=16*torsion/(math.pi*min_root**3)
            eq=math.hypot(pull/area,math.sqrt(3)*(tau+shear/area))
            seat_load=max(pull,high+down)
            support_centers=j['tubeWidth']-j['wall']
            # The machined bottom bridge has physical web-aligned lands. Its
            # lower-bound width uses only the real head diameter at the load;
            # a 30-degree through-thickness spreading screen is listed separately.
            t=6.5
            narrow=7.925
            strip_width=narrow+2*t*math.tan(math.pi/6)
            def strip_stress(width,thick,span,force_width,force):
                return (6/width/thick**2)*force*(2*span-force_width)/8
            # Bounds shown for an actual narrow load strip and a 30° width
            # estimate, rather than asserting every adjacent mm shares equally.
            bottom_narrow=strip_stress(narrow,t,support_centers,0.,seat_load)
            bottom_spread=strip_stress(strip_width,t,support_centers,0.,seat_load)
            # Upper plate has a reduced 5.7 mm section in the nut pocket.
            upper_effective=5.7
            major_width=16. if cross else 10.
            lateral_width=10. if cross else 16.
            upper_narrow=strip_stress(lateral_width,upper_effective,support_centers,0.,seat_load)
            upper_spread=strip_stress(lateral_width+2*upper_effective*math.tan(math.pi/6),upper_effective,
                           support_centers,min(major_width,support_centers),seat_load)
            # A short socket and contained tip give a true usable screw section at all tolerated tube depths.
            samples[str(variation)]=dict(preloadAfterSettlingN=low,maximumInitialClampN=high,
                   clampAfterFactoredUpliftN=low-up,combinedScrewRequiredProofMPa=eq,
                   retainedThreadTorqueNm=torsion/1000,screwTensileMinCertificateN=pull,
                   approximateExternalTorqueUpperNm=(torsion+.18*high*3.9625)/1000,
                   lowerHeadContactMPa=seat_load/j['screwHeadLandMm2'],topInsertContactMPa=seat_load/j['upperNutLandMm2'],
                   broadLowerHostContactMPa=seat_load/j['lowerLandMm2'],bottomBridgeNarrowStripMPa=bottom_narrow,
                   bottomBridge30degSpreadMPa=bottom_spread,existingUpperFootNarrowStripMPa=upper_narrow,
                   existingUpperFoot30degSpreadMPa=upper_spread,
                   internalThreadHalfPitchShearMPa=pull/(.5*math.pi*3.8*engaged),
                   radialPilotOnOneWallMPa=shear/(6.95*j['wall']),
                   screwHeadOnBottomBarProofFloorMPa=105*MPA_PER_KSI,
                   lowerBackingProofFloorMPa=105*MPA_PER_KSI)
        cases[key]=dict(measuredJoint=j,minFullyEngagedMm=engaged,minTipBelowNutTopMm=tip_gap,
             minUpperNutToLowerWallAwayMm=(471-(j['bottom']+j['wall']))-1.35,
             geometryToleranceMm=dict(tubeDepth=1.,backing=.2,seat=.15,screwLength=.30),tests=samples)
    result=dict(authority=current,sourceFitSha256=digest(root/'underframe-brace-fit.json'),dutySha256=digest(root/'r3-brace-section-strength.json'),
         screw='McMaster 91251A351 as represented by original SDS 2023 purchased hardware; A354 is the longer geometric control.',
         basis='Current exact part catalog states 170 ksi ultimate and offers a material certificate. No derived proof fraction. Compare the calculated required proof directly with the purchased certificate.',
         nominalInstallationClampN=nominal,assumedTubeAndTopPlateWeldedYieldMPa=103.,sensitivity='NASA 5020A Table 3 torque control ±25% lubricated/±35% as received used as comparison; additional 20% assumed preload loss applied to minimum.',
         cases=cases,limits='Upper plate bending over the hollow tube, local wall contribution, end welds, thread assembly, antirotation, and clamp capacity must close. The analytical strip results are screens, not a validated 3D stress solution; actual load-spreading assumptions are explicit.')
    (root/'underframe-brace-strength.json').write_text(json.dumps(result,indent=2)+'\n')
    for metric in ['clampAfterFactoredUpliftN','combinedScrewRequiredProofMPa','existingUpperFootNarrowStripMPa','existingUpperFoot30degSpreadMPa','bottomBridgeNarrowStripMPa','topInsertContactMPa','approximateExternalTorqueUpperNm']:
        vals=[(j['tests']['0.35'][metric],s) for s,j in cases.items()]
        print(metric,(min if metric.startswith('clamp') else max)(vals))
    print('min engagement',min(j['minFullyEngagedMm'] for j in cases.values()))


if __name__=='__main__':solve()
