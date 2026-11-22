"""Compare the tube-spool proposal under full-clock loads before detailed machining."""
import json
import math
from output_mount import ROOT,digest,read


def solve():
    p=ROOT/'artifacts/red2-integrated/pan-thrust'
    duty=read(p/'r3-brace-section-strength.json')
    prior=read(p/'underframe-brace-strength.json')
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert authority==prior['authority']==duty['authority'] and digest(ROOT/authority['file'])==authority['sha256']
    rows={}
    for name,station in duty['stations'].items():
        cross=name.endswith('cross')
        OD=18. if cross else 13.5
        tube=25. if cross else 30.;skin=2.5 if cross else 2.
        clear_width=20. if cross else 14.
        # Bias the registration/reaming allowance to the worse side; welded
        # bosses are faced after welding. OD stock and dressed bore tolerances
        # and the effective WPS throat are separate entries on the traveler.
        eccentricity=.35;radial_leeway=.05
        bore=7.1;edge=OD/2-eccentricity-radial_leeway
        shell_area=math.pi*(edge**2-(bore/2+radial_leeway)**2)
        s=station['factoredN'];rise=s['upliftN'];down=s['compressionN'];H=s['torsionInclusiveShearN']
        nominal_low=3600*.65*.8;high=3600*1.35
        full_bearing=high+max(rise,down)
        # Upper bearing is a 16×10 stadium with the centre pilot removed. Its
        # total extremity is 8 mm. Worst outer overhang under a source top seat
        # is loaded as a short cantilever strip with the welded HAZ floor.
        # Lower bound: every circular sector through this annulus lies inside
        # the stadium only through radius 5. Use exact area integral below
        # between the reduced support radius and the stadium, clipped by bore.
        def both_seat_area(r):
            # Integrate x slices of the symmetric intersecting disc and stadium.
            def height(x):
                disk=math.sqrt(max(0.,r*r-x*x))
                if x<=3:return min(disk,5.)
                return min(disk,math.sqrt(max(0.,25.-(x-3)**2)))
            step=min(8.,r)/8192
            # Both heights are monotone decreasing, so right endpoints give a lower bound.
            return 4*step*sum(height(i*step) for i in range(1,8193))-math.pi*(bore/2+eccentricity+radial_leeway)**2
        top_land=both_seat_area(edge)
        bottom_land=math.pi*(min(edge,7.)**2-(2.7+eccentricity+radial_leeway)**2)
        topP=full_bearing/top_land
        over=max(0.,8.-edge)
        foot_bending=3*topP*over**2/5.7**2
        # A single full circumferential weld is charged ALL external axial
        # load plus nearby screw shear. A positive second ring supplies the
        # opposite shear of the e/h force couple. Compare vector traction
        # magnitude to the lower HAZ shear yield (2× load already applied).
        throat=1.25 if cross else 1.0
        r=OD/2
        area=2*math.pi*r*throat
        largest_axial=max(rise,down)
        w_axial=largest_axial/area+largest_axial*eccentricity/(math.pi*r*r*throat)
        w_shear=H*(1+7/(tube-skin))/area
        w_traction=math.hypot(w_axial,w_shear)
        # Width to both sidewalls determines local skin strip bridging. The
        # load at the boss is distributed around a weld ring. Two 45-deg arcs
        # facing each web give only the central chord of that wall effective
        # width. Put half the axial force on each strip to the actual web.
        unsupported=(clear_width-OD)/2+eccentricity+radial_leeway
        contact_width=2*r*math.sin(math.pi/4)
        narrow_skin=6*(largest_axial/2)*max(0,unsupported)/(contact_width*skin**2)
        rows[name]=dict(ODmm=OD,wallMm=skin,eccentricityMm=eccentricity,throatMm=throat,
            lowerClampN=nominal_low-rise,upperCompressiveLoadN=full_bearing,topGuaranteedSeatMm2=top_land,
            bottomGuaranteedSeatMm2=bottom_land,spoolCompressionMPa=full_bearing/shell_area,
            upperFootFaceCompressionMPa=topP,lowerLandMPa=full_bearing/bottom_land,
            upperFootOverhangMm=over,upperFootCantileverScreenMPa=foot_bending,
            singleWeldTractionMPa=w_traction,HAZShearYieldMPa=103/math.sqrt(3),
            axialRingMPa=w_axial,ringTransverseMPa=w_shear,hostSidewallLigamentToBossMm=unsupported,
            localSkinCantileverScreenMPa=narrow_skin,HAZYieldMPa=103.,
            originalStationForceN=s)
    result=dict(authority=authority,wholeClockDuty=digest(p/'r3-all-clock-station-reactions.json'),factoredDuty=digest(p/'r3-brace-section-strength.json'),
        originalFastenerFit=digest(p/'underframe-brace-fit.json'),statedLoadFactor=2.,rows=rows,
        scope='Pre-machining engineering screen. Lower-bounded seats assume concentric reaming through the clamped foot, traveler 0.35 mm registration, ±0.05 radius; material HAZ floor is 103 MPa. Positive circumferential joints are assumed at both tube skins. Effective weld throat, tube-to-crossbeam terminal connection, HAZ skin distortion/fatigue and differing pressure distribution still need qualification.')
    (p/'welded-spool-feasibility.json').write_text(json.dumps(result,indent=2)+'\n')
    for k in ('lowerClampN','upperFootFaceCompressionMPa','spoolCompressionMPa','upperFootCantileverScreenMPa','singleWeldTractionMPa','localSkinCantileverScreenMPa'):
        print(k,(min if k=='lowerClampN' else max)((r[k],n) for n,r in rows.items()))


if __name__=='__main__':solve()
