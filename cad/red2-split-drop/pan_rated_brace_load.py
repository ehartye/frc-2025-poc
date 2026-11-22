"""Bound preload, screw torsion, flange bearing and the sleeve thread for the rated tower joint."""
import json
import math
from pathlib import Path

from output_mount import ROOT, digest, read

TENSILE_PROOF_MPA = 140000 * .006894757293168
H1150_YIELD_MPA = 105000 * .006894757293168
WELDED_ALUMINUM_YIELD_MPA = 103.


def solve():
    directory = ROOT/'artifacts/red2-integrated/pan-thrust'
    geometry_path = directory/'rated-barrel-fit.json'
    duty_path = directory/'r3-brace-section-strength.json'
    geometry = read(geometry_path)
    duty = read(duty_path)
    authority = read(ROOT/'cad/fusion-authority.json')['source']['archive']
    if geometry['authority'] != authority or digest(ROOT/authority['file']) != authority['sha256'] or digest(duty_path) != geometry['dutySha256']:
        raise ValueError('Source revision or factored load history changed')
    major = .138 * 25.4
    pitch = 25.4/32.
    area = math.pi/4 * (major-.9743*pitch)**2
    pitch_d = major-.64952*pitch
    # Undercut the nominal external root additionally for a conservative torque bound.
    root_d = 2.48
    clamp_low, clamp_high = 1950., 2200.
    mu = .12
    thread_torque = clamp_high*pitch_d/2 * math.tan(math.atan(pitch/(math.pi*pitch_d))+
                      math.atan(mu/math.cos(math.radians(30))))
    twist_stress = 16*thread_torque/(math.pi*root_d**3)
    rows = {}
    for key,joint in geometry['joints'].items():
        loads = duty['stations'][key]['factoredN']
        up, downward, horizontal = (loads[k] for k in ('upliftN','compressionN','torsionInclusiveShearN'))
        max_t = clamp_high+up
        # Fully transfer the factored service shear to the smallest threaded section,
        # disregard interface friction; keep the entire upper tightening thread torque.
        tension_stress = max_t/area
        shear_stress = horizontal/area
        screw_eq = math.hypot(tension_stress, math.sqrt(3)*(shear_stress+twist_stress))
        section = math.pi/4*(5.24**2-major**2)
        # Assume only six mm of the 10+ mm available full thread effectively share the
        # strip load, with one-half pitch shear thickness and finished internal minor >=2.65 mm.
        thread_shear = max_t/(.5*math.pi*2.65*6.)
        # Top pilot bears in the pierced foot and directs horizontal force to the
        # lower body and host walls. Credit only one tube wall in the latter.
        pilot_shear = horizontal/section
        pilot_bearing = horizontal/(5.24*3.8)
        host_t = 2. if key.endswith('sill') else 2.5
        host_bearing = horizontal/(11.04*host_t)
        lower_bearing = max_t/joint['flangeToHostMm2']
        washer_bearing = (clamp_high+max(up,downward))/joint['washerToFootMm2']
        contact_at_head = max_t/joint['headToWasherMm2']
        # The lower flange overlaps the 11.1 mm host bore. Equilibrium ring
        # bound with cantilever arm <= 2.5 mm and minimum 2.95 mm flange.
        flange_bending = 6*max_t*2.5/(math.pi*11.04*2.95**2)
        # Upper washer bridges the clearance hole. Outside the hole it has
        # continuous nominal support. Entire head load placed at the inner edge.
        washer_bending = 6*max_t*(2.67-1.865)/(math.pi*3.73*2.45**2)
        # Bound the OD5.24 pilot root if every in-plane reaction acted at its
        # upper tip, and full bolt tension remains in the reduced thread-bored ring.
        I = math.pi*(5.24**4-major**4)/64
        pilot_stress = max_t/section + horizontal*4.05*2.62/I
        pilot_eq = math.hypot(pilot_stress,math.sqrt(3)*pilot_shear)
        rows[key] = dict(minClampResidualN=clamp_low-up,maxBoltTensionN=max_t,
                         fullThreadTorqueNmm=thread_torque,screwEquivalentMPa=screw_eq,
                         screwProofRatio=TENSILE_PROOF_MPA/screw_eq,threadStripShearMPa=thread_shear,
                         gripMaterialShearYieldMPa=H1150_YIELD_MPA/math.sqrt(3),
                         footRadialBearingMPa=pilot_bearing,hostRadialBearingMPa=host_bearing,
                         lowerFlangeContactMPa=lower_bearing,upperWasherContactMPa=washer_bearing,
                         screwHeadOnH1150WasherMPa=contact_at_head,
                         bareHeadToFootMm2=joint['bareHeadToFootMm2'],
                         withoutWasherHeadOnAluminumMPa=(max_t/joint['bareHeadToFootMm2'] if joint['bareHeadToFootMm2']>0 else None),
                         flangeBendingMPa=flange_bending,washerBendingMPa=washer_bending,
                         pilotEquivalentMPa=pilot_eq,
                         conditionalPass=all([clamp_low>up,screw_eq<TENSILE_PROOF_MPA,
                            pilot_eq<H1150_YIELD_MPA,thread_shear<H1150_YIELD_MPA/math.sqrt(3),
                            max(pilot_bearing,host_bearing,lower_bearing,washer_bearing)<WELDED_ALUMINUM_YIELD_MPA,
                            max(flange_bending,washer_bending,contact_at_head)<H1150_YIELD_MPA]))
    result=dict(authority=authority,geometrySha256=digest(geometry_path),dutySha256=digest(duty_path),
                selectedScrew=geometry['source'],screwStressAreaMm2=area,externalRootDesignMinMm=root_d,
                procurement=dict(screw='Certified zinc-plated alloy 632120, A574, 6-32 UNC; incoming gage and material/lot trace.',
                  sleeveAndSpreader='Machine A564 Grade 630 H1150 bar, min 105 ksi yield, after heat treatment. Ream two host bores in line after welding; check spanner flats/pins, pilot and flange faces.',
                  tubeAndFoot='6061-T6 procurement with 103 MPa local welded yield screening assumption'),
                assembly=dict(achievedClampN=[clamp_low,clamp_high],maximumQualifiedThreadFriction=mu,
                      minimumFullEngagedLengthMm=6,minimumMeasuredExternalRootMm=root_d,
                      serviceCondition='The preload bounds apply after settling. Calibrate the actual screw finish, tapped sleeve finish and assembly lubricant in a load-cell fixture. Use a matched fastener lot, inspect tool access, record torque-angle and residual clamp. Reject installations outside that measured preload/friction window.'),
                cases=rows,limits=['No capacity is assigned to unverified fastener torque alone. Bound uses the concurrent source-time load history at two times project load, and conservatively combines independent uplift/shear extrema. Source-time pan inertia and additional robot load cases remain open.',
                'Flat strip bending and average contact/strip checks are screening calculations. Actual thread pitch-diameter gages, local prying/stress concentration, wall stretching, web/end welds, preload loss, fatigue, tool release and installed load path need separate review.',
                'The barrels in the proposal are machined with nominal 0.010 mm shoulder setback from the top host after welding; inspect at each station. Their actual installed flex and load sharing are unresolved.'])
    (directory/'rated-barrel-strength.json').write_text(json.dumps(result,indent=2)+'\n')
    for k in ['minClampResidualN','screwEquivalentMPa','pilotEquivalentMPa','washerBendingMPa','upperWasherContactMPa']:
        values=[(v[k],id) for id,v in rows.items()]
        print(k,(min if k.startswith('min') else max)(values))
    print('conditional',sum(r['conditionalPass'] for r in rows.values()),len(rows))


if __name__=='__main__':
    solve()
