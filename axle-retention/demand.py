"""Screen the grooved bar, split ring, housing and fasteners under project loads."""
import argparse
import json
import math
from pathlib import Path
from ring_washer import overlap, read_faces, report, section

HERE=Path(__file__).resolve().parent


def stress(moment,torque,axial,diameter,bore):
    den=math.pi*(diameter**4-bore**4)
    sigma=32*moment*1000*diameter/den + axial/(math.pi*(diameter**2-bore**2)/4)
    tau=16*torque*1000*diameter/den
    return math.sqrt(sigma*sigma+3*tau*tau)


def keeper_lip_screen(axial_force, fraction):
    # A full annular band is contained in both flat faces at 0.25 mm offset.
    # The circular lip is idealized as radial strips clamped at the pocket edge.
    inner_radius = 13.4
    outer_radius = 15.5575 - .25
    contact_area = math.pi * (outer_radius**2 - inner_radius**2) * fraction
    pressure = axial_force / contact_area
    span = 16.0 - inner_radius
    thickness = 3.10 - 1.65
    return 3 * pressure * span**2 / thickness**2


def run(archive):
    route=json.loads((HERE.parent/"design/drivetrain/route-receipt.json").read_bytes())["candidate"]["supportScreen"]
    face=report(archive,250)
    verts,tris=read_faces(archive)
    # Inscribe a complete annulus within the hex shoulder. The fillet, the
    # tolerance on the stock/groove and 0.10 mm relative ring offset consume land.
    inner=.453*25.4/2+.11+.10
    outer=12.65/2-.10
    groove_areas=[]
    for z in sorted({p[2] for p in verts}):
        triangles,_=section(verts,tris,z)
        groove_areas.append(overlap(triangles,(0,0),inner,outer))
    if min(groove_areas)<=0:
        raise ValueError("Clip has no guaranteed flat bearing on its groove shoulder")
    washer_land=min(d["areaBoundsAllOffsetsMm2"][0] for d in face["faces"])
    profile="Worst received groove root diameter and 5 mm major-bore reservation; no strength allowable is assigned."
    rows=[]
    for force in (250,500):
        moment=route["shaftMomentNm"]
        local=route["outerBearingRadialN"]*(325.35-314.16)/1000
        # Lateral play can shift the ring to one side of the washer. Mean pressures
        # are conditional on planar distributed contact; local peak can be higher.
        rows.append({"axialN":force,"netSectionGlobalMPa":stress(moment,route["shaftTorqueNm"],force,.447*25.4,5),
                     "netSectionAtGrooveMPa":stress(local,route["shaftTorqueNm"],force,.447*25.4,5),
                     "localBendingMomentNm":local,"clipToGrooveMeanUpperMPa":force/min(groove_areas),
                     "clipToWasherMeanUpperMPa":force/washer_land,
                     "thrustFaceMeanMPa":force/(math.pi*(8.35**2-7.5**2)),
                     "keeperAxialPerScrewUniformN":force/4,
                     "keeperTwoScrewShareN":force/2,
                     "keeperLipFullRingStripMPa":keeper_lip_screen(force,1.0),
                     "keeperLipQuarterRingStripMPa":keeper_lip_screen(force,.25),
                     "averageEndThreadShearDemandMPa":force/(.5*math.pi*4.04*12.7)})
    # Select the cut washer against the actual groove flank and ring with the
    # gearbox fully engaged. Rechecked after shimming by dial indicator.
    end_bounds=[317.4125-(314.19+d+t+w) for d,t,w in
                ((.03,.037*25.4,2.20),(-.03,.033*25.4,2.16))]
    return {"sourceClipSha256":face["sourceArchiveSha256"],"sourceRoute":route,
            "clipWasherProjection":face,"minimumGrooveFaceAreaMm2":min(groove_areas),
            "inscribedGrooveContactRadiiMm":[inner,outer],"stressBasis":profile,
            "shaftToRaceOutwardLimitsMm":end_bounds,"remainingEngagementBeforeInternalBearingPlayMm":10-max(end_bounds),
            "loads":rows}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clip_archive",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    args.output.write_text(json.dumps(run(args.clip_archive),indent=2)+"\n",encoding="utf-8",newline="\n")
