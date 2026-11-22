"""Check one internal double-sided socket shoe and its positive cross-pin against the original complete module."""
import hashlib, json, math, re, sys
from pathlib import Path

from crossmember_envelope import box, cylinder
from direct_saddle_packaging import join
from port_anchor_fit import cut
from four_corner_packaging import rotated
from structural_perimeter import relocate_lower

HERE=Path(__file__).resolve().parent
UP=(0,0,1)


def run(plugin):
    sys.path.insert(0,str(plugin/"scripts"))
    from fusion_authority import register_path,verify
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import bounds,distance,common
    from component_shape import volume
    root=HERE.parents[3];reg=register_path(root);lin=reg.parent/"issue-005/lineage.json"
    authority=verify(root,reg,lin); allparts,sheets=load_solids(lin.with_name("assembly.step"))
    if len(allparts)!=552 or sheets:raise ValueError("Wrong edition")
    defs=json.loads((HERE/"socket-holes.json").read_bytes())["rails"]
    parts=relocate_lower({k:v for k,v in allparts.items() if int(re.search(r"\[1\.(\d+)\.",k)[1])<=8},defs,12.7)
    group=defs["Port longitudinal socket rail"]; cy=330.975-12.7
    xs=sorted(x-12.7 for x,y in group["boltCentersMm"] if x>0)
    inn,mid,out=xs;pin_x=(mid+inn)/2
    left,right=inn-7.0,out+5.35
    lo,hi=group["minimumMm"][2],group["maximumMm"][2]
    inner_y,outer_y=cy-9.475,cy+9.475
    loadfile=HERE/"inset-stagger-all-contacts.json";load=json.loads(loadfile.read_bytes());peaks=load["peaks"]
    if authority!=load["authority"]:raise ValueError("Mismatched loads")
    def block(xa,xb,inner,outer):return box((xa,inner_y,inner),(xb,outer_y,outer))
    shapes={};masses={}
    for end in ("bottom","top"):
        z=lo+3.175+.025 if end=="bottom" else hi-3.175-.025
        def height(a,b,h):return block(a,b,z,z+h) if end=="bottom" else block(a,b,z-h,z)
        # One piece from 6061-T6 stock. The thickness change is machined outside the
        # loaded pin-bearing lands, leaving continuous side contact at each location.
        form=join(height(left,right,4.25),height(left,inn+7.,9.0),height(out-6.5,right,6.))
        for x,d in [(x,4.98) for x in xs]+[(pin_x,10.05)]:
            form=cut(form,cylinder((x,cy,lo-1),UP,d/2,hi-lo+2))
        shapes[end]=form
    pin=cut(cylinder((pin_x,cy,lo+.10),UP,5.0,hi-lo-.20),
            cylinder((pin_x,cy,lo-.2),UP,2.40,hi-lo+.4))
    shapes["pin"]=pin
    for name,v in shapes.items():masses[name]=volume(v)*2.7e-6
    host_name=next(k for k,v in parts.items() if "socket rail [" in k and abs(bounds(v)[1]-(cy-12.7))<.01 and abs(bounds(v)[4]-(cy+12.7))<.01)
    host=parts[host_name];drilled=cut(host,cylinder((pin_x,cy,lo-1),UP,5.025,hi-lo+2))
    receivers={k:(v,bounds(v)) for k,v in parts.items() if k!=host_name}
    receivers[host_name]=(drilled,bounds(drilled))
    fit={}
    for name,s in shapes.items():
        a=bounds(s);row={}
        for k,(v,b) in receivers.items():
            if box_gap(a,b)>.12:continue
            gap=distance(s,v);vol=common(s,v) if gap<.001 else 0.
            if gap<.12:row[k]=dict(gapMm=gap,commonMm3=vol)
        fit[name]=row
    # Prove positive front/back side contact using the actual complete machined tube.
    # Retain original nominal float and measure a 0.10 mm one-sided relative displacement.
    contacts={}
    for end in ('top','bottom'):
        contacts[end]={}
        for direction in (-1,1):
            shifted=rotated(shapes[end],0,(0,direction*.1,0))
            contacts[end][str(direction)]=common(shifted,drilled)
    # Neither exterior plate is optional for positive axial retention of this pin.
    # Measure exact penetration when withdrawing it. No keeper is inferred from a box test.
    stops={}
    for sign in (-1,1):
        s=rotated(pin,0,(0,0,2.*sign));b=bounds(s)
        stops[str(sign)]={k:common(s,v) for k,(v,bb) in receivers.items() if k!=host_name and box_gap(b,bb)<.001}
    requirements={}
    for rank,t in (("inner",9.),("middle",4.25),("outer",6.)):
        f=peaks[rank+"ScrewForceUpperN"]["value"];ax=peaks[rank+"ScrewAlongAbsN"]["value"]
        # Required effective pin-bearing diameter survives a real shank/root check
        # only if the actual manufacturer's screw provides that local diameter.
        requirements[rank]=dict(thicknessMinMm=t-.12,peakLoadN=f,requiredEffectiveDiameterAt120Mm=.6*f/(120*(t-.12)),
             maxAcrossAtSideWallPerShoeN=.6*peaks[rank+"ScrewAcrossAbsN"]["value"],
             projectedHoleBearingAt4mmMPa=.6*f/(4*(t-.12)),axialEndTearAtOuterMPa=.6*ax/(2*(5.35-2.55-.25)*(t-.12)) if rank=="outer" else None)
    axial=sum(peaks[r+"ScrewAlongAbsN"]["value"] for r in ('inner','middle','outer'))
    return dict(authority=authority,sourceLoadsSha256=hashlib.sha256(loadfile.read_bytes()).hexdigest(),
        module="FL at inset (+269.3,+264.3) mm",tube=host_name,originalSocketEndMm=227.3,
        axesMm=xs,pinAxisMm=[pin_x,cy],oneStationKg=masses,allEightGrossEquivalentKg=8*sum(masses.values()),
        priorUnmodifiedPinOverlapMm3=common(pin,host),fittedContacts=fit,transverseEngagementAt01mm=contacts,
        pinAxialStopAt2mm=stops,holeScreens=requirements,maxIndependentAxialSumN=axial,
        pinHostBearingAt60PercentMPa=.6*axial/(3*9.95),pinShearAt60PercentMPa=.6*axial/(math.pi/4*(9.95**2-4.9**2)),
        scope="Pre-native centerline and fit check on a single port rail end. No tube extension or oversized module screw holes. The two plates are loaded in positive bearing against the inside sidewalls. The new positive pin connects both shoes to the tube along the rail. Qualify its axial keeper, sidewall flexibility, signed screw bending/grip, seating/tolerance/preload and repair before adoption. The 60/40 distribution is a design premise and the rated section is for unwelded 6061-T6. No corner replication or torsional strength credit.")


if __name__=="__main__":
    d=run(Path(sys.argv[1]));(HERE/"inset-socket-shoe.json").write_text(json.dumps(d,indent=2)+"\n")
    print(d["allEightGrossEquivalentKg"],d["transverseEngagementAt01mm"],d["pinAxialStopAt2mm"])
    print(d["holeScreens"])

