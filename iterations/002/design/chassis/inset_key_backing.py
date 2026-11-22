"""Size the governing keyed roof, retained nutbar and exposed top cap under the same load histories."""
from pathlib import Path
import hashlib,json,math
import numpy as np

HERE=Path(__file__).resolve().parent


def run(contact=False):
    source=HERE/("inset-key-contact-register.json" if contact else "inset-deck-register.json");register=json.loads(source.read_bytes())
    # Upper crossmembers lie at X110/X250. Their 50.8 mm wide roof is at
    # Z187.325. The 25.4 mm opposite dimension of the perimeter and rail
    # sidewalls is not substituted for that actual crossmember cross section.
    rows=[]
    for row in register["rows"]:
        x,y=row["xyMm"];sheet=np.asarray(row["loadsN"]);fx,fy=-sheet;host=row["host"]
        if host=="perimeter":
            h=2.1844;dx=0.;dy=y-np.sign(y)*345.95
            # Load reaches the inner-backed 3 mm tube roof through the same
            # profile as the deck. The two hold screws run longitudinally.
            bolt_xy=[[-24,-5],[24,5]]
        else:
            h=(15.875 if host=="upper crossmember" else 41.275)+2.1844
            dx=0.;dy=0. if host=="upper crossmember" else (y-115.3*np.sign(y))
            bolt_xy=[[-10,-18],[10,-18],[-10,18],[10,18]]
        # Contact patch can support compression on either side. Separating loads
        # use a conservative mixed peak bracket prying amplification of 2.0.
        # Required values are retained to verify the actual shape, screw and threads.
        magnitude=float(np.hypot(fx,fy));mx=-h*fy;my=h*fx;mz=dx*fy-dy*fx
        b=np.array(bolt_xy)
        if host=="perimeter":
            # Two diagonal bolts alone cannot resolve an arbitrary 2-axis
            # vertical torque distribution; the bottom flange has finite land.
            hold=2*(abs(mx)/(2*8.)+abs(my)/(2*24.))
        else:hold=2*(abs(mx)*18/(b[:,1]@b[:,1])+abs(my)*10/(b[:,0]@b[:,0]))
        # Nonlinear local edge stress remains outside a projected-bearing screen.
        # The root uses two opposed flat lands based on the commissioned metal key,
        # and accounts for both in-plane forces at the receiving roof.
        roof=abs(fx)/(10*3.)+abs(fy)/(32*3.)
        rows.append(dict(variant=row["variant"],case=row["case"],site=[x,y],host=host,sheetForceN=sheet.tolist(),deckOnHostForceN=[fx,fy],
            registeredVerticalLeverMm=h,lateralLeverMm=dy,supportOnSheetWrenchAboutRoofNm=[-z/1000 for z in (mx,my,mz)],
            deckOnHostWrenchAboutRoofNm=[z/1000 for z in (mx,my,mz)],
            requiredRetentionPerWorstScrewN=hold,projectedTubeRoofBearingMPa=roof,
            pairedDirectionsKeyRootDirectShearMPa=magnitude/(36*14-(4-math.pi)*2**2)))
    return dict(authority=register["authority"],sourceSha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        deckCharge=register["sheet"],groups={k:dict(worstRoof=max((r for r in rows if r["host"]==k),key=lambda r:r["projectedTubeRoofBearingMPa"]),
                    worstRetention=max((r for r in rows if r["host"]==k),key=lambda r:r["requiredRetentionPerWorstScrewN"])) for k in sorted({r["host"] for r in rows})},rows=rows,
        scope="Required retention and projected tube roof bearing for all source rows. Each upper crossmember has a key directly over its roof. Each rear sill needs a positive offset pedestal down to its narrower rail. Prying factor is a sizing hypothesis. Thread stripping, eventual distributed planar pressure/fatigue, metal form, real screw seating, assembly ordering and lateral motion require physical/independent evidence.")


if __name__=="__main__":
    import sys
    contact="--clearance" in sys.argv
    r=run(contact);(HERE/("inset-key-contact-backing.json" if contact else "inset-key-backing.json")).write_text(json.dumps(r,indent=2)+"\n")
    print({k:(v["worstRoof"]["projectedTubeRoofBearingMPa"],v["worstRetention"]["requiredRetentionPerWorstScrewN"]) for k,v in r["groups"].items()})
