"""Bound the flanged stations and two drilled crossbeams from the recorded matched duties."""
import json
import math

import numpy as np

from output_mount import ROOT, digest, read

DATA = ROOT / "artifacts/red2-integrated/pan-thrust"
ALUMINUM_YIELD_MPA = 103.0  # Conservative floor from 15 ksi HAZ yield for thin 6061-T6 with 4043 filler.
E_MPA = 69600.
LOAD_FACTOR = 2.0
D534 = "https://documents.dps.ny.gov/public/Common/ViewDoc.aspx?DocRefId=%7B4B04BC4B-2A0A-476A-9B01-843A9B5FB9A4%7D"
REV_DRAWING = "https://revrobotics.ca/content/docs/10-32_Button_Head_Cap_Screws-DR.pdf"


def solve():
    trace_file = DATA / "r3-all-clock-station-reactions.json"
    fit_file = DATA / "r3-counterbrace-flanged.json"
    trace,fit=read(trace_file),read(fit_file)
    if digest(ROOT/trace["authority"]["file"]) != trace["authority"]["sha256"] or trace["authority"]!=fit["authority"]:
        raise ValueError("Need the qualified source lineage")
    if (len(fit["throughTests"])!=8 or trace["matchedPhysicalCases"]!=40 or trace["postReleaseCases"]!=120
            or any(c["installedClashesMm3"] or c["pathBlockersMm3"] for c in fit["cases"].values())):
        raise ValueError("Need the complete measured geometry and load cases")
    duty=trace["perTowerAndStation"]
    stations={}
    def peak(side,name):
        # Carry both independent historical subsets. Including pipe alone also covers the unhelpful
        # sign of a gravitational counterload; new rotating hardware receives a separate load entry.
        candidates=[(group,duty[group][name]) for group in ("pipe/"+side,"pipeAndShopMass/"+side)]
        group,value=max(candidates,key=lambda entry:entry[1]["value"])
        return dict(group=group,**value)
    # UNC tensile-stress-area construction; d=.190 inch, 32 turns/inch. Major shank is not credited.
    stress_area=math.pi/4*((.190-.9743/32)*25.4)**2
    for side in ("port","starboard"):
        for i,x in enumerate((186,224),1):
            for site in ("cross","sill"):
                key=f"x{x}/{site}/"
                support=fit["throughTests"][f"{side.title()} {i} {site}"]
                rows={metric:peak(side,key+metric) for metric in ("upliftN","compressionN","torsionInclusiveShearN","combinedEquivalentN")}
                factored={k:LOAD_FACTOR*v["value"] for k,v in rows.items()}
                t=support["hostWallMm"];area=support["activeProjectedBearingAreaMm2"]
                stations[f"{side}/{x}/{site}"]=dict(peakRecords=rows,factoredN=factored,
                    flangeContactCompressionMPa=factored["upliftN"]/area,
                    nominalPunchingShearMPa=factored["upliftN"]/(math.pi*11.1*t),
                    oneWallRadialBearingMPa=factored["torsionInclusiveShearN"]/(11*t),
                    footBearingMPa=factored["torsionInclusiveShearN"]/(4.826*7.),
                    threadedScrewEquivalentAddedStressMPa=factored["combinedEquivalentN"]/stress_area,
                    stressAreaMm2=stress_area)
    beams={}
    width=25.;t=2.5;hole=11.1
    gross=(width**4-(width-2*t)**4)/12
    ix=gross-2*(hole*t**3/12+hole*t*((width-t)/2)**2)
    iz=gross-2*t*hole**3/12
    area=width**2-(width-2*t)**2-2*hole*t
    length=466.;locations=[28.,438.]
    for x in (186,224):
        loads=[stations[f"{s}/{x}/cross"]["factoredN"] for s in ("starboard","port")]
        z=np.array([max(a["upliftN"],a["compressionN"]) for a in loads])
        shear=np.array([a["torsionInclusiveShearN"] for a in loads])
        # Elastic simply supported response to two point forces. Independent absolute per-end maxima
        # bound all simultaneous states, including opposing signs. A conservative one-hole net section
        # is used for the full length for both bending and elastic deflection.
        y=np.unique(np.r_[np.linspace(0,length,1865),locations])
        m=np.array([np.where(y<=a,y*(length-a)/length,a*(length-y)/length) for a in locations])
        deflection=[]
        for a in locations:
            b=length-a
            deflection.append(np.where(y<=a,b*y*(length**2-b**2-y**2)/(6*length),
                                        a*(length-y)*(length**2-a**2-(length-y)**2)/(6*length)))
        disp=np.array(deflection)
        bending_x=z@m
        bending_z=shear@m
        # The upper contact plane lies 12.5 mm off the centroid. Include the total absolute
        # horizontal demand as a simultaneous upper bound on axial force and its end couple.
        eccentric=12.5*sum(shear)
        normal=(bending_x+eccentric)*12.5/ix+bending_z*12.5/iz+sum(shear)/area
        ends=[np.array([(length-a)/length for a in locations]),np.array([a/length for a in locations])]
        vz=max(float(s@z) for s in ends)+eccentric/length
        vx=max(float(s@shear) for s in ends)
        web_shear=1.5*(vx+vz)/(2*t*(width-2*t))
        eq=np.sqrt(normal**2+3*web_shear**2)
        maximum=int(np.argmax(eq))
        dz=z@disp/(E_MPA*ix)
        dx=shear@disp/(E_MPA*iz)
        beams[str(x)]=dict(loadFactor=LOAD_FACTOR,netAreaMm2=area,netIxMm4=ix,netIzMm4=iz,
                           factoredLoadEnvelopesN={"vertical":z.tolist(),"horizontalResultant":shear.tolist()},
                           conservativeSectionNormalMaxMPa=float(max(normal)),grossWebShearScreenMPa=web_shear,
                           combinedScreenMPa=float(eq[maximum]),peakFromStarboardEndMm=float(y[maximum]),
                           hazYieldScreenPass=bool(eq[maximum]<ALUMINUM_YIELD_MPA),hazYieldFactor=ALUMINUM_YIELD_MPA/float(eq[maximum]),
                           verticalDeflectionFromPointForcesMm=float(max(dz)),horizontalDeflectionFromPointForcesMm=float(max(dx)),
                           verticalAtBoltsMm=[float(dz[np.argmin(abs(y-a))]) for a in locations],
                           horizontalAtBoltsMm=[float(dx[np.argmin(abs(y-a))]) for a in locations])
    result=dict(authority=trace["authority"],traceSha256=digest(trace_file),geometrySha256=digest(fit_file),loadFactor=LOAD_FACTOR,
                materialEnvelope=dict(purchase="6061-T6 sheet for cross box, 6061-T6 hollow rail; 4043 weld procedure. 2.0 to 2.5 mm nominal walls.",
                                      weldedYieldFloorMPa=ALUMINUM_YIELD_MPA,elasticModulusMPa=E_MPA,specReference=D534,
                                      weldContext="Aluminum Association ADM 2010 A.3.5/A.3.5M, <=9.5 mm with 4043 filler; 15 ksi / 105 MPa tabulated. 103 MPa used.",fastenerDrawing=REV_DRAWING),
                stations=stations,beams=beams,limits=[
                 "The factored time-window demand includes the known pan shop stock and pipe. Installed new moving inertia, drive torque return, drivetrain impact and fatigue spectra remain separate.",
                 "The section calculation is a global elastic screening model. End fixity, joint weld groups, local strip bending around the sleeve, flange spreading, net ligament concentration, preload distribution and combined torsion require their own design checks. The deflections shown are contributions from point loads, not total axle misalignment.",
                 "The REV drawing specifies alloy steel with black oxide and gives no screw proof strength or nut proof load. Minimum certificate/proof load and clamp window are engineering requirements to close. The threaded area and equivalent added stress exclude clamp preload."])
    (DATA/"r3-brace-section-strength.json").write_text(json.dumps(result,indent=2)+"\n")
    for x,beam in beams.items():print(x,beam)
    for key in ("flangeContactCompressionMPa","nominalPunchingShearMPa","oneWallRadialBearingMPa","threadedScrewEquivalentAddedStressMPa"):
        print(key,max((row[key],name) for name,row in stations.items()))


if __name__ == "__main__":solve()
