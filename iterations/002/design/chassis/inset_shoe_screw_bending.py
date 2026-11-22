"""Bound the OEM screw bending when only the inboard fitted shoes support the screw."""
import json,math,hashlib
from pathlib import Path
here=Path(__file__).resolve().parent
shoe=here/"inset-socket-shoe.json";core=here/"inset-shoe-screw-cores.json"
S=json.loads(shoe.read_bytes());C=json.loads(core.read_bytes())
if S["authority"]!=C["authority"]:raise ValueError("Changed screw installation")
cases=[]
for rank in ("inner","middle","outer"):
    F=S["holeScreens"][rank]["peakLoadN"]*math.cos(math.radians(2.5))   # directional sample, conservative lower norm
    d=4.826 # full nominal major diameter, favorable section even where true core is smaller
    force_at_one_end=F/2
    q=120*d
    for gap,tag in [(3.2,"inboard shoe alone"),(0.0,"positive bearing flush to OEM plate, ideal lower bound")]:
        # One end of a double-shear pin must react at least F/2. Between that end
        # and the nearest usable shoe, V is constant. Even immediately loading the
        # shoe to allowable projected bearing gives minimum bending excursion:
        # integral V dx = R*g + R²/(2q). Some section must reach >= half this
        # range even when end moments can take the optimally offset signs.
        excursion=force_at_one_end*gap+force_at_one_end**2/(2*q)
        m=excursion/2
        cases.append(dict(rank=rank,configuration=tag,forceAtLeastN=F,oneEndReactionAtLeastN=force_at_one_end,
            favorableMajorDiameterMm=d,usableUnsupportedGapMm=gap,peakSupportNPerMm=q,minimumLoadedLandMm=force_at_one_end/q,
            minimumMomentNm=m/1000,minimumElasticBendingMPa=32*m/(math.pi*d**3),
            requiredYieldFoS2MPa=64*m/(math.pi*d**3)))
out=dict(authority=S["authority"],sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (shoe,core)},
    cases=cases,scope="Necessary conditions for the no-friction, shoe-only shear path with at most 120 MPa nominal projected aluminum bearing. The uncoupled minimum is favorable to the screw: zero outer plate thickness, coincident ideal reaction, arbitrary favorable end moment, entire nominal screw major diameter solid. It does not establish actual thread strength or bending. Contact at the original tube-wall bores, a separately registered direct module seat, preload or an integral raised bearing can change this load path. Flush control removes the gap only; it is not an installed part. No full-robot claim.")
(here/"inset-shoe-screw-bending.json").write_text(json.dumps(out,indent=2)+"\n")
for r in cases:print(r["rank"],r["configuration"],round(r["minimumMomentNm"],3),round(r["minimumElasticBendingMPa"],2),round(r["requiredYieldFoS2MPa"],2))

