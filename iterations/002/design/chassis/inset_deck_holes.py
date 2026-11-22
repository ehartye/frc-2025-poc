"""Identify physical vertical mounting faces in each base-down manufacturer model."""
from pathlib import Path
import json,sys
import numpy as np
from OCP.STEPControl import STEPControl_Reader
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from inset_electrical_package import CAD,sha,FILES

HERE=Path(__file__).resolve().parent


def run(pkg):
    sys.path.insert(0,str(pkg/"scripts"))
    from mechanical_motion import bounds
    e=json.loads((HERE/"inset-electrical-package.json").read_bytes())
    out={}
    for n in ("Power distribution","Control computer","Radio","Main 120 A breaker","Pan controller"):
        part=e["candidatePlacements"][n];src=part["source"];file=CAD/FILES[src]
        if sha(file)!=part["sourceSha256"]:raise ValueError(n)
        tf=np.array(part["worldMatrixMm"]);R,t=tf[:,:3],tf[:,3]
        reader=STEPControl_Reader();reader.ReadFile(str(file));reader.TransferRoots()
        it=TopExp_Explorer(reader.OneShape(),TopAbs_FACE);rows={}
        while it.More():
            face=(getattr(TopoDS,"Face_s",None) or TopoDS.Face)(it.Current());it.Next()
            s=BRepAdaptor_Surface(face)
            if s.GetType()!=GeomAbs_Cylinder:continue
            cy=s.Cylinder();a=R@cy.Axis().Direction().Coord()
            if abs(a[2])<.999999 or not 1.05<cy.Radius()<6.5:continue
            p=R@cy.Location().Coord()+t;b=np.array(bounds(face))
            z=sorted(((R@b[:3]+t)[2],(R@b[3:]+t)[2]))
            if z[0]>215. or z[1]<212.:continue
            xy=tuple(round(v,4) for v in p[:2])
            rows.setdefault(str(xy),[]).append(dict(xyMm=p[:2].tolist(),radiusMm=cy.Radius(),zMm=z,sweepDegrees=(s.LastUParameter()-s.FirstUParameter())*180/np.pi))
        out[n]=dict(sourceSha256=part["sourceSha256"],worldMatrixMm=part["worldMatrixMm"],features=list(rows.values()))
        print(n,[(x[0]["xyMm"],round(x[0]["radiusMm"],2)) for x in rows.values()],flush=True)
    return dict(authority=e["authority"],subjects=out,scope="Original cylindrical faces normal to the deck and terminating at the base elevation are filtered by the recorded local-to-world transform and vertical axis. Candidate register only; underside seats, tool, fastener selection and thread/slot identity are separate.")


if __name__=="__main__":
    r=run(Path(sys.argv[1]));(HERE/"inset-deck-holes.json").write_text(json.dumps(r,indent=2)+"\n")
