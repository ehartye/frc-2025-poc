"""Register the same public exchange imported by Fusion, in the actual OCCT solid order."""
import argparse
from pathlib import Path
from exchange import HERE, SW, base
from OCP.TDF import TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_ShapeTool
from OCP.collections import Sequence_TDF_Label


def title(label):
    name=TDataStd_Name()
    if not label.FindAttribute(TDataStd_Name.GetID_s(),name):
        raise ValueError("Unnamed CAD occurrence")
    return name.Get().ToExtString()


def read_assembly(source):
    doc,t=base.cad.xcaf_doc()
    r=base.STEPCAFControl_Reader()
    if r.ReadFile(str(source))!=base.cad.IFSelect_RetDone or not r.Transfer(doc):
        raise ValueError("Cannot reopen the student exchange")
    labels=Sequence_TDF_Label();t.GetFreeShapes(labels)
    if labels.Length()!=1:raise ValueError("Expected one public root")
    hierarchy=[]
    def walk(label,ancestors):
        children=Sequence_TDF_Label()
        if t.GetComponents_s(label,children):
            for child in children:
                name=title(child);ref=TDF_Label()
                if not t.GetReferredShape_s(child,ref):raise ValueError("Missing child shape")
                walk(ref,ancestors+[name])
        else:
            solids=list(base.subsolids(t.GetShape_s(label)))
            if len(solids)!=1 or len(ancestors)!=2:raise ValueError(ancestors)
            hierarchy.append((ancestors[0],ancestors[1],solids[0]))
    walk(labels.Value(1),[])
    return hierarchy


def run(source,exchange):
    roster=base.read(exchange)
    historical={r["id"]:r for r in base.read(HERE.parent/"full-swerve-r4/appearances.json")["solids"]}
    kit={r['id']:r for r in base.read(SW/'retention/open-yokes.json')['parts']}
    native=base.read(HERE/"student-reopen.json")
    if base.sha(source)!=roster["exchange"]:raise ValueError("Source differs from the exchange imported into Fusion")
    parts=read_assembly(source)
    expected={(gr,n) for gr,names in roster["groups"].items() for n in names}
    if expected!={(g,n) for g,n,_ in parts} or len(parts)!=1002:raise ValueError("Unexpected named roster")
    rows=[]
    for group,name,shape in parts:
        path=group+":1+"+name+":1"
        if native["components"][path]["bodies"]!=1:raise ValueError(path)
        row=dict(historical.get(name,{}))
        if name in kit:
            row.update(id=name,role="sensing" if "probe" in name else "shaft" if "tube" in name else "receiver",
                       source="Team 1780 bore-pin capture study",representation="registered-envelope" if "servo-envelope" in name else "project-cad")
        elif name=="Practice CORAL tube":
            row=dict(id=name,role="game-piece",source="Project nominal hollow practice pipe",representation="registered-envelope")
        if name in ("r6-polymer-facing","r6-receiver-insert"):row["source"]="Team 1780 continuous underside cam lands"
        row.update(boundsMm=[round(v,5) for v in base.cad.bounds(shape)],motionGroup=group)
        rows.append(row)
    return dict(schema="agent-frc/assembly-appearances@1",sourceSha256=base.sha(source),nativeArchiveSha256=native["sha256"],
                configuration=native["configuration"],units="mm",solids=rows)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("exchange_register",type=Path);a=p.parse_args()
    source=HERE/"public-robot.step"
    base.save(HERE/"appearances.json",run(source,a.exchange_register))
