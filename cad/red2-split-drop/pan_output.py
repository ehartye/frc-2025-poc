"""Separate the real rotating and fixed members of the issued V2 output without changing either."""
import sys
import json
from pathlib import Path
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound
from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from output_mount import ROOT,OUT,digest,read
D=ROOT/'artifacts/red2-integrated'
def run(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    file=D/'pan-drive-registration.json';d=read(file)
    original=next((r for r in d['components'] if r['id']=='REV-41-1615'),None)
    if original is None:raise ValueError('Already separated')
    src=ROOT/original['source'];assert digest(src)==original['sourceSha256']
    solids=load_solids(src)[0]
    movers={k for k in solids if k in ('REV-41-1611_Through Hole','REV-41-1612') or k.startswith(('M2 SHCS SS_03mm','DOWEL_PIN_2mm'))}
    assert len(movers)==8 and len(solids)==16
    pairs=[]
    for mobile,names in [(False,set(solids)-movers),(True,movers)]:
        name='rotating' if mobile else 'stationary'
        out=D/'pan-supplier'/('REV-41-1615-'+name+'.step')
        group=TopoDS_Compound();add=BRep_Builder();add.MakeCompound(group)
        for key in sorted(names):add.Add(group,solids[key])
        if not out.exists():
            writer=STEPControl_Writer();writer.Transfer(group,STEPControl_AsIs);assert writer.Write(str(out))==IFSelect_RetDone
        pairs.append(dict(original,id='REV-41-1615 '+name,source=out.relative_to(ROOT).as_posix(),sourceSha256=digest(out),moving=mobile,originalMemberNames=sorted(names),unchangedSource=original['sourceSha256']))
    d['components']=[r for r in d['components'] if r is not original]+pairs
    file.write_text(json.dumps(d,indent=2)+'\n')
    print([(r['id'],len(r['originalMemberNames'])) for r in pairs])
if __name__=='__main__':run(Path(sys.argv[1]))
