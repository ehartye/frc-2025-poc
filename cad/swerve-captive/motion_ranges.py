"""Prove the moving-body ranges against the actual orbit STL byte by byte."""
import gzip
import hashlib
import json
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
from OCP.BRep import BRep_Builder
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS_Compound

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def run():
    cad=ROOT/'docs/robot/cad'
    register=read(cad/'receipt.json');source=ROOT/register['source']
    assert sha(source)==register['sourceSha256']
    appearance=read(HERE/'appearances.json')['solids']
    reader=STEPControl_Reader()
    if reader.ReadFile(str(source))!=IFSelect_RetDone or reader.TransferRoots()!=reader.NbRootsForTransfer():
        raise ValueError('Incomplete public exchange')
    shape=reader.OneShape()
    mesh=BRepMesh_IncrementalMesh(shape,.18,False,.18,True)
    if not mesh.IsDone():raise ValueError("Tessellation failed")
    roles={}
    explorer=TopExp_Explorer(shape,TopAbs_SOLID)
    for row in appearance:
        if not explorer.More():raise ValueError('Missing shape')
        role=row['role'];group=row['motionGroup']
        if group.endswith(('module base','steering fork','tread')):group='Fixed chassis'
        batches=roles.setdefault(role,[])
        if not batches or batches[-1][0]!=group:
            body=TopoDS_Compound();BRep_Builder().MakeCompound(body);batches.append([group,body])
        BRep_Builder().Add(batches[-1][1],explorer.Current());explorer.Next()
    if explorer.More():raise ValueError('Unregistered shape')
    intervals=[]
    with gzip.open(cad/'assembly.stl.gz','rb') as packed, TemporaryDirectory() as tmp:
        count=struct.unpack_from('<I',packed.read(84),80)[0];total=0
        for role,batches in roles.items():
            for group,compound in batches:
                piece=Path(tmp)/'facet.stl';writer=StlAPI_Writer();writer.ASCIIMode=False
                if not writer.Write(compound,str(piece)):raise ValueError(f'Cannot export {group}')
                data=piece.read_bytes();n=struct.unpack_from('<I',data,80)[0]
                if len(data)!=84+50*n or packed.read(n*50)!=data[84:]:
                    raise ValueError(f'Wrong ordered facet bytes for {role} / {group}')
                intervals.append([role,group,n]);total+=n
        if total!=count or packed.read(1):raise ValueError('Incorrect assembly extent')
    return dict(mesh=register['meshSha256'],archive=read(HERE/'student-reopen.json')['sha256'],triangles=total,batches=intervals)


if __name__=='__main__':
    result=run();(HERE/'motion-ranges.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8',newline='\n')
    print(result['triangles'],len(result['batches']))
