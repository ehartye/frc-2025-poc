"""Compare each exported STEP placement and its local-to-parent transforms with its registered supplier pose."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
try:
    from OCP.TDF import TDF_LabelSequence
except ImportError:
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence


def placements(path):
    doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    table = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    reader = STEPCAFControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError("Unusable issued STEP")
    roots = TDF_LabelSequence()
    table.GetFreeShapes(roots)
    assert roots.Length() == 1
    cases = {}
    def walk(definition, names, matrices):
        children = TDF_LabelSequence()
        table.GetComponents_s(definition, children)
        if children.Length() == 0:
            cases[tuple(names)] = matrices
        for i in range(1, children.Length() + 1):
            child = children.Value(i)
            target = TDF_Label()
            assert table.GetReferredShape_s(child, target)
            transform = table.GetLocation_s(child).Transformation()
            levels = [[transform.Value(r,c) for c in range(1,5)] for r in range(1,4)] + [[0,0,0,1]]
            text = TDataStd_Name()
            assert child.FindAttribute(TDataStd_Name.GetID_s(), text)
            walk(target, names + [text.Get().ToExtString()], matrices + [levels])
    walk(roots.Value(1), [], [])
    return cases


def analyze(plan, step):
    cases = placements(step)
    rows = {}
    for item in json.loads(plan.read_bytes())['items']:
        if item['kind'] not in ('bought','plate'):
            continue
        (path, matrices), = [(path, ms) for path, ms in cases.items() if item['id'] + ':1' in path]
        expected = np.eye(4)
        expected[:3,:] = item['matrixMm']
        placed = np.eye(4)
        for matrix in matrices:
            placed = placed @ matrix
        rows[item['id']] = dict(path=path, localToParent=matrices, composed=placed.tolist(),
                                expected=expected.tolist(), maximumMatrixDifference=float(abs(placed-expected).max()))
    if len(rows) != 56:
        raise ValueError("Incomplete purchased-article cohort")
    return dict(planSha256=hashlib.sha256(plan.read_bytes()).hexdigest(),
                stepSha256=hashlib.sha256(step.read_bytes()).hexdigest(),rows=rows)


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    for key in ('plan','step','output'):
        parser.add_argument(key,type=Path)
    args=parser.parse_args()
    result=analyze(args.plan,args.step)
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print('distinct suppliers',sum(row['maximumMatrixDifference']<.001 for row in result['rows'].values()),
          '/',len(result['rows']))
