"""Prepare one supplier configuration from its intact original occurrence tree."""
import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path

from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader, STEPCAFControl_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
try:
    from OCP.TDF import TDF_LabelSequence
except ImportError:
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def label_name(label):
    attribute = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attribute):
        return attribute.Get().ToExtString()
    return ''


def prepare(source, register, destination):
    if digest(source) != 'ee5e74af75e1b89fde94e37dbfbafc0b1e131a018c08cb906877d75cc1a34720':
        raise ValueError('The selected NEO 2023-08-16 supplier exchange changed')
    if digest(register) != '7fb6a4b961769925601fb4628c8831dcb3bfa2eb39ea7200e005f5dbd16a554f':
        raise ValueError('The commissioned option register changed')
    record = json.loads(register.read_bytes())
    options = record['selectableFamilies']['L2']
    excluded = set(options['excluded'])
    if len(excluded) != 4 or len(record['commonSolids']) + len(options['selected']) != 121:
        raise ValueError('Unexpected source configuration')
    destination.mkdir(parents=True, exist_ok=True)
    output = destination / 'sds-mk4i-neo-l2.step'
    if output.exists():
        raise FileExistsError(output)
    doc = TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    table = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    reader = STEPCAFControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError('STEP supplier import failed')
    roots = TDF_LabelSequence()
    table.GetFreeShapes(roots)
    if roots.Length() != 1:
        raise ValueError('Expected exactly one supplier installation root')
    root = roots.Value(1)
    removed = []

    def find_options(assembly):
        children = TDF_LabelSequence()
        table.GetComponents_s(assembly, children)
        for i in range(1, children.Length() + 1):
            item, definition = children.Value(i), TDF_Label()
            if not table.GetReferredShape_s(item, definition):
                raise ValueError('Unresolved supplier occurrence')
            name = label_name(definition)
            if name in excluded:
                removed.append((name, item))
            else:
                find_options(definition)

    find_options(root)
    if Counter(n for n, _ in removed) != Counter(excluded):
        raise ValueError('Alternative gears must each resolve to exactly one instance')
    for name, occurrence in removed:
        table.RemoveComponent(occurrence)
    table.UpdateAssemblies()
    writer = STEPCAFControl_Writer()
    if not writer.Transfer(root) or writer.Write(str(output)) != IFSelect_RetDone:
        raise ValueError('Selected source root failed to export')
    return dict(sourceSha256=digest(source), selectorSha256=digest(register), configuration=options,
                derivedStepSha256=digest(output), selectedCount=121,
                scope='Supplier STEP option preparation. Actual native chassis installation and steering remain pending.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source', 'register', 'destination'):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    result = prepare(args.source, args.register, args.destination)
    (args.destination / 'selection.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
