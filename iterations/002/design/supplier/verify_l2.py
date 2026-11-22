"""Read back the selected exchange and compare each actual supplier occurrence in its source frame."""
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path
import numpy as np
from itertools import product
from OCP.BRep import BRep_Tool
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopAbs import TopAbs_VERTEX, TopAbs_FACE, TopAbs_EDGE
from OCP.TopExp import TopExp
try:
    from OCP.TopTools import TopTools_IndexedMapOfShape
except ImportError:
    from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as TopTools_IndexedMapOfShape
from OCP.TopoDS import TopoDS
from select_l2 import digest


def mapped(solid, kind):
    result = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(solid, kind, result)
    return result


def points(solid):
    cast_vertex = getattr(TopoDS, 'Vertex_s', None) or TopoDS.Vertex
    vertices = mapped(solid, TopAbs_VERTEX)
    return np.asarray([BRep_Tool.Pnt_s(cast_vertex(vertices.FindKey(i))).Coord()
                       for i in range(1, vertices.Extent() + 1)])


def directed_nearest(first, second, tolerance=.0001):
    grid = defaultdict(list)
    for pt in second:
        grid[tuple(np.floor(pt/tolerance).astype(int))].append(pt)
    worst = 0.
    offsets = list(product((-1,0,1), repeat=3))
    for pt in first:
        x,y,z = np.floor(pt/tolerance).astype(int)
        nearest = min((np.linalg.norm(pt-q) for dx,dy,dz in offsets
                       for q in grid.get((x+dx,y+dy,z+dz), ())), default=float('inf'))
        worst = max(worst, nearest)
        if worst > tolerance:
            break
    return worst


def readback(plugin, source, register, issue):
    sys.path.insert(0, str(plugin / 'scripts'))
    from showcase_contacts import load_solids
    from component_shape import volume
    receipt = json.loads((issue / 'selection.json').read_bytes())
    out = issue / 'sds-mk4i-neo-l2.step'
    if [digest(p) for p in (source, register, out)] != [receipt[k] for k in ('sourceSha256','selectorSha256','derivedStepSha256')]:
        raise ValueError('Input digest changed during supplier preparation')
    original_labels, configured_labels = {}, {}
    original, a = load_solids(source, original_labels)
    configured, b = load_solids(out, configured_labels)
    common = json.loads(register.read_bytes())['commonSolids']
    expected = set(common + receipt['configuration']['selected'])
    if a or b or len(original) != 125 or len(expected) != 121 or len(configured) != 121:
        raise ValueError('Original or configured source contains unexpected bodies')
    by_definition = defaultdict(list)
    position = {}
    for key, shape in configured.items():
        by_definition[configured_labels[key]['definition']].append(key)
        position[key] = points(shape)
    results = {}
    allocated = set()
    for key in sorted(expected):
        shape = original[key]
        pts = points(shape)
        matches = []
        for candidate in by_definition[original_labels[key]['definition']]:
            other = position[candidate]
            if len(pts) != len(other) or np.linalg.norm(pts.mean(axis=0) - other.mean(axis=0)) > .001:
                continue
            distance = max(directed_nearest(pts, other), directed_nearest(other, pts))
            if distance < .0001:
                matches.append((candidate, float(distance)))
        if len(matches) != 1 or matches[0][0] in allocated:
            raise ValueError(f'Could not uniquely register purchased occurrence {key}: {matches}')
        candidate, error = matches[0]
        allocated.add(candidate)
        placed = configured[candidate]
        topology = [mapped(shape,k).Extent() for k in (TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE)]
        if topology != [mapped(placed,k).Extent() for k in (TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE)]:
            raise ValueError('Topology count changed: '+key)
        vol = volume(shape); delta = abs(vol-volume(placed))
        results[key] = dict(configuredOccurrence=candidate,
                            sourcePath=original_labels[key]['occurrencePath'],
                            configuredPath=configured_labels[candidate]['occurrencePath'],
                            vertexEdgeFaceCounts=topology, maxVertexDifferenceMm=error,
                            sourceVolumeMm3=vol, volumeDifferenceMm3=delta,
                            validTopology=[BRepCheck_Analyzer(obj).IsValid() for obj in (shape,placed)])
    return dict(selection=receipt, roster=results, compared=len(results),
                maxVertexDifferenceMm=max(v['maxVertexDifferenceMm'] for v in results.values()),
                maxVolumeDifferenceMm3=max(v['volumeDifferenceMm3'] for v in results.values()),
                occupiedVolumeExceptions={k:v['volumeDifferenceMm3'] for k,v in results.items() if v['volumeDifferenceMm3']>max(.001,abs(v['sourceVolumeMm3'])*1e-8)},
                topologyExceptions={k:v['validTopology'] for k,v in results.items() if not all(v['validTopology'])},
                scope='Same named source definition, unique installed location and all finite vertices, three topology counts, occupied volume and BRep validity per body. This is the supplier manufacturing representation, before Fusion installation.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('plugin','source','register','issue'):
        p.add_argument(name,type=Path)
    args=p.parse_args()
    result=readback(args.plugin,args.source,args.register,args.issue)
    (args.issue/'readback.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(result['compared'],result['maxVertexDifferenceMm'],result['maxVolumeDifferenceMm3'],result['topologyExceptions'])
