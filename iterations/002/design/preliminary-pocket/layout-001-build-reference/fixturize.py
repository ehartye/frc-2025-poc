"""Move one placed field group out of a preliminary package and register its solids as motion-study fixtures.

Usage: fixturize.py INPUT_DIR GROUP FIXTURE_STEP BASIS_JSON [--frozen REGEX]
BASIS_JSON maps each fixture ID to its source, dimensions and placement basis (the motion study's `basis`).
The robot solids are re-exported unchanged by name; every SHA-256 that named a changed file is re-propagated.
"""
import hashlib, json, re, sys
from pathlib import Path

sys.path.insert(0, r"PLUGIN/scripts")
from showcase_contacts import load_solids
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool

root, group, fixture_step, basis_file = Path(sys.argv[1]), sys.argv[2], sys.argv[3], sys.argv[4]
frozen = re.compile(sys.argv[sys.argv.index("--frozen") + 1]) if "--frozen" in sys.argv else re.compile(r"^receipt")
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
rd = lambda n: json.loads((root / n).read_text(encoding="utf-8-sig"))
def wr(n, obj): (root / n).write_text(json.dumps(obj, indent=2, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")

layout = rd("layout.json")
members = layout["groups"][group]["members"]
basis = json.loads(Path(basis_file).read_text(encoding="utf-8"))
assert set(basis) == set(members), "Give every fixture its source, dimensions and placement basis"
step_name = layout["step"]["file"]
old = {}
for p in root.rglob("*"):
    if p.is_file(): old[p.relative_to(root).as_posix()] = sha(p)

solids, non = load_solids(root / step_name)
assert not non and set(members) <= set(solids)

def write(path, names):
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    table = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    for n in names:
        TDataStd_Name.Set_s(table.AddShape(solids[n], False), TCollection_ExtendedString(n))
    w = STEPCAFControl_Writer()
    assert w.Transfer(doc, STEPControl_AsIs) and w.Write(str(path)) == IFSelect_RetDone

write(root / step_name, sorted(n for n in solids if n not in members))
write(root / fixture_step, sorted(members))
again, _ = load_solids(root / step_name); fx, _ = load_solids(root / fixture_step)
assert set(again) == set(solids) - set(members) and set(fx) == set(members)

# Motion study: every case places the same fixtures from the pinned fixture STEP.
motion_name = layout["motion"]["file"]
motion = rd(motion_name)
pin = dict(file=fixture_step, sha256=sha(root / fixture_step))
for case in motion["cases"]:
    case["fixtures"] = [dict(id=n, file=pin, solid=n, basis=basis[n]) for n in sorted(members)]
wr(motion_name, motion)

# The checker's roster is the placed group members, which equals the package STEP plus its fixtures,
# so the group, its zero mass line and its explanations stay; only the solids move to the fixture STEP.
gone = []
# Propagate every changed digest through the editable JSON until stable.
editable = [p.relative_to(root).as_posix() for p in root.rglob("*.json") if not frozen.search(p.name)]
for _ in range(50):
    now = {n: sha(root / n) for n in old if (root / n).exists()}
    swaps = {old[n]: now[n] for n in now if old[n] != now[n]}
    changed = False
    for n in editable:
        t = (root / n).read_text(encoding="utf-8")
        u = t
        for a, b in swaps.items(): u = u.replace(a, b)
        if u != t: (root / n).write_text(u, encoding="utf-8", newline="\n"); changed = True
    old.update(now)
    if not changed: break
else:
    raise SystemExit("digest propagation did not converge")
print(json.dumps(dict(fixtures=sorted(members), removedMassLines=gone, step=sha(root / step_name), fixtureStep=pin["sha256"]), indent=1))
