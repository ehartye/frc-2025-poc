"""Run merge_study.py unchanged except that its plugin-commit gate admits exactly one reviewed pair.

part-2 (266,-158) was rerun on agent-frc fd251a5 after its 7e1ed2d run failed (part-2-failed.log). Between the two
commits the only scripts/ change the study can reach is mechanical_motion.py, where bounds, common and bearing_area are
unchanged (transform was refactored into transforms + transform; evaluate gained keyword options the study does not use).
Inputs must still be identical. The merged record names each part's commit.

usage: python -I -B merge_mixed_commit.py PART_DIR OUT_DIR
"""
import hashlib, json, sys, types
from pathlib import Path

ALLOWED = {"7e1ed2dccd67e8f3fdeb023c295ff2d364090140", "fd251a52e89cd04e56e9ea495b8f6ec3d54f4185"}
GATE = 'p["plugin"]["commit"] != base["plugin"]["commit"]'


def main(part_dir, out_dir):
    here = Path(__file__).resolve().parent
    source = (Path(part_dir) / "merge_study.py").read_text(encoding="utf-8")
    if source.count(GATE) != 1: raise SystemExit("merge_study.py gate changed; review the wrapper")
    patched = source.replace(GATE, 'p["plugin"]["commit"] not in ALLOWED')
    m = types.ModuleType("merge_study"); m.ALLOWED = ALLOWED
    exec(compile(patched.replace('if __name__ == "__main__":', "if False:"), "merge_study.py", "exec"), m.__dict__)
    m.main(part_dir, out_dir)
    out = Path(out_dir) / "seated-screw-study.json"
    study = json.loads(out.read_bytes())
    commits = {}
    for f in sorted(Path(part_dir).glob("part-*.json")):
        commits[f.name] = json.loads(f.read_bytes())["plugin"]["commit"]
    study["execution"]["pluginCommits"] = commits
    study["execution"]["mixedCommitBasis"] = __doc__.split("usage:")[0].strip()
    study["execution"]["mergeWrapperSha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    out.write_text(json.dumps(study, indent=1) + "\n")
    (Path(out_dir) / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    (Path(out_dir) / "part-2-failed.log").write_bytes((Path(part_dir) / "part-2-failed.log").read_bytes())


if __name__ == "__main__":
    main(*sys.argv[1:3])
