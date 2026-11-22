"""Re-root a checked preliminary package into a self-contained release input directory.

Usage: reroot.py SOURCE_ROOT LAYOUT DEST --map OLD_PREFIX=NEW_PREFIX ... [--frozen REGEX] [--skip PATH]
Paths are rewritten textually in copied JSON and every SHA-256 pin is re-propagated to a fixed point.
"""
import argparse, hashlib, json, re, shutil
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("source"); p.add_argument("layout"); p.add_argument("dest")
p.add_argument("--map", action="append", default=[])
p.add_argument("--frozen", default=r"(^|/)receipt[^/]*\.json$")
p.add_argument("--skip", action="append", default=[])
a = p.parse_args()
src, dest = Path(a.source), Path(a.dest)
maps = [m.split("=", 1) for m in a.map]
sha = lambda b: hashlib.sha256(b).hexdigest()


def target(name):
    for old, new in maps:
        if name == old or (old.endswith("/") and name.startswith(old)):
            return new + name[len(old):] if old.endswith("/") else new
    return name


refs, todo, seen = {}, [a.layout], set()
def walk(o):
    if isinstance(o, dict):
        if isinstance(o.get("file"), str): yield o["file"]
        for k, v in o.items():
            if k == "view" and isinstance(v, str): continue
            yield from walk(v)
    elif isinstance(o, list):
        for v in o: yield from walk(v)

missing = []
while todo:
    f = todo.pop()
    if f in seen: continue
    seen.add(f)
    if f in a.skip: continue
    path = src / f
    if not path.is_file(): missing.append(f); continue
    refs[f] = target(f)
    if f.endswith(".json") and not re.search(a.frozen, f):
        todo += list(walk(json.loads(path.read_text(encoding="utf-8-sig"))))
dest.mkdir(parents=True, exist_ok=False)
for f, t in refs.items():
    out = dest / t; out.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src / f, out)
frozen = lambda t: bool(re.search(a.frozen, t))
editable = [t for t in refs.values() if t.endswith(".json") and not frozen(t)]
# Rewrite paths, longest first so prefixes do not clobber longer names.
for t in editable:
    text = (dest / t).read_text(encoding="utf-8-sig")
    for f in sorted(refs, key=len, reverse=True):
        if f != refs[f]:
            text = re.sub(r'(?<=")' + re.escape(f) + r'(?=")', refs[f], text)
    (dest / t).write_text(text, encoding="utf-8", newline="\n")
# Propagate digests to a fixed point.
original = {refs[f]: sha((src / f).read_bytes()) for f in refs}
for _ in range(50):
    current = {t: sha((dest / t).read_bytes()) for t in refs.values()}
    swaps = {original[t]: current[t] for t in current if original[t] != current[t]}
    changed = False
    for t in editable:
        text = (dest / t).read_text(encoding="utf-8")
        new = text
        for old, nv in swaps.items():
            new = new.replace(old, nv)
        if new != text:
            (dest / t).write_text(new, encoding="utf-8", newline="\n"); changed = True
    for t in current:
        if original[t] != current[t]:
            original[t] = current[t]  # later swaps start from the latest digest
    # swaps are cumulative: the last-seen digest is the one embedded now
    if not changed: break
else:
    raise SystemExit("digest propagation did not converge")
print(json.dumps(dict(copied=len(refs), missing=missing, map=refs), indent=1))
