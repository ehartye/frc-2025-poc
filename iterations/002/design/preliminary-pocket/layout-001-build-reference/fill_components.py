"""Fill presentation.components name, subsystem and volumeType for every placed ID (Agent FRC 0.90 inspector).

Usage: fill_components.py LAYOUT.json [--write]
"""
import json, re, sys

path = sys.argv[1]
plan = json.load(open(path, encoding="utf-8"))
comps = plan["presentation"].setdefault("components", {})
group_of = {m: g for g, spec in plan["groups"].items() for m in spec["members"]}
role_of = {m: plan["groups"][g]["role"] for m, g in group_of.items()}
ENVELOPE = re.compile(r"envelope|sweep|reserve|loom|trunk|lead|housing|module body|module motors|electronics panel|"
                      r"battery tray|dressed battery|plug and bracket|steer|azimuth|cassettes|sills|trusses|perimeter|"
                      r"field|hub |tower |trough|bumper|SPARK MAX|radio|roboRIO|distribution|breaker|reducer|motor|winch|belt module|servo|switch|sensor", re.I)
report = []
for member in sorted(group_of):
    c = comps.setdefault(member, {})
    c.setdefault("name", member[0].upper() + member[1:])
    c.setdefault("subsystem", group_of[member])
    if "volumeType" not in c:
        if role_of[member] == "service" or "hand" in member.lower():
            c["volumeType"] = "gauge"
        elif ENVELOPE.search(member) or role_of[member] in ("drive",):
            c["volumeType"] = "envelope"
        else:
            c["volumeType"] = "material"
    for key in ("category", "material", "purpose", "rationale"):
        if not c.get(key):
            report.append((member, key))
print(json.dumps({"components": len(group_of), "missing": report,
                  "volumeTypes": {t: sum(1 for c in comps.values() if c.get("volumeType") == t) for t in ("material", "envelope", "gauge")}}, indent=1))
if "--write" in sys.argv:
    open(path, "w", encoding="utf-8", newline="\n").write(json.dumps(plan, indent=2, ensure_ascii=False) + "\n")
