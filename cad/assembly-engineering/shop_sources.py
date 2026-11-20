"""Resolve an exact incremental shop drawing over its previously issued neutral parts."""

import json
from attachments import base
from installed_audit import HERE, sha


def issue_shapes(folder, register):
    inherited = {}
    if "parentIssue" in register:
        parent = HERE / register["parentIssue"]
        previous = json.loads((parent / "interfaces.json").read_bytes())
        if sha(parent / "interfaces.json") != register["parentRegisterSha256"]:
            raise ValueError("Changed parent registration")
        inherited.update(issue_shapes(parent, previous))
    step = folder / "station-and-jack-interfaces.step"
    if sha(step) != register["outputSha256"]:
        raise ValueError("Changed shop STEP")
    inherited.update(base.named(step))
    for key in register["fixedHeight"]["removed"]:
        inherited.pop(key, None)
    return inherited
