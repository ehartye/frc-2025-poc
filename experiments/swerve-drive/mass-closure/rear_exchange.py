"""Count the elevated rear host on its actual four-module installed inventory."""

import json
import os

from surrendered_stock import read, same, sha


def register(project, source_file, roster, output):
    """Preserve relocated part identities and remove each remachined ancestor once."""
    folder = project / "experiments/swerve-drive/packaging"
    hist = project / "experiments/six-wheel"
    report = read(folder / "rear-electronics.json")
    if report["materialHitsWithRearMotorsMm3"] or report["hostPortAndPostRiseMm"] != 23:
        raise ValueError("Rear repair is not the registered clear, 23 mm elevated installation")
    step = folder / "rear-electronics.step"
    if sha(step) != report["stepSha256"] or report["originalChassis"] != sha(
        folder / "swerve-frame.step"
    ):
        raise ValueError("Different drawing for the local electronics repair")
    from build_assembly import read_named, volume

    shapes = read_named(step, [{"id": k} for k in report["partsMm3"]])
    volumes = {name: volume(item)[0] for name, item in shapes.items()}
    for name, v in volumes.items():
        same(v, report["partsMm3"][name], 1e-4)
    old_guides = read(hist / "fixed-camera-route/guides.json")
    moved = {
        "fixed-leads/" + name.removeprefix("lifted-"): name
        for name in shapes
        if name.startswith("lifted-")
    }
    for key, new in moved.items():
        same(
            volumes[new],
            old_guides["volumeMm3"][key.removeprefix("fixed-leads/")],
            1e-4,
        )
    if len(moved) != 5 or not set(moved) <= roster:
        raise ValueError("Changed relocated posts or guide foot")
    changed = {
        "electrical-backplate": "edge-pocket-electrical-backplate",
        "observer-pi-and-converter-backplate": "edge-pocket-host-converter-backplate",
        "swerve/swerve-relieved-ribbed-deck": "host-drilled-swerve-deck",
    }
    for n in range(2):
        old = f"base-guide-rib-{n}"
        key = "routed-" + old
        if (
            old not in old_guides["supportBlanks"]
            or volumes[key] > old_guides["volumeMm3"][old] + 1e-4
        ):
            raise ValueError(
                "Replacement guide is not cut from the same registered shorter aluminum stock"
            )
        changed["fixed-leads/" + old] = key
    leads = {
        "shop-fixed-camera-usb-lead": "fixed-host-data-lead",
        "service-U3HLA01C12-020": "roof-host-data-lead",
    }
    if not set(changed) | set(leads) <= roster:
        raise ValueError("Rear repair must fit the selected four-module source")
    added_bodies = set(shapes) - set(moved.values()) - set(changed.values())
    expected = {"elevated-host-plate"} | {
        f"host-{label}-{n}"
        for n in range(1, 5)
        for label in ("upright", "through-screw", "washer", "nut")
    }
    if added_bodies != expected:
        raise ValueError("Unexpected new material in rear build")
    metal = {
        name: volumes[name]
        * (
            7850
            if name.startswith(("host-through", "host-washer", "host-nut"))
            else 2700
        )
        / 1e9
        for name in sorted(added_bodies)
    }
    # All three relieved parent plates lose material, as do both same-stock guide
    # ribs. Keep the whole originals charged. The complete new plate, fasteners,
    # four spacers and a positive electronics/finish allowance are additional.
    gross = sum(metal.values())
    cap = 1.10 * gross + 0.080
    same(cap, report["noChipUpperKg"], 1e-6)
    key = "elevated-rear-camera-host"
    additions = {
        key + "/" + name
        for name in set(changed.values()) | added_bodies | set(leads.values())
    }
    removed = set(changed) | set(leads)
    after = (roster - removed) | additions
    if roster & additions or len(after) != len(roster) + 17:
        raise ValueError("Rear host replacement counted twice")
    bill = dict(
        stepSha256=sha(step),
        sourceReceiptSha256=sha(folder / "rear-electronics.json"),
        oldGuideSourceSha256=sha(hist / "fixed-camera-route/guides.json"),
        changedNativeBodies=changed,
        relocatedNativeBodies=moved,
        replacementLeadRoles=leads,
        positiveNewMaterialKg=metal,
        machiningCreditKg=0,
        additionalCableAndFinishKg=0.080,
        upperAllowanceKg=cap,
        lowerAllowanceKg=0.0,
        parentCount=len(roster),
        installedCount=len(after),
        drawingKitShapes=len(shapes),
        incomingDataPathQualified=None,
    )
    source = read(source_file)
    output.mkdir(parents=True, exist_ok=True)
    for ref in source["sources"]:
        orig = (source_file.parent / ref["file"]).resolve()
        if sha(orig) != ref["sha256"]:
            raise ValueError("Changed parent source " + ref["id"])
        ref["file"] = os.path.relpath(orig, output).replace(os.sep, "/")

    def write(path, data):
        path.write_text(
            json.dumps(data, indent=2) + "\n", encoding="utf8", newline="\n"
        )

    changefile = output / "rear-material.json"
    write(changefile, dict(deltaKg=[0, cap], **bill))
    source["sources"].append(dict(id=key, file=changefile.name, sha256=sha(changefile)))
    for name in sorted(additions):
        bought = (
            "through-screw" in name
            or "washer-" in name
            or "nut-" in name
            or name.endswith("-data-lead")
        )
        source["instances"].append(
            dict(
                id=name,
                presence="candidate",
                disposition="undecided" if bought else "make",
            )
        )
    source["changes"].append(
        dict(
            id=key,
            revision=source["revision"],
            configuration=source["configuration"],
            add=sorted(additions),
            remove=sorted(removed),
            delta=dict(
                source=key, pointer="/deltaKg", rangeKg=[0, cap], basis="estimated"
            ),
        )
    )
    source["selected"].append(key)
    dest = output / "uncorrelated-parent-plus-rear.json"
    write(dest, source)
    return bill, after, dest
