"""Carry an actual acquired pipe through the first four-wheel lease and braking tail."""

import argparse
from contextlib import ExitStack
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "motion"))


def run(case, clocks, traction_loss=False):
    import departure_contact
    import first_service

    setup = json.loads((case / "result.json").read_bytes())
    feed = setup["feed"]
    evidence = json.loads((case / "qualified-rims-core.json").read_bytes())
    if (
        not evidence["observations"]["nominal"]["start"]
        or feed["pipeStateSha256"] != evidence["terminalStateSha256"]
    ):
        raise ValueError(
            "Required independent camera custody is absent for this source state"
        )
    coupled = first_service.engine.coupled
    shelf = coupled.Shelf
    plate = sys.modules[coupled.plate.__module__]
    out = case / "loaded"
    out.mkdir(exist_ok=True)
    with ExitStack() as context:
        context.enter_context(patch.object(plate, "E_PE", setup["modulusPa"]))
        context.enter_context(patch.object(plate, "RHO_PE", setup["densityKgM3"]))
        context.enter_context(
            patch.object(
                coupled,
                "Shelf",
                lambda *a, **k: replace(
                    shelf(*a, **k), friction=setup["shelfFriction"]
                ),
            )
        )
        coupled.tables.cache_clear()
        service = (
            json.loads((out / "result.json").read_bytes())["service"]
            if traction_loss
            else first_service.propagate(case, feed, out)
        )
        if service["loss"] or service["hardStop"]:
            raise ValueError("An actual wheel-service state is required")
        sign = feed["case"]
        # Both historical programs have same-named field modules. Each normal
        # entry point must load its own field layout in a clean interpreter.
        if not traction_loss:
            (out / "service.json").write_text(json.dumps(service), encoding="utf8")
            subprocess.run(
                [sys.executable, __file__, str(case), str(clocks), "--wheel-stage"],
                check=True,
            )
        drive = json.loads((out / "wheel.json").read_bytes())
        target = out
        if traction_loss:
            # Fault occurs at the first wheel service. All earlier light,
            # encoders, ports, modal states and the attained cargo pose are identical.
            target = out / "no-belt-traction"
            target.mkdir(exist_ok=True)
            context.enter_context(
                patch.object(
                    first_service.engine.feed,
                    "cfg",
                    replace(first_service.engine.feed.cfg, belt_mu=0.0),
                )
            )
        contact = departure_contact.run(
            case,
            feed,
            service,
            out / f"{sign}-{feed['plantStepUs']:g}us-service.npz",
            (out / f"{sign}-600.npz", drive),
            target,
        )
    receipt = {
        "materialState": evidence["terminalStateSha256"],
        "service": service,
        "drive": drive,
        "contact": contact,
    }
    (target / "result.json").write_text(
        json.dumps(receipt, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
    print(
        case.name,
        contact["lost"],
        contact["hardStop"],
        contact["peakTravelMm"],
        flush=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", type=Path)
    parser.add_argument("clocks", type=Path)
    parser.add_argument("--wheel-stage", action="store_true")
    parser.add_argument("--lose-belt-traction", action="store_true")
    args = parser.parse_args()
    if args.wheel_stage:
        import departure_drive

        folder = args.case / "loaded"
        service = json.loads((folder / "service.json").read_bytes())
        drive = departure_drive.run(service["case"], args.clocks, 0.6, folder, service)
        (folder / "wheel.json").write_text(json.dumps(drive), encoding="utf8")
    else:
        run(args.case, args.clocks, args.lose_belt_traction)
