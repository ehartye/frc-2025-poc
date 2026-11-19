"""Screen occupied-receiver transit leads with the same network as the pinned empty opening."""

import importlib.util
from pathlib import Path

import numpy as np


def supply_review(samples):
    path = Path(__file__).resolve().parents[1] / "power/electrical.py"
    spec = importlib.util.spec_from_file_location("opening_electrical", path)
    supply = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(supply)
    readings = np.asarray(samples)
    watts = supply.other_watts(len(readings), receiver="loaded")
    requests = {}
    for axis, phase_r, v_column, i_column in (
        ("Drive", supply.DRIVE_PHASE_R, 0, 1),
        ("Steer", supply.STEER_PHASE_R, 2, 3),
    ):
        amps = readings[:, i_column, :]
        esc_voltage = readings[:, v_column, :] + amps * phase_r
        for wheel, label in enumerate(supply.IDS):
            branch = label + axis
            requests[branch] = abs(esc_voltage[:, wheel])
            # Do not credit regenerative power. Bound the modeled source using
            # braking power magnitude, as in the original four-corner study.
            watts[branch] = 0.7 + abs(esc_voltage[:, wheel] * amps[:, wheel]) / 0.9
    result = {}
    for name, pack in supply.PACKS.items():
        branches, bus, terminals, _ = supply.source(watts, pack)
        if not np.isfinite(terminals).all():
            result[name] = {"supplied": False, "reason": "network voltage collapse"}
            continue
        controlled = np.stack(
            [terminals[:, branches.index(key)] for key in requests], axis=1
        )
        margins = controlled - np.stack(list(requests.values()), axis=1)
        result[name] = dict(
            minBusV=float(bus.min()),
            minControllerV=float(controlled.min()),
            minMotorHeadroomV=float(margins.min()),
            supplied=bool(controlled.min() >= 9.5 and margins.min() >= 0.0),
        )
    return result
