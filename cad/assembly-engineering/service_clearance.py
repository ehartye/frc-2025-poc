"""Test the changed harness hardware through the registered receiver and cage motions."""

from attachments import base, bounds
from battery_fit import collisions
from clearance import evaluate
from installed_audit import HERE, sha
from issue import document, rows
from receiver_motion import assembly, material, OUT
from service_variant import PRODUCTION, RETIRED_ROOF, roof_changes


def run():
    original, installed, frames, _ = assembly()
    changes, _ = roof_changes()
    stops = base.named(OUT / "stop-bearings.step")
    installed.update(stops | changes)
    for key in RETIRED_ROOF:
        installed.pop(key)
    frame_by_id = {
        r["id"]: r["occurrence"].split(":")[0]
        for r in rows(HERE / "issue/cold-comparison.csv")
    }
    boxes = {k: bounds(s) for k, s in installed.items()}
    bodies = {}
    positive = {}
    for key, shape in (changes | stops).items():
        delta = material(key, shape, original) if key in original else shape
        if delta is None:
            continue
        # The bridge is machined only for holes and the pocket, with no
        # material addition. Test the new fixed supports, seats and fasteners.
        positive[key] = collisions(delta, installed, boxes, {key})
        test = {}
        for fraction in (0.5, 1.0):
            peers = [
                (j, s, boxes[j])
                for j, s in installed.items()
                if frames.get(j) == fraction
            ]
            test["rack-" + str(fraction)] = evaluate(
                delta, peers, (-1, 0, 0), 0, 370 * fraction
            )
        for group, axis, travel in [
            ("Cage crosshead", (0, 0, -1), 319.4),
            ("Left cage shoe", (0, 0, -1), 319.4),
            ("Right cage shoe", (0, 0, -1), 319.4),
            ("Lower plate clamp", (0, 0, -1), 319.4),
        ]:
            peers = [
                (j, s, boxes[j])
                for j, s in installed.items()
                if frame_by_id.get(j) == group
            ]
            test[group] = evaluate(delta, peers, axis, 0, travel)
        bodies[key] = test
    failed = [
        (k, run, r)
        for k, d in bodies.items()
        for run, v in d.items()
        for r in v["adverse"]
    ]
    document(
        OUT / "installed-clearance.json",
        dict(
            inputs=sha(PRODUCTION / "interfaces.json"),
            installed=len(installed),
            atRestIntersectionsMm3={k: v for k, v in positive.items() if v},
            swept=bodies,
            adverse=failed,
            scope="Changed positive material against relative motion, in an unladen robot. Rack translates 370 mm with a half-travel intermediate. Each cage member follows the published upward chassis-frame slider for 319.4 mm in the parked shoe setting. Published attachment perturbations separately check intended bolt threads and bearing. Continuous cage swing and load deflection require additional assessment.",
        ),
    )
    print("Swept failures", failed, flush=True)


if __name__ == "__main__":
    run()
