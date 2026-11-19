"""Bind measured braking and retained cargo to the two simulated custody vetoes."""

import argparse
from pathlib import Path
import numpy as np
from register import read, sha, write


def collect(raw):
    cases = {}
    for sign in ("plus", "minus"):
        folder = raw / f"{sign}-31.25"
        clock = read(folder / "internal-clock.json")
        drive = read(folder / "hold-retry/route.json")
        prefix = read(folder / "internal-stop/route.json")
        cargo = read(folder / "internal-stop-contact/contact.json")
        samples = folder / "internal-stop-contact" / cargo["file"]
        if not (
            clock["decision"] == "brake"
            and clock["piece"] == drive["piece"] == cargo["piece"]
            and abs(clock["decisionWallS"] - drive["brakeWallS"]) < 1e-9
            and sha(samples) == cargo["sha256"]
            and sha(samples.with_suffix(".npz")) == cargo["terminalStateSha256"]
            and prefix["trajectory"] == cargo["sourceRoute"]
            and sha(folder / "hold-retry/feedback.npz") == drive["feedback"]
            and abs(
                cargo["startWallS"]
                + cargo["endRelativeS"]
                - drive["stopFeedback"]["wallS"]
            )
            < 1e-9
        ):
            raise ValueError(
                f"Brake, measured stop or cargo source does not join for {sign}"
            )
        with (
            np.load(folder / "internal-stop/path.npz") as a,
            np.load(folder / "hold-retry/path.npz") as b,
        ):
            for key in a.files:
                np.testing.assert_array_equal(a[key], b[key][: len(a[key])])
        cases[sign] = dict(
            piece=cargo["piece"],
            stepUs=cargo["stepUs"],
            clock=sha(folder / "internal-clock.json"),
            brakeWallS=drive["brakeWallS"],
            stopFeedback=drive["stopFeedback"],
            feedbackSha256=drive["feedback"],
            routeSha256=drive["trajectory"],
            cargo=cargo,
            stopField=prefix["field"],
            reacquisition="unverified",
            resumed=False,
            deliveries=0,
        )
    return dict(cases=cases)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    write(a.out, collect(a.artifacts))
