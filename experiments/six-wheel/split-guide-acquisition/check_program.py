"""Reconstruct Java memory from the chronological public input stream, including the departure tick."""

import argparse, json, subprocess, tempfile
from pathlib import Path
from observability import evaluate, sample
from split_controller import candidate, digest
from split_checkpoint import ROOT, HERE, coupled


def run(jdk):
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        c = Path(tmp)
        program = c / "PortAcquisition.java"
        program.write_text(candidate(coupled.feed), encoding="utf8", newline="\n")
        subprocess.run(
            [
                str(jdk / "bin/javac"),
                "-d",
                str(c),
                str(program),
                str(ROOT / "observable-acquisition/PortConsole.java"),
            ],
            check=True,
        )
        for key in ("candidate", "locked", "traction"):
            for series in json.loads((HERE / (key + ".json")).read_bytes())["cases"]:
                if digest(program) != series["sourceProgramSha256"]:
                    raise ValueError("Different controller")
                old = series["portTicks"]
                loaded = evaluate(
                    c, jdk, [sample(t, entry, seat) for t, _, entry, seat, _ in old]
                )
                for expected, actual in zip(old, loaded, strict=True):
                    if (expected[0], expected[1], expected[-1]) != (
                        actual["t"],
                        actual["state"],
                        actual["command"],
                    ):
                        raise ValueError("Java state restoration failed")
                exposures = {}
                for ms in series["exposures"]:
                    q = next(r for r in reversed(loaded) if r["t"] <= int(ms))
                    if (
                        q["state"] != series["exposures"][ms]["javaState"]
                        or q["command"] != series["exposures"][ms]["javaCommandMps"]
                    ):
                        raise ValueError("Exposure command mismatch")
                    exposures[ms] = [q["t"], q["state"], q["command"]]
                results.append(
                    {
                        "case": key,
                        "side": series["side"],
                        "stepS": series["stepS"],
                        "inputFrames": len(old),
                        "lastOutput": loaded[-1],
                        "exposureFrameReplay": exposures,
                    }
                )
    return {"program": digest(HERE / "split_controller.py"), "cases": results}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("jdk", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.jdk), indent=2) + "\n", encoding="utf8", newline="\n"
    )
