"""Read only the two digital ports on the issued 20 ms clock and replay their actual controller."""
import argparse
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "return-station"))
from feed import Pipe, port_program


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def port_frames(trace):
    pipe = Pipe((0, 0, 0), relief=True)
    for tick in range(0, int(trace[-1, 0] + 1e-6) + 1, 20):
        if tick <= 100:
            yield tick, False, False
            continue
        k = int(np.searchsorted(trace[:, 0], tick - 1e-6))
        if abs(trace[k, 0] - tick) > 1e-6:
            raise ValueError("Missing physical state at port exposure")
        pipe.p = trace[k, 1:4] / 1000
        pipe.a = trace[k, 4:7]
        yield tick, *pipe.ports()


def record(cache, jdk):
    issued = json.loads((ROOT / "station-landmark/feed.json").read_bytes())
    result = {}
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        src = td / "PortAcquisition.java"
        src.write_text(port_program(True), encoding="utf8", newline="\n")
        if digest(src) != issued["programSha256"]:
            raise ValueError("Changed controller")
        subprocess.run([str(jdk / "bin/javac"), "-d", str(td), str(src),
                        str(ROOT / "observable-acquisition/PortConsole.java")], check=True)
        with subprocess.Popen([str(jdk / "bin/java"), "-cp", str(td), "org.reefscape.controls.PortConsole"],
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1) as proc:
            for name, outcome in issued["outcomes"].items():
                path = cache / (name + ".npy")
                if digest(path) != outcome["trajectorySHA256"]:
                    raise ValueError("Changed physical trace: " + name)
                proc.stdin.write("COUNTERFEED\n")
                transitions = []
                history = []
                for tick, entry, seat in port_frames(np.load(path)):
                    proc.stdin.write(f"{tick},{tick},1,1,1,1,{int(entry)},{int(seat)}\n")
                    proc.stdin.flush()
                    state, _, speed, _ = proc.stdout.readline().strip().split(",")
                    sample = [int(entry), int(seat), state, float(speed)]
                    if not transitions or transitions[-1][1:] != sample:
                        transitions.append([tick, *sample])
                    history.append(sample)
                if state != outcome["lastPortState"]:
                    raise ValueError(f"Port replay disagrees with source in {name}")
                hard = outcome["hardStop"]
                result[name] = {"controllerEdges": transitions, "history": history,
                                "stopAtControllerMs": None if hard is None else round(100 + 1000*hard[0], 6),
                                "observedThroughMs": tick, "traceSha256": digest(path)}
            proc.stdin.close()
            proc.wait()
    for name, row in result.items():
        if row["stopAtControllerMs"] is not None:
            hist = row["history"]
            row["indistinguishableBeforeStop"] = [
                other for other, b in result.items()
                if b["stopAtControllerMs"] is None and b["history"][:len(hist)] == hist]
    for row in result.values():
        del row["history"]
    return {"portEpochWallUs": issued["epochWallUs"], "issuedFeedSha256": digest(ROOT/"station-landmark/feed.json"),
            "programSha256": issued["programSha256"], "trials": result}


if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("traces", type=Path)
    cli.add_argument("jdk", type=Path)
    args = cli.parse_args()
    (HERE / "port-witness.json").write_text(json.dumps(record(args.traces, args.jdk), indent=2)+"\n",
                                           encoding="utf8", newline="\n")
