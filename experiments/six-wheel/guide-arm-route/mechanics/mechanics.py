"""Screen the routed arm or replay its custody on a source-pinned station assembly."""

import argparse
import importlib.util
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import project_inputs


def local(name):
    path = Path(__file__).with_name(name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--project",
        type=Path,
        required=True,
        help="Checkout containing the registered guide-impact and carriage studies",
    )
    parser.add_argument(
        "--out", type=Path, required=True, help="Destination for generated records"
    )
    actions = parser.add_subparsers(dest="action", required=True)
    static = actions.add_parser(
        "static", help="Screen both prior histories and candidate cross sections"
    )
    static.add_argument(
        "--raw",
        type=Path,
        required=True,
        help="Original guide-impact raw trial directories",
    )
    static.add_argument(
        "--routes",
        type=Path,
        required=True,
        help="Folder containing the four regenerated route STEP exchanges",
    )
    distributed = actions.add_parser(
        "distributed",
        help="Reconstruct all registered lateral contact forces against both arms",
    )
    distributed.add_argument(
        "--raw",
        type=Path,
        required=True,
        help="Original guide-impact raw trial directories",
    )
    trial = actions.add_parser(
        "trial", help="Run one signed station trial on its actual Java controller"
    )
    trial.add_argument("--case", choices=("leftSmall", "rightSmall"), required=True)
    trial.add_argument("--step", choices=(31.25, 15.625), type=float, required=True)
    trial.add_argument("--jdk", type=Path, required=True)
    trial.add_argument(
        "--configuration",
        type=Path,
        required=True,
        help="Folder containing selected route.step and revision.json",
    )
    handoff = actions.add_parser(
        "handoff", help="Reopen every raw run and write the terminal optical packets"
    )
    handoff.add_argument(
        "--raw",
        type=Path,
        required=True,
        help="The four completed successor trial directories",
    )
    handoff.add_argument("--configuration", type=Path, required=True)
    args = parser.parse_args()
    project_inputs.configure(args.project)
    args.out.mkdir(parents=True, exist_ok=True)
    screen = local("screen")
    if args.action == "static":
        screen.run(args.raw, args.routes, args.out)
        return
    joined = local("joined")
    if args.action == "distributed":
        local("distributed").run(args.raw, args.out)
    elif args.action == "trial":
        args.output = args.out
        joined.run(args)
    else:
        runs, frames = local("handoff").run(
            args.raw, args.configuration, args.out / "windows"
        )
        summary = args.out / "joined.json"
        joined.replay.write(summary, runs)
        frames["source"] = joined.prior.digest(summary)
        joined.replay.write(args.out / "station-frames.json", frames)
        for row in runs["cases"]:
            print(row["id"], row["lastState"], row["lastJavaMs"], row["firstBandMs"])


if __name__ == "__main__":
    main()
