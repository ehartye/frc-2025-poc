"""Correct the actual loaded cam robot from its retained reef stop."""

import argparse
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "reef-correction"))

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("side", choices=("plus", "minus"))
    p.add_argument(
        "--mode", choices=("nominal", "lost-packets", "missing-eye"), default="nominal"
    )
    p.add_argument("--advance-mm", type=float, default=25)
    p.add_argument(
        "--cargo",
        action="store_true",
        help="Advance the same pipe along the registered corrected wheel trace",
    )
    args = p.parse_args()
    if args.cargo:
        from importlib.util import spec_from_file_location, module_from_spec

        sys.path.insert(0, str(HERE.parent / "cam-scoring"))
        import captivity

        spec = spec_from_file_location(
            "cam_placement_cargo", HERE.parent / "reef-correction/cargo.py"
        )
        cargo = module_from_spec(spec)
        spec.loader.exec_module(cargo)
        print(
            cargo.run(
                args.artifacts,
                args.side,
                cam_run=args.artifacts
                / "low-placement"
                / f"advance-{args.advance_mm:g}",
            )
        )
        sys.exit(0)
    import approach

    result = approach.run(
        args.artifacts,
        args.side,
        args.mode,
        cam_run=args.artifacts / "low-placement" / f"advance-{args.advance_mm:g}",
        forward_mm=args.advance_mm,
    )
    print(
        {
            key: result.get(key)
            for key in ("case", "endWallS", "reason", "terminalMmMmRad", "enclosure")
        }
    )
