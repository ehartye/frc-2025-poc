"""Bind the selected project's public sources before loading its engineering models."""

import hashlib
import json
from pathlib import Path

project = None
registry = json.loads(Path(__file__).with_name("inputs.json").read_bytes())
routes = registry["routeSteps"]


def configure(source):
    global project
    project = source.resolve(strict=True)
    for name, expected in registry["project"].items():
        with (project / name).open("rb") as stream:
            if hashlib.file_digest(stream, "sha256").hexdigest() != expected:
                raise ValueError(f"Unexpected project input {name}")
