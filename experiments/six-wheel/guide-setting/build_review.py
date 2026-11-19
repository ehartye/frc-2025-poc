"""Publish the three survey examples with their registered station targets."""
import json
from survey import HERE, assess, example, load_layout


def build():
    probe = load_layout()
    cases = {name: assess(example(probe, name), probe)
             for name in ("nominal", "sideways", "both-outboard")}
    content = json.dumps(cases, allow_nan=False, separators=(",", ":"))
    (HERE / "examples.js").write_text("const cases = " + content + ";\n", encoding="utf8", newline="\n")


if __name__ == "__main__":
    build()
