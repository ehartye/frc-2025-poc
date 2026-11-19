"""Draw a source-inspectable scanline through the two renders of one attained pipe."""

import argparse
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from finish import select_views, witness


def plot(cache, output):
    source = witness.read(witness.HERE / "receiver-witness.json")
    row = next(
        r
        for r in source["cases"]
        if r["case"] == "plus" and r["finish"] == "matched" and r["stepUs"] == 31.25
    )
    frame = row["frames"][4]
    svg = ET.Element(
        "svg", xmlns="http://www.w3.org/2000/svg", viewBox="0 0 900 430", role="img"
    )
    ET.SubElement(svg, "title").text = "The same pipe on matched and dark robot backing"
    ET.SubElement(
        svg, "desc"
    ).text = "The occupied and empty scanlines overlap on equal-brightness surfaces. Darkening the surveyed robot part makes a rear edge visible in the overhead image."
    ET.SubElement(svg, "rect", width="900", height="430", fill="#faf9f5")

    def label(x, y, content, size, color="#202529"):
        element = ET.SubElement(svg, "text", x=str(x), y=str(y), fill=color)
        element.set("font-size", str(size))
        element.set("font-family", "system-ui, sans-serif")
        element.text = content

    def line(points, color, width):
        element = ET.SubElement(
            svg,
            "polyline",
            points=" ".join(f"{x:.2f},{y:.2f}" for x, y in points),
            stroke=color,
            fill="none",
        )
        element.set("stroke-width", str(width))

    label(24, 38, "Can the camera see the back of the pipe?", 27)
    label(
        24,
        67,
        "R4 left roof pupil  /  capture 5440 ms  /  same pipe pose and robot geometry",
        15,
    )
    for index, (title, gray) in enumerate(
        (("Facing matches the pipe", 220), ("Dark bulk-polymer facing", 80))
    ):
        _, (actual, ref), _ = select_views(cache / "plus.npz", gray)
        image = witness.sensor.silhouette(
            actual, np.array(frame["trueCenterMm"]), np.array(frame["trueAxis"])
        )[0]
        column = image.shape[1] // 2
        x0 = 50 + index * 440
        top, bottom = 154, 334
        label(x0 - 22, 114, title, 19)
        line([(x0, top), (x0, bottom), (x0 + 376, bottom)], "#555555", 1)
        for value in (0, 128, 255):
            y = bottom - (bottom - top) * value / 255
            line([(x0, y), (x0 + 376, y)], "#dedbd1", 1)
            label(x0 - 28, y + 4, str(value), 11)
        for values, color, width in (
            (ref[0][0][:, column], "#167a64", 4),
            (image[:, column], "#a54c24", 2),
        ):
            # Adjacent equal samples need only the endpoints of each horizontal run.
            keep = {0, len(values) - 1}
            for j in np.flatnonzero(np.diff(values)):
                keep.update((int(j), int(j + 1)))
            line(
                [
                    (
                        x0 + 376 * i / (len(values) - 1),
                        bottom - (bottom - top) * float(values[i]) / 255,
                    )
                    for i in sorted(keep)
                ],
                color,
                width,
            )
        for j in (0, 120, 240, 360, len(image) - 1):
            label(x0 + 376 * j / (len(image) - 1) - 8, bottom + 19, str(j), 11)
    label(26, 389, "Green  empty robot reference", 15, "#167a64")
    label(300, 389, "Orange  unmodified pipe in view", 15, "#a54c24")
    label(
        26,
        417,
        "Horizontal axis: image row in pixels. Vertical axis: simulated 8-bit grayscale.",
        13,
    )
    ET.indent(svg)
    output.write_text(
        ET.tostring(svg, encoding="unicode") + "\n", encoding="utf8", newline="\n"
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("cache", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    plot(args.cache, args.output)
