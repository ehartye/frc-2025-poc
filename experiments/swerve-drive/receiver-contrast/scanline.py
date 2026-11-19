"""Draw the left roof pupil's central scanline through the same attained pipe."""

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from finish import select_views, witness


def plot(cache, output):
    row = next(
        r
        for r in witness.read(witness.HERE / "receiver-witness.json")["cases"]
        if r["case"] == "plus" and r["finish"] == "matched" and r["stepUs"] == 31.25
    )
    frame = row["frames"][4]
    card = Image.new("RGB", (900, 430), "#faf9f5")
    pen = ImageDraw.Draw(card)

    def label(pos, text, size, color="#202529"):
        pen.text(pos, text, fill=color, font=ImageFont.load_default(size=size))

    label((24, 14), "Can the camera see the back of the pipe?", 27)
    label(
        (24, 54),
        "R4 left roof pupil  /  capture 5440 ms  /  same pipe pose and robot geometry",
        15,
    )
    for index, (title, gray) in enumerate(
        (("Facing matches the pipe", 220), ("Dark bulk-polymer facing", 80))
    ):
        _, (actual, ref), _ = select_views(cache / "plus.npz", gray)
        images = witness.sensor.silhouette(
            actual, np.array(frame["trueCenterMm"]), np.array(frame["trueAxis"])
        )
        column = images[0].shape[1] // 2
        observed = images[0][:, column].astype(float)
        empty = ref[0][0][:, column].astype(float)
        x0 = 50 + index * 440
        top, bottom = 154, 334
        label((x0 - 22, 96), title, 19)
        pen.line((x0, top, x0, bottom, x0 + 376, bottom), fill="#555555", width=1)
        for value in (0, 128, 255):
            y = bottom - (bottom - top) * value / 255
            pen.line((x0, y, x0 + 376, y), fill="#dedbd1")
            label((x0 - 28, y - 8), str(value), 11)
        for a, color, wide in ((empty, "#167a64", 4), (observed, "#a54c24", 2)):
            pen.line(
                [
                    (x0 + 376 * i / (len(a) - 1), bottom - (bottom - top) * v / 255)
                    for i, v in enumerate(a)
                ],
                fill=color,
                width=wide,
            )
        for j in (0, 120, 240, 360, len(observed) - 1):
            label((x0 + 376 * j / (len(observed) - 1) - 8, bottom + 6), str(j), 11)
    label((26, 375), "Green  empty robot reference", 15, "#167a64")
    label((300, 375), "Orange  unmodified pipe in view", 15, "#a54c24")
    label(
        (26, 402),
        "Horizontal axis: image row in pixels.  Vertical axis: simulated 8-bit grayscale.",
        13,
    )
    card.save(output)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("cache", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    plot(args.cache, args.output)
