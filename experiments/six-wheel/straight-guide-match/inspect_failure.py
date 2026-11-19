"""Reproject the stored state just before the first denied exposure."""

import sys, json
from pathlib import Path
import numpy as np
from camera import views, sensor, measure, peak

a = Path(sys.argv[1])
r = json.loads(Path(sys.argv[2]).read_bytes())
tr = np.load(a / "straight-guide-match" / (r["id"] + ".npy"))
_, opts, _ = views(a, "nominal", write_labels=False)
for elapsed in (0.133, 0.150, 0.151):
    i = np.abs(tr[:, 0] - elapsed).argmin()
    row = tr[i]
    for mode in ("nominal", "guarded"):
        actual, ref = opts[mode]
        ims = sensor.silhouette(actual, row[1:4] * 1000, row[4:7])
        f = measure(ims, ref, peak_bound=peak)
        print(row[0], mode, row[1:4] * 1000, f, flush=True)
