"""Per feature of the stand: its volume, and how much of its surface touches the robot.

A feature that touches nothing is only there to carry other features (or is waste).
Contact = surface within CONTACT_MM of the zero-pose robot (with the NP-F550).
"""
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

import zero_stand as zs

HERE = Path(__file__).resolve().parent
CONTACT_MM = 1.0

parts = dict(np.load(HERE / "ref" / "parts.npz"))
robot = np.concatenate(list(parts.values()))
tree = cKDTree(robot)

vol = defaultdict(float)
touch = defaultdict(float)
for name, solid in zs.features():
    vol[name] += solid.volume
    for face in solid.faces():
        area = face.area
        if area < 1e-6:
            continue
        # sample the face on a grid in its own parameter space
        pts = []
        for u in np.linspace(0.02, 0.98, 12):
            for v in np.linspace(0.02, 0.98, 12):
                try:
                    p = face.position_at(u, v)
                except Exception:
                    continue
                pts.append((p.X, p.Y, p.Z))
        if not pts:
            continue
        d, _ = tree.query(np.array(pts), distance_upper_bound=CONTACT_MM + 1e-9)
        touch[name] += area * float(np.mean(d <= CONTACT_MM))

print(f"{'feature':34s} {'volume cm3':>10s} {'contact mm2':>12s}")
for name in sorted(vol, key=lambda n: -vol[n]):
    print(f"{name:34s} {vol[name] / 1000:10.1f} {touch[name]:12.0f}")
print(f"{'total':34s} {sum(vol.values()) / 1000:10.1f}")
