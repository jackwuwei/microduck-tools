"""Check the stand STL against the zero-pose robot.

1. Drop-in: every stand surface point lies below the lowest robot point of its
   XY column (so the robot lowers straight in and never overlaps the stand).
2. Contacts: the gap at each locating feature, from the robot's own meshes.
"""
import struct
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import minimum_filter

HERE = Path(__file__).resolve().parent
stl = Path(sys.argv[1])
d = stl.read_bytes()
if d[:5] == b"solid" and b"facet" in d[:300]:
    v = np.array([[float(a) for a in l.split()[1:4]] for l in d.decode().splitlines() if l.strip().startswith("vertex")])
    tris = v.reshape(-1, 3, 3)
else:
    n = struct.unpack("<I", d[80:84])[0]
    tris = np.frombuffer(d[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"].astype(float)

# dense surface samples of the stand
pts = [tris.reshape(-1, 3)]
for a, b in ((1 / 3, 1 / 3), (0.6, 0.2), (0.2, 0.6), (0.2, 0.2), (0.5, 0.5), (0.5, 0), (0, 0.5)):
    pts.append(tris[:, 0] * (1 - a - b) + tris[:, 1] * a + tris[:, 2] * b)
P = np.concatenate(pts)

dr = np.load(HERE / "ref" / "drop.npz")
bottom, x0, y0, st = dr["bottom"], float(dr["x0"]), float(dr["y0"]), float(dr["step"])
# Vertical locating faces sit 0.6 mm beside the robot, so a column test is exact
# only for points inside a cell; lateral gaps are reported under "contacts".
bottom_n = bottom
ix = np.floor((P[:, 0] - x0) / st).astype(int)
iy = np.floor((P[:, 1] - y0) / st).astype(int)
ok = (ix >= 0) & (ix < bottom.shape[0]) & (iy >= 0) & (iy < bottom.shape[1])
slack = np.full(len(P), np.inf)
slack[ok] = bottom_n[ix[ok], iy[ok]] - P[ok, 2]
floor = P[:, 2] <= 0.01           # the base top the soles stand on
bad = (slack < 0.3) & ~(floor & (slack >= -0.01))
print(f"drop-in check: {len(P)} stand surface samples, min vertical slack {slack.min():.2f} mm, violations {bad.sum()}")
if bad.any():
    q = P[bad]
    print("  violations span", q.min(0).round(1), q.max(0).round(1))

parts = dict(np.load(HERE / "ref" / "parts.npz"))

# dense stand surface (every triangle subdivided to <= 0.7 mm edges)
from scipy.spatial import cKDTree
e = np.linalg.norm(np.stack([tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 1], tris[:, 0] - tris[:, 2]], 1), axis=2).max(1)
k = np.maximum(1, np.ceil(e / 0.7)).astype(int)
S = [tris.reshape(-1, 3)]
for kk in np.unique(k):
    if kk == 1:
        continue
    Tk = tris[k == kk]
    i_, j_ = np.meshgrid(np.arange(kk + 1), np.arange(kk + 1), indexing="ij")
    m_ = (i_ + j_) <= kk
    a_, b_ = (i_[m_] / kk)[None, :, None], (j_[m_] / kk)[None, :, None]
    S.append((Tk[:, None, 0] * (1 - a_ - b_) + Tk[:, None, 1] * a_ + Tk[:, None, 2] * b_).reshape(-1, 3))
tree = cKDTree(np.concatenate(S))

print("\nnearest stand surface to each robot part (contact = locating face, 0.6 mm design gap):")
rows = []
for name, pts in parts.items():
    dist, _ = tree.query(pts, distance_upper_bound=5.0)
    rows.append((dist.min(), name.replace("__", "/"), int((dist < 1.0).sum())))
for dmin, name, n in sorted(rows):
    if dmin < 5.0:
        tag = "contact" if dmin < 1.0 else "near"
        print(f"  {dmin:5.2f} mm  {tag:7s} {name}  ({n} samples within 1 mm)")
bat = [r for r in rows if r[1].endswith("npf550")][0]
print(f"battery (NP-F550, 38.2 x 70.6 x 20.5 mm): nearest stand surface {bat[0]:.2f} mm" if bat[0] < 5 else "battery (NP-F550, 38.2 x 70.6 x 20.5 mm): no stand surface within 5 mm")
