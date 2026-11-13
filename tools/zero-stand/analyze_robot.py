"""Per-mesh dense samples of the zero-pose robot, plus XY maps for a vertical-drop stand.

Usage: python analyze_robot.py <robot_walk.xml with an assets/ folder of STL meshes> [out_dir]
The MJCF and meshes are Pollen's microduck sim model (robot_walk.xml +
assets/*.stl); out_dir defaults to ./ref next to this script.

bottom[ix, iy] : lowest robot z in each XY column (+inf where empty)
owner[ix, iy]  : index (into names) of the mesh that owns that lowest point
Grid: x from X0, y from Y0, step 0.5 mm. Robot frame, mm, soles on z = 0.
"""
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

MJCF = Path(sys.argv[1])
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parent / "ref"
A = MJCF.parent / "assets"
STEP = 0.5


def qm(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf(e):
    T = np.eye(4)
    if e.get("quat"):
        T[:3, :3] = qm(np.array(e.get("quat").split(), float))
    if e.get("pos"):
        T[:3, 3] = np.array(e.get("pos").split(), float)
    return T


def stl(p):
    d = p.read_bytes()
    n = struct.unpack("<I", d[80:84])[0]
    return np.frombuffer(d[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"].astype(float)


def dense(tris):
    e = np.stack([tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 1], tris[:, 0] - tris[:, 2]], 1)
    L = np.linalg.norm(e, axis=2).max(1)
    k = np.maximum(1, np.ceil(L / (STEP * 0.7))).astype(int)
    out = [tris.reshape(-1, 3)]
    for kk in np.unique(k):
        if kk == 1:
            continue
        T = tris[k == kk]
        i, j = np.meshgrid(np.arange(kk + 1), np.arange(kk + 1), indexing="ij")
        m = (i + j) <= kk
        a, b = (i[m] / kk)[None, :, None], (j[m] / kk)[None, :, None]
        out.append((T[:, None, 0] * (1 - a - b) + T[:, None, 1] * a + T[:, None, 2] * b).reshape(-1, 3))
    return np.concatenate(out)


root = ET.parse(MJCF).getroot()
parts = []  # (label, dense points, triangles)

# The sim model's pack (mesh "np_f970") is replaced by a standard Sony NP-F550:
# 38.2 (y) x 70.6 (z) x 20.5 (x) mm, front face on the mesh pack's front face
# (where it clips onto the power_support), centred on the mesh pack in y and z.
NPF550 = (20.5, 38.2, 70.6)


def box_tris(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v[np.array(f)]


def walk(b, Tp):
    T = Tp @ tf(b)
    for g in b.findall("geom"):
        if not g.get("mesh") or "collision" in (g.get("class") or ""):
            continue
        Tg = T @ tf(g)
        tris = stl(A / (g.get("mesh") + ".stl")) * 1000.0
        w = (tris.reshape(-1, 3) @ Tg[:3, :3].T + Tg[:3, 3] * 1000.0).reshape(-1, 3, 3)
        name = f"{b.get('name')}/{g.get('mesh')}"
        if g.get("mesh") == "np_f970":
            p = w.reshape(-1, 3)
            lo, hi = p.min(0), p.max(0)
            c = (lo + hi) / 2
            dx, dy, dz = NPF550
            w = box_tris((hi[0] - dx, c[1] - dy / 2, c[2] - dz / 2), (hi[0], c[1] + dy / 2, c[2] + dz / 2))
            name = f"{b.get('name')}/npf550"
        parts.append((name, dense(w), w))
    for c in b.findall("body"):
        walk(c, T)


for b in root.find("worldbody").findall("body"):
    walk(b, np.eye(4))

# put the soles on z = 0
floor = min(p[:, 2].min() for _, p, _ in parts)
parts = [(n, p - [0.0, 0.0, floor], t - [0.0, 0.0, floor]) for n, p, t in parts]
OUT.mkdir(parents=True, exist_ok=True)
np.save(OUT / "floor_shift.npy", np.float64(floor))  # mm the MJCF frame was lowered by
names = [p[0] for p in parts]
X0, X1, Y0, Y1 = -70.0, 100.0, -90.0, 90.0
nx, ny = int((X1 - X0) / STEP), int((Y1 - Y0) / STEP)
bottom = np.full((nx, ny), np.inf)
owner = np.full((nx, ny), -1)
for idx, (_, p, _) in enumerate(parts):
    ix = np.floor((p[:, 0] - X0) / STEP).astype(int)
    iy = np.floor((p[:, 1] - Y0) / STEP).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    cur = np.full((nx, ny), np.inf)
    np.minimum.at(cur, (ix[ok], iy[ok]), p[ok, 2])
    better = cur < bottom
    bottom[better] = cur[better]
    owner[better] = idx

np.savez(OUT / "drop.npz", bottom=bottom, owner=owner, names=np.array(names), x0=X0, y0=Y0, step=STEP)
np.savez(OUT / "parts.npz", **{n.replace("/", "__"): p[:: max(1, len(p) // 300000)] for n, p, _ in parts})
# the whole robot (with the NP-F550) as one STL, for renders and viewers
allt = np.concatenate([t for _, _, t in parts]).astype("<f4")
with open(OUT / "robot_zero_npf550.stl", "wb") as f:
    f.write(b"microduck zero pose + NP-F550".ljust(80, b" "))
    f.write(struct.pack("<I", len(allt)))
    for t in allt:
        f.write(b"\0" * 12 + t.tobytes() + b"\0\0")
for n, p, _ in parts:
    print(f"{n:45s} min {p.min(0).round(1)} max {p.max(0).round(1)}")
