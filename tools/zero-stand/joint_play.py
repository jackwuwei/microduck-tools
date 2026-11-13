"""How far each joint can turn away from zero before the robot meets the stand.

Each joint is turned on its own, everything downstream of it moving with it and
everything else held, in small steps either way, until any moved point of the
robot comes within TOUCH_MM of the stand's surface (or of the floor, z = 0, for
points that start above it). The angle reached is the play the stand allows that
joint; a joint the stand holds at zero shows a small play both ways.

That whole-subtree test is dominated by the foot: every leg joint carries the
foot, the sole rims hold it, so every leg joint stops almost at once. With
--body-only only the body the joint drives turns (its children stay put), which
asks the question that matters for a joint: does the stand itself hold the body
on the moving side of it? The part that touches first is printed.

--depth N moves the joint's body and N-1 levels below it (--body-only is
--depth 1): with the next joint held at zero by the stand, the two bodies move
as one (hip roll with hip pitch held, say).

usage: python joint_play.py [--body-only | --depth N] <robot_walk.xml with assets/> <stand.stl> [joint ...]
"""
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
TOUCH_MM = 0.05
STEP_DEG = 0.05
MAX_DEG = 10.0
PER_PART = 20000

args = sys.argv[1:]
DEPTH = None
if "--body-only" in args:
    DEPTH = 1
    args.remove("--body-only")
if "--depth" in args:
    i = args.index("--depth")
    DEPTH = int(args[i + 1])
    del args[i:i + 2]
mjcf, stand_stl = Path(args[0]), Path(args[1])
wanted = args[2:] or ["left_hip_roll", "left_hip_pitch", "right_hip_roll", "right_hip_pitch",
                          "left_hip_yaw", "right_hip_yaw", "left_knee", "right_knee", "left_ankle", "right_ankle"]


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


# joints (world axis at zero pose) and each body's subtree
root = ET.parse(mjcf).getroot()
joints, subtree, level = {}, {}, {}


def walk(b, Tp, ancestors):
    T = Tp @ tf(b)
    name = b.get("name")
    for k, a in enumerate(ancestors + [name]):
        subtree.setdefault(a, set()).add(name)
        level[(a, name)] = len(ancestors) - k
    for j in b.findall("joint"):
        if j.get("type", "hinge") == "hinge":
            axis = T[:3, :3] @ np.array(j.get("axis", "0 0 1").split(), float)
            joints[j.get("name")] = (name, T[:3, 3] * 1000.0, axis / np.linalg.norm(axis))
    for c in b.findall("body"):
        walk(c, T, ancestors + [name])


for b in root.find("worldbody").findall("body"):
    walk(b, np.eye(4), [])

parts = dict(np.load(HERE / "ref" / "parts.npz"))
SHIFT = float(np.load(HERE / "ref" / "floor_shift.npy"))  # parts are lowered by this (soles on z = 0)
pts_by_body, key_by_body = {}, {}
for key, p in parts.items():
    body = key.split("__", 1)[0]
    q = p[:: max(1, len(p) // PER_PART)]
    pts_by_body.setdefault(body, []).append(q)
    key_by_body.setdefault(body, []).extend([key.replace("__", "/")] * len(q))
pts_by_body = {k: np.concatenate(v) for k, v in pts_by_body.items()}

d = stand_stl.read_bytes()
n = struct.unpack("<I", d[80:84])[0]
tris = np.frombuffer(d[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"].astype(float)
e = np.linalg.norm(np.stack([tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 1], tris[:, 0] - tris[:, 2]], 1), axis=2).max(1)
k = np.maximum(1, np.ceil(e / 0.4)).astype(int)
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

def rot(axis, deg):
    t = np.radians(deg)
    a = axis
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


print("mode:", f"{DEPTH} level(s) of bodies move, the rest held" if DEPTH else "whole subtree")
print(f"{'joint':18s} {'ID':>4s} {'play -':>8s} {'play +':>8s}   first touch (-) / (+)")
IDS = {"left_hip_yaw": 20, "left_hip_roll": 21, "left_hip_pitch": 22, "left_knee": 23, "left_ankle": 24,
       "right_hip_yaw": 10, "right_hip_roll": 11, "right_hip_pitch": 12, "right_knee": 13, "right_ankle": 14}
for jn in wanted:
    body, origin, axis = joints[jn]
    origin = origin - [0.0, 0.0, SHIFT]
    bodies = [b for b in subtree[body] if b in pts_by_body and (DEPTH is None or level[(body, b)] < DEPTH)]
    moving = np.concatenate([pts_by_body[b] for b in bodies])
    labels = sum((key_by_body[b] for b in bodies), [])
    rel = moving - origin
    d0, _ = tree.query(moving)
    above_floor = moving[:, 2] > 0.5
    res, who = [], []
    for sign in (-1, 1):
        ang = 0.0
        hit, part = None, ""
        while ang < MAX_DEG:
            ang += STEP_DEG
            p = rel @ rot(axis, sign * ang).T + origin
            dist, _ = tree.query(p, distance_upper_bound=5.0)
            bad = ((dist < TOUCH_MM) & (d0 > TOUCH_MM)) | (above_floor & (p[:, 2] < 0.0))
            if np.any(bad):
                hit, part = ang, labels[int(np.flatnonzero(bad)[0])]
                break
        res.append(hit)
        who.append(part or "-")
    fmt = lambda v: f"{v:6.2f}°" if v is not None else f">{MAX_DEG:4.0f}° "
    print(f"{jn:18s} {IDS.get(jn, ''):>4} {fmt(res[0]):>8s} {fmt(res[1]):>8s}   {who[0]} / {who[1]}")
