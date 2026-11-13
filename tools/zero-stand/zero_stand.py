"""Fanduck zero-pose calibration stand (r3): a column the robot leans on from behind.

Layout follows the reference stand: a narrow column stands in FRONT of the
robot and the head rests on its T-shaped top; a shelf at hip height carries
the two hip servos and ends in cradles whose hooks stand in front of the
thighs; thin outer legs carry the shelf ends; the feet stand in rims on the
base. There is no back plate: at zero pose the centre of mass sits over the
toes, so the robot settles forward onto the stand.

Only locating faces and what carries them are kept (audit_features.py lists
each feature's volume and contact area). Locating faces, all 0.6 mm clear of
the zero-pose robot with an NP-F550:

- head rest  : a flat plate under the head (head z, pitch, roll)
- shelf      : under both hip servos (z = 64.5)
- cradles    : follow each thigh's sloped underside and turn up as a hook in
               front of its nose (x = 16.2)
- sole rims  : a closed rim around each sole (feet x, y, yaw)
- hip stops  : front, rear, outer side and a centre block on each hip-yaw link
- pins       : slid in from the front after the robot is seated, under the
               hip-pitch servos (12/22) on both sides of their axis (PIN_*;
               0.1-0.3 mm gap, zero_stand_pin.py makes them)

The trunk and the battery are not touched: with feet, hips, thighs and head
held, the trunk's pose follows from the joint chains.

Every feature obeys one rule, so the robot lowers straight in: stand material
only exists below the lowest robot point of its XY column (check_stand.py).

Frame: robot frame at zero pose, mm. +x forward (beak), +y left, +z up, soles
on z = 0. Print upright, base on the bed.
"""
from pathlib import Path

import numpy as np
from scipy.ndimage import grey_opening
from build123d import Align, Axis, Box, Face, Pos, Vector, Wire, extrude, fillet

HERE = Path(__file__).resolve().parent
DROP = HERE / "ref" / "drop.npz"

C = 0.6               # clearance to the robot everywhere

# base
BASE_T = 5.0
BASE_X = (-58.0, 60.0)
BASE_HALF_W = 77.0
RIM_H = 4.0
RIM_W = 3.0
BASE_CORNER_R = 8.0
WINDOW_R = 5.0         # round corners on every window

# robot faces the stand locates on (from ref/parts.npz, zero pose)
SHELL_FRONT = 34.5     # trunk shell front, |y| <= 32.3
SHELL_HALF_W = 32.3
NECK_FRONT = 36.0      # neck servo front, |y| <= 14.5
NECK_HALF_W = 14.5
HEAD_HALF_W = 45.88    # head shell sides

# spine
SPINE_X1 = 60.0
SPINE_HALF_W = 24.0
SPINE_X0_HIGH = NECK_FRONT + 2 * C    # above the neck contact the spine starts here
REST_X = (38.0, 56.0)                  # flat head rest: exactly the column depth (COL_X), no flat overhang
REST_HALF_W = 40.0                     # and |y| <= 40, under the head's flat underside
REST_T = 6.0                           # plate thickness above its gussets
SPINE_WINDOWS = ((8.0, 46.0), (84.0, 118.0), (140.0, 172.0))
WINDOW_HALF_W = 12.0

# column (the narrow upright that carries the head rest)
COL_X = (38.0, 56.0)
COL_HALF_W = 16.0
COL_WINDOWS = ((76.0, 112.0), (128.0, 170.0))  # solid below z 76, where the 45 deg tie meets it
COL_WINDOW_HALF_W = 8.0
HIP_SHELF_X1 = 21.0     # the hip servos' undersides end at x = 20.5
BASE_WINDOWS = ((-40.0, -8.0, -22.0, 22.0), (24.0, 34.0, -40.0, 40.0), (8.0, 20.0, 4.0, 22.0), (8.0, 20.0, -22.0, -4.0))  # clear of every leg

# shoulder tabs beside the trunk shell front (r2; not used since r3)
TAB_X0 = 30.2          # hips end at x = 29.0
TAB_T = 4.0
TAB_Z0 = 90.0          # tabs only at shell height (shell sides run z 106.8-148.6)
TAB_TOP = 147.0

# shelf, hooks, legs
SHELF_X = (4.5, SPINE_X0_HIGH + 0.01)  # starts clear of the toe (x <= 2.2)
SHELF_Z0 = 52.0
SHELF_TOP = 64.0       # level shelf; capped lower wherever the robot comes down
THIGH_Y = (43.0, 70.0)
THIGH_FRONT = 16.2     # thigh shell nose
CRADLE_X1 = THIGH_FRONT + C + 4.0   # the hook is 4 mm thick in front of the nose
HOOK_TOP = 97.0
LEG_T = 4.0            # the two legs under the shelf ends
MAX_BRIDGE = 14.0      # longest span under the shelf (Bambu Studio flags 24 mm as a floating cantilever)

# hip-yaw link faces (yaw2roll body, zero pose)
HIP_Y = (8.0, 27.0)      # stops span this |y| band of the servo
HIP_FRONT_LOW = 18.5     # front face, z 64.5-76
HIP_REAR_X0 = -12.0      # rear stop's back; the servo's upper rear face is at x = -8.5
HIP_INNER = 6.0          # inner faces, either side of the centre plane
HIP_OUTER = 29.0         # outer faces
HIP_TOP_MAX = 95.0       # no hip stop rises above this (capped lower by the robot)
THIGH_OUT = 70.5         # thigh outer face
THIGH_X_EAR1 = 16.0
SHELF_HALF_W = THIGH_OUT + C + 3.0   # the shelf runs out under the thigh ears
EAR_TOP = 96.0

# Hip-pitch pins (servos 12/22). Behind the pitch axis (x = 4) the servo's underside
# (z = 79, x down to -20.5) is shadowed by the foot, so nothing a robot lowers onto
# can reach it. Each pin slides in from the front through a slot in the cradle and
# hook after the robot is seated, and carries the servo out to x = -19; with the
# cradle in front of the axis, the thigh cannot pitch either way. The path below
# the servo is free of the robot from x -19.6 to 32, |y| 46.4-61.6, z 73.8-78.4.
PIN_X0 = -19.0             # rear end, 23 mm behind the pitch axis
PIN_Y = (48.0, 60.0)       # |y| band, under the servo, clear of its walls
SERVO_UNDER = 79.0         # servo underside over the whole pin
PIN_Z0 = 74.4              # slot floor = pin bottom
PIN_GAPS = (0.1, 0.2, 0.3) # one pin per gap to the servo; use the tightest that slides in
PIN_FIT = 0.3              # slot clearance each side, and above the thickest pin
PIN_STOP_L = 6.0           # stop block in front of the hook, sets the depth
PIN_STOP_Y = (46.0, 62.0)
PIN_TIP_CHAMFER = 1.0      # on the rear top edge, to lead the pin under the servo

# seat under the power_support floor
SEAT_X = (-30.0, -15.0)
SEAT_HALF_W = 26.0


def load_drop():
    d = np.load(DROP)
    return d["bottom"], float(d["x0"]), float(d["y0"]), float(d["step"])


BOTTOM, X0, Y0, ST = load_drop()


def lowest(x0, x1, y0, y1):
    """Lowest robot z over an XY rectangle, grown by the clearance."""
    i0 = int(np.floor((x0 - C - X0) / ST)); i1 = int(np.ceil((x1 + C - X0) / ST))
    j0 = int(np.floor((y0 - C - Y0) / ST)); j1 = int(np.ceil((y1 + C - Y0) / ST))
    return float(BOTTOM[max(i0, 0):i1, max(j0, 0):j1].min())


def box(x0, x1, y0, y1, z0, z1):
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN, Align.MIN, Align.MIN))


def capped_box(x0, x1, y0, y1, z0, z1):
    """A box whose top is cut down to clear whatever robot is above it."""
    top = min(z1, lowest(x0, x1, y0, y1) - C)
    if top <= z0 + 0.5:
        raise ValueError(f"feature x[{x0},{x1}] y[{y0},{y1}] has no room under the robot (top {top:.1f})")
    return box(x0, x1, y0, y1, z0, top)


def mirrored(y0, y1):
    return [(y0, y1), (-y1, -y0)]


def sole_hull(sign):
    from scipy.spatial import ConvexHull
    ix, iy = np.nonzero(np.isfinite(BOTTOM) & (BOTTOM < RIM_H + 1.0))
    xs, ys = X0 + (ix + 0.5) * ST, Y0 + (iy + 0.5) * ST
    m = ys * sign > 0
    p = np.c_[xs[m], ys[m]]
    return p[ConvexHull(p).vertices]


def offset_poly(poly, d):
    c = poly.mean(0)
    r = poly - c
    return c + r * (1 + d / np.linalg.norm(r, axis=1))[:, None]


def prism_xy(poly, z0, z1):
    w = Wire.make_polygon([Vector(x, y, z0) for x, y in poly], close=True)
    return extrude(Face(w), amount=z1 - z0)


PROFILE_STEP = 0.5     # the curved faces are sampled this finely (the drop map's own grid)
MIN_FEATURE = 3.0      # narrower peaks are shaved off the curves


def smooth_top(x0, x1, y0, y1, cap):
    """Points (x, z) of a curve that follows the robot's underside over [x0, x1].

    The robot's lowest point is taken per 0.5 mm cell; each cell boundary takes
    the lower of its two neighbours, and the boundaries are joined by straight
    segments. Inside any cell the curve is then at or below that cell's own
    limit, so the 0.6 mm clearance holds everywhere along the curve."""
    edges = np.arange(x0, x1, PROFILE_STEP)
    edges = np.append(edges, x1)
    cells = np.array([min(lowest(a, b, y0, y1) - C, cap) for a, b in zip(edges[:-1], edges[1:])])
    # Grey opening: shave any peak narrower than MIN_FEATURE (it would print as a
    # fragile fin). An opening never raises a value, so the clearance still holds.
    w = max(1, int(round(MIN_FEATURE / PROFILE_STEP)))
    cells = grey_opening(cells, size=w, mode="nearest").tolist()
    z = [cells[0]] + [min(a, b) for a, b in zip(cells[:-1], cells[1:])] + [cells[-1]]
    pts = [(float(edges[0]), z[0])]
    for x, zz in zip(edges[1:], z[1:]):
        if len(pts) >= 2:  # drop points that sit on a straight run
            (xa, za), (xb, zb) = pts[-2], pts[-1]
            if abs((zb - za) * (x - xb) - (zz - zb) * (xb - xa)) < 1e-6:
                pts[-1] = (float(x), zz)
                continue
        pts.append((float(x), zz))
    return pts


def xz_prism(top, z_floor, y0, y1):
    pts = [(top[0][0], z_floor)] + top + [(top[-1][0], z_floor)]
    w = Wire.make_polygon([Vector(x, y0, z) for x, z in pts], close=True)
    return extrude(Face(w), amount=y1 - y0, dir=(0, 1, 0))


def head_rest():
    """A flat plate the head sits on, with 45 deg gussets down to the column.

    One symmetric YZ profile, extruded along x, so both sides are built the same
    way. Its top is the lowest point of the head over the plate, minus C."""
    top = lowest(REST_X[0], REST_X[1], -REST_HALF_W, REST_HALF_W) - C
    under = top - REST_T
    foot = under - (REST_HALF_W - COL_HALF_W)          # 45 deg: drop = run
    prof = [(-COL_HALF_W, foot), (COL_HALF_W, foot), (REST_HALF_W, under), (REST_HALF_W, top),
            (-REST_HALF_W, top), (-REST_HALF_W, under)]
    w = Wire.make_polygon([Vector(REST_X[0], y, z) for y, z in prof], close=True)
    return extrude(Face(w), amount=REST_X[1] - REST_X[0], dir=(1, 0, 0)), foot


def thigh_cradle(y0, y1):
    """Thigh rest and hook in one profile, standing on the shelf: it curves along the
    thigh's sloped underside and rises in front of its nose (x = 16.2)."""
    return xz_prism(smooth_top(SHELF_X[0], CRADLE_X1, y0, y1, HOOK_TOP), SHELF_Z0, y0, y1)


def rounded_cut(x0, x1, y0, y1, z0, z1, axis, r):
    """A box to subtract, its edges along `axis` rounded (a window with round corners)."""
    b = box(x0, x1, y0, y1, z0, z1)
    spans = {"X": (y1 - y0, z1 - z0), "Y": (x1 - x0, z1 - z0), "Z": (x1 - x0, y1 - y0)}
    key = "X" if axis == Axis.X else ("Y" if axis == Axis.Y else "Z")
    r = min(r, min(spans[key]) / 2 - 0.5)  # never more than the window allows
    return fillet(b.edges().filter_by(axis), radius=r)


def features():
    """Every feature as (name, solid), before they are fused.

    Only what locates the robot, and what carries those faces, is kept:
    feet in their rims, both hip-yaw servos on the shelf, both thighs in their
    cradles and hooks, and the head on the spine top with its key. With those
    fixed, the trunk's pose follows from the joint chains, so the trunk itself
    is not touched (anything more would fight the model wherever the real robot
    differs from it by more than the clearance)."""
    parts = []

    def add(name, solid):
        parts.append((name, solid))

    # --- base: a frame under the feet, the legs and the column ----------------------
    base = box(BASE_X[0], BASE_X[1], -BASE_HALF_W, BASE_HALF_W, -BASE_T, 0)
    base = fillet(base.edges().filter_by(Axis.Z), radius=BASE_CORNER_R)
    for x0, x1, y0, y1 in BASE_WINDOWS:
        base = base - rounded_cut(x0, x1, y0, y1, -BASE_T - 1, 1.0, Axis.Z, WINDOW_R)
    add("base frame", base)
    for sign in (1, -1):
        hull = sole_hull(sign)
        outer = prism_xy(offset_poly(hull, C + RIM_W), 0, RIM_H)
        inner = prism_xy(offset_poly(hull, C), -0.1, RIM_H + 0.1)
        add("sole rim", outer - inner)

    # --- column carrying the head rest ------------------------------------------------
    rest, rest_foot = head_rest()
    col = box(COL_X[0], COL_X[1], -COL_HALF_W, COL_HALF_W, 0, rest_foot + 0.5)
    for z0, z1 in COL_WINDOWS:
        col = col - rounded_cut(COL_X[0] - 1, COL_X[1] + 1, -COL_WINDOW_HALF_W, COL_WINDOW_HALF_W, z0, z1, Axis.X, WINDOW_R)
    add("column", col)
    add("head rest (flat plate)", rest)

    # --- shelf under the hip servos, its tie to the column, thigh cradles, legs ----------
    for y0, y1 in [(-SPINE_HALF_W, SPINE_HALF_W)] + mirrored(SPINE_HALF_W - 0.01, SHELF_HALF_W):
        add("shelf", capped_box(SHELF_X[0], HIP_SHELF_X1, y0, y1, SHELF_Z0, SHELF_TOP))
    # The tie from the column to the shelf carries the shelf's middle (no centre
    # leg): its underside rises at 45 deg from the column face back to the shelf's
    # rear edge, so it prints without support.
    run = COL_X[0] + 1.0 - SHELF_X[0]
    prof = [(SHELF_X[0], SHELF_Z0), (COL_X[0] + 1.0, SHELF_Z0 - run),
            (COL_X[0] + 1.0, SHELF_TOP), (SHELF_X[0], SHELF_TOP)]
    w = Wire.make_polygon([Vector(x, -COL_HALF_W, z) for x, z in prof], close=True)
    add("shelf tie", extrude(Face(w), amount=2 * COL_HALF_W, dir=(0, 1, 0)))
    for y0, y1 in mirrored(*THIGH_Y):
        add("thigh cradle + hook", thigh_cradle(y0, y1))
    # two legs only, one under each end of the shelf
    for y0, y1 in mirrored(SHELF_HALF_W - LEG_T, SHELF_HALF_W):
        add("leg", capped_box(SHELF_X[0], HIP_SHELF_X1, y0, y1, 0.0, SHELF_Z0 + 1.0))
    # a 45 deg gusset on each leg's inner face, so the bridge under the shelf from
    # the tie (|y| = COL_HALF_W) to the gusset is at most MAX_BRIDGE
    g = SHELF_HALF_W - LEG_T - COL_HALF_W - MAX_BRIDGE
    for sgn in (1, -1):
        yl = sgn * (SHELF_HALF_W - LEG_T + 0.01)
        prof = [(yl, SHELF_Z0 - g), (yl, SHELF_Z0 + 1.0), (yl - sgn * (g + 1.0), SHELF_Z0 + 1.0)]
        w = Wire.make_polygon([Vector(SHELF_X[0], y, z) for y, z in prof], close=True)
        add("leg gusset", extrude(Face(w), amount=HIP_SHELF_X1 - SHELF_X[0], dir=(1, 0, 0)))

    # --- hip-yaw link (servo IDs 10/20 drive it; 11/21 and 12/22 sit on it) ------------
    # Holding this link in x, y and yaw (the shelf already holds z, pitch, roll) makes
    # each leg a chain fixed at both ends (here and at the sole rim), which fixes the
    # hip roll (11/21), hip pitch (12/22), knee and ankle at zero.
    for sgn in (1, -1):
        def ys(a, b):
            return (a, b) if sgn > 0 else (-b, -a)
        # front stop on the servo's lower front face (x = 18.5, z 64.5-76)
        add("hip front stop", capped_box(HIP_FRONT_LOW + C, HIP_FRONT_LOW + C + 4.0, *ys(*HIP_Y), SHELF_Z0, HIP_TOP_MAX))
        # rear stop on the servo's upper rear face (x = -8.5, z 76-83); its top follows
        # the servo's underside. It stands on the bed (between the feet, |y| < 29, the
        # robot leaves the column free down to the floor), which also carries the
        # shelf's middle at its rear edge: hung off the shelf it was a cantilever.
        y0, y1 = ys(*HIP_Y)
        top = smooth_top(HIP_REAR_X0, SHELF_X[0], y0, y1, HIP_TOP_MAX)
        pts = top + [(SHELF_X[0], -BASE_T), (HIP_REAR_X0, -BASE_T)]
        w = Wire.make_polygon([Vector(x, y0, z) for x, z in pts], close=True)
        add("hip rear stop", extrude(Face(w), amount=y1 - y0, dir=(0, 1, 0)))
        # outer side stop on the hip link's outer face (|y| = 29.0)
        add("hip side stop", capped_box(SHELF_X[0], HIP_FRONT_LOW + C + 4.0, *ys(HIP_OUTER + C, HIP_OUTER + C + 3.0), SHELF_Z0, HIP_TOP_MAX))
        # outer ear on the thigh (|y| = 70.5), as the reference's ears
        add("thigh ear", capped_box(SHELF_X[0], THIGH_X_EAR1, *ys(THIGH_OUT + C, THIGH_OUT + C + 3.0), SHELF_Z0, EAR_TOP))
    # centre block between the two hip links' inner faces (|y| = 6.0)
    add("hip centre block", capped_box(SHELF_X[0], HIP_FRONT_LOW + C + 4.0, -(HIP_INNER - C), HIP_INNER - C, SHELF_Z0, HIP_TOP_MAX))
    return parts


def pin_slots():
    """The slot each pin runs in, through the cradle and the hook (open on top where
    the cradle is lower than the slot)."""
    return [box(SHELF_X[0] - 1.0, CRADLE_X1 + 1.0, y0 - PIN_FIT, y1 + PIN_FIT,
                PIN_Z0, SERVO_UNDER + PIN_FIT) for y0, y1 in mirrored(*PIN_Y)]


def pin(sign=1, gap=PIN_GAPS[0]):
    """One pin in place (sign +1 left, -1 right), its top `gap` under the servo: a bar
    from x = PIN_X0 (behind the pitch axis) to the servo's front end, and a stop block
    that meets the hook's front face. Notches on the stop's front face count its gap
    (one per 0.1 mm)."""
    (y0, y1), (s0, s1) = [mirrored(*b)[0 if sign > 0 else 1] for b in (PIN_Y, PIN_STOP_Y)]
    top = SERVO_UNDER - gap
    prof = [(PIN_X0, PIN_Z0), (CRADLE_X1 + 0.5, PIN_Z0), (CRADLE_X1 + 0.5, top),
            (PIN_X0 + PIN_TIP_CHAMFER, top), (PIN_X0, top - PIN_TIP_CHAMFER)]
    w = Wire.make_polygon([Vector(x, y0, z) for x, z in prof], close=True)
    bar = extrude(Face(w), amount=y1 - y0, dir=(0, 1, 0))
    stop = box(CRADLE_X1, CRADLE_X1 + PIN_STOP_L, s0, s1, PIN_Z0, top)
    p = bar.fuse(stop)
    n = int(round(gap / 0.1))
    xf = CRADLE_X1 + PIN_STOP_L
    for i in range(n):
        yc = (s0 + s1) / 2 + (i - (n - 1) / 2) * 3.0
        p = p.cut(box(xf - 0.8, xf + 0.1, yc - 0.6, yc + 0.6, PIN_Z0 - 0.1, top + 0.1))
    return p.clean()


def gen_step():
    parts = [p for _, p in features()]
    body = parts[0].fuse(*parts[1:])
    body = body.cut(*pin_slots())

    body = body.clean()
    body.label = "fanduck_zero_stand_r3"
    return body


if __name__ == "__main__":
    s = gen_step()
    bb = s.bounding_box()
    print("volume cm3", round(s.volume / 1000, 1), "bbox", bb.min, bb.max, "solids", len(s.solids()))
