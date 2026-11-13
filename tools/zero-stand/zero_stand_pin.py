"""Hip-pitch pins for the zero stand: see PIN_* in zero_stand.py.

One pin per gap in PIN_GAPS, side by side, lying flat as printed (z = 0 down).
Print two sets. Seat the robot, then push a pin into each cradle's slot from the
front until its stop block meets the hook; use the tightest pin that slides in
(the notches on the stop count its gap to the servo, one per 0.1 mm).
"""
from build123d import Compound, Pos

import zero_stand as zs


def gen_step():
    yc = (zs.PIN_STOP_Y[0] + zs.PIN_STOP_Y[1]) / 2
    pitch = zs.PIN_STOP_Y[1] - zs.PIN_STOP_Y[0] + 4.0
    pins = [Pos(0, -yc + i * pitch, -zs.PIN_Z0) * zs.pin(1, g) for i, g in enumerate(zs.PIN_GAPS)]
    c = Compound(children=pins)
    c.label = "fanduck_zero_stand_pins"
    return c


if __name__ == "__main__":
    s = gen_step()
    bb = s.bounding_box()
    print("volume cm3", round(s.volume / 1000, 2), "bbox", bb.min, bb.max)
