"""
Small angle-arithmetic helpers shared by anything doing heading or angle control.

Degrees, not radians, throughout - the rest of the flight code (ComplementaryFilter's
*_deg properties, HMC5883L.heading()) already works in degrees, and converting back
and forth at every call site is a needless way to introduce a sign error.
"""


def wrap_deg_error(target_deg, current_deg):
    """
    Shortest signed angular distance from current_deg to target_deg, in (-180, 180].

    Compass headings wrap at 0/360, so a plain subtraction breaks exactly where a
    heading-hold controller needs it most: pointed at 359 degrees with a 1 degree
    target, naive subtraction says "turn -358 degrees" instead of "turn +2".
    """
    error = (target_deg - current_deg) % 360.0
    if error > 180.0:
        error -= 360.0
    return error
