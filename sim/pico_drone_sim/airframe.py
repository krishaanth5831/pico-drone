"""
The real airframe for the simulator: the printed frame and mast STLs from
hardware/frame/, plus every component as the same fit-check dummy the Onshape
model uses, at the same place and with the same mass.

This is a straight port of the "FIT-CHECK DUMMIES" block and the layout maths
in hardware/frame/pico_drone_frame.fs, at that file's default dimensions. If
you change the frame in Onshape, change the matching constant here; MuJoCo
then recomputes mass, centre of mass and inertia from the parts.

Coordinates are flight orientation in millimetres: +x nose, +y left, +z up,
deck top at z = 0 (boards sit on it, the battery hangs below it).

The STLs in the repo are in PRINT orientation (deck / platform face on the
bed), i.e. rotated 180 degrees about X from flight: (x, y, z) -> (x, -y, -z).
Checked from the meshes themselves: the battery bay's front stop wall is at +x
and the mast legs (flight y -12..+2 mm) sit at y -2..+12 in the STL.
"""

from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FRAME_STL = REPO / "hardware" / "frame" / "pico_drone_frame.stl"
MAST_STL = REPO / "hardware" / "frame" / "pico_drone_gps_mast.stl"

# Masses from hardware/frame/README.md (PLA at 1.24 g/cm^3, from Onshape).
FRAME_MASS_G = 8.7
MAST_MASS_G = 2.7

# --- pico_drone_frame.fs defaults (mm) ------------------------------------------
M_D, M_FIT, M_LEN, M_RISE = 7.0, 0.1, 20.0, 11.0     # 720 motor: dia, bore fit, length, rise
PROP_D = 55.0
PITCH = 84.0                                         # motor spacing, adjacent centres
CLR, HDR, FOAM = 0.3, 8.5, 1.0
IMU_L, IMU_W = 21.2, 16.4                            # GY-521, X arrow along the long edge
DRV_L, DRV_W = 18.5, 16.0
MAG_L, MAG_W = 14.8, 13.5
GPS_L, GPS_W, ANT_L, ANT_H = 36.0, 25.0, 25.0, 6.2
BAT_L, BAT_W, BAT_T, BAT_CLR = 42.0, 20.0, 8.5, 0.4
BAY = 10.0
WALL_T = 1.0
MAST_TOP = 17.0

PICO_L, PICO_W, PICO_ROW_DY, PICO_STRIP, USB_OVER = 51.0, 21.0, 8.89, 50.8, 1.3

# --- derived layout, same expressions as the .fs --------------------------------
A = PITCH / 2.0                                      # motor centres at (+-A, +-A)
IMU_X, IMU_Y = IMU_L, IMU_W
_pk_w = PICO_W + 2 * CLR
_rc_w = max(DRV_L, IMU_Y) + 2 * CLR
_tot_w = _pk_w + 2.0 + _rc_w
_y_min = -_tot_w / 2
RC_Y = _y_min + _rc_w / 2                            # right column (DRVs, IMU), -y
PICO_Y = _y_min + _rc_w + 2.0 + _pk_w / 2            # Pico column, +y
_drv_px = DRV_W + 2 * CLR
_imu_px = IMU_X + 2 * CLR
_rc_len = 2 * _drv_px + _imu_px + 2 * 1.6
DRV1_X = _rc_len / 2 - _drv_px / 2                   # front driver
DRV2_X = -DRV1_X                                     # rear driver
DRV_ROW_DY = DRV_L / 2 - 1.63
Z_MB = M_RISE - M_LEN                                # motor bottom
Z_POD = Z_MB - 1.6                                   # pod bottom
Z_BB = -BAY - (BAT_T + BAT_CLR)                      # battery bottom
Z_FLOOR = Z_BB - WALL_T                              # bay floor underside (lowest point)
_mag_x = MAG_L + 2 * CLR
_gps_x = GPS_L + 2 * CLR
_stack = _mag_x + 3 + _gps_x
MAG_CX = -_stack / 2 + _mag_x / 2
GPS_CX = _stack / 2 - _gps_x / 2

# Motor number -> (x sign, y sign, spin), from the .fs `arms` and flight/mixer.py.
ARMS = {1: (+1, -1, "CCW"), 2: (-1, +1, "CCW"), 3: (+1, +1, "CW"), 4: (-1, -1, "CW")}

# Where the MPU6050 chip sits (centre of the GY-521, on its foam tape).
IMU_POS = (0.0, RC_Y, FOAM + 0.05 + 1.6)


def parts(gps=True, mag=False):
    """
    [(name, mass_g, rgba, shapes)], shapes being ("B", lo, hi) boxes or
    ("C", x, y, z0, z1, r) cylinders, exactly as the .fs dummies.
    """
    e = 0.05
    out = []

    sp = [("B", (-PICO_L / 2, PICO_Y - PICO_W / 2, e), (PICO_L / 2, PICO_Y + PICO_W / 2, e + 1)),
          ("B", (-PICO_L / 2 - USB_OVER, PICO_Y - 4, e + 1), (-PICO_L / 2 + 4.2, PICO_Y + 4, e + 3.7)),
          ("B", (-4, PICO_Y - 4, e + 1), (4, PICO_Y + 4, e + 1.9))]
    for sy in (-1, 1):
        yr = PICO_Y + sy * PICO_ROW_DY
        sp.append(("B", (-PICO_STRIP / 2, yr - 1.27, e - 2.5), (PICO_STRIP / 2, yr + 1.27, e)))
        sp.append(("B", (-PICO_STRIP / 2 + 1, yr - 0.32, e - HDR), (PICO_STRIP / 2 - 1, yr + 0.32, e - 2.5)))
    out.append(("pico", 5.5, "0.1 0.55 0.2 1", sp))

    for k, dx in enumerate((DRV1_X, DRV2_X)):
        sd = [("B", (dx - DRV_W / 2, RC_Y - DRV_L / 2, e), (dx + DRV_W / 2, RC_Y + DRV_L / 2, e + 1.6)),
              ("B", (dx - 3, RC_Y - 3, e + 1.6), (dx + 3, RC_Y + 3, e + 2.8))]
        for sy in (-1, 1):
            yr = RC_Y + sy * DRV_ROW_DY
            sd.append(("B", (dx - 7.62, yr - 1.27, e - 2.5), (dx + 7.62, yr + 1.27, e)))
            sd.append(("B", (dx - 6.6, yr - 0.32, e - HDR), (dx + 6.6, yr + 0.32, e - 2.5)))
        out.append(("drv%d" % (k + 1), 1.5, "0.8 0.1 0.1 1", sd))

    z0i = FOAM + e
    hy = RC_Y - (IMU_Y / 2 - 1.27)
    si = [("B", (-IMU_X / 2, RC_Y - IMU_Y / 2, z0i), (IMU_X / 2, RC_Y + IMU_Y / 2, z0i + 1.6)),
          ("B", (-2, RC_Y - 2, z0i + 1.6), (2, RC_Y + 2, z0i + 2.5)),
          ("B", (-10.16, hy - 1.27, z0i - 2.5), (10.16, hy + 1.27, z0i)),
          ("B", (-9.2, hy - 0.32, z0i - HDR), (9.2, hy + 0.32, z0i - 2.5))]
    out.append(("imu", 2.0, "0.15 0.3 0.85 1", si))

    out.append(("battery", 12.5, "0.75 0.75 0.78 1",
                [("B", (-BAT_L / 2, -BAT_W / 2, Z_BB + BAT_CLR / 2),
                  (BAT_L / 2, BAT_W / 2, Z_BB + BAT_CLR / 2 + BAT_T))]))

    for n, (sx, sy, _) in ARMS.items():
        mx, my = A * sx, A * sy
        out.append(("motor%d" % n, 3.4, "0.6 0.6 0.62 1",
                    [("C", mx, my, Z_MB + e, M_RISE, M_D / 2), ("C", mx, my, M_RISE, M_RISE + 5, 0.5)]))
        colour = "0.85 0.35 0.25 0.45" if ARMS[n][2] == "CCW" else "0.25 0.5 0.9 0.45"
        out.append(("prop%d" % n, 0.6, colour,
                    [("C", mx, my, M_RISE + 0.6, M_RISE + 5.6, 2.6),
                     ("C", mx, my, M_RISE + 1.5, M_RISE + 4.5, PROP_D / 2)]))

    if gps:
        z0g = MAST_TOP + FOAM + e
        hxg = GPS_CX + (GPS_L / 2 - 1.27)
        out.append(("gps", 14.0, "0.2 0.2 0.2 1",
                    [("B", (GPS_CX - GPS_L / 2, -GPS_W / 2, z0g), (GPS_CX + GPS_L / 2, GPS_W / 2, z0g + 1.6)),
                     ("B", (GPS_CX - 8, -6.1, z0g + 1.6), (GPS_CX + 8, 6.1, z0g + 4.0)),
                     ("B", (GPS_CX - ANT_L / 2, -ANT_L / 2, z0g + 4.0), (GPS_CX + ANT_L / 2, ANT_L / 2, z0g + 5.0 + ANT_H)),
                     ("B", (hxg - 1.27, -5.08, z0g - 2.5), (hxg + 1.27, 5.08, z0g)),
                     ("B", (hxg - 0.32, -4.2, z0g - HDR), (hxg + 0.32, 4.2, z0g - 2.5))]))
    if gps and mag:
        z0g = MAST_TOP + FOAM + e
        hxm = MAG_CX - (MAG_L / 2 - 1.27)
        out.append(("mag", 1.3, "0.55 0.2 0.7 1",
                    [("B", (MAG_CX - MAG_L / 2, -MAG_W / 2, z0g), (MAG_CX + MAG_L / 2, MAG_W / 2, z0g + 1.6)),
                     ("B", (MAG_CX - 1.5, -1.5, z0g + 1.6), (MAG_CX + 1.5, 1.5, z0g + 2.5)),
                     ("B", (hxm - 1.27, -6.35, z0g - 2.5), (hxm + 1.27, 6.35, z0g)),
                     ("B", (hxm - 0.32, -5.5, z0g - HDR), (hxm + 0.32, 5.5, z0g - 2.5))]))
    return out


def cad_mass_g(gps=True, mag=False):
    """All-up mass without wires, as the Onshape summary reports it."""
    return FRAME_MASS_G + (MAST_MASS_G if gps else 0.0) + sum(m for _, m, _, _ in parts(gps, mag))


def _shape_volume(s):
    if s[0] == "B":
        lo, hi = s[1], s[2]
        return abs((hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2]))
    _, _, _, z0, z1, r = s
    return 3.141592653589793 * r * r * abs(z1 - z0)


def _geom(name, s, mass_g, rgba, collide=False):
    """One MJCF geom for a dummy shape, in metres, carrying its share of the mass."""
    con = "" if collide else ' contype="0" conaffinity="0"'
    colour = rgba
    if s[0] == "B":
        lo, hi = s[1], s[2]
        c = [(lo[i] + hi[i]) / 2000.0 for i in range(3)]
        h = [abs(hi[i] - lo[i]) / 2000.0 for i in range(3)]
        return ('<geom name="%s" type="box" pos="%g %g %g" size="%g %g %g" mass="%g" rgba="%s"%s/>'
                % (name, *c, *h, mass_g / 1000.0, colour, con))
    _, x, y, z0, z1, r = s
    return ('<geom name="%s" type="cylinder" pos="%g %g %g" size="%g %g" mass="%g" rgba="%s"%s/>'
            % (name, x / 1000.0, y / 1000.0, (z0 + z1) / 2000.0, r / 1000.0, abs(z1 - z0) / 2000.0,
               mass_g / 1000.0, colour, con))


def mjcf_assets(gps=True):
    s = '<mesh name="frame" file="%s" scale="0.001 0.001 0.001"/>' % FRAME_STL
    if gps:
        s += '\n    <mesh name="mast" file="%s" scale="0.001 0.001 0.001"/>' % MAST_STL
    return s


def mjcf_body(gps=True, mag=False, extra_mass_g=0.0):
    """
    Geoms for the drone body. Every geom carries mass, so MuJoCo works out
    the total mass, CoM and full inertia tensor from the real layout.

    `extra_mass_g` is wiring, tape and zip ties: spread as a thin slab in the
    wiring bay under the deck (between the header pins), centred.
    """
    g = []
    # Frame and mast meshes, rotated from print to flight orientation.
    g.append('<geom name="frame" type="mesh" mesh="frame" quat="0 1 0 0" mass="%g" '
             'rgba="0.93 0.93 0.9 1" contype="0" conaffinity="0"/>' % (FRAME_MASS_G / 1000.0))
    if gps:
        g.append('<geom name="mast" type="mesh" mesh="mast" pos="0 0 %g" quat="0 1 0 0" mass="%g" '
                 'rgba="0.93 0.93 0.9 1" contype="0" conaffinity="0"/>'
                 % (MAST_TOP / 1000.0, MAST_MASS_G / 1000.0))
    for name, mass_g, rgba, shapes in parts(gps, mag):
        vols = [_shape_volume(s) for s in shapes]
        total = sum(vols)
        for i, s in enumerate(shapes):
            g.append(_geom("%s_%d" % (name, i), s, mass_g * vols[i] / total, rgba))
    if extra_mass_g > 0:
        g.append(_geom("wires", ("B", (-20, -18, -9), (20, 18, -3)), extra_mass_g, "0.9 0.6 0.1 0.25"))

    # Contacts: the bay floor (lowest point) and the four motor-pod bottoms.
    g.append('<geom name="bay_floor" type="box" pos="0 0 %g" size="0.0222 0.0112 0.0005" mass="0" '
             'rgba="0 0 0 0"/>' % ((Z_FLOOR + 0.5) / 1000.0))
    for n, (sx, sy, _) in ARMS.items():
        g.append('<geom name="pod_foot%d" type="sphere" pos="%g %g %g" size="0.003" mass="0" rgba="0 0 0 0"/>'
                 % (n, A * sx / 1000.0, A * sy / 1000.0, (Z_POD + 3.0) / 1000.0))
    return g
