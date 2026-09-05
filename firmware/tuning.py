"""
Every hand-tunable number for the attitude-hold flight controller, in one file.

This board has never flown. Every gain below is a conservative generic starting
point for a cascaded-PID multirotor, not a value validated on this airframe -
there is no way to know good gains without your specific frame, motors, and prop
combination, which is exactly why manual tuning was chosen over an automated
routine for this first build (no kill switch exists yet to make an unattended
autotune safe).

Tune in this order, changing ONE number at a time, retesting between each change:

  1. Everything starts at LIVE_MOTORS = False. Watch the printed mixer output
     respond as you tilt the board by hand - props off, nothing armed, nothing
     can spin. Confirm the sign of every axis is correct before going further:
     tilt right, ROLL output should push the right motors DOWN and left motors UP.
     Get this wrong and the first live attempt will flip immediately.

  2. Props on, propped in a restraining rig so it cannot leave the ground even
     at full deflection. Set LIVE_MOTORS = True, RUN_SECONDS short (5-10s).

  3. Rate loop first, angle loop still disabled (set the ANGLE_* gains to 0 so
     the angle loop commands zero rate regardless of tilt). Raise *_RATE_KP from
     zero until the airframe visibly oscillates at a fixed frequency when
     disturbed by hand, then back off to roughly 50% of that value. Add a little
     *_RATE_KD to damp the remaining wobble. *_RATE_KI stays near zero unless you
     see a persistent one-sided lean that P+D alone won't correct.

  4. Re-enable the angle loop with a small *_ANGLE_KP and raise it until the
     board holds level under hand disturbance without overshooting back past
     level. This is far less sensitive than the rate loop; small changes matter
     less here.

  5. Only once roll and pitch are solid, do the same for yaw using the compass.

If it ever oscillates violently, diverges, or does anything that looks like it
could flip - LIVE_MOTORS = False immediately and pull the battery. That is not
a tuning problem to push through; it is the sign to lower gains and restart the
restraint rig setup.
"""

# --- master switch -----------------------------------------------------------
# False: read sensors, run the full control loop, print what the motors WOULD
#        do - but MotorBank is never armed, so SLP stays low and nothing can
#        spin no matter what a bug in this file does.
# True:  motors actually spin. Only flip this after step 1 above is done and
#        the airframe is restrained.
LIVE_MOTORS = False

# --- throttle ------------------------------------------------------------
# There is no altitude sensor on this airframe (see firmware/README.md), so
# there is no such thing as a commanded "hover throttle" the code can compute -
# only one you find empirically in the restraining rig and hardcode here.
HOVER_THROTTLE = 0.0          # 0.0 until you have actually found this value
THROTTLE_RAMP_SECONDS = 3.0   # slow ramp - see docs/power.md on inrush/brownout
IDLE_THROTTLE = 0.15          # mixer floor while armed - keeps motors spun up,
                               # avoids spool-up lag inside the control loop

# --- safety limits, apply even in dry-run so you can test them risk-free -----
RUN_SECONDS = 10.0     # auto-disarm after this long, no matter what
TILT_LIMIT_DEG = 40.0  # auto-disarm if roll or pitch exceeds this

# --- rate loop: gyro deg/s error -> normalized motor correction (-1..1) ------
# Starting point is deliberately near-zero. Follow the procedure above.
ROLL_RATE_KP = 0.010
ROLL_RATE_KI = 0.0
ROLL_RATE_KD = 0.0003

PITCH_RATE_KP = 0.010
PITCH_RATE_KI = 0.0
PITCH_RATE_KD = 0.0003

YAW_RATE_KP = 0.012
YAW_RATE_KI = 0.0
YAW_RATE_KD = 0.0

RATE_INTEGRAL_LIMIT = 50.0   # deg/s worth of accumulated error, before scaling

# --- angle loop: degrees of tilt error -> desired rate (deg/s) --------------
MAX_ANGLE_RATE_DPS = 200.0   # clamp on what the angle loop may ask the rate loop for

ROLL_ANGLE_KP = 0.0   # start at 0 - see tuning order above
ROLL_ANGLE_KI = 0.0
ROLL_ANGLE_KD = 0.0

PITCH_ANGLE_KP = 0.0
PITCH_ANGLE_KI = 0.0
PITCH_ANGLE_KD = 0.0

# --- heading hold: degrees of heading error -> desired yaw rate (deg/s) -----
MAX_YAW_RATE_DPS = 90.0
HEADING_KP = 0.0   # start at 0 - tune only after roll/pitch are solid

# --- magnetometer calibration ------------------------------------------------
# Paste the offset/scale printed by testing/05_hmc5883l_compass/ once you have
# run that calibration WITH THE AIRFRAME FULLY ASSEMBLED. Left at the identity
# (no correction) means heading will be distorted by whatever is nearby.
MAG_OFFSET = (0.0, 0.0, 0.0)
MAG_SCALE = (1.0, 1.0, 1.0)

# --- sensor fusion ------------------------------------------------------------
COMPLEMENTARY_ALPHA = 0.98   # see flight/fusion.py for what this trades off
