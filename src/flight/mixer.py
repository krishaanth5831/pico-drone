"""
Motor mixer: turns a desired (throttle, roll, pitch, yaw) command into four
individual motor throttles.

Airframe is a standard X quad seen from above, nose up the page. Diagonal pairs
share a rotation direction so their yaw torques cancel in the hover:

        M3 (CW)          M1 (CCW)
          \\                /
           \\   +--------+ /
            +--|  PICO  |-+
               |  IMU   |
            +--|        |-+
           /   +--------+ \\
          /                \\
        M2 (CCW)          M4 (CW)

  M1 front-right  CCW      M3 front-left   CW
  M2 rear-left    CCW      M4 rear-right   CW

Roll right  -> left motors up,  right motors down
Pitch fwd   -> rear motors up,  front motors down
Yaw right   -> CCW pair up,     CW pair down
"""

# (roll, pitch, yaw) sign for each motor. Yaw sign is the OPPOSITE of the motor's
# own rotation: to yaw right you speed up the motors spinning left.
_MIX = {
    1: (-1.0, -1.0, +1.0),  # front-right, CCW
    2: (+1.0, +1.0, +1.0),  # rear-left,   CCW
    3: (+1.0, -1.0, -1.0),  # front-left,  CW
    4: (-1.0, +1.0, -1.0),  # rear-right,  CW
}


def mix(throttle, roll, pitch, yaw, idle=0.0):
    """
    throttle: 0.0 to 1.0 collective
    roll, pitch, yaw: correction terms, nominally -1.0 to 1.0

    Returns {motor: throttle} with every value clamped to 0.0-1.0.

    `idle` holds motors spinning at a floor while armed. Coreless motors take
    tens of milliseconds to spool from a dead stop, and that lag inside a control
    loop reads as sluggish, asymmetric response.
    """
    raw = {}
    for motor, (kr, kp, ky) in _MIX.items():
        raw[motor] = throttle + kr * roll + kp * pitch + ky * yaw

    # Desaturate rather than clip. If any motor is asked for more than 1.0,
    # subtract the excess from all four equally: that preserves the *differences*
    # between motors, which is what actually controls attitude. Clipping one
    # motor silently distorts the commanded torque and the aircraft rolls off.
    highest = max(raw.values())
    if highest > 1.0:
        excess = highest - 1.0
        for motor in raw:
            raw[motor] -= excess

    floor = idle if throttle > 0.0 else 0.0
    return {
        motor: (floor if value < floor else (1.0 if value > 1.0 else value))
        for motor, value in raw.items()
    }


MOTOR_DIRECTIONS = {1: "CCW", 2: "CCW", 3: "CW", 4: "CW"}
MOTOR_POSITIONS = {
    1: "front-right",
    2: "rear-left",
    3: "front-left",
    4: "rear-right",
}
