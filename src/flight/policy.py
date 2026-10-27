"""
Runs the PPO-trained attitude policy on the board: a tiny tanh MLP in plain
Python, no numpy, no ulab.

The policy replaces only the levelling PID. It reads what the board already
has (fused roll/pitch, gyro, throttle) and returns roll/pitch/yaw corrections
in the same units the rate PIDs produce, so they go straight into
flight.mixer.mix() - desaturation, the IDLE floor and the MAX_DUTY ceiling all
still apply exactly as they do for the PID.

The observation layout below is the contract with the simulator
(sim/pico_drone_sim/env.py builds the identical vector). Change one, change
both, retrain. tests/test_policy.py checks the two agree.

  index  value                                   scale (divide by)
  0      roll error  (level - roll), deg         OBS_ANGLE_DEG
  1      pitch error (level - pitch), deg        OBS_ANGLE_DEG
  2-4    gyro x, y, z, deg/s                     OBS_RATE_DPS
  5-6    integral of roll / pitch error, deg*s   OBS_INTEGRAL
  7      collective throttle, 0..1               1
  8-10   previous action (roll, pitch, yaw)      1

Why the integral is in there: a plain MLP has no memory, so it cannot learn
"I have been leaning left for two seconds, push harder". Without it, a CoM that
sits a few mm off-centre (wires, a GPS mast) leaves a permanent lean. With it,
the policy learns the same job the PID's I term does.
"""

import math

try:
    import micropython

    _native = micropython.native
except ImportError:  # CPython: tests and the simulator

    def _native(f):
        return f


OBS_SIZE = 11
ACT_SIZE = 3

OBS_ANGLE_DEG = 30.0
OBS_RATE_DPS = 300.0
OBS_INTEGRAL = 10.0
INTEGRAL_LIMIT = 20.0   # deg*s, clamp on obs 5-6 before scaling

# Policy output is tanh-squashed to -1..1 per axis, then scaled to this many
# units of mixer correction. 0.35 is a little over what the rate PID asks for
# at a 100 deg/s error with lift.py's gains, so it is the same authority.
ACT_SCALE = 0.35


def build_obs(roll_err, pitch_err, gyro, i_roll, i_pitch, throttle, last_action):
    """The observation vector, scaled. Shared by the board and the simulator."""
    return [
        roll_err / OBS_ANGLE_DEG,
        pitch_err / OBS_ANGLE_DEG,
        gyro[0] / OBS_RATE_DPS,
        gyro[1] / OBS_RATE_DPS,
        gyro[2] / OBS_RATE_DPS,
        i_roll / OBS_INTEGRAL,
        i_pitch / OBS_INTEGRAL,
        throttle,
        last_action[0],
        last_action[1],
        last_action[2],
    ]


class Policy:
    """
    layers: list of (weights, biases). weights[j] is the row of input weights
    for output j, so a layer maps len(weights[0]) inputs to len(weights)
    outputs. Every layer but the last is followed by tanh; the last is followed
    by tanh too (squashes to -1..1), then multiplied by ACT_SCALE.
    """

    def __init__(self, layers):
        self.layers = [([list(row) for row in w], list(b)) for w, b in layers]
        # Preallocated buffers, so a call allocates nothing it can avoid -
        # fewer GC pauses inside the control loop.
        self._bufs = [[0.0] * len(b) for _, b in self.layers]
        self.in_size = len(self.layers[0][0][0])
        self.out_size = len(self.layers[-1][1])

    @_native
    def act(self, obs):
        """obs: list of OBS_SIZE floats (from build_obs). Returns [roll, pitch, yaw]."""
        x = obs
        for k in range(len(self.layers)):
            w, b = self.layers[k]
            out = self._bufs[k]
            n_in = len(x)
            for j in range(len(b)):
                row = w[j]
                s = b[j]
                for i in range(n_in):
                    s += row[i] * x[i]
                out[j] = math.tanh(s)
            x = out
        return [x[0] * ACT_SCALE, x[1] * ACT_SCALE, x[2] * ACT_SCALE]

    @property
    def macs(self):
        """Multiply-adds per call - the number that sets the loop-time cost."""
        return sum(len(w) * len(w[0]) for w, _ in self.layers)


def load():
    """The trained policy from flight/policy_weights.py (written by sim/export_policy.py)."""
    from flight import policy_weights

    return Policy(policy_weights.LAYERS)
