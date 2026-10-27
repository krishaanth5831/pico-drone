"""
Gymnasium environment: hold the drone level, the way firmware/rl_lift.py flies.

Each control step mirrors one pass of the board's loop:
  read IMU (noisy, filtered, possibly misaligned) -> complementary filter
  (flight/fusion.py, the board's own code) -> observation (flight/policy.py's
  build_obs) -> action -> flight/mixer.py mix() -> motors (lag, curve, sag).

The reward uses the TRUE attitude, which the board never has - that is fine,
the reward only exists during training.

Throttle is open loop, like on the board (no altitude sensor): either a
lift.py-style ramp from the floor, or already airborne near hover. The weight
relief (balloon) is randomized so "hover throttle" lands anywhere in
drone_params.json's hover_throttle range.
"""

import math

import gymnasium as gym
import numpy as np

from flight.fusion import ComplementaryFilter
from flight.mixer import mix
from flight.policy import ACT_SCALE, ACT_SIZE, INTEGRAL_LIMIT, OBS_SIZE, build_obs

from . import params as P
from .drone import Drone

TILT_LIMIT_DEG = 35.0   # same as lift.py: the board disarms past this


class HoverEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, nominal=None, randomize_scale=1.0, episode_s=6.0, ground_start_prob=0.35,
                 kick_rate_hz=0.25, seed=None):
        self.nominal = nominal if nominal is not None else P.load()
        self.randomize_scale = randomize_scale
        self.episode_s = episode_s
        self.ground_start_prob = ground_start_prob
        self.kick_rate_hz = kick_rate_hz
        self.drone = Drone(self.nominal)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (OBS_SIZE,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (ACT_SIZE,), np.float32)
        self.rng = np.random.default_rng(seed)
        self.trim = (0.0, 0.0)   # roll/pitch trim added to commands, like lift.py's TRIM_*

    # ------------------------------------------------------------------------------
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        options = options or {}
        rng = self.rng
        p = options.get("params") or P.randomize(self.nominal, rng, self.randomize_scale)
        self.p = p
        self.drone.set_params(p)

        ground = options.get("ground", rng.random() < self.ground_start_prob)
        self.ground = ground
        if ground:
            # lift.py: spin up to IDLE, ramp to LIFT_THROTTLE, hold.
            self.ramp_s = float(rng.uniform(1.5, 3.0))
            self.throttle_cmd = p.hover_throttle + float(rng.uniform(0.01, 0.06))
            self.drone.reset(throttles=(p.idle,) * 4)
        else:
            self.ramp_s = 0.0
            self.throttle_cmd = p.hover_throttle + float(rng.uniform(-0.03, 0.03))
            tilt = options.get("tilt_deg", 20.0)
            self.drone.reset(
                roll_deg=float(rng.uniform(-tilt, tilt)), pitch_deg=float(rng.uniform(-tilt, tilt)),
                yaw_deg=float(rng.uniform(-180, 180)), z=1.0,
                rates_dps=rng.uniform(-80, 80, 3) * options.get("rate_scale", 1.0),
                throttles=(p.hover_throttle,) * 4)
        # Slow throttle wander: a battery sagging, a balloon drifting.
        self.wander_amp = float(rng.uniform(0.0, 0.03))
        self.wander_hz = float(rng.uniform(0.05, 0.3))
        self.wander_phase = float(rng.uniform(0, 2 * math.pi))

        # "level" the way lift.py's LEVEL_AT_START finds it: whatever the filter
        # reads with the drone flat. With a misaligned IMU that is not zero.
        self.level = self._flat_reading()
        self.fusion = ComplementaryFilter(alpha=0.98)
        accel, gyro = self.drone.read_imu(rng)
        self.fusion.update(accel, gyro, 0.002)
        if not ground:
            # Mid-air start: pretend the filter has been tracking since take-off,
            # i.e. it reads the true attitude through the IMU mount, +-1 deg.
            # Seeding it from one accelerometer read instead (as on the floor)
            # is meaningless in flight - thrust, not gravity, dominates it - and
            # put the estimate up to 70 deg off, so the first second of every
            # mid-air episode was spent chasing a phantom tilt.
            roll, pitch, _ = self.drone.attitude_deg()
            self.fusion.roll = math.radians(self.level[0] + roll + float(rng.normal(0, 1.0)))
            self.fusion.pitch = math.radians(self.level[1] + pitch + float(rng.normal(0, 1.0)))

        self.t = 0.0
        self.integral = [0.0, 0.0]
        self.last_action = np.zeros(ACT_SIZE)
        self.gyro = gyro
        self.est = (self.fusion.roll_deg, self.fusion.pitch_deg)
        self.out = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
        self.period = 1.0 / p.control_hz   # length of the last loop pass, like lift.py's dt
        return self._obs(), {}

    def _flat_reading(self):
        """Noise-free filter estimate of a flat drone: gravity through the mount rotation."""
        ax, ay, az = self.drone._mount @ np.array([0.0, 0.0, P.G])
        roll = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        return (roll, pitch)

    def throttle_at(self, t):
        base = self.throttle_cmd * t / self.ramp_s if t < self.ramp_s else self.throttle_cmd
        return base + self.wander_amp * math.sin(2 * math.pi * self.wander_hz * t + self.wander_phase)

    @property
    def integrating(self):
        """Integrators run once the ramp is done - rl_lift.py does the same."""
        return self.t >= self.ramp_s

    def _obs(self):
        roll_err = self.level[0] - self.est[0]
        pitch_err = self.level[1] - self.est[1]
        return np.array(build_obs(roll_err, pitch_err, self.gyro, self.integral[0], self.integral[1],
                                  self.throttle_at(self.t), self.last_action), dtype=np.float32)

    # ------------------------------------------------------------------------------
    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        cmd = a * ACT_SCALE
        obs, reward, term, trunc, info = self.step_cmd(cmd)
        # Smoothness and effort, in action units (penalties the PID path skips).
        da = a - self.last_action
        reward -= 0.25 * float(da @ da) + 0.02 * float(a @ a)
        self.last_action = a
        return self._obs(), reward, term, trunc, info

    def step_cmd(self, cmd):
        """Apply roll/pitch/yaw corrections in mixer units (what a PID or the policy outputs)."""
        p, rng = self.p, self.rng
        period = 1.0 / p.control_hz * float(rng.uniform(0.9, 1.1))
        throttle = self.throttle_at(self.t)
        self.out = mix(throttle, cmd[0] + self.trim[0], cmd[1] + self.trim[1], cmd[2], idle=p.idle)
        self.drone.step(self.out, period)
        self.t += period
        self.period = period

        if rng.random() < self.kick_rate_hz * period:
            self.drone.kick(rng.uniform(-150, 150, 3) * np.array([1.0, 1.0, 0.5]))

        accel, gyro = self.drone.read_imu(rng)
        self.fusion.update(accel, gyro, period)
        self.gyro = gyro
        self.est = (self.fusion.roll_deg, self.fusion.pitch_deg)
        if self.integrating:
            for i in range(2):
                v = self.integral[i] + (self.level[i] - self.est[i]) * period
                self.integral[i] = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, v))

        roll, pitch, _ = self.drone.attitude_deg()
        w = np.radians(self.drone.rates_dps())
        tilt = math.radians(roll) ** 2 + math.radians(pitch) ** 2
        reward = 1.0 - 3.0 * tilt - 0.02 * (w[0] ** 2 + w[1] ** 2) - 0.01 * w[2] ** 2
        term = abs(roll) > TILT_LIMIT_DEG or abs(pitch) > TILT_LIMIT_DEG
        if term:
            reward -= 20.0
        trunc = self.t >= self.episode_s
        info = {"roll": roll, "pitch": pitch, "throttle": throttle}
        return self._obs(), float(reward), term, trunc, info
