"""
The pico-drone as a MuJoCo rigid body, plus everything MuJoCo doesn't model:
coreless motor lag and thrust curve, supply sag, the weight-relief force, and
an MPU6050 with noise, bias, vibration, its DLPF and mounting error.

Frames (they match the board, which is what makes the trained policy portable):

  Body / IMU frame is FLU: +x nose, +y left, +z up. The GY-521 sits with its
  silkscreen X arrow to the nose and the chip facing up (hardware/frame), so the
  MPU6050's own axes are exactly this frame.

    roll  > 0  right side down     gyro x > 0 while rolling right
    pitch > 0  nose down           gyro y > 0 while pitching nose down
    yaw rate   gyro z > 0 = counter-clockwise seen from above (nose left)

  Motor positions and spin come from flight/mixer.py's table, NOT from the
  mixer's sign conventions: the physics here is independent of the mixer so
  the sim can catch a wrong sign in it. A prop spinning CCW (seen from above)
  pushes the body clockwise, i.e. a -z reaction torque.
"""

import math

import mujoco
import numpy as np

from .params import G, rad

# Motor number -> (x sign, y sign, spin). From flight/mixer.py: M1 front-right
# CCW, M2 rear-left CCW, M3 front-left CW, M4 rear-right CW. Right is -y.
MOTORS = {
    1: (+1, -1, "CCW"),
    2: (-1, +1, "CCW"),
    3: (+1, +1, "CW"),
    4: (-1, -1, "CW"),
}

PHYSICS_DT = 0.001            # s, MuJoCo timestep
IMU_DLPF_HZ = 44.0            # MPU6050 DLPF setting 3, as the driver configures it
START_Z = 0.0125              # body origin height when resting on the floor


def _mjcf(p):
    a = p.motor_half_pitch_mm / 1000.0
    pods, props, legs, sites, arms = [], [], [], [], []
    for n, (sx, sy, spin) in MOTORS.items():
        x, y = sx * a, sy * a
        colour = "0.85 0.35 0.25 0.55" if spin == "CCW" else "0.25 0.5 0.9 0.55"
        pods.append(f'<geom name="pod{n}" type="cylinder" pos="{x} {y} 0.002" size="0.0045 0.006" '
                    f'rgba="0.2 0.2 0.22 1" contype="0" conaffinity="0"/>')
        props.append(f'<geom name="prop{n}" type="cylinder" pos="{x} {y} 0.0095" size="0.0275 0.0006" '
                     f'rgba="{colour}" contype="0" conaffinity="0"/>')
        legs.append(f'<geom name="leg{n}" type="sphere" pos="{x} {y} -0.0095" size="0.003" '
                    f'rgba="0.2 0.2 0.22 1"/>')
        sites.append(f'<site name="m{n}" pos="{x} {y} 0.008" size="0.002"/>')
        arms.append(f'<geom type="capsule" fromto="0 0 0 {x} {y} 0" size="0.0025" '
                    f'rgba="0.92 0.92 0.9 1" contype="0" conaffinity="0"/>')
    actuators = "\n".join(
        f'<motor name="thrust{n}" site="m{n}" gear="0 0 1 0 0 0"/>' for n in MOTORS
    )
    ix, iy, iz = p.inertia()
    nl = "\n        "
    return f"""
<mujoco model="pico-drone">
  <option timestep="{PHYSICS_DT}" gravity="0 0 {-G}" integrator="implicitfast"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.6 0.6 0.6"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.93 0.93 0.9" rgb2="0.82 0.82 0.8"
             width="512" height="512" mark="edge" markrgb="0.6 0.6 0.6"/>
    <material name="grid" texture="grid" texrepeat="12 12"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.75 0.85 0.95" rgb2="1 1 1"
             width="256" height="256"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.6 0.6 0.6"/>
    <geom name="floor" type="plane" size="4 4 0.05" material="grid"/>
    <body name="drone" pos="0 0 {START_Z}">
      <freejoint/>
      <inertial pos="0 0 0" mass="{p.mass}" diaginertia="{ix} {iy} {iz}"/>
      <geom name="deck" type="box" size="0.03 0.0225 0.0015" rgba="0.95 0.95 0.92 1"
            contype="0" conaffinity="0"/>
      <geom name="pico" type="box" pos="0 0.011 0.004" size="0.0255 0.0105 0.002"
            rgba="0.1 0.45 0.2 1" contype="0" conaffinity="0"/>
      <geom name="nose" type="box" pos="0.034 0 0.001" size="0.004 0.004 0.002"
            rgba="0.9 0.2 0.2 1" contype="0" conaffinity="0"/>
      <geom name="battery" type="box" pos="0 0 -0.0065" size="0.021 0.01 0.0045"
            rgba="0.3 0.3 0.35 1"/>
      {nl.join(arms)}
      {nl.join(pods)}
      {nl.join(props)}
      {nl.join(legs)}
      {nl.join(sites)}
      <site name="imu" pos="0 0 0" size="0.002"/>
    </body>
  </worldbody>
  <actuator>
    {actuators}
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>
</mujoco>
"""


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class Drone:
    def __init__(self, params):
        self.model = mujoco.MjModel.from_xml_string(_mjcf(params))
        self.data = mujoco.MjData(self.model)
        self.body = self.model.body("drone").id
        self.set_params(params)

    # --- configuration ----------------------------------------------------------
    def set_params(self, p):
        """Apply a (possibly randomized) parameter set without recompiling."""
        self.p = p
        m = self.model
        m.body_mass[self.body] = p.mass
        m.body_inertia[self.body] = p.inertia()
        m.body_ipos[self.body] = np.array(p.com_offset_mm) / 1000.0
        # Reaction torque per newton of thrust: CCW prop -> -z on the body.
        k = p.yaw_coef_mm / 1000.0
        for i, (_, _, spin) in enumerate(MOTORS.values()):
            m.actuator_gear[i, 5] = -k if spin == "CCW" else +k
        self._scale = np.array(p.motor_scale, dtype=float)
        mis = p.imu_misalign_deg
        # Board-from-body rotation: how the IMU chip sits relative to the frame.
        self._mount = (_rot_y(rad(mis[1])) @ _rot_x(rad(mis[0]))).T
        self._bias = np.array(p.gyro_bias, dtype=float)
        self._lpf_alpha = 1.0 - math.exp(-PHYSICS_DT * 2.0 * math.pi * IMU_DLPF_HZ)

    # --- state ------------------------------------------------------------------
    def reset(self, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, z=None, rates_dps=(0, 0, 0),
              throttles=(0, 0, 0, 0)):
        mujoco.mj_resetData(self.model, self.data)
        d = self.data
        d.qpos[0:3] = [0.0, 0.0, START_Z if z is None else z]
        q = np.zeros(4)
        mujoco.mju_euler2Quat(q, np.array([rad(roll_deg), rad(pitch_deg), rad(yaw_deg)]), "XYZ")
        d.qpos[3:7] = q
        d.qvel[3:6] = np.radians(rates_dps)
        self.thrust = np.array([self._steady(t, throttles) for t in throttles], dtype=float)
        self.thrust *= self._scale
        self.cmd = np.array(throttles, dtype=float)
        if z is None:
            # On the floor: let it settle onto its legs first, the way the real
            # one sits still during gyro calibration. Otherwise the first IMU
            # read happens mid-contact with ~0 g on the accelerometer and the
            # filter seeds itself at a nonsense angle.
            for _ in range(300):
                self.data.ctrl[:] = self.thrust
                self._external_forces()
                mujoco.mj_step(self.model, self.data)
            self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self._gyro_f = d.sensordata[0:3].copy()
        self._accel_f = d.sensordata[3:6].copy()

    def _steady(self, throttle, all_throttles):
        sag = 1.0 - self.p.supply_sag * float(np.mean(all_throttles))
        return self.p.thrust_n(throttle) * sag

    def step(self, throttles, period_s, latency_s=None):
        """
        Hold the last motor command for `latency_s`, then apply `throttles`
        (mixer outputs 0..1, motors 1-4) for the rest of `period_s`. That is
        the board's loop: read IMU, think, then write PWM.
        """
        p = self.p
        lat = p.latency_ms / 1000.0 if latency_s is None else latency_s
        n = max(1, int(round(period_s / PHYSICS_DT)))
        n_lat = min(n, int(round(lat / PHYSICS_DT)))
        new = np.array([throttles[1], throttles[2], throttles[3], throttles[4]], dtype=float)
        tau_up = max(p.motor_tau_up_ms, 1e-3) / 1000.0
        tau_dn = max(p.motor_tau_down_ms, 1e-3) / 1000.0
        a_up = 1.0 - math.exp(-PHYSICS_DT / tau_up)
        a_dn = 1.0 - math.exp(-PHYSICS_DT / tau_dn)
        for i in range(n):
            if i == n_lat:
                self.cmd = new
            target = np.array([self._steady(t, self.cmd) for t in self.cmd]) * self._scale
            alpha = np.where(target > self.thrust, a_up, a_dn)
            self.thrust += (target - self.thrust) * alpha
            self.data.ctrl[:] = self.thrust
            self._external_forces()
            mujoco.mj_step(self.model, self.data)
            # MPU6050 DLPF, run at physics rate.
            s = self.data.sensordata
            self._gyro_f += (s[0:3] - self._gyro_f) * self._lpf_alpha
            self._accel_f += (s[3:6] - self._accel_f) * self._lpf_alpha

    def _external_forces(self):
        """Weight relief (balloon), air drag, and a soft ceiling. World frame, at the CoM."""
        p, d = self.p, self.data
        v = d.qvel[0:3]
        R = d.xmat[self.body].reshape(3, 3)
        w_world = R @ d.qvel[3:6]
        f = np.zeros(6)
        f[0:2] = -p.drag_lin * v[0:2]
        f[2] = p.relief_n - p.relief_drag_z * v[2] - p.drag_lin * v[2]
        z = d.qpos[2]
        if z > 2.5:
            f[2] -= 2.0 * (z - 2.5) + 0.5 * v[2]
        f[3:6] = -p.drag_rot * w_world
        d.xfrc_applied[self.body] = f

    # --- what the board sees ------------------------------------------------------
    def read_imu(self, rng):
        """(accel m/s^2, gyro deg/s) in the IMU's frame, like MPU6050.read()."""
        p = self.p
        load = float(np.mean(self.cmd))
        g_sigma = p.gyro_noise_dps + p.gyro_vibration_dps * load
        a_sigma = p.accel_noise_ms2 + p.accel_vibration_ms2 * load
        gyro = self._mount @ np.degrees(self._gyro_f) + self._bias + rng.normal(0, g_sigma, 3)
        accel = self._mount @ self._accel_f + rng.normal(0, a_sigma, 3)
        return tuple(float(v) for v in accel), tuple(float(v) for v in gyro)

    # --- ground truth (reward, viewer, tests) -------------------------------------
    def attitude_deg(self):
        """True (roll, pitch, yaw) in degrees, ZYX convention."""
        w, x, y, z = self.data.qpos[3:7]
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def rates_dps(self):
        return np.degrees(self.data.qvel[3:6])

    def position(self):
        return self.data.qpos[0:3].copy()

    def kick(self, rates_dps):
        """Add a sudden body-rate disturbance (a gust or a bump)."""
        self.data.qvel[3:6] += np.radians(rates_dps)
