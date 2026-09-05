"""
Python port of the Robotis DARwIn-OP walking gait engine.

This is a faithful, line-by-line port of the C++ sources shipped with Webots:

  * libraries/robotis-op2/robotis/Framework/src/motion/modules/Walking.cpp
  * libraries/robotis-op2/robotis/Framework/src/math/Matrix.cpp
  * libraries/robotis-op2/robotis/Framework/include/MX28.h / Kinematics.h

The algorithm computes, for every 8ms control tick, the 14 target positions
(hip/leg/ankle/shoulder joints) used to make the humanoid walk.  It exposes a
small API compatible with the Webots `RobotisOp2GaitManager` so it can be
driven like the official C++ `walk` example.

NOTE: the quirk in `Matrix3D.__mul__` (starting the result as the identity
matrix before accumulating) is kept on purpose: the shipped C++ behaves exactly
that way and the resulting IK angles are the ones that move the real/simulated
robot correctly.  Any "mathematically correct" version produces wrong angles.
"""

import math

# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------

PI = 3.141592

TIME_UNIT = 8.0  # control tick in ms

# MX-28 (12-bit mode used by the Webots Darwin-op)
MX28_CENTER_VALUE = 2048
MX28_RATIO_ANGLE2VALUE = 11.378  # value units per degree
MX28_RATIO_VALUE2ANGLE = 0.088   # degrees per value unit

# Kinematics (mm)
THIGH_LENGTH = 93.0
CALF_LENGTH = 93.0
ANKLE_LENGTH = 33.5
LEG_LENGTH = 219.5

# Order of the 14 joints driven through inverse kinematics + arm swing:
# 0..5  right leg (hip yaw, hip roll, hip pitch, knee, ankle pitch, ankle roll)
# 6..11 left leg  (same layout)
# 12..13 arms     (right / left shoulder pitch)
JOINT_DIR = [-1, -1, 1, 1, -1, 1,
             -1, -1, -1, -1, 1, 1,
             1, -1]
JOINT_INIT_ANGLE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    -48.345, 41.313]


class Vector3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.X = x
        self.Y = y
        self.Z = z

    def length(self):
        return math.sqrt(self.X * self.X + self.Y * self.Y + self.Z * self.Z)


class Matrix3D:
    """4x4 column-major matrix, ported from the Robotis Matrix3D class."""

    def __init__(self, source=None):
        self.m = [0.0] * 16
        if source is None:
            self.identity()
        else:
            self.m = source.m[:]

    def identity(self):
        self.m[:] = [1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1]

    def __mul__(self, other):
        result = Matrix3D()  # identity (see module docstring: intentional)
        m = self.m
        mat = other.m
        for j in range(4):
            for i in range(4):
                acc = 0.0
                for c in range(4):
                    acc += m[j * 4 + c] * mat[c * 4 + i]
                result.m[j * 4 + i] += acc
        return result

    def inverse(self):
        src = [0.0] * 16
        dst = [0.0] * 16
        tmp = [0.0] * 16

        for i in range(4):
            src[i] = self.m[i * 4]
            src[i + 4] = self.m[i * 4 + 1]
            src[i + 8] = self.m[i * 4 + 2]
            src[i + 12] = self.m[i * 4 + 3]

        tmp[0] = src[10] * src[15]
        tmp[1] = src[11] * src[14]
        tmp[2] = src[9] * src[15]
        tmp[3] = src[11] * src[13]
        tmp[4] = src[9] * src[14]
        tmp[5] = src[10] * src[13]
        tmp[6] = src[8] * src[15]
        tmp[7] = src[11] * src[12]
        tmp[8] = src[8] * src[14]
        tmp[9] = src[10] * src[12]
        tmp[10] = src[8] * src[13]
        tmp[11] = src[9] * src[12]

        dst[0] = (tmp[0] * src[5] + tmp[3] * src[6] + tmp[4] * src[7]) - (tmp[1] * src[5] + tmp[2] * src[6] + tmp[5] * src[7])
        dst[1] = (tmp[1] * src[4] + tmp[6] * src[6] + tmp[9] * src[7]) - (tmp[0] * src[4] + tmp[7] * src[6] + tmp[8] * src[7])
        dst[2] = (tmp[2] * src[4] + tmp[7] * src[5] + tmp[10] * src[7]) - (tmp[3] * src[4] + tmp[6] * src[5] + tmp[11] * src[7])
        dst[3] = (tmp[5] * src[4] + tmp[8] * src[5] + tmp[11] * src[6]) - (tmp[4] * src[4] + tmp[9] * src[5] + tmp[10] * src[6])
        dst[4] = (tmp[1] * src[1] + tmp[2] * src[2] + tmp[5] * src[3]) - (tmp[0] * src[1] + tmp[3] * src[2] + tmp[4] * src[3])
        dst[5] = (tmp[0] * src[0] + tmp[7] * src[2] + tmp[8] * src[3]) - (tmp[1] * src[0] + tmp[6] * src[2] + tmp[9] * src[3])
        dst[6] = (tmp[3] * src[0] + tmp[6] * src[1] + tmp[11] * src[3]) - (tmp[2] * src[0] + tmp[7] * src[1] + tmp[10] * src[3])
        dst[7] = (tmp[4] * src[0] + tmp[9] * src[1] + tmp[10] * src[2]) - (tmp[5] * src[0] + tmp[8] * src[1] + tmp[11] * src[2])

        tmp[0] = src[2] * src[7]
        tmp[1] = src[3] * src[6]
        tmp[2] = src[1] * src[7]
        tmp[3] = src[3] * src[5]
        tmp[4] = src[1] * src[6]
        tmp[5] = src[2] * src[5]
        tmp[6] = src[0] * src[7]
        tmp[7] = src[3] * src[4]
        tmp[8] = src[0] * src[6]
        tmp[9] = src[2] * src[4]
        tmp[10] = src[0] * src[5]
        tmp[11] = src[1] * src[4]

        dst[8] = (tmp[0] * src[13] + tmp[3] * src[14] + tmp[4] * src[15]) - (tmp[1] * src[13] + tmp[2] * src[14] + tmp[5] * src[15])
        dst[9] = (tmp[1] * src[12] + tmp[6] * src[14] + tmp[9] * src[15]) - (tmp[0] * src[12] + tmp[7] * src[14] + tmp[8] * src[15])
        dst[10] = (tmp[2] * src[12] + tmp[7] * src[13] + tmp[10] * src[15]) - (tmp[3] * src[12] + tmp[6] * src[13] + tmp[11] * src[15])
        dst[11] = (tmp[5] * src[12] + tmp[8] * src[13] + tmp[11] * src[14]) - (tmp[4] * src[12] + tmp[9] * src[13] + tmp[10] * src[14])
        dst[12] = (tmp[2] * src[10] + tmp[5] * src[11] + tmp[1] * src[9]) - (tmp[4] * src[11] + tmp[0] * src[9] + tmp[3] * src[10])
        dst[13] = (tmp[8] * src[11] + tmp[0] * src[8] + tmp[7] * src[10]) - (tmp[6] * src[10] + tmp[9] * src[11] + tmp[1] * src[8])
        dst[14] = (tmp[6] * src[9] + tmp[11] * src[11] + tmp[3] * src[8]) - (tmp[10] * src[11] + tmp[2] * src[8] + tmp[7] * src[9])
        dst[15] = (tmp[10] * src[10] + tmp[4] * src[8] + tmp[9] * src[9]) - (tmp[8] * src[9] + tmp[11] * src[10] + tmp[5] * src[8])

        det = src[0] * dst[0] + src[1] * dst[1] + src[2] * dst[2] + src[3] * dst[3]
        if det == 0.0:
            return False
        det = 1.0 / det
        for i in range(16):
            self.m[i] = dst[i] * det
        return True

    def set_transform(self, point, angle):
        # point: Vector3D (translation), angle: Vector3D in DEGREES
        cx = math.cos(angle.X * PI / 180.0)
        cy = math.cos(angle.Y * PI / 180.0)
        cz = math.cos(angle.Z * PI / 180.0)
        sx = math.sin(angle.X * PI / 180.0)
        sy = math.sin(angle.Y * PI / 180.0)
        sz = math.sin(angle.Z * PI / 180.0)
        self.identity()
        m = self.m
        m[0] = cz * cy
        m[1] = cz * sy * sx - sz * cx
        m[2] = cz * sy * cx + sz * sx
        m[3] = point.X
        m[4] = sz * cy
        m[5] = sz * sy * sx + cz * cx
        m[6] = sz * sy * cx - cz * sx
        m[7] = point.Y
        m[8] = -sy
        m[9] = cy * sx
        m[10] = cy * cx
        m[11] = point.Z


def compute_ik(x, y, z, a, b, c):
    """Inverse kinematics for one leg.

    Returns the 6 joint angles (radians) in the order
    (hip yaw, hip roll, hip pitch, knee, ankle pitch, ankle roll) or None.
    """
    x = float(x)
    y = float(y)
    z = float(z)
    out = [0.0] * 6

    tad = Matrix3D()
    tad.set_transform(Vector3D(x, y, z - LEG_LENGTH),
                      Vector3D(a * 180.0 / PI, b * 180.0 / PI, c * 180.0 / PI))

    vec = Vector3D(
        x + tad.m[2] * ANKLE_LENGTH,
        y + tad.m[6] * ANKLE_LENGTH,
        (z - LEG_LENGTH) + tad.m[10] * ANKLE_LENGTH)

    # Knee
    rac = vec.length()
    arg = (rac * rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH)
    if arg < -1.0 or arg > 1.0:
        return None
    out[3] = math.acos(arg)

    # Ankle roll
    tda = Matrix3D(tad)
    if not tda.inverse():
        return None
    k = math.sqrt(tda.m[7] * tda.m[7] + tda.m[11] * tda.m[11])
    l = math.sqrt(tda.m[7] * tda.m[7] + (tda.m[11] - ANKLE_LENGTH) * (tda.m[11] - ANKLE_LENGTH))
    m = (k * k - l * l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * l * ANKLE_LENGTH)
    if m > 1.0:
        m = 1.0
    elif m < -1.0:
        m = -1.0
    acos_val = math.acos(m)
    out[5] = -acos_val if tda.m[7] < 0.0 else acos_val

    # Hip yaw
    tcd = Matrix3D()
    tcd.set_transform(Vector3D(0.0, 0.0, -ANKLE_LENGTH),
                      Vector3D(out[5] * 180.0 / PI, 0.0, 0.0))
    tdc = Matrix3D(tcd)
    if not tdc.inverse():
        return None
    tac = tad * tdc
    out[0] = math.atan2(-tac.m[1], tac.m[5])

    # Hip roll
    out[1] = math.atan2(tac.m[9], -tac.m[1] * math.sin(out[0]) + tac.m[5] * math.cos(out[0]))

    # Hip pitch & ankle pitch
    theta = math.atan2(tac.m[2] * math.cos(out[0]) + tac.m[6] * math.sin(out[0]),
                       tac.m[0] * math.cos(out[0]) + tac.m[4] * math.sin(out[0]))
    k = math.sin(out[3]) * CALF_LENGTH
    l = -THIGH_LENGTH - math.cos(out[3]) * CALF_LENGTH
    m = math.cos(out[0]) * vec.X + math.sin(out[0]) * vec.Y
    n = math.cos(out[1]) * vec.Z + math.sin(out[0]) * math.sin(out[1]) * vec.X - \
        math.cos(out[0]) * math.sin(out[1]) * vec.Y
    s = (k * n + l * m) / (k * k + l * l)
    c = (n - k * s) / l
    out[2] = math.atan2(s, c)
    out[4] = theta - out[3] - out[2]

    return out


class Walking:
    """The Robotis walking gait engine."""

    def __init__(self):
        # Public tuning parameters (mirror Walking.cpp constructor)
        self.x_offset = -10.0
        self.y_offset = 5.0
        self.z_offset = 20.0
        self.r_offset = 0.0
        self.p_offset = 0.0
        self.a_offset = 0.0
        self.hip_pitch_offset = 13.0
        self.period_time = 600.0
        self.dsp_ratio = 0.1
        self.step_fb_ratio = 0.28
        self.z_move_amplitude_param = 40.0  # "foot height"
        self.y_swap_amplitude = 20.0
        self.z_swap_amplitude = 5.0
        self.pelvis_offset = 3.0
        self.arm_swing_gain = 1.5
        self.balance_knee_gain = 0.3
        self.balance_ankle_pitch_gain = 0.9
        self.balance_hip_roll_gain = 0.0
        self.balance_ankle_roll_gain = 0.0
        self.p_gain = 32
        self.i_gain = 0
        self.d_gain = 0

        # Walk control amplitudes (deg, radius)
        self.x_move_amplitude = 0.0  # forward/back
        self.y_move_amplitude = 0.0  # right/left
        self.a_move_amplitude = 0.0  # turn
        self.a_move_aim_on = False
        self.balance_enable = True

        # Gyro feedback (set by the controller each tick)
        self.rl_gyro = 0.0
        self.fb_gyro = 0.0

        self._joint_values = [0] * 14

    # -- internal state helpers ----------------------------------------------

    @property
    def joint_values(self):
        return self._joint_values

    def _wsin(self, time, period, period_shift, mag, mag_shift):
        return mag * math.sin(2 * PI / period * time - period_shift) + mag_shift

    def _update_param_time(self):
        self._m_period_time = self.period_time
        self._m_dsp_ratio = self.dsp_ratio
        self._m_ssp_ratio = 1.0 - self.dsp_ratio

        self._m_x_swap_period_time = self._m_period_time / 2.0
        self._m_x_move_period_time = self._m_period_time * self._m_ssp_ratio
        self._m_y_swap_period_time = self._m_period_time
        self._m_y_move_period_time = self._m_period_time * self._m_ssp_ratio
        self._m_z_swap_period_time = self._m_period_time / 2.0
        self._m_z_move_period_time = self._m_period_time * self._m_ssp_ratio / 2.0
        self._m_a_move_period_time = self._m_period_time * self._m_ssp_ratio

        self._m_ssp_time = self._m_period_time * self._m_ssp_ratio
        self._m_ssp_time_start_l = (1.0 - self._m_ssp_ratio) * self._m_period_time / 4.0
        self._m_ssp_time_end_l = (1.0 + self._m_ssp_ratio) * self._m_period_time / 4.0
        self._m_ssp_time_start_r = (3.0 - self._m_ssp_ratio) * self._m_period_time / 4.0
        self._m_ssp_time_end_r = (3.0 + self._m_ssp_ratio) * self._m_period_time / 4.0

        self._m_phase_time1 = (self._m_ssp_time_end_l + self._m_ssp_time_start_l) / 2.0
        self._m_phase_time2 = (self._m_ssp_time_start_r + self._m_ssp_time_end_l) / 2.0
        self._m_phase_time3 = (self._m_ssp_time_end_r + self._m_ssp_time_start_r) / 2.0

        self._m_pelvis_offset = self.pelvis_offset * MX28_RATIO_ANGLE2VALUE
        self._m_pelvis_swing = self._m_pelvis_offset * 0.35
        self._m_arm_swing_gain = self.arm_swing_gain

    def _update_param_move(self):
        # Forward/Back
        self._m_x_move_amplitude = self.x_move_amplitude
        self._m_x_swap_amplitude = self.x_move_amplitude * self.step_fb_ratio

        # Right/Left
        self._m_y_move_amplitude = self.y_move_amplitude / 2.0
        if self._m_y_move_amplitude > 0:
            self._m_y_move_amplitude_shift = self._m_y_move_amplitude
        else:
            self._m_y_move_amplitude_shift = -self._m_y_move_amplitude
        self._m_y_swap_amplitude = self.y_swap_amplitude + self._m_y_move_amplitude_shift * 0.04

        self._m_z_move_amplitude = self.z_move_amplitude_param / 2.0
        self._m_z_move_amplitude_shift = self._m_z_move_amplitude / 2.0
        self._m_z_swap_amplitude = self.z_swap_amplitude
        self._m_z_swap_amplitude_shift = self._m_z_swap_amplitude

        # Direction (turn)
        if not self.a_move_aim_on:
            self._m_a_move_amplitude = self.a_move_amplitude * PI / 180.0 / 2.0
            if self._m_a_move_amplitude > 0:
                self._m_a_move_amplitude_shift = self._m_a_move_amplitude
            else:
                self._m_a_move_amplitude_shift = -self._m_a_move_amplitude
        else:
            self._m_a_move_amplitude = -self.a_move_amplitude * PI / 180.0 / 2.0
            if self._m_a_move_amplitude > 0:
                self._m_a_move_amplitude_shift = -self._m_a_move_amplitude
            else:
                self._m_a_move_amplitude_shift = self._m_a_move_amplitude

    def _update_param_balance(self):
        self._m_x_offset = self.x_offset
        self._m_y_offset = self.y_offset
        self._m_z_offset = self.z_offset
        self._m_r_offset = self.r_offset * PI / 180.0
        self._m_p_offset = self.p_offset * PI / 180.0
        self._m_a_offset = self.a_offset * PI / 180.0
        self._m_hip_pitch_offset = self.hip_pitch_offset * MX28_RATIO_ANGLE2VALUE

    def initialize(self):
        self.x_move_amplitude = 0.0
        self.y_move_amplitude = 0.0
        self.a_move_amplitude = 0.0

        self._m_body_swing_y = 0.0
        self._m_body_swing_z = 0.0

        self._m_x_swap_phase_shift = PI
        self._m_x_swap_amplitude_shift = 0.0
        self._m_x_move_phase_shift = PI / 2.0
        self._m_x_move_amplitude_shift = 0.0
        self._m_y_swap_phase_shift = 0.0
        self._m_y_swap_amplitude_shift = 0.0
        self._m_y_move_phase_shift = PI / 2.0
        self._m_z_swap_phase_shift = PI * 3.0 / 2.0
        self._m_z_move_phase_shift = PI / 2.0
        self._m_a_move_phase_shift = PI / 2.0

        self._m_ctrl_running = False
        self._m_real_running = False
        self._m_time = 0.0

        self._update_param_time()
        self._update_param_move()
        self.process()

    def start(self):
        self._m_ctrl_running = True
        self._m_real_running = True

    def stop(self):
        self._m_ctrl_running = False

    def is_running(self):
        return self._m_real_running

    def process(self):
        t = self._m_time

        if t == 0.0:
            self._update_param_time()
            self._m_phase = 0
            if not self._m_ctrl_running:
                if (self.x_move_amplitude == 0.0 and self.y_move_amplitude == 0.0
                        and self.a_move_amplitude == 0.0):
                    self._m_real_running = False
                else:
                    self.x_move_amplitude = 0.0
                    self.y_move_amplitude = 0.0
                    self.a_move_amplitude = 0.0
        elif (self._m_phase_time1 - TIME_UNIT / 2.0) <= t < (self._m_phase_time1 + TIME_UNIT / 2.0):
            self._update_param_move()
            self._m_phase = 1
        elif (self._m_phase_time2 - TIME_UNIT / 2.0) <= t < (self._m_phase_time2 + TIME_UNIT / 2.0):
            self._update_param_time()
            self._m_time = self._m_phase_time2
            self._m_phase = 2
            if not self._m_ctrl_running:
                if (self.x_move_amplitude == 0.0 and self.y_move_amplitude == 0.0
                        and self.a_move_amplitude == 0.0):
                    self._m_real_running = False
                else:
                    self.x_move_amplitude = 0.0
                    self.y_move_amplitude = 0.0
                    self.a_move_amplitude = 0.0
        elif (self._m_phase_time3 - TIME_UNIT / 2.0) <= t < (self._m_phase_time3 + TIME_UNIT / 2.0):
            self._update_param_move()
            self._m_phase = 3

        self._update_param_balance()
        t = self._m_time

        # ------------------------------------------------------------------
        # Swap (body) trajectories
        # ------------------------------------------------------------------
        x_swap = self._wsin(t, self._m_x_swap_period_time, self._m_x_swap_phase_shift,
                            self._m_x_swap_amplitude, self._m_x_swap_amplitude_shift)
        y_swap = self._wsin(t, self._m_y_swap_period_time, self._m_y_swap_phase_shift,
                            self._m_y_swap_amplitude, self._m_y_swap_amplitude_shift)
        z_swap = self._wsin(t, self._m_z_swap_period_time, self._m_z_swap_phase_shift,
                            self._m_z_swap_amplitude, self._m_z_swap_amplitude_shift)
        a_swap = 0.0
        b_swap = 0.0
        c_swap = 0.0

        # ------------------------------------------------------------------
        # Move (foot) trajectories, split left/right
        # ------------------------------------------------------------------
        if t <= self._m_ssp_time_start_l:
            x_move_l = self._wsin(self._m_ssp_time_start_l, self._m_x_move_period_time,
                                  self._m_x_move_phase_shift + 2 * PI / self._m_x_move_period_time * self._m_ssp_time_start_l,
                                  self._m_x_move_amplitude, self._m_x_move_amplitude_shift)
            y_move_l = self._wsin(self._m_ssp_time_start_l, self._m_y_move_period_time,
                                  self._m_y_move_phase_shift + 2 * PI / self._m_y_move_period_time * self._m_ssp_time_start_l,
                                  self._m_y_move_amplitude, self._m_y_move_amplitude_shift)
            z_move_l = self._wsin(self._m_ssp_time_start_l, self._m_z_move_period_time,
                                  self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_l,
                                  self._m_z_move_amplitude, self._m_z_move_amplitude_shift)
            c_move_l = self._wsin(self._m_ssp_time_start_l, self._m_a_move_period_time,
                                  self._m_a_move_phase_shift + 2 * PI / self._m_a_move_period_time * self._m_ssp_time_start_l,
                                  self._m_a_move_amplitude, self._m_a_move_amplitude_shift)
            x_move_r = self._wsin(self._m_ssp_time_start_l, self._m_x_move_period_time,
                                  self._m_x_move_phase_shift + 2 * PI / self._m_x_move_period_time * self._m_ssp_time_start_l,
                                  -self._m_x_move_amplitude, -self._m_x_move_amplitude_shift)
            y_move_r = self._wsin(self._m_ssp_time_start_l, self._m_y_move_period_time,
                                  self._m_y_move_phase_shift + 2 * PI / self._m_y_move_period_time * self._m_ssp_time_start_l,
                                  -self._m_y_move_amplitude, -self._m_y_move_amplitude_shift)
            z_move_r = self._wsin(self._m_ssp_time_start_r, self._m_z_move_period_time,
                                  self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_r,
                                  self._m_z_move_amplitude, self._m_z_move_amplitude_shift)
            c_move_r = self._wsin(self._m_ssp_time_start_l, self._m_a_move_period_time,
                                  self._m_a_move_phase_shift + 2 * PI / self._m_a_move_period_time * self._m_ssp_time_start_l,
                                  -self._m_a_move_amplitude, -self._m_a_move_amplitude_shift)
            pelvis_offset_l = 0.0
            pelvis_offset_r = 0.0
        elif t <= self._m_ssp_time_end_l:
            x_move_l = self._wsin(t, self._m_x_move_period_time,
                                  self._m_x_move_phase_shift + 2 * PI / self._m_x_move_period_time * self._m_ssp_time_start_l,
                                  self._m_x_move_amplitude, self._m_x_move_amplitude_shift)
            y_move_l = self._wsin(t, self._m_y_move_period_time,
                                  self._m_y_move_phase_shift + 2 * PI / self._m_y_move_period_time * self._m_ssp_time_start_l,
                                  self._m_y_move_amplitude, self._m_y_move_amplitude_shift)
            z_move_l = self._wsin(t, self._m_z_move_period_time,
                                  self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_l,
                                  self._m_z_move_amplitude, self._m_z_move_amplitude_shift)
            c_move_l = self._wsin(t, self._m_a_move_period_time,
                                  self._m_a_move_phase_shift + 2 * PI / self._m_a_move_period_time * self._m_ssp_time_start_l,
                                  self._m_a_move_amplitude, self._m_a_move_amplitude_shift)
            x_move_r = self._wsin(t, self._m_x_move_period_time,
                                  self._m_x_move_phase_shift + 2 * PI / self._m_x_move_period_time * self._m_ssp_time_start_l,
                                  -self._m_x_move_amplitude, -self._m_x_move_amplitude_shift)
            y_move_r = self._wsin(t, self._m_y_move_period_time,
                                  self._m_y_move_phase_shift + 2 * PI / self._m_y_move_period_time * self._m_ssp_time_start_l,
                                  -self._m_y_move_amplitude, -self._m_y_move_amplitude_shift)
            z_move_r = self._wsin(self._m_ssp_time_start_r, self._m_z_move_period_time,
                                  self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_r,
                                  self._m_z_move_amplitude, self._m_z_move_amplitude_shift)
            c_move_r = self._wsin(t, self._m_a_move_period_time,
                                  self._m_a_move_phase_shift + 2 * PI / self._m_a_move_period_time * self._m_ssp_time_start_l,
                                  -self._m_a_move_amplitude, -self._m_a_move_amplitude_shift)
            pelvis_offset_l = self._wsin(t, self._m_z_move_period_time,
                                         self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_l,
                                         self._m_pelvis_swing / 2.0, self._m_pelvis_swing / 2.0)
            pelvis_offset_r = self._wsin(t, self._m_z_move_period_time,
                                         self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_l,
                                         -self._m_pelvis_offset / 2.0, -self._m_pelvis_offset / 2.0)
        elif t <= self._m_ssp_time_start_r:
            x_move_l = self._wsin(self._m_ssp_time_end_l, self._m_x_move_period_time,
                                  self._m_x_move_phase_shift + 2 * PI / self._m_x_move_period_time * self._m_ssp_time_start_l,
                                  self._m_x_move_amplitude, self._m_x_move_amplitude_shift)
            y_move_l = self._wsin(self._m_ssp_time_end_l, self._m_y_move_period_time,
                                  self._m_y_move_phase_shift + 2 * PI / self._m_y_move_period_time * self._m_ssp_time_start_l,
                                  self._m_y_move_amplitude, self._m_y_move_amplitude_shift)
            z_move_l = self._wsin(self._m_ssp_time_end_l, self._m_z_move_period_time,
                                  self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_l,
                                  self._m_z_move_amplitude, self._m_z_move_amplitude_shift)
            c_move_l = self._wsin(self._m_ssp_time_end_l, self._m_a_move_period_time,
                                  self._m_a_move_phase_shift + 2 * PI / self._m_a_move_period_time * self._m_ssp_time_start_l,
                                  self._m_a_move_amplitude, self._m_a_move_amplitude_shift)
            x_move_r = self._wsin(self._m_ssp_time_end_l, self._m_x_move_period_time,
                                  self._m_x_move_phase_shift + 2 * PI / self._m_x_move_period_time * self._m_ssp_time_start_l,
                                  -self._m_x_move_amplitude, -self._m_x_move_amplitude_shift)
            y_move_r = self._wsin(self._m_ssp_time_end_l, self._m_y_move_period_time,
                                  self._m_y_move_phase_shift + 2 * PI / self._m_y_move_period_time * self._m_ssp_time_start_l,
                                  -self._m_y_move_amplitude, -self._m_y_move_amplitude_shift)
            z_move_r = self._wsin(self._m_ssp_time_start_r, self._m_z_move_period_time,
                                  self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_r,
                                  self._m_z_move_amplitude, self._m_z_move_amplitude_shift)
            c_move_r = self._wsin(self._m_ssp_time_end_l, self._m_a_move_period_time,
                                  self._m_a_move_phase_shift + 2 * PI / self._m_a_move_period_time * self._m_ssp_time_start_l,
                                  -self._m_a_move_amplitude, -self._m_a_move_amplitude_shift)
            pelvis_offset_l = 0.0
            pelvis_offset_r = 0.0
        elif t <= self._m_ssp_time_end_r:
            x_move_l = self._wsin(t, self._m_x_move_period_time,
                                  self._m_x_move_phase_shift + 2 * PI / self._m_x_move_period_time * self._m_ssp_time_start_r + PI,
                                  self._m_x_move_amplitude, self._m_x_move_amplitude_shift)
            y_move_l = self._wsin(t, self._m_y_move_period_time,
                                  self._m_y_move_phase_shift + 2 * PI / self._m_y_move_period_time * self._m_ssp_time_start_r + PI,
                                  self._m_y_move_amplitude, self._m_y_move_amplitude_shift)
            z_move_l = self._wsin(self._m_ssp_time_end_l, self._m_z_move_period_time,
                                  self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_l,
                                  self._m_z_move_amplitude, self._m_z_move_amplitude_shift)
            c_move_l = self._wsin(t, self._m_a_move_period_time,
                                  self._m_a_move_phase_shift + 2 * PI / self._m_a_move_period_time * self._m_ssp_time_start_r + PI,
                                  self._m_a_move_amplitude, self._m_a_move_amplitude_shift)
            x_move_r = self._wsin(t, self._m_x_move_period_time,
                                  self._m_x_move_phase_shift + 2 * PI / self._m_x_move_period_time * self._m_ssp_time_start_r + PI,
                                  -self._m_x_move_amplitude, -self._m_x_move_amplitude_shift)
            y_move_r = self._wsin(t, self._m_y_move_period_time,
                                  self._m_y_move_phase_shift + 2 * PI / self._m_y_move_period_time * self._m_ssp_time_start_r + PI,
                                  -self._m_y_move_amplitude, -self._m_y_move_amplitude_shift)
            z_move_r = self._wsin(t, self._m_z_move_period_time,
                                  self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_r,
                                  self._m_z_move_amplitude, self._m_z_move_amplitude_shift)
            c_move_r = self._wsin(t, self._m_a_move_period_time,
                                  self._m_a_move_phase_shift + 2 * PI / self._m_a_move_period_time * self._m_ssp_time_start_r + PI,
                                  -self._m_a_move_amplitude, -self._m_a_move_amplitude_shift)
            pelvis_offset_l = self._wsin(t, self._m_z_move_period_time,
                                         self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_r,
                                         self._m_pelvis_offset / 2.0, self._m_pelvis_offset / 2.0)
            pelvis_offset_r = self._wsin(t, self._m_z_move_period_time,
                                         self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_r,
                                         -self._m_pelvis_swing / 2.0, -self._m_pelvis_swing / 2.0)
        else:
            x_move_l = self._wsin(self._m_ssp_time_end_r, self._m_x_move_period_time,
                                  self._m_x_move_phase_shift + 2 * PI / self._m_x_move_period_time * self._m_ssp_time_start_r + PI,
                                  self._m_x_move_amplitude, self._m_x_move_amplitude_shift)
            y_move_l = self._wsin(self._m_ssp_time_end_r, self._m_y_move_period_time,
                                  self._m_y_move_phase_shift + 2 * PI / self._m_y_move_period_time * self._m_ssp_time_start_r + PI,
                                  self._m_y_move_amplitude, self._m_y_move_amplitude_shift)
            z_move_l = self._wsin(self._m_ssp_time_end_l, self._m_z_move_period_time,
                                  self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_l,
                                  self._m_z_move_amplitude, self._m_z_move_amplitude_shift)
            c_move_l = self._wsin(self._m_ssp_time_end_r, self._m_a_move_period_time,
                                  self._m_a_move_phase_shift + 2 * PI / self._m_a_move_period_time * self._m_ssp_time_start_r + PI,
                                  self._m_a_move_amplitude, self._m_a_move_amplitude_shift)
            x_move_r = self._wsin(self._m_ssp_time_end_r, self._m_x_move_period_time,
                                  self._m_x_move_phase_shift + 2 * PI / self._m_x_move_period_time * self._m_ssp_time_start_r + PI,
                                  -self._m_x_move_amplitude, -self._m_x_move_amplitude_shift)
            y_move_r = self._wsin(self._m_ssp_time_end_r, self._m_y_move_period_time,
                                  self._m_y_move_phase_shift + 2 * PI / self._m_y_move_period_time * self._m_ssp_time_start_r + PI,
                                  -self._m_y_move_amplitude, -self._m_y_move_amplitude_shift)
            z_move_r = self._wsin(self._m_ssp_time_end_r, self._m_z_move_period_time,
                                  self._m_z_move_phase_shift + 2 * PI / self._m_z_move_period_time * self._m_ssp_time_start_r,
                                  self._m_z_move_amplitude, self._m_z_move_amplitude_shift)
            c_move_r = self._wsin(self._m_ssp_time_end_r, self._m_a_move_period_time,
                                  self._m_a_move_phase_shift + 2 * PI / self._m_a_move_period_time * self._m_ssp_time_start_r + PI,
                                  -self._m_a_move_amplitude, -self._m_a_move_amplitude_shift)
            pelvis_offset_l = 0.0
            pelvis_offset_r = 0.0

        a_move_l = 0.0
        b_move_l = 0.0
        a_move_r = 0.0
        b_move_r = 0.0

        # ------------------------------------------------------------------
        # End-points of the feet
        # ------------------------------------------------------------------
        ep = [0.0] * 12
        ep[0] = x_swap + x_move_r + self._m_x_offset
        ep[1] = y_swap + y_move_r - self._m_y_offset / 2.0
        ep[2] = z_swap + z_move_r + self._m_z_offset
        ep[3] = a_swap + a_move_r - self._m_r_offset / 2.0
        ep[4] = b_swap + b_move_r + self._m_p_offset
        ep[5] = c_swap + c_move_r - self._m_a_offset / 2.0
        ep[6] = x_swap + x_move_l + self._m_x_offset
        ep[7] = y_swap + y_move_l + self._m_y_offset / 2.0
        ep[8] = z_swap + z_move_l + self._m_z_offset
        ep[9] = a_swap + a_move_l + self._m_r_offset / 2.0
        ep[10] = b_swap + b_move_l + self._m_p_offset
        ep[11] = c_swap + c_move_l + self._m_a_offset / 2.0

        # ------------------------------------------------------------------
        # Body swing
        # ------------------------------------------------------------------
        if t <= self._m_ssp_time_end_l:
            self._m_body_swing_y = -ep[7]
            self._m_body_swing_z = ep[8]
        else:
            self._m_body_swing_y = -ep[1]
            self._m_body_swing_z = ep[2]
        self._m_body_swing_z -= LEG_LENGTH

        # ------------------------------------------------------------------
        # Arm swing
        # ------------------------------------------------------------------
        angle = [0.0] * 14
        if self._m_x_move_amplitude == 0.0:
            angle[12] = 0.0  # right arm
            angle[13] = 0.0  # left arm
        else:
            angle[12] = self._wsin(t, self._m_period_time, PI * 1.5,
                                   -self._m_x_move_amplitude * self._m_arm_swing_gain, 0.0)
            angle[13] = self._wsin(t, self._m_period_time, PI * 1.5,
                                   self._m_x_move_amplitude * self._m_arm_swing_gain, 0.0)

        if self._m_real_running:
            self._m_time += TIME_UNIT
            if self._m_time >= self._m_period_time:
                self._m_time = 0.0

        # ------------------------------------------------------------------
        # Inverse kinematics
        # ------------------------------------------------------------------
        ik_r = compute_ik(ep[0], ep[1], ep[2], ep[3], ep[4], ep[5])
        ik_l = compute_ik(ep[6], ep[7], ep[8], ep[9], ep[10], ep[11])
        if ik_r is None or ik_l is None:
            return  # keep previous target positions

        for i in range(6):
            angle[i] = ik_r[i] * 180.0 / PI
            angle[i + 6] = ik_l[i] * 180.0 / PI

        # ------------------------------------------------------------------
        # Convert to MX-28 value units
        # ------------------------------------------------------------------
        out_value = [0] * 14
        for i in range(14):
            offset = JOINT_DIR[i] * angle[i] * MX28_RATIO_ANGLE2VALUE
            if i == 1:  # R_HIP_ROLL
                offset += JOINT_DIR[i] * pelvis_offset_r
            elif i == 7:  # L_HIP_ROLL
                offset += JOINT_DIR[i] * pelvis_offset_l
            elif i == 2 or i == 8:  # R/L HIP_PITCH
                offset -= JOINT_DIR[i] * self._m_hip_pitch_offset
            out_value[i] = int(JOINT_INIT_ANGLE[i] * MX28_RATIO_ANGLE2VALUE) + MX28_CENTER_VALUE + int(offset)

        # ------------------------------------------------------------------
        # Balance correction from gyro
        # ------------------------------------------------------------------
        if self.balance_enable:
            rl_err = self.rl_gyro
            fb_err = self.fb_gyro
            out_value[1] += int(JOINT_DIR[1] * rl_err * self.balance_hip_roll_gain * 4)
            out_value[7] += int(JOINT_DIR[7] * rl_err * self.balance_hip_roll_gain * 4)
            out_value[3] -= int(JOINT_DIR[3] * fb_err * self.balance_knee_gain * 4)
            out_value[9] -= int(JOINT_DIR[9] * fb_err * self.balance_knee_gain * 4)
            out_value[4] -= int(JOINT_DIR[4] * fb_err * self.balance_ankle_pitch_gain * 4)
            out_value[10] -= int(JOINT_DIR[10] * fb_err * self.balance_ankle_pitch_gain * 4)
            out_value[5] -= int(JOINT_DIR[5] * rl_err * self.balance_ankle_roll_gain * 4)
            out_value[11] -= int(JOINT_DIR[11] * rl_err * self.balance_ankle_roll_gain * 4)

        self._joint_values = out_value

    # -- convenience helpers ------------------------------------------------

    @staticmethod
    def value_to_position(value):
        """Convert a MX-28 value unit to a Webots motor position (radians)."""
        degree = (value - MX28_CENTER_VALUE) * MX28_RATIO_VALUE2ANGLE
        return degree / 180.0 * PI