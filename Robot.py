"""Python port of Robot.cs.

Dependencies:
    pip install numpy

The public API intentionally keeps the original C# property/method names
(`Angle`, `Position`, `TransMatrix`, `set_robot`, `ikine8`, `dh2motor`, ...)
so code translated from the original DMArmDLL project can call it directly.
`Robot` can work with the converted DMMotor.py from the previous step; only
`Position` and `angle_lim` are needed by `motor2dh()`/`dh2motor()`.
"""
from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple, Union

import numpy as np

from TreeStruct import Tree

ArrayLike = Union[Sequence[float], np.ndarray]


class Robot:
    """Forward/inverse kinematics, Jacobian, and torque model for the DM arm."""

    def __init__(self) -> None:
        self.ratio = np.array([1.0, 1.0, -1.0, 1.0, -1.0, 1.0], dtype=float)
        self.zero_offset = np.array([-1.57079632679490, 2.7321, -1.58889015515032, 0.0, 0.127707486697677, 0.0], dtype=float)

        self.fh = np.zeros(6, dtype=float)

        self.d1 = 0.111
        self.d4 = 0.245
        self.d6 = 0.087
        self.a1 = 0.0
        self.a2 = 0.294
        self.a3 = -0.0665

        # The C# constructor assigns a preset q and then immediately resets it to zeros.
        self.q = np.zeros(6, dtype=float)
        self.pos = np.zeros(3, dtype=float)
        self.rpy = np.zeros(3, dtype=float)
        self.c = np.zeros(6, dtype=float)
        self.s = np.zeros(6, dtype=float)

        self.trans = np.zeros((4, 4), dtype=float)
        self.rot = np.eye(3, dtype=float)
        self.jacob0 = np.zeros((6, 6), dtype=float)
        self.jacob6 = np.zeros((6, 6), dtype=float)

        self.g0 = np.array([0.0, 0.0, -9.80665], dtype=float)
        self.tau_g = np.zeros(6, dtype=float)
        self.tau_fh = np.zeros(6, dtype=float)
        self.tau = np.zeros(6, dtype=float)

        self.m = np.zeros(6, dtype=float)
        mr = np.array([
            [0.161449, 0.0, 0.0, -0.036144],
            [1.455285, 0.142943, 0.00000, -0.003],
            [0.597494, -0.066003, -0.118093, 0.0],
            [0.521769, 0.0, -0.000857, -0.003509],
            [0.368476, 0.0, -0.05902, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ], dtype=float)
        self._set_mass_center(mr)

        self.safe = True
        self.set_robot()

    # ------------------------------------------------------------------
    # C#-compatible properties
    # ------------------------------------------------------------------
    @property
    def Angle(self) -> np.ndarray:
        return self.q.copy()

    @Angle.setter
    def Angle(self, value: ArrayLike) -> None:
        value_arr = np.asarray(value, dtype=float)
        if value_arr.shape == (6,):
            self.q = self.angle_clip_pnpi(value_arr)
            self.safe = True
        else:
            self.safe = False

    @property
    def Position(self) -> np.ndarray:
        return self.pos.copy()

    @Position.setter
    def Position(self, value: ArrayLike) -> None:
        value_arr = np.asarray(value, dtype=float)
        if value_arr.shape != (3,):
            self.safe = False
            return
        q_now = self.ikine_near(self.rotpos2trans(self.rot, value_arr), self.q)
        if q_now is None:
            self.safe = False
            return
        self.q = q_now.copy()
        self.pos = value_arr.copy()
        self.trans = self.rotpos2trans(self.rot, self.pos)
        self.safe = True

    @property
    def RPY(self) -> np.ndarray:
        return self.rpy.copy()

    @RPY.setter
    def RPY(self, value: ArrayLike) -> None:
        value_arr = np.asarray(value, dtype=float)
        if value_arr.shape != (3,):
            self.safe = False
            return
        rot_new = self.rpy2rot(value_arr)
        q_now = self.ikine_near(self.rotpos2trans(rot_new, self.pos), self.q)
        if q_now is None:
            self.safe = False
            return
        self.q = q_now.copy()
        self.rot = rot_new
        self.trans = self.rotpos2trans(self.rot, self.pos)
        self.safe = True

    @property
    def TransMatrix(self) -> np.ndarray:
        return self.trans.copy()

    @TransMatrix.setter
    def TransMatrix(self, value: ArrayLike) -> None:
        value_arr = np.asarray(value, dtype=float)
        if value_arr.shape != (4, 4):
            self.safe = False
            return
        q_now = self.ikine_near(value_arr, self.q)
        if q_now is None:
            self.safe = False
            return
        self.q = q_now.copy()
        self.trans = value_arr.copy()
        self.rot = self.trans2rot(self.trans)
        self.pos = self.trans2pos(self.trans)
        self.safe = True

    @property
    def RotMatrix(self) -> np.ndarray:
        return self.rot.copy()

    @RotMatrix.setter
    def RotMatrix(self, value: ArrayLike) -> None:
        value_arr = np.asarray(value, dtype=float)
        if value_arr.shape != (3, 3):
            self.safe = False
            return
        q_now = self.ikine_near(self.rotpos2trans(value_arr, self.pos), self.q)
        if q_now is None:
            self.safe = False
            return
        self.rot = value_arr.copy()
        self.trans = self.rotpos2trans(self.rot, self.pos)
        self.q = q_now.copy()
        self.safe = True

    @property
    def Jacob0(self) -> np.ndarray:
        return self.jacob0.copy()

    @property
    def Jacob6(self) -> np.ndarray:
        return self.jacob6.copy()

    @property
    def Fh(self) -> np.ndarray:
        return self.fh.copy()

    @Fh.setter
    def Fh(self, value: ArrayLike) -> None:
        value_arr = np.asarray(value, dtype=float).reshape(-1)
        n = min(6, value_arr.size)
        self.fh[:n] = value_arr[:n]

    @property
    def Tau_G(self) -> np.ndarray:
        return self.tau_g.copy()

    @property
    def Tau_Fh(self) -> np.ndarray:
        return self.tau_fh.copy()

    @property
    def Tau(self) -> np.ndarray:
        return self.tau.copy()

    @property
    def Tau_G_Motor(self) -> np.ndarray:
        return self.tau_g / self.ratio

    @property
    def Tau_Fh_Motor(self) -> np.ndarray:
        return self.tau_fh / self.ratio

    @property
    def G_Tool(self) -> np.ndarray:
        return self.R06 @ self.g0

    # ------------------------------------------------------------------
    # Parameter reset and motor-angle conversion
    # ------------------------------------------------------------------
    def _set_mass_center(self, mr: ArrayLike) -> None:
        mr_arr = np.asarray(mr, dtype=float)
        if mr_arr.shape != (6, 4):
            raise ValueError("mr must have shape (6, 4): [mass, x, y, z] per link")
        self.m = mr_arr[:, 0].copy()
        self.r1 = mr_arr[0, 1:4].copy()
        self.r2 = mr_arr[1, 1:4].copy()
        self.r3 = mr_arr[2, 1:4].copy()
        self.r4 = mr_arr[3, 1:4].copy()
        self.r5 = mr_arr[4, 1:4].copy()
        self.r6 = mr_arr[5, 1:4].copy()

    def reset_param(self, mr: ArrayLike, D1: float, D4: float, D6: float, A1: float, A2: float, A3: float, Ratio: ArrayLike, ZeroOffset: ArrayLike) -> None:
        self.d1, self.d4, self.d6 = float(D1), float(D4), float(D6)
        self.a1, self.a2, self.a3 = float(A1), float(A2), float(A3)
        self.q = np.array([-math.pi / 2, -math.pi / 2, math.pi / 2, 0.0, 0.0, math.pi / 2], dtype=float)
        self.ratio = np.asarray(Ratio, dtype=float).reshape(6).copy()
        self.zero_offset = np.asarray(ZeroOffset, dtype=float).reshape(6).copy()
        self._set_mass_center(mr)
        self.safe = True
        self.set_robot()

    def reset_tool_param(self, tool_mr: ArrayLike) -> None:
        mr = np.array([
            [0.161449, 0.0, 0.0, -0.036144],
            [1.455285, 0.142943, 0.00000, -0.003],
            [0.597494, -0.066003, -0.118093, 0.0],
            [0.521769, 0.0, -0.000857, -0.003509],
            np.asarray(tool_mr, dtype=float).reshape(4),
            [0.0, 0.0, 0.0, 0.0],
        ], dtype=float)
        self._set_mass_center(mr)
        self.safe = True
        self.set_robot()

    def motor2dh(self, motors: Sequence[object]) -> np.ndarray:
        dh = np.zeros(6, dtype=float)
        for i in range(6):
            pos = getattr(motors[i], "Position")
            dh[i] = self.angle_clip_pnpi(float(pos) / self.ratio[i] + self.zero_offset[i])
        return dh

    def _in_motor_lim(self, motor_i: object, dh_i: float, r: float, z: float) -> bool:
        angle_lim = getattr(motor_i, "angle_lim")
        q1 = (dh_i - z) * r
        for d in (q1, q1 + 2 * math.pi, q1 - 2 * math.pi):
            if angle_lim[0] <= d <= angle_lim[1]:
                return True
        return False

    def _in_all_motor_lim(self, motors: Sequence[object], dh: ArrayLike) -> Tuple[bool, np.ndarray]:
        dh_arr = np.asarray(dh, dtype=float).reshape(6)
        in_range = np.zeros(6, dtype=bool)
        for i in range(6):
            in_range[i] = self._in_motor_lim(motors[i], dh_arr[i], self.ratio[i], self.zero_offset[i])
        return bool(np.all(in_range)), in_range

    def dh2motor(self, motors: Sequence[object], dh: ArrayLike):
        """Convert DH angle(s) to motor angle(s).

        For a 1-D six-element input, returns `(success, motor_angle, in_range)`.
        For a 2-D input with shape `(N, 6)`, returns `(success, motor_angle_matrix)`.
        """
        dh_arr = np.asarray(dh, dtype=float)
        if dh_arr.ndim == 2:
            res = np.zeros(dh_arr.shape, dtype=np.float32)
            for i in range(dh_arr.shape[0]):
                ok, motor_angle, _ = self.dh2motor(motors, dh_arr[i, :])
                if not ok:
                    return False, res
                res[i, :] = motor_angle
            return True, res

        dh_vec = dh_arr.reshape(6)
        ok, in_range = self._in_all_motor_lim(motors, dh_vec)
        motor_angle = np.zeros(6, dtype=np.float32)
        if not ok:
            return False, motor_angle, in_range

        for i in range(6):
            temp = (dh_vec[i] - self.zero_offset[i]) * self.ratio[i]
            angle_lim = getattr(motors[i], "angle_lim")
            for candidate in (temp, temp + 2 * math.pi, temp - 2 * math.pi):
                if angle_lim[0] < candidate < angle_lim[1]:
                    motor_angle[i] = np.float32(candidate)
                    break
            else:
                motor_angle[i] = np.float32(temp)
        return True, motor_angle, in_range

    # ------------------------------------------------------------------
    # Robot state calculation
    # ------------------------------------------------------------------
    def set_robot(self) -> bool:
        if not self.safe:
            print("无解！")
            return False

        self.s = np.sin(self.q)
        self.c = np.cos(self.q)
        c, s = self.c, self.s
        a1, a2, a3 = self.a1, self.a2, self.a3
        d1, d4, d6 = self.d1, self.d4, self.d6

        self.T10 = np.array([[c[0], -s[0], 0.0, 0.0], [s[0], c[0], 0.0, 0.0], [0.0, 0.0, 1.0, d1], [0.0, 0.0, 0.0, 1.0]], dtype=float)
        self.T21 = np.array([[c[1], -s[1], 0.0, a1], [0.0, 0.0, 1.0, 0.0], [-s[1], -c[1], 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=float)
        self.T32 = np.array([[c[2], -s[2], 0.0, a2], [s[2], c[2], 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=float)
        self.T43 = np.array([[c[3], -s[3], 0.0, a3], [0.0, 0.0, -1.0, -d4], [s[3], c[3], 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=float)
        self.T54 = np.array([[c[4], -s[4], 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [-s[4], -c[4], 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=float)
        self.T65 = np.array([[c[5], -s[5], 0.0, 0.0], [0.0, 0.0, -1.0, -d6], [s[5], c[5], 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]], dtype=float)

        self.T20 = self.T10 @ self.T21
        self.T30 = self.T20 @ self.T32
        self.T40 = self.T30 @ self.T43
        self.T50 = self.T40 @ self.T54
        self.T60 = self.T50 @ self.T65

        self.T61 = self.T21 @ self.T32 @ self.T43 @ self.T54 @ self.T65
        self.T62 = self.T32 @ self.T43 @ self.T54 @ self.T65
        self.T63 = self.T43 @ self.T54 @ self.T65
        self.T64 = self.T54 @ self.T65

        self.trans = self.T60.copy()
        self.pos = self.trans2pos(self.trans)
        self.rot = self.trans2rot(self.trans)
        self.rpy = self.rot2rpy(self.trans)

        self.R10 = self.T10[:3, :3]
        self.R21 = self.T21[:3, :3]
        self.R32 = self.T32[:3, :3]
        self.R43 = self.T43[:3, :3]
        self.R54 = self.T54[:3, :3]
        self.R65 = self.T65[:3, :3]

        self.R20 = self.T20[:3, :3]
        self.R30 = self.T30[:3, :3]
        self.R40 = self.T40[:3, :3]
        self.R50 = self.T50[:3, :3]
        self.R60 = self.T60[:3, :3]

        self.R01 = self.R10.T
        self.R02 = self.R20.T
        self.R03 = self.R30.T
        self.R04 = self.R40.T
        self.R05 = self.R50.T
        self.R06 = self.R60.T

        self.P10 = self.T10[:3, 3]
        self.P21 = self.T21[:3, 3]
        self.P32 = self.T32[:3, 3]
        self.P43 = self.T43[:3, 3]
        self.P54 = self.T54[:3, 3]
        self.P65 = self.T65[:3, 3]

        self.Z1 = self.T10[:3, 2]
        self.Z2 = self.T20[:3, 2]
        self.Z3 = self.T30[:3, 2]
        self.Z4 = self.T40[:3, 2]
        self.Z5 = self.T50[:3, 2]
        self.Z6 = self.T60[:3, 2]

        self.P61 = self.T61[:3, 3]
        self.P62 = self.T62[:3, 3]
        self.P63 = self.T63[:3, 3]
        self.P64 = self.T64[:3, 3]
        self.P66 = np.zeros(3, dtype=float)

        J1 = self._append(np.cross(self.Z1, self.R10 @ self.P61), self.Z1)
        J2 = self._append(np.cross(self.Z2, self.R20 @ self.P62), self.Z2)
        J3 = self._append(np.cross(self.Z3, self.R30 @ self.P63), self.Z3)
        J4 = self._append(np.cross(self.Z4, self.R40 @ self.P64), self.Z4)
        J5 = self._append(np.cross(self.Z5, self.R50 @ self.P65), self.Z5)
        J6 = self._append(np.cross(self.Z6, self.R60 @ self.P66), self.Z6)

        self.Ja0 = np.column_stack((J1, J2, J3, J4, J5, J6))
        R06_block = np.block([[self.R06, np.zeros((3, 3))], [np.zeros((3, 3)), self.R06]])
        self.Ja6 = R06_block @ self.Ja0
        self.jacob0 = self.Ja0.copy()
        self.jacob6 = self.Ja6.copy()

        self.G1 = self.R01 @ self.g0 * self.m[0]
        self.G2 = self.R02 @ self.g0 * self.m[1]
        self.G3 = self.R03 @ self.g0 * self.m[2]
        self.G4 = self.R04 @ self.g0 * self.m[3]
        self.G5 = self.R05 @ self.g0 * self.m[4]
        self.G6 = self.R06 @ self.g0 * self.m[5]

        self.F6 = -self.G6
        self.F5 = self.R65 @ self.F6 - self.G5
        self.F4 = self.R54 @ self.F5 - self.G4
        self.F3 = self.R43 @ self.F4 - self.G3
        self.F2 = self.R32 @ self.F3 - self.G2
        self.F1 = self.R21 @ self.F2 - self.G1

        self.M6 = -np.cross(self.r6, self.G6)
        self.M5 = self.R65 @ self.M6 + np.cross(self.P65, self.R65 @ self.F6) - np.cross(self.r5, self.G5)
        self.M4 = self.R54 @ self.M5 + np.cross(self.P54, self.R54 @ self.F5) - np.cross(self.r4, self.G4)
        self.M3 = self.R43 @ self.M4 + np.cross(self.P43, self.R43 @ self.F4) - np.cross(self.r3, self.G3)
        self.M2 = self.R32 @ self.M3 + np.cross(self.P32, self.R32 @ self.F3) - np.cross(self.r2, self.G2)
        self.M1 = self.R21 @ self.M2 + np.cross(self.P21, self.R21 @ self.F2) - np.cross(self.r1, self.G1)

        self.tau_g = np.array([self.M1[2], self.M2[2], self.M3[2], self.M4[2], self.M5[2], self.M6[2]], dtype=float)
        self.tau_fh = self.Ja0.T @ self.fh
        self.tau = self.tau_g + self.tau_fh
        return True

    @staticmethod
    def _append(left: ArrayLike, right: ArrayLike) -> np.ndarray:
        return np.concatenate((np.asarray(left, dtype=float), np.asarray(right, dtype=float)))

    @staticmethod
    def cross(a: ArrayLike, b: ArrayLike) -> np.ndarray:
        return np.cross(np.asarray(a, dtype=float), np.asarray(b, dtype=float))

    # ------------------------------------------------------------------
    # Cartesian incremental motion helpers
    # ------------------------------------------------------------------
    def move_world(self, distance: float, axis: str) -> bool:
        pos_now = self.pos.copy()
        idx = {"x": 0, "y": 1, "z": 2}.get(axis)
        if idx is None:
            return False
        pos_now[idx] += distance
        q_now = self.ikine_near(self.rotpos2trans(self.rot, pos_now), self.q)
        if q_now is None:
            return False
        self.q = q_now.copy()
        return self.set_robot()

    def move_self(self, distance: float, axis: str) -> bool:
        idx = {"x": 0, "y": 1, "z": 2}.get(axis)
        if idx is None:
            return False
        I4 = np.eye(4, dtype=float)
        I4[idx, 3] = distance
        trans_now = self.trans @ I4
        q_now = self.ikine_near(trans_now, self.q)
        if q_now is None:
            return False
        self.q = q_now.copy()
        return self.set_robot()

    def rotate_world(self, angle: float, axis: str) -> bool:
        if axis == "x":
            rot_new = self.rot_x(angle) @ self.rot
        elif axis == "y":
            rot_new = self.rot_y(angle) @ self.rot
        elif axis == "z":
            rot_new = self.rot_z(angle) @ self.rot
        else:
            return False
        trans_now = self.rotpos2trans(rot_new, self.pos)
        q_now = self.ikine_near(trans_now, self.q)
        if q_now is None:
            return False
        self.q = q_now.copy()
        return self.set_robot()

    def rotate_self(self, angle: float, axis: str) -> bool:
        if axis == "x":
            rot_new = self.rot @ self.rot_x(angle)
        elif axis == "y":
            rot_new = self.rot @ self.rot_y(angle)
        elif axis == "z":
            rot_new = self.rot @ self.rot_z(angle)
        else:
            return False
        trans_now = self.rotpos2trans(rot_new, self.pos)
        q_now = self.ikine_near(trans_now, self.q)
        if q_now is None:
            return False
        self.q = q_now.copy()
        return self.set_robot()

    # ------------------------------------------------------------------
    # Inverse kinematics
    # ------------------------------------------------------------------
    def ikine8(self, T6_0: ArrayLike) -> List[np.ndarray]:
        T = np.asarray(T6_0, dtype=float).copy()
        if T.shape != (4, 4):
            raise ValueError("T6_0 must have shape (4, 4)")
        T = T @ np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, -self.d6], [0.0, 0.0, 0.0, 1.0]], dtype=float)
        T[2, 3] -= self.d1

        nx, ny, nz = T[0, 0], T[1, 0], T[2, 0]
        ox, oy, oz = T[0, 1], T[1, 1], T[2, 1]
        ax, ay, az = T[0, 2], T[1, 2], T[2, 2]
        px, py, pz = T[0, 3], T[1, 3], T[2, 3]

        t = Tree(0.0)
        t0 = 0
        q1 = [math.atan2(py, px), math.atan2(-py, -px)]
        t1 = [t.addNode(t0, q1[0]), t.addNode(t0, q1[1])]

        t3 = [0] * 4
        for i in range(2):
            s1 = t.node[t1[i]].sin
            c1 = t.node[t1[i]].cos
            denom = 2 * self.a2
            k3 = (px * px * c1 * c1 - py * py * c1 * c1 + self.a1 * self.a1 + py * py + pz * pz - 2 * self.a1 * px * c1 - 2 * self.a1 * py * s1 + 2 * px * py * c1 * s1 - self.a2 * self.a2 - self.a3 * self.a3 - self.d4 * self.d4) / denom
            q3_arg = k3 / math.sqrt(self.a3 * self.a3 + self.d4 * self.d4)
            q3 = [self._asin_like_csharp(q3_arg) - math.atan2(self.a3, self.d4), math.pi - self._asin_like_csharp(q3_arg) - math.atan2(self.a3, self.d4)]
            t3[i] = t.addNode(t1[i], q3[0])
            t3[i + 2] = t.addNode(t1[i], q3[1])

        t2 = [0] * 4
        for i in range(4):
            s3 = t.node[t3[i]].sin
            c3 = t.node[t3[i]].cos
            s1 = t.node[t.getParent(t3[i])].sin
            c1 = t.node[t.getParent(t3[i])].cos
            e2 = self.a2 + self.d4 * s3 + self.a3 * c3
            f2 = self.d4 * c3 - self.a3 * s3
            g2 = px * c1 - self.a1 + py * s1
            q2 = math.atan2(f2 * g2 - e2 * pz, g2 * e2 + f2 * pz)
            t2[i] = t.addNode(t3[i], q2)

        t5 = [0] * 8
        for i in range(4):
            s2 = t.node[t2[i]].sin
            c2 = t.node[t2[i]].cos
            s3 = t.node[t.getParent(t2[i])].sin
            c3 = t.node[t.getParent(t2[i])].cos
            s1 = t.node[t.getParent(t2[i], 2)].sin
            c1 = t.node[t.getParent(t2[i], 2)].cos
            q5_arg = az * c2 * c3 + ax * c1 * c2 * s3 + ax * c1 * c3 * s2 + ay * c2 * s1 * s3 + ay * c3 * s1 * s2 - az * s2 * s3
            q5_base = self._acos_like_csharp(q5_arg)
            q5 = [q5_base, -q5_base]
            t5[i] = t.addNode(t2[i], q5[0])
            t5[i + 4] = t.addNode(t2[i], q5[1])

        t4 = [0] * 8
        t6 = [0] * 8
        for i in range(8):
            q5_ = t.node[t5[i]].q
            s5 = t.node[t5[i]].sin
            s2 = t.node[t.getParent(t5[i], 1)].sin
            c2 = t.node[t.getParent(t5[i], 1)].cos
            s3 = t.node[t.getParent(t5[i], 2)].sin
            c3 = t.node[t.getParent(t5[i], 2)].cos
            s1 = t.node[t.getParent(t5[i], 3)].sin
            c1 = t.node[t.getParent(t5[i], 3)].cos

            if q5_ == 0:
                c46 = nx * c1 * c2 * c3 - nz * c3 * s2 - nz * c2 * s3 + ny * c2 * c3 * s1 - nx * c1 * s2 * s3 - ny * s1 * s2 * s3
                s46 = ny * c1 - nx * s1
                q46 = math.atan2(s46, c46)
                q4 = math.pi / 2
                q6 = q46 - q4
            else:
                e4 = ax * c1 * c2 * c3 - az * c3 * s2 - az * c2 * s3 + ay * c2 * c3 * s1 - ax * c1 * s2 * s3 - ay * s1 * s2 * s3
                f4 = ay * c1 - ax * s1
                k6 = oz * c2 * c3 + ox * c1 * c2 * s3 + ox * c1 * c3 * s2 + oy * c2 * s1 * s3 + oy * c3 * s1 * s2 - oz * s2 * s3
                r6 = nz * s2 * s3 - nz * c2 * c3 - nx * c1 * c2 * s3 - nx * c1 * c3 * s2 - ny * c2 * s1 * s3 - ny * c3 * s1 * s2
                q4 = math.atan2(f4 / s5, e4 / s5)
                q6 = math.atan2(k6 / s5, r6 / s5)
            t4[i] = t.addNode(t5[i], q4)
            t6[i] = t.addNode(t4[i], q6)

        Q = np.zeros((8, 6), dtype=float)
        for i in range(8):
            Q[i, 0] = self.angle_clip_pnpi(t.node[t.getParent(t6[i], 5)].q)
            Q[i, 1] = self.angle_clip_pnpi(t.node[t.getParent(t6[i], 3)].q)
            Q[i, 2] = self.angle_clip_pnpi(t.node[t.getParent(t6[i], 4)].q)
            Q[i, 3] = self.angle_clip_pnpi(t.node[t.getParent(t6[i], 1)].q)
            Q[i, 4] = self.angle_clip_pnpi(t.node[t.getParent(t6[i], 2)].q)
            Q[i, 5] = self.angle_clip_pnpi(t.node[t6[i]].q)

        return [Q[i].copy() for i in range(8) if not np.isnan(np.sum(Q[i]))]

    def ikine_near(self, T6_0: ArrayLike, q0: ArrayLike) -> Optional[np.ndarray]:
        QS = self.ikine8(T6_0)
        if len(QS) == 0:
            return None
        q0_arr = np.asarray(q0, dtype=float).reshape(6)
        min_norm = math.inf
        min_index = 0
        for i, q in enumerate(QS):
            q_work = q.copy()
            if q_work[4] == 0:
                q46 = q_work[3] + q_work[5]
                q_work[3] = q0_arr[3]
                q_work[5] = q46 - q_work[3]
                QS[i] = q_work
            minor_arc_norm = np.linalg.norm(self.minor_arc(q_work, q0_arr))
            if minor_arc_norm < min_norm:
                min_norm = minor_arc_norm
                min_index = i
        return QS[min_index].copy()

    @staticmethod
    def _asin_like_csharp(x: float) -> float:
        # C# Math.Asin returns NaN for out-of-domain values instead of raising.
        return math.asin(x) if -1.0 <= x <= 1.0 else math.nan

    @staticmethod
    def _acos_like_csharp(x: float) -> float:
        # C# Math.Acos returns NaN for out-of-domain values instead of raising.
        return math.acos(x) if -1.0 <= x <= 1.0 else math.nan

    # ------------------------------------------------------------------
    # Static math helpers
    # ------------------------------------------------------------------
    @staticmethod
    def angle_clip_pnpi(q: Union[float, ArrayLike]) -> Union[float, np.ndarray]:
        arr = np.asarray(q, dtype=float)
        scalar = arr.ndim == 0
        out = arr.astype(float, copy=True)
        invalid = ~np.isfinite(out)
        out = ((out + math.pi) % (2 * math.pi)) - math.pi
        # C# range is (-pi, pi], so map -pi to +pi.
        out = np.where(np.isclose(out, -math.pi), math.pi, out)
        out = np.where(invalid, np.nan, out)
        return float(out) if scalar else out

    @staticmethod
    def angle_clip_02pi(q: Union[float, ArrayLike]) -> Union[float, np.ndarray]:
        arr = np.asarray(q, dtype=float)
        scalar = arr.ndim == 0
        out = arr.astype(float, copy=True)
        out = out % (2 * math.pi)
        return float(out) if scalar else out

    @staticmethod
    def minor_arc(angle1: Union[float, ArrayLike], angle2: Union[float, ArrayLike]) -> Union[float, np.ndarray]:
        return np.abs(Robot.minor_arc_dir(angle1, angle2))

    @staticmethod
    def minor_arc_dir(start: Union[float, ArrayLike], target: Union[float, ArrayLike]) -> Union[float, np.ndarray]:
        start_arr = np.asarray(start, dtype=float)
        target_arr = np.asarray(target, dtype=float)
        delta = Robot.angle_clip_pnpi(2 * math.pi - (start_arr - target_arr))
        if np.asarray(delta).ndim == 0:
            return float(delta)
        return delta

    @staticmethod
    def rotpos2trans(Rot: ArrayLike, Pos: ArrayLike) -> np.ndarray:
        R = np.asarray(Rot, dtype=float)
        P = np.asarray(Pos, dtype=float).reshape(3)
        if R.shape != (3, 3):
            raise ValueError("Rot must have shape (3, 3)")
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = P
        return T

    @staticmethod
    def trans2rot(T: ArrayLike) -> np.ndarray:
        T_arr = np.asarray(T, dtype=float)
        return T_arr[:3, :3].copy()

    @staticmethod
    def trans2pos(T: ArrayLike) -> np.ndarray:
        T_arr = np.asarray(T, dtype=float)
        return T_arr[:3, 3].copy()

    @staticmethod
    def rpy2rot(RPY: ArrayLike) -> np.ndarray:
        a, b, c = np.asarray(RPY, dtype=float).reshape(3)
        sinA, cosA = math.sin(a), math.cos(a)
        sinB, cosB = math.sin(b), math.cos(b)
        sinC, cosC = math.sin(c), math.cos(c)
        return np.array([
            [cosB * cosC, cosC * sinA * sinB - cosA * sinC, sinA * sinC + cosA * cosC * sinB],
            [cosB * sinC, cosA * cosC + sinA * sinB * sinC, cosA * sinB * sinC - cosC * sinA],
            [-sinB, cosB * sinA, cosA * cosB],
        ], dtype=float)

    @staticmethod
    def rot_x(a: float) -> np.ndarray:
        return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]], dtype=float)

    @staticmethod
    def rot_y(b: float) -> np.ndarray:
        return np.array([[math.cos(b), 0.0, math.sin(b)], [0.0, 1.0, 0.0], [-math.sin(b), 0.0, math.cos(b)]], dtype=float)

    @staticmethod
    def rot_z(c: float) -> np.ndarray:
        return np.array([[math.cos(c), -math.sin(c), 0.0], [math.sin(c), math.cos(c), 0.0], [0.0, 0.0, 1.0]], dtype=float)

    @staticmethod
    def rot2rpy(R: ArrayLike) -> np.ndarray:
        R_arr = np.asarray(R, dtype=float)
        if R_arr.shape == (4, 4):
            R_arr = R_arr[:3, :3]
        if abs(R_arr[2, 0] - 1.0) < 1.0e-15:
            a = 0.0
            b = -math.pi / 2.0
            c = math.atan2(-R_arr[0, 1], -R_arr[0, 2])
        elif abs(R_arr[2, 0] + 1.0) < 1.0e-15:
            a = 0.0
            b = math.pi / 2.0
            c = -math.atan2(R_arr[0, 1], R_arr[0, 2])
        else:
            a = math.atan2(R_arr[2, 1], R_arr[2, 2])
            c = math.atan2(R_arr[1, 0], R_arr[0, 0])
            cosC = math.cos(c)
            sinC = math.sin(c)
            if abs(cosC) > abs(sinC):
                b = math.atan2(-R_arr[2, 0], R_arr[0, 0] / cosC)
            else:
                b = math.atan2(-R_arr[2, 0], R_arr[1, 0] / sinC)
        return np.array([a, b, c], dtype=float)

    @staticmethod
    def islegal(a: float) -> bool:
        return not (math.isinf(a) or math.isnan(a))
