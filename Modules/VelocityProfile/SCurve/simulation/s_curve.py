# -*- coding: utf-8 -*-

"""
本文件是通过 AI 将 s_curve.c 转换得来
--------------------------------------------------------------------------
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
import math


class SCurveAccel:
    def __init__(self):
        self.has_uniform = False
        self.vs = 0.0
        self.jm = 0.0

        self.total_time = 0.0
        self.total_distance = 0.0

        self.t1 = 0.0
        self.x1 = 0.0
        self.v1 = 0.0
        self.t2 = 0.0
        self.x2 = 0.0
        self.v2 = 0.0

        self.ap = 0.0
        self.vp = 0.0

    def init(self, vs, vp, am, jm):
        self.has_uniform = jm * (vp - vs) > am * am
        self.vs = vs
        self.vp = vp
        self.jm = jm

        if self.has_uniform:
            self.ap = am

            self.t1 = am / jm
            self.t2 = (vp - vs) / am
            dt2 = self.t2 - self.t1

            self.v1 = vs + 0.5 * am * self.t1
            self.v2 = vp - 0.5 * am * self.t1

            self.x1 = vs * self.t1 + (1 / 6.0) * am * self.t1 * self.t1
            self.x2 = self.x1 + self.v1 * dt2 + 0.5 * am * dt2 * dt2

            self.total_time = self.t2 + self.t1
            self.total_distance = ((vp * vp - vs * vs) / (2.0 * am) +
                                   0.5 * (vs + vp) * self.t1)
        else:
            self.ap = math.sqrt(jm * (vp - vs))

            self.t1 = self.ap / jm
            self.t2 = self.t1

            self.v1 = vs + 0.5 * self.ap * self.ap / jm
            self.v2 = self.v1

            self.x1 = vs * self.t1 + (1 / 6.0) * self.ap * self.t1 * self.t1
            self.x2 = self.x1

            self.total_time = 2.0 * math.sqrt((vp - vs) / jm)
            self.total_distance = (vs + vp) * math.sqrt((vp - vs) / jm)

    def get_distance(self, t):
        if t <= 0:
            return 0
        if t < self.t1:
            return self.vs * t + (1 / 6.0) * self.jm * t * t * t
        if self.has_uniform and t < self.t2:
            _t = t - self.t1
            return self.x1 + self.v1 * _t + 0.5 * self.ap * _t * _t
        if t < self.total_time:
            _t = self.total_time - t
            return self.total_distance - self.vp * _t + (1 / 6.0) * self.jm * _t * _t * _t
        return self.total_distance

    def get_velocity(self, t):
        if t <= 0:
            return self.vs
        if t < self.t1:
            return self.vs + 0.5 * self.jm * t * t
        if self.has_uniform and t < self.t2:
            return self.v1 + self.ap * (t - self.t1)
        if t < self.total_time:
            _t = self.total_time - t
            return self.vp - 0.5 * self.jm * _t * _t
        return self.vp

    def get_acceleration(self, t):
        if t <= 0:
            return 0
        if t < self.t1:
            return self.jm * t
        if self.has_uniform and t < self.t2:
            return self.ap
        if t < self.total_time:
            return self.jm * (self.total_time - t)
        return 0

    def __repr__(self):
        return (f"SCurveAccel(\n"
                f"    t1: {self.t1}, t2: {self.t2}, tot_t: {self.total_time},\n"
                f"    x1: {self.x1}, x2: {self.x2}, tot_x: {self.total_distance},\n"
                f"    vs: {self.vs}, v1: {self.v1}, v2: {self.v2}, vp: {self.vp},\n"
                f"    ap: {self.ap}\n"
                f")")


class SCurve:
    S_CURVE_FAILED = 0
    S_CURVE_SUCCESS = 1

    def __init__(self):
        self.has_const = False
        self.direction = 1.0
        self.vp = 0.0
        self.vs = 0.0
        self.as_ = 0.0
        self.jm = 0.0

        self.t0 = 0.0
        self.x0 = 0.0

        self.xs = 0.0
        self.x1 = 0.0
        self.x2 = 0.0
        self.xe = 0.0

        self.process1 = SCurveAccel()
        self.ts1 = 0.0
        self.xs1 = 0.0
        self.t1 = 0.0

        self.t2 = 0.0
        self.process3 = SCurveAccel()

        self.total_time = 0.0

    def __repr__(self):
        return (f"SCurve(\n"
                f"    has_const: {self.has_const}, dir: {self.direction},\n"
                f"    vp: {self.vp},"
                f"    xs: {self.xs}, x1: {self.x1}, x2: {self.x2}, xe: {self.xe},\n"
                f"    ts1: {self.ts1}, t1: {self.t1}, t2: {self.t2}, tot: {self.total_time},\n"
                f"    proc1: {self.process1},\n"
                f"    proc3: {self.process3},\n"
                f")")

    def init(self, xs, xe, vs, as_, vm, am, jm):
        vm = abs(vm)
        am = abs(am)
        jm = abs(jm)

        dir_ = 1.0 if xe > xs else -1.0
        self.direction = dir_
        self.xs = xs
        self.xe = xe
        length = abs(xe - xs)
        vs = vs * dir_
        as_ = as_ * dir_

        self.vs = vs
        self.as_ = as_
        self.jm = jm

        if abs(vs) > vm or abs(as_) > am:
            return SCurve.S_CURVE_FAILED

        vp = vm
        vs0 = None
        dx0 = None
        vp_min = None
        vs1 = None
        ts1 = None

        if as_ < 0:
            vs0 = vs - 0.5 * as_ * as_ / jm
            if abs(vs0) > vm:
                return SCurve.S_CURVE_FAILED
            vp_min = vs0

            ts0 = -as_ / jm
            self.t0 = ts0

            dx0 = vs * ts0 + (1 / 3.0) * as_ * ts0 * ts0
            self.x0 = xs + dir_ * dx0
            vs1 = vs0
            ts1 = 0
        else:
            vp_min = vs + 0.5 * as_ * as_ / jm
            vs0 = vs
            dx0 = 0
            self.t0 = 0
            self.x0 = xs

            if vm < vp_min:
                return SCurve.S_CURVE_FAILED

            ts1 = as_ / jm
            vs1 = vs - 0.5 * as_ * ts1
        vp_min = max(0, vp_min)
        len0 = length - dx0

        self.process1.init(vs1, vp, am, jm)
        self.process3.init(0, vp, am, jm)
        self.xs1 = self.process1.get_distance(ts1)
        dx1 = self.process1.total_distance - self.xs1
        dx3 = self.process3.total_distance
        x_const = len0 - dx1 - dx3

        if x_const > 0:
            self.has_const = True
            self.ts1 = ts1
            self.t1 = self.t0 + self.process1.total_time - ts1

            t_const = x_const / vm
            self.t2 = self.t1 + t_const
            self.total_time = self.t2 + self.process3.total_time

            self.x1 = self.x0 + self.direction * dx1
            self.x2 = self.x1 + self.direction * x_const
            self.vp = vm
            return SCurve.S_CURVE_SUCCESS

        l = vp_min
        r = vm
        delta_d = len0

        cnt = 0

        while (r - vp_min) > 0.001:
            cnt += 1
            mid = 0.5 * (l + r)
            self.process1.init(vs1, mid, am, jm)
            self.process3.init(0, mid, am, jm)
            dx1 = self.process1.total_distance - self.process1.get_distance(ts1)
            dx3 = self.process3.total_distance
            delta_d = dx1 + dx3 - len0
            if -0.001 < delta_d < 0.001:
                r = l = mid
                break
            if delta_d > 0:
                r = mid
            else:
                l = mid

        print(cnt)

        if delta_d > 0.001:
            return SCurve.S_CURVE_FAILED

        self.has_const = False
        self.ts1 = ts1
        self.t1 = self.t0 + self.process1.total_time - ts1
        self.t2 = self.t1
        self.total_time = self.t2 + self.process3.total_time

        self.x1 = self.x0 + self.direction * dx1
        self.x2 = self.x1
        self.vp = r

        return SCurve.S_CURVE_SUCCESS

    def calc_x(self, t):
        if t <= 0:
            return self.xs
        if t < self.t0:
            t2 = t * t
            t3 = t2 * t
            return self.xs + self.direction * (self.vs * t +
                                               0.5 * self.as_ * t2 +
                                               (1 / 6.0) * self.jm * t3)
        if t < self.t1:
            return self.x0 + self.direction * (self.process1.get_distance(t + self.ts1 - self.t0) - self.xs1)
        if self.has_const and t < self.t2:
            return self.x1 + self.direction * self.vp * (t - self.t1)
        if t < self.total_time:
            return self.xe - self.direction * self.process3.get_distance(self.total_time - t)
        return self.xe

    def calc_v(self, t):
        if t <= 0:
            return self.direction * self.vs
        if t < self.t0:
            return self.direction * (self.vs + self.as_ * t + 0.5 * self.jm * t * t)
        if t < self.t1:
            return self.direction * self.process1.get_velocity(t + self.ts1 - self.t0)
        if self.has_const and t < self.t2:
            return self.direction * self.vp
        if t < self.total_time:
            return self.direction * self.process3.get_velocity(self.total_time - t)
        return 0.0

    def calc_a(self, t):
        if t <= 0:
            return self.direction * self.as_
        if t < self.t0:
            return self.direction * (self.as_ + self.jm * t)
        if t < self.t1:
            return self.direction * self.process1.get_acceleration(t + self.ts1 - self.t0)
        if self.has_const and t < self.t2:
            return 0.0
        if t < self.total_time:
            return -self.direction * self.process3.get_acceleration(self.total_time - t)
        return 0.0
