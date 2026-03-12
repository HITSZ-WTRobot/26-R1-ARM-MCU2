# -*- coding: utf-8 -*-
from s_curve import SCurve

import numpy as np
import matplotlib.pyplot as plt

def plot_scurve(xs, xe, vs, as_, vm, am, jm, dt=0.001):
    s = SCurve()
    if s.init(xs, xe, vs, as_, vm, am, jm) == s.S_CURVE_FAILED:
        print("S curve init failed")
        return

    total_time = s.total_time

    t = np.arange(0, total_time + dt, dt)

    x = np.array([s.calc_x(ti) for ti in t])
    v = np.array([s.calc_v(ti) for ti in t])
    a = np.array([s.calc_a(ti) for ti in t])

    # x(t)
    plt.figure()
    plt.plot(t, x)
    plt.title("x(t)")
    plt.xlabel("t")
    plt.ylabel("x")
    plt.grid()

    # v(t)
    plt.figure()
    plt.plot(t, v)
    plt.title("v(t)")
    plt.xlabel("t")
    plt.ylabel("v")
    plt.grid()

    # a(t)
    plt.figure()
    plt.plot(t, a)
    plt.title("a(t)")
    plt.xlabel("t")
    plt.ylabel("a")
    plt.grid()

    plt.show()

if __name__ == "__main__":
    plot_scurve(
        xs=0.0,
        xe=3.0,
        vs=0.0,
        as_=0.0,
        vm=5.0,
        am=1.2,
        jm=2.4
    )
