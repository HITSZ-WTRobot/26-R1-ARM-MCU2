/**
 * @file    s_curve.cpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#include "s_curve.hpp"

#include <cmath>

namespace velocity_profile
{
SCurveProfile::SCurveAccel::SCurveAccel() :
    has_uniform_(false), vs_(0), jm_(0), total_time_(0), total_distance_(0), t1_(0), x1_(0), v1_(0),
    t2_(0), x2_(0), v2_(0), ap_(0), vp_(0)
{
}

void SCurveProfile::SCurveAccel::init(const float vs,
                                      const float vp,
                                      const float am,
                                      const float jm)
{
    has_uniform_ = jm * (vp - vs) > am * am;
    vs_          = vs;
    vp_          = vp;
    jm_          = jm;
    if (has_uniform_)
    {
        ap_ = am;

        t1_             = am / jm;
        t2_             = (vp - vs) / am;
        const float dt2 = t2_ - t1_;

        v1_ = vs + 0.5f * am * t1_;
        v2_ = vp - 0.5f * am * t1_;

        x1_ = vs * t1_ + 1 / 6.0f * am * t1_ * t1_;
        x2_ = x1_ + v1_ * dt2 + 0.5f * am * dt2 * dt2;

        total_time_     = t2_ + t1_;
        total_distance_ = (vp * vp - vs * vs) / (2.0f * am) +
                          0.5f * (vs + vp) * t1_ /*(vs + vp) * am / (2.0f * jm)*/;
    }
    else
    {
        ap_ = std::sqrtf(jm * (vp - vs));

        t1_ = ap_ / jm;
        t2_ = t1_; // 没有匀加速段

        v1_ = vs + 0.5f * ap_ * ap_ / jm;
        v2_ = v1_;

        x1_ = vs * t1_ + 1 / 6.0f * ap_ * t1_ * t1_;
        x2_ = x1_;

        total_time_     = 2.0f * sqrtf((vp - vs) / jm);
        total_distance_ = (vs + vp) * sqrtf((vp - vs) / jm);
    }
}
float SCurveProfile::SCurveAccel::getDistance(const float t) const
{
    if (t <= 0)
    {
        return 0;
    }
    if (t < t1_)
    {
        return vs_ * t + 1 / 6.0f * jm_ * t * t * t;
    }
    // 除了这段以外两种情况都一样
    if (has_uniform_ && t < t2_)
    {
        const float _t = t - t1_;
        return x1_ + v1_ * _t + 0.5f * ap_ * _t * _t;
    }
    if (t < total_time_)
    {
        const float _t = total_time_ - t;
        return total_distance_ - vp_ * _t + 1 / 6.0f * jm_ * _t * _t * _t;
    }
    return total_distance_;
}
float SCurveProfile::SCurveAccel::getVelocity(const float t) const
{
    if (t <= 0)
    {
        return vs_;
    }
    if (t < t1_)
    {
        return vs_ + 0.5f * jm_ * t * t;
    }
    if (has_uniform_ && t < t2_)
    {
        return v1_ + ap_ * (t - t1_);
    }
    if (t < total_time_)
    {
        const float _t = total_time_ - t;
        return vp_ - 0.5f * jm_ * _t * _t;
    }
    return vp_;
}
float SCurveProfile::SCurveAccel::getAcceleration(const float t) const
{
    if (t <= 0)
        return 0;
    if (t < t1_)
        return jm_ * t;
    if (has_uniform_ && t < t2_)
        return ap_;
    if (t < total_time_)
        return jm_ * (total_time_ - t);
    return 0;
}

SCurveProfile::SCurveProfile(const Config& cfg, float xs, float vs, float as, float xe)
{
    auto vm = fabsf(cfg.max_spd), am = fabsf(cfg.max_acc), jm = fabsf(cfg.max_jerk);

    // 全部归到正向移动判断
    const float dir = xe > xs ? 1.0f : -1.0f;
    direction_      = dir;
    xs_             = xs;
    xe_             = xe;
    const float len = fabsf(xe - xs);
    vs = vs * dir, as = as * dir;
    vs_ = vs, as_ = as;
    jm_ = jm;

    // 如果目标基本没有变化，则不动
    if (len < 1e-6f)
    {
        t0_         = 0;
        t1_         = 0;
        has_const_  = false;
        t2_         = 0;
        total_time_ = 0;
        success_    = true;
        return;
    }

    if (fabsf(vs) > vm || fabsf(as) > am)
    {
        // 如果初速度或者初加速度超限，则生成失败
        success_ = false;
        return;
    }

    // 首先假定存在匀速段
    const float vp = vm;
    float       vs0, dx0, vp_min;

    float vs1, ts1;

    if (as < 0)
    {
        // 如果初加速度与目标方向相反，则必须先刹到 0
        // 这段位移是必须的

        // 加速度刹到 0 之后的速度，作为之后过程的初速度
        vs0 = vs - 0.5f * as * as / jm;
        // 已经超限则无法构建曲线
        if (fabsf(vs0) > vm)
        {
            success_ = false;
            return;
        }
        vp_min = vs0;

        const float ts0 = -as / jm;
        t0_             = ts0;

        // 加速度刹到 0 的位移
        dx0 = vs * ts0 + 1 / 3.0f * as * ts0 * ts0;
        x0_ = xs + dir * dx0;
        vs1 = vs0; // 之后无须构造偏移
        ts1 = 0;
    }
    else
    {
        // 如果初始带有同方向的加速度，必须预留速度刹车
        vp_min = vs + 0.5f * as * as / jm;
        vs0    = vs;
        dx0    = 0;
        t0_    = 0;
        x0_    = xs;
        // 构造带偏移的梯形加速
        if (vm < vp_min)
        {
            // 即使 vp = vm 也无法构造出带偏移的梯形加速
            success_ = false;
            return;
        }
        ts1 = as / jm;
        vs1 = vs - 0.5f * as * ts1;
    }
    if (vp_min < 0)
        vp_min = 0;
    // 实际需要使用的 len0
    const float len0 = len - dx0;
    {
        process1_.init(vs1, vp, am, jm);
        // 构造逆过程梯形加速
        process3_.init(0, vp, am, jm);
        xs1_                = process1_.getDistance(ts1);
        const float dx1     = process1_.getTotalDistance() - xs1_;
        const float dx3     = process3_.getTotalDistance();
        const float x_const = len0 - dx1 - dx3;
        if (x_const > 0)
        {
            // 存在匀速段
            has_const_ = true;
            ts1_       = ts1;

            // 这里是时刻分界点，所以需要加上开头部分
            t1_ = t0_ + process1_.getTotalTime() - ts1;

            const float t_const = x_const / vm;
            t2_                 = t1_ + t_const;
            total_time_         = t2_ + process3_.getTotalTime();

            x1_ = x0_ + direction_ * dx1;
            x2_ = x1_ + direction_ * x_const;

            vp_ = vm;

            success_ = true;
            return;
        }
    }
    // 不存在匀速段，求最大速度
    // 由于代数表达式太过复杂，这里直接上二分查找
    float l = vp_min, r = vm;
    float delta_d = len0, dx1 = 0, dx3 = 0;

#ifdef DEBUG
    binary_search_count_ = 0;
#endif
    // 最大迭代次数约 13 次
    while (r - vp_min > 0.001f)
    {
#ifdef DEBUG
        binary_search_count_++;
#endif
        const float mid = 0.5f * (l + r);
        process1_.init(vs1, mid, am, jm);
        process3_.init(0, mid, am, jm);
        // 更新，用于判断是否找到解
        dx1     = process1_.getTotalDistance() - process1_.getDistance(ts1);
        dx3     = process3_.getTotalDistance();
        delta_d = dx1 + dx3 - len0;
        if (delta_d < S_CURVE_MAX_BS_ERROR && delta_d > -S_CURVE_MAX_BS_ERROR)
        {
            r = l = mid;
            break;
        }
        if (delta_d > 0)
            r = mid;
        else
            l = mid;
    }
    if (delta_d > S_CURVE_MAX_BS_ERROR)
    {
        // 即使 vp 降到最低也无法找到解
        success_ = false;
        return;
    }

    has_const_ = false;
    ts1_       = ts1;
    // 这里是时刻分界点，所以需要加上开头部分
    t1_         = t0_ + process1_.getTotalTime() - ts1;
    t2_         = t1_;
    total_time_ = t2_ + process3_.getTotalTime();

    x1_ = x0_ + direction_ * dx1;
    x2_ = x1_;

    vp_ = r;

    success_ = true;
}

float SCurveProfile::CalcX(const float t) const
{
    if (!success_)
        return 0;
    // 起始之前
    if (t <= 0)
        return xs_;
    // 反向加速度刹车
    if (t < t0_)
    {
        const float t2 = t * t;
        const float t3 = t2 * t;
        return xs_ + direction_ * (vs_ * t + 0.5f * as_ * t2 + 1 / 6.0f * jm_ * t3);
    }
    // 加速过程
    if (t < t1_)
        return x0_ + direction_ * (process1_.getDistance(t - t0_ + ts1_) - xs1_);
    // 匀速过程
    if (has_const_ && t < t2_)
        return x1_ + direction_ * vp_ * (t - t1_);
    // 减速过程
    if (t < total_time_)
        return xe_ - direction_ * process3_.getDistance(total_time_ - t);
    return xe_;
}

float SCurveProfile::CalcV(const float t) const
{
    if (!success_)
        return 0;
    // 起始之前
    if (t <= 0)
        return direction_ * vs_;
    // 反向加速度刹车
    if (t < t0_)
        return direction_ * (vs_ + as_ * t + 0.5f * jm_ * t * t);
    // 加速过程
    if (t < t1_)
        return direction_ * process1_.getVelocity(t - t0_ + ts1_);
    // 匀速过程
    if (has_const_ && t < t2_)
        return direction_ * vp_;
    // 减速过程
    if (t < total_time_)
        return direction_ * process3_.getVelocity(total_time_ - t);
    return 0;
}

float SCurveProfile::CalcA(const float t) const
{
    if (!success_)
        return 0;
    // 起始之前
    if (t <= 0)
        return direction_ * as_;
    // 反向加速度刹车
    if (t < t0_)
        return direction_ * (as_ + jm_ * t);
    // 加速过程
    if (t < t1_)
        return direction_ * process1_.getAcceleration(t - t0_ + ts1_);
    // 匀速过程
    if (has_const_ && t < t2_)
        return 0;
    // 减速过程
    if (t < total_time_)
        return -direction_ * process3_.getAcceleration(total_time_ - t);
    return 0;
}
} // namespace velocity_profile