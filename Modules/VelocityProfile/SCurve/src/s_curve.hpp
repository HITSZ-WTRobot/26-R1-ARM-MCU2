/**
 * @file    s_curve.hpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#ifndef S_CURVE_HPP
#define S_CURVE_HPP
#include <cstdint>
#include "IVelocityProfile.hpp"

// 最大二分查找误差
#ifndef S_CURVE_MAX_BS_ERROR
#    define S_CURVE_MAX_BS_ERROR (0.001f)
#endif

namespace velocity_profile
{

class SCurveProfile final : public IVelocityProfile
{
public:
    struct Config
    {
        float max_spd;
        float max_acc;
        float max_jerk;
    };

    SCurveProfile(const Config& cfg, float xs, float vs, float as, float xe);

    [[nodiscard]] float CalcX(float t) const override;
    [[nodiscard]] float CalcV(float t) const override;
    [[nodiscard]] float CalcA(float t) const override;
    [[nodiscard]] float getTotalTime() const override
    {
        return total_time_;
    }
    [[nodiscard]] bool success() const override
    {
        return success_;
    }

private:
    class SCurveAccel
    {
    public:
        SCurveAccel();
        void init(float vs, float vp, float am, float jm);

        [[nodiscard]] float getDistance(float t) const;
        [[nodiscard]] float getVelocity(float t) const;
        [[nodiscard]] float getAcceleration(float t) const;
        [[nodiscard]] float getTotalDistance() const
        {
            return total_distance_;
        }
        [[nodiscard]] float getTotalTime() const
        {
            return total_time_;
        }

    private:
        bool  has_uniform_; ///< 是否有匀加速段
        float vs_;
        float jm_;

        float total_time_;
        float total_distance_;

        float t1_; ///< 加加速段与匀加速段时刻分界
        float x1_; ///< 加加速段与匀加速段距离分界
        float v1_; ///< 加加速段与匀加速段速度分界
        float t2_; ///< 匀加速段与减加速段时刻分界
        float x2_; ///< 匀加速段与减加速段距离分界
        float v2_; ///< 匀加速段与减加速段速度分界

        float ap_;
        float vp_;
    };

    bool success_;

    bool  has_const_; ///< 是否有匀速段
    float direction_; ///< 运行方向
    float vp_;        ///< 最大速度
    float vs_;        ///< 初始速度
    float as_;        ///< 初始加速度
    float jm_;        ///< 最大加加速度

    // 可能的加速度刹车过程
    float t0_;
    float x0_;

    float xs_; ///< 初始位置
    float x1_; ///< 加速与匀速过程位置分界
    float x2_; ///< 匀速与减速过程位置分界
    float xe_; ///< 末位置

    SCurveAccel process1_{};
    float       ts1_; ///< 第一段非对称过程的时间偏移
    float       xs1_; ///< 第一段非对称过程的起始位置
    float       t1_;  ///< 加速与匀速过程时刻分界

    float t2_; ///< 匀速与减速过程时刻分界

    SCurveAccel process3_{};

    float total_time_;

#ifdef DEBUG
    uint32_t binary_search_count_;
#endif
};

} // namespace velocity_profile
#endif // S_CURVE_HPP
