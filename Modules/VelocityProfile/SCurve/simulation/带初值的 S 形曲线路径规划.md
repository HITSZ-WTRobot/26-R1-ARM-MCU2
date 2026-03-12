# 带初值的 S 形曲线路径规划

## 问题描述

现在我要在从位置 $x_s$ 到 $x_e$ 之间构造一条 $x(t)$ 曲线，保证：位置，速度，加速度均连续。
并且有如下限制：速度最大值为 $v_m$；加速度最大值为 $a_m$；加加速度最大值为 $j_m$
在以下边界值条件下求使得总用时最小的曲线函数：初速度 $v_s$, 初加速度 $a_s$，末速度 $0$，末加速度 $0$

## 分析

在整个分析过程中我们假定 $x_s < x_e$

先考虑 $a_s=0$ 的情况，当 $\Delta x$ 和 $v_m$ 足够大时，曲线为七段

- 加加速度加速 $j=j_m$
- 匀加速 $a=a_m$
- 减加速度加速 $j=-j_m$
- 匀速 $v=v_m$
- 加加速度减速 $j=-j_m$
- 匀减速 $a=-a_m$
- 减加速度减速 $j=j_m$

此时整体为对称过程（先忽略 $v_s$）

故可以定义一个加速过程求解器 `SCurveAccel_t`，初参数 $v_s, v_p, v_m, a_m, j_m$

现分析加速过程。

> **情况一**：$v_p - v_s$ 较大，此时该过程有三段
>
> 1. $a \uparrow$（分析其正过程）: $a = j_mt, v=v_s+\frac12j_mt^2,\Delta x=v_st+\frac16j_mt^3$
>
>    可以得到
>    $$
>    \begin{cases}
>    t_1  &=\frac{a_m}{j_m}\\
>    v_1  &=v_s + \frac12j_mt_1=v_s+\frac12\frac{a_m^2}{j_m}\\
>    \Delta x_1 &=\frac{v_sa_m}{j_m}+\frac16\frac{a_m^3}{j_m^2}
>    \end{cases}
>    $$
>    
>
> 3. $a \downarrow$（分析其逆过程）: $a=-j_mt_-,v=v_p-\frac12j_mt_-^2,\Delta x=v_pt_--\frac16j_mt_-^3$
>
>    可以得到
>    $$
>    \begin{cases}
>    t_3&=\frac{a_m}{j_m}\\
>    v_2&=v_p - \frac12j_mt_3=v_p-\frac12\frac{a_m^2}{j_m}\\
>    v_3&=vp\\
>    \Delta x_1&=\frac{v_sa_m}{j_m}+\frac16\frac{a_m^3}{j_m^2}
>    \end{cases}
>    $$
>
> 易得到匀加速段的时间是
> $$
> t_2 = \frac{v_2-v_1}{a_m} = \frac{v_p-v_s}{a_m}-\frac{a_m}{j_m}
> $$
> 所以很容易得到临界条件 $t_2>0$，即
> $$
> j_m(v_p-v_s)>a_m^2
> $$
> 总位移和总时间
> $$
> \begin{cases}
> t_{total}&=\frac{v_p-v_s}{a_m}+\frac{a_m}{j_m}\\
> \Delta x_{total}&=\frac{v_p^2-v_s^2}{2a_m} + \frac{(v_p+v_s)a_m}{2j_m}
> \end{cases}
> $$
> **情况二**：$v_p-v_s$ 较小，此时只有两段，设最大加速度为 $a_p$
>
> 1. $a \uparrow$（分析其正过程）：$a = j_mt, v=v_s+\frac12j_mt^2,\Delta x=v_st+\frac16j_mt^3$
>    $$
>    \begin{cases}
>    t_1  &=\frac{a_p}{j_m}\\
>    v_1  &=v_s + \frac12j_mt_1=v_s+\frac12\frac{a_p^2}{j_m}\\
>    \Delta x_1 &=\frac{v_sa_p}{j_m}+\frac16\frac{a_p^3}{j_m^2}
>    \end{cases}
>    $$
>
> 2. 略过
>
> 能发现：1, 2 过程对称，得到
> $$
> \begin{cases}
> a_p &=\sqrt{j_m(v_p-v_s)}\\
> t_{total} &=2\sqrt{\frac{v_p-v_s}{j_m}}\\
> x_{total} &= (v_s+v_p)\sqrt{\frac{v_p-v_s}{j_m}}
> \end{cases}
> $$

实现如下

```c
typedef struct
{
    bool  has_uniform;
    float vs;
    float jm;

    float total_time;
    float total_distance;

    float t1; ///< 加加速段与匀加速段时刻分界
    float x1; ///< 加加速段与匀加速段距离分界
    float v1; ///< 加加速段与匀加速段速度分界
    float t2; ///< 匀加速段与减加速段时刻分界
    float x2; ///< 匀加速段与减加速段距离分界
    float v2; ///< 匀加速段与减加速段速度分界

    float ap;
    float vp;
} SCurveAccel_t;

static void SCurveAccel_Init(
        SCurveAccel_t* s, const float vs, const float vp, const float am, const float jm)
{
    s->has_uniform = jm * (vp - vs) > am * am;
    s->vs          = vs;
    s->vp          = vp;
    s->jm          = jm;
    if (s->has_uniform)
    {
        s->ap = am;

        s->t1           = am / jm;
        s->t2           = (vp - vs) / am;
        const float dt2 = s->t2 - s->t1;

        s->v1 = vs + 0.5f * am * s->t1;
        s->v2 = vp - 0.5f * am * s->t1;

        s->x1 = vs * s->t1 + 1 / 6.0f * am * s->t1 * s->t1;
        s->x2 = s->x1 + s->v1 * dt2 + 0.5f * am * dt2 * dt2;

        s->total_time     = s->t2 + s->t1;
        s->total_distance = (vp * vp - vs * vs) / (2.0f * am) +
                            0.5f * (vs + vp) * s->t1 /*(vs + vp) * am / (2.0f * jm)*/;
    }
    else
    {
        s->ap = sqrtf(jm * (vp - vs));

        s->t1 = s->ap / jm;
        s->t2 = s->t1; // 没有匀加速段

        s->v1 = vs + 0.5f * s->ap * s->ap / jm;
        s->v2 = s->v1;

        s->x1 = vs * s->t1 + 1 / 6.0f * s->ap * s->t1 * s->t1;
        s->x2 = s->x1;

        s->total_time     = 2.0f * sqrtf((vp - vs) / jm);
        s->total_distance = (vs + vp) * sqrtf((vp - vs) / jm);
    }
};

static float SCurveAccel_GetDistance(const SCurveAccel_t* s, const float t)
{
    if (t <= 0)
    {
        return 0;
    }
    if (t < s->t1)
    {
        return s->vs * t + 1 / 6.0f * s->jm * t * t * t;
    }
    // 除了这段以外两种情况都一样
    if (s->has_uniform && t < s->t2)
    {
        const float _t = t - s->t1;
        return s->x1 + s->v1 * _t + 0.5f * s->ap * _t * _t;
    }
    if (t < s->total_time)
    {
        const float _t = s->total_time - t;
        return s->total_distance - s->vp * _t + 1 / 6.0f * s->jm * _t * _t * _t;
    }
    return s->total_distance;
}

static float SCurveAccel_GetVelocity(const SCurveAccel_t* s, const float t)
{
    if (t <= 0)
    {
        return s->vs;
    }
    if (t < s->t1)
    {
        return s->vs + 0.5f * s->jm * t * t;
    }
    if (s->has_uniform && t < s->t2)
    {
        return s->v1 + s->ap * (t - s->t1);
    }
    if (t < s->total_time)
    {
        const float _t = s->total_time - t;
        return s->vp - 0.5f * s->jm * _t * _t;
    }
    return s->vp;
}

static float SCurveAccel_GetAcceleration(const SCurveAccel_t* s, const float t)
{
    if (t <= 0)
        return 0;
    if (t < s->t1)
        return s->jm * t;
    if (s->has_uniform && t < s->t2)
        return s->ap;
    if (t < s->total_time)
        return s->jm * (s->total_time - t);
    return 0;
}
```

（一）如果 $a_s=0$，则可以变为三（或两）个过程

```
SCurveAccel_Init(&p1, vs, vp, am, jm);
// 匀速段，如果有
SCurveAccel_Init(&p3, 0, vp, am, jm); // 通过逆过程分析
```

其中 $v_p$ 是过程最大速度，如果有匀速段，则 $v_p=v_m$。没有匀速段的情况放到后面讨论。

（二）如果 $a_s>0$，即与运行方向相同，则可以构造一个带偏移的加速过程 `SCurveAccel_t`，

因为 $a_s$ 这个加速度值必然出现在  `SCurveAccel_t` 的第一段（或者第一段与第二段临界点），所以可以有
$$
\begin{array}{left}
a &= j_mt_s &= a_s\\
v &= v_s' + \frac12 j_mt_s^2 &= v_s
\end{array}
$$
得到构造的过程
$$
\begin{array}{left}
t_s&=\frac{a_s}{j_m}\\
v_s'&=v_s-\frac12\frac{a_s^2}{j_m}
\end{array}
$$
从 $t_s$ 之后的过程正好是初加速度 $a_s$ 之后的过程

（三）如果 $a_s<0$，即与运行方向相反，则开头必然存在一个给加速度刹车的过程，而之后就是一个新的初加速度为零的 S 曲线过程



现在开始分析没有匀速段的情况，这种情况必然存在一个最大速度 $v_p$ 使得匀速段时间恰好等于零。

经过 MATLAB 简单分析发现其数学表达式过于负载，所以在在代码里运行一个精度达到要求的二分查找来找到合适的 $v_p$

```c

    // 由于代数表达式太过复杂，这里直接上二分查找
    float l = vp_min, r = vm;
    float delta_d = len0, dx1 = 0, dx3 = 0;
    // 最大迭代次数约 13 次（假定 vm < 5)
    while (r - l > 0.001f)
    {
        const float mid = 0.5f * (l + r);
        SCurveAccel_Init(&s->process1, vs1, mid, am, jm);
        SCurveAccel_Init(&s->process3, 0, mid, am, jm);
        // 更新，用于判断是否找到解
        dx1     = s->process1.total_distance - SCurveAccel_GetDistance(&s->process1, ts1);
        dx3     = s->process3.total_distance;
        delta_d = dx1 + dx3 - len0;
        if (delta_d < 0.001 && delta_d > -0.001)
        {
            r = l = mid;
            break;
        }
        if (delta_d > 0)
            r = mid;
        else
            l = mid;
    }
    if (delta_d > 0.001)
    {
        // 即使 vp 降到最低也无法找到解
        return S_CURVE_FAILED;
    }
```

将 $v_m$ 替换为查找到的 $v_p$ 再重复上面分析即可 