# 挑战 7：PID 反馈控制与数值模拟

**代码定位：** [`Project/APP_FUNC/pid.c`](../../Project/APP_FUNC/pid.c#L72) 的 `PID_Calc`、[`pid.c`](../../Project/APP_FUNC/pid.c#L172) 的 `Motor_Calc`、[`pid.c`](../../Project/APP_FUNC/pid.c#L243) 的遥控映射；任务周期见 [`Project/APP/main.c`](../../Project/APP/main.c#L145)。

## 1. 闭环控制对象

姿态环的输入是遥控期望角度，输出是四个 ESC 的 PWM。以滚转为例，简化对象可写为

\[
I_x\ddot\phi=\tau_\phi+d_\phi,
\]

其中 `dφ` 包含气流、机架不对称和电机扰动。陀螺仪给出角速度 `p`，姿态解算给出角度 `φ`。只用角度 P 会在快速扰动时反应慢，只用角速度又没有绝对姿态参考，所以仓库采用“角度外环 + 角速度内环”的串级结构。

## 2. 连续 PID 和离散实现

连续 PID：

\[
u(t)=K_p e(t)+K_i\int_0^t e(\tau)d\tau+K_d\frac{de(t)}{dt}.
\]

代码使用参数化形式 `Ki=Kp/Ti`、`Kd=Kp·Td`，并以采样周期 `Ts` 离散化：

\[
u_k=K_p\left[e_k+\frac{T_s}{T_i}\sum_{j=0}^ke_j
 +\frac{T_d}{T_s}(e_k-e_{k-1})\right].
\]

前向矩形积分和后向差分简单、适合 Cortex-M4，但对采样抖动敏感；`pidT` 必须是本次实际周期而不是编译时常量。

## 3. `PID_Calc` 逐段映射

滚转/俯仰分支在 `Project/APP_FUNC/pid.c`：

```c
coreKi = pidT / core->Ti;
coreKd = core->Td / pidT;
shell->eK = shellErr;
shell->output = shell->Kp * shell->eK;
core->eK = shell->output - coreStatus;
```

外环只使用角度误差 `shellErr` 的比例项，输出目标角速度；`coreStatus` 是 `fGyro` 转换后的 deg/s，内环误差为“目标角速度 − 实测角速度”。内环随后计算积分、比例和差分并限幅：

```c
core->eSum += core->eK;
core->output = core->Kp *
    (core->eK + core->eSum * coreKi
     + (core->eK - core->eK_1) * coreKd);
core->output = Limit(core->output, -PID_OUT_MAX, PID_OUT_MAX);
```

`core->eSum` 没有乘 `pidT`，但乘积 `eSum*coreKi` 中的 `coreKi=pidT/Ti` 已补上积分时间；当 `pidT` 为秒时，量纲是正确的。若 `pidT==0` 或异常大，会发生除零/巨大微分，必须先做 `0.0001 < pidT < pidT_max` 检查。

当前增益初始化为：roll/pitch 外环 `Kp=4.4`，内环 `Kp=2.6, Ti=0.5, Td=0.08`；它们依赖角度和角速度的单位，换成弧度后不能原样复用。

若把电机混控和机体在工作点附近线性化，单轴对象约为 `G(s)=1/(Ix·s²)`；内环 PID 后的闭环特征多项式由 `Kp、Ti、Td、Ix` 和执行器延迟共同决定。串级设计通常先让内环带宽达到外环的 3–5 倍，再把内环看成一个较慢的一阶对象。这个频带分离比单独调三个角度 P 增益更能解释“为什么会振荡”。

`PID_Time_Init` 把 TIM4 预分频为 80，在 84 MHz 定时器时钟下计数约 1.05 MHz，而 `Get_PID_Time` 却固定除以 `1000000.0f`。因此函数返回的时间比真实时间**大约 5%**（例如真实 0.002 s 会报告 0.0021 s）；积分系数偏大约 5%，微分系数偏小约 4.8%。应使用由时钟寄存器计算出的实际 tick 频率，或把 PSC 改为 83 后再除以 1e6。该误差会同时改变积分和微分增益。

## 4. 积分饱和和微分陷阱

`pid.h` 定义 `CORE_INT_MAX=4000`、`PID_OUT_MAX=500`。代码通过限制 `eSum` 方向来做基础 anti-windup：输出已经饱和且误差仍把输出推向饱和时不再积分。但它没有检查“输出是否饱和”，只检查积分和；二者并不等价。更可靠的条件是：

```c
bool drives_further = (u_raw > UMAX && e > 0) ||
                      (u_raw < UMIN && e < 0);
if (!drives_further) integral += e * Ts;
```

微分项对设定值阶跃会产生 derivative kick。内环使用测量角速度时问题较小；若给外环加 D，应对测量值微分并加一阶滤波。停止、失控或倾覆保护触发时应清零 `eSum` 和 `eK_1`，否则重新解锁会带着旧积分冲击电机。

## 5. 从遥控器到电机的完整链路

`Motor_Exp_Calc` 把 TIM5 捕获的 1000–2000 µs 脉宽限制后映射为：

```c
expRoll  = (PWMInCh4 - 1500) * 0.03f;
expPitch = (PWMInCh2 - 1500) * 0.04f;
expYaw   = (PWMInCh1 - 1500) * 0.02f;
expMode  = PWMInCh3;
```

`Motor_Calc` 再调用 roll/pitch PID，并用混控矩阵叠加到 `expMode`。注意偏航计算行当前被注释：

```c
// pidYaw = PID_Calc(0, fGyro.z * RAD_TO_ANGLE, 0, &yawCore);
```

因此 `pidYaw` 仍是全局变量的初值（通常为 0），偏航杆不会产生闭环力矩。若启用，应明确 `expYaw` 是期望偏航角还是期望角速度；当前函数的 yaw 分支内部使用 `expYaw - coreStatus`，更像角速度目标。

电机输出先限幅到 1000–2000，再在 `pwm.c` 乘 `PWM_IN_TO_OUT=0.05427` 转为 TIM3 CCR。限幅发生在每台电机独立通道，会改变总推力和姿态力矩；高级混控应优先保持姿态差分，再整体平移/缩放油门（desaturation）。

还要区分“控制周期”和“传感器新数据周期”：PID 任务每 3 个 OS tick 运行一次，但姿态任务每 1 tick，故 PID 可能重复使用同一姿态样本，也可能在 I²C 阻塞后跳过样本。将 `angle`、`fGyro` 和时间戳打包成同一快照，再在 PID 任务中消费，可避免读到不同采样时刻的混合状态。

## 6. 飞行模式状态机

`Judge_FlyMode` 定义 STOP、DOWN、HOVER、UP 四态，阈值为 1050、1350、1650。状态转换带有一定迟滞，减少临界点抖动；但 `Motor_Calc` 中调用它的代码被整体注释，当前实际控制始终直接使用 `expMode`。若恢复高度控制，必须先初始化 `MS5611`、验证 `height` 单位（源码注释在 cm/m 间不一致），再将 `pidThr` 安全地加入混控。

## 7. 数值仿真模型

最小姿态仿真可以只保留滚转：

```python
phi, rate = 0.0, 0.0
integ, e_prev = 0.0, 0.0
for k in range(N):
    e_outer = ref - phi
    rate_ref = Kps * e_outer
    e_inner = rate_ref - rate
    integ += e_inner * Ts
    u = Kp * (e_inner + Ki * integ
              + Kd * (e_inner - e_prev) / Ts)
    u = clip(u, -500, 500)
    rate += (u / Ix) * Ts
    phi += rate * Ts
    e_prev = e_inner
```

更真实的仿真应加入四电机混控、`Ti=kf·ωi²`、电机一阶滞后、PWM 饱和、陀螺偏置、姿态解算延迟和任务周期抖动。对阶跃响应计算上升时间、超调量、2% 调节时间、稳态误差和饱和占比；对外部脉冲力矩计算恢复时间。先固定 `Ts=1 ms` 验证算法，再将 `Get_PID_Time()` 记录的实际周期回放。

## 8. 调参顺序和验收

1. 无桨、锁定油门，验证角度和角速度正负号。
2. 关闭积分和微分，只调内环 P，直到快速但不振荡。
3. 增加内环 D 抑制超调，再逐步加入 I 消除恒定偏差。
4. 调外环 P，使期望角度响应不过度激进；外环带宽应明显低于内环。
5. 最后启用偏航和高度，并测试遥控失联、倾覆、传感器断线时的安全输出。

建议把每次 PID 计算的 `eK`、`eSum`、未限幅输出 `u_raw`、限幅后输出 `u` 和四路电机命令都记录下来。若 `u_raw` 长时间超限，说明油门工作点、混控比例或电机能力不足，继续增大增益只会加剧积分饱和。

每次改变参数都应记录编译版本、`pidT` 分布、PWM 饱和情况和串口姿态数据。只看“能飞”不能证明闭环稳定，必须用可重复的阶跃/扰动数据说明相位裕度和鲁棒性趋势。

## 9. 串级 PID 的闭环推导

把滚转角记为 `φ`、机体滚转角速度记为 `p`，在小角度、悬停工作点附近可近似为

\[
\dot\phi=p,\qquad I_x\dot p=K_\tau u+d.
\]

外环把角度误差 `e_s=φ_ref-φ` 变成角速度参考 `p_ref=K_{ps}e_s`；内环再用 `e_c=p_ref-p` 产生电机差分 `u`。若内环带宽远高于外环，可把 `p≈p_ref` 代回，外环近似一阶：

\[
\frac{\phi(s)}{\phi_{ref}(s)}\approx\frac{K_{ps}}{s+K_{ps}}.
\]

这解释了“先调内环，再调外环”的工程顺序。若两个环频率接近，外环会把内环相位延迟放大，表现为慢振荡，即使每个单独 PID 在静态测试中都没有发散。

源码 `PID_Calc()` 的串级关系可按信号流逐项标注：

```c
shell->eK = shellErr;                 // 角度误差，单位 deg
shell->output = shell->Kp * shell->eK; // 目标角速度，deg/s
core->eK = shell->output - coreStatus; // 内环误差，deg/s
```

因此 `rollShellKp=4.4` 的量纲近似是 `(deg/s)/deg=1/s`，而 `rollCoreKp=2.6` 把 deg/s 误差映射为 PWM 微秒。把姿态改成弧度后必须按量纲重新标定，不能只把输入乘 `PI/180`。

## 10. 当前参数的一次采样计算

假设首次有效 PID 周期 `pidT=0.003 s`、滚转角误差 `10°`、实测角速度为 0，且历史误差/积分均为 0：

```text
shell.output = 4.4 * 10 = 44 deg/s
core.eK      = 44 deg/s
coreKi       = 0.003 / 0.5 = 0.006
coreKd       = 0.08 / 0.003 ≈ 26.67
```

若误差从 0 突然跳到 44，微分项约为 `44*26.67=1173`，乘内环 `Kp=2.6` 后原始输出约 `3172`，立即被 `PID_OUT_MAX=500` 截断。这个算例不等于参数一定错误（真实遥控阶跃可能经过死区和滤波），但说明必须记录 `u_raw`，否则只看到 500 无法判断是增益过大、周期错误还是执行器能力不足。启动/解锁时还应把 `eK_1` 初始化为当前误差，避免人为阶跃产生微分冲击。

## 11. 更稳健的离散实现模板

推荐把“计算未限幅输出、判断饱和、更新积分、滤波微分”分开。下面模板使用测量微分，避免设定值阶跃的 derivative kick：

```c
float pid_step(PidState *s, float ref, float y, float dt)
{
    if (!(dt > 1.0e-4f && dt < 0.02f)) return s->last;
    float e = ref - y;
    float d_raw = -(y - s->y_prev) / dt;
    float a = dt / (s->tau_d + dt);
    s->d_filt += a * (d_raw - s->d_filt);

    float i_trial = s->integral + e * dt;
    float u_raw = s->kp*e + s->ki*i_trial + s->kd*s->d_filt;
    bool high = (u_raw > s->umax && e > 0.0f);
    bool low  = (u_raw < s->umin && e < 0.0f);
    if (!high && !low) s->integral = i_trial;
    float u = clamp(u_raw, s->umin, s->umax);
    s->y_prev = y; s->last = u;
    return u;
}
```

仓库现有实现采用 `eSum` 保存“误差和”，再乘 `pidT/Ti`；若迁移到上面的积分状态，需把 `Ki` 改成 `Kp/Ti` 并统一单位，不能同时保留两次 `pidT`。停机、传感器失效或倾覆保护触发时调用 `pid_reset()` 清零积分、微分滤波器和历史输出。

## 12. 偏航通道和角度环的语义

`Motor_Exp_Calc()` 把 `expYaw=(PWMInCh1-1500)*0.02`，注释意图是 deg/s；`Motor_Calc()` 中本应调用的行却被注释：

```c
// pidYaw = PID_Calc(0, fGyro.z * RAD_TO_ANGLE, 0, &yawCore);
```

即使取消注释，也要明确 `PID_Calc` 的 yaw 分支内部使用全局 `expYaw-coreStatus`，它是角速度误差而不是 yaw 角误差。若希望“摇杆控制角速度”，应把 yaw 设定值限制为 deg/s、只使用速率 PID；若希望“摇杆控制航向角”，则必须积分设定角、处理 ±180° wrap，并用磁力计/AHRS 的 yaw 作为反馈。两种模式的积分器、失控行为和调参方法不同，不能混在一个变量名里。

## 13. 混控饱和、任务周期和可重复仿真

四个电机分别 `Limit()` 会改变分配矩阵。建议在记录中同时保存 `base、pidRoll、pidPitch、pidYaw、u_raw[4]、u_sat[4]`，计算饱和占比；饱和超过约 10% 的时间时，继续增大 PID 只会降低相位裕度。更好的做法是先对姿态差分做共同缩放，再加回油门，并在油门过低时冻结积分。

`Task_PID` 每 `OSTimeDly(3)` 运行一次，即名义 3 ms；I²C 和高优先级任务会造成抖动。仿真应先用固定 `dt=0.003` 验证控制律，再把实测 `Get_PID_Time()` 序列逐步回放。最小单轴仿真可加入执行器一阶滞后和测量延迟：

```python
for k, dt in enumerate(dt_log):
    e_s = ref - phi
    rate_ref = kp_shell * e_s
    e_c = rate_ref - rate_meas_delayed[k]
    integ += e_c * dt
    d = (e_c - e_prev) / max(dt, 1e-5)
    u_raw = kp_core * (e_c + integ/Ti + Td*d)
    u = np.clip(u_raw, -500, 500)
    motor += (motor_cmd(u) - motor) * dt / tau_m
    rate += (motor * k_tau / Ix) * dt
    phi += rate * dt
    e_prev = e_c
```

对阶跃输入报告上升时间、超调、2% 调节时间、稳态误差和饱和比例；对外部脉冲力矩报告恢复时间。只有这些指标在不同电池电压、温度和周期抖动下都可重复，才足以说明 PID 调参有效。
