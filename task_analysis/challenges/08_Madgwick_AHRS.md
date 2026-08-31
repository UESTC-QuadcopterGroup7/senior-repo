# 挑战 8：AHRS（Madgwick）姿态解算

**代码定位：** [`Project/APP_FUNC/IMU.c`](../../Project/APP_FUNC/IMU.c#L251) 的现有 Mahony 风格更新；[`Project/APP/main.c`](../../Project/APP/main.c#L121) 的调用频率；计时器实现见 [`IMU.c`](../../Project/APP_FUNC/IMU.c#L499)。

## 1. AHRS 的任务和输入

AHRS（Attitude and Heading Reference System）要从三轴陀螺仪、加速度计和磁力计估计姿态。陀螺仪短期平滑但会漂移；加速度计在没有线加速度时给出重力方向，只能约束 roll/pitch；磁力计给出地磁方向，才能长期约束 yaw。Madgwick 算法把陀螺积分作为预测，把重力和地磁方向误差作为优化目标。

仓库的数据链路是 `main.c:Task_Angel` → `GY86_Read(ACC_GYRO_MAG)` → `Attitude_Update(...)`。`GY86_Read` 已把陀螺原始值转换为 `fGyro`（rad/s），但加速度和磁场仍是原始整数；算法内部只使用方向，因此先归一化即可。

## 2. 姿态约束的目标函数

令四元数按 `(q0,q1,q2,q3)=(w,x,y,z)` 排列，且 `|q|=1`。把参考重力方向旋转到机体系，可得到

\[
\hat{\mathbf g}(q)=
\begin{bmatrix}
2(q_1q_3-q_0q_2)\\
2(q_0q_1+q_2q_3)\\
q_0^2-q_1^2-q_2^2+q_3^2
\end{bmatrix}.
\]

归一化加速度观测为 `a=[ax,ay,az]ᵀ`，重力残差可定义为

\[
f_g(q)=\hat{\mathbf g}(q)-\mathbf a.
\]

对 MARG（磁力计辅助）版本，先根据当前 q 把磁场估计到导航系，取水平分量 `bx` 和垂直分量 `bz`，再构造磁场残差 `fm(q)`。总目标可加权为

\[
F(q)=w_a\|f_g(q)\|^2+w_m\|f_m(q)\|^2.
\]

权重不是传感器“可信度常数”：电机振动或线性加速时 `wa` 应降低，附近有电源线或磁体时 `wm` 应降低。

实现 MARG 时通常按以下顺序计算参考磁场，避免把当地磁倾角当成 yaw：

1. 用当前 q 将归一化磁测量 `mᵇ` 旋到导航系，得到 `hⁿ`。
2. 取 `bx=sqrt(hn_x²+hn_y²)`（水平强度）和 `bz=hn_z`（垂直强度）。
3. 用 `bx,bz` 与 q 构造预测磁场 `m_hatᵇ(q)`，以 `m_hatᵇ-mᵇ` 组成残差，并求其 Jacobian。

不同资料会把中间量 `h` 定义成完整旋转结果，或定义成完整结果的一半；两种写法都可以，但后续的 `2*bx`、`2*bz` 必须配套。本文后面的代码采用 `h=R(q)m/2`，因此显式把水平量和垂直量乘 2 后再构造残差。

仓库代码的 `hx/hy/hz → by/bz → wx/wy/wz` 正在完成同一件事，但采用了不同的变量命名和参考轴（注释中的 `bx=0` 也说明作者曾假设无水平分量）。改写时不要只替换最后的 `ex/ey/ez`；必须先确认磁北、地理北、NED/ENU 以及 HMC5883L 轴向，否则 yaw 会固定偏 90° 或反向。

## 3. 梯度下降和四元数微分

对重力目标，上述残差的 Jacobian（列顺序 w,x,y,z）为

\[
J_g=\begin{bmatrix}
-2q_2&2q_3&-2q_0&2q_1\\
2q_1&2q_0&2q_3&2q_2\\
0&-4q_1&-4q_2&0
\end{bmatrix}.
\]

梯度为 `s = Jᵀ f`，归一化后以 β 控制修正强度：

\[
\dot q=\frac12q\otimes(0,\boldsymbol\omega)-\beta\frac{s}{\|s\|},
\qquad q_{k+1}=\operatorname{normalize}(q_k+\dot q\Delta t).
\]

β 的单位近似为 rad/s，越大越快拉回参考方向，但会把加速度噪声传入姿态；越小则漂移消除慢。β 应通过静止噪声、旋转阶跃和振动测试选择，不能直接套用其他板子的数值。

## 4. 与当前 `IMU.c` 的逐段对照

当前 `Attitude_Update` 的归一化和参考向量计算：

```c
norm = invSqrt(ax * ax + ay * ay + az * az);
ax *= norm; ay *= norm; az *= norm;
vx = 2 * (q1q3 - q0q2);
vy = 2 * (q0q1 + q2q3);
vz = q0q0 - q1q1 - q2q2 + q3q3;
```

这正是 `g_hat(q)` 的展开。磁场部分先算 `hx/hy/hz`，再取 `by=sqrt(hx²+hy²)`、`bz=hz`，形成水平/垂直参考，随后求 `wx/wy/wz`。误差用两个叉乘相加：

```c
ex = (ay*vz - az*vy) + (my*wz - mz*wy);
ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
```

叉乘误差是 Mahony 反馈形式的等价几何梯度方向近似。之后 `Kp=2.0f`、`Ki=0.005f` 修正角速度，再用四元数运动方程积分。因此源码注释写“互补滤波”，其实际结构更接近 Mahony PI；它不是严格的 Madgwick 解析 Jacobian 实现。

## 5. 按本项目接口改写的 IMU-only 骨架

下面骨架展示应如何替换校正段，变量仍使用仓库的 q0–q3。磁力计有效时可在 `step_gradient` 中叠加 (J_m^Tf_m)：

```c
void Madgwick_Update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt, float beta)
{
    float n = invSqrt(ax*ax + ay*ay + az*az);
    if (!(n > 0.0f)) return;       // 拒绝零向量/NaN
    ax *= n; ay *= n; az *= n;

    float f0 = 2.0f*(q1*q3 - q0*q2) - ax;
    float f1 = 2.0f*(q0*q1 + q2*q3) - ay;
    /* 使用 |q|=1 约束后的第三个残差，和上面的 J_g 完全一致。 */
    float f2 = 2.0f*(0.5f - q1*q1 - q2*q2) - az;
    float s0 = -2*q2*f0 + 2*q1*f1;
    float s1 =  2*q3*f0 + 2*q0*f1 - 4*q1*f2;
    float s2 = -2*q0*f0 + 2*q3*f1 - 4*q2*f2;
    float s3 =  2*q1*f0 + 2*q2*f1;
    float s2_norm = s0*s0+s1*s1+s2*s2+s3*s3;
    n = s2_norm > 1.0e-12f ? invSqrt(s2_norm) : 0.0f;
    if (n > 0.0f) { s0*=n; s1*=n; s2*=n; s3*=n; }

    float q0d = 0.5f*(-q1*gx-q2*gy-q3*gz) - beta*s0;
    float q1d = 0.5f*( q0*gx+q2*gz-q3*gy) - beta*s1;
    float q2d = 0.5f*( q0*gy-q1*gz+q3*gx) - beta*s2;
    float q3d = 0.5f*( q0*gz+q1*gy-q2*gx) - beta*s3;
    q0 += q0d*dt; q1 += q1d*dt;
    q2 += q2d*dt; q3 += q3d*dt;
    s2_norm = q0*q0+q1*q1+q2*q2+q3*q3;
    n = s2_norm > 1.0e-12f ? invSqrt(s2_norm) : 0.0f;
    if (n > 0.0f) { q0*=n; q1*=n; q2*=n; q3*=n; }
}
```

这段是说明算法结构的参考实现，不应未经测试直接烧录。`dt` 应由 `2*Get_AHRS_Time()` 得到，而不是固定写 `0.001f`。MARG 版本还需在归一化磁场后计算磁梯度，并在磁场异常时跳过该项。

Madgwick 的梯度计算量约为几十次乘加和一次平方根，适合 Cortex-M4；真正昂贵的是浮点 `sqrtf`、`invSqrt` 和磁场分支。可在编译器开启硬件 FPU 的前提下测量 WCET，并确保它小于最短任务周期。若磁力计只有 75 Hz，可每次有新样本时更新 `bx/bz`，中间周期只运行陀螺 + 加速度（IMU-only）分支。

## 6. 当前代码中的关键边界问题

1. `if (ex != 0.0f && ey != 0.0f && ez != 0.0f)` 要求三个分量都非零；只要一个轴误差恰好为零，三个轴的 PI 校正都会被跳过。应改为检查误差范数大于阈值。
2. `invSqrt` 对零或负输入没有保护；I²C 读失败、磁场断线或未初始化数据都可能触发 NaN。
3. `Get_AHRS_Time` 使用 TIM2 16 位计数器，PSC=83 时约 1 MHz，返回计数/2e6 作为半周期。任务被阻塞超过一次计数器回绕时，时间会错误；应处理溢出或使用 32 位自由运行时间戳。
4. `i2cread` 总返回 0，调用方无法区分 ACK 失败；姿态算法可能继续使用旧/垃圾传感器数据。
5. `Quat_Init` 只在水平初始姿态下用磁力计设 yaw，且没有磁硬铁/软铁校准；启动方向和附近磁场会决定初始航向偏差。

## 7. 自适应观测门控

可用模长判断观测是否可信：

```c
float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
float mag_norm = sqrtf(mx*mx + my*my + mz*mz);
float wa = (fabsf(acc_norm - 1.0f) < 0.15f) ? 1.0f : 0.0f;
float wm = (fabsf(mag_norm - mag_reference) < mag_gate) ? 1.0f : 0.0f;
```

门限应根据 `Send_Senser` 记录的均值、标准差和电机转速标定；不要把“1 g”门限用于尚未换算/归一化的 raw 数据。权重改变时保持四元数连续，只改变修正项，不要重置 q。

## 8. 约束梯度的推导与实现边界

四元数有单位球约束 `qᵀq=1`，所以优化不能把四个分量当作完全独立的欧氏参数。Madgwick 先在当前 `q` 处计算残差 `f(q)`，再用 `s=Jᵀf` 得到局部下降方向；最后的四元数归一化相当于把积分结果投影回单位三球面。若不归一化，`q` 的尺度会被误当成姿态，重力预测矩阵也不再正交。

在加速度分支中，为了让 Jacobian 与实现一致，应使用单位约束后的第三个残差

\[
f_3=2(0.5-q_1^2-q_2^2)-a_z,
\]

而不是同时把 `q0²+q1²+q2²+q3²` 当成独立变量。对应的三行 Jacobian 正是文档第 3 节的 `J_g`。代码中 `f0/f1/f2` 为零时，`s` 也应为零；这提供了一个很有用的静止单元测试：`q=(1,0,0,0), a=(0,0,1)` 不应产生任何修正。

如果需要暂时验证磁场分支而不信任手写 Jacobian，可以对固定样本做中心差分（只用于主机测试）：

```c
for (int j = 0; j < 4; ++j) {
    float qp[4], qm[4];
    memcpy(qp, q, sizeof(qp)); memcpy(qm, q, sizeof(qm));
    qp[j] += 1e-5f; qm[j] -= 1e-5f;
    normalize4(qp); normalize4(qm);
    float cp = 0.5f * cost_marg(qp, a, m);
    float cm = 0.5f * cost_marg(qm, a, m);
    grad[j] = (cp - cm) / (2e-5f);
}
```

把数值梯度与解析 `Jᵀf` 比较，最大相对误差应在浮点容差内；通过后再将解析式移植到 Cortex-M4，避免在飞控循环中执行 8 次残差和大量 `sqrtf`。

## 9. MARG 磁场项的代码化步骤

磁场项不能简单把 `mx、my、mz` 再做一次叉乘。按标准 Madgwick MARG 形式，先在当前四元数下求导航系投影：

```c
float hx = mx*(0.5f-q2*q2-q3*q3)
          + my*(q1*q2-q0*q3)
          + mz*(q1*q3+q0*q2);
float hy = mx*(q1*q2+q0*q3)
          + my*(0.5f-q1*q1-q3*q3)
          + mz*(q2*q3-q0*q1);
/* hx/hy 上式采用 0.5 形式，因此这里乘 2 才得到实际水平参考量。 */
float bx = 2.0f * sqrtf(hx*hx + hy*hy);
float bz = 2.0f * (mz*(0.5f-q1*q1-q2*q2)
          + mx*(q1*q3-q0*q2)
          + my*(q2*q3+q0*q1));
```

再构造三个磁场残差（示例使用与重力相同的机体系约定）：

```c
fm0 = 2.0f*bx*(0.5f-q2*q2-q3*q3)
    + 2.0f*bz*(q1*q3-q0*q2) - mx;
fm1 = 2.0f*bx*(q1*q2-q0*q3)
    + 2.0f*bz*(q0*q1+q2*q3) - my;
fm2 = 2.0f*bx*(q0*q2+q1*q3)
    + 2.0f*bz*(0.5f-q1*q1-q2*q2) - mz;
```

生产代码应把 `J_m^T[f_m0,f_m1,f_m2]` 加到重力梯度，或用符号工具生成并离线验证的展开式。`bx`/`bz` 的命名、磁北方向和 NED/ENU 选择必须与 `Quat_Init` 保持一致；代码当前的 `by/bz` 只是同一思想的不同轴命名，不能只凭变量名替换。

## 10. 当前 Mahony 风格实现的量纲检查

`IMU.c:308` 使用

```c
if (ex != 0.0f && ey != 0.0f && ez != 0.0f) { ... }
```

这不是“误差非零”的正确判断。静止时某一个分量恰好为零很常见，结果会跳过另外两个轴的校正。应使用 `ex*ex+ey*ey+ez*ez > ε²`，并分别允许有效/无效的磁场项。

另一个容易被忽略的量纲问题是积分：`halfT=dt/2` 适合四元数微分式，但 PI 积分应为 `exInt += ex*Ki*dt`。源码使用 `ex*Ki*halfT`，所以在 `Ki` 按标准单位调参时，实际积分作用约为预期的一半；若保留现状，报告中必须把 `Ki` 解释为“半周期标定值”。此外，`Kp` 修正的是 rad/s，不能与 PID 的 `Kp`（PWM/deg 或 deg/s）共用。

叉乘为什么可以近似梯度方向？若真实参考向量为 `v`、当前观测为 `a=R(δθ)v`，在小角度下
`a≈v+δθ×v`，于是

\[
a\times v\approx(\delta\theta\times v)\times v
 =-\bigl(I-vv^T\bigr)\delta\theta.
\]

它是把姿态误差投影到观测向量切平面的负梯度。加速度只提供两个独立方向，所以绕重力轴的误差无法由这一项观测；磁场叉乘补足水平航向。该推导说明当前实现虽没有显式写出四元数 Jacobian，仍属于合理的局部几何观测器，但其增益与 Madgwick 的 `β` 不能直接等同。

## 11. 带观测门控的更新框架

建议把“预测、观测权重、梯度修正、积分”明确分开。下面是接口级示例：

```c
void ahrs_step(ImuSample *s)
{
    float dt = s->dt_s;
    if (!(dt > 1e-4f && dt < 0.01f)) return;
    float an = norm3(s->acc);
    float mn = s->mag_valid ? norm3(s->mag) : 0.0f;
    bool use_acc = fabsf(an - 1.0f) < acc_gate_g;
    bool use_mag = s->mag_valid && fabsf(mn-mag_ref) < mag_gate;

    float grad[4] = {0,0,0,0};
    if (use_acc) add_gravity_gradient(q, s->acc, grad);
    if (use_mag) add_magnetic_gradient(q, s->mag, grad);
    float gn = norm4(grad);
    if (gn > 1e-9f) for (int j=0; j<4; ++j) grad[j] /= gn;

    float qdot[4];
    quat_derivative(q, s->gyro_rad_s, qdot);
    for (int j=0; j<4; ++j) q[j] +=
        (qdot[j] - beta*grad[j]) * dt;
    normalize4(q);
}
```

门限应从 `Send_Senser` 的静止均值/标准差和电机振动数据估计；固定写 `0.15` 只是起始值。磁场失真时只关闭 `use_mag`，不要重置四元数，否则会把一次干扰变成姿态跳变。

## 12. 数值算例：β 如何影响收敛

假设当前 q 与真实姿态相差约 10°，归一化梯度方向近似单位向量，采样周期 `dt=0.001 s`。若 `β=0.1 rad/s`，单次修正幅度约 `0.0001 rad=0.0057°`，理论上需要约 1700 个样本（1.7 s）消除误差；`β=1.0` 时速度约快十倍，但静止噪声也会以近似十倍的带宽进入姿态。实际调参应同时测量静止角度 RMS、90° 阶跃上升时间和电机振动下的异常门控比例，而不能只看“收敛更快”。

比较当前 PI 与 Madgwick 时，固定同一 `dt`、初始 q 和归一化传感器数据，记录
`||f_g||、||f_m||、q_norm-1、yaw 漂移、每次更新 CPU 周期`。若只比较最终欧拉角，无法区分是磁场轴向错、β 不合适，还是时间戳错误。

## 13. Cortex-M4 上的实现和验收清单

`invSqrt()` 通过 `long*` 与 `float*` 强制转换，可能违反 C 的严格别名规则；升级编译器或开启更高优化级别时应改用 `memcpy`/CMSIS `arm_sqrt_f32`，并先拒绝 `x<=0`、NaN 和无穷大。MARG 分支应只在 HMC5883L 的 `RDY` 标志产生新样本时运行，其他 1 kHz 周期使用 IMU-only 分支，以免重复磁样本。

最终验收分四层：

1. 主机单元测试：轴角、静止零梯度、四元数范数和解析/数值 Jacobian 一致性；
2. 记录回放：60 s 静止漂移、单轴 ±90°、磁场异常时的门控；
3. 无桨硬件：验证轴向、任务周期、最坏执行时间（WCET）小于最短调度周期；
4. 低风险飞行：逐步增加油门，观察饱和、失联和倾覆保护。

只有当这些测试通过后，才应把 `Attitude_Update` 替换为真正的 Madgwick MARG；目前仓库版本应明确标注为 Mahony 风格 PI 融合，避免算法名称与实际代码不一致。

## 14. 验证顺序

先用离线记录的静止数据验证梯度应把重力残差降到噪声水平；再用已知 ±90° 单轴旋转检查方向和角度；随后加入磁场数据验证 yaw 长期不漂移；最后在无桨状态开启电机，观察振动下的观测门控。比较 Mahony 与 Madgwick 时应使用同一传感器数据、同一真实时间戳和同一初始四元数，并报告 RMS 姿态误差、稳态漂移、CPU 时间和异常样本数。
