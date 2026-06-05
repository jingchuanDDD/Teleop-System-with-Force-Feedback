# 双飞特舵机遥操测试记录

日期：2026-06-03

## 当前硬件拓扑

当前使用两块调试板，两条串口总线：

```text
COM17 / ID2 = leader 电机
COM16 / ID1 = follower 电机
baudrate = 1000000
model = sms_sts
```

配置文件：

```text
leader_follower_config.json
```

关键配置：

```json
{
  "leader_port": "COM17",
  "leader_id": 2,
  "follower_port": "COM16",
  "follower_id": 1,
  "leader_center": 2048,
  "follower_center": 2047,
  "direction": 1,
  "gain": 1.0
}
```

## 基础检查命令

### 查看双端口状态

```powershell
python leader_follower.py status
```

用途：

- 从 COM17 读取 leader 位置
- 从 COM16 读取 follower 位置和速度
- 根据标定 center 计算 follower 目标位置

典型输出：

```text
leader ID2 pos=2048 center=2048 offset=0
follower ID1 pos=1986 speed=0 center=2047 target=2047 range=[0, 4096]
```

### 读取 follower PID

```powershell
python leader_follower.py pid-read
```

用途：

- 读取 follower ID1 的位置环 PID 参数
- 当前使用的是 follower 端口 COM16

当前推荐值：

```text
P=36 D=24 I=0
```

### 写 follower PID

```powershell
python leader_follower.py pid-write --p 36 --d 24 --i 0
```

用途：

- 修改 follower 的 PID 参数
- 写入前会打印旧值和新值
- 脚本会 unlock EPROM，写入后重新 lock

恢复默认值：

```powershell
python leader_follower.py pid-write --p 32 --d 32 --i 0
```

## 遥操运行命令

## 重要修正：位置模式下 speed=0 的含义

后续测试发现，STS/SMS 位置模式下：

```text
speed=0 不是停止
speed=0 更像“不限速 / 使用最快速度”
```

此前使用：

```text
speed=800
```

时，follower 实际上被限制了运行速度。快速拖动 leader 时出现的大延迟，很大一部分来自 follower 被这个非零速度参数限速。

将遥操命令改为：

```text
speed=0
```

后，快速拖动 leader 时之前明显的 follower 延迟基本不再出现。

因此当前遥操推荐优先使用：

```powershell
--speed 0 --acc 255
```

只有在需要故意降低 follower 速度、减小冲击或限制运动速度时，才使用非零 `speed` 值。

### 稳定版遥操命令

```powershell
python leader_follower.py follow --period 0 --write-interval 0.01 --filter-alpha 1.0 --deadband 1 --speed 0 --acc 255 --report-interval 1 --timing --ignore-write-errors
```

用途：

- 高频读取 leader
- 最多每 10 ms 给 follower 写一次目标
- 不使用 `tx-only`，会等待 follower 写应答
- 若偶发写应答超时，记录错误并继续运行

适用场景：

- 调试阶段
- 希望写命令更可确认
- 对极限延迟不敏感

### 低延迟版遥操命令

```powershell
python leader_follower.py follow --period 0 --write-interval 0.01 --filter-alpha 1.0 --deadband 1 --speed 0 --acc 255 --tx-only --report-interval 1 --timing
```

用途：

- 只发送 follower 目标位置，不等待写应答
- 降低通信等待时间
- 控制循环更轻

适用场景：

- 追求最低延迟
- 已确认总线和命令较稳定

### 参数说明

```text
--period 0
```

不主动 sleep，尽量高频读取 leader。实测比 `--period 0.01` 更跟手。

```text
--write-interval 0.01
```

限制 follower 目标写入频率为最多 100 Hz。该参数不是目标速度限制，只是避免过密写命令。

```text
--filter-alpha 1.0
```

关闭软件滤波。leader 读到什么位置，就直接映射成 follower 目标。

```text
--deadband 1
```

目标变化超过 1 step 才重新发命令。死区很小，灵敏度高。

```text
--speed 0
```

follower 位置模式运行速度限制。当前实验发现 `speed=0` 在该模式下并不是停止，而更像“不限速/最快速度”。`speed=800` 反而会限制 follower 速度。

```text
--acc 255
```

follower 加速度参数。`sms_sts` SDK 中该字段为 1 字节，合法范围为 `0..255`。

```text
--tx-only
```

只发命令，不等待 follower 状态应答。低延迟，但调试信息更少。

```text
--timing
```

打印控制循环频率、读 leader 耗时、写 follower 耗时、读写错误计数。

## Leader 速度监控

```powershell
python leader_follower.py monitor-leader --period 0.01 --report-interval 0.2
```

用途：

- 只监控 leader ID2
- 根据位置差分估算手拖速度
- 将速度换算成 RPM

换算关系：

```text
4096 steps = 1 revolution
rpm = step/s * 60 / 4096
```

复现 follower 跟不上时曾测得：

```text
leader_v ~= 2533 step/s ~= 37.1 rpm
```

STS3215-C018 空载最高速度约为 `45 rpm`，因此该手拖速度已经接近电机空载极限。

## 自动 Sweep 测试

### 基础 sweep-test

```powershell
python leader_follower.py sweep-test --start 1300 --end 2800 --rate 300 --period 0 --write-interval 0.01 --speed 0 --acc 255 --tx-only --duration 5 --report-interval 1
```

用途：

- 不依赖手拖 leader
- 直接让 follower 跟随程序生成的目标轨迹
- 用于比较 PID、速度、加速度、写入频率等参数

核心输出：

```text
avg_abs_err
max_abs_err
read_errors
```

含义：

```text
avg_abs_err = 平均绝对跟踪误差
max_abs_err = 最大跟踪误差
read_errors = follower 状态读取错误次数
```

## PID 实验结果（历史 speed=800 条件）

以下 PID 实验是在历史参数 `speed=800` 下完成的。由于后续确认 `speed=0` 在位置模式下更像“不限速/最快速度”，这些结果主要用于记录当时的调参过程；若后续继续优化 PID，建议在 `speed=0` 条件下重新测试。

测试条件：

```text
sweep-test 1300..2800
rate=300
speed=800
acc=255
tx-only
duration=5
```

结果：

```text
P=32 D=32 I=0
avg_abs_err=144.8
max_abs_err=323
```

```text
P=32 D=40 I=0
avg_abs_err=217.9
max_abs_err=395
```

```text
P=32 D=24 I=0
avg_abs_err=55.3
max_abs_err=118
```

```text
P=28 D=24 I=0
avg_abs_err=132.8
max_abs_err=292
```

```text
P=36 D=24 I=0
avg_abs_err=43.7
max_abs_err=85
```

```text
P=40 D=24 I=0
avg_abs_err=122.6
max_abs_err=203
```

当前保留的 follower PID：

```text
P=36 D=24 I=0
```

注意：上述 PID 实验是在 `speed=800` 条件下完成的。后续发现位置模式下 `speed=0` 更像“不限速/最快速度”，快速拖动 leader 时延迟显著减少。因此，后续遥操推荐优先使用 `speed=0`，必要时再重新做 PID/sweep 对比。

结论：

- 简单增大 P 不一定改善跟随
- 增大 D 到 40 明显变差
- 降低 D 到 24 后效果明显改善
- 当前最佳小步测试结果是 `P=36 D=24 I=0`

## 写入频率和控制周期实验（历史 speed=800 条件）

以下写入频率实验同样是在 `speed=800` 条件下完成，主要用于比较控制周期和写入节奏对误差的影响。当前实际遥操建议仍优先使用 `speed=0`。

测试条件：

```text
sweep-test 1300..2800
rate=300
speed=800
acc=255
tx-only
```

结果：

```text
period=0, write-interval=0
avg_abs_err=28.7
max_abs_err=47
read_errors=105
```

```text
period=0, write-interval=0.01
avg_abs_err=27.5
max_abs_err=37
read_errors=111
```

```text
period=0, write-interval=0.02
avg_abs_err=27.1
max_abs_err=37
read_errors=90
```

```text
period=0.01, write-interval=0.02
avg_abs_err=192.5
max_abs_err=350
read_errors=4
```

结论：

- `period=0` 下 follower 跟随效果最好
- Windows 下 `period=0.01` 实际调度不够稳定，会明显增加跟踪误差
- 高频读取 follower 状态会产生较多 `read_errors`，但遥操控制本身不依赖高频读取 follower
- 日常遥操建议降低报告频率，减少 follower 状态读取干扰

## 当前阶段结论

低速拖动 leader 时：

```text
follower 基本无明显延迟
```

快速拖动 leader 时：

```text
follower 会出现明显滞后
```

原因：

- leader 快速运动时，目标位置变化速度接近或超过 follower 可实现动态响应
- follower 受最大速度、加速度、负载惯量、供电能力和内部 PID 限制
- 软件优化可以降低通信和调度延迟，但不能突破执行器物理极限

## 当前推荐配置

PID：

```text
P=36 D=24 I=0
```

稳定遥操：

```powershell
python leader_follower.py follow --period 0 --write-interval 0.01 --filter-alpha 1.0 --deadband 1 --speed 0 --acc 255 --report-interval 1 --timing --ignore-write-errors
```

低延迟遥操：

```powershell
python leader_follower.py follow --period 0 --write-interval 0.01 --filter-alpha 1.0 --deadband 1 --speed 0 --acc 255 --tx-only --report-interval 1 --timing
```

## 后续方向：Impedance Control 和力反馈遥操

当前系统仍是纯位置遥操：

```text
leader position -> follower target position
```

后续如果考虑 impedance control 和力反馈，需要进一步设计：

### 1. follower 侧阻抗控制

目标不再只是硬跟随位置，而是模拟弹簧-阻尼系统：

```text
tau_cmd = K * position_error + B * velocity_error
```

需要关注：

- 舵机是否支持电流/扭矩控制模式
- 当前 SDK 能否稳定读 current/load
- 写入目标是位置、速度、PWM，还是 torque/current

### 2. leader 侧力反馈

将 follower 遇到的负载或位置误差反馈到 leader：

```text
follower load/current/position_error -> leader torque/阻尼/反作用力
```

需要解决：

- leader 不能完全关闭扭矩，否则无法提供力反馈
- leader 输出力矩要有限幅，避免夹手或冲击
- 需要稳定双向通信循环
- 需要明确力反馈量的单位和缩放

### 3. 需要新增的数据读取

建议后续增加命令读取：

```text
当前负载 address 60
当前电压 address 62
当前温度 address 63
当前电流 address 69
当前位置 address 56
当前速度 address 58
```

这些量可以帮助判断：

- follower 是否接近力矩/电流限制
- 是否出现电压跌落
- 负载变化能否作为力反馈信号

### 4. 安全策略

力反馈遥操必须加入：

- 力矩/电流限幅
- 速度限幅
- 位置限幅
- 通信超时卸力
- 急停
- 温度、电压、电流保护检查

当前阶段建议先保持位置遥操稳定，再逐步加入负载/电流反馈观测，最后再进入闭环力反馈控制。
