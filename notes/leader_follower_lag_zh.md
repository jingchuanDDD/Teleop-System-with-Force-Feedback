# Leader/Follower 跟随滞后问题记录

日期：2026-06-03

## 实验设置

- 电机型号：飞特 STS3215 / ST-3215-C018
- 当前拓扑：双调试板
- Leader：COM17 / ID2
- Follower：COM16 / ID1
- 波特率：1000000
- 供电：12 V
- 控制方式：读取 leader 当前位置，映射成 follower 目标位置，再下发给 follower

标定文件中位近似设置为：

```text
leader_center = 2048
follower_center = 2047
direction = 1
gain = 1.0
```

## 问题现象

早期测试中，快速拖动 leader 摆杆时，follower 摆杆会明显滞后。

典型日志：

```text
leader=1921 offset=-127 leader_v=-2533step/s 37.1rpm target=1920 follower=2619 err=-699 speed=-950
```

含义：

- leader 当前速度约为 `2533 step/s`，约 `37.1 rpm`
- follower 目标位置已经计算到 `1920`
- follower 实际位置仍在 `2619`
- follower 落后 `699 steps`，约 `61.4 deg`
- follower 速度反馈为负，说明它正在朝正确方向追赶，但没有及时追上目标

## 原因分析

遥操流程是：

```text
读取 leader 位置 -> 计算 follower 目标 -> 下发 follower 目标位置
```

因此 follower 不是瞬间复制 leader，而是通过内部位置环 PID 去追一个不断变化的目标。

follower 的跟随能力受以下因素限制：

- 电机最大转速
- 加速度限制
- 输出扭矩
- 摆杆负载和转动惯量
- 齿轮/机构摩擦
- 供电电压和电流能力
- 舵机内部 PID 响应

STS3215-C018 在 12 V 下空载最高转速约为 `45 rpm`。早期复现跟不上时，leader 实测速度约为：

```text
37.1 rpm
```

这已经达到空载最高转速的约：

```text
37.1 / 45 ~= 82%
```

所以如果 follower 带负载运行、需要加速/刹车/反向，并经过内部 PID 控制，它的稳定跟随速度会低于理论空载最高速度。

## 重要修正：位置模式下 speed=0

后续测试发现，我们之前对 `speed` 的理解有偏差。

在 STS/SMS 位置模式下，当前这套系统表现为：

```text
speed=0 不是停止
speed=0 更像“不限速 / 使用最快速度”
```

此前使用：

```text
speed=800
```

时，follower 实际上受到了速度限制。将 follower 位置模式速度改为：

```text
speed=0
```

后，之前快速拖动 leader 时明显出现的 follower 大延迟基本不再出现。

因此当前结论需要修正为：

```text
早期 follower 大滞后，一部分来自 leader 运动过快；
另一部分来自 follower 被非零 speed 参数限速。
当前位置模式优先使用 speed=0。
```

## 当前推荐遥操命令

低延迟版本：

```powershell
python leader_follower.py follow --period 0 --write-interval 0.01 --filter-alpha 1.0 --deadband 1 --speed 0 --acc 255 --tx-only --report-interval 1
```

更稳的非 tx-only 版本：

```powershell
python leader_follower.py follow --period 0 --write-interval 0.01 --filter-alpha 1.0 --deadband 1 --speed 0 --acc 255 --report-interval 1 --timing --ignore-write-errors
```

参数含义：

- `--speed 0`：位置模式下不限速/最快
- `--acc 255`：当前 SDK 中合法最大加速度
- `--filter-alpha 1.0`：不做软件滤波
- `--deadband 1`：小死区，高灵敏度
- `--period 0`：不主动 sleep，尽量高频读取 leader
- `--write-interval 0.01`：最多每 10 ms 给 follower 写一次目标

## 历史 Sweep Test 数据（speed=800 条件）

以下数据是在发现 `speed=0` 修正之前记录的，属于历史调参数据。测试条件是显式使用非零速度限制 `speed=800/1200/1500`。

自动 follower 轨迹测试，范围 `1300..2800`，目标变化速率 `300 step/s`，`acc=255`，`tx-only`：

```text
speed=800
avg_abs_err=47.1
max_abs_err=87
```

```text
speed=1200
avg_abs_err=57.8
max_abs_err=105
```

```text
speed=1500
avg_abs_err=84.9
max_abs_err=157
```

目标变化速率提高后：

```text
rate=600, speed=1500, acc=250
avg_abs_err=320.8
max_abs_err=664
```

这些结果说明，在非零速度限制条件下，目标轨迹变化越快，follower 越容易产生误差。但后续 `speed=0` 测试表明，早期大滞后不能完全归因于电机物理极限，还包括速度限制参数造成的影响。

## Leader 速度测量方法

使用：

```powershell
python leader_follower.py monitor-leader --period 0.01 --report-interval 0.2
```

根据 leader 位置差分估算速度：

```text
4096 steps = 1 revolution
rpm = step/s * 60 / 4096
```

早期复现跟不上时测得：

```text
max = 37.1 rpm
```

这说明短摆杆被快速拨动时，leader 很容易达到较高角速度。

## 当前结论

当前阶段结论应分两层看：

1. 软件和通信链路  
   双调试板、`period=0`、小死区、不滤波后，低速遥操基本无明显延迟。

2. follower 动态能力  
   快速拖动 leader 时，如果 follower 目标变化超过其实际动态能力，仍然会出现滞后。但在位置模式下应优先使用 `speed=0`，避免人为限速。

修正后的核心结论：

```text
speed=0 是当前 STS/SMS 位置模式下的推荐高速设置；
非零 speed 应理解为速度限制，只有需要降低速度或减小冲击时才使用。
```

## 后续建议

- 遥操默认使用 `speed=0`
- 需要安全限速时，再使用非零 `speed`
- 保持 `acc=255`，除非动作冲击过大
- 保持 `filter-alpha=1.0` 和 `deadband=1`
- PID 当前可使用 `P=36 D=24 I=0`
- 继续关注供电电流能力和摆杆惯量
- 后续做 impedance control / 力反馈前，需要进一步读取电流、负载、电压、温度等状态量
