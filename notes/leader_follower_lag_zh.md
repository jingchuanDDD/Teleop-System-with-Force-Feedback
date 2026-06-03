# Leader/Follower 跟随滞后问题记录

日期：2026-06-03

## 实验设置

- 电机型号：飞特 STS3215 / ST-3215-C018
- 串口：COM16
- 波特率：1000000
- Leader：ID2
- Follower：ID1
- 供电：12 V
- 控制方式：读取 leader 当前位置，映射成 follower 目标位置，再下发给 follower

当前标定文件中位近似设置为：

```text
leader_center = 2048
follower_center = 2047
direction = 1
gain = 1.0
```

## 问题现象

手动快速拖动 leader 摆杆时，follower 摆杆会明显滞后，不能实时跟上 leader。

典型日志如下：

```text
leader=1921 offset=-127 leader_v=-2533step/s 37.1rpm target=1920 follower=2619 err=-699 speed=-950
```

含义：

- leader 当前速度约为 `2533 step/s`，换算约为 `37.1 rpm`
- 程序已经计算出 follower 目标位置为 `1920`
- follower 实际位置仍在 `2619`
- follower 落后 `699 steps`，约为 `61.4 deg`
- follower 的速度反馈为负值，说明它正在朝正确方向追赶，但速度不足以追上目标

## 原因分析

该问题主要是电机物理性能和内部伺服控制能力限制，不是 Python 控制循环本身的 bug。

当前控制流程是：

```text
读取 leader 位置 -> 计算 follower 目标 -> 下发 follower 目标位置
```

当 leader 被快速拖动时，follower 的目标位置也会快速变化。此时 follower 并不是瞬间复制 leader 的位置，而是通过自身内部 PID 位置闭环去追逐一个不断变化的目标。

follower 的实际跟随能力受以下因素限制：

- 电机最大转速
- 加速度限制
- 输出扭矩
- 摆杆负载和惯量
- 齿轮/机构摩擦
- 供电电压和电流能力
- 舵机内部 PID 响应

STS3215-C018 在 12 V 下空载最高转速约为 `45 rpm`。本次复现跟不上时，leader 实测最大速度约为：

```text
37.1 rpm
```

这已经达到空载最高转速的约：

```text
37.1 / 45 ~= 82%
```

但 follower 并不是空载运行，它需要带动摆杆，并且需要加速、刹车、反向，还要经过内部 PID 位置控制。因此 follower 的稳定可跟随速度会低于理论空载最高速度。

## 速度参数实测结论

飞特教程中说明运行速度字段默认单位为：

```text
50 steps/s = 0.732 rpm
```

并且文档明确提到：

```text
设置超过舵机本体最高速度将响应滞后
```

不过在当前 STS3215-C018 + SDK 控制路径下，不能简单认为速度值越大越好。实测结果表明：

- `speed=60/80/100`：速度明显偏慢，严重滞后
- `speed=800`：当前 sweep 测试中效果最好
- `speed=1200`：可用，但略差于 800
- `speed=1500`：误差继续增大
- `speed=2500`：没有明显改善，甚至可能使滞后更明显

当前较推荐的遥操参数为：

```powershell
python leader_follower.py follow --period 0.01 --filter-alpha 1.0 --deadband 1 --speed 800 --acc 255 --tx-only --report-interval 1
```

## Sweep Test 数据

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

当目标变化速率提高后：

```text
rate=600, speed=1500, acc=250
avg_abs_err=320.8
max_abs_err=664
```

这说明 follower 的误差会随着目标轨迹变化速度增大而明显增大。

## Leader 速度测量方法

新增 `monitor-leader` 命令，用 leader 位置差分估算手拖速度：

```powershell
python leader_follower.py monitor-leader --period 0.01 --report-interval 0.2
```

换算关系：

```text
4096 steps = 1 revolution
rpm = step/s * 60 / 4096
```

本次复现跟不上时测得：

```text
max = 37.1 rpm
```

## 结论

当 leader 手拖速度过快时，follower 跟不上是正常现象。

leader 可以被人手快速拖动到接近电机空载最高转速，但 follower 需要在带负载、有限加速度、有限扭矩和内部 PID 控制下追踪不断变化的目标位置，因此无法做到任意速度下实时同步。

这不是单纯的软件延迟问题，而是物理系统和舵机内部控制能力的限制。

## 后续建议

- 遥操时避免过快拖动 leader，否则 follower 必然产生滞后
- 软件侧保持低延迟参数：不滤波、小死区、`tx-only`
- follower 速度参数建议优先使用实测较好的 `800..1200` 区间，不要盲目增大
- 加速度 `acc` 可使用合法最大值 `255`，前提是动作冲击可以接受
- 检查 12 V 供电电流能力，避免负载下电压波动
- 尽量减小 follower 摆杆负载和转动惯量
- 后续如需进一步提升动态跟随能力，可以谨慎调整 PID，其中跟随滞后主要考虑增大 P，若出现超调再调整 D
