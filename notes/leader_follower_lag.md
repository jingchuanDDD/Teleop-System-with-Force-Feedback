# Leader/Follower Lag Notes

Date: 2026-06-03

## Setup

- Servo model: Feetech STS3215 / ST-3215-C018
- Bus: COM16, 1000000 baud
- Leader: ID2
- Follower: ID1
- Power: 12 V
- Control mode: leader position is read and mapped to follower target position

Calibration file currently uses near-equal centers:

```text
leader_center = 2048
follower_center = 2047
direction = 1
gain = 1.0
```

## Observed Problem

When the leader lever is dragged quickly, the follower visibly lags behind.

Example runtime log:

```text
leader=1921 offset=-127 leader_v=-2533step/s 37.1rpm target=1920 follower=2619 err=-699 speed=-950
```

Interpretation:

- Leader was moving at about `2533 step/s`, or `37.1 rpm`.
- Follower target had already moved to `1920`.
- Follower actual position was still `2619`.
- Follower was behind by `699 steps`, about `61.4 deg`.
- Follower speed feedback was negative, so it was moving in the correct direction, but not fast enough to close the gap.

## Why It Happens

This is primarily a physical and internal servo-control limit, not a Python loop bug.

The follower receives a stream of changing target positions:

```text
read leader position -> calculate follower target -> write target to follower
```

If the leader moves quickly, the follower target also moves quickly. The follower then has to chase a moving target through its internal PID controller, with finite speed, acceleration, torque, load, and inertia.

STS3215-C018 no-load speed is about `45 rpm` at 12 V. The measured leader speed during the lag case was about `37.1 rpm`, already about `82%` of the no-load maximum speed:

```text
37.1 / 45 ~= 82%
```

The follower is not operating under ideal no-load conditions. It has lever inertia, friction, acceleration limits, and internal PID response. Therefore its stable tracking speed is lower than the no-load maximum.

## Speed Parameter Findings

The manual states that the running speed field can use a default unit of `50 steps/s = 0.732 rpm`, and that setting speed above what the servo body can respond to causes lag.

In practice, this setup does not behave well with either very low or very high speed values:

- `speed=60/80/100`: very slow, severe lag
- `speed=800`: best observed result in sweep tests
- `speed=1200`: usable, slightly worse than 800
- `speed=1500`: worse than 800/1200
- `speed=2500`: not reliably better; can increase lag

Useful current setting:

```powershell
python leader_follower.py follow --period 0.01 --filter-alpha 1.0 --deadband 1 --speed 800 --acc 255 --tx-only --report-interval 1
```

## Sweep Test Results

Automatic follower sweep test, range `1300..2800`, target rate `300 step/s`, `acc=255`, `tx-only`:

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

At faster target rate:

```text
rate=600, speed=1500, acc=250
avg_abs_err=320.8
max_abs_err=664
```

This confirms that follower error grows when the target trajectory changes faster than the follower can physically track.

## Leader Speed Measurement

`monitor-leader` estimates leader speed using position difference over time:

```powershell
python leader_follower.py monitor-leader --period 0.01 --report-interval 0.2
```

Conversion:

```text
4096 steps = 1 revolution
rpm = step/s * 60 / 4096
```

Lag reproduction measured:

```text
max=37.1 rpm
```

## Conclusion

Follower lag is expected when the leader is moved too fast.

The leader can be hand-dragged near the servo's no-load speed, but the follower cannot perfectly track that motion while also carrying lever inertia and following through internal PID position control.

For real-time teleoperation:

- Move the leader more slowly for accurate tracking.
- Use low-latency software parameters: no filter, small deadband, tx-only writes.
- Use empirically good follower speed values around `800..1200`, not blindly larger values.
- Keep `acc` at the legal maximum `255` if aggressive response is acceptable.
- Improve hardware if needed: reduce load/inertia, ensure sufficient 12 V current capacity, or use a faster/stronger actuator.
- PID tuning may help, but should be changed carefully because it affects stability, overshoot, and oscillation.
