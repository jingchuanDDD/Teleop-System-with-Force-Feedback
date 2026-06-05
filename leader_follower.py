#!/usr/bin/env python
"""
稳定版遥操：
python leader_follower.py follow --period 0 --write-interval 0.01 --filter-alpha 1.0 --deadband 1 --speed 0 --acc 255 --report-interval 1 --timing --ignore-write-errors
"""
import argparse
import asyncio
import json
import os

from dual_async_control import AsyncServoBus
from scservo_sdk import *


DEFAULT_CONFIG = "leader_follower_config.json"
STS3215_MAX_RPM = 45.0
STEPS_PER_REV = 4096.0
PID_ADDR = {
    "p": 21,
    "d": 22,
    "i": 23,
}

TORQUE_ENABLE_ADDR = {
    "sms_sts": SMS_STS_TORQUE_ENABLE,
    "scscl": SCSCL_TORQUE_ENABLE,
    "hls": HLS_TORQUE_ENABLE,
}


def clamp(value, low, high):
    return max(low, min(high, value))


def load_config(path):
    with open(path, "r") as fp:
        return json.load(fp)


def save_config(path, config):
    with open(path, "w") as fp:
        json.dump(config, fp, indent=2, sort_keys=True)
        fp.write("\n")


def target_from_leader(config, leader_pos):
    offset = leader_pos - config["leader_center"]
    target = config["follower_center"] + config["direction"] * config["gain"] * offset
    return int(round(clamp(target, config["follower_min"], config["follower_max"])))


def validate_motion_args(speed, acc):
    if speed < 0 or speed > 32767:
        raise RuntimeError("--speed must be in range 0..32767")
    if acc < 0 or acc > 255:
        raise RuntimeError("--acc must be in range 0..255 for sms_sts")


def speed_help_text():
    return ("follower position-mode speed limit. For STS/SMS position control, "
            "0 appears to mean no speed limit / fastest in this setup.")


def steps_per_second_to_rpm(steps_per_second):
    return steps_per_second * 60.0 / STEPS_PER_REV


def apply_follow_overrides(config, args):
    config = dict(config)
    if args.direction is not None:
        config["direction"] = args.direction
    if args.gain is not None:
        config["gain"] = args.gain
    if args.deadband is not None:
        config["deadband"] = args.deadband
    if args.filter_alpha is not None:
        config["filter_alpha"] = args.filter_alpha
    return config


def leader_port(config, args):
    return args.leader_port or args.port or config.get("leader_port") or config.get("port")


def follower_port(config, args):
    return args.follower_port or args.port or config.get("follower_port") or config.get("port")


def leader_baudrate(config, args):
    return (args.leader_baudrate or args.baudrate or
            config.get("leader_baudrate") or config.get("baudrate"))


def follower_baudrate(config, args):
    return (args.follower_baudrate or args.baudrate or
            config.get("follower_baudrate") or config.get("baudrate"))


def model_name(config, args):
    return args.model or config.get("model", "sms_sts")


async def set_torque(bus, servo_id, enabled):
    address = TORQUE_ENABLE_ADDR[bus.model]
    value = 1 if enabled else 0
    async with bus.lock:
        comm, err = bus.packet.write1ByteTxRx(servo_id, address, value)
    bus._check(comm, err)


async def read_u8(bus, servo_id, address):
    async with bus.lock:
        value, comm, err = bus.packet.read1ByteTxRx(servo_id, address)
    bus._check(comm, err)
    return value


async def write_u8(bus, servo_id, address, value):
    if value < 0 or value > 255:
        raise RuntimeError("1-byte value must be in range 0..255")
    async with bus.lock:
        comm, err = bus.packet.write1ByteTxRx(servo_id, address, value)
    bus._check(comm, err)


async def read_pid(bus, servo_id):
    return {
        "p": await read_u8(bus, servo_id, PID_ADDR["p"]),
        "d": await read_u8(bus, servo_id, PID_ADDR["d"]),
        "i": await read_u8(bus, servo_id, PID_ADDR["i"]),
    }


async def set_eprom_lock(bus, servo_id, locked):
    async with bus.lock:
        if locked:
            comm, err = bus.packet.LockEprom(servo_id)
        else:
            comm, err = bus.packet.unLockEprom(servo_id)
    bus._check(comm, err)


async def write_pos_tx_only(bus, servo_id, position, speed, acc):
    if bus.model != "sms_sts":
        raise RuntimeError("--tx-only is currently implemented for sms_sts only")
    validate_motion_args(speed, acc)

    position = clamp(position, 0, 4095)
    scs_position = bus.packet.scs_toscs(position, 15)
    txpacket = [
        acc,
        bus.packet.scs_lobyte(scs_position),
        bus.packet.scs_hibyte(scs_position),
        0,
        0,
        bus.packet.scs_lobyte(speed),
        bus.packet.scs_hibyte(speed),
    ]

    async with bus.lock:
        comm = bus.packet.writeTxOnly(servo_id, SMS_STS_ACC, len(txpacket), txpacket)
    if comm != COMM_SUCCESS:
        raise RuntimeError(bus.packet.getTxRxResult(comm))
    return position


async def run_calibrate(args):
    leader_bus_args = (
        args.leader_port or args.port,
        args.leader_baudrate or args.baudrate,
        args.model,
    )
    follower_bus_args = (
        args.follower_port or args.port,
        args.follower_baudrate or args.baudrate,
        args.model,
    )

    async with AsyncServoBus(*leader_bus_args) as leader_bus:
        leader_model = await leader_bus.ping(args.leader)
        leader_center = await leader_bus.read_pos(args.leader)

    async with AsyncServoBus(*follower_bus_args) as follower_bus:
        follower_model = await follower_bus.ping(args.follower)
        follower_center = await follower_bus.read_pos(args.follower)

    old_config = {}
    if os.path.exists(args.config):
        old_config = load_config(args.config)

    follower_range = args.follower_range
    config = {
        "port": args.port,
        "baudrate": args.baudrate,
        "leader_port": leader_bus_args[0],
        "leader_baudrate": leader_bus_args[1],
        "follower_port": follower_bus_args[0],
        "follower_baudrate": follower_bus_args[1],
        "model": args.model,
        "leader_id": args.leader,
        "follower_id": args.follower,
        "leader_center": leader_center,
        "follower_center": follower_center,
        "leader_model": leader_model,
        "follower_model": follower_model,
        "direction": args.direction if args.direction is not None else old_config.get("direction", 1),
        "gain": args.gain if args.gain is not None else old_config.get("gain", 1.0),
        "follower_min": args.follower_min if args.follower_min is not None
        else old_config.get("follower_min", max(0, follower_center - follower_range)),
        "follower_max": args.follower_max if args.follower_max is not None
        else old_config.get("follower_max", min(4095, follower_center + follower_range)),
        "deadband": args.deadband if args.deadband is not None else old_config.get("deadband", 5),
        "filter_alpha": args.filter_alpha if args.filter_alpha is not None
        else old_config.get("filter_alpha", 0.4),
    }
    save_config(args.config, config)

    print("Calibration saved to %s" % args.config)
    print("leader ID%d center=%d model=%d" %
          (args.leader, leader_center, leader_model))
    print("follower ID%d center=%d model=%d range=[%d, %d]" %
          (args.follower, follower_center, follower_model,
           config["follower_min"], config["follower_max"]))


async def run_status(args):
    config = load_config(args.config)
    model = model_name(config, args)
    async with AsyncServoBus(leader_port(config, args),
                             leader_baudrate(config, args),
                             model) as leader_bus:
        leader_pos = await leader_bus.read_pos(config["leader_id"])

    async with AsyncServoBus(follower_port(config, args),
                             follower_baudrate(config, args),
                             model) as follower_bus:
        follower_pos, follower_speed = await follower_bus.read_pos_speed(config["follower_id"])

    target = target_from_leader(config, leader_pos)
    print("leader ID%d pos=%d center=%d offset=%d" %
          (config["leader_id"], leader_pos, config["leader_center"],
           leader_pos - config["leader_center"]))
    print("follower ID%d pos=%d speed=%d center=%d target=%d range=[%d, %d]" %
          (config["follower_id"], follower_pos, follower_speed,
           config["follower_center"], target,
           config["follower_min"], config["follower_max"]))
    print("mapping: target = follower_center + direction(%d) * gain(%.3f) * leader_offset" %
          (config["direction"], config["gain"]))


async def run_follow(args):
    # 参数覆盖：命令行参数覆盖配置文件中的对应项
    config = apply_follow_overrides(load_config(args.config), args)
    model = model_name(config, args)
    leader_id = config["leader_id"]
    follower_id = config["follower_id"]
    validate_motion_args(args.speed, args.acc)

    async with AsyncServoBus(leader_port(config, args),
                             leader_baudrate(config, args),
                             model) as leader_bus, \
            AsyncServoBus(follower_port(config, args),
                          follower_baudrate(config, args),
                          model) as follower_bus:
        if args.disable_leader_torque and not args.dry_run:
            # 正常遥操时leader要跟手，不能开扭矩
            await set_torque(leader_bus, leader_id, False)
            print("leader ID%d torque disabled" % leader_id)
        if not args.dry_run:
            # 不只看数值就把follower的扭矩打开，观察跟随效果
            await set_torque(follower_bus, follower_id, True)
            print("follower ID%d torque enabled" % follower_id)

        leader_pos = await leader_bus.read_pos(leader_id)
        follower_pos = await follower_bus.read_pos(follower_id)
        target = target_from_leader(config, leader_pos)
        print("start leader=%d follower=%d target=%d dry_run=%s" %
              (leader_pos, follower_pos, target, args.dry_run))
        print("control period=%.3fs speed=%d acc=%d gain=%.3f direction=%d "
              "deadband=%d filter_alpha=%.3f tx_only=%s" %
              (args.period, args.speed, args.acc, config["gain"],
               config["direction"], config["deadband"], config["filter_alpha"],
               args.tx_only))

        loop = asyncio.get_event_loop()
        start_time = loop.time()
        next_report = start_time
        filtered_target = float(target)
        last_sent = None
        last_write_time = 0.0
        last_leader_pos = leader_pos
        last_leader_time = start_time
        leader_velocity_sps = 0.0
        loop_count = 0
        read_leader_total = 0.0
        write_follower_total = 0.0
        loop_total = 0.0
        follower_read_errors = 0
        follower_write_errors = 0

        while True:
            loop_start = loop.time()
            now = loop_start
            if args.duration > 0 and now - start_time >= args.duration:
                break

            read_start = loop.time()
            leader_pos = await leader_bus.read_pos(leader_id)
            read_leader_total += loop.time() - read_start
            dt = now - last_leader_time
            if dt > 0:
                leader_velocity_sps = (leader_pos - last_leader_pos) / dt
                last_leader_pos = leader_pos
                last_leader_time = now
            raw_target = target_from_leader(config, leader_pos)
            alpha = config["filter_alpha"]
            filtered_target = (1.0 - alpha) * filtered_target + alpha * raw_target
            target = int(round(filtered_target))
            target = clamp(target, config["follower_min"], config["follower_max"])

            should_send = last_sent is None or abs(target - last_sent) >= config["deadband"]
            write_due = args.write_interval <= 0 or now - last_write_time >= args.write_interval
            if should_send and write_due and not args.dry_run:
                write_start = loop.time()
                try:
                    if args.tx_only:
                        last_sent = await write_pos_tx_only(follower_bus, follower_id, target,
                                                            args.speed, args.acc)
                    else:
                        last_sent = await follower_bus.write_pos(follower_id, target,
                                                        args.speed, args.acc)
                except RuntimeError as exc:
                    follower_write_errors += 1
                    if not args.ignore_write_errors:
                        raise
                    if args.timing:
                        print("follower_write_error target=%d error=%s" %
                              (target, exc))
                write_follower_total += loop.time() - write_start
                last_write_time = now
            elif should_send and args.dry_run:
                last_sent = target

            loop_count += 1
            loop_total += loop.time() - loop_start

            if now >= next_report:
                try:
                    follower_pos, follower_speed = await follower_bus.read_pos_speed(follower_id)
                    leader_rpm = steps_per_second_to_rpm(abs(leader_velocity_sps))
                    print("leader=%d offset=%d leader_v=%.0fstep/s %.1frpm target=%d "
                          "follower=%d err=%d speed=%d" %
                          (leader_pos, leader_pos - config["leader_center"],
                           leader_velocity_sps, leader_rpm, target,
                           follower_pos, target - follower_pos, follower_speed))
                except RuntimeError as exc:
                    follower_read_errors += 1
                    print("leader=%d offset=%d target=%d follower_read_error=%s" %
                          (leader_pos, leader_pos - config["leader_center"],
                           target, exc))
                if args.timing and loop_count:
                    elapsed = max(loop.time() - start_time, 0.001)
                    print("timing loop_hz=%.1f avg_loop=%.2fms "
                          "avg_read_leader=%.2fms avg_write_follower=%.2fms "
                          "follower_read_errors=%d follower_write_errors=%d" %
                          (loop_count / elapsed,
                           loop_total / loop_count * 1000.0,
                           read_leader_total / loop_count * 1000.0,
                           write_follower_total / loop_count * 1000.0,
                           follower_read_errors, follower_write_errors))
                next_report = now + args.report_interval

            await asyncio.sleep(args.period)

    print("follow stopped")


async def run_monitor_leader(args):
    config = load_config(args.config)
    model = model_name(config, args)
    leader_id = args.leader or config["leader_id"]

    async with AsyncServoBus(leader_port(config, args),
                             leader_baudrate(config, args),
                             model) as bus:
        if args.disable_leader_torque:
            await set_torque(bus, leader_id, False)
            print("leader ID%d torque disabled" % leader_id)

        loop = asyncio.get_event_loop()
        start_time = loop.time()
        last_time = start_time
        last_pos = await bus.read_pos(leader_id)
        max_abs_sps = 0.0
        max_abs_rpm = 0.0
        next_report = start_time
        print("monitor leader ID%d start_pos=%d max_ref=%.1frpm" %
              (leader_id, last_pos, args.max_rpm))

        while True:
            now = loop.time()
            if args.duration > 0 and now - start_time >= args.duration:
                break

            pos = await bus.read_pos(leader_id)
            dt = now - last_time
            if dt > 0:
                velocity_sps = (pos - last_pos) / dt
            else:
                velocity_sps = 0.0
            rpm = steps_per_second_to_rpm(abs(velocity_sps))
            max_abs_sps = max(max_abs_sps, abs(velocity_sps))
            max_abs_rpm = max(max_abs_rpm, rpm)

            if now >= next_report:
                pct = rpm / args.max_rpm * 100.0 if args.max_rpm > 0 else 0.0
                print("pos=%d v=%.0fstep/s %.1frpm %.0f%%of%.1frpm "
                      "max=%.1frpm" %
                      (pos, velocity_sps, rpm, pct, args.max_rpm,
                       max_abs_rpm))
                next_report = now + args.report_interval

            last_pos = pos
            last_time = now
            await asyncio.sleep(args.period)

    print("monitor stopped max=%.0fstep/s %.1frpm" %
          (max_abs_sps, max_abs_rpm))


async def run_sweep_test(args):
    config = load_config(args.config)
    model = model_name(config, args)
    follower_id = args.follower or config["follower_id"]

    start_pos = clamp(args.start, 0, 4095)
    end_pos = clamp(args.end, 0, 4095)
    span = abs(end_pos - start_pos)
    if span == 0:
        raise RuntimeError("--start and --end must be different")
    validate_motion_args(args.speed, args.acc)

    async with AsyncServoBus(follower_port(config, args),
                             follower_baudrate(config, args),
                             model) as bus:
        await set_torque(bus, follower_id, True)
        print("sweep follower ID%d range=[%d, %d] period=%.3fs speed=%d "
              "acc=%d tx_only=%s" %
              (follower_id, start_pos, end_pos, args.period, args.speed,
               args.acc, args.tx_only))
        await bus.write_pos(follower_id, start_pos, args.speed, args.acc)
        await bus.wait_until_position(follower_id, start_pos, 0.02,
                                      args.settle_timeout,
                                      args.position_tolerance)
        print("settled at %d" % start_pos)

        loop = asyncio.get_event_loop()
        start_time = loop.time()
        next_report = start_time
        samples = 0
        read_errors = 0
        total_abs_err = 0
        max_abs_err = 0
        last_target = None
        last_write_time = 0.0

        while True:
            now = loop.time()
            elapsed = now - start_time
            if args.duration > 0 and elapsed >= args.duration:
                break

            cycle_pos = (elapsed * args.rate) % (2.0 * span)
            if cycle_pos <= span:
                target = int(round(start_pos + cycle_pos))
            else:
                target = int(round(end_pos - (cycle_pos - span)))

            write_due = args.write_interval <= 0 or now - last_write_time >= args.write_interval
            if write_due and (last_target is None or abs(target - last_target) >= args.deadband):
                if args.tx_only:
                    last_target = await write_pos_tx_only(bus, follower_id, target,
                                                          args.speed, args.acc)
                else:
                    last_target = await bus.write_pos(follower_id, target,
                                                      args.speed, args.acc)
                last_write_time = now

            try:
                follower_pos, follower_speed = await bus.read_pos_speed(follower_id)
                err = target - follower_pos
                abs_err = abs(err)
                total_abs_err += abs_err
                max_abs_err = max(max_abs_err, abs_err)
                samples += 1
            except RuntimeError:
                read_errors += 1
                await asyncio.sleep(args.period)
                continue

            if now >= next_report:
                avg_abs_err = total_abs_err / samples if samples else 0
                print("t=%.2f target=%d follower=%d err=%d speed=%d "
                      "avg_abs_err=%.1f max_abs_err=%d read_errors=%d" %
                      (elapsed, target, follower_pos, err, follower_speed,
                       avg_abs_err, max_abs_err, read_errors))
                next_report = now + args.report_interval

            await asyncio.sleep(args.period)

        avg_abs_err = total_abs_err / samples if samples else 0
        print("sweep complete samples=%d avg_abs_err=%.1f max_abs_err=%d "
              "read_errors=%d" %
              (samples, avg_abs_err, max_abs_err, read_errors))


async def run_pid_read(args):
    config = load_config(args.config)
    model = model_name(config, args)
    follower_id = args.follower or config["follower_id"]

    async with AsyncServoBus(follower_port(config, args),
                             follower_baudrate(config, args),
                             model) as bus:
        pid = await read_pid(bus, follower_id)

    print("follower ID%d PID: P=%d D=%d I=%d" %
          (follower_id, pid["p"], pid["d"], pid["i"]))


async def run_pid_write(args):
    config = load_config(args.config)
    model = model_name(config, args)
    follower_id = args.follower or config["follower_id"]

    if args.p is None and args.d is None and args.i is None:
        raise RuntimeError("provide at least one of --p, --d, --i")

    async with AsyncServoBus(follower_port(config, args),
                             follower_baudrate(config, args),
                             model) as bus:
        old_pid = await read_pid(bus, follower_id)
        new_pid = dict(old_pid)
        if args.p is not None:
            new_pid["p"] = args.p
        if args.d is not None:
            new_pid["d"] = args.d
        if args.i is not None:
            new_pid["i"] = args.i

        print("follower ID%d PID old: P=%d D=%d I=%d" %
              (follower_id, old_pid["p"], old_pid["d"], old_pid["i"]))
        print("follower ID%d PID new: P=%d D=%d I=%d" %
              (follower_id, new_pid["p"], new_pid["d"], new_pid["i"]))

        if args.dry_run:
            print("dry run: PID not written")
            return

        await set_eprom_lock(bus, follower_id, False)
        try:
            if new_pid["p"] != old_pid["p"]:
                await write_u8(bus, follower_id, PID_ADDR["p"], new_pid["p"])
            if new_pid["d"] != old_pid["d"]:
                await write_u8(bus, follower_id, PID_ADDR["d"], new_pid["d"])
            if new_pid["i"] != old_pid["i"]:
                await write_u8(bus, follower_id, PID_ADDR["i"], new_pid["i"])
        finally:
            await set_eprom_lock(bus, follower_id, True)

        actual_pid = await read_pid(bus, follower_id)

    print("follower ID%d PID actual: P=%d D=%d I=%d" %
          (follower_id, actual_pid["p"], actual_pid["d"], actual_pid["i"]))


def main():
    parser = argparse.ArgumentParser(description="Leader/follower FTServo teleoperation")
    parser.add_argument("--config", default=DEFAULT_CONFIG)

    subparsers = parser.add_subparsers(dest="command")
    # 标定功能
    calibrate_parser = subparsers.add_parser("calibrate")
    calibrate_parser.add_argument("--port", default="COM16")
    calibrate_parser.add_argument("--leader-port")
    calibrate_parser.add_argument("--follower-port")
    calibrate_parser.add_argument("--baudrate", type=int, default=1000000)
    calibrate_parser.add_argument("--leader-baudrate", type=int)
    calibrate_parser.add_argument("--follower-baudrate", type=int)
    calibrate_parser.add_argument("--model", choices=sorted(TORQUE_ENABLE_ADDR), default="sms_sts")
    calibrate_parser.add_argument("--leader", type=int, default=2)
    calibrate_parser.add_argument("--follower", type=int, default=1)
    calibrate_parser.add_argument("--direction", type=int, choices=[-1, 1])
    calibrate_parser.add_argument("--gain", type=float)
    calibrate_parser.add_argument("--follower-range", type=int, default=500)
    calibrate_parser.add_argument("--follower-min", type=int)
    calibrate_parser.add_argument("--follower-max", type=int)
    calibrate_parser.add_argument("--deadband", type=int)
    calibrate_parser.add_argument("--filter-alpha", type=float)
    calibrate_parser.set_defaults(func=run_calibrate)

    status_parser = subparsers.add_parser("status")
    status_parser.add_argument("--port")
    status_parser.add_argument("--leader-port")
    status_parser.add_argument("--follower-port")
    status_parser.add_argument("--baudrate", type=int)
    status_parser.add_argument("--leader-baudrate", type=int)
    status_parser.add_argument("--follower-baudrate", type=int)
    status_parser.add_argument("--model", choices=sorted(TORQUE_ENABLE_ADDR))
    status_parser.set_defaults(func=run_status)

    follow_parser = subparsers.add_parser("follow")
    follow_parser.add_argument("--port")
    follow_parser.add_argument("--leader-port")
    follow_parser.add_argument("--follower-port")
    follow_parser.add_argument("--baudrate", type=int)
    follow_parser.add_argument("--leader-baudrate", type=int)
    follow_parser.add_argument("--follower-baudrate", type=int)
    follow_parser.add_argument("--model", choices=sorted(TORQUE_ENABLE_ADDR))
    follow_parser.add_argument("--period", type=float, default=0.03)
    follow_parser.add_argument("--write-interval", type=float, default=0.0)
    follow_parser.add_argument("--speed", type=int, default=0,
                               help=speed_help_text())
    follow_parser.add_argument("--acc", type=int, default=50)
    follow_parser.add_argument("--direction", type=int, choices=[-1, 1])
    follow_parser.add_argument("--gain", type=float)
    follow_parser.add_argument("--deadband", type=int)
    follow_parser.add_argument("--filter-alpha", type=float)
    follow_parser.add_argument("--duration", type=float, default=0.0)
    follow_parser.add_argument("--report-interval", type=float, default=0.5)
    follow_parser.add_argument("--dry-run", action="store_true")
    follow_parser.add_argument("--tx-only", action="store_true")
    follow_parser.add_argument("--ignore-write-errors", action="store_true")
    follow_parser.add_argument("--timing", action="store_true")
    follow_parser.add_argument("--keep-leader-torque", dest="disable_leader_torque",
                               action="store_false")
    follow_parser.set_defaults(func=run_follow, disable_leader_torque=True)

    monitor_parser = subparsers.add_parser("monitor-leader")
    monitor_parser.add_argument("--port")
    monitor_parser.add_argument("--leader-port")
    monitor_parser.add_argument("--follower-port")
    monitor_parser.add_argument("--baudrate", type=int)
    monitor_parser.add_argument("--leader-baudrate", type=int)
    monitor_parser.add_argument("--follower-baudrate", type=int)
    monitor_parser.add_argument("--model", choices=sorted(TORQUE_ENABLE_ADDR))
    monitor_parser.add_argument("--leader", type=int)
    monitor_parser.add_argument("--period", type=float, default=0.01)
    monitor_parser.add_argument("--duration", type=float, default=0.0)
    monitor_parser.add_argument("--report-interval", type=float, default=0.2)
    monitor_parser.add_argument("--max-rpm", type=float, default=STS3215_MAX_RPM)
    monitor_parser.add_argument("--keep-leader-torque", dest="disable_leader_torque",
                                action="store_false")
    monitor_parser.set_defaults(func=run_monitor_leader,
                                disable_leader_torque=True)

    sweep_parser = subparsers.add_parser("sweep-test")
    sweep_parser.add_argument("--port")
    sweep_parser.add_argument("--leader-port")
    sweep_parser.add_argument("--follower-port")
    sweep_parser.add_argument("--baudrate", type=int)
    sweep_parser.add_argument("--leader-baudrate", type=int)
    sweep_parser.add_argument("--follower-baudrate", type=int)
    sweep_parser.add_argument("--model", choices=sorted(TORQUE_ENABLE_ADDR))
    sweep_parser.add_argument("--follower", type=int)
    sweep_parser.add_argument("--start", type=int, default=1300)
    sweep_parser.add_argument("--end", type=int, default=2800)
    sweep_parser.add_argument("--rate", type=float, default=600.0)
    sweep_parser.add_argument("--period", type=float, default=0.01)
    sweep_parser.add_argument("--write-interval", type=float, default=0.0)
    sweep_parser.add_argument("--speed", type=int, default=0,
                              help=speed_help_text())
    sweep_parser.add_argument("--acc", type=int, default=200)
    sweep_parser.add_argument("--deadband", type=int, default=1)
    sweep_parser.add_argument("--duration", type=float, default=8.0)
    sweep_parser.add_argument("--report-interval", type=float, default=0.5)
    sweep_parser.add_argument("--settle-timeout", type=float, default=10.0)
    sweep_parser.add_argument("--position-tolerance", type=int, default=20)
    sweep_parser.add_argument("--tx-only", action="store_true")
    sweep_parser.set_defaults(func=run_sweep_test)

    pid_read_parser = subparsers.add_parser("pid-read")
    pid_read_parser.add_argument("--port")
    pid_read_parser.add_argument("--leader-port")
    pid_read_parser.add_argument("--follower-port")
    pid_read_parser.add_argument("--baudrate", type=int)
    pid_read_parser.add_argument("--leader-baudrate", type=int)
    pid_read_parser.add_argument("--follower-baudrate", type=int)
    pid_read_parser.add_argument("--model", choices=sorted(TORQUE_ENABLE_ADDR))
    pid_read_parser.add_argument("--follower", type=int)
    pid_read_parser.set_defaults(func=run_pid_read)

    pid_write_parser = subparsers.add_parser("pid-write")
    pid_write_parser.add_argument("--port")
    pid_write_parser.add_argument("--leader-port")
    pid_write_parser.add_argument("--follower-port")
    pid_write_parser.add_argument("--baudrate", type=int)
    pid_write_parser.add_argument("--leader-baudrate", type=int)
    pid_write_parser.add_argument("--follower-baudrate", type=int)
    pid_write_parser.add_argument("--model", choices=sorted(TORQUE_ENABLE_ADDR))
    pid_write_parser.add_argument("--follower", type=int)
    pid_write_parser.add_argument("--p", type=int)
    pid_write_parser.add_argument("--d", type=int)
    pid_write_parser.add_argument("--i", type=int)
    pid_write_parser.add_argument("--dry-run", action="store_true")
    pid_write_parser.set_defaults(func=run_pid_write)

    args = parser.parse_args()
    if not hasattr(args, "func"):
        parser.print_help()
        return 2

    loop = asyncio.get_event_loop()
    return loop.run_until_complete(args.func(args)) or 0


if __name__ == "__main__":
    raise SystemExit(main())
