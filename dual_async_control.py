#!/usr/bin/env python
"""
常用命令：
异步独立控制两个舵机：
虽然用了 asyncio，但同一个串口一次只能发一包，所以实际仍然是一前一后发。两包之间通常只差几毫秒，人眼看起来就是同时动。
python dual_async_control.py --port COM16 --baudrate 1000000 --model sms_sts goto --pos1 300 --pos2 600 --speed 80 --acc 20
同步同一帧下发两个目标位置：
把 ID1 和 ID2 的目标位置打包进同一个 sync write 数据包
一次性发到总线，两个舵机会在收到同一个广播包后同时更新目标。时间一致性更好。
python dual_async_control.py --port COM16 --baudrate 1000000 --model sms_sts sync-goto --pos1 300 --pos2 600 --speed 80 --acc 20
异步独立控制两个舵机做波动运动：ID2电机比ID1电机晚0.35秒开始动
python dual_async_control.py --port COM16 --baudrate 1000000 --model sms_sts demo --delta1 100 --delta2 100 --speed 60 --acc 20
"""
import argparse
import asyncio
import sys

from scservo_sdk import *


HANDLERS = {
    "sms_sts": sms_sts,
    "scscl": scscl,
    "hls": hls,
}


class AsyncServoBus(object):
    def __init__(self, port_name, baudrate, model):
        self.port_name = port_name
        self.baudrate = baudrate
        self.model = model
        self.port = None
        self.packet = None
        self.lock = asyncio.Lock()

    async def __aenter__(self):
        """
        上下文管理器入口方法：Open the port and initialize the packet handler.
        """
        self.port = PortHandler(self.port_name)
        self.packet = HANDLERS[self.model](self.port)
        if not self.port.openPort():
            raise RuntimeError("failed to open port %s" % self.port_name)
        if not self.port.setBaudRate(self.baudrate):
            self.port.closePort()
            raise RuntimeError("failed to set baudrate %d" % self.baudrate)
        return self

    async def __aexit__(self, exc_type, exc, tb):
        if self.port is not None:
            self.port.closePort()

    async def ping(self, servo_id):
        async with self.lock:
            model_number, comm, err = self.packet.ping(servo_id)
        self._check(comm, err)
        return model_number

    async def read_pos(self, servo_id):
        last_error = None
        for _ in range(3):
            try:
                async with self.lock:
                    pos, comm, err = self.packet.ReadPos(servo_id)
                self._check(comm, err)
                return pos
            except IndexError as exc:
                last_error = exc
                await asyncio.sleep(0.002)
        raise RuntimeError("[ID:%03d] short position response: %s" %
                           (servo_id, last_error))

    async def read_pos_speed(self, servo_id):
        last_error = None
        for _ in range(3):
            try:
                async with self.lock:
                    pos, speed, comm, err = self.packet.ReadPosSpeed(servo_id)
                self._check(comm, err)
                return pos, speed
            except IndexError as exc:
                last_error = exc
                await asyncio.sleep(0.002)
        raise RuntimeError("[ID:%03d] short position/speed response: %s" %
                           (servo_id, last_error))

    async def read_moving(self, servo_id):
        last_error = None
        for _ in range(3):
            try:
                async with self.lock:
                    moving, comm, err = self.packet.ReadMoving(servo_id)
                self._check(comm, err)
                return moving
            except IndexError as exc:
                last_error = exc
                await asyncio.sleep(0.002)
        raise RuntimeError("[ID:%03d] short moving response: %s" %
                           (servo_id, last_error))

    async def wait_until_position(self, servo_id, target_position, poll_interval,
                                  timeout, position_tolerance):
        loop = asyncio.get_event_loop()
        deadline = loop.time() + timeout
        target_position = max(0, min(4095, target_position))

        while True:
            pos, speed = await self.read_pos_speed(servo_id)
            moving = await self.read_moving(servo_id)
            if moving == 0 and abs(pos - target_position) <= position_tolerance:
                return
            if loop.time() >= deadline:
                raise RuntimeError(
                    "[ID:%03d] move timeout: target=%d pos=%d speed=%d moving=%d" %
                    (servo_id, target_position, pos, speed, moving)
                )
            await asyncio.sleep(poll_interval)

    async def write_pos(self, servo_id, position, speed, acc):
        position = max(0, min(4095, position))
        async with self.lock:
            comm, err = self.packet.WritePosEx(servo_id, position, speed, acc)
        self._check(comm, err)
        return position

    async def sync_write_pos(self, targets, speed, acc):
        async with self.lock:
            self.packet.groupSyncWrite.clearParam()
            # 写缓冲区，先把所有舵机的目标位置都打包进同一个 sync write 数据包
            for servo_id, position in targets.items():
                position = max(0, min(4095, position))
                ok = self.packet.SyncWritePosEx(servo_id, position, speed, acc)
                if not ok:
                    self.packet.groupSyncWrite.clearParam()
                    raise RuntimeError("[ID:%03d] groupSyncWrite addParam failed" % servo_id)
            # 把刚才累积的所有参数组装成一个 sync write 包一次性发到串口总线
            comm = self.packet.groupSyncWrite.txPacket()
            self.packet.groupSyncWrite.clearParam()

        if comm != COMM_SUCCESS:
            raise RuntimeError(self.packet.getTxRxResult(comm))

    def _check(self, comm, err):
        if comm != COMM_SUCCESS:
            raise RuntimeError(self.packet.getTxRxResult(comm))
        if err != 0:
            raise RuntimeError(self.packet.getRxPacketError(err))


async def servo_wave(bus, servo_id, delta, speed, acc, poll_interval,
                     move_timeout, position_tolerance, cycles, phase_delay):
    await asyncio.sleep(phase_delay)
    start = await bus.read_pos(servo_id)
    high = max(0, min(4095, start + delta))
    low = max(0, min(4095, start - delta))
    print("[ID:%03d] start=%d high=%d low=%d" % (servo_id, start, high, low))

    for cycle in range(1, cycles + 1):
        print("[ID:%03d] cycle %d/%d -> %d" %
              (servo_id, cycle, cycles, high))
        await bus.write_pos(servo_id, high, speed, acc)
        await bus.wait_until_position(servo_id, high, poll_interval,
                                      move_timeout, position_tolerance)
        print("[ID:%03d] cycle %d/%d -> %d" %
              (servo_id, cycle, cycles, low))
        await bus.write_pos(servo_id, low, speed, acc)
        await bus.wait_until_position(servo_id, low, poll_interval,
                                      move_timeout, position_tolerance)

    await bus.write_pos(servo_id, start, speed, acc)
    await bus.wait_until_position(servo_id, start, poll_interval,
                                  move_timeout, position_tolerance)
    print("[ID:%03d] returned to %d" % (servo_id, start))


async def run_read(args):
    async with AsyncServoBus(args.port, args.baudrate, args.model) as bus:
        for servo_id in args.ids:
            model_number = await bus.ping(servo_id)
            pos, speed = await bus.read_pos_speed(servo_id)
            print("[ID:%03d] ping OK model=%d pos=%d speed=%d" %
                  (servo_id, model_number, pos, speed))


async def run_goto(args):
    async with AsyncServoBus(args.port, args.baudrate, args.model) as bus:
        tasks = [
            bus.write_pos(args.id1, args.pos1, args.speed, args.acc),
            bus.write_pos(args.id2, args.pos2, args.speed, args.acc),
        ]
        positions = await asyncio.gather(*tasks)
        print("[ID:%03d] target=%d" % (args.id1, positions[0]))
        print("[ID:%03d] target=%d" % (args.id2, positions[1]))


async def run_sync_goto(args):
    async with AsyncServoBus(args.port, args.baudrate, args.model) as bus:
        await bus.sync_write_pos({
            args.id1: args.pos1,
            args.id2: args.pos2,
        }, args.speed, args.acc)
        print("sync target: ID%d=%d ID%d=%d" %
              (args.id1, max(0, min(4095, args.pos1)),
               args.id2, max(0, min(4095, args.pos2))))


async def run_demo(args):
    async with AsyncServoBus(args.port, args.baudrate, args.model) as bus:
        await asyncio.gather(
            servo_wave(bus, args.id1, args.delta1, args.speed, args.acc,
                       args.poll_interval, args.move_timeout,
                       args.position_tolerance, args.cycles, 0.0),
            servo_wave(bus, args.id2, args.delta2, args.speed, args.acc,
                       args.poll_interval, args.move_timeout,
                       args.position_tolerance, args.cycles, args.phase),
        )
        print("async demo complete")


def main():
    parser = argparse.ArgumentParser(description="Async dual FTServo control")
    parser.add_argument("--port", default="COM16")
    parser.add_argument("--baudrate", type=int, default=1000000)
    parser.add_argument("--model", choices=sorted(HANDLERS), default="sms_sts")

    subparsers = parser.add_subparsers(dest="command")

    read_parser = subparsers.add_parser("read")
    read_parser.add_argument("--ids", nargs="+", type=int, default=[1, 2])
    read_parser.set_defaults(func=run_read)

    goto_parser = subparsers.add_parser("goto")
    goto_parser.add_argument("--id1", type=int, default=1)
    goto_parser.add_argument("--pos1", type=int, required=True)
    goto_parser.add_argument("--id2", type=int, default=2)
    goto_parser.add_argument("--pos2", type=int, required=True)
    goto_parser.add_argument("--speed", type=int, default=80)
    goto_parser.add_argument("--acc", type=int, default=20)
    goto_parser.set_defaults(func=run_goto)

    sync_goto_parser = subparsers.add_parser("sync-goto")
    sync_goto_parser.add_argument("--id1", type=int, default=1)
    sync_goto_parser.add_argument("--pos1", type=int, required=True)
    sync_goto_parser.add_argument("--id2", type=int, default=2)
    sync_goto_parser.add_argument("--pos2", type=int, required=True)
    sync_goto_parser.add_argument("--speed", type=int, default=80)
    sync_goto_parser.add_argument("--acc", type=int, default=20)
    sync_goto_parser.set_defaults(func=run_sync_goto)

    demo_parser = subparsers.add_parser("demo")
    demo_parser.add_argument("--id1", type=int, default=1)
    demo_parser.add_argument("--id2", type=int, default=2)
    demo_parser.add_argument("--delta1", type=int, default=150)
    demo_parser.add_argument("--delta2", type=int, default=150)
    demo_parser.add_argument("--speed", type=int, default=80)
    demo_parser.add_argument("--acc", type=int, default=20)
    demo_parser.add_argument("--poll-interval", "--delay", dest="poll_interval",
                             type=float, default=0.05)
    demo_parser.add_argument("--move-timeout", type=float, default=10.0)
    demo_parser.add_argument("--position-tolerance", type=int, default=10)
    demo_parser.add_argument("--cycles", type=int, default=2)
    demo_parser.add_argument("--phase", type=float, default=0.35)
    demo_parser.set_defaults(func=run_demo)

    args = parser.parse_args()
    if not hasattr(args, "func"):
        parser.print_help()
        return 2

    loop = asyncio.get_event_loop()
    return loop.run_until_complete(args.func(args)) or 0


if __name__ == "__main__":
    sys.exit(main())
