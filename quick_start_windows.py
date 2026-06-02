#!/usr/bin/env python

"""
常用的命令
python quick_start_windows.py --port COM16 --model sms_sts move --baudrate 1000000 --id 1 --delta 300 --speed 100 --acc 30
"""
import argparse
import sys
import time

from scservo_sdk import *


HANDLERS = {
    "sms_sts": sms_sts,
    "scscl": scscl,
    "hls": hls,
}

BAUDRATES = [1000000, 500000, 250000, 128000, 115200, 57600, 38400]


def open_bus(port_name, baudrate, model):
    port = PortHandler(port_name)
    packet = HANDLERS[model](port)
    if not port.openPort():
        raise RuntimeError("failed to open port %s" % port_name)
    if not port.setBaudRate(baudrate):
        port.closePort()
        raise RuntimeError("failed to set baudrate %d" % baudrate)
    return port, packet


def scan(args):
    found = []
    for baudrate in args.baudrates:
        port, packet = open_bus(args.port, baudrate, args.model)
        try:
            for servo_id in range(args.start_id, args.end_id + 1):
                model_number, comm, err = packet.ping(servo_id)
                if comm == COMM_SUCCESS and err == 0:
                    print("FOUND model=%s port=%s baud=%d id=%d servo_model=%d" %
                          (args.model, args.port, baudrate, servo_id, model_number))
                    found.append((baudrate, servo_id, model_number))
        finally:
            port.closePort()
    if not found:
        print("No servo replied. Check power, wiring, servo ID, baudrate, and model family.")
    return 0


def ping(args):
    port, packet = open_bus(args.port, args.baudrate, args.model)
    try:
        model_number, comm, err = packet.ping(args.id)
        if comm != COMM_SUCCESS:
            print(packet.getTxRxResult(comm))
            return 1
        if err != 0:
            print(packet.getRxPacketError(err))
            return 1
        print("Ping OK: model=%s port=%s baud=%d id=%d servo_model=%d" %
              (args.model, args.port, args.baudrate, args.id, model_number))
        return 0
    finally:
        port.closePort()


def move(args):
    port, packet = open_bus(args.port, args.baudrate, args.model)
    try:
        pos, comm, err = packet.ReadPos(args.id)
        if comm != COMM_SUCCESS:
            print(packet.getTxRxResult(comm))
            return 1
        if err != 0:
            print(packet.getRxPacketError(err))
            return 1

        low = max(0, pos - args.delta)
        high = min(4095, pos + args.delta)
        print("Current position: %d; moving to %d then %d" % (pos, high, low))

        for target in [high, low, pos]:
            comm, err = packet.WritePosEx(args.id, target, args.speed, args.acc)
            if comm != COMM_SUCCESS:
                print(packet.getTxRxResult(comm))
                return 1
            if err != 0:
                print(packet.getRxPacketError(err))
                return 1
            time.sleep(args.delay)
        print("Move command sequence sent.")
        return 0
    finally:
        port.closePort()


def main():
    parser = argparse.ArgumentParser(description="FTServo Windows quick start helper")
    parser.add_argument("--port", default="COM16")
    parser.add_argument("--model", choices=sorted(HANDLERS), default="sms_sts")

    subparsers = parser.add_subparsers(dest="command")

    scan_parser = subparsers.add_parser("scan")
    scan_parser.add_argument("--baudrates", nargs="+", type=int, default=BAUDRATES)
    scan_parser.add_argument("--start-id", type=int, default=1)
    scan_parser.add_argument("--end-id", type=int, default=20)
    scan_parser.set_defaults(func=scan)

    ping_parser = subparsers.add_parser("ping")
    ping_parser.add_argument("--baudrate", type=int, default=1000000)
    ping_parser.add_argument("--id", type=int, default=1)
    ping_parser.set_defaults(func=ping)

    move_parser = subparsers.add_parser("move")
    move_parser.add_argument("--baudrate", type=int, default=1000000)
    move_parser.add_argument("--id", type=int, default=1)
    move_parser.add_argument("--delta", type=int, default=150)
    move_parser.add_argument("--speed", type=int, default=60)
    move_parser.add_argument("--acc", type=int, default=20)
    move_parser.add_argument("--delay", type=float, default=1.0)
    move_parser.set_defaults(func=move)

    args = parser.parse_args()
    if not hasattr(args, "func"):
        parser.print_help()
        return 2
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
