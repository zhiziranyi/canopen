#!/usr/bin/env python3
"""CANopen CiA402 drive acceptance tool for node 1 at 1 Mbps."""

import argparse
from pathlib import Path
import sys
import time


STATUS_MASK = 0x006F
STATUS_SWITCH_ON_DISABLED = 0x0040
STATUS_READY_TO_SWITCH_ON = 0x0021
STATUS_SWITCHED_ON = 0x0023
STATUS_OPERATION_ENABLED = 0x0027


def parse_args():
    epilog = """PcanView/raw SDO examples for node 1 (request COB-ID 0x601):
  read  6041h: 40 41 60 00 00 00 00 00
  write 6040h=0006h: 2B 40 60 00 06 00 00 00
  write 6040h=0007h: 2B 40 60 00 07 00 00 00
  write 6040h=000Fh: 2B 40 60 00 0F 00 00 00
  write 6060h=03h:   2F 60 60 00 03 00 00 00
Responses use COB-ID 0x581. Send NMT start on COB-ID 0x000 with data 01 01.
"""
    parser = argparse.ArgumentParser(
        description="CANopen servo: status, standard enable, motion and disable",
        epilog=epilog,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--interface", default="slcan",
                        choices=["slcan", "gs_usb", "pcan", "socketcan"])
    parser.add_argument("--port", default="COM7",
                        help="channel: COM7, PCAN_USBBUS1, can0, or gs_usb index")
    parser.add_argument("--node", type=int, default=1)
    parser.add_argument("--eds", default=str(Path(__file__).with_name("drive.eds")))
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--cmd", required=True,
                        choices=["info", "status", "enable", "disable",
                                 "mode", "pos", "vel", "monitor"])
    parser.add_argument("--value", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=5.0)
    return parser.parse_args()


def build_network(canopen_module, args):
    network = canopen_module.Network()
    channel = 0 if args.interface == "gs_usb" and args.port == "COM7" else args.port
    network.connect(interface=args.interface, channel=channel, bitrate=1_000_000)
    return network


def add_node(network, args):
    node = network.create_node(args.node, args.eds)
    node.nmt.state = "OPERATIONAL"
    return node


def read_status(node):
    return int(node.sdo[0x6041].raw)


def read_error(node):
    try:
        return int(node.sdo[0x603F].raw)
    except Exception:
        return 0xFFFF


def wait_status(node, expected, label, timeout):
    deadline = time.monotonic() + timeout
    last_status = 0
    last_exception = None
    while time.monotonic() < deadline:
        try:
            last_status = read_status(node)
            if (last_status & STATUS_MASK) == expected:
                print(f"{label}: statusword=0x{last_status:04X}")
                return last_status
        except Exception as exc:  # hardware backends expose different exception types
            last_exception = exc
        time.sleep(0.05)
    detail = f", last transport error={last_exception}" if last_exception else ""
    raise RuntimeError(
        f"timeout waiting for {label}: statusword=0x{last_status:04X}, "
        f"error=0x{read_error(node):04X}{detail}"
    )


def wait_ready(node, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            status = read_status(node)
            print(f"node responding: statusword=0x{status:04X}")
            return
        except Exception:
            time.sleep(0.1)
    raise RuntimeError("node did not answer SDO requests")


def cmd_info(node):
    try:
        name = node.sdo[0x1008].read().decode(errors="ignore")
    except Exception:
        name = "(0x1008 unavailable)"
    identity = node.sdo[0x1018]
    print("Device   :", name)
    print(f"Vendor   : 0x{identity[1].raw:08X}")
    print(f"Product  : 0x{identity[2].raw:08X}")
    print(f"Revision : 0x{identity[3].raw:08X}")
    print(f"Heartbeat: {node.sdo[0x1017].raw} ms")
    cmd_status(node)


def cmd_status(node):
    print(f"Statusword: 0x{read_status(node):04X}")
    print(f"Controlword: 0x{int(node.sdo[0x6040].raw):04X}")
    print(f"Mode: {int(node.sdo[0x6061].raw)}")
    print(f"Position: {int(node.sdo[0x6064].raw)} counts")
    print(f"Velocity: {int(node.sdo[0x606C].raw)} counts/s")
    print(f"Error: 0x{read_error(node):04X}")


def cmd_enable(node, timeout):
    sequence = (
        (0x0006, STATUS_READY_TO_SWITCH_ON, "ready-to-switch-on"),
        (0x0007, STATUS_SWITCHED_ON, "switched-on"),
        (0x000F, STATUS_OPERATION_ENABLED, "operation-enabled"),
    )
    for controlword, expected, label in sequence:
        node.sdo[0x6040].raw = controlword
        wait_status(node, expected, label, timeout)


def cmd_disable(node, timeout):
    node.sdo[0x60FF].raw = 0
    node.sdo[0x6040].raw = 0
    wait_status(node, STATUS_SWITCH_ON_DISABLED, "switch-on-disabled", timeout)


def cmd_mode(node, value):
    if value not in (1, 3, 6):
        raise ValueError("supported modes are 1 (PP), 3 (PV), and 6 (HM)")
    node.sdo[0x6060].raw = value
    print(f"Modes of operation -> {value}")


def cmd_position(node, value, timeout):
    cmd_mode(node, 1)
    cmd_enable(node, timeout)
    node.sdo[0x607A].raw = value
    node.sdo[0x6040].raw = 0x001F
    time.sleep(0.05)
    node.sdo[0x6040].raw = 0x000F
    print(f"Target position -> {value} counts")


def cmd_velocity(node, value, timeout):
    cmd_mode(node, 3)
    cmd_enable(node, timeout)
    node.sdo[0x60FF].raw = value
    print(f"Target velocity -> {value} counts/s")


def cmd_monitor(node, seconds):
    print("time(s)  status  error   position  velocity")
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        status = read_status(node)
        error = read_error(node)
        position = int(node.sdo[0x6064].raw)
        velocity = int(node.sdo[0x606C].raw)
        print(f"{time.monotonic() - start:6.2f}  0x{status:04X}  0x{error:04X}  "
              f"{position:9d}  {velocity:8d}")
        time.sleep(0.2)


def main():
    args = parse_args()
    try:
        import canopen
    except ModuleNotFoundError:
        print("Missing dependency. Run: pip install python-can canopen pyserial", file=sys.stderr)
        return 2

    network = build_network(canopen, args)
    try:
        node = add_node(network, args)
        wait_ready(node, args.timeout)
        if args.cmd == "info":
            cmd_info(node)
        elif args.cmd == "status":
            cmd_status(node)
        elif args.cmd == "enable":
            cmd_enable(node, args.timeout)
        elif args.cmd == "disable":
            cmd_disable(node, args.timeout)
        elif args.cmd == "mode":
            cmd_mode(node, args.value)
        elif args.cmd == "pos":
            cmd_position(node, args.value, args.timeout)
        elif args.cmd == "vel":
            cmd_velocity(node, args.value, args.timeout)
        elif args.cmd == "monitor":
            cmd_monitor(node, args.seconds)
    finally:
        network.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
