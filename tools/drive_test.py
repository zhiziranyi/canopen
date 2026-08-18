#!/usr/bin/env python3
"""
CANopen CiA402 FOC 伺服驱动 - 上位机测试脚本

依赖: pip install python-can canopen pyserial

示例 (USB-CAN 工具 slcan 固件, COM7):
  python drive_test.py --port COM7 --cmd info
  python drive_test.py --port COM7 --cmd mode --value 3
  python drive_test.py --port COM7 --cmd enable
  python drive_test.py --port COM7 --cmd pos --value 4096
  python drive_test.py --port COM7 --cmd vel --value 5000
  python drive_test.py --port COM7 --cmd disable

接口切换: --interface gs_usb / pcan / socketcan / slcan
"""

import argparse
import time

import canopen


def build_network(args):
    net = canopen.Network()
    kwargs = {}
    if args.interface == "slcan":
        kwargs["channel"] = args.port
        kwargs["bitrate"] = 1000000
    elif args.interface == "gs_usb":
        kwargs["channel"] = 0
        kwargs["bitrate"] = 1000000
    elif args.interface == "pcan":
        kwargs["channel"] = args.port
        kwargs["bitrate"] = 1000000
    else:
        kwargs["channel"] = args.port
        kwargs["bitrate"] = 1000000
    net.connect(interface=args.interface, **kwargs)
    return net


def add_node(net, args):
    node = net.create_node(args.node, args.eds)
    node.nmt.state = "OPERATIONAL"
    return node


def wait_ready(node, timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            node.sdo[0x6041].read()
            return
        except Exception:
            time.sleep(0.1)
    raise RuntimeError("node not responding")


def cmd_info(node):
    try:
        name = node.sdo[0x1008].read().decode(errors="ignore")
        print("Device:", name)
    except Exception:
        print("Device: (本固件未提供 0x1008 设备名)")
    identity = node.sdo[0x1018]
    print("Vendor : 0x%08X" % identity[1].raw)
    print("Product: 0x%08X" % identity[2].raw)
    print("Revision: 0x%08X" % identity[3].raw)
    print("Heartbeat: %d ms" % node.sdo[0x1017].raw)


def cmd_enable(node):
    node.sdo[0x6040].raw = 0x06   # shutdown
    time.sleep(0.05)
    node.sdo[0x6040].raw = 0x07   # switch on
    time.sleep(0.05)
    node.sdo[0x6040].raw = 0x0F   # enable operation
    sw = node.sdo[0x6041].raw
    print("Enabled, statusword=0x%04X" % sw)


def cmd_disable(node):
    node.sdo[0x6040].raw = 0x07
    print("Disabled")


def cmd_mode(node, value):
    node.sdo[0x6060].raw = value
    print("Modes of operation -> %d" % value)


def cmd_pos(node, value):
    node.sdo[0x607A].raw = value
    cw = node.sdo[0x6040].raw
    cw |= 0x10            # new set point
    node.sdo[0x6040].raw = cw
    time.sleep(0.05)
    cw &= ~0x10
    node.sdo[0x6040].raw = cw
    print("Target position -> %d" % value)


def cmd_vel(node, value):
    node.sdo[0x60FF].raw = value
    print("Target velocity -> %d" % value)


def cmd_monitor(node, seconds):
    print("time(s)  status   pos   vel")
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            sw = node.sdo[0x6041].raw
            pos = node.sdo[0x6064].raw
            vel = node.sdo[0x606C].raw
            print("%5.1f  0x%04X  %7d %8d" % (time.time() - t0, sw, pos, vel))
        except Exception:
            print("(read error)")
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description="CANopen servo drive test tool")
    parser.add_argument("--interface", default="slcan",
                        choices=["slcan", "gs_usb", "pcan", "socketcan"])
    parser.add_argument("--port", default="COM7", help="serial port or channel")
    parser.add_argument("--node", type=int, default=1)
    parser.add_argument("--eds", default="drive.eds")
    parser.add_argument("--cmd", required=True,
                        choices=["info", "enable", "disable", "mode", "pos", "vel", "monitor"])
    parser.add_argument("--value", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()

    net = build_network(args)
    try:
        node = add_node(net, args)
        wait_ready(node)
        if args.cmd == "info":
            cmd_info(node)
        elif args.cmd == "enable":
            cmd_enable(node)
        elif args.cmd == "disable":
            cmd_disable(node)
        elif args.cmd == "mode":
            cmd_mode(node, args.value)
        elif args.cmd == "pos":
            cmd_pos(node, args.value)
        elif args.cmd == "vel":
            cmd_vel(node, args.value)
        elif args.cmd == "monitor":
            cmd_monitor(node, args.seconds)
    finally:
        net.disconnect()


if __name__ == "__main__":
    main()
