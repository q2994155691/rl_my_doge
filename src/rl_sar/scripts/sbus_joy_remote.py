#!/usr/bin/env python3
"""Convert Microzone SBUS channels into rl_sar /joy commands."""

import argparse
import importlib.util
import sys
import time
from pathlib import Path

import serial


SBUS_FRAME_LEN = 25
SBUS_START = 0x0F
DEFAULT_SBMON = "/home/luis/microzone_rc/sbus_monitor.py"


def load_sbus_monitor(path):
    spec = importlib.util.spec_from_file_location("sbus_monitor", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load sbus monitor from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def scale_stick(raw, lo=200, center=1000, hi=1800, dead=20, invert=False):
    if center - dead <= raw <= center + dead:
        value = 0.0
    elif raw < center - dead:
        value = (raw - (center - dead)) / float((center - dead) - lo)
    else:
        value = (raw - (center + dead)) / float(hi - (center + dead))

    value = max(-1.0, min(1.0, value))
    return -value if invert else value


def switch_pos(raw, up_threshold=600, down_threshold=1400):
    if raw < up_threshold:
        return "up"
    if raw > down_threshold:
        return "down"
    return "mid"


def make_joy(x, y, yaw, event):
    axes = [0.0] * 8
    buttons = [0] * 11

    # rl_sar JoyCallback expects F710 layout:
    # axes[0]=LX -> control.y, axes[1]=LY -> control.x, axes[3]=RX -> control.yaw.
    axes[0] = y
    axes[1] = x
    axes[3] = yaw

    if event == "stand_up":
        buttons[0] = 1  # A -> RLFSMStateGetUp
    elif event == "rl_mode":
        buttons[5] = 1  # RB
        axes[7] = 1.0   # RB + DPadUp -> RLFSMStateRLLocomotion
    elif event == "get_down":
        buttons[1] = 1  # B -> RLFSMStateGetDown
    elif event == "passive":
        buttons[4] = 1  # LB
        buttons[2] = 1  # LB + X -> RLFSMStatePassive

    return axes, buttons


def choose_event(ch5_pos, ch6_pos):
    # CH6 raw high is the physical down/passive position on this transmitter.
    if ch6_pos == "down":
        return "passive"
    if ch6_pos == "up":
        return "get_down"
    if ch5_pos == "up":
        return "stand_up"
    if ch5_pos == "down":
        return "rl_mode"
    return None


def make_publisher(ros_version, topic):
    if ros_version == "none":
        return None, None

    if ros_version in ("auto", "ros2"):
        try:
            import rclpy
            from sensor_msgs.msg import Joy

            rclpy.init(args=None)
            node = rclpy.create_node("sbus_joy_remote")
            pub = node.create_publisher(Joy, topic, 10)

            def publish(axes, buttons):
                msg = Joy()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = "sbus"
                msg.axes = axes
                msg.buttons = buttons
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.0)

            return publish, lambda: (node.destroy_node(), rclpy.shutdown())
        except ImportError:
            if ros_version == "ros2":
                raise

    if ros_version in ("auto", "ros1"):
        try:
            import rospy
            from sensor_msgs.msg import Joy

            rospy.init_node("sbus_joy_remote", anonymous=True, disable_signals=True)
            pub = rospy.Publisher(topic, Joy, queue_size=10)

            def publish(axes, buttons):
                msg = Joy()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "sbus"
                msg.axes = axes
                msg.buttons = buttons
                pub.publish(msg)

            return publish, lambda: None
        except ImportError:
            if ros_version == "ros1":
                raise

    raise RuntimeError("Neither rclpy nor rospy is importable. Use --dry-run to test mapping only.")


def main():
    parser = argparse.ArgumentParser(description="Publish rl_sar /joy commands from Microzone SBUS.")
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        help="SBUS serial device",
    )
    parser.add_argument("--sbus-monitor", default=DEFAULT_SBMON, help="path to sbus_monitor.py")
    parser.add_argument("--topic", default="/joy", help="Joy topic, default: /joy")
    parser.add_argument("--ros", choices=["auto", "ros1", "ros2", "none"], default="auto")
    parser.add_argument("--dry-run", action="store_true", help="print mapped commands without publishing ROS")
    parser.add_argument("--rate", type=float, default=50.0, help="max publish/log rate in Hz")
    parser.add_argument("--deadband", type=int, default=20, help="stick center deadband around 1000")
    parser.add_argument("--x-channel", type=int, default=2, help="1-based channel for x command")
    parser.add_argument("--y-channel", type=int, default=4, help="1-based channel for y command")
    parser.add_argument("--yaw-channel", type=int, default=1, help="1-based channel for yaw command")
    parser.add_argument("--ch5", type=int, default=5, help="1-based channel for switch 5")
    parser.add_argument("--ch6", type=int, default=6, help="1-based channel for switch 6")
    parser.add_argument("--invert-x", action="store_true")
    parser.add_argument("--invert-y", action="store_true")
    parser.add_argument("--invert-yaw", action="store_true")
    args = parser.parse_args()

    sbus_path = Path(args.sbus_monitor)
    if not sbus_path.exists():
        raise FileNotFoundError(sbus_path)

    sbus_monitor = load_sbus_monitor(str(sbus_path))
    publish, shutdown = (None, None) if args.dry_run else make_publisher(args.ros, args.topic)

    ser = serial.Serial(
        args.port,
        baudrate=100000,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_TWO,
        timeout=0.02,
    )

    buffer = bytearray()
    min_period = 1.0 / args.rate if args.rate > 0 else 0.0
    last_emit = 0.0
    print(f"Reading SBUS from {args.port}; publishing {args.topic if publish else 'dry-run output'}.")
    print("Mapping: CH%d->x CH%d->y CH%d->yaw CH%d/CH%d->FSM" %
          (args.x_channel, args.y_channel, args.yaw_channel, args.ch5, args.ch6))

    try:
        while True:
            data = ser.read(256)
            if data:
                buffer.extend(data)

            frames = sbus_monitor.find_frames(buffer)
            if not frames:
                continue

            now = time.monotonic()
            if now - last_emit < min_period:
                continue

            decoded = sbus_monitor.decode_sbus(frames[-1])
            channels = decoded["channels"]
            if decoded["failsafe"] or decoded["lost_frame"]:
                x = y = yaw = 0.0
                event = "passive"
            else:
                x = scale_stick(channels[args.x_channel - 1], dead=args.deadband, invert=args.invert_x)
                y = scale_stick(channels[args.y_channel - 1], dead=args.deadband, invert=args.invert_y)
                yaw = scale_stick(channels[args.yaw_channel - 1], dead=args.deadband, invert=args.invert_yaw)
                ch5_pos = switch_pos(channels[args.ch5 - 1])
                ch6_pos = switch_pos(channels[args.ch6 - 1])
                event = choose_event(ch5_pos, ch6_pos)

            # Hold the virtual button while the switch remains in position.
            # rl_sar clears button input every control cycle, so pulses can be missed.
            axes, buttons = make_joy(x, y, yaw, event)

            if publish:
                publish(axes, buttons)
            else:
                ch5_pos = switch_pos(channels[args.ch5 - 1])
                ch6_pos = switch_pos(channels[args.ch6 - 1])
                print(
                    f"x={x:+.2f} y={y:+.2f} yaw={yaw:+.2f} "
                    f"ch5={channels[args.ch5 - 1]:4d}/{ch5_pos:4s} "
                    f"ch6={channels[args.ch6 - 1]:4d}/{ch6_pos:4s} "
                    f"event={event or '-'}"
                )

            last_emit = now
    except KeyboardInterrupt:
        print()
    finally:
        ser.close()
        if shutdown:
            shutdown()


if __name__ == "__main__":
    sys.exit(main())
