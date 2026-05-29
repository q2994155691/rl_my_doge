#!/usr/bin/env python3

import json
import math
from pathlib import Path
import sys

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class ImuRviz2Visualizer(Node):
    def __init__(self):
        super().__init__("imu_rviz2_visualizer")

        default_imu_python_path = (
            Path(__file__).resolve().parents[1]
            / "library"
            / "thirdparty"
            / "test_motor_thread"
            / "bin"
        )
        self.declare_parameter("imu_python_path", str(default_imu_python_path))
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("axis_length", 0.35)

        imu_python_path = self.get_parameter("imu_python_path").value
        if imu_python_path not in sys.path:
            sys.path.append(imu_python_path)

        from imu_interface import get_imu_data  # pylint: disable=import-error,import-outside-toplevel

        self.get_imu_data = get_imu_data
        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.axis_length = float(self.get_parameter("axis_length").value)

        self.imu_pub = self.create_publisher(Imu, "/opus_imu/data", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/opus_imu/axes", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(period, self.update)
        self.last_log_time = self.get_clock().now()

        self.get_logger().info(
            "Publishing /opus_imu/data, /opus_imu/axes and TF "
            f"{self.fixed_frame} -> {self.imu_frame}"
        )

    @staticmethod
    def _normalize_quat(w, x, y, z):
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm < 1e-6:
            return 1.0, 0.0, 0.0, 0.0, 0.0
        return w / norm, x / norm, y / norm, z / norm, norm

    def update(self):
        try:
            raw = self.get_imu_data()
            data = json.loads(raw) if isinstance(raw, str) else raw
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Failed to read IMU: {exc}")
            return

        qw, qx, qy, qz, qnorm = self._normalize_quat(
            float(data.get("qw", 1.0)),
            float(data.get("qx", 0.0)),
            float(data.get("qy", 0.0)),
            float(data.get("qz", 0.0)),
        )

        now = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.imu_frame
        imu_msg.orientation.w = qw
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.angular_velocity.x = float(data.get("RollSpeed", 0.0))
        imu_msg.angular_velocity.y = float(data.get("PitchSpeed", 0.0))
        imu_msg.angular_velocity.z = float(data.get("HeadingSpeed", 0.0))
        imu_msg.linear_acceleration.x = float(data.get("Accelerometer_X", 0.0))
        imu_msg.linear_acceleration.y = float(data.get("Accelerometer_Y", 0.0))
        imu_msg.linear_acceleration.z = float(data.get("Accelerometer_Z", 0.0))
        self.imu_pub.publish(imu_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.fixed_frame
        tf_msg.child_frame_id = self.imu_frame
        tf_msg.transform.rotation = imu_msg.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        self.marker_pub.publish(self.make_axes(now))

        log_now = self.get_clock().now()
        if (log_now - self.last_log_time).nanoseconds > 1_000_000_000:
            self.last_log_time = log_now
            self.get_logger().info(
                "q_wxyz=[%.3f %.3f %.3f %.3f] norm=%.4f gyro_xyz=[%.3f %.3f %.3f]"
                % (
                    qw,
                    qx,
                    qy,
                    qz,
                    qnorm,
                    imu_msg.angular_velocity.x,
                    imu_msg.angular_velocity.y,
                    imu_msg.angular_velocity.z,
                )
            )

    def make_axes(self, stamp):
        axes = [
            (0, "x_forward", (self.axis_length, 0.0, 0.0), (1.0, 0.05, 0.05, 1.0)),
            (1, "y_left", (0.0, self.axis_length, 0.0), (0.05, 0.9, 0.05, 1.0)),
            (2, "z_up", (0.0, 0.0, self.axis_length), (0.1, 0.3, 1.0, 1.0)),
        ]

        marker_array = MarkerArray()
        for marker_id, name, end, color in axes:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.imu_frame
            marker.ns = name
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=end[0], y=end[1], z=end[2])]
            marker.scale.x = 0.025
            marker.scale.y = 0.055
            marker.scale.z = 0.08
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            marker_array.markers.append(marker)

        return marker_array


def main():
    rclpy.init()
    node = ImuRviz2Visualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
