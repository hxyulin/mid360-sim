#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Optional ROS adapter for measured RM bundles. No world or dynamics are created.

Used by lio.py; native Rust publishers remain the primary ROS integration.
"""

import struct

from live import Validator


def multiply(a, b):
    """Hamilton product of unit xyzw quaternions (dimensionless)."""
    x, y, z, w = a
    X, Y, Z, W = b
    return [
        w * X + x * W + y * Z - z * Y,
        w * Y - x * Z + y * W + z * X,
        w * Z + x * Y - y * X + z * W,
        w * W - x * X - y * Y - z * Z,
    ]


def inverse(q):
    return [-q[0], -q[1], -q[2], q[3]]


def rotate(q, v):
    return multiply(multiply(q, [*v, 0.0]), inverse(q))[:3]


def pose_from_rm(pose):
    w, x, y, z = pose["rotation_wxyz"]
    return pose["translation_m"], [x, y, z, w]


def relative(parent, child):
    """Return child-to-parent translation (metres) and xyzw rotation."""
    p, q = parent
    v, r = child
    return rotate(inverse(q), [v[i] - p[i] for i in range(3)]), multiply(inverse(q), r)


def stamp(header, ns, frame):
    header.frame_id = frame
    header.stamp.sec, header.stamp.nanosec = divmod(ns, 10**9)


class RosStream:
    """Publish measured SI IMU and raw timed PointCloud2; no world TF or odometry."""

    def __init__(self, node, config, prefix="rm", publish_clock=False):
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, PointCloud2
        from tf2_msgs.msg import TFMessage

        self.node, self.config, self.prefix = node, config, prefix
        self.validator = Validator(config)
        self.imu = node.create_publisher(Imu, "/synchronized/imu", 200)
        self.cloud = node.create_publisher(PointCloud2, "/synchronized/points", 5)
        self.clock = (
            node.create_publisher(Clock, "/clock", 1) if publish_clock else None
        )
        self.fixed = node.create_publisher(
            TFMessage,
            "/tf_static",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.moving = node.create_publisher(TFMessage, "/tf", 100)
        self.last_joint_stamp = None
        self.dropped = 0

    def mounts(self):
        """Host defaults or explicit carrier-to-sensor mounts, metres/xyzw."""
        default = {
            "translation_m": [0.0, 0.0, 0.4],
            "rotation_wxyz": [1.0, 0.0, 0.0, 0.0],
        }
        return [
            pose_from_rm(self.config.get(name, default))
            for name in ("lidar_mount", "imu_mount")
        ]

    def transform(self, parent, child, ns, pose):
        from geometry_msgs.msg import TransformStamped

        msg = TransformStamped()
        stamp(msg.header, ns, parent)
        msg.child_frame_id = child
        p, q = pose
        (
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        ) = p
        (
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ) = q
        return msg

    def publish_mounts(self):
        from tf2_msgs.msg import TFMessage

        carrier = self.prefix + (
            "_gimbal" if self.config.get("carrier", "Chassis") == "Gimbal" else "_body"
        )
        self.fixed.publish(
            TFMessage(
                transforms=[
                    self.transform(carrier, self.prefix + "_" + name, 0, pose)
                    for name, pose in zip(("lidar", "imu"), self.mounts())
                ]
            )
        )

    def publish_joint(self, time_ns, body, gimbal):
        """Actual relative joint pose at its snapshot stamp; never requested aim.

        Caller must supply same-snapshot body/gimbal poses. This encoder surrogate
        is ideal simulator state, not a calibrated physical joint measurement.
        """
        from tf2_msgs.msg import TFMessage

        if self.config.get("carrier", "Chassis") != "Gimbal":
            return
        if self.last_joint_stamp is not None and time_ns <= self.last_joint_stamp:
            return
        self.last_joint_stamp = time_ns
        self.moving.publish(
            TFMessage(
                transforms=[
                    self.transform(
                        self.prefix + "_body",
                        self.prefix + "_gimbal",
                        time_ns,
                        relative(body, gimbal),
                    )
                ]
            )
        )

    def publish(self, batch):
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, PointCloud2, PointField

        self.dropped = max(self.dropped, batch["dropped_intervals"])
        for frame in batch["frames"]:
            self.validator.accept(frame, batch["time_ns"])
            for s in frame["imu"]:
                m = Imu()
                stamp(m.header, s["time_ns"], self.prefix + "_imu")
                m.orientation_covariance[0] = -1.0
                m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = s[
                    "angular_velocity_rad_s"
                ]
                (
                    m.linear_acceleration.x,
                    m.linear_acceleration.y,
                    m.linear_acceleration.z,
                ) = s["specific_force_m_s2"]
                self.imu.publish(m)
            points = frame["lidar"]["points"]
            base = points[0]["time_ns"]
            data = bytearray()
            for p in points:
                if p["point_sensor_m"] is not None:
                    data.extend(
                        struct.pack(
                            "<fffIBBBx",
                            *p["point_sensor_m"],
                            p["time_ns"] - base,
                            100,
                            0x10,
                            p["sample_index"] % 4,
                        )
                    )
            m = PointCloud2()
            stamp(m.header, base, self.prefix + "_lidar")
            m.height, m.width = 1, len(data) // 20
            m.point_step, m.row_step, m.is_dense = 20, len(data), True
            m.fields = [
                PointField(name=n, offset=o, datatype=t, count=1)
                for n, o, t in [
                    ("x", 0, 7),
                    ("y", 4, 7),
                    ("z", 8, 7),
                    ("offset_time", 12, 6),
                    ("reflectivity", 16, 2),
                    ("tag", 17, 2),
                    ("line", 18, 2),
                ]
            ]
            m.data = bytes(data)
            self.cloud.publish(m)
        if self.clock:
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(batch["time_ns"], 10**9)
            self.clock.publish(clock)
