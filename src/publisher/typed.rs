// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Optional generated RMW-native Livox endpoint; interfaces remain environment-owned.
use crate::{LivoxPacket, RosTime, sensor::MAX_LIDAR_RAYS};
use anyhow::{Result, ensure};
use rclrs::{
    DynamicMessageMetadata, DynamicPublisher, IntoPrimitiveOptions, MessageTypeName, Node,
    Publisher,
};
use ros_env::livox_ros_driver2::msg::rmw::{CustomMsg, CustomPoint};

/// Typed Driver 2 `CustomMsg` publisher, enabled by `ros2-typed`.
///
/// Requires generated `livox_ros_driver2` Rust interfaces exported through
/// `ros-env` in the sourced ROS workspace. Uses fresh owned C message storage,
/// including exact-length empty and singleton sequences. No local unsafe code
/// or borrowed measurement buffers are used. The dynamic [`super::LivoxPublisher`]
/// remains available for C-introspection-only installations.
///
/// Points use best-effort volatile depth 5; `/clock` uses depth 1. Publication
/// may block in the RMW; call outside the physics thread or use a bounded queue.
///
/// ```no_run
/// # fn publish(node: &rclrs::Node, frame: &mid360_sim::sensor::RollingFrame) -> anyhow::Result<()> {
/// use mid360_sim::{LivoxConfig, LivoxPacket, publisher::TypedLivoxPublisher};
/// let publisher = TypedLivoxPublisher::new(node, "/livox/lidar")?;
/// let packet = LivoxPacket::from_scan(frame, &LivoxConfig::default())
///     .map_err(anyhow::Error::msg)?;
/// publisher.publish(&packet)?;
/// # Ok(()) }
/// ```
pub struct TypedLivoxPublisher {
    points: Publisher<CustomMsg>,
    clock: DynamicPublisher,
    clock_metadata: DynamicMessageMetadata,
}
impl TypedLivoxPublisher {
    /// Create endpoints using installed, unmodified Driver 2 and clock interfaces.
    pub fn new(node: &Node, topic: &str) -> Result<Self> {
        let clock_type = MessageTypeName::try_from("rosgraph_msgs/msg/Clock")?;
        Ok(Self {
            points: node.create_publisher::<CustomMsg>(topic.keep_last(5).best_effort())?,
            clock_metadata: DynamicMessageMetadata::new(clock_type.clone())?,
            clock: node
                .create_dynamic_publisher(clock_type, "/clock".keep_last(1).best_effort())?,
        })
    }

    /// Current point-cloud subscriber count, useful for bounded startup waits.
    pub fn subscription_count(&self) -> Result<usize> {
        Ok(self.points.get_subscription_count()?)
    }

    /// Publish raw emission-local XYZ in metres and unchanged nanosecond offsets.
    ///
    /// Start with [`LivoxPacket::from_scan`]. Empty/all-miss and singleton packets
    /// are supported. Header/timebase, frame, count, finite XYZ and strictly
    /// increasing retained offsets are checked before allocating message storage.
    /// Reflectivity, tags, lanes and device ID are copied as supplied; no physical
    /// intensity model, pose transformation, time renumbering or padding is added.
    /// Reserved bytes are zero. Invalid packets are rejected before publication
    /// and leave this endpoint usable; the caller retains ownership of the packet
    /// on success and failure. RMW publication errors are propagated.
    pub fn publish(&self, packet: &LivoxPacket) -> Result<()> {
        self.points.publish(message(packet)?)?;
        Ok(())
    }

    /// Publish explicit simulation time in nanoseconds, without a wall-clock substitute.
    pub fn publish_clock(&self, time_ns: u64) -> Result<()> {
        super::publish_clock(&self.clock, &self.clock_metadata, time_ns)
    }
}

fn message(packet: &LivoxPacket) -> Result<CustomMsg> {
    ensure!(
        packet.points.len() <= MAX_LIDAR_RAYS,
        "LiDAR packet exceeds allocation budget"
    );
    ensure!(
        RosTime::from_ns(packet.timebase_ns).map_err(anyhow::Error::msg)? == packet.stamp,
        "LiDAR header/timebase mismatch"
    );
    ensure!(
        !packet.config.frame_id.is_empty() && !packet.config.frame_id.contains('\0'),
        "invalid LiDAR frame ID"
    );
    let mut previous = None;
    for p in &packet.points {
        ensure!(
            [p.x_m, p.y_m, p.z_m].iter().all(|v| v.is_finite())
                && previous.is_none_or(|t| p.offset_time_ns > t),
            "nonfinite or nonmonotonic LiDAR point"
        );
        previous = Some(p.offset_time_ns);
    }
    let mut msg = CustomMsg::default();
    msg.header.stamp.sec = packet.stamp.sec;
    msg.header.stamp.nanosec = packet.stamp.nanosec;
    msg.header.frame_id = packet.config.frame_id.as_str().into();
    msg.timebase = packet.timebase_ns;
    msg.point_num = packet.points.len() as u32;
    msg.lidar_id = packet.config.lidar_id;
    msg.points = packet
        .points
        .iter()
        .map(|p| CustomPoint {
            offset_time: p.offset_time_ns,
            x: p.x_m,
            y: p.y_m,
            z: p.z_m,
            reflectivity: p.reflectivity,
            tag: p.tag,
            line: p.line,
        })
        .collect();
    Ok(msg)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{LivoxConfig, LivoxPoint};

    fn packet(n: usize) -> LivoxPacket {
        LivoxPacket {
            config: LivoxConfig {
                frame_id: "typed_lidar".into(),
                lidar_id: 17,
                reflectivity: 100,
            },
            stamp: RosTime {
                sec: 2,
                nanosec: 123,
            },
            timebase_ns: 2_000_000_123,
            points: (0..n)
                .map(|i| LivoxPoint {
                    offset_time_ns: (i as u32 + 1) * 15_000,
                    x_m: -0.,
                    y_m: i as f32,
                    z_m: -1.,
                    reflectivity: 83,
                    tag: 0x10,
                    line: (i % 4) as u8,
                })
                .collect(),
        }
    }

    #[test]
    fn owned_message_preserves_fields_gaps_and_source_storage() {
        for n in [0, 1, 2, 20_000] {
            let mut packet = packet(n);
            let msg = message(&packet).unwrap();
            assert_eq!((msg.header.stamp.sec, msg.header.stamp.nanosec), (2, 123));
            assert_eq!(msg.header.frame_id.to_string(), "typed_lidar");
            assert_eq!(
                (msg.timebase, msg.point_num, msg.lidar_id, msg.rsvd),
                (2_000_000_123, n as u32, 17, [0; 3])
            );
            assert_eq!(msg.points.len(), n);
            for (a, b) in msg.points.iter().zip(&packet.points) {
                assert_eq!(
                    (a.x.to_bits(), a.y.to_bits(), a.z.to_bits()),
                    (b.x_m.to_bits(), b.y_m.to_bits(), b.z_m.to_bits())
                );
                assert_eq!(
                    (a.offset_time, a.reflectivity, a.tag, a.line),
                    (b.offset_time_ns, b.reflectivity, b.tag, b.line)
                );
            }
            packet.points.clear();
            assert_eq!(msg.points.len(), n);
        }
    }

    #[test]
    fn rejects_mutated_packets_before_encoding() {
        let original = packet(2);
        let mut invalid = Vec::new();
        let mut p = original.clone();
        p.stamp.nanosec += 1;
        invalid.push(p);
        let mut p = original.clone();
        p.timebase_ns = u64::MAX;
        invalid.push(p);
        for frame in ["", "bad\0frame"] {
            let mut p = original.clone();
            p.config.frame_id = frame.into();
            invalid.push(p);
        }
        let mut p = original.clone();
        p.points[1].offset_time_ns = p.points[0].offset_time_ns;
        invalid.push(p);
        let mut p = original.clone();
        p.points[1].x_m = f32::NAN;
        invalid.push(p);
        let mut p = original.clone();
        p.points[1].z_m = f32::INFINITY;
        invalid.push(p);
        let mut p = original.clone();
        p.points.resize(MAX_LIDAR_RAYS + 1, p.points[0]);
        invalid.push(p);
        for p in invalid {
            assert!(message(&p).is_err());
        }
        assert!(message(&original).is_ok());
    }
}
