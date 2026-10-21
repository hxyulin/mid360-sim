// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Safe rclrs dynamic publishers using installed standard/Livox C introspection.
//! No generated Rust bindings or unsafe code. Publishing may block in the RMW;
//! call outside a shared simulation worker or provide caller-owned bounded queues.
use crate::{LivoxPacket, RosTime};
use anyhow::{Result, bail};
use rclrs::{
    DynamicMessage, DynamicPublisher, IntoPrimitiveOptions, MessageTypeName, Node,
    SequenceValueMut, SimpleValueMut, ValueMut,
};
macro_rules! scalar {
    ($view:expr,$name:expr,$kind:ident,$value:expr) => {
        match $view.get_mut($name) {
            Some(ValueMut::Simple(SimpleValueMut::$kind(v))) => *v = $value,
            _ => bail!("ROS message schema mismatch at {}", $name),
        }
    };
}
/// Publishers for Driver 2 `CustomMsg` and ROS simulation clock. Points use
/// best-effort volatile depth 5; clock uses best-effort volatile depth 1.
pub struct LivoxPublisher {
    points: DynamicPublisher,
    clock: DynamicPublisher,
}
impl LivoxPublisher {
    /// Create DDS endpoints. Installed interfaces must contain unmodified
    /// `livox_ros_driver2/msg/CustomMsg` and `rosgraph_msgs/msg/Clock`.
    pub fn new(node: &Node, topic: &str) -> Result<Self> {
        Ok(Self {
            points: node.create_dynamic_publisher(
                MessageTypeName::try_from("livox_ros_driver2/msg/CustomMsg")?,
                topic.keep_last(5).best_effort(),
            )?,
            clock: node.create_dynamic_publisher(
                MessageTypeName::try_from("rosgraph_msgs/msg/Clock")?,
                "/clock".keep_last(1).best_effort(),
            )?,
        })
    }
    /// Current DDS point-cloud subscriber count, useful for bounded startup waits.
    pub fn subscription_count(&self) -> Result<usize> {
        Ok(self.points.get_subscription_count()?)
    }
    /// Publish validated measured points. Header/timebase use simulation time;
    /// data remain distorted local sensor points with original per-point offsets.
    /// Reflectivity/tag/lane are explicit simulation policies, not sensor truth.
    pub fn publish(&self, packet: &LivoxPacket) -> Result<()> {
        let mut msg = DynamicMessage::new(MessageTypeName::try_from(
            "livox_ros_driver2/msg/CustomMsg",
        )?)?;
        scalar!(msg, "timebase", Uint64, packet.timebase_ns);
        scalar!(
            msg,
            "point_num",
            Uint32,
            u32::try_from(packet.points.len())?
        );
        scalar!(msg, "lidar_id", Uint8, packet.config.lidar_id);
        // rsvd is initialized to zero by the ROS generated message constructor.
        if let Some(ValueMut::Simple(SimpleValueMut::Message(mut header))) = msg.get_mut("header") {
            scalar!(
                header,
                "frame_id",
                String,
                packet.config.frame_id.as_str().into()
            );
            if let Some(ValueMut::Simple(SimpleValueMut::Message(mut stamp))) =
                header.get_mut("stamp")
            {
                scalar!(stamp, "sec", Int32, packet.stamp.sec);
                scalar!(stamp, "nanosec", Uint32, packet.stamp.nanosec);
            } else {
                bail!("ROS header stamp schema mismatch");
            }
        } else {
            bail!("ROS header schema mismatch");
        }
        if let Some(ValueMut::Sequence(SequenceValueMut::MessageSequence(mut points))) =
            msg.get_mut("points")
        {
            points.reset(packet.points.len());
            for (view, p) in points.as_mut_slice().iter_mut().zip(&packet.points) {
                scalar!(view, "offset_time", Uint32, p.offset_time_ns);
                scalar!(view, "x", Float, p.x_m);
                scalar!(view, "y", Float, p.y_m);
                scalar!(view, "z", Float, p.z_m);
                scalar!(view, "reflectivity", Uint8, p.reflectivity);
                scalar!(view, "tag", Uint8, p.tag);
                scalar!(view, "line", Uint8, p.line);
            }
        } else {
            bail!("ROS points sequence schema mismatch");
        }
        self.points.publish(msg)?;
        Ok(())
    }
    /// Publish explicit simulation time. No wall-clock timestamp is substituted.
    pub fn publish_clock(&self, time_ns: u64) -> Result<()> {
        let time = RosTime::from_ns(time_ns).map_err(anyhow::Error::msg)?;
        let mut msg = DynamicMessage::new(MessageTypeName::try_from("rosgraph_msgs/msg/Clock")?)?;
        if let Some(ValueMut::Simple(SimpleValueMut::Message(mut clock))) = msg.get_mut("clock") {
            scalar!(clock, "sec", Int32, time.sec);
            scalar!(clock, "nanosec", Uint32, time.nanosec);
        } else {
            bail!("ROS clock schema mismatch");
        }
        self.clock.publish(msg)?;
        Ok(())
    }
}
