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

// Adapted from rm-simulator c3784e168cf21167f15c8728ba8e8edb8d1d80a9,
// integrations/ros2/src/publisher.rs (same MIT OR Apache-2.0 licensing).
/// SI inertial publisher with no orientation estimate or privileged world pose.
pub struct ImuPublisher {
    publisher: DynamicPublisher,
    frame_id: String,
}
impl ImuPublisher {
    /// Create a best-effort volatile IMU endpoint (depth 200) in a named sensor frame.
    pub fn new(node: &Node, topic: &str, frame_id: &str) -> Result<Self> {
        anyhow::ensure!(
            !frame_id.is_empty() && !frame_id.contains('\0'),
            "invalid IMU frame ID"
        );
        Ok(Self {
            publisher: node.create_dynamic_publisher(
                MessageTypeName::try_from("sensor_msgs/msg/Imu")?,
                topic.keep_last(200).best_effort(),
            )?,
            frame_id: frame_id.into(),
        })
    }
    /// Current IMU subscription count for bounded startup discovery.
    pub fn subscription_count(&self) -> Result<usize> {
        Ok(self.publisher.get_subscription_count()?)
    }
    /// Publish rad/s and m/s² at the actual simulation sample time in nanoseconds.
    /// `orientation_covariance[0]` is -1 (unavailable); other zero covariances mean
    /// unknown, not perfect measurements. No acceleration conversion to g occurs.
    pub fn publish(&self, sample: &crate::imu::ImuSample) -> Result<()> {
        sample.validate().map_err(anyhow::Error::msg)?;
        let stamp = RosTime::from_ns(sample.time_ns).map_err(anyhow::Error::msg)?;
        let mut msg = DynamicMessage::new(MessageTypeName::try_from("sensor_msgs/msg/Imu")?)?;
        if let Some(ValueMut::Simple(SimpleValueMut::Message(mut header))) = msg.get_mut("header") {
            scalar!(header, "frame_id", String, self.frame_id.as_str().into());
            if let Some(ValueMut::Simple(SimpleValueMut::Message(mut time))) =
                header.get_mut("stamp")
            {
                scalar!(time, "sec", Int32, stamp.sec);
                scalar!(time, "nanosec", Uint32, stamp.nanosec);
            } else {
                bail!("IMU stamp schema mismatch");
            }
        } else {
            bail!("IMU header schema mismatch");
        }
        for (field, values) in [
            ("angular_velocity", sample.angular_velocity_rad_s),
            ("linear_acceleration", sample.specific_force_m_s2),
        ] {
            if let Some(ValueMut::Simple(SimpleValueMut::Message(mut vector))) = msg.get_mut(field)
            {
                scalar!(vector, "x", Double, values[0]);
                scalar!(vector, "y", Double, values[1]);
                scalar!(vector, "z", Double, values[2]);
            } else {
                bail!("IMU vector schema mismatch");
            }
        }
        if let Some(ValueMut::Array(rclrs::ArrayValueMut::DoubleArray(values))) =
            msg.get_mut("orientation_covariance")
        {
            values[0] = -1.;
        } else {
            bail!("IMU covariance schema mismatch");
        }
        self.publisher.publish(msg)?;
        Ok(())
    }
}
