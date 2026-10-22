// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Safe rclrs dynamic publishers using installed standard/Livox C introspection.
//! No generated Rust bindings or unsafe code. Publishing may block in the RMW;
//! call outside a shared simulation worker or provide caller-owned bounded queues.
//! Each endpoint retains message metadata; each publish creates a fresh owned
//! message because rclrs consumes it when publishing.
use crate::{LivoxPacket, RosTime};
use anyhow::{Result, bail};
use rclrs::{
    DynamicMessageMetadata, DynamicPublisher, IntoPrimitiveOptions, MessageTypeName, Node,
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
    points_metadata: DynamicMessageMetadata,
    clock_metadata: DynamicMessageMetadata,
}
impl LivoxPublisher {
    /// Create DDS endpoints. Installed interfaces must contain unmodified
    /// `livox_ros_driver2/msg/CustomMsg` and `rosgraph_msgs/msg/Clock`.
    pub fn new(node: &Node, topic: &str) -> Result<Self> {
        let points_type = MessageTypeName::try_from("livox_ros_driver2/msg/CustomMsg")?;
        let clock_type = MessageTypeName::try_from("rosgraph_msgs/msg/Clock")?;
        Ok(Self {
            points_metadata: DynamicMessageMetadata::new(points_type.clone())?,
            clock_metadata: DynamicMessageMetadata::new(clock_type.clone())?,
            points: node.create_dynamic_publisher(points_type, topic.keep_last(5).best_effort())?,
            clock: node
                .create_dynamic_publisher(clock_type, "/clock".keep_last(1).best_effort())?,
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
        let mut msg = self.points_metadata.create()?;
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
        let mut msg = self.clock_metadata.create()?;
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
    metadata: DynamicMessageMetadata,
    frame_id: String,
}
impl ImuPublisher {
    /// Create a best-effort volatile IMU endpoint (depth 200) in a named sensor frame.
    pub fn new(node: &Node, topic: &str, frame_id: &str) -> Result<Self> {
        Self::new_with_qos(
            node,
            topic,
            frame_id,
            rclrs::QoSProfile::sensor_data_default().keep_last(200),
        )
    }
    /// Create an IMU endpoint with caller-selected delivery QoS; units remain SI.
    pub fn new_with_qos(
        node: &Node,
        topic: &str,
        frame_id: &str,
        qos: rclrs::QoSProfile,
    ) -> Result<Self> {
        anyhow::ensure!(
            !frame_id.is_empty() && !frame_id.contains('\0'),
            "invalid IMU frame ID"
        );
        let ty = MessageTypeName::try_from("sensor_msgs/msg/Imu")?;
        Ok(Self {
            metadata: DynamicMessageMetadata::new(ty.clone())?,
            publisher: node.create_dynamic_publisher(ty, topic.qos(qos))?,
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
        let mut msg = self.metadata.create()?;
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

/// Standard PointCloud2 publisher with explicit nanosecond timing fields.
pub struct PointCloudPublisher {
    publisher: DynamicPublisher,
    metadata: DynamicMessageMetadata,
}
impl PointCloudPublisher {
    /// Create a best-effort volatile cloud endpoint with depth 5.
    pub fn new(node: &Node, topic: &str) -> Result<Self> {
        Self::new_with_qos(
            node,
            topic,
            rclrs::QoSProfile::sensor_data_default().keep_last(5),
        )
    }
    /// Create a cloud endpoint with caller-selected delivery QoS.
    pub fn new_with_qos(node: &Node, topic: &str, qos: rclrs::QoSProfile) -> Result<Self> {
        let ty = MessageTypeName::try_from("sensor_msgs/msg/PointCloud2")?;
        Ok(Self {
            metadata: DynamicMessageMetadata::new(ty.clone())?,
            publisher: node.create_dynamic_publisher(ty, topic.qos(qos))?,
        })
    }
    /// Current cloud subscription count for bounded discovery waits.
    pub fn subscription_count(&self) -> Result<usize> {
        Ok(self.publisher.get_subscription_count()?)
    }
    /// Publish raw, unorganized local points in metres. Header is the packet
    /// timebase; UINT32 `offset_time` retains nanoseconds, including miss gaps.
    pub fn publish(&self, packet: &LivoxPacket) -> Result<()> {
        anyhow::ensure!(
            crate::RosTime::from_ns(packet.timebase_ns).map_err(anyhow::Error::msg)?
                == packet.stamp,
            "cloud header/timebase mismatch"
        );
        anyhow::ensure!(
            !packet.config.frame_id.is_empty() && !packet.config.frame_id.contains('\0'),
            "invalid cloud frame"
        );
        let data = crate::pointcloud::encode(packet).map_err(anyhow::Error::msg)?;
        let mut msg = self.metadata.create()?;
        if let Some(ValueMut::Simple(SimpleValueMut::Message(mut h))) = msg.get_mut("header") {
            scalar!(
                h,
                "frame_id",
                String,
                packet.config.frame_id.as_str().into()
            );
            if let Some(ValueMut::Simple(SimpleValueMut::Message(mut t))) = h.get_mut("stamp") {
                scalar!(t, "sec", Int32, packet.stamp.sec);
                scalar!(t, "nanosec", Uint32, packet.stamp.nanosec);
            } else {
                bail!("cloud timestamp schema mismatch");
            }
        } else {
            bail!("cloud header schema mismatch");
        }
        scalar!(msg, "height", Uint32, 1);
        scalar!(msg, "width", Uint32, u32::try_from(packet.points.len())?);
        scalar!(msg, "is_bigendian", Boolean, false);
        scalar!(msg, "is_dense", Boolean, true);
        scalar!(msg, "point_step", Uint32, crate::pointcloud::POINT_STEP);
        scalar!(msg, "row_step", Uint32, u32::try_from(data.len())?);
        if let Some(ValueMut::Sequence(SequenceValueMut::MessageSequence(mut fields))) =
            msg.get_mut("fields")
        {
            fields.reset(crate::pointcloud::FIELDS.len());
            for (f, (name, offset, datatype)) in fields
                .as_mut_slice()
                .iter_mut()
                .zip(crate::pointcloud::FIELDS)
            {
                scalar!(f, "name", String, name.into());
                scalar!(f, "offset", Uint32, offset);
                scalar!(f, "datatype", Uint8, datatype);
                scalar!(f, "count", Uint32, 1);
            }
        } else {
            bail!("cloud fields schema mismatch");
        }
        if let Some(ValueMut::Sequence(SequenceValueMut::Uint8Sequence(bytes))) =
            msg.get_mut("data")
        {
            *bytes = data.as_slice().into();
        } else {
            bail!("cloud data schema mismatch");
        }
        self.publisher.publish(msg)?;
        Ok(())
    }
}

/// Physical TF edges only. The caller owns the tree, authority and joint history.
/// No world-to-body pose or odometry is generated by this publisher.
pub struct TransformPublisher {
    fixed: DynamicPublisher,
    moving: DynamicPublisher,
    metadata: DynamicMessageMetadata,
}
impl TransformPublisher {
    /// Create reliable `/tf` (depth 100) and transient-local `/tf_static` (depth 1).
    /// Publish the complete set of this authority's static edges in one call.
    pub fn new(node: &Node) -> Result<Self> {
        let ty = MessageTypeName::try_from("tf2_msgs/msg/TFMessage")?;
        Ok(Self {
            metadata: DynamicMessageMetadata::new(ty.clone())?,
            fixed: node.create_dynamic_publisher(
                ty.clone(),
                "/tf_static".keep_last(1).reliable().transient_local(),
            )?,
            moving: node.create_dynamic_publisher(ty, "/tf".keep_last(100).reliable())?,
        })
    }
    /// Publish a bounded batch (at most 4096) of caller-stamped edges. Static
    /// batches must contain each child once. Dynamic history may repeat children
    /// at strictly increasing times in nanoseconds. Never freeze a moving joint.
    pub fn publish(&self, edges: &[crate::frames::FrameTransform], is_static: bool) -> Result<()> {
        anyhow::ensure!(edges.len() <= 4096, "TF batch exceeds allocation budget");
        let mut previous = std::collections::BTreeMap::new();
        for e in edges {
            e.validate().map_err(anyhow::Error::msg)?;
            if let Some((parent, time)) = previous.insert(&e.child, (&e.parent, e.time_ns)) {
                anyhow::ensure!(
                    !is_static && parent == &e.parent && time < e.time_ns,
                    "duplicate or nonmonotonic TF child"
                );
            }
        }
        let mut msg = self.metadata.create()?;
        if let Some(ValueMut::Sequence(SequenceValueMut::MessageSequence(mut transforms))) =
            msg.get_mut("transforms")
        {
            transforms.reset(edges.len());
            for (view, e) in transforms.as_mut_slice().iter_mut().zip(edges) {
                scalar!(view, "child_frame_id", String, e.child.as_str().into());
                let stamp = RosTime::from_ns(e.time_ns).map_err(anyhow::Error::msg)?;
                if let Some(ValueMut::Simple(SimpleValueMut::Message(mut h))) =
                    view.get_mut("header")
                {
                    scalar!(h, "frame_id", String, e.parent.as_str().into());
                    if let Some(ValueMut::Simple(SimpleValueMut::Message(mut t))) =
                        h.get_mut("stamp")
                    {
                        scalar!(t, "sec", Int32, stamp.sec);
                        scalar!(t, "nanosec", Uint32, stamp.nanosec);
                    } else {
                        bail!("TF stamp schema mismatch");
                    }
                } else {
                    bail!("TF header schema mismatch");
                }
                if let Some(ValueMut::Simple(SimpleValueMut::Message(mut tf))) =
                    view.get_mut("transform")
                {
                    if let Some(ValueMut::Simple(SimpleValueMut::Message(mut p))) =
                        tf.get_mut("translation")
                    {
                        scalar!(p, "x", Double, e.transform.translation_m[0]);
                        scalar!(p, "y", Double, e.transform.translation_m[1]);
                        scalar!(p, "z", Double, e.transform.translation_m[2]);
                    } else {
                        bail!("TF translation schema mismatch");
                    }
                    if let Some(ValueMut::Simple(SimpleValueMut::Message(mut q))) =
                        tf.get_mut("rotation")
                    {
                        let [x, y, z, w] = e.transform.rotation_xyzw;
                        scalar!(q, "x", Double, x);
                        scalar!(q, "y", Double, y);
                        scalar!(q, "z", Double, z);
                        scalar!(q, "w", Double, w);
                    } else {
                        bail!("TF rotation schema mismatch");
                    }
                } else {
                    bail!("TF transform schema mismatch");
                }
            }
        } else {
            bail!("TF sequence schema mismatch");
        }
        if is_static {
            self.fixed.publish(msg)?;
        } else {
            self.moving.publish(msg)?;
        }
        Ok(())
    }
}
