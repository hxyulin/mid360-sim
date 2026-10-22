// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
//! Deterministic packet faults for caller-owned adapters and explicit timelines.
//!
//! This module never reads a clock or inspects/changes sensor payloads. Schedule
//! complete packets after acquisition. Jitter may reorder delivery; it must not
//! rewrite emission times, IMU times or packet sequence numbers.
use std::collections::{BTreeMap, BTreeSet};

/// Stable packet identity, independent of submission order or point decimation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct PacketId {
    /// Caller-assigned stream identity; use a distinct value for each sensor/topic.
    pub stream: u64,
    /// Original packet sequence, retained even when previous packets were lost.
    pub sequence: u64,
}

/// Synthetic transport stress settings, not measured MID-360/DDS error models.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaultConfig {
    /// Reproducible experiment seed; packet identities select independent outcomes.
    pub seed: u64,
    /// Independent whole-packet loss probability in `[0,1]`.
    pub loss_probability: f64,
    /// Fixed nonnegative delivery delay, nanoseconds on the caller's timeline.
    pub delay_ns: u64,
    /// Additional uniform integer delay in `[0,jitter_ns]`, nanoseconds.
    /// Unlike signed jitter, this never releases a packet before availability.
    pub jitter_ns: u64,
}
impl Default for FaultConfig {
    fn default() -> Self {
        Self {
            seed: 7,
            loss_probability: 0.,
            delay_ns: 0,
            jitter_ns: 0,
        }
    }
}
impl FaultConfig {
    /// Reject invalid probabilities or a delay-plus-jitter overflow in nanoseconds.
    pub fn validate(&self) -> Result<(), &'static str> {
        if !self.loss_probability.is_finite() || !(0. ..=1.).contains(&self.loss_probability) {
            return Err("packet loss probability must be finite and in [0,1]");
        }
        self.delay_ns
            .checked_add(self.jitter_ns)
            .ok_or("packet delay plus jitter overflows nanoseconds")?;
        Ok(())
    }
    /// Return an immutable release decision for a packet available at `available_ns`.
    /// `None` means injected loss. `Some` is a release deadline in the same caller
    /// timeline, never a sensor stamp. Identity, seed and availability determine
    /// the result independently of call order, other streams or retained points.
    pub fn release_ns(&self, id: PacketId, available_ns: u64) -> Result<Option<u64>, &'static str> {
        self.validate()?;
        // Separate packet fault domains from measurement-effect streams. Mixing
        // stream identity first also avoids a mutable RNG shared between sensors.
        let seed = mix(self.seed ^ mix(id.stream));
        if self.loss_probability == 1.
            || (self.loss_probability > 0.
                && crate::effects::uniform(seed, id.sequence, 100) < self.loss_probability)
        {
            return Ok(None);
        }
        let bits = mix(seed ^ mix(id.sequence) ^ 0x7061636b65745f6a);
        let jitter = ((u128::from(bits) * (u128::from(self.jitter_ns) + 1)) >> 64) as u64;
        available_ns
            .checked_add(self.delay_ns)
            .and_then(|n| n.checked_add(jitter))
            .map(Some)
            .ok_or("packet release deadline overflows nanoseconds")
    }
}
fn mix(mut x: u64) -> u64 {
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
    x ^ (x >> 31)
}

/// Result of admitting one complete packet; sensor payloads are never modified.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Admission {
    /// Packet retained until its release deadline, subject to caller draining.
    Queued,
    /// Deterministic configured loss; payload discarded without occupying a slot.
    InjectedLoss,
    /// Pending queue already full; newest submitted payload discarded.
    CapacityDrop,
}
/// Cumulative packet counts, saturating at `u64::MAX`; never point/byte counts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Statistics {
    /// Packets successfully admitted, including those subsequently delivered.
    pub queued: u64,
    /// Packets discarded by the configured deterministic loss decision.
    pub injected_losses: u64,
    /// Packets discarded because the bounded pending queue was full.
    pub capacity_drops: u64,
    /// Packets removed by successful due-packet polls.
    pub delivered: u64,
}
/// Original payload plus delivery metadata, with no synthesized sensor timestamps.
#[derive(Clone, Debug, PartialEq)]
pub struct Delivery<T> {
    /// Original stream and sequence identity.
    pub id: PacketId,
    /// Caller-provided time the completed packet became available, nanoseconds.
    pub available_ns: u64,
    /// Earliest allowed release time, nanoseconds on the same caller timeline.
    pub release_ns: u64,
    /// Unmodified caller payload; may be a LiDAR packet or a paired LiDAR/IMU bundle.
    pub payload: T,
}
/// Bounded adapter queue; the caller supplies all times and drains due packets.
///
/// Equal deadlines use stream/sequence order, independent of insertion order.
/// Fault decisions are order-invariant; capacity drops depend on submission and
/// drain order. Use identical admission schedules for repeatable overload tests.
/// The bound counts packets, not bytes: callers must also bound each payload.
///
/// ```
/// use mid360_sim::transport::{Admission, FaultConfig, FaultInjector, PacketId};
/// let mut adapter = FaultInjector::new(FaultConfig {
///     delay_ns: 10_000_000, ..Default::default()
/// }, 4)?;
/// let id = PacketId { stream: 0, sequence: 42 };
/// let sensor_stamp_ns = 123;
/// assert_eq!(adapter.submit(id, 200, sensor_stamp_ns)?, Admission::Queued);
/// assert!(adapter.pop_due(10_000_199).is_none());
/// assert_eq!(adapter.pop_due(10_000_200).unwrap().payload, sensor_stamp_ns);
/// # Ok::<(), &'static str>(())
/// ```
pub struct FaultInjector<T> {
    config: FaultConfig,
    capacity: usize,
    pending: BTreeMap<(u64, PacketId), Delivery<T>>,
    identities: BTreeSet<PacketId>,
    statistics: Statistics,
}
impl<T> FaultInjector<T> {
    /// Validate fault settings and a nonzero maximum number of pending packets.
    pub fn new(config: FaultConfig, capacity: usize) -> Result<Self, &'static str> {
        config.validate()?;
        if capacity == 0 {
            return Err("packet queue capacity must be nonzero");
        }
        Ok(Self {
            config,
            capacity,
            pending: BTreeMap::new(),
            identities: BTreeSet::new(),
            statistics: Statistics::default(),
        })
    }
    /// Submit one complete payload at caller availability time in nanoseconds.
    /// Injected loss takes precedence over capacity loss. A duplicate pending ID
    /// or overflowing deadline returns an error without changing queue/counters;
    /// the supplied payload is consumed. IDs must not be reused within a stream.
    pub fn submit(
        &mut self,
        id: PacketId,
        available_ns: u64,
        payload: T,
    ) -> Result<Admission, &'static str> {
        if self.identities.contains(&id) {
            return Err("duplicate pending packet identity");
        }
        let Some(release_ns) = self.config.release_ns(id, available_ns)? else {
            self.statistics.injected_losses = self.statistics.injected_losses.saturating_add(1);
            return Ok(Admission::InjectedLoss);
        };
        if self.pending.len() >= self.capacity {
            self.statistics.capacity_drops = self.statistics.capacity_drops.saturating_add(1);
            return Ok(Admission::CapacityDrop);
        }
        self.pending.insert(
            (release_ns, id),
            Delivery {
                id,
                available_ns,
                release_ns,
                payload,
            },
        );
        self.identities.insert(id);
        self.statistics.queued = self.statistics.queued.saturating_add(1);
        Ok(Admission::Queued)
    }
    /// Pop the earliest packet whose deadline is at most `now_ns` on the caller
    /// timeline. Later polling can add latency; no sleeping or clock reading occurs.
    pub fn pop_due(&mut self, now_ns: u64) -> Option<Delivery<T>> {
        if self.next_release_ns()? > now_ns {
            return None;
        }
        let (_, packet) = self.pending.pop_first()?;
        self.identities.remove(&packet.id);
        self.statistics.delivered = self.statistics.delivered.saturating_add(1);
        Some(packet)
    }
    /// Earliest pending release deadline, nanoseconds on the caller timeline.
    /// An adapter may schedule its own timer for this value; no timer is created.
    pub fn next_release_ns(&self) -> Option<u64> {
        self.pending.first_key_value().map(|((time, _), _)| *time)
    }
    /// Number of pending packets, bounded by the capacity passed to `new`.
    pub fn pending(&self) -> usize {
        self.pending.len()
    }
    /// Copy cumulative packet counts, separating injected loss from overload.
    pub fn statistics(&self) -> Statistics {
        self.statistics
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn id(sequence: u64) -> PacketId {
        PacketId {
            stream: 3,
            sequence,
        }
    }
    #[test]
    fn seeded_release_regression_keeps_original_packet_identity() {
        let config = FaultConfig {
            seed: 7,
            loss_probability: 0.1,
            delay_ns: 20_000_000,
            jitter_ns: 300_000_000,
        };
        let expected = [
            Some(1_178_037_137),
            Some(1_272_480_303),
            Some(1_291_984_925),
            None,
            Some(1_627_918_055),
            Some(1_655_264_264),
            Some(1_639_864_032),
            Some(1_758_397_734),
        ];
        for (sequence, expected) in expected.into_iter().enumerate() {
            assert_eq!(
                config.release_ns(
                    PacketId {
                        stream: 0,
                        sequence: sequence as u64
                    },
                    1_000_000_000 + sequence as u64 * 100_000_000
                ),
                Ok(expected)
            );
        }
    }
    #[test]
    fn fault_decisions_and_delivery_order_ignore_submission_chunks_and_other_streams() {
        let config = FaultConfig {
            seed: 43,
            loss_probability: 0.2,
            delay_ns: 50,
            jitter_ns: 1000,
        };
        let acquire = |order: Vec<u64>| {
            let mut q = FaultInjector::new(config, 200).unwrap();
            for sequence in order {
                // Payload deliberately retains its original sensor timestamp.
                q.submit(id(sequence), sequence * 10, (sequence, 123 + sequence))
                    .unwrap();
                config
                    .release_ns(
                        PacketId {
                            stream: 99,
                            sequence,
                        },
                        0,
                    )
                    .unwrap();
            }
            let mut out = Vec::new();
            while let Some(packet) = q.pop_due(u64::MAX) {
                out.push(packet);
            }
            (out, q.statistics())
        };
        let (all, stats) = acquire((0..100).collect());
        assert_eq!((all.clone(), stats), acquire((0..100).rev().collect()));
        assert_eq!(
            (all.clone(), stats),
            acquire((0..37).chain(37..100).collect())
        );
        assert!(stats.injected_losses > 0 && stats.injected_losses < 100);
        assert_eq!(stats.injected_losses + stats.delivered, 100);
        assert!(all.windows(2).any(|p| p[0].id.sequence > p[1].id.sequence));
        let mut interleaved = FaultInjector::new(config, 200).unwrap();
        for sequence in 0..100 {
            for stream in [99, 3] {
                interleaved
                    .submit(
                        PacketId { stream, sequence },
                        sequence * 10,
                        (sequence, 123 + sequence),
                    )
                    .unwrap();
            }
        }
        let same_stream: Vec<_> = std::iter::from_fn(|| interleaved.pop_due(u64::MAX))
            .filter(|packet| packet.id.stream == 3)
            .collect();
        assert_eq!(all, same_stream);
        let mut retained = FaultInjector::new(config, 100).unwrap();
        for sequence in (0..100).step_by(3) {
            retained
                .submit(id(sequence), sequence * 10, (sequence, 123 + sequence))
                .unwrap();
        }
        let expected: Vec<_> = all.into_iter().filter(|p| p.id.sequence % 3 == 0).collect();
        let actual: Vec<_> = std::iter::from_fn(|| retained.pop_due(u64::MAX)).collect();
        assert_eq!(expected, actual);
    }
    #[test]
    fn bounded_admission_no_early_delivery_and_unchanged_payloads() {
        let mut q = FaultInjector::new(
            FaultConfig {
                delay_ns: 10,
                ..Default::default()
            },
            2,
        )
        .unwrap();
        assert_eq!(
            q.submit(id(2), 100, vec![2, 20]).unwrap(),
            Admission::Queued
        );
        assert_eq!(
            q.submit(id(1), 100, vec![1, 10]).unwrap(),
            Admission::Queued
        );
        assert_eq!(
            q.submit(id(3), 100, vec![3]).unwrap(),
            Admission::CapacityDrop
        );
        assert_eq!(q.pending(), 2);
        assert_eq!(q.next_release_ns(), Some(110));
        assert!(q.pop_due(109).is_none());
        assert_eq!(q.pop_due(110).unwrap().payload, vec![1, 10]);
        assert_eq!(q.pop_due(110).unwrap().payload, vec![2, 20]);
        assert!(q.pop_due(u64::MAX).is_none());
        assert_eq!(q.next_release_ns(), None);
        assert_eq!(
            q.statistics(),
            Statistics {
                queued: 2,
                delivered: 2,
                capacity_drops: 1,
                injected_losses: 0
            }
        );
        let mut loss = FaultInjector::new(
            FaultConfig {
                loss_probability: 1.,
                ..Default::default()
            },
            1,
        )
        .unwrap();
        assert_eq!(loss.submit(id(0), 0, ()).unwrap(), Admission::InjectedLoss);
        assert_eq!(loss.pending(), 0);
        assert_eq!(loss.statistics().injected_losses, 1);
    }
    #[test]
    fn invalid_settings_and_deadlines_leave_pending_state_intact() {
        for probability in [-0.1, 1.1, f64::NAN, f64::INFINITY] {
            assert!(
                FaultConfig {
                    loss_probability: probability,
                    ..Default::default()
                }
                .validate()
                .is_err()
            );
        }
        assert!(FaultInjector::<()>::new(FaultConfig::default(), 0).is_err());
        assert!(
            FaultConfig {
                delay_ns: u64::MAX,
                jitter_ns: 1,
                ..Default::default()
            }
            .validate()
            .is_err()
        );
        let mut q = FaultInjector::new(
            FaultConfig {
                delay_ns: 1,
                ..Default::default()
            },
            1,
        )
        .unwrap();
        q.submit(id(0), 1, 17).unwrap();
        let before = q.statistics();
        assert!(q.submit(id(0), 2, 99).is_err());
        assert!(q.submit(id(1), u64::MAX, 99).is_err());
        assert_eq!(q.statistics(), before);
        assert_eq!(q.pop_due(2).unwrap().payload, 17);
        // All-off decisions retain availability exactly, even at the time bound.
        assert_eq!(
            FaultConfig::default().release_ns(id(0), u64::MAX),
            Ok(Some(u64::MAX))
        );
    }
}
