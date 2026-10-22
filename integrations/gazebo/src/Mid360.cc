// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
#include "mid360/pattern.hh"
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <gz/common/Console.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Physics.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <limits>
#include <optional>
#include <string>
#include <vector>
namespace mid360 {
/// Physics-step-mounted sensor; caller's Gazebo engine performs all
/// intersections.
class Mid360 final : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemPostUpdate,
                     public gz::sim::ISystemReset {
public:
  ~Mid360() override {
    if (this->profile.is_open()) {
      this->profile << "{\"steps\":" << this->profileSteps
                    << ",\"pre_ms\":" << this->preMs
                    << ",\"physics_scheduler_gap_ms\":" << this->gapMs
                    << ",\"post_ms\":" << this->postMs << "}\n";
    }
  }

public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    this->mount = gz::sim::Model(entity).LinkByName(
        ecm, sdf->Get<std::string>("link_name", "lidar").first);
    if (this->mount == gz::sim::kNullEntity) {
      gzerr << "MID360: missing mount link\n";
      return;
    }
    this->minimum = sdf->Get<double>("min_range_m", .1).first;
    this->maximum = sdf->Get<double>("max_range_m", 70.).first;
    this->stride = sdf->Get<unsigned int>("stride", 1).first;
    this->frame = sdf->Get<std::string>("frame_id", "livox_frame").first;
    if (!std::isfinite(this->minimum) || !std::isfinite(this->maximum) ||
        this->minimum < 0 || this->minimum >= this->maximum ||
        this->stride == 0 || this->stride > 1001) {
      gzerr << "MID360: invalid configuration\n";
      return;
    }
    this->publisher = this->node.Advertise<gz::msgs::PointCloudPacked>(
        sdf->Get<std::string>("topic", "/mid360/points").first);
    const auto report = sdf->Get<std::string>("report_file", "").first;
    if (!report.empty()) {
      this->journal.open(report);
      if (!this->journal) {
        gzerr << "MID360: cannot open report\n";
        return;
      }
    }
    if (ecm.Component<gz::sim::components::RaycastData>(this->mount)) {
      gzerr << "MID360: mount already has a ray owner\n";
      return;
    }
    ecm.CreateComponent(this->mount, gz::sim::components::RaycastData());
    const auto profilePath = sdf->Get<std::string>("profile_file", "").first;
    if (!profilePath.empty()) {
      this->profile.open(profilePath);
      if (!this->profile) {
        gzerr << "MID360: cannot open profile report\n";
        return;
      }
    }
    this->bytes.reserve(20000 * 20);
    this->enabled = true;
  }

public:
  void Reset(const gz::sim::UpdateInfo &,
             gz::sim::EntityComponentManager &ecm) override {
    this->next = 0;
    this->first = 0;
    this->pending = 0;
    this->scheduled = 0;
    this->bytes.clear();
    this->lastNs = -1;
    if (auto c = ecm.Component<gz::sim::components::RaycastData>(this->mount)) {
      c->Data().rays.clear();
      c->Data().results.clear();
    }
  }

public:
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (!this->enabled || info.paused)
      return;
    const auto begin =
        this->profile.is_open() ? Clock::now() : Clock::time_point{};
    const auto world = ecm.EntityByComponents(gz::sim::components::World());
    const auto detector =
        ecm.Component<gz::sim::components::PhysicsCollisionDetector>(world);
    if (!detector || detector->Data() != "bullet") {
      this->enabled = false;
      gzerr << "MID360: requires DART with explicit Bullet collision detector; "
               "refusing empty unsupported scans\n";
      return;
    }
    const auto ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
            .count();
    if (ns < 0)
      return;
    const auto until = (static_cast<std::uint64_t>(ns) + 4999) / 5000;
    if (ns < this->lastNs)
      this->Reset(info, ecm);
    this->lastNs = ns;
    const auto available =
        until > this->next
            ? (until - this->next + this->stride - 1) / this->stride
            : 0;
    if (available > 200000) {
      this->enabled = false;
      gzerr << "MID360: step exceeds bounded acquisition budget\n";
      return;
    }
    auto component =
        ecm.Component<gz::sim::components::RaycastData>(this->mount);
    if (!component) {
      this->enabled = false;
      return;
    }
    auto &data = component->Data();
    data.rays.clear();
    data.results.clear();
    this->pending = static_cast<std::size_t>(available);
    this->pendingFirst = this->next;
    data.rays.reserve(this->pending);
    std::optional<DirectionGroup> optics;
    std::uint64_t previousGroup = std::numeric_limits<std::uint64_t>::max();
    for (std::size_t i = 0; i < this->pending; ++i) {
      const auto index = this->next + i * this->stride;
      if (index / 4 != previousGroup) {
        optics.emplace(index);
        previousGroup = index / 4;
      }
      const auto d = optics->Lane(index % 4);
      data.rays.push_back(
          {gz::math::Vector3d::Zero,
           gz::math::Vector3d(d[0], d[1], d[2]) * this->maximum});
    }
    this->next += available * this->stride;
    if (this->profile.is_open()) {
      this->preEnd = Clock::now();
      this->preMs += Milliseconds(this->preEnd - begin).count();
    }
  }

private:
  static void U32(std::string &out, std::uint32_t value) {
    for (unsigned int i = 0; i < 4; ++i)
      out.push_back(static_cast<char>((value >> (8 * i)) & 255));
  }

private:
  static void Float(std::string &out, float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, 4);
    U32(out, bits);
  }

private:
  void Publish() {
    gz::msgs::PointCloudPacked message;
    const auto time = this->first * 5000;
    message.mutable_header()->mutable_stamp()->set_sec(
        static_cast<std::int64_t>(time / 1000000000));
    message.mutable_header()->mutable_stamp()->set_nsec(
        static_cast<std::int32_t>(time % 1000000000));
    auto item = message.mutable_header()->add_data();
    item->set_key("frame_id");
    item->add_value(this->frame);
    item = message.mutable_header()->add_data();
    item->set_key("first_sample_index");
    item->add_value(std::to_string(this->first));
    item = message.mutable_header()->add_data();
    item->set_key("stride");
    item->add_value(std::to_string(this->stride));
    const char *names[] = {"x",   "y",   "z", "offset_time", "reflectivity",
                           "tag", "line"};
    const unsigned int offsets[] = {0, 4, 8, 12, 16, 17, 18};
    for (unsigned int i = 0; i < 7; ++i) {
      auto f = message.add_field();
      f->set_name(names[i]);
      f->set_offset(offsets[i]);
      f->set_count(1);
      f->set_datatype(i < 3 ? gz::msgs::PointCloudPacked::Field::FLOAT32
                            : (i == 3
                                   ? gz::msgs::PointCloudPacked::Field::UINT32
                                   : gz::msgs::PointCloudPacked::Field::UINT8));
    }
    message.set_height(1);
    message.set_width(static_cast<std::uint32_t>(this->bytes.size() / 20));
    message.set_point_step(20);
    message.set_row_step(static_cast<std::uint32_t>(this->bytes.size()));
    message.set_is_dense(true);
    message.set_is_bigendian(false);
    message.set_data(this->bytes);
    this->publisher.Publish(message);
    if (this->journal.is_open()) {
      this->journal << "{\"first_index\":" << this->first
                    << ",\"scheduled\":" << this->scheduled
                    << ",\"points\":" << message.width()
                    << ",\"stride\":" << this->stride << ",\"min_hit_m\":"
                    << (this->bytes.empty() ? 0 : this->minHit)
                    << ",\"max_hit_m\":" << this->maxHit << "}\n";
      this->journal.flush();
    }
    this->bytes.clear();
    this->scheduled = 0;
  }

public:
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (!this->enabled || info.paused || this->pending == 0)
      return;
    const auto begin =
        this->profile.is_open() ? Clock::now() : Clock::time_point{};
    if (this->profile.is_open())
      this->gapMs += Milliseconds(begin - this->preEnd).count();
    const auto &data =
        ecm.Component<gz::sim::components::RaycastData>(this->mount)->Data();
    if (data.results.size() != this->pending) {
      this->enabled = false;
      gzerr << "MID360: physics engine did not return the requested rays; "
               "select a supported engine\n";
      return;
    }
    for (std::size_t i = 0; i < this->pending; ++i) {
      const auto index = this->pendingFirst + i * this->stride;
      if (this->scheduled == 0) {
        this->first = index;
        this->minHit = std::numeric_limits<double>::infinity();
        this->maxHit = 0;
      }
      const auto &hit = data.results[i];
      const double distance = hit.fraction * this->maximum;
      if (std::isfinite(distance) && distance >= this->minimum &&
          distance <= this->maximum) {
        // Use generated local ray and measured distance; exact sample identity
        // and source frame are retained even when previous rays missed.
        const auto point = data.rays[i].end * hit.fraction;
        Float(this->bytes, static_cast<float>(point.X()));
        Float(this->bytes, static_cast<float>(point.Y()));
        Float(this->bytes, static_cast<float>(point.Z()));
        U32(this->bytes,
            static_cast<std::uint32_t>((index - this->first) * 5000));
        this->bytes.push_back(100);
        this->bytes.push_back(0x10);
        this->bytes.push_back(static_cast<char>(index % 4));
        this->bytes.push_back(0);
        this->minHit = std::min(this->minHit, distance);
        this->maxHit = std::max(this->maxHit, distance);
      }
      ++this->scheduled;
      // Publish at most 100 ms of nominal emissions. Decimation changes output
      // count, never acquisition times or scan evolution.
      if ((index - this->first + this->stride) >= 20000)
        this->Publish();
    }
    this->pending = 0;
    if (this->profile.is_open()) {
      this->postMs += Milliseconds(Clock::now() - begin).count();
      ++this->profileSteps;
    }
  }

private:
  using Clock = std::chrono::steady_clock;
  using Milliseconds = std::chrono::duration<double, std::milli>;
  std::ofstream profile;
  Clock::time_point preEnd;
  double preMs = 0, gapMs = 0, postMs = 0;
  std::uint64_t profileSteps = 0;

private:
  gz::sim::Entity mount = gz::sim::kNullEntity;

private:
  gz::transport::Node node;

private:
  gz::transport::Node::Publisher publisher;

private:
  bool enabled = false;

private:
  double minimum = .1, maximum = 70, minHit = 0, maxHit = 0;

private:
  unsigned int stride = 1;

private:
  std::string frame = "livox_frame", bytes;

private:
  std::ofstream journal;

private:
  std::uint64_t next = 0, first = 0, pendingFirst = 0, scheduled = 0;

private:
  std::size_t pending = 0;

private:
  std::int64_t lastNs = -1;
};
} // namespace mid360
GZ_ADD_PLUGIN(mid360::Mid360, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate,
              gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(mid360::Mid360, "mid360::Mid360")
