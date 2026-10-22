// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
// Test-only prescribed MoCap poses; no changes to the sensor implementation.
#include <fstream>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <stdexcept>
#include <vector>
namespace mid360 {
class Replay final : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemConfigurePriority,
                     public gz::sim::ISystemPreUpdate {
public:
  gz::sim::System::PriorityType ConfigurePriority() override { return -1000; }
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override {
    model = entity;
    std::ifstream input(sdf->Get<std::string>("poses_file"));
    std::int64_t ns;
    double x, y, z, qx, qy, qz, qw;
    while (input >> ns >> x >> y >> z >> qx >> qy >> qz >> qw) {
      if (poses.size() == 1)
        stepNs = ns;
      if (stepNs <= 0 || ns != static_cast<std::int64_t>(poses.size()) * stepNs)
        throw std::runtime_error("invalid replay time grid");
      poses.emplace_back(gz::math::Vector3d(x, y, z),
                         gz::math::Quaterniond(qw, qx, qy, qz));
    }
    if (poses.size() < 2 || poses.size() > 40001 ||
        (poses.size() - 1) * stepNs != 2000000000)
      throw std::runtime_error("need bounded uniform two-second replay");
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    const auto ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
            .count();
    if (ns < 0 || ns % stepNs ||
        static_cast<std::size_t>(ns / stepNs) >= poses.size())
      throw std::runtime_error(
          "replay outside fixture grid: ns=" + std::to_string(ns) +
          " step=" + std::to_string(stepNs));
    gz::sim::Model(model).SetWorldPoseCmd(ecm, poses.at(ns / stepNs));
  }

private:
  std::int64_t stepNs = 1000000;
  gz::sim::Entity model = gz::sim::kNullEntity;
  std::vector<gz::math::Pose3d> poses;
};
} // namespace mid360
GZ_ADD_PLUGIN(mid360::Replay, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemConfigurePriority, gz::sim::ISystemPreUpdate)
