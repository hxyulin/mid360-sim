// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
// Executes the pinned upstream plugin unchanged; external source is
// caller-owned.
#include <chrono>
#include <fstream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <iostream>
#include <livox_laser_simulation/livox_points_plugin.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <stdexcept>
#include <vector>
class ManualLivox : public gazebo::LivoxPointsPlugin {
public:
  using gazebo::LivoxPointsPlugin::OnNewLaserScans;
};
int main(int argc, char **argv) {
  if (argc != 5)
    return 2;
  ros::init(argc, argv, "mid360_reference");
  gazebo::setupServer();
  auto world = gazebo::loadWorld(argv[1]);
  if (!world)
    throw std::runtime_error("world load failed");
  auto model = world->ModelByName("sensor");
  gazebo::sensors::SensorManager::Instance()->Init();
  gazebo::sensors::SensorManager::Instance()->Update(true);
  auto sensors = gazebo::sensors::SensorManager::Instance()->GetSensors();
  if (sensors.size() != 1)
    throw std::runtime_error("need one native ray sensor");
  auto sensor = sensors.front();
  sensor->SetActive(false);
  sdf::SDFPtr config(new sdf::SDF());
  sdf::init(config);
  if (!sdf::readFile(argv[2], config))
    throw std::runtime_error("plugin SDF failed");
  std::vector<ignition::math::Pose3d> poses;
  std::ifstream input(argv[3]);
  long long ns;
  double x, y, z, qx, qy, qz, qw;
  while (input >> ns >> x >> y >> z >> qx >> qy >> qz >> qw)
    poses.emplace_back(ignition::math::Vector3d(x, y, z),
                       ignition::math::Quaterniond(qw, qx, qy, qz));
  if (poses.size() != 2001)
    throw std::runtime_error("need 2001 poses");
  std::ofstream output(argv[4], std::ios::binary);
  std::size_t frames = 0, points = 0;
  ros::NodeHandle node;
  auto sub = node.subscribe<sensor_msgs::PointCloud>(
      "/mid360_points", 100, [&](const sensor_msgs::PointCloud::ConstPtr &msg) {
        for (const auto &p : msg->points) {
          output.write(reinterpret_cast<const char *>(&p.x), 4);
          output.write(reinterpret_cast<const char *>(&p.y), 4);
          output.write(reinterpret_cast<const char *>(&p.z), 4);
        }
        ++frames;
        points += msg->points.size();
      });
  {
    ManualLivox plugin;
    plugin.Load(sensor, config->Root()
                            ->GetElement("model")
                            ->GetElement("link")
                            ->GetElement("sensor")
                            ->GetElement("plugin"));
    model->SetWorldPose(poses.front());
    world->RunBlocking(100);
    for (int i = 0; i < 100 && sub.getNumPublishers() == 0; ++i) {
      ros::WallDuration(.01).sleep();
      ros::spinOnce();
    }
    if (sub.getNumPublishers() != 1)
      throw std::runtime_error("ROS subscriber not connected");
    std::size_t step = 0;
    double querySeconds = 0.;
    auto before = gazebo::event::Events::ConnectWorldUpdateBegin(
        [&](const gazebo::common::UpdateInfo &) {
          ++step;
          if (step <= 2000)
            model->SetWorldPose(poses[step]);
        });
    auto after = gazebo::event::Events::ConnectWorldUpdateEnd([&]() {
      if (step % 100 || step > 2000)
        return;
      ros::Time::setNow(ros::Time(world->SimTime().Double()));
      const auto start = std::chrono::steady_clock::now();
      plugin.OnNewLaserScans();
      ros::spinOnce();
      querySeconds += std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    });
    const auto start = std::chrono::steady_clock::now();
    world->RunBlocking(2000);
    const auto wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    for (int i = 0; i < 100 && frames < 20; ++i) {
      ros::spinOnce();
      ros::WallDuration(.005).sleep();
    }
    if (frames != 20 || points != 400000)
      throw std::runtime_error("incomplete official output");
    std::cout << "OFFICIAL_RESULT {\"wall_s\":" << wall
              << ",\"query_publish_s\":" << querySeconds
              << ",\"frames\":" << frames << ",\"points\":" << points << "}"
              << std::endl;
  }
  output.close();
  gazebo::shutdown();
  ros::shutdown();
}
