// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
#include "mid360/pattern.hh"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <gz/common/Console.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
namespace {
std::uint32_t Read32(const std::string &s, std::size_t at) {
  std::uint32_t v = 0;
  for (unsigned int i = 0; i < 4; ++i)
    v |= static_cast<std::uint32_t>(static_cast<unsigned char>(s[at + i]))
         << (8 * i);
  return v;
}
float ReadFloat(const std::string &s, std::size_t at) {
  const auto u = Read32(s, at);
  float f;
  std::memcpy(&f, &u, 4);
  return f;
}
} // namespace
int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  gz::common::Console::SetVerbosity(4);
  gz::sim::ServerConfig config;
  config.SetSdfFile(argv[1]);
  gz::sim::Server server(config);
  gz::transport::Node node;
  std::mutex mutex;
  std::size_t frames = 0, points = 0;
  std::string error;
  if (!node.Subscribe<gz::msgs::PointCloudPacked>(
          "/mid360/points", [&](const gz::msgs::PointCloudPacked &msg) {
            std::lock_guard<std::mutex> lock(mutex);
            if (msg.point_step() != 20 ||
                msg.data().size() != msg.width() * 20 || msg.height() != 1 ||
                msg.is_bigendian()) {
              error = "bad layout";
              return;
            }
            std::uint64_t first = 0;
            bool found = false;
            for (const auto &data : msg.header().data())
              if (data.key() == "first_sample_index" &&
                  data.value_size() == 1) {
                first = std::stoull(data.value(0));
                found = true;
              }
            if (!found) {
              error = "missing emission metadata";
              return;
            }
            std::uint64_t stride = 1;
            for (const auto &data : msg.header().data())
              if (data.key() == "stride" && data.value_size() == 1)
                stride = std::stoull(data.value(0));
            if (msg.width() != (20000 + stride - 1) / stride) {
              error = "missing enclosure returns";
              return;
            }
            const auto stamp =
                static_cast<std::uint64_t>(msg.header().stamp().sec()) *
                    1000000000 +
                msg.header().stamp().nsec();
            if (stamp != first * 5000) {
              error = "wrong timebase";
              return;
            }
            for (std::size_t i = 0; i < msg.width(); ++i) {
              const auto at = i * 20;
              const auto offset = Read32(msg.data(), at + 12);
              if (offset % 5000) {
                error = "invalid offset";
                return;
              }
              const auto index = first + offset / 5000;
              const auto d = mid360::Direction(index);
              if (static_cast<unsigned char>(msg.data()[at + 18]) !=
                  index % 4) {
                error = "wrong lane";
                return;
              }
              double expected = std::numeric_limits<double>::infinity();
              for (unsigned int k = 0; k < 3; ++k) {
                const double origin = k == 2 ? .8 : 0;
                expected = std::min(expected,
                                    ((d[k] > 0 ? 9.9 : -9.9) - origin) / d[k]);
              }
              for (unsigned int k = 0; k < 3; ++k)
                if (std::abs(ReadFloat(msg.data(), at + 4 * k) -
                             d[k] * expected) > 2e-4) {
                  error = "geometry distance mismatch";
                  return;
                }
            }
            ++frames;
            points += msg.width();
          }))
    return 3;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto start = std::chrono::steady_clock::now();
  if (!server.Run(true, 2000, false))
    return 4;
  const double elapsed = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - start)
                             .count();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  std::lock_guard<std::mutex> lock(mutex);
  if (!error.empty() || frames < 5) {
    std::cerr << "native test failed: " << error << " frames=" << frames
              << "\n";
    return 1;
  }
  std::cout << "MID360_GAZEBO_RESULT {\"frames\":" << frames
            << ",\"validated_points\":" << points
            << ",\"simulation_steps\":2000,\"simulated_s\":2,\"run_ms\":"
            << elapsed << "}\n";
}
