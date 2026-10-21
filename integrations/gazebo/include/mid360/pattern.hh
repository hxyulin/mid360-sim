// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
#pragma once
#include "mid360/model.hh"
#include <array>
#include <cmath>
#include <cstddef>
namespace mid360 {
/// Nominal sensor FLU unit vector at an explicit emission index (5 us cadence).
inline std::array<double, 3> Direction(std::uint64_t index) {
  constexpr double Pi = 3.141592653589793238462643383279502884;
  constexpr auto N = sizeof(data::Terms) / sizeof(data::Terms[0]);
  const auto group = (index / 4) % data::CycleGroups;
  const double spin =
      2 * Pi * static_cast<double>((group * data::SpinHz) % 50000) / 50000;
  const double sweep =
      2 * Pi *
      static_cast<double>((group * data::SweepTurns) % data::CycleGroups) /
      static_cast<double>(data::CycleGroups);
  const auto &c = data::Coefficients[index % 4];
  double az = c[0][0], el = c[0][1];
  for (std::size_t i = 0; i < N; ++i) {
    const double phase = data::Terms[i][0] * spin + data::Terms[i][1] * sweep;
    const double co = std::cos(phase), si = std::sin(phase);
    az += c[1 + i][0] * co + c[1 + N + i][0] * si;
    el += c[1 + i][1] * co + c[1 + N + i][1] * si;
  }
  az = spin + az * Pi / 720;
  el *= Pi / 720;
  return {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
          std::sin(el)};
}
} // namespace mid360
