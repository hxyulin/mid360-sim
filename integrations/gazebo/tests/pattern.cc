// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 hxyulin <hxyulin@proton.me>
#include "mid360/pattern.hh"
#include <iostream>
int main() {
  for (const auto &fixture : mid360::data::Fixtures) {
    const auto d = mid360::Direction(fixture.index);
    for (unsigned int k = 0; k < 3; ++k)
      if (std::abs(d[k] - fixture.direction[k]) >= 2e-14) {
        std::cerr << "reference mismatch at " << fixture.index << "\n";
        return 1;
      }
    if (d != mid360::Direction(fixture.index + 4 * mid360::data::CycleGroups))
      return 2;
  }
}
