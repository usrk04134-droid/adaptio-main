#pragma once

#include "common/groove/groove.h"

namespace macs {

using Groove = common::Groove;  // Backward-compatible alias

inline constexpr int ABW_POINTS = common::ABW_POINTS;
inline constexpr int ABW_UPPER_LEFT = common::ABW_UPPER_LEFT;
inline constexpr int ABW_LOWER_LEFT = common::ABW_LOWER_LEFT;
inline constexpr int ABW_LOWER_RIGHT = common::ABW_LOWER_RIGHT;
inline constexpr int ABW_UPPER_RIGHT = common::ABW_UPPER_RIGHT;

}  // namespace macs
