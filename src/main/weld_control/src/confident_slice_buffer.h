#pragma once

#include "common/containers/slot_buffer.h"
#include "macs/macs_groove.h"

namespace weld_control {

struct ConfidentSliceData {
  double edge_position;
  macs::Groove groove;
};

/* 500mm  radius ->  3141 / 5.0 =  628 slots */
/* 5000mm radius -> 31415 / 5.0 = 6282 slots */
auto const CONFIDENT_SLICE_RESOLUTION = 5.0; /* in mm */

auto inline CalculateConfidentSliceBufferSlots(double radius) -> size_t {
  /* calculate the circumference and divide with desired resolution */
  return static_cast<size_t>(2.0 * std::numbers::pi * radius / CONFIDENT_SLICE_RESOLUTION);
};

using ConfidentSliceBuffer = common::containers::SlotBuffer<ConfidentSliceData>;

}  // namespace weld_control
