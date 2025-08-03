#pragma once

#include <boost/range/combine.hpp>

#include "bead_control/src/weld_position_data_storage.h"
#include "macs/macs_groove.h"

namespace bead_control {

class BeadCalc {
 public:
  explicit BeadCalc() = default;

  auto static MeanLayerArea(const macs::Groove& groove, double left_bead_area, double right_bead_area,
                            double step_up_value) -> double;

  // wire_lin_velocity: mm/s
  // wire_diameter: mm
  // weld_object_lin_velocity: mm/s
  auto static BeadArea(double wire_lin_velocity, double wire_diameter, double weld_object_lin_velocity) -> double;

  auto static MeanLowerGrooveWidth(WeldPositionDataStorage::Slice data) -> double;

  auto static Distance2Angle(double radius, double arc_length) -> double {
    // Calculates angle in radians with radius (mm) and arc length (mm)
    // Avoid division with zero, should not happen
    if (radius <= 0.0) {
      radius = 1.0;
    }

    return arc_length / radius;
  };

  auto static MeanBeadArea(WeldPositionDataStorage::Slice data, double wire_diameter_ws1, bool twin_wire_ws1,
                           double wire_diameter_ws2, bool twin_wire_ws2) -> double;

  // Calculate the relative size of a bead's slice area compared to the groove's average bead slice area.
  // A return value of 1.12 -> 12% larger than the average, 0.9 -> 10% smaller, and so on.
  auto static BeadSliceAreaRatio(const macs::Groove& groove, int bead, int beads) -> double;

  // Calculate a new bead position for a given groove by calculating the top and bottom slopes of the groove
  // and move the bead position towards the side with a larger area. bead_pos and return values are relative
  // bead position between 0-1 where 0 and 1 are the left and right corners, respectively.
  auto static BeadPositionAdjustment(const macs::Groove& groove, double bead_pos, double k_gain) -> double;
};
}  // namespace bead_control
