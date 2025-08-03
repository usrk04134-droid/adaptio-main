#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <numeric>
#include <vector>

#include "calibration/calibration_configuration.h"
#include "common/logging/application_log.h"
#include "joint_geometry/joint_geometry.h"
#include "macs/macs_point.h"

namespace calibration {

const double REDUCTION_FACTOR = 0.95;
const double HALF             = 0.5;

// Note: In this file we reason about the coordinates for the wire tip in
// relation to the groove for simplicity. However, GenerateCalibrationDots will be
// called with macs::Point coordinates which are defined as the location of
// the tip of the contact tube. This is fine since it is only the relative
// position values which define the grid.

struct GridLayout {
  double x_center{};  // Center between c_left and c_right
  double z_start{};   // Starting z position (bottom row, at c_z)
  double spacing{};   // Spacing between points
};

struct GridPoint {
  int index_x{};  // Horizontal index (relative to center)
  int index_z{};  // Vertical index (upwards)
  std::shared_ptr<GridLayout> layout;

  auto GetX() const -> double { return layout->x_center + (index_x * layout->spacing); }
  auto GetZ() const -> double { return layout->z_start + (index_z * layout->spacing); }
};

// TODO: Move this function to another file
inline auto ValidateAndCalculateGrooveTopCenter(const joint_geometry::JointGeometry& joint_geometry,
                                                double min_touch_width_ratio, double max_touch_width_ratio,
                                                double wire_diameter, double stickout,
                                                const macs::Point& left_touch_position,
                                                const macs::Point& right_touch_position) -> std::optional<macs::Point> {
  const auto touch_width = left_touch_position.horizontal - right_touch_position.horizontal + wire_diameter;
  auto touch_ratio       = touch_width / joint_geometry.upper_joint_width_mm;

  if (touch_ratio < min_touch_width_ratio || touch_ratio > max_touch_width_ratio) {
    LOG_ERROR("Touch width ratio invalid: {}", touch_ratio);
    return {};
  }

  const auto min_groove_angle = std::min(joint_geometry.left_joint_angle_rad, joint_geometry.right_joint_angle_rad);

  // depth of touch points relative to groove top
  const double touch_depth = HALF * (joint_geometry.upper_joint_width_mm - touch_width) / std::tan(min_groove_angle);

  if (touch_depth > joint_geometry.groove_depth_mm || touch_depth < 0.0) {
    LOG_ERROR("Touch depth invalid: {}", touch_depth);
    return {};
  }

  return macs::Point{.horizontal = std::midpoint(left_touch_position.horizontal, right_touch_position.horizontal),
                     .vertical   = left_touch_position.vertical - stickout + touch_depth};
}

// depth_c is the vertical distance from the touch points to the groove top
inline auto GenerateCalibrationDots(const GridConfiguration& grid_config, double c_left_x, double c_right_x, double c_z,
                                    double depth_c) -> std::vector<GridPoint> {
  const double c_width  = c_left_x - c_right_x;
  const double x_center = std::midpoint(c_left_x, c_right_x);

  // A is a rectangle with base corners at c_right and c_left
  // and top corners a distance margin_top above the top surface.
  const double a_xmin = c_right_x;
  const double a_xmax = c_left_x;
  const double a_zmin = c_z;
  const double a_zmax = c_z + depth_c + grid_config.margin_top;

  // B is a rectangle with base at the top of rectangle A
  // The height is margin_z, width extends margin_x outside
  // rectangle A on both sides.
  const double b_xmin = c_right_x - grid_config.margin_x;
  const double b_xmax = c_left_x + grid_config.margin_x;
  const double b_zmin = a_zmax;
  const double b_zmax = b_zmin + grid_config.margin_z;

  // This lambda determines if a point is inside the combined region of A & B
  auto inside_area = [&](double px, double pz) -> bool {
    const bool in_a = (px >= a_xmin) && (px <= a_xmax) && (pz >= a_zmin) && (pz <= a_zmax);
    const bool in_b = (px >= b_xmin) && (px <= b_xmax) && (pz >= b_zmin) && (pz <= b_zmax);
    return in_a || in_b;
  };

  // The initial value of the spacing is set very large
  // then the algorithm reduces the value with REDUCTION_FACTOR
  // until the number of dots exceed the target.
  double spacing = HALF * c_width;
  std::vector<GridPoint> ordered_points;

  while (true) {
    std::vector<GridPoint> raw_points;
    auto layout = std::make_shared<GridLayout>(GridLayout{.x_center = x_center, .z_start = c_z, .spacing = spacing});

    for (int index_z = 0;; ++index_z) {
      const double pz = layout->z_start + (index_z * layout->spacing);
      if (pz > (c_z + depth_c + grid_config.margin_top + grid_config.margin_z)) {
        break;
      }

      const int max_index_x = static_cast<int>((c_width / 2.0 + grid_config.margin_x) / spacing) + 1;
      for (int index_x = -max_index_x; index_x <= max_index_x; ++index_x) {
        const double px = layout->x_center + (index_x * layout->spacing);

        if (index_z == 0) {
          if (px > (c_left_x - grid_config.margin_c) || px < (c_right_x + grid_config.margin_c)) {
            // Avoid the region close to c_right and c_left which is near the wall
            continue;
          }
        }

        if (inside_area(px, pz)) {
          raw_points.emplace_back(GridPoint{.index_x = index_x, .index_z = index_z, .layout = layout});
        }
      }
    }

    if (raw_points.size() >= grid_config.target_nr_gridpoints) {
      // The target number of dots has been reached.
      // Sort the dots according to a snake pattern and break

      std::map<int, std::vector<GridPoint>> rows;
      for (const auto& point : raw_points) {
        rows[point.index_z].push_back(point);
      }

      ordered_points.clear();
      size_t row_index = 0;

      for (auto& [row_z, row_points] : rows) {
        if (row_index % 2 == 0) {
          // Sort in order right to left. Note that x increases from right to left(!)
          // That means low index_x values should be sorted before high index_x values
          std::ranges::sort(row_points,
                            [](const GridPoint& lhs, const GridPoint& rhs) { return lhs.index_x < rhs.index_x; });
        } else {
          // Sort in order left to right
          std::ranges::sort(row_points,
                            [](const GridPoint& lhs, const GridPoint& rhs) { return lhs.index_x > rhs.index_x; });
        }
        ordered_points.insert(ordered_points.end(), row_points.begin(), row_points.end());
        ++row_index;
      }

      break;
    }

    spacing *= REDUCTION_FACTOR;
  }

  return ordered_points;
}

}  // namespace calibration
