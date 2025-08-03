#pragma once

#include <fmt/core.h>

#include <chrono>
#include <string>

namespace weld_control {

enum class Mode {
  IDLE,
  JOINT_TRACKING,
  AUTOMATIC_BEAD_PLACEMENT,
};

enum class LayerType {
  NOT_APPLICABLE,
  FILL,
  CAP,
};

struct Configuration {
  enum class ImageLoggingMode {
    OFF,
    DIRECT,
    ON_ERROR,
    ON_ERROR_WELDING,
  };

  struct {
    struct {
      double upper_width{0.0};
      double wall_angle{0.0};
    } tolerance;
  } scanner_groove_geometry_update;
  struct {
    std::chrono::milliseconds arcing_lost_grace{0};
  } supervision;
  struct {
    struct {
      uint32_t kernel_size;
      double sigma;
    } gaussian_filter;
  } adaptivity;
  std::chrono::milliseconds scanner_input_interval{0};
  double fill_layer_groove_depth_threshold;
  std::chrono::seconds handover_grace;
  std::chrono::seconds scanner_low_confidence_grace{0};
  std::chrono::seconds scanner_no_confidence_grace{0};
};

auto inline ConfigurationToString(const Configuration& config) -> std::string {
  return fmt::format(
      "scanner_groove_geometry_update: {{tolerance: {{upper_width: {:.3f}, wall_angle: {:.3f}}}}}, "
      "supervision: {{arcing_lost_grace_ms: {}}}, adaptivity: {{gaussian_filter: {{kernel_size: {}, sigma: {}}}}}, "
      "scanner_input_interval_ms: {}, fill_layer_groove_depth_threshold_mm: {}, handover_grace_seconds: {}, "
      "scanner_low_confidence_grace_seconds: {}, scanner_no_confidence_grace_seconds: {}",
      config.scanner_groove_geometry_update.tolerance.upper_width,
      config.scanner_groove_geometry_update.tolerance.wall_angle, config.supervision.arcing_lost_grace.count(),
      config.adaptivity.gaussian_filter.kernel_size, config.adaptivity.gaussian_filter.sigma,
      config.scanner_input_interval.count(), config.fill_layer_groove_depth_threshold, config.handover_grace.count(),
      config.scanner_low_confidence_grace.count(), config.scanner_no_confidence_grace.count());
}

auto inline ModeToString(weld_control::Mode mode) -> std::string {
  switch (mode) {
    case weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT:
      return "abp";
    case weld_control::Mode::JOINT_TRACKING:
      return "jt";
    case weld_control::Mode::IDLE:
      return "idle";
    default:
      break;
  }
  return "invalid";
}

}  // namespace weld_control
